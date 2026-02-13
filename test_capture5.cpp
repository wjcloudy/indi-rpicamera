/*
 * test_capture5.cpp — Prove raw capture + render works from libcamera
 *
 * Uses AE (auto-exposure) to find good exposure/gain, then captures:
 *
 *   Phase 1 — RAW Bayer (StillCapture + SGBRG16 on PiSP):
 *     - /tmp/raw_proof.pgm          16-bit greyscale Bayer
 *     - /tmp/raw_proof_colour.ppm   Demosaiced colour (8-bit PPM)
 *     - /tmp/raw_proof.fits         2D FITS (NAXIS=2, USHORT, Bayer keywords)
 *
 *   Phase 2 — RGB ISP (StillCapture + BGR888):
 *     - /tmp/rgb_proof.ppm          24-bit colour PPM
 *     - /tmp/rgb_proof.fits         3D FITS (NAXIS=3, 3×H×W, plane-sequential)
 *
 * Build:
 *   g++ -std=c++17 -O2 -o test_capture5 test_capture5.cpp \
 *       $(pkg-config --cflags --libs libcamera) -lcfitsio
 */

#include <libcamera/libcamera.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

#include <fitsio.h>
#include <sys/mman.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <algorithm>

namespace lc = libcamera;

// ============================================================
//  Synchronisation
// ============================================================
static std::mutex g_mutex;
static std::condition_variable g_cv;
static bool g_frameReady = false;
static lc::Request *g_completedRequest = nullptr;

struct MappedPlane { void *memory = nullptr; size_t length = 0; };
static std::map<const lc::FrameBuffer *, std::vector<MappedPlane>> g_mapped;

static void requestComplete(lc::Request *request) {
    if (request->status() == lc::Request::RequestCancelled) return;
    std::lock_guard<std::mutex> lock(g_mutex);
    g_completedRequest = request;
    g_frameReady = true;
    g_cv.notify_all();
}

static void mapBuffers(lc::FrameBufferAllocator &alloc, lc::Stream *stream) {
    for (const auto &buffer : alloc.buffers(stream)) {
        std::vector<MappedPlane> planes;
        for (const auto &plane : buffer->planes()) {
            void *mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, plane.fd.get(), plane.offset);
            if (mem != MAP_FAILED) planes.push_back({mem, plane.length});
        }
        g_mapped[buffer.get()] = std::move(planes);
    }
}

static void unmapAll() {
    for (auto &[fb, planes] : g_mapped)
        for (auto &p : planes)
            if (p.memory && p.memory != MAP_FAILED)
                munmap(p.memory, p.length);
    g_mapped.clear();
}

static bool waitFrame(int timeoutSec = 15) {
    std::unique_lock<std::mutex> lock(g_mutex);
    return g_cv.wait_for(lock, std::chrono::seconds(timeoutSec),
                         [] { return g_frameReady; });
}

static void resetFrame() {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_frameReady = false;
    g_completedRequest = nullptr;
}

static const uint8_t *getFrameData(size_t &len) {
    if (!g_completedRequest) return nullptr;
    const auto &bufs = g_completedRequest->buffers();
    if (bufs.empty()) return nullptr;
    auto it = g_mapped.find(bufs.begin()->second);
    if (it == g_mapped.end() || it->second.empty()) return nullptr;
    len = it->second[0].length;
    return static_cast<const uint8_t *>(it->second[0].memory);
}

// Read back AE-chosen exposure and gain from completed request metadata
static void readAEMetadata(int32_t &expUs, float &gain) {
    if (!g_completedRequest) return;
    const auto &meta = g_completedRequest->metadata();
    auto e = meta.get(lc::controls::ExposureTime);
    if (e) expUs = *e;
    auto g = meta.get(lc::controls::AnalogueGain);
    if (g) gain = *g;
}

// ============================================================
//  Bayer helpers
// ============================================================
static std::string bayerPattern(const lc::PixelFormat &fmt) {
    std::string s = fmt.toString();
    if (s.find("GBRG") != std::string::npos) return "GBRG";
    if (s.find("BGGR") != std::string::npos) return "BGGR";
    if (s.find("RGGB") != std::string::npos) return "RGGB";
    if (s.find("GRBG") != std::string::npos) return "GRBG";
    return "";
}

static lc::PixelFormat bayer16For(const std::string &pat) {
    if (pat == "GBRG") return lc::formats::SGBRG16;
    if (pat == "BGGR") return lc::formats::SBGGR16;
    if (pat == "RGGB") return lc::formats::SRGGB16;
    if (pat == "GRBG") return lc::formats::SGRBG16;
    return lc::formats::SGBRG16;
}

static bool detectPiSP(std::shared_ptr<lc::Camera> &cam,
                       const lc::PixelFormat &fmt, const lc::Size &sz) {
    auto cfg = cam->generateConfiguration({lc::StreamRole::Raw});
    if (!cfg || cfg->empty()) return false;
    cfg->at(0).pixelFormat = fmt;
    cfg->at(0).size = sz;
    cfg->validate();
    std::string f = cfg->at(0).pixelFormat.toString();
    return f.find("PISP") != std::string::npos || f.find("_COMP") != std::string::npos;
}

// ============================================================
//  Pixel stats
// ============================================================
struct PixelStats {
    uint16_t minV, maxV;
    double mean;
    int effectiveBits;
    bool alreadyShifted;
};

static PixelStats analyzePixels(const uint16_t *px, size_t count, int nativeBits) {
    PixelStats s{};
    s.minV = 65535; s.maxV = 0;
    uint64_t sum = 0;
    for (size_t i = 0; i < count; i++) {
        if (px[i] < s.minV) s.minV = px[i];
        if (px[i] > s.maxV) s.maxV = px[i];
        sum += px[i];
    }
    s.mean = (double)sum / count;
    s.alreadyShifted = (s.maxV > ((1u << nativeBits) - 1));
    s.effectiveBits = 0;
    uint16_t test = s.maxV;
    while (test > 0) { s.effectiveBits++; test >>= 1; }
    return s;
}

// ============================================================
//  Simple Bayer → RGB demosaic (bilinear)
// ============================================================
static void demosaic(const uint16_t *bayer, unsigned w, unsigned h,
                     const std::string &pat, uint8_t *rgb,
                     uint16_t blackLevel, uint16_t whiteLevel)
{
    char cmap[2][2];
    if      (pat == "GBRG") { cmap[0][0]='G'; cmap[0][1]='B'; cmap[1][0]='R'; cmap[1][1]='G'; }
    else if (pat == "RGGB") { cmap[0][0]='R'; cmap[0][1]='G'; cmap[1][0]='G'; cmap[1][1]='B'; }
    else if (pat == "BGGR") { cmap[0][0]='B'; cmap[0][1]='G'; cmap[1][0]='G'; cmap[1][1]='R'; }
    else                    { cmap[0][0]='G'; cmap[0][1]='R'; cmap[1][0]='B'; cmap[1][1]='G'; }

    float range = (float)(whiteLevel - blackLevel);
    if (range < 1) range = 1;

    auto to8 = [&](float v) -> uint8_t {
        float norm = (v - blackLevel) / range;
        if (norm < 0) norm = 0;
        if (norm > 1) norm = 1;
        return (uint8_t)(norm * 255.0f);
    };

    for (unsigned y = 1; y + 1 < h; y++) {
        for (unsigned x = 1; x + 1 < w; x++) {
            float r, g, b;
            char c = cmap[y & 1][x & 1];
            uint16_t p  = bayer[y * w + x];
            uint16_t n  = bayer[(y-1) * w + x];
            uint16_t s  = bayer[(y+1) * w + x];
            uint16_t ww = bayer[y * w + (x-1)];
            uint16_t e  = bayer[y * w + (x+1)];
            uint16_t nw = bayer[(y-1) * w + (x-1)];
            uint16_t ne = bayer[(y-1) * w + (x+1)];
            uint16_t sw = bayer[(y+1) * w + (x-1)];
            uint16_t se = bayer[(y+1) * w + (x+1)];

            if (c == 'R') {
                r = p; g = (n+s+ww+e)/4.0f; b = (nw+ne+sw+se)/4.0f;
            } else if (c == 'B') {
                b = p; g = (n+s+ww+e)/4.0f; r = (nw+ne+sw+se)/4.0f;
            } else {
                g = p;
                bool rowHasRed = (cmap[y & 1][0] == 'R' || cmap[y & 1][1] == 'R');
                if (rowHasRed) { r = (ww+e)/2.0f; b = (n+s)/2.0f; }
                else           { b = (ww+e)/2.0f; r = (n+s)/2.0f; }
            }

            size_t idx = ((size_t)y * w + x) * 3;
            rgb[idx+0] = to8(r);
            rgb[idx+1] = to8(g);
            rgb[idx+2] = to8(b);
        }
    }
}

// ============================================================
//  File writers
// ============================================================

static bool writePGM(const char *path, const uint16_t *px, unsigned w, unsigned h) {
    FILE *f = fopen(path, "wb");
    if (!f) return false;
    fprintf(f, "P5\n%u %u\n65535\n", w, h);
    std::vector<uint8_t> row(w * 2);
    for (unsigned y = 0; y < h; y++) {
        for (unsigned x = 0; x < w; x++) {
            uint16_t v = px[y * w + x];
            row[x*2+0] = (v >> 8) & 0xFF;
            row[x*2+1] = v & 0xFF;
        }
        fwrite(row.data(), 1, w * 2, f);
    }
    fclose(f);
    return true;
}

static bool writePPM(const char *path, const uint8_t *rgb, unsigned w, unsigned h) {
    FILE *f = fopen(path, "wb");
    if (!f) return false;
    fprintf(f, "P6\n%u %u\n255\n", w, h);
    fwrite(rgb, 1, (size_t)w * h * 3, f);
    fclose(f);
    return true;
}

static bool writeFITS2D(const char *path, const uint16_t *px,
                        unsigned w, unsigned h,
                        const std::string &bayerPat,
                        int nativeBits, double exposureSec,
                        float gain)
{
    fitsfile *fptr = nullptr;
    int status = 0;
    char err[80];
    remove(path);

    if (fits_create_file(&fptr, path, &status)) {
        fits_get_errstatus(status, err); printf("    FITS error: %s\n", err); return false;
    }

    long naxes[2] = { (long)w, (long)h };
    fits_create_img(fptr, USHORT_IMG, 2, naxes, &status);

    // Bayer keywords
    int xoff = 0, yoff = 0;
    fits_update_key(fptr, TINT, "XBAYROFF", &xoff, "X offset of Bayer array", &status);
    fits_update_key(fptr, TINT, "YBAYROFF", &yoff, "Y offset of Bayer array", &status);
    char bayerStr[8]; strncpy(bayerStr, bayerPat.c_str(), sizeof(bayerStr));
    fits_update_key(fptr, TSTRING, "BAYERPAT", bayerStr, "Bayer color pattern", &status);

    int rawbpp = nativeBits;
    fits_update_key(fptr, TINT, "RAWBPP", &rawbpp, "Native sensor bit depth", &status);
    fits_update_key(fptr, TDOUBLE, "EXPTIME", &exposureSec, "Exposure time (s)", &status);
    double dGain = gain;
    fits_update_key(fptr, TDOUBLE, "GAIN", &dGain, "Analog gain", &status);
    fits_write_comment(fptr, "Raw Bayer from test_capture5 (no INDI)", &status);

    fits_write_img(fptr, TUSHORT, 1, (long)w * h,
                   const_cast<uint16_t *>(px), &status);
    fits_close_file(fptr, &status);
    return (status == 0);
}

static bool writeFITS3D(const char *path, const uint8_t *rgb,
                        unsigned w, unsigned h,
                        double exposureSec, float gain)
{
    fitsfile *fptr = nullptr;
    int status = 0;
    remove(path);

    if (fits_create_file(&fptr, path, &status)) return false;

    // NAXIS=3: plane-sequential R, G, B
    long naxes[3] = { (long)w, (long)h, 3 };
    fits_create_img(fptr, BYTE_IMG, 3, naxes, &status);

    fits_update_key(fptr, TDOUBLE, "EXPTIME", &exposureSec, "Exposure time (s)", &status);
    double dGain = gain;
    fits_update_key(fptr, TDOUBLE, "GAIN", &dGain, "Analog gain", &status);
    fits_write_comment(fptr, "RGB ISP from test_capture5 (no INDI)", &status);

    // Convert interleaved RGB → plane-sequential for FITS
    size_t planeSize = (size_t)w * h;
    std::vector<uint8_t> planeSeq(planeSize * 3);
    for (size_t i = 0; i < planeSize; i++) {
        planeSeq[i]               = rgb[i * 3 + 0]; // R
        planeSeq[planeSize + i]   = rgb[i * 3 + 1]; // G
        planeSeq[2*planeSize + i] = rgb[i * 3 + 2]; // B
    }

    fits_write_img(fptr, TBYTE, 1, (long)planeSize * 3,
                   planeSeq.data(), &status);
    fits_close_file(fptr, &status);
    return (status == 0);
}

// ============================================================
//  Session helpers (configure → allocate → map → request)
// ============================================================

struct Session {
    std::shared_ptr<lc::Camera> camera;
    std::unique_ptr<lc::CameraConfiguration> config;
    std::unique_ptr<lc::FrameBufferAllocator> alloc;
    std::vector<std::unique_ptr<lc::Request>> requests;
    lc::Stream *stream = nullptr;
    unsigned W = 0, H = 0, stride = 0;

    bool configure() {
        auto st = config->validate();
        printf("  Validation: %s\n",
               st == lc::CameraConfiguration::Valid ? "Valid" :
               st == lc::CameraConfiguration::Adjusted ? "Adjusted" : "INVALID");
        if (st == lc::CameraConfiguration::Invalid) return false;

        auto &sc = config->at(0);
        printf("  Negotiated: %s  %ux%u  stride=%u\n",
               sc.pixelFormat.toString().c_str(),
               sc.size.width, sc.size.height, sc.stride);

        if (camera->configure(config.get())) {
            printf("  ERROR: configure() failed\n");
            return false;
        }

        W = sc.size.width; H = sc.size.height; stride = sc.stride;
        stream = sc.stream();

        alloc = std::make_unique<lc::FrameBufferAllocator>(camera);
        int n = alloc->allocate(stream);
        if (n < 0) { printf("  ERROR: allocate\n"); return false; }

        mapBuffers(*alloc, stream);

        for (const auto &buf : alloc->buffers(stream)) {
            auto req = camera->createRequest();
            req->addBuffer(stream, buf.get());
            requests.push_back(std::move(req));
        }
        return true;
    }

    void cleanup() {
        requests.clear();
        alloc.reset();
        config.reset();
        unmapAll();
    }
};

// Capture N frames with AE, return last frame's AE-chosen exposure/gain.
// Then capture one more "keeper" frame with those fixed values.
static bool captureWithAE(Session &sess, int warmup,
                          int32_t &outExpUs, float &outGain)
{
    sess.camera->requestCompleted.connect(requestComplete);
    if (sess.camera->start()) {
        printf("  ERROR: start failed\n");
        sess.camera->requestCompleted.disconnect(requestComplete);
        return false;
    }

    // Phase A: AE warmup frames
    printf("  AE convergence: %d frames...\n", warmup);
    int32_t aeExp = 33000;
    float aeGain = 1.0f;

    for (int i = 0; i < warmup; i++) {
        resetFrame();
        auto &req = sess.requests[i % sess.requests.size()];
        if (i > 0) req->reuse(lc::Request::ReuseBuffers);
        req->controls().set(lc::controls::AeEnable, true);
        sess.camera->queueRequest(req.get());
        if (!waitFrame(15)) { printf("  frame %d TIMEOUT\n", i); break; }
        readAEMetadata(aeExp, aeGain);
        if (i % 3 == 0 || i == warmup - 1)
            printf("    frame %d: AE → exp=%d µs  gain=%.2f\n", i, aeExp, aeGain);
    }

    outExpUs = aeExp;
    outGain = aeGain;

    // Phase B: one final frame with fixed AE-chosen values
    printf("  Final capture: exp=%d µs  gain=%.2f (AE locked)\n", aeExp, aeGain);
    resetFrame();
    auto &req = sess.requests[0];
    req->reuse(lc::Request::ReuseBuffers);
    req->controls().set(lc::controls::AeEnable, false);
    req->controls().set(lc::controls::ExposureTime, aeExp);
    req->controls().set(lc::controls::AnalogueGain, aeGain);
    sess.camera->queueRequest(req.get());

    if (!waitFrame(15)) {
        printf("  TIMEOUT on final frame\n");
        sess.camera->stop();
        sess.camera->requestCompleted.disconnect(requestComplete);
        return false;
    }

    sess.camera->stop();
    sess.camera->requestCompleted.disconnect(requestComplete);
    return true;
}

// ============================================================
//  FITS verification
// ============================================================
static void verifyFITS(const char *path) {
    fitsfile *fptr = nullptr;
    int status = 0;
    fits_open_file(&fptr, path, READONLY, &status);
    if (status != 0) { printf("    Cannot open %s\n", path); return; }

    int naxis = 0;
    long naxes[3] = {0};
    int bitpix = 0;
    fits_get_img_param(fptr, 3, &bitpix, &naxis, naxes, &status);
    printf("    NAXIS=%d  NAXIS1=%ld  NAXIS2=%ld", naxis, naxes[0], naxes[1]);
    if (naxis > 2) printf("  NAXIS3=%ld", naxes[2]);
    printf("  BITPIX=%d\n", bitpix);

    char bayerVal[32] = "";
    int bstatus = 0;
    fits_read_key(fptr, TSTRING, "BAYERPAT", bayerVal, nullptr, &bstatus);
    if (bstatus == 0) printf("    BAYERPAT=%s\n", bayerVal);

    // Read first 8 pixels
    long fpixel[3] = {1, 1, 1};
    if (bitpix == 16) {
        uint16_t px[8];
        fits_read_pix(fptr, TUSHORT, fpixel, 8, nullptr, px, nullptr, &status);
        printf("    First 8 px: ");
        for (int i = 0; i < 8; i++) printf("%u ", px[i]);
        printf("\n");
    } else if (bitpix == 8) {
        uint8_t px[8];
        fits_read_pix(fptr, TBYTE, fpixel, 8, nullptr, px, nullptr, &status);
        printf("    First 8 px: ");
        for (int i = 0; i < 8; i++) printf("%u ", px[i]);
        printf("\n");
    }

    fits_close_file(fptr, &status);
}

// ============================================================
//  MAIN
// ============================================================
int main()
{
    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║  Raw Capture Proof Test — AE + Multi-format Output      ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n\n");

    auto cm = std::make_shared<lc::CameraManager>();
    if (cm->start()) { printf("ERROR: CameraManager\n"); return 1; }
    auto cameras = cm->cameras();
    if (cameras.empty()) { printf("ERROR: no cameras\n"); cm->stop(); return 1; }
    auto camera = cameras[0];
    printf("Camera: %s\n", camera->id().c_str());
    if (camera->acquire()) { printf("ERROR: acquire\n"); cm->stop(); return 1; }

    auto model = camera->properties().get(lc::properties::Model);
    if (model) printf("Sensor: %s\n", model->c_str());

    // ---- Discover sensor ----
    lc::PixelFormat sensorFmt;
    lc::Size sensorSize;
    std::string pattern;
    unsigned nativeBits = 10;
    {
        auto cfg = camera->generateConfiguration({lc::StreamRole::Raw});
        if (cfg && !cfg->empty()) {
            for (const auto &pf : cfg->at(0).formats().pixelformats()) {
                std::string name = pf.toString();
                if (name.find("PISP") != std::string::npos || name.find("_COMP") != std::string::npos)
                    continue;
                auto sizes = cfg->at(0).formats().sizes(pf);
                if (!sizes.empty()) {
                    sensorFmt = pf;
                    sensorSize = sizes[0];
                    pattern = bayerPattern(pf);
                    auto upos = name.find('_');
                    std::string base = (upos != std::string::npos) ? name.substr(0, upos) : name;
                    std::string digits;
                    for (auto it = base.rbegin(); it != base.rend() && isdigit(*it); ++it)
                        digits.insert(digits.begin(), *it);
                    if (!digits.empty()) nativeBits = std::stoi(digits);
                    break;
                }
            }
        }
    }
    printf("Sensor: %s  %dx%d  (%s, %u-bit)\n",
           sensorFmt.toString().c_str(), sensorSize.width, sensorSize.height,
           pattern.c_str(), nativeBits);

    bool isPiSP = detectPiSP(camera, sensorFmt, sensorSize);
    printf("PiSP: %s\n\n", isPiSP ? "YES (Pi 5)" : "no");

    // ================================================================
    //  PHASE 1: RAW Bayer capture with AE
    // ================================================================
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  PHASE 1: RAW Bayer Capture (auto-exposure)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    {
        Session sess;
        sess.camera = camera;

        if (isPiSP) {
            sess.config = camera->generateConfiguration({lc::StreamRole::StillCapture});
            sess.config->at(0).pixelFormat = bayer16For(pattern);
            sess.config->at(0).size = sensorSize;
            printf("  Path: StillCapture + %s\n",
                   bayer16For(pattern).toString().c_str());
        } else {
            sess.config = camera->generateConfiguration({lc::StreamRole::Raw});
            sess.config->at(0).pixelFormat = sensorFmt;
            sess.config->at(0).size = sensorSize;
            printf("  Path: Raw + %s\n", sensorFmt.toString().c_str());
        }

        if (!sess.configure()) goto phase2;

        {
            int32_t aeExpUs = 0;
            float aeGain = 0;
            if (!captureWithAE(sess, 10, aeExpUs, aeGain)) goto phase1_done;

            size_t dataLen;
            const uint8_t *data = getFrameData(dataLen);
            if (!data) { printf("  No frame data!\n"); goto phase1_done; }

            unsigned W = sess.W, H = sess.H, stride = sess.stride;
            printf("\n  Frame: %zu bytes  (%ux%u, stride %u)\n", dataLen, W, H, stride);

            // Copy with stride
            std::vector<uint16_t> pixels(W * H);
            for (unsigned row = 0; row < H; row++) {
                const uint16_t *srcRow = reinterpret_cast<const uint16_t *>(
                    data + row * stride);
                std::memcpy(&pixels[row * W], srcRow, W * sizeof(uint16_t));
            }

            auto stats = analyzePixels(pixels.data(), pixels.size(), nativeBits);
            printf("  Pixels: min=%u  max=%u  mean=%.0f  shifted=%s\n",
                   stats.minV, stats.maxV, stats.mean,
                   stats.alreadyShifted ? "YES" : "no");
            if (stats.alreadyShifted) {
                unsigned sh = 16 - nativeBits;
                printf("  10-bit equiv: min=%u  max=%u  mean=%.1f\n",
                       stats.minV >> sh, stats.maxV >> sh, stats.mean / (1 << sh));
            }

            // Check saturation
            uint16_t satLevel = stats.alreadyShifted
                ? (uint16_t)(((1u << nativeBits) - 1) << (16 - nativeBits))
                : (uint16_t)((1u << nativeBits) - 1);
            size_t nSat = 0;
            for (size_t i = 0; i < pixels.size(); i++)
                if (pixels[i] >= satLevel) nSat++;
            printf("  Saturated pixels: %zu (%.1f%%)\n",
                   nSat, 100.0 * nSat / pixels.size());

            // Determine black/white for demosaic using percentile stretch
            std::vector<uint16_t> sorted(pixels.begin(), pixels.end());
            std::nth_element(sorted.begin(), sorted.begin() + sorted.size()/100,
                             sorted.end());
            uint16_t blackLevel = sorted[sorted.size() / 100];
            std::nth_element(sorted.begin(),
                             sorted.begin() + sorted.size() * 99 / 100,
                             sorted.end());
            uint16_t whiteLevel = sorted[sorted.size() * 99 / 100];
            if (whiteLevel > satLevel) whiteLevel = satLevel;
            printf("  Stretch: black=%u  white=%u (1st–99th percentile)\n",
                   blackLevel, whiteLevel);

            // Bayer channel stats
            {
                double ch[4] = {0};
                size_t cnt[4] = {0};
                for (unsigned y = 0; y < H; y++)
                    for (unsigned x = 0; x < W; x++) {
                        int idx = (y & 1) * 2 + (x & 1);
                        ch[idx] += pixels[y * W + x];
                        cnt[idx]++;
                    }
                printf("  Bayer channels: ");
                const char *labels[4];
                if      (pattern == "GBRG") { labels[0]="G "; labels[1]="B "; labels[2]="R "; labels[3]="G2"; }
                else if (pattern == "RGGB") { labels[0]="R "; labels[1]="G "; labels[2]="G2"; labels[3]="B "; }
                else if (pattern == "BGGR") { labels[0]="B "; labels[1]="G "; labels[2]="G2"; labels[3]="R "; }
                else                        { labels[0]="G "; labels[1]="R "; labels[2]="B "; labels[3]="G2"; }
                for (int i = 0; i < 4; i++)
                    printf("%s=%.0f  ", labels[i], ch[i] / cnt[i]);
                printf("\n");
            }

            printf("\n  ── Saving RAW outputs ──\n");

            // 1. PGM (raw 16-bit as-is)
            if (writePGM("/tmp/raw_proof.pgm", pixels.data(), W, H))
                printf("  ✓ /tmp/raw_proof.pgm           (16-bit Bayer greyscale)\n");

            // 2. PPM (demosaiced colour with percentile stretch)
            std::vector<uint8_t> rgb(W * H * 3, 0);
            demosaic(pixels.data(), W, H, pattern, rgb.data(), blackLevel, whiteLevel);
            if (writePPM("/tmp/raw_proof_colour.ppm", rgb.data(), W, H))
                printf("  ✓ /tmp/raw_proof_colour.ppm    (demosaiced colour)\n");

            // 3. FITS 2D (same format INDI should produce)
            if (writeFITS2D("/tmp/raw_proof.fits", pixels.data(), W, H,
                            pattern, nativeBits, aeExpUs / 1e6, aeGain))
                printf("  ✓ /tmp/raw_proof.fits          (FITS 2D NAXIS=2)\n");

            // 4. Verify
            printf("\n  ── FITS verification ──\n");
            verifyFITS("/tmp/raw_proof.fits");
        }

phase1_done:
        sess.cleanup();
    }

    // ================================================================
    //  PHASE 2: RGB ISP capture (for comparison)
    // ================================================================
phase2:
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  PHASE 2: RGB ISP Capture (auto-exposure, for comparison)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    {
        Session sess;
        sess.camera = camera;
        sess.config = camera->generateConfiguration({lc::StreamRole::StillCapture});
        sess.config->at(0).pixelFormat = lc::formats::BGR888;
        sess.config->at(0).size = sensorSize;
        printf("  Path: StillCapture + BGR888\n");

        if (!sess.configure()) goto done;

        {
            int32_t aeExpUs = 0;
            float aeGain = 0;
            if (!captureWithAE(sess, 10, aeExpUs, aeGain)) goto phase2_done;

            size_t dataLen;
            const uint8_t *data = getFrameData(dataLen);
            if (!data) { printf("  No frame data!\n"); goto phase2_done; }

            unsigned W = sess.W, H = sess.H, stride = sess.stride;
            printf("\n  Frame: %zu bytes  (%ux%u, stride %u)\n", dataLen, W, H, stride);

            std::string fmtStr = sess.config->at(0).pixelFormat.toString();
            int srcBpp = (fmtStr.find("X") != std::string::npos ||
                          fmtStr.find("A") != std::string::npos) ? 4 : 3;

            // Convert BGR → interleaved RGB
            std::vector<uint8_t> rgb(W * H * 3);
            for (unsigned row = 0; row < H; row++) {
                const uint8_t *srcRow = data + row * stride;
                uint8_t *dstRow = &rgb[row * W * 3];
                for (unsigned col = 0; col < W; col++) {
                    dstRow[col*3+0] = srcRow[col*srcBpp+2]; // R
                    dstRow[col*3+1] = srcRow[col*srcBpp+1]; // G
                    dstRow[col*3+2] = srcRow[col*srcBpp+0]; // B
                }
            }

            // Quick brightness check
            uint64_t sum = 0;
            for (size_t i = 0; i < rgb.size(); i++) sum += rgb[i];
            printf("  RGB mean brightness: %.1f / 255\n",
                   (double)sum / rgb.size());

            printf("\n  ── Saving RGB outputs ──\n");

            // PPM
            if (writePPM("/tmp/rgb_proof.ppm", rgb.data(), W, H))
                printf("  ✓ /tmp/rgb_proof.ppm           (24-bit colour)\n");

            // FITS 3D (plane-sequential, like INDI RGB output)
            if (writeFITS3D("/tmp/rgb_proof.fits", rgb.data(), W, H,
                            aeExpUs / 1e6, aeGain))
                printf("  ✓ /tmp/rgb_proof.fits          (FITS 3D NAXIS=3)\n");

            printf("\n  ── FITS verification ──\n");
            verifyFITS("/tmp/rgb_proof.fits");
        }

phase2_done:
        sess.cleanup();
    }

done:
    camera->release();
    cm->stop();

    printf("\n╔═══════════════════════════════════════════════════════════╗\n");
    printf("║  Output files:                                           ║\n");
    printf("╠═══════════════════════════════════════════════════════════╣\n");
    printf("║  RAW:                                                    ║\n");
    printf("║    /tmp/raw_proof.pgm           Bayer greyscale (16-bit) ║\n");
    printf("║    /tmp/raw_proof_colour.ppm    Demosaiced colour        ║\n");
    printf("║    /tmp/raw_proof.fits          FITS 2D (NAXIS=2)       ║\n");
    printf("║  RGB:                                                    ║\n");
    printf("║    /tmp/rgb_proof.ppm           ISP colour               ║\n");
    printf("║    /tmp/rgb_proof.fits          FITS 3D (NAXIS=3)       ║\n");
    printf("╠═══════════════════════════════════════════════════════════╣\n");
    printf("║  View:                                                   ║\n");
    printf("║    feh /tmp/raw_proof_colour.ppm /tmp/rgb_proof.ppm      ║\n");
    printf("║    Open .fits in KStars FITS Viewer or DS9               ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    return 0;
}
