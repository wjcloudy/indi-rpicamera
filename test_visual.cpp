/*
 * test_visual.cpp — Visual verification tool for RPi camera capture paths
 *
 * Captures and saves to viewable files:
 *   1. RAW Bayer → PGM (16-bit greyscale, viewable in any image viewer)
 *   2. RGB still → PPM (24-bit colour)
 *   3. MJPEG video → .mjpeg file (10 frames, playable with mpv/ffplay/vlc)
 *
 * On Pi 5 (PiSP), uses StillCapture + Bayer16 for raw (same as fixed driver).
 * On older Pi, uses Raw stream + CSI2P unpack.
 *
 * Build:
 *   g++ -std=c++17 -O2 -o test_visual test_visual.cpp \
 *       $(pkg-config --cflags --libs libcamera) -ljpeg
 */

#include <libcamera/libcamera.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

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
#include <thread>
#include <jpeglib.h>

namespace lc = libcamera;

// ============================================================
//  Frame completion signalling
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

// ============================================================
//  Helpers
// ============================================================

static void mapAllBuffers(lc::FrameBufferAllocator &alloc, lc::Stream *stream)
{
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

static void unmapAll()
{
    for (auto &[fb, planes] : g_mapped)
        for (auto &p : planes)
            if (p.memory && p.memory != MAP_FAILED)
                munmap(p.memory, p.length);
    g_mapped.clear();
}

static bool waitFrame(int timeoutSec = 10)
{
    std::unique_lock<std::mutex> lock(g_mutex);
    return g_cv.wait_for(lock, std::chrono::seconds(timeoutSec),
                         [] { return g_frameReady; });
}

static void resetFrame()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_frameReady = false;
    g_completedRequest = nullptr;
}

// Get the mapped data for the first buffer of the completed request
static const uint8_t *getFrameData(size_t &outLen)
{
    if (!g_completedRequest) return nullptr;
    const auto &bufs = g_completedRequest->buffers();
    if (bufs.empty()) return nullptr;
    const lc::FrameBuffer *fb = bufs.begin()->second;
    auto it = g_mapped.find(fb);
    if (it == g_mapped.end() || it->second.empty()) return nullptr;
    outLen = it->second[0].length;
    return static_cast<const uint8_t *>(it->second[0].memory);
}

// Detect Bayer pattern from format string
static std::string bayerPattern(const lc::PixelFormat &fmt)
{
    std::string s = fmt.toString();
    if (s.find("GBRG") != std::string::npos) return "GBRG";
    if (s.find("BGGR") != std::string::npos) return "BGGR";
    if (s.find("RGGB") != std::string::npos) return "RGGB";
    if (s.find("GRBG") != std::string::npos) return "GRBG";
    return "";
}

// Get matching 16-bit Bayer format
static lc::PixelFormat bayer16For(const std::string &pattern)
{
    if (pattern == "GBRG") return lc::formats::SGBRG16;
    if (pattern == "BGGR") return lc::formats::SBGGR16;
    if (pattern == "RGGB") return lc::formats::SRGGB16;
    if (pattern == "GRBG") return lc::formats::SGRBG16;
    return lc::formats::SGBRG16;
}

// Detect PiSP backend (same logic as the driver)
static bool detectPiSP(std::shared_ptr<lc::Camera> &cam,
                       const lc::PixelFormat &sensorFmt,
                       const lc::Size &sensorSize)
{
    auto cfg = cam->generateConfiguration({lc::StreamRole::Raw});
    if (!cfg || cfg->empty()) return false;
    cfg->at(0).pixelFormat = sensorFmt;
    cfg->at(0).size = sensorSize;
    cfg->validate();
    std::string f = cfg->at(0).pixelFormat.toString();
    return f.find("PISP") != std::string::npos || f.find("_COMP") != std::string::npos;
}

// Write BGR888 buffer to a JPEG in memory (for MJPEG)
static std::vector<uint8_t> bgrToJpeg(const uint8_t *bgr, unsigned int width,
                                       unsigned int height, unsigned int stride,
                                       int quality = 80)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    uint8_t *outBuf = nullptr;
    unsigned long outSize = 0;
    jpeg_mem_dest(&cinfo, &outBuf, &outSize);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    // Convert BGR → RGB row by row
    std::vector<uint8_t> rgbRow(width * 3);
    while (cinfo.next_scanline < cinfo.image_height) {
        const uint8_t *srcRow = bgr + cinfo.next_scanline * stride;
        for (unsigned int x = 0; x < width; x++) {
            rgbRow[x * 3 + 0] = srcRow[x * 3 + 2]; // R
            rgbRow[x * 3 + 1] = srcRow[x * 3 + 1]; // G
            rgbRow[x * 3 + 2] = srcRow[x * 3 + 0]; // B
        }
        uint8_t *rowPtr = rgbRow.data();
        jpeg_write_scanlines(&cinfo, &rowPtr, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    std::vector<uint8_t> result(outBuf, outBuf + outSize);
    free(outBuf);
    return result;
}

// Simple bilinear Bayer → RGB for display (GBRG example, adapts to pattern)
static void demosaicSimple(const uint16_t *bayer, unsigned int w, unsigned int h,
                           unsigned int strideBytes, const std::string &pattern,
                           uint8_t *rgb, unsigned int bitDepth)
{
    // Map pattern to colour offsets: [row%2][col%2] → R/G/B
    // GBRG: (0,0)=G (0,1)=B (1,0)=R (1,1)=G
    // RGGB: (0,0)=R (0,1)=G (1,0)=G (1,1)=B
    // BGGR: (0,0)=B (0,1)=G (1,0)=G (1,1)=R
    // GRBG: (0,0)=G (0,1)=R (1,0)=B (1,1)=G

    unsigned int stridePixels = strideBytes / 2;
    float scale = 255.0f / ((1 << bitDepth) - 1);

    for (unsigned int y = 1; y + 1 < h; y++) {
        for (unsigned int x = 1; x + 1 < w; x++) {
            float r, g, b;
            int py = y % 2, px = x % 2;
            uint16_t c  = bayer[y * stridePixels + x];
            uint16_t n  = bayer[(y-1) * stridePixels + x];
            uint16_t s  = bayer[(y+1) * stridePixels + x];
            uint16_t ww = bayer[y * stridePixels + (x-1)];
            uint16_t e  = bayer[y * stridePixels + (x+1)];
            uint16_t nw = bayer[(y-1) * stridePixels + (x-1)];
            uint16_t ne = bayer[(y-1) * stridePixels + (x+1)];
            uint16_t sw = bayer[(y+1) * stridePixels + (x-1)];
            uint16_t se = bayer[(y+1) * stridePixels + (x+1)];

            char colour = '?';
            if (pattern == "GBRG") {
                const char map[2][2] = {{'G','B'},{'R','G'}};
                colour = map[py][px];
            } else if (pattern == "RGGB") {
                const char map[2][2] = {{'R','G'},{'G','B'}};
                colour = map[py][px];
            } else if (pattern == "BGGR") {
                const char map[2][2] = {{'B','G'},{'G','R'}};
                colour = map[py][px];
            } else { // GRBG
                const char map[2][2] = {{'G','R'},{'B','G'}};
                colour = map[py][px];
            }

            if (colour == 'R') {
                r = c; g = (n + s + ww + e) / 4.0f; b = (nw + ne + sw + se) / 4.0f;
            } else if (colour == 'B') {
                b = c; g = (n + s + ww + e) / 4.0f; r = (nw + ne + sw + se) / 4.0f;
            } else { // G
                g = c;
                if (py == 0) {
                    // Green in red/blue row
                    if (pattern == "RGGB" || pattern == "GRBG") {
                        r = (ww + e) / 2.0f; b = (n + s) / 2.0f;
                    } else {
                        b = (ww + e) / 2.0f; r = (n + s) / 2.0f;
                    }
                } else {
                    if (pattern == "RGGB" || pattern == "GRBG") {
                        b = (ww + e) / 2.0f; r = (n + s) / 2.0f;
                    } else {
                        r = (ww + e) / 2.0f; b = (n + s) / 2.0f;
                    }
                }
            }

            size_t idx = ((size_t)y * w + x) * 3;
            rgb[idx + 0] = std::min(255, (int)(r * scale));
            rgb[idx + 1] = std::min(255, (int)(g * scale));
            rgb[idx + 2] = std::min(255, (int)(b * scale));
        }
    }
}

// ============================================================
//  Capture helper — captures N warmup frames, returns last
// ============================================================

struct CaptureSession {
    std::shared_ptr<lc::Camera> camera;
    std::unique_ptr<lc::CameraConfiguration> config;
    std::unique_ptr<lc::FrameBufferAllocator> allocator;
    std::vector<std::unique_ptr<lc::Request>> requests;

    void cleanup() {
        requests.clear();
        allocator.reset();
        config.reset();
    }
};

static bool setupSession(CaptureSession &sess)
{
    auto *stream = sess.config->at(0).stream();
    sess.allocator = std::make_unique<lc::FrameBufferAllocator>(sess.camera);
    int nbufs = sess.allocator->allocate(stream);
    if (nbufs < 0) { printf("  ERROR: allocate failed\n"); return false; }

    mapAllBuffers(*sess.allocator, stream);

    for (const auto &buf : sess.allocator->buffers(stream)) {
        auto req = sess.camera->createRequest();
        if (!req || req->addBuffer(stream, buf.get())) {
            printf("  ERROR: request/buffer setup failed\n");
            return false;
        }
        sess.requests.push_back(std::move(req));
    }
    return true;
}

static bool captureFrames(CaptureSession &sess, int count,
                          int32_t expUs = 100000, float gain = 2.0f)
{
    sess.camera->requestCompleted.connect(requestComplete);
    if (sess.camera->start()) {
        printf("  ERROR: camera start failed\n");
        sess.camera->requestCompleted.disconnect(requestComplete);
        return false;
    }

    for (int i = 0; i < count; i++) {
        resetFrame();
        auto &req = sess.requests[i % sess.requests.size()];
        if (i > 0) req->reuse(lc::Request::ReuseBuffers);
        req->controls().set(lc::controls::ExposureTime, expUs);
        req->controls().set(lc::controls::AnalogueGain, gain);
        req->controls().set(lc::controls::AeEnable, false);
        sess.camera->queueRequest(req.get());
        if (!waitFrame(15)) {
            printf("  ERROR: frame %d timeout\n", i);
            break;
        }
        if (i < count - 1 && count > 1)
            printf("  warmup frame %d OK\n", i);
    }

    return g_frameReady;
}

static void stopSession(CaptureSession &sess)
{
    sess.camera->stop();
    sess.camera->requestCompleted.disconnect(requestComplete);
    unmapAll();
    sess.cleanup();
}

// ============================================================
//  MAIN
// ============================================================

int main()
{
    printf("╔════════════════════════════════════════════════════╗\n");
    printf("║  RPi Camera Visual Verification Tool              ║\n");
    printf("╚════════════════════════════════════════════════════╝\n\n");

    auto cm = std::make_shared<lc::CameraManager>();
    if (cm->start()) { printf("ERROR: CameraManager start failed\n"); return 1; }

    auto cameras = cm->cameras();
    if (cameras.empty()) { printf("ERROR: no cameras found\n"); cm->stop(); return 1; }

    auto camera = cameras[0];
    printf("Camera: %s\n", camera->id().c_str());
    if (camera->acquire()) { printf("ERROR: acquire failed\n"); cm->stop(); return 1; }

    const auto &props = camera->properties();
    auto model = props.get(lc::properties::Model);
    if (model) printf("Sensor: %s\n", model->c_str());

    // Discover sensor mode
    lc::PixelFormat sensorFmt;
    lc::Size sensorSize;
    std::string sensorPattern;
    unsigned int nativeBitDepth = 10;
    {
        auto cfg = camera->generateConfiguration({lc::StreamRole::Raw});
        if (cfg && !cfg->empty()) {
            auto fmts = cfg->at(0).formats();
            for (const auto &pf : fmts.pixelformats()) {
                std::string name = pf.toString();
                if (name.find("PISP") != std::string::npos ||
                    name.find("_COMP") != std::string::npos)
                    continue;
                auto sizes = fmts.sizes(pf);
                if (!sizes.empty()) {
                    sensorFmt = pf;
                    sensorSize = sizes[0];
                    sensorPattern = bayerPattern(pf);
                    // Extract bit depth from format name
                    auto pos = name.find_last_of("0123456789");
                    if (pos != std::string::npos) {
                        auto start = name.find_first_of("0123456789");
                        // Find trailing digits
                        std::string base = name;
                        auto upos = base.find('_');
                        if (upos != std::string::npos) base = base.substr(0, upos);
                        std::string digits;
                        for (auto it = base.rbegin(); it != base.rend() && isdigit(*it); ++it)
                            digits.insert(digits.begin(), *it);
                        if (!digits.empty()) nativeBitDepth = std::stoi(digits);
                    }
                    break;
                }
            }
        }
    }

    printf("Sensor format: %s %dx%d (%s, %u-bit)\n",
           sensorFmt.toString().c_str(), sensorSize.width, sensorSize.height,
           sensorPattern.c_str(), nativeBitDepth);

    bool isPiSP = detectPiSP(camera, sensorFmt, sensorSize);
    printf("PiSP backend: %s\n\n", isPiSP ? "YES (Pi 5)" : "no");

    const char *rawFile   = "/tmp/test_raw.pgm";
    const char *rawPPM    = "/tmp/test_raw_demosaic.ppm";
    const char *rgbFile   = "/tmp/test_rgb.ppm";
    const char *videoFile = "/tmp/test_video.mjpeg";

    // ============================================================
    //  TEST 1: RAW Bayer capture → PGM + demosaiced PPM
    // ============================================================
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  TEST 1: RAW Bayer capture\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    {
        CaptureSession sess;
        sess.camera = camera;

        if (isPiSP) {
            printf("  Using StillCapture + %s (Pi 5 ISP path)\n",
                   bayer16For(sensorPattern).toString().c_str());
            sess.config = camera->generateConfiguration({lc::StreamRole::StillCapture});
            sess.config->at(0).pixelFormat = bayer16For(sensorPattern);
            sess.config->at(0).size = sensorSize;
        } else {
            printf("  Using Raw stream + %s (direct path)\n",
                   sensorFmt.toString().c_str());
            sess.config = camera->generateConfiguration({lc::StreamRole::Raw});
            sess.config->at(0).pixelFormat = sensorFmt;
            sess.config->at(0).size = sensorSize;
        }

        auto status = sess.config->validate();
        printf("  Validated: %s\n", status == lc::CameraConfiguration::Invalid ? "INVALID" :
               status == lc::CameraConfiguration::Adjusted ? "Adjusted" : "Valid");

        auto &sc = sess.config->at(0);
        printf("  Negotiated: %s %dx%d stride=%u\n",
               sc.pixelFormat.toString().c_str(),
               sc.size.width, sc.size.height, sc.stride);

        if (camera->configure(sess.config.get())) {
            printf("  ERROR: configure failed\n");
        } else {
            unsigned int w = sc.size.width, h = sc.size.height, stride = sc.stride;
            printf("  Actual: %s %dx%d stride=%u\n",
                   sc.pixelFormat.toString().c_str(), w, h, stride);

            if (setupSession(sess) && captureFrames(sess, 3, 200000, 4.0f)) {
                size_t dataLen;
                const uint8_t *data = getFrameData(dataLen);
                if (data) {
                    printf("  Frame captured: %zu bytes\n", dataLen);

                    std::string fmt = sc.pixelFormat.toString();
                    bool is16bit = fmt.find("16") != std::string::npos ||
                                   (isPiSP && fmt.find("PISP") == std::string::npos);
                    bool isCSI2P = fmt.find("CSI2P") != std::string::npos;
                    unsigned int bitDepth = isPiSP ? nativeBitDepth : nativeBitDepth;

                    // We need 16-bit pixel values
                    std::vector<uint16_t> pixels(w * h);

                    if (is16bit && !isCSI2P) {
                        // Data is already 16-bit per pixel
                        for (unsigned int row = 0; row < h; row++) {
                            const uint16_t *srcRow = reinterpret_cast<const uint16_t *>(
                                data + row * stride);
                            memcpy(&pixels[row * w], srcRow, w * sizeof(uint16_t));
                        }
                        printf("  Interpreted as 16-bit unpacked\n");
                    } else if (isCSI2P) {
                        // 10-bit packed CSI2P
                        for (unsigned int row = 0; row < h; row++) {
                            const uint8_t *srcRow = data + row * stride;
                            uint16_t *dstRow = &pixels[row * w];
                            size_t groups = w / 4;
                            for (size_t g = 0; g < groups; g++) {
                                uint8_t lsb = srcRow[4];
                                dstRow[0] = (uint16_t(srcRow[0]) << 2) | ((lsb >> 0) & 0x03);
                                dstRow[1] = (uint16_t(srcRow[1]) << 2) | ((lsb >> 2) & 0x03);
                                dstRow[2] = (uint16_t(srcRow[2]) << 2) | ((lsb >> 4) & 0x03);
                                dstRow[3] = (uint16_t(srcRow[3]) << 2) | ((lsb >> 6) & 0x03);
                                srcRow += 5;
                                dstRow += 4;
                            }
                        }
                        printf("  Unpacked from 10-bit CSI2P\n");
                    } else {
                        printf("  WARNING: unknown format, copying raw bytes\n");
                        memcpy(pixels.data(), data, std::min(dataLen, (size_t)w * h * 2));
                    }

                    // Stats
                    uint16_t minV = 65535, maxV = 0;
                    uint64_t sum = 0;
                    for (size_t i = 0; i < (size_t)w * h; i++) {
                        if (pixels[i] < minV) minV = pixels[i];
                        if (pixels[i] > maxV) maxV = pixels[i];
                        sum += pixels[i];
                    }
                    printf("  Pixel stats: min=%u  max=%u  mean=%.1f  (max possible=%u)\n",
                           minV, maxV, (double)sum / (w * h), (1u << bitDepth) - 1);

                    // Save as 16-bit PGM (big-endian per PGM spec)
                    FILE *f = fopen(rawFile, "wb");
                    if (f) {
                        fprintf(f, "P5\n%u %u\n65535\n", w, h);
                        // PGM 16-bit is big-endian
                        // Scale to full 16-bit range for visibility
                        float scale16 = 65535.0f / ((1 << bitDepth) - 1);
                        std::vector<uint8_t> pgmRow(w * 2);
                        for (unsigned int row = 0; row < h; row++) {
                            for (unsigned int col = 0; col < w; col++) {
                                uint16_t v = std::min(65535, (int)(pixels[row * w + col] * scale16));
                                pgmRow[col * 2 + 0] = (v >> 8) & 0xFF; // MSB first
                                pgmRow[col * 2 + 1] = v & 0xFF;
                            }
                            fwrite(pgmRow.data(), 1, w * 2, f);
                        }
                        fclose(f);
                        printf("  ✓ Saved raw Bayer as PGM: %s\n", rawFile);
                    }

                    // Demosaic and save as PPM
                    std::vector<uint8_t> rgb(w * h * 3, 0);
                    demosaicSimple(pixels.data(), w, h, w * 2,
                                  sensorPattern, rgb.data(), bitDepth);
                    FILE *f2 = fopen(rawPPM, "wb");
                    if (f2) {
                        fprintf(f2, "P6\n%u %u\n255\n", w, h);
                        fwrite(rgb.data(), 1, w * h * 3, f2);
                        fclose(f2);
                        printf("  ✓ Saved demosaiced as PPM: %s\n", rawPPM);
                    }
                }
            }
            stopSession(sess);
        }
    }

    // ============================================================
    //  TEST 2: RGB still capture → PPM
    // ============================================================
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  TEST 2: RGB still capture (ISP-processed)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    {
        CaptureSession sess;
        sess.camera = camera;
        sess.config = camera->generateConfiguration({lc::StreamRole::StillCapture});
        sess.config->at(0).pixelFormat = lc::formats::BGR888;
        sess.config->at(0).size = sensorSize;

        auto status = sess.config->validate();
        auto &sc = sess.config->at(0);
        printf("  Config: %s %dx%d stride=%u (%s)\n",
               sc.pixelFormat.toString().c_str(),
               sc.size.width, sc.size.height, sc.stride,
               status == lc::CameraConfiguration::Valid ? "Valid" : "Adjusted");

        if (camera->configure(sess.config.get())) {
            printf("  ERROR: configure failed\n");
        } else {
            unsigned int w = sc.size.width, h = sc.size.height, stride = sc.stride;
            std::string fmt = sc.pixelFormat.toString();
            int srcBpp = (fmt.find("X") != std::string::npos ||
                          fmt.find("A") != std::string::npos) ? 4 : 3;
            printf("  Actual: %s %dx%d stride=%u srcBpp=%d\n",
                   fmt.c_str(), w, h, stride, srcBpp);

            if (setupSession(sess)) {
                // RGB from the ISP requires auto-exposure to produce proper output.
                // Capture several frames with AE on to let the ISP converge.
                camera->requestCompleted.connect(requestComplete);
                if (camera->start()) {
                    printf("  ERROR: camera start failed\n");
                    camera->requestCompleted.disconnect(requestComplete);
                } else {
                int warmup = 8;
                printf("  Capturing %d frames (with AE) to let ISP converge...\n", warmup);
                for (int i = 0; i < warmup; i++) {
                    resetFrame();
                    auto &req = sess.requests[i % sess.requests.size()];
                    if (i > 0) req->reuse(lc::Request::ReuseBuffers);
                    req->controls().set(lc::controls::AeEnable, true);
                    req->controls().set(lc::controls::AnalogueGain, 4.0f);
                    camera->queueRequest(req.get());
                    if (!waitFrame(15)) { printf("  frame %d timeout\n", i); break; }
                    if (i < warmup - 1) printf("  warmup frame %d\n", i);
                }

                size_t dataLen;
                const uint8_t *data = getFrameData(dataLen);
                if (data) {
                    printf("  Frame captured: %zu bytes\n", dataLen);

                    // Check for all-zero
                    size_t nonZero = 0;
                    for (size_t i = 0; i < std::min(dataLen, (size_t)stride * h); i += 7)
                        if (data[i] != 0) nonZero++;
                    size_t sampled = std::min(dataLen, (size_t)stride * h) / 7;
                    printf("  Non-zero bytes (sampling): %zu / %zu (%.1f%%)\n",
                           nonZero, sampled, 100.0 * nonZero / sampled);

                    // Save as PPM (BGR → RGB conversion)
                    FILE *f = fopen(rgbFile, "wb");
                    if (f) {
                        fprintf(f, "P6\n%u %u\n255\n", w, h);
                        std::vector<uint8_t> rgbRow(w * 3);
                        for (unsigned int row = 0; row < h; row++) {
                            const uint8_t *srcRow = data + row * stride;
                            for (unsigned int col = 0; col < w; col++) {
                                if (srcBpp == 3) {
                                    rgbRow[col*3+0] = srcRow[col*3+2]; // R
                                    rgbRow[col*3+1] = srcRow[col*3+1]; // G
                                    rgbRow[col*3+2] = srcRow[col*3+0]; // B
                                } else {
                                    rgbRow[col*3+0] = srcRow[col*4+2]; // R
                                    rgbRow[col*3+1] = srcRow[col*4+1]; // G
                                    rgbRow[col*3+2] = srcRow[col*4+0]; // B
                                }
                            }
                            fwrite(rgbRow.data(), 1, w * 3, f);
                        }
                        fclose(f);
                        printf("  ✓ Saved RGB still as PPM: %s\n", rgbFile);
                    }
                }
                camera->stop();
                camera->requestCompleted.disconnect(requestComplete);
                }
            }
            unmapAll();
            sess.cleanup();
        }
    }

    // ============================================================
    //  TEST 3: MJPEG video (10 frames at streaming resolution)
    // ============================================================
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  TEST 3: MJPEG video capture (streaming path)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    {
        CaptureSession sess;
        sess.camera = camera;
        sess.config = camera->generateConfiguration({lc::StreamRole::VideoRecording});

        int vidW = 1280, vidH = 720;
        sess.config->at(0).pixelFormat = lc::formats::BGR888;
        sess.config->at(0).size = {(unsigned)vidW, (unsigned)vidH};

        auto status = sess.config->validate();
        auto &sc = sess.config->at(0);
        vidW = sc.size.width;
        vidH = sc.size.height;
        printf("  Config: %s %dx%d stride=%u (%s)\n",
               sc.pixelFormat.toString().c_str(), vidW, vidH, sc.stride,
               status == lc::CameraConfiguration::Valid ? "Valid" : "Adjusted");

        std::string fmt = sc.pixelFormat.toString();
        int srcBpp = (fmt.find("X") != std::string::npos ||
                      fmt.find("A") != std::string::npos) ? 4 : 3;

        if (camera->configure(sess.config.get())) {
            printf("  ERROR: configure failed\n");
        } else {
            unsigned int stride = sc.stride;
            printf("  Actual: %s %dx%d stride=%u srcBpp=%d\n",
                   sc.pixelFormat.toString().c_str(), vidW, vidH, stride, srcBpp);

            auto *stream = sc.stream();
            sess.allocator = std::make_unique<lc::FrameBufferAllocator>(camera);
            int nbufs = sess.allocator->allocate(stream);
            if (nbufs < 0) { printf("  ERROR: allocate failed\n"); goto done; }
            mapAllBuffers(*sess.allocator, stream);

            for (const auto &buf : sess.allocator->buffers(stream)) {
                auto req = camera->createRequest();
                req->addBuffer(stream, buf.get());
                sess.requests.push_back(std::move(req));
            }

            camera->requestCompleted.connect(requestComplete);
            if (camera->start()) {
                printf("  ERROR: start failed\n");
                camera->requestCompleted.disconnect(requestComplete);
                goto done;
            }

            FILE *mjpeg = fopen(videoFile, "wb");
            int totalFrames = 15; // 3 warmup + 12 real
            int savedFrames = 0;
            printf("  Capturing %d frames (%d warmup + %d video)...\n",
                   totalFrames, 3, totalFrames - 3);

            for (int i = 0; i < totalFrames; i++) {
                resetFrame();
                auto &req = sess.requests[i % sess.requests.size()];
                if (i > 0) req->reuse(lc::Request::ReuseBuffers);
                req->controls().set(lc::controls::ExposureTime, (int32_t)33000);
                req->controls().set(lc::controls::AnalogueGain, 4.0f);
                camera->queueRequest(req.get());

                if (!waitFrame(5)) {
                    printf("  frame %d timeout\n", i);
                    continue;
                }

                size_t dataLen;
                const uint8_t *data = getFrameData(dataLen);
                if (!data) continue;

                if (i < 3) {
                    printf("  warmup frame %d\n", i);
                    continue;
                }

                // Handle potential 4bpp format: convert to BGR888 first
                std::vector<uint8_t> bgr888;
                const uint8_t *bgrData = data;
                unsigned int bgrStride = stride;

                if (srcBpp == 4) {
                    bgr888.resize(vidW * vidH * 3);
                    for (int row = 0; row < vidH; row++) {
                        const uint8_t *srcRow = data + row * stride;
                        uint8_t *dstRow = bgr888.data() + row * vidW * 3;
                        for (int col = 0; col < vidW; col++) {
                            dstRow[col*3+0] = srcRow[col*4+0];
                            dstRow[col*3+1] = srcRow[col*4+1];
                            dstRow[col*3+2] = srcRow[col*4+2];
                        }
                    }
                    bgrData = bgr888.data();
                    bgrStride = vidW * 3;
                }

                auto jpeg = bgrToJpeg(bgrData, vidW, vidH, bgrStride, 85);

                if (mjpeg && !jpeg.empty()) {
                    fwrite(jpeg.data(), 1, jpeg.size(), mjpeg);
                    savedFrames++;
                    printf("  frame %d: JPEG %zu bytes\n", i, jpeg.size());
                }

                // Also save first video frame as a standalone JPEG for quick check
                if (savedFrames == 1) {
                    FILE *fj = fopen("/tmp/test_video_frame.jpg", "wb");
                    if (fj) {
                        fwrite(jpeg.data(), 1, jpeg.size(), fj);
                        fclose(fj);
                        printf("  ✓ Saved first frame as: /tmp/test_video_frame.jpg\n");
                    }
                }
            }

            if (mjpeg) fclose(mjpeg);

            camera->stop();
            camera->requestCompleted.disconnect(requestComplete);
            unmapAll();
            sess.cleanup();

            if (savedFrames > 0) {
                printf("  ✓ Saved %d-frame MJPEG: %s\n", savedFrames, videoFile);
                printf("    (Play with: mpv %s  or  ffplay %s)\n", videoFile, videoFile);
            } else {
                printf("  ✗ No video frames captured!\n");
            }
        }
    }

done:
    camera->release();
    cm->stop();

    printf("\n╔════════════════════════════════════════════════════╗\n");
    printf("║  Summary                                          ║\n");
    printf("╠════════════════════════════════════════════════════╣\n");
    printf("║  RAW Bayer (greyscale): %-26s ║\n", rawFile);
    printf("║  RAW demosaiced (colour): %-24s ║\n", rawPPM);
    printf("║  RGB still: %-38s ║\n", rgbFile);
    printf("║  Video frame: %-36s ║\n", "/tmp/test_video_frame.jpg");
    printf("║  MJPEG video: %-36s ║\n", videoFile);
    printf("╠════════════════════════════════════════════════════╣\n");
    printf("║  View with:                                       ║\n");
    printf("║    feh /tmp/test_raw.pgm                          ║\n");
    printf("║    feh /tmp/test_raw_demosaic.ppm                 ║\n");
    printf("║    feh /tmp/test_rgb.ppm                          ║\n");
    printf("║    feh /tmp/test_video_frame.jpg                  ║\n");
    printf("║    mpv /tmp/test_video.mjpeg                      ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");
    return 0;
}
