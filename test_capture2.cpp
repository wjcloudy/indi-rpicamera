/*
 * test_capture2.cpp — test dual-stream approach on Pi 5
 *
 * On Pi 5, the Raw stream gives PISP-compressed data. rpicam-apps work
 * around this by using a dual-stream config: StillCapture + Raw, and then
 * the ISP decompresses the raw Bayer data.
 *
 * This test tries several approaches to get usable raw Bayer data.
 */

#include <libcamera/libcamera.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

#include <sys/mman.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>

namespace lc = libcamera;

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

void mapBuffers(std::shared_ptr<lc::Camera> &camera, lc::FrameBufferAllocator &alloc,
                lc::Stream *stream) {
    for (const auto &buffer : alloc.buffers(stream)) {
        std::vector<MappedPlane> planes;
        for (const auto &plane : buffer->planes()) {
            void *mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, plane.fd.get(), plane.offset);
            if (mem != MAP_FAILED)
                planes.push_back({mem, plane.length});
        }
        g_mapped[buffer.get()] = std::move(planes);
    }
}

int main() {
    printf("=== Pi 5 raw capture workaround test ===\n\n");

    auto cm = std::make_shared<lc::CameraManager>();
    cm->start();
    auto camera = cm->cameras()[0];
    camera->acquire();

    // ============================================================
    //  TEST 1: StillCapture + Raw dual stream (what rpicam-still does)
    // ============================================================
    printf("=== TEST 1: Dual-stream (StillCapture + Raw) ===\n");
    {
        auto config = camera->generateConfiguration(
            {lc::StreamRole::StillCapture, lc::StreamRole::Raw});
        if (config && config->size() >= 2) {
            printf("Stream 0 (StillCapture): %s %dx%d\n",
                   config->at(0).pixelFormat.toString().c_str(),
                   config->at(0).size.width, config->at(0).size.height);
            printf("Stream 1 (Raw): %s %dx%d\n",
                   config->at(1).pixelFormat.toString().c_str(),
                   config->at(1).size.width, config->at(1).size.height);

            // Try setting raw stream to the raw format
            // The ISP processes the PISP-compressed raw into a normal format
            auto status = config->validate();
            printf("After validate:\n");
            printf("  Stream 0: %s %dx%d stride=%u\n",
                   config->at(0).pixelFormat.toString().c_str(),
                   config->at(0).size.width, config->at(0).size.height,
                   config->at(0).stride);
            printf("  Stream 1: %s %dx%d stride=%u\n",
                   config->at(1).pixelFormat.toString().c_str(),
                   config->at(1).size.width, config->at(1).size.height,
                   config->at(1).stride);
            printf("  Status: %s\n",
                   status == lc::CameraConfiguration::Valid ? "Valid" :
                   status == lc::CameraConfiguration::Adjusted ? "Adjusted" : "Invalid");
        } else {
            printf("  Dual-stream config not supported or empty\n");
        }
    }

    // ============================================================
    //  TEST 2: StillCapture with explicit unpacked Bayer format
    // ============================================================
    printf("\n=== TEST 2: StillCapture with SGBRG16 (unpacked 16-bit Bayer) ===\n");
    {
        auto config = camera->generateConfiguration({lc::StreamRole::StillCapture});
        if (config && !config->empty()) {
            auto &sc = config->at(0);
            printf("Available StillCapture formats:\n");
            auto fmts = sc.formats();
            for (const auto &pf : fmts.pixelformats()) {
                printf("  %s:", pf.toString().c_str());
                for (const auto &sz : fmts.sizes(pf))
                    printf(" %dx%d", sz.width, sz.height);
                printf("\n");
            }

            // Try setting a Bayer format
            sc.pixelFormat = lc::formats::SGBRG16;
            sc.size = {3864, 2192};
            auto status = config->validate();
            printf("After requesting SGBRG16:\n");
            printf("  Result: %s  format: %s %dx%d stride=%u\n",
                   status == lc::CameraConfiguration::Valid ? "Valid" :
                   status == lc::CameraConfiguration::Adjusted ? "Adjusted" : "Invalid",
                   sc.pixelFormat.toString().c_str(),
                   sc.size.width, sc.size.height, sc.stride);
        }
    }

    // ============================================================
    //  TEST 3: StillCapture with explicit SGBRG10 (unpacked 10-bit)
    // ============================================================
    printf("\n=== TEST 3: StillCapture with SGBRG10 (unpacked 10-bit) ===\n");
    {
        auto config = camera->generateConfiguration({lc::StreamRole::StillCapture});
        if (config && !config->empty()) {
            auto &sc = config->at(0);
            sc.pixelFormat = lc::formats::SGBRG10;
            sc.size = {3864, 2192};
            auto status = config->validate();
            printf("  Result: %s  format: %s %dx%d stride=%u\n",
                   status == lc::CameraConfiguration::Valid ? "Valid" :
                   status == lc::CameraConfiguration::Adjusted ? "Adjusted" : "Invalid",
                   sc.pixelFormat.toString().c_str(),
                   sc.size.width, sc.size.height, sc.stride);
        }
    }

    // ============================================================
    //  TEST 4: Raw-only with SGBRG16 (Pi 5 ISP may accept this)
    // ============================================================
    printf("\n=== TEST 4: Raw with SGBRG16 ===\n");
    {
        auto config = camera->generateConfiguration({lc::StreamRole::Raw});
        if (config && !config->empty()) {
            auto &rc = config->at(0);
            printf("Raw available formats:\n");
            auto fmts = rc.formats();
            for (const auto &pf : fmts.pixelformats()) {
                printf("  %s:", pf.toString().c_str());
                for (const auto &sz : fmts.sizes(pf))
                    printf(" %dx%d", sz.width, sz.height);
                printf("\n");
            }

            rc.pixelFormat = lc::formats::SGBRG16;
            rc.size = {3864, 2192};
            auto status = config->validate();
            printf("After requesting SGBRG16:\n");
            printf("  Result: %s  format: %s %dx%d stride=%u\n",
                   status == lc::CameraConfiguration::Valid ? "Valid" :
                   status == lc::CameraConfiguration::Adjusted ? "Adjusted" : "Invalid",
                   rc.pixelFormat.toString().c_str(),
                   rc.size.width, rc.size.height, rc.stride);
        }
    }

    // ============================================================
    //  TEST 5: Actually capture a frame with dual-stream and read
    //          the raw stream data to see if it's decompressed
    // ============================================================
    printf("\n=== TEST 5: Dual-stream actual capture ===\n");
    {
        auto config = camera->generateConfiguration(
            {lc::StreamRole::StillCapture, lc::StreamRole::Raw});
        if (!config || config->size() < 2) {
            printf("Dual-stream not available\n");
            goto cleanup;
        }

        // Request BGR888 for still, let raw be whatever Pi 5 gives
        config->at(0).pixelFormat = lc::formats::BGR888;
        config->at(0).size = {3864, 2192};

        auto status = config->validate();
        printf("Config validated: %s\n",
               status == lc::CameraConfiguration::Valid ? "Valid" :
               status == lc::CameraConfiguration::Adjusted ? "Adjusted" : "Invalid");

        printf("Stream 0 (Still): %s %dx%d stride=%u\n",
               config->at(0).pixelFormat.toString().c_str(),
               config->at(0).size.width, config->at(0).size.height,
               config->at(0).stride);
        printf("Stream 1 (Raw):   %s %dx%d stride=%u\n",
               config->at(1).pixelFormat.toString().c_str(),
               config->at(1).size.width, config->at(1).size.height,
               config->at(1).stride);

        if (camera->configure(config.get())) {
            printf("Configure failed\n");
            goto cleanup;
        }

        printf("After configure:\n");
        printf("  Stream 0 (Still): %s %dx%d stride=%u\n",
               config->at(0).pixelFormat.toString().c_str(),
               config->at(0).size.width, config->at(0).size.height,
               config->at(0).stride);
        printf("  Stream 1 (Raw):   %s %dx%d stride=%u\n",
               config->at(1).pixelFormat.toString().c_str(),
               config->at(1).size.width, config->at(1).size.height,
               config->at(1).stride);

        auto *stream0 = config->at(0).stream();
        auto *stream1 = config->at(1).stream();

        lc::FrameBufferAllocator alloc(camera);
        alloc.allocate(stream0);
        alloc.allocate(stream1);
        mapBuffers(camera, alloc, stream0);
        mapBuffers(camera, alloc, stream1);

        // Create request with buffers for both streams
        auto request = camera->createRequest();
        request->addBuffer(stream0, alloc.buffers(stream0)[0].get());
        request->addBuffer(stream1, alloc.buffers(stream1)[0].get());

        camera->requestCompleted.connect(requestComplete);
        camera->start();

        request->controls().set(lc::controls::ExposureTime, 100000);
        request->controls().set(lc::controls::AnalogueGain, 1.0f);
        camera->queueRequest(request.get());

        {
            std::unique_lock<std::mutex> lock(g_mutex);
            g_cv.wait_for(lock, std::chrono::seconds(10), [] { return g_frameReady; });
        }

        if (g_frameReady) {
            printf("\nFrame received! Analyzing buffers:\n");
            const auto &bufs = g_completedRequest->buffers();
            for (const auto &[strm, fb] : bufs) {
                auto it = g_mapped.find(fb);
                if (it == g_mapped.end() || it->second.empty()) continue;

                const uint8_t *data = static_cast<const uint8_t *>(it->second[0].memory);
                size_t dataLen = it->second[0].length;

                // Figure out which stream this is
                const char *streamName = (strm == stream0) ? "StillCapture" : "Raw";
                unsigned int stride = (strm == stream0) ? config->at(0).stride : config->at(1).stride;
                std::string fmt = (strm == stream0)
                    ? config->at(0).pixelFormat.toString()
                    : config->at(1).pixelFormat.toString();
                unsigned int w = (strm == stream0) ? config->at(0).size.width : config->at(1).size.width;
                unsigned int h = (strm == stream0) ? config->at(0).size.height : config->at(1).size.height;

                printf("\n  [%s] format=%s %ux%u stride=%u bufLen=%zu\n",
                       streamName, fmt.c_str(), w, h, stride, dataLen);

                // Print first 32 bytes
                printf("  First 32 bytes: ");
                for (int i = 0; i < 32 && i < (int)dataLen; i++)
                    printf("%02x ", data[i]);
                printf("\n");

                // If this is the raw stream, check if data looks like 16-bit Bayer
                if (strm == stream1) {
                    printf("  Stride analysis for raw stream:\n");
                    printf("    stride=%u  w*2=%u (16-bit)  w*5/4=%u (10-bit packed)\n",
                           stride, w * 2, (w * 5 + 3) / 4);

                    // Check if it's 16-bit per pixel
                    if (stride >= w * 2) {
                        const uint16_t *pix = reinterpret_cast<const uint16_t *>(data);
                        printf("  Interpreting as 16-bit pixels:\n");
                        printf("    First 8: ");
                        for (int i = 0; i < 8; i++)
                            printf("%u ", pix[i]);
                        printf("\n");

                        // Stats on first row
                        uint16_t minv = 65535, maxv = 0;
                        for (unsigned int i = 0; i < w; i++) {
                            if (pix[i] < minv) minv = pix[i];
                            if (pix[i] > maxv) maxv = pix[i];
                        }
                        printf("    Row 0 range: %u - %u\n", minv, maxv);
                        if (maxv <= 1023)
                            printf("    >>> Looks like 10-bit range in 16-bit words!\n");
                        else if (maxv <= 4095)
                            printf("    >>> Looks like 12-bit range in 16-bit words!\n");
                        else if (maxv <= 16383)
                            printf("    >>> Looks like 14-bit range in 16-bit words!\n");
                        else
                            printf("    >>> Values exceed 14-bit — may be corrupt or compressed\n");
                    }

                    // Save raw stream data
                    FILE *f = fopen("/tmp/test_raw_stream1.bin", "wb");
                    if (f) {
                        fwrite(data, 1, dataLen, f);
                        fclose(f);
                        printf("  Saved to /tmp/test_raw_stream1.bin\n");
                    }
                }
            }
        } else {
            printf("Timeout!\n");
        }

        camera->stop();
        camera->requestCompleted.disconnect(requestComplete);
    }

cleanup:
    // Cleanup
    for (auto &[fb, planes] : g_mapped) {
        for (auto &p : planes)
            if (p.memory && p.memory != MAP_FAILED)
                munmap(p.memory, p.length);
    }
    g_mapped.clear();

    camera->release();
    cm->stop();

    printf("\n=== All tests complete ===\n");
    return 0;
}
