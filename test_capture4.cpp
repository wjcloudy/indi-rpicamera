/*
 * test_capture4.cpp — Test RGB/BGR output for stills and streaming on Pi 5
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
static std::map<const lc::FrameBuffer *, std::vector<std::pair<void*, size_t>>> g_mapped;

static void requestComplete(lc::Request *request) {
    if (request->status() == lc::Request::RequestCancelled) return;
    std::lock_guard<std::mutex> lock(g_mutex);
    g_completedRequest = request;
    g_frameReady = true;
    g_cv.notify_all();
}

void analyzeRGB(const char *label, const uint8_t *data, size_t dataLen,
                unsigned int width, unsigned int height, unsigned int stride, int bpp) {
    printf("\n--- %s: %dx%d stride=%u bpp=%d ---\n", label, width, height, stride, bpp);
    printf("Buffer: %zu bytes, expected: %u\n", dataLen, stride * height);

    // Check first few pixels
    printf("First 8 pixels (row 0): ");
    for (int i = 0; i < 8; i++) {
        if (bpp == 3)
            printf("[%u,%u,%u] ", data[i*3], data[i*3+1], data[i*3+2]);
        else
            printf("[%u,%u,%u,%u] ", data[i*4], data[i*4+1], data[i*4+2], data[i*4+3]);
    }
    printf("\n");

    // Center row
    unsigned int centerRow = height / 2;
    const uint8_t *cRow = data + centerRow * stride;
    printf("First 8 pixels (row %u): ", centerRow);
    for (int i = 0; i < 8; i++) {
        if (bpp == 3)
            printf("[%u,%u,%u] ", cRow[i*3], cRow[i*3+1], cRow[i*3+2]);
        else
            printf("[%u,%u,%u,%u] ", cRow[i*4], cRow[i*4+1], cRow[i*4+2], cRow[i*4+3]);
    }
    printf("\n");

    // Count non-zero pixels
    size_t nonZeroPixels = 0;
    size_t totalPixels = 0;
    for (unsigned int r = 0; r < height; r++) {
        const uint8_t *row = data + r * stride;
        for (unsigned int c = 0; c < width; c += 10) {
            totalPixels++;
            if (bpp == 3) {
                if (row[c*3] != 0 || row[c*3+1] != 0 || row[c*3+2] != 0)
                    nonZeroPixels++;
            } else {
                if (row[c*4] != 0 || row[c*4+1] != 0 || row[c*4+2] != 0)
                    nonZeroPixels++;
            }
        }
    }
    printf("Non-zero pixels (sampling 1/10): %zu / %zu (%.1f%%)\n",
           nonZeroPixels, totalPixels, 100.0 * nonZeroPixels / totalPixels);
}

int main() {
    printf("=== Test: RGB/BGR capture paths ===\n\n");

    auto cm = std::make_shared<lc::CameraManager>();
    cm->start();
    auto camera = cm->cameras()[0];
    camera->acquire();

    // ============================================================
    //  Test A: StillCapture with BGR888 (what INDI driver uses for RGB stills)
    // ============================================================
    printf("=== Test A: StillCapture BGR888 ===\n");
    {
        auto config = camera->generateConfiguration({lc::StreamRole::StillCapture});
        auto &sc = config->at(0);
        sc.pixelFormat = lc::formats::BGR888;
        sc.size = {3864, 2192};

        auto status = config->validate();
        printf("Validate: %s  ->  %s %dx%d stride=%u\n",
               status == lc::CameraConfiguration::Valid ? "Valid" : "Adjusted",
               sc.pixelFormat.toString().c_str(), sc.size.width, sc.size.height, sc.stride);

        camera->configure(config.get());
        printf("Configured: %s %dx%d stride=%u\n",
               sc.pixelFormat.toString().c_str(), sc.size.width, sc.size.height, sc.stride);

        unsigned int width = sc.size.width, height = sc.size.height, stride = sc.stride;

        // Determine bpp
        std::string fmt = sc.pixelFormat.toString();
        int bpp = (fmt.find("X") != std::string::npos || fmt.find("A") != std::string::npos) ? 4 : 3;
        printf("Detected bpp: %d\n", bpp);
        printf("Expected stride for %d bpp: %u\n", bpp, width * bpp);

        auto *stream = sc.stream();
        lc::FrameBufferAllocator alloc(camera);
        alloc.allocate(stream);

        for (const auto &buffer : alloc.buffers(stream)) {
            std::vector<std::pair<void*, size_t>> planes;
            for (const auto &plane : buffer->planes()) {
                void *mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, plane.fd.get(), plane.offset);
                if (mem != MAP_FAILED)
                    planes.push_back({mem, plane.length});
            }
            g_mapped[buffer.get()] = std::move(planes);
        }

        std::vector<std::unique_ptr<lc::Request>> requests;
        for (const auto &buffer : alloc.buffers(stream)) {
            auto req = camera->createRequest();
            req->addBuffer(stream, buffer.get());
            requests.push_back(std::move(req));
        }

        camera->requestCompleted.connect(requestComplete);
        camera->start();

        // Capture 3 frames for warmup
        for (int i = 0; i < 3; i++) {
            g_frameReady = false; g_completedRequest = nullptr;
            auto &req = requests[i % requests.size()];
            if (i > 0) req->reuse(lc::Request::ReuseBuffers);
            req->controls().set(lc::controls::ExposureTime, 100000);
            req->controls().set(lc::controls::AnalogueGain, 4.0f);
            camera->queueRequest(req.get());

            std::unique_lock<std::mutex> lock(g_mutex);
            g_cv.wait_for(lock, std::chrono::seconds(10), [] { return g_frameReady; });
        }

        if (g_frameReady) {
            const auto &bufs = g_completedRequest->buffers();
            for (const auto &[strm, fb] : bufs) {
                auto it = g_mapped.find(fb);
                if (it != g_mapped.end() && !it->second.empty()) {
                    analyzeRGB("StillCapture BGR888",
                               static_cast<const uint8_t*>(it->second[0].first),
                               it->second[0].second, width, height, stride, bpp);
                }
            }
        }

        camera->stop();
        camera->requestCompleted.disconnect(requestComplete);

        for (auto &[fb, planes] : g_mapped)
            for (auto &p : planes)
                if (p.first && p.first != MAP_FAILED)
                    munmap(p.first, p.second);
        g_mapped.clear();
    }

    // ============================================================
    //  Test B: VideoRecording with BGR888 (what INDI driver uses for streaming)
    // ============================================================
    printf("\n=== Test B: VideoRecording BGR888 (streaming path) ===\n");
    {
        auto config = camera->generateConfiguration({lc::StreamRole::VideoRecording});
        auto &sc = config->at(0);
        sc.pixelFormat = lc::formats::BGR888;
        sc.size = {1280, 720};

        auto status = config->validate();
        printf("Validate: %s  ->  %s %dx%d stride=%u\n",
               status == lc::CameraConfiguration::Valid ? "Valid" : "Adjusted",
               sc.pixelFormat.toString().c_str(), sc.size.width, sc.size.height, sc.stride);

        camera->configure(config.get());
        printf("Configured: %s %dx%d stride=%u\n",
               sc.pixelFormat.toString().c_str(), sc.size.width, sc.size.height, sc.stride);

        unsigned int width = sc.size.width, height = sc.size.height, stride = sc.stride;
        std::string fmt = sc.pixelFormat.toString();
        int bpp = (fmt.find("X") != std::string::npos || fmt.find("A") != std::string::npos) ? 4 : 3;
        printf("Detected bpp: %d  expected stride: %u\n", bpp, width * bpp);

        auto *stream = sc.stream();
        lc::FrameBufferAllocator alloc(camera);
        alloc.allocate(stream);

        for (const auto &buffer : alloc.buffers(stream)) {
            std::vector<std::pair<void*, size_t>> planes;
            for (const auto &plane : buffer->planes()) {
                void *mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, plane.fd.get(), plane.offset);
                if (mem != MAP_FAILED)
                    planes.push_back({mem, plane.length});
            }
            g_mapped[buffer.get()] = std::move(planes);
        }

        std::vector<std::unique_ptr<lc::Request>> requests;
        for (const auto &buffer : alloc.buffers(stream)) {
            auto req = camera->createRequest();
            req->addBuffer(stream, buffer.get());
            requests.push_back(std::move(req));
        }

        camera->requestCompleted.connect(requestComplete);
        camera->start();

        for (int i = 0; i < 3; i++) {
            g_frameReady = false; g_completedRequest = nullptr;
            auto &req = requests[i % requests.size()];
            if (i > 0) req->reuse(lc::Request::ReuseBuffers);
            req->controls().set(lc::controls::ExposureTime, 33000);
            req->controls().set(lc::controls::AnalogueGain, 4.0f);
            camera->queueRequest(req.get());

            std::unique_lock<std::mutex> lock(g_mutex);
            g_cv.wait_for(lock, std::chrono::seconds(10), [] { return g_frameReady; });
        }

        if (g_frameReady) {
            const auto &bufs = g_completedRequest->buffers();
            for (const auto &[strm, fb] : bufs) {
                auto it = g_mapped.find(fb);
                if (it != g_mapped.end() && !it->second.empty()) {
                    analyzeRGB("VideoRecording BGR888",
                               static_cast<const uint8_t*>(it->second[0].first),
                               it->second[0].second, width, height, stride, bpp);
                }
            }
        }

        camera->stop();
        camera->requestCompleted.disconnect(requestComplete);

        for (auto &[fb, planes] : g_mapped)
            for (auto &p : planes)
                if (p.first && p.first != MAP_FAILED)
                    munmap(p.first, p.second);
        g_mapped.clear();
    }

    camera->release();
    cm->stop();

    printf("\n=== All tests complete ===\n");
    return 0;
}
