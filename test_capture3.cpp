/*
 * test_capture3.cpp — Verify Pi 5 ISP raw Bayer output via StillCapture role
 *
 * On Pi 5, Raw stream gives PISP compressed data (unusable without libpisp).
 * The ISP can output unpacked Bayer via StillCapture role with SxxxxNN format.
 * This test verifies that approach produces valid image data.
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

int main() {
    printf("=== Test: StillCapture with SGBRG16 for ISP-decompressed Bayer ===\n\n");

    auto cm = std::make_shared<lc::CameraManager>();
    cm->start();
    auto camera = cm->cameras()[0];
    camera->acquire();

    // ============================================================
    //  Configure StillCapture with SGBRG16
    // ============================================================
    auto config = camera->generateConfiguration({lc::StreamRole::StillCapture});
    if (!config || config->empty()) {
        printf("ERROR: No StillCapture config\n");
        camera->release(); cm->stop(); return 1;
    }

    auto &sc = config->at(0);
    sc.pixelFormat = lc::formats::SGBRG16;
    sc.size = {3864, 2192};

    auto status = config->validate();
    printf("Validate: %s\n",
           status == lc::CameraConfiguration::Valid ? "Valid" :
           status == lc::CameraConfiguration::Adjusted ? "Adjusted" : "Invalid");
    printf("Config: %s %dx%d stride=%u\n",
           sc.pixelFormat.toString().c_str(),
           sc.size.width, sc.size.height, sc.stride);

    if (camera->configure(config.get())) {
        printf("ERROR: configure() failed\n");
        camera->release(); cm->stop(); return 1;
    }

    printf("After configure: %s %dx%d stride=%u\n",
           sc.pixelFormat.toString().c_str(),
           sc.size.width, sc.size.height, sc.stride);

    unsigned int width = sc.size.width;
    unsigned int height = sc.size.height;
    unsigned int stride = sc.stride;

    // Allocate & map
    auto *stream = sc.stream();
    lc::FrameBufferAllocator alloc(camera);
    alloc.allocate(stream);

    std::vector<std::unique_ptr<lc::Request>> requests;
    for (const auto &buffer : alloc.buffers(stream)) {
        std::vector<MappedPlane> planes;
        for (const auto &plane : buffer->planes()) {
            void *mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, plane.fd.get(), plane.offset);
            if (mem != MAP_FAILED)
                planes.push_back({mem, plane.length});
            printf("Buffer: fd=%d len=%u\n", plane.fd.get(), plane.length);
        }
        g_mapped[buffer.get()] = std::move(planes);

        auto req = camera->createRequest();
        req->addBuffer(stream, buffer.get());
        requests.push_back(std::move(req));
    }

    // Start and capture multiple frames (Pi 5 ISP may need warmup)
    camera->requestCompleted.connect(requestComplete);
    camera->start();

    printf("\nCapturing 3 frames for warmup...\n");
    int capturedFrames = 0;
    for (int i = 0; i < 3; i++) {
        g_frameReady = false;
        g_completedRequest = nullptr;

        auto &req = requests[i % requests.size()];
        if (i > 0) req->reuse(lc::Request::ReuseBuffers);
        req->controls().set(lc::controls::ExposureTime, 100000); // 100ms
        req->controls().set(lc::controls::AnalogueGain, 1.0f);
        // Disable AE so exposure is manual
        req->controls().set(lc::controls::AeEnable, false);
        camera->queueRequest(req.get());

        std::unique_lock<std::mutex> lock(g_mutex);
        if (g_cv.wait_for(lock, std::chrono::seconds(10), [] { return g_frameReady; })) {
            capturedFrames++;
            printf("  Frame %d received\n", i);
        } else {
            printf("  Frame %d timeout!\n", i);
            break;
        }
    }

    if (!g_frameReady || !g_completedRequest) {
        printf("ERROR: No frame captured\n");
        camera->stop();
        camera->requestCompleted.disconnect(requestComplete);
        camera->release(); cm->stop(); return 1;
    }

    // Analyze the last frame
    printf("\n=== Analyzing last captured frame ===\n");
    const auto &bufs = g_completedRequest->buffers();
    for (const auto &[strm, fb] : bufs) {
        auto it = g_mapped.find(fb);
        if (it == g_mapped.end() || it->second.empty()) continue;

        const uint8_t *data = static_cast<const uint8_t *>(it->second[0].memory);
        size_t dataLen = it->second[0].length;

        printf("Buffer: %zu bytes\n", dataLen);
        printf("Expected: stride=%u * height=%u = %u bytes\n",
               stride, height, stride * height);

        // Interpret as 16-bit pixels
        const uint16_t *pixels = reinterpret_cast<const uint16_t *>(data);
        size_t numPixelsPerRow = stride / 2;

        // Row 0 stats
        uint16_t minVal = 65535, maxVal = 0;
        uint64_t sum = 0;
        for (unsigned int i = 0; i < width; i++) {
            uint16_t v = pixels[i];
            if (v < minVal) minVal = v;
            if (v > maxVal) maxVal = v;
            sum += v;
        }
        printf("\nRow 0: min=%u max=%u mean=%.1f\n", minVal, maxVal, (double)sum / width);

        // Center row stats
        unsigned int centerRow = height / 2;
        const uint16_t *centerPixels = reinterpret_cast<const uint16_t *>(
            data + centerRow * stride);
        minVal = 65535; maxVal = 0; sum = 0;
        for (unsigned int i = 0; i < width; i++) {
            uint16_t v = centerPixels[i];
            if (v < minVal) minVal = v;
            if (v > maxVal) maxVal = v;
            sum += v;
        }
        printf("Row %u (center): min=%u max=%u mean=%.1f\n",
               centerRow, minVal, maxVal, (double)sum / width);

        // Check overall data range
        if (maxVal <= 1023)
            printf(">>> Data in 10-bit range (0-1023) — sensor is 10-bit\n");
        else if (maxVal <= 4095)
            printf(">>> Data in 12-bit range (0-4095)\n");
        else if (maxVal <= 16383)
            printf(">>> Data in 14-bit range (0-16383)\n");
        else
            printf(">>> Data uses full 16-bit range or corrupt\n");

        // First 16 pixels
        printf("\nFirst 16 pixel values (row 0):\n");
        for (int i = 0; i < 16; i++)
            printf("  [%d] = %u\n", i, pixels[i]);

        printf("\nFirst 16 pixel values (center row):\n");
        for (int i = 0; i < 16; i++)
            printf("  [%d] = %u\n", i, centerPixels[i]);

        // Check for all-zero rows (corruption indicator)
        int zeroRows = 0;
        for (unsigned int r = 0; r < height; r++) {
            const uint16_t *row = reinterpret_cast<const uint16_t *>(data + r * stride);
            bool allZero = true;
            for (unsigned int c = 0; c < width && allZero; c += 100)
                if (row[c] != 0) allZero = false;
            if (allZero) zeroRows++;
        }
        printf("\nAll-zero rows (sampling every 100th pixel): %d / %u\n",
               zeroRows, height);

        // Save the full frame for external analysis
        FILE *f = fopen("/tmp/test_bayer16.bin", "wb");
        if (f) {
            // Save with stride (raw DMA buffer)
            fwrite(data, 1, (size_t)stride * height, f);
            fclose(f);
            printf("Saved %u bytes to /tmp/test_bayer16.bin\n", stride * height);
        }

        printf("\n>>> This data format is what the INDI driver should use on Pi 5!\n");
        printf(">>> Use StillCapture role with SGBRG16 instead of Raw role.\n");
    }

    camera->stop();
    camera->requestCompleted.disconnect(requestComplete);

    for (auto &[fb, planes] : g_mapped)
        for (auto &p : planes)
            if (p.memory && p.memory != MAP_FAILED)
                munmap(p.memory, p.length);

    camera->release();
    cm->stop();

    printf("\n=== Test complete ===\n");
    return 0;
}
