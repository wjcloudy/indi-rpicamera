/*
 * test_capture.cpp — standalone libcamera test to diagnose raw capture
 *
 * Mimics what the INDI driver does:
 *   1. Open camera
 *   2. Request Raw stream with SGBRG10_CSI2P
 *   3. See what format libcamera actually negotiates
 *   4. Capture one frame and dump diagnostics
 *   5. Save raw data to a file for inspection
 *
 * Build:
 *   g++ -std=c++17 -O2 -o test_capture test_capture.cpp \
 *       $(pkg-config --cflags --libs libcamera)
 */

#include <libcamera/libcamera.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace lc = libcamera;

// Globals for the completion callback
static std::mutex g_mutex;
static std::condition_variable g_cv;
static bool g_frameReady = false;
static lc::Request *g_completedRequest = nullptr;

struct MappedPlane {
    void *memory = nullptr;
    size_t length = 0;
};
static std::map<const lc::FrameBuffer *, std::vector<MappedPlane>> g_mapped;

static void requestComplete(lc::Request *request) {
    if (request->status() == lc::Request::RequestCancelled)
        return;
    std::lock_guard<std::mutex> lock(g_mutex);
    g_completedRequest = request;
    g_frameReady = true;
    g_cv.notify_all();
}

int main() {
    printf("=== libcamera raw capture test ===\n\n");

    // 1. Start CameraManager
    auto cm = std::make_shared<lc::CameraManager>();
    int ret = cm->start();
    if (ret) {
        printf("ERROR: CameraManager::start() failed: %d\n", ret);
        return 1;
    }

    auto cameras = cm->cameras();
    printf("Found %zu camera(s)\n", cameras.size());
    if (cameras.empty()) {
        printf("No cameras found.\n");
        cm->stop();
        return 1;
    }

    auto camera = cameras[0];
    printf("Camera ID: %s\n", camera->id().c_str());

    if (camera->acquire()) {
        printf("ERROR: Failed to acquire camera\n");
        cm->stop();
        return 1;
    }

    // 2. Print camera properties
    const auto &props = camera->properties();
    auto model = props.get(lc::properties::Model);
    if (model)
        printf("Sensor model: %s\n", model->c_str());

    // 3. Generate Raw configuration
    printf("\n--- Generating Raw configuration ---\n");
    auto config = camera->generateConfiguration({lc::StreamRole::Raw});
    if (!config || config->empty()) {
        printf("ERROR: generateConfiguration(Raw) failed\n");
        camera->release();
        cm->stop();
        return 1;
    }

    auto &streamCfg = config->at(0);
    printf("Default raw config: %s %dx%d stride=%u\n",
           streamCfg.pixelFormat.toString().c_str(),
           streamCfg.size.width, streamCfg.size.height,
           streamCfg.stride);

    // 3a. List available raw formats
    printf("\n--- Available raw formats ---\n");
    auto formats = streamCfg.formats();
    for (const auto &pixFmt : formats.pixelformats()) {
        printf("  Format: %s\n", pixFmt.toString().c_str());
        for (const auto &sz : formats.sizes(pixFmt)) {
            printf("    Size: %dx%d\n", sz.width, sz.height);
        }
    }

    // 3b. Try to request the SGBRG10_CSI2P format explicitly (what the driver does)
    printf("\n--- Requesting SGBRG10_CSI2P 3864x2192 ---\n");
    streamCfg.pixelFormat = lc::PixelFormat(lc::formats::SGBRG10_CSI2P);
    streamCfg.size = {3864, 2192};

    printf("Before validate: %s %dx%d\n",
           streamCfg.pixelFormat.toString().c_str(),
           streamCfg.size.width, streamCfg.size.height);

    auto status = config->validate();
    const char *statusStr = (status == lc::CameraConfiguration::Valid) ? "Valid" :
                            (status == lc::CameraConfiguration::Adjusted) ? "ADJUSTED" :
                            "INVALID";
    printf("Validate result: %s\n", statusStr);
    printf("After validate: %s %dx%d stride=%u\n",
           streamCfg.pixelFormat.toString().c_str(),
           streamCfg.size.width, streamCfg.size.height,
           streamCfg.stride);

    // Check for PISP format
    std::string fmtStr = streamCfg.pixelFormat.toString();
    bool isPISP = (fmtStr.find("PISP") != std::string::npos ||
                   fmtStr.find("_COMP") != std::string::npos);
    printf("Is PISP compressed: %s\n", isPISP ? "YES" : "no");

    // 3c. Now also try a StillCapture role to see what that gives us
    printf("\n--- Generating StillCapture configuration ---\n");
    auto stillConfig = camera->generateConfiguration({lc::StreamRole::StillCapture});
    if (stillConfig && !stillConfig->empty()) {
        auto &sc = stillConfig->at(0);
        printf("Default still config: %s %dx%d stride=%u\n",
               sc.pixelFormat.toString().c_str(),
               sc.size.width, sc.size.height, sc.stride);
    }

    // 4. Configure and capture
    printf("\n--- Configuring camera ---\n");
    if (camera->configure(config.get())) {
        printf("ERROR: configure() failed\n");
        camera->release();
        cm->stop();
        return 1;
    }

    // Re-read actual config after configure
    printf("Actual config: %s %dx%d stride=%u\n",
           streamCfg.pixelFormat.toString().c_str(),
           streamCfg.size.width, streamCfg.size.height,
           streamCfg.stride);

    fmtStr = streamCfg.pixelFormat.toString();
    isPISP = (fmtStr.find("PISP") != std::string::npos ||
              fmtStr.find("_COMP") != std::string::npos);
    printf("Post-configure is PISP: %s\n", isPISP ? "YES" : "no");

    // Calculate expected sizes
    unsigned int width = streamCfg.size.width;
    unsigned int height = streamCfg.size.height;
    unsigned int stride = streamCfg.stride;
    printf("Expected buffer size (stride*height): %u bytes\n", stride * height);
    printf("If 10-bit packed CSI2P (5 bytes/4 pixels): %u bytes per row, total %u\n",
           (width * 5 + 3) / 4, ((width * 5 + 3) / 4) * height);
    printf("If 16-bit unpacked: %u bytes per row, total %u\n",
           width * 2, width * 2 * height);

    // 5. Allocate buffers
    auto *stream = streamCfg.stream();
    lc::FrameBufferAllocator allocator(camera);
    ret = allocator.allocate(stream);
    if (ret < 0) {
        printf("ERROR: allocate() failed: %d\n", ret);
        camera->release();
        cm->stop();
        return 1;
    }
    printf("\nAllocated %d buffer(s)\n", ret);

    // Map buffers
    std::vector<std::unique_ptr<lc::Request>> requests;
    for (const auto &buffer : allocator.buffers(stream)) {
        std::vector<MappedPlane> planes;
        for (const auto &plane : buffer->planes()) {
            void *mem = mmap(nullptr, plane.length,
                             PROT_READ | PROT_WRITE, MAP_SHARED,
                             plane.fd.get(), plane.offset);
            if (mem == MAP_FAILED) {
                printf("ERROR: mmap failed\n");
                camera->release();
                cm->stop();
                return 1;
            }
            planes.push_back({mem, plane.length});
            printf("Buffer plane: fd=%d offset=%u length=%u mapped=%p\n",
                   plane.fd.get(), plane.offset, plane.length, mem);
        }
        g_mapped[buffer.get()] = std::move(planes);

        auto request = camera->createRequest();
        request->addBuffer(stream, buffer.get());
        requests.push_back(std::move(request));
    }

    // 6. Connect completion signal and start
    camera->requestCompleted.connect(requestComplete);

    if (camera->start()) {
        printf("ERROR: start() failed\n");
        camera->release();
        cm->stop();
        return 1;
    }

    // Set exposure and queue one request
    auto &req = requests[0];
    req->controls().set(lc::controls::ExposureTime, 100000); // 100ms
    req->controls().set(lc::controls::AnalogueGain, 1.0f);
    camera->queueRequest(req.get());

    // Wait for frame
    printf("\nWaiting for frame...\n");
    {
        std::unique_lock<std::mutex> lock(g_mutex);
        g_cv.wait_for(lock, std::chrono::seconds(10), [] { return g_frameReady; });
    }

    if (!g_frameReady) {
        printf("ERROR: Timeout waiting for frame\n");
        camera->stop();
        camera->requestCompleted.disconnect(requestComplete);
        camera->release();
        cm->stop();
        return 1;
    }

    printf("Frame received!\n\n");

    // 7. Analyze the received buffer
    const auto &buffers = g_completedRequest->buffers();
    for (const auto &[stream_ptr, fb] : buffers) {
        printf("--- Buffer analysis ---\n");
        printf("Planes in buffer: %zu\n", fb->planes().size());
        const auto &metadata = fb->metadata();
        for (size_t i = 0; i < fb->planes().size(); i++) {
            const auto &plane = fb->planes()[i];
            unsigned int bytesused = (i < metadata.planes().size()) ? metadata.planes()[i].bytesused : 0;
            printf("  Plane %zu: bytesused=%u length=%u\n",
                   i, bytesused, plane.length);
        }

        auto it = g_mapped.find(fb);
        if (it != g_mapped.end() && !it->second.empty()) {
            const uint8_t *data = static_cast<const uint8_t *>(it->second[0].memory);
            size_t dataLen = it->second[0].length;
            size_t bytesUsed = metadata.planes().size() > 0 ? metadata.planes()[0].bytesused : dataLen;

            printf("\nMapped memory: %p, length=%zu\n", data, dataLen);
            printf("Bytes used (from metadata): %zu\n", bytesUsed);
            printf("Stride * height = %u\n", stride * height);

            // Print first 64 bytes
            printf("\nFirst 64 bytes of raw data:\n");
            for (int i = 0; i < 64 && i < (int)dataLen; i++) {
                printf("%02x ", data[i]);
                if ((i + 1) % 16 == 0) printf("\n");
            }
            printf("\n");

            // Print first few bytes of row 1 (at offset = stride)
            printf("First 32 bytes at row 1 (offset %u):\n", stride);
            if (dataLen > stride + 32) {
                for (int i = 0; i < 32; i++) {
                    printf("%02x ", data[stride + i]);
                    if ((i + 1) % 16 == 0) printf("\n");
                }
                printf("\n");
            }

            // Check if data looks like packed 10-bit (5 bytes per 4 pixels)
            // or 16-bit (2 bytes per pixel) or something else
            size_t expectedPacked10 = ((size_t)width * 5 + 3) / 4;
            size_t expected16bit = (size_t)width * 2;
            printf("\nStride analysis:\n");
            printf("  Actual stride: %u\n", stride);
            printf("  Expected for packed 10-bit: %zu\n", expectedPacked10);
            printf("  Expected for 16-bit: %zu\n", expected16bit);
            printf("  Stride matches packed 10: %s\n",
                   (stride == expectedPacked10 || stride >= expectedPacked10) ? "possible" : "no");
            printf("  Stride matches 16-bit: %s\n",
                   (stride == expected16bit || stride >= expected16bit) ? "possible" : "no");

            // Check for non-zero data
            size_t nonZero = 0;
            for (size_t i = 0; i < std::min(dataLen, bytesUsed); i++) {
                if (data[i] != 0) nonZero++;
            }
            printf("\nNon-zero bytes in buffer: %zu / %zu (%.1f%%)\n",
                   nonZero, std::min(dataLen, bytesUsed),
                   100.0 * nonZero / std::min(dataLen, bytesUsed));

            // Save first 1MB to a file for external analysis
            size_t saveLen = std::min(bytesUsed, dataLen);
            FILE *f = fopen("/tmp/test_raw_buffer.bin", "wb");
            if (f) {
                fwrite(data, 1, saveLen, f);
                fclose(f);
                printf("Saved %zu bytes to /tmp/test_raw_buffer.bin\n", saveLen);
            }

            // Also test unpacking the first row as 10-bit CSI2P
            if (fmtStr.find("CSI2P") != std::string::npos) {
                printf("\n--- Testing 10-bit CSI2P unpack on row 0 ---\n");
                std::vector<uint16_t> unpacked(width);
                // Unpack 4 pixels at a time from 5 bytes
                size_t groups = width / 4;
                const uint8_t *src = data;
                uint16_t *dst = unpacked.data();
                for (size_t g = 0; g < groups; g++) {
                    uint8_t lsb = src[4];
                    dst[0] = (static_cast<uint16_t>(src[0]) << 2) | ((lsb >> 0) & 0x03);
                    dst[1] = (static_cast<uint16_t>(src[1]) << 2) | ((lsb >> 2) & 0x03);
                    dst[2] = (static_cast<uint16_t>(src[2]) << 2) | ((lsb >> 4) & 0x03);
                    dst[3] = (static_cast<uint16_t>(src[3]) << 2) | ((lsb >> 6) & 0x03);
                    src += 5;
                    dst += 4;
                }

                printf("First 16 unpacked 10-bit pixel values:\n");
                for (int i = 0; i < 16 && i < (int)width; i++) {
                    printf("  [%d] = %u (0x%04x)\n", i, unpacked[i], unpacked[i]);
                }

                // Stats
                uint16_t minVal = 65535, maxVal = 0;
                uint64_t sum = 0;
                for (size_t i = 0; i < width; i++) {
                    if (unpacked[i] < minVal) minVal = unpacked[i];
                    if (unpacked[i] > maxVal) maxVal = unpacked[i];
                    sum += unpacked[i];
                }
                printf("Row 0 stats: min=%u max=%u mean=%.1f\n",
                       minVal, maxVal, (double)sum / width);
                printf("Expected range for 10-bit: 0-1023\n");
            } else {
                printf("\nFormat is NOT CSI2P packed.\n");
                printf("Interpreting as 16-bit per pixel:\n");
                const uint16_t *pixels = reinterpret_cast<const uint16_t *>(data);
                printf("First 16 pixel values:\n");
                for (int i = 0; i < 16 && i * 2 < (int)dataLen; i++) {
                    printf("  [%d] = %u (0x%04x)\n", i, pixels[i], pixels[i]);
                }
            }
        }
    }

    // Cleanup
    camera->stop();
    camera->requestCompleted.disconnect(requestComplete);

    for (auto &[fb, planes] : g_mapped) {
        for (auto &p : planes) {
            if (p.memory && p.memory != MAP_FAILED)
                munmap(p.memory, p.length);
        }
    }

    camera->release();
    cm->stop();

    printf("\n=== Test complete ===\n");
    return 0;
}
