/*
 * indi_rpicamera.h — INDI CCD driver for Raspberry Pi cameras via libcamera
 *
 * Uses the libcamera C++ API directly.  Subclasses INDI::CCD so that FITS
 * creation, BLOB transport, streaming (StreamManager), SER recording
 * (RecordManager), telescope snooping, configuration persistence, and all
 * standard CCD properties come from the framework for free.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#pragma once

// ---- INDI ----
#include <indiccd.h>
#include <indielapsedtimer.h>

// ---- libcamera ----
#include <libcamera/libcamera.h>

// ---- Standard library ----
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ---- Project ----
#include "sensor_info.h"

// =================================================================
// SensorMode — describes one raw sensor mode (size + pixel format)
// =================================================================

struct SensorMode
{
    libcamera::Size size;
    libcamera::PixelFormat format;
    unsigned int bitDepth{0};
    unsigned int hBin{1};
    unsigned int vBin{1};
    std::string label;                 ///< e.g. "4056x3040 12-bit RAW"
};

// =================================================================
// RPiCamera — main driver class
// =================================================================

class RPiCamera : public INDI::CCD
{
  public:
    /**
     * @param cm        Shared CameraManager (owned by the static Loader).
     * @param index     Zero-based index into CameraManager::cameras().
     * @param cameraId  The libcamera camera->id() string.
     */
    RPiCamera(std::shared_ptr<libcamera::CameraManager> cm,
              int index, const std::string &cameraId);
    ~RPiCamera() override;

    const char *getDefaultName() override;

    // ---- INDI::DefaultDevice overrides ----
    bool initProperties() override;
    bool updateProperties() override;
    bool Connect() override;
    bool Disconnect() override;
    bool ISNewNumber(const char *dev, const char *name, double values[],
                     char *names[], int n) override;
    bool ISNewSwitch(const char *dev, const char *name, ISState *states,
                     char *names[], int n) override;

    // ---- INDI::CCD overrides ----
    bool StartExposure(float duration) override;
    bool AbortExposure() override;
    bool UpdateCCDFrame(int x, int y, int w, int h) override;
    bool UpdateCCDBin(int binx, int biny) override;
    bool UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType) override;

    // ---- Streaming (via built-in StreamManager) ----
    bool StartStreaming() override;
    bool StopStreaming() override;

    // ---- FITS header augmentation ----
    void addFITSKeywords(INDI::CCDChip *targetChip,
                         std::vector<INDI::FITSRecord> &fitsKeywords) override;

  protected:
    void TimerHit() override;
    bool saveConfigItems(FILE *fp) override;

  private:
    // ============================================================
    //  libcamera state
    // ============================================================
    int m_CameraIndex{0};
    std::string m_CameraId;
    std::string m_DeviceName;
    std::shared_ptr<libcamera::CameraManager> m_CameraManager;
    std::shared_ptr<libcamera::Camera> m_Camera;
    std::unique_ptr<libcamera::CameraConfiguration> m_Config;
    std::unique_ptr<libcamera::FrameBufferAllocator> m_Allocator;
    std::vector<std::unique_ptr<libcamera::Request>> m_Requests;
    bool m_CameraRunning{false};

    // Memory-mapped DMA buffers
    struct MappedPlane
    {
        void *memory{nullptr};
        size_t length{0};
    };
    std::map<const libcamera::FrameBuffer *,
             std::vector<MappedPlane>> m_MappedBuffers;

    // ============================================================
    //  Exposure state
    // ============================================================
    INDI::ElapsedTimer m_ExposureTimer;
    double m_ExposureRequest{0};
    std::atomic<bool> m_FrameReady{false};
    std::atomic<bool> m_InExposure{false};
    libcamera::Request *m_CompletedRequest{nullptr};
    std::mutex m_CompletedMutex;

    // Active capture format / raw mode for the current exposure
    libcamera::PixelFormat m_ActivePixelFormat;
    libcamera::Size m_ActiveSize;
    bool m_ActiveIsRaw{true};

    // ============================================================
    //  Streaming state
    // ============================================================
    std::atomic<bool> m_IsStreaming{false};
    unsigned int m_StreamFrameCount{0};
    std::vector<uint8_t> m_StreamBuffer;  ///< Pre-allocated BGR→RGB swap buffer

    // Streaming FPS measurement
    std::chrono::steady_clock::time_point m_StreamFpsStart;
    unsigned int m_StreamFpsFrames{0};
    double m_StreamEstFps{0};

    // Streaming resolution preset
    enum { STREAM_720P, STREAM_1080P, STREAM_4K, STREAM_CUSTOM, STREAM_RES_COUNT };
    INDI::PropertySwitch StreamResSP{STREAM_RES_COUNT};
    INDI::PropertyNumber StreamCustomResNP{2}; ///< Custom width/height

    // Streaming target FPS
    INDI::PropertyNumber StreamFpsNP{1};

    // Streaming FPS display (read-only)
    INDI::PropertyNumber StreamEstFpsNP{1};

    // ============================================================
    //  Sensor information
    // ============================================================
    std::vector<SensorMode> m_SensorModes;
    int m_CurrentModeIndex{0};
    SensorAdjustment m_SensorAdj;
    std::string m_SensorModel;
    float m_PixelSizeUm{0};           ///< Pixel pitch in µm (from UnitCellSize)
    bool m_IsPiSP{false};             ///< True when Pi 5 PiSP backend is in use
    unsigned int m_NativeBitDepth{0}; ///< Sensor's native bit depth (before ISP promotion)
    float m_ExposureMinS{0.000001f};  ///< Min exposure in seconds (from hardware)
    float m_ExposureMaxS{600.0f};     ///< Max exposure in seconds (from hardware)

    // ============================================================
    //  Custom INDI properties — camera controls
    // ============================================================

    // Gain (AnalogueGain)
    INDI::PropertyNumber GainNP{1};

    // Auto-exposure on/off
    INDI::PropertySwitch AutoExposureSP{2};

    // Auto white-balance on/off
    INDI::PropertySwitch AutoWhiteBalanceSP{2};

    // AWB preset mode
    enum
    {
        AWB_AUTO, AWB_TUNGSTEN, AWB_FLUORESCENT, AWB_INDOOR,
        AWB_DAYLIGHT, AWB_CLOUDY, AWB_CUSTOM, AWB_COUNT
    };
    INDI::PropertySwitch AwbModeSP{AWB_COUNT};

    // Manual colour gains (red, blue) when AWB is off
    INDI::PropertyNumber ColourGainsNP{2};

    // ISP image tuning
    INDI::PropertyNumber BrightnessNP{1};
    INDI::PropertyNumber ContrastNP{1};
    INDI::PropertyNumber SaturationNP{1};
    INDI::PropertyNumber SharpnessNP{1};

    // Noise reduction
    enum { NR_OFF, NR_FAST, NR_HQ, NR_COUNT };
    INDI::PropertySwitch NoiseReductionSP{NR_COUNT};

    // Raw sensor mode selector (dynamic size, up to 16)
    static constexpr int MAX_SENSOR_MODES = 16;
    INDI::PropertySwitch RawFormatSP{MAX_SENSOR_MODES};
    int m_NumSensorModes{0};

    // Autofocus controls (optional — only if the sensor advertises AfMode)
    enum { AF_MANUAL, AF_AUTO, AF_CONTINUOUS, AF_MODE_COUNT };
    INDI::PropertySwitch AfModeSP{AF_MODE_COUNT};
    enum { AF_TRIGGER_START, AF_TRIGGER_CANCEL, AF_TRIGGER_COUNT };
    INDI::PropertySwitch AfTriggerSP{AF_TRIGGER_COUNT};
    bool m_HasAF{false};

    // Manual focus position (optional — only if the lens advertises LensPosition)
    INDI::PropertyNumber LensPositionNP{1};

    // Sensor temperature (read-only, updated from frame metadata)
    INDI::PropertyNumber TemperatureNP{1};

    // Raw left-shift: normalize raw data to fill 16-bit range
    // (shift left by 16−bitdepth).  Configurable, default ON.
    INDI::PropertySwitch RawLeftShiftSP{2};

    // AWB/AEC warmup: discard N initial frames so auto algorithms converge
    INDI::PropertyNumber WarmupFramesNP{1};

    // Fast Exposure mode: keep camera running between frames
    INDI::PropertySwitch FastExposureSP{2};   ///< ON / OFF toggle
    INDI::PropertyNumber FastCountNP{1};      ///< Number of frames (0 = unlimited until abort)
    std::atomic<int> m_FastFramesRemaining{0};
    std::atomic<bool> m_FastMode{false};

    // AE sub-mode controls (optional — only if camera advertises them)
    enum { AE_CONSTRAINT_NORMAL, AE_CONSTRAINT_HIGHLIGHT, AE_CONSTRAINT_SHADOWS, AE_CONSTRAINT_CUSTOM, AE_CONSTRAINT_COUNT };
    INDI::PropertySwitch AeConstraintModeSP{AE_CONSTRAINT_COUNT};
    bool m_HasAeConstraintMode{false};

    enum { AE_EXPOSURE_NORMAL, AE_EXPOSURE_SHORT, AE_EXPOSURE_LONG, AE_EXPOSURE_CUSTOM, AE_EXPOSURE_COUNT };
    INDI::PropertySwitch AeExposureModeSP{AE_EXPOSURE_COUNT};
    bool m_HasAeExposureMode{false};

    enum { AE_METERING_CENTRE, AE_METERING_SPOT, AE_METERING_MATRIX, AE_METERING_CUSTOM, AE_METERING_COUNT };
    INDI::PropertySwitch AeMeteringModeSP{AE_METERING_COUNT};
    bool m_HasAeMeteringMode{false};

    // Exposure Value (EV compensation, −8 to +8)
    INDI::PropertyNumber ExposureValueNP{1};
    bool m_HasExposureValue{false};

    // Extended AF controls (optional — only if the camera advertises them)
    enum { AF_METERING_AUTO, AF_METERING_WINDOWS, AF_METERING_COUNT };
    INDI::PropertySwitch AfMeteringSP{AF_METERING_COUNT};
    bool m_HasAfMetering{false};

    enum { AF_PAUSE_DEFERRED, AF_PAUSE_IMMEDIATE, AF_PAUSE_RESUME, AF_PAUSE_COUNT };
    INDI::PropertySwitch AfPauseSP{AF_PAUSE_COUNT};
    bool m_HasAfPause{false};

    enum { AF_RANGE_NORMAL, AF_RANGE_MACRO, AF_RANGE_FULL, AF_RANGE_COUNT };
    INDI::PropertySwitch AfRangeSP{AF_RANGE_COUNT};
    bool m_HasAfRange{false};

    enum { AF_SPEED_NORMAL, AF_SPEED_FAST, AF_SPEED_COUNT };
    INDI::PropertySwitch AfSpeedSP{AF_SPEED_COUNT};
    bool m_HasAfSpeed{false};

    // CCD_PROCFRAME — adjustable ISP output size for RGB/Mono
    INDI::PropertyNumber ProcFrameNP{2};  ///< Width, Height

    // Track which libcamera controls the connected camera actually supports
    bool m_HasAE{false};
    bool m_HasAWB{false};
    bool m_HasLensPosition{false};

    // ============================================================
    //  Frame metadata (extracted from completed request)
    // ============================================================
    double m_SensorTemperature{0};
    int32_t m_LastBlackLevels[4]{0, 0, 0, 0};
    int64_t m_LastActualExposureUs{0};
    float m_LastDigitalGain{0};
    std::string m_ExposureDateObs;
    std::string m_ExposureDateEnd;  ///< Wall-clock time at frame completion

    // Active capture format name for the current exposure
    std::string m_ActiveCaptureFmt;  ///< "INDI_RAW", "INDI_RGB", "INDI_RAW_MONO", or "INDI_MONO"

    // ============================================================
    //  Helper methods
    // ============================================================

    // Camera lifecycle
    bool configureForStill();
    bool configureForStreaming(int width, int height);
    bool startCamera();
    void stopCamera();
    void requestComplete(libcamera::Request *request);
    int  downloadImage();

    // Buffer management
    void mapBuffers(libcamera::Stream *stream);
    void unmapBuffers();
    void cleanupRequests();

    // Camera setup
    void enumerateSensorModes();
    void detectSensorAdjustments();
    void createCameraControlProperties();

    // Camera control application
    void applyCameraControls(libcamera::ControlList &ctrlList);

    // Metadata extraction from completed requests
    void readRequestMetadata(libcamera::Request *request);

    // Bayer / raw format utilities
    std::string bayerPatternFromFormat(const libcamera::PixelFormat &fmt) const;
    unsigned int bitDepthFromFormat(const libcamera::PixelFormat &fmt) const;
    bool isPackedCSI2(const libcamera::PixelFormat &fmt) const;
    libcamera::PixelFormat matchingBayer16Format(const libcamera::PixelFormat &rawFmt) const;
    static void unpack10bitCSI2(const uint8_t *src, uint16_t *dst,
                                size_t numPixels);
    static void unpack12bitCSI2(const uint8_t *src, uint16_t *dst,
                                size_t numPixels);

    // Left-shift raw pixel data to fill 16-bit range
    void applyRawLeftShift(uint16_t *data, size_t numPixels, unsigned int bitDepth);

    // RAW Mono: sum 2×2 Bayer superpixels into mono
    void convertRawToMono(const uint16_t *src, uint16_t *dst,
                          int width, int height);

    // Fast exposure: handle completed fast-mode frame
    void handleFastExposureFrame();

    // Garbage column count for current sensor mode
    int garbageColumnsForCurrentMode() const;
};
