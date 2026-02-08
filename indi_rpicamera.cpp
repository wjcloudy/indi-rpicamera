/*
 * indi_rpicamera.cpp — INDI CCD driver for Raspberry Pi cameras via libcamera
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "indi_rpicamera.h"
#include "config.h"

#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <sstream>

// Convenience aliases
namespace lc = libcamera;

// ============================================================
//  Constructor / Destructor
// ============================================================

RPiCamera::RPiCamera(std::shared_ptr<lc::CameraManager> cm,
                     int index, const std::string &cameraId)
    : m_CameraIndex(index), m_CameraId(cameraId), m_CameraManager(cm)
{
    // Build a unique device name.  Single camera → "RPi Camera",
    // multiple → "RPi Camera #0", "RPi Camera #1", …
    auto cameras = m_CameraManager->cameras();
    if (cameras.size() <= 1)
        m_DeviceName = "RPi Camera";
    else
        m_DeviceName = "RPi Camera #" + std::to_string(index);

    setDeviceName(m_DeviceName.c_str());

    // Version
    setVersion(RPICAMERA_VERSION_MAJOR, RPICAMERA_VERSION_MINOR);
}

RPiCamera::~RPiCamera()
{
    if (m_CameraRunning)
        stopCamera();
    if (m_Camera)
    {
        m_Camera->release();
        m_Camera.reset();
    }
}

// ============================================================
//  Device Identity
// ============================================================

const char *RPiCamera::getDefaultName()
{
    return m_DeviceName.c_str();
}

// ============================================================
//  INDI Lifecycle — initProperties
// ============================================================

bool RPiCamera::initProperties()
{
    INDI::CCD::initProperties();

    // --- Capability flags ---
    uint32_t cap = CCD_CAN_ABORT
                 | CCD_CAN_BIN
                 | CCD_CAN_SUBFRAME
                 | CCD_HAS_BAYER
                 | CCD_HAS_STREAMING;
    SetCCDCapability(cap);

    // --- Capture format selector (framework creates the switch) ---
    addCaptureFormat({"INDI_RAW", "RAW", 16, true});
    addCaptureFormat({"INDI_RGB", "RGB", 8, false});

    // --- Gain ---
    GainNP[0].fill("GAIN", "Gain", "%.1f", 1.0, 16.0, 0.1, 1.0);
    GainNP.fill(getDeviceName(), "CCD_GAIN", "Gain",
                MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // All other camera-control properties are created dynamically in
    // createCameraControlProperties() after we know what the sensor supports.

    // Standard INDI properties (CCD_EXPOSURE, CCD_FRAME, CCD_BINNING,
    // CCD_INFO, ACTIVE_DEVICES, UPLOAD_MODE, STREAMING …) are already
    // created by INDI::CCD::initProperties().

    addDebugControl();
    addConfigurationControl();

    return true;
}

// ============================================================
//  INDI Lifecycle — updateProperties
// ============================================================

bool RPiCamera::updateProperties()
{
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        defineProperty(GainNP);

        // Camera-control properties were created in Connect() →
        // createCameraControlProperties().  Define them now.
        if (m_HasAE)
            defineProperty(AutoExposureSP);
        if (m_HasAWB)
        {
            defineProperty(AutoWhiteBalanceSP);
            defineProperty(AwbModeSP);
            defineProperty(ColourGainsNP);
        }
        defineProperty(BrightnessNP);
        defineProperty(ContrastNP);
        defineProperty(SaturationNP);
        defineProperty(SharpnessNP);
        defineProperty(NoiseReductionSP);

        if (m_NumSensorModes > 0)
            defineProperty(RawFormatSP);

        if (m_HasAF)
        {
            defineProperty(AfModeSP);
            defineProperty(AfTriggerSP);
        }

        SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(GainNP);

        if (m_HasAE)
            deleteProperty(AutoExposureSP);
        if (m_HasAWB)
        {
            deleteProperty(AutoWhiteBalanceSP);
            deleteProperty(AwbModeSP);
            deleteProperty(ColourGainsNP);
        }
        deleteProperty(BrightnessNP);
        deleteProperty(ContrastNP);
        deleteProperty(SaturationNP);
        deleteProperty(SharpnessNP);
        deleteProperty(NoiseReductionSP);

        if (m_NumSensorModes > 0)
            deleteProperty(RawFormatSP);

        if (m_HasAF)
        {
            deleteProperty(AfModeSP);
            deleteProperty(AfTriggerSP);
        }
    }

    return true;
}

// ============================================================
//  Connect / Disconnect
// ============================================================

bool RPiCamera::Connect()
{
    auto cameras = m_CameraManager->cameras();
    if (m_CameraIndex < 0 ||
        static_cast<size_t>(m_CameraIndex) >= cameras.size())
    {
        LOG_ERROR("Camera index out of range.  Is the camera connected?");
        return false;
    }

    m_Camera = cameras[m_CameraIndex];
    if (m_Camera->acquire())
    {
        LOG_ERROR("Failed to acquire camera — is another process using it?");
        m_Camera.reset();
        return false;
    }

    LOGF_INFO("Acquired camera: %s", m_CameraId.c_str());

    // ---- Read camera properties ----
    const auto &props = m_Camera->properties();
    auto model = props.get(lc::properties::Model);
    if (model)
    {
        m_SensorModel = *model;
        LOGF_INFO("Sensor model: %s", m_SensorModel.c_str());
    }

    // Pixel pitch (UnitCellSize is in nanometres)
    auto cellSize = props.get(lc::properties::UnitCellSize);
    if (cellSize)
    {
        m_PixelSizeUm = cellSize->width / 1000.0f;  // nm → µm
        LOGF_INFO("Pixel pitch: %.2f µm", m_PixelSizeUm);
    }

    // ---- Sensor modes & adjustments ----
    enumerateSensorModes();
    detectSensorAdjustments();

    // ---- Populate CCD_INFO from the first (largest) raw mode ----
    if (!m_SensorModes.empty())
    {
        auto &mode = m_SensorModes[0];
        float pixSizeX = m_PixelSizeUm > 0 ? m_PixelSizeUm : 1.55f;
        float pixSizeY = pixSizeX;  // square pixels assumed
        SetCCDParams(mode.size.width, mode.size.height,
                     mode.bitDepth, pixSizeX, pixSizeY);

        // Set Bayer pattern from the first raw mode's pixel format
        std::string bayer = bayerPatternFromFormat(mode.format);
        if (!bayer.empty())
        {
            IUSaveText(&BayerT[0], "0");              // X offset
            IUSaveText(&BayerT[1], "0");              // Y offset
            IUSaveText(&BayerT[2], bayer.c_str());    // e.g. "RGGB"
        }
    }

    // ---- Create camera-control properties based on what the sensor supports ----
    createCameraControlProperties();

    // ---- Allocate the default frame buffer ----
    PrimaryCCD.setFrameBufferSize(PrimaryCCD.getXRes() *
                                  PrimaryCCD.getYRes() * 2); // 16-bit raw

    LOG_INFO("RPi Camera connected successfully.");
    return true;
}

bool RPiCamera::Disconnect()
{
    if (m_CameraRunning)
        stopCamera();

    unmapBuffers();
    cleanupRequests();
    m_Allocator.reset();
    m_Config.reset();

    if (m_Camera)
    {
        m_Camera->release();
        m_Camera.reset();
    }

    LOG_INFO("RPi Camera disconnected.");
    return true;
}

// ============================================================
//  Camera Setup — Sensor Mode Enumeration
// ============================================================

void RPiCamera::enumerateSensorModes()
{
    m_SensorModes.clear();
    m_NumSensorModes = 0;

    if (!m_Camera)
        return;

    // Generate a raw config to discover the sensor's native formats & sizes
    auto config = m_Camera->generateConfiguration({lc::StreamRole::Raw});
    if (!config || config->empty())
    {
        LOG_WARN("Could not enumerate raw sensor modes.");
        return;
    }

    auto &streamCfg = config->at(0);
    auto formats = streamCfg.formats();

    for (const auto &pixFmt : formats.pixelformats())
    {
        for (const auto &sz : formats.sizes(pixFmt))
        {
            if (m_NumSensorModes >= MAX_SENSOR_MODES)
                break;

            SensorMode mode;
            mode.size   = sz;
            mode.format = pixFmt;
            mode.bitDepth = bitDepthFromFormat(pixFmt);

            // Infer binning: compare to the largest mode's size
            if (!m_SensorModes.empty())
            {
                auto &largest = m_SensorModes[0];
                mode.hBin = (largest.size.width  > 0 && sz.width  > 0)
                    ? largest.size.width  / sz.width  : 1;
                mode.vBin = (largest.size.height > 0 && sz.height > 0)
                    ? largest.size.height / sz.height : 1;
                if (mode.hBin < 1) mode.hBin = 1;
                if (mode.vBin < 1) mode.vBin = 1;
            }

            // Human-readable label
            std::ostringstream oss;
            oss << sz.width << "x" << sz.height
                << " " << mode.bitDepth << "-bit";
            if (mode.hBin > 1 || mode.vBin > 1)
                oss << " (bin " << mode.hBin << "x" << mode.vBin << ")";
            mode.label = oss.str();

            m_SensorModes.push_back(mode);
            m_NumSensorModes++;

            LOGF_DEBUG("Sensor mode %d: %s  format=%s",
                       m_NumSensorModes - 1, mode.label.c_str(),
                       pixFmt.toString().c_str());
        }
    }

    // Build the RawFormat switch property
    if (m_NumSensorModes > 0)
    {
        for (int i = 0; i < m_NumSensorModes; i++)
        {
            std::string name = "RAWFORMAT" + std::to_string(i);
            RawFormatSP[i].fill(name.c_str(), m_SensorModes[i].label.c_str(),
                                (i == 0) ? ISS_ON : ISS_OFF);
        }
        RawFormatSP.fill(getDeviceName(), "RAW_FORMAT", "Raw Mode",
                         IMAGE_SETTINGS_TAB, IP_RW, ISR_1OFMANY,
                         60, IPS_IDLE);
        RawFormatSP.shrink(m_NumSensorModes);
    }

    LOGF_INFO("Enumerated %d raw sensor mode(s).", m_NumSensorModes);
}

// ============================================================
//  Camera Setup — Sensor Adjustments
// ============================================================

void RPiCamera::detectSensorAdjustments()
{
    m_SensorAdj = lookupSensor(m_CameraId);
    if (!m_SensorAdj.friendlyName.empty())
        LOGF_INFO("Sensor identified: %s", m_SensorAdj.friendlyName.c_str());
    if (m_SensorAdj.forceRestart)
        LOG_INFO("Sensor requires forced camera restart between frames.");
}

// ============================================================
//  Camera Setup — Camera Control Properties
// ============================================================

void RPiCamera::createCameraControlProperties()
{
    if (!m_Camera)
        return;

    const auto &ctrlMap = m_Camera->controls();

    // --- Gain: update range from hardware ---
    {
        auto it = ctrlMap.find(&lc::controls::AnalogueGain);
        if (it != ctrlMap.end())
        {
            float minG = it->second.min().get<float>();
            float maxG = it->second.max().get<float>();
            float defG = it->second.def().get<float>();
            GainNP[0].setMinMax(minG, maxG);
            GainNP[0].setStep((maxG - minG) / 100.0);
            GainNP[0].setValue(defG);
        }
    }

    // --- Auto Exposure ---
    m_HasAE = ctrlMap.find(&lc::controls::AeEnable) != ctrlMap.end();
    if (m_HasAE)
    {
        AutoExposureSP[0].fill("AE_ON",  "On",  ISS_OFF);
        AutoExposureSP[1].fill("AE_OFF", "Off", ISS_ON);
        AutoExposureSP.fill(getDeviceName(), "AUTO_EXPOSURE", "Auto Exposure",
                            "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    }

    // --- Auto White Balance ---
    m_HasAWB = ctrlMap.find(&lc::controls::AwbEnable) != ctrlMap.end();
    if (m_HasAWB)
    {
        AutoWhiteBalanceSP[0].fill("AWB_ON",  "On",  ISS_ON);
        AutoWhiteBalanceSP[1].fill("AWB_OFF", "Off", ISS_OFF);
        AutoWhiteBalanceSP.fill(getDeviceName(), "AUTO_WHITE_BALANCE",
                                "Auto WB", "Camera Controls",
                                IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

        // AWB mode presets
        AwbModeSP[AWB_AUTO       ].fill("AWB_AUTO",        "Auto",        ISS_ON);
        AwbModeSP[AWB_TUNGSTEN   ].fill("AWB_TUNGSTEN",    "Tungsten",    ISS_OFF);
        AwbModeSP[AWB_FLUORESCENT].fill("AWB_FLUORESCENT", "Fluorescent", ISS_OFF);
        AwbModeSP[AWB_INDOOR     ].fill("AWB_INDOOR",      "Indoor",      ISS_OFF);
        AwbModeSP[AWB_DAYLIGHT   ].fill("AWB_DAYLIGHT",    "Daylight",    ISS_OFF);
        AwbModeSP[AWB_CLOUDY     ].fill("AWB_CLOUDY",      "Cloudy",      ISS_OFF);
        AwbModeSP[AWB_CUSTOM     ].fill("AWB_CUSTOM",      "Custom",      ISS_OFF);
        AwbModeSP.fill(getDeviceName(), "AWB_MODE", "AWB Mode",
                       "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

        // Manual colour gains (used when AWB is off)
        ColourGainsNP[0].fill("RED_GAIN",  "Red",  "%.2f", 0.0, 32.0, 0.1, 1.0);
        ColourGainsNP[1].fill("BLUE_GAIN", "Blue", "%.2f", 0.0, 32.0, 0.1, 1.0);
        ColourGainsNP.fill(getDeviceName(), "COLOUR_GAINS", "Colour Gains",
                           "Camera Controls", IP_RW, 60, IPS_IDLE);
    }

    // --- Brightness ---
    BrightnessNP[0].fill("BRIGHTNESS", "Brightness", "%.2f",
                         -1.0, 1.0, 0.05, 0.0);
    BrightnessNP.fill(getDeviceName(), "CCD_BRIGHTNESS", "Brightness",
                      "Camera Controls", IP_RW, 60, IPS_IDLE);

    // --- Contrast ---
    ContrastNP[0].fill("CONTRAST", "Contrast", "%.2f", 0.0, 32.0, 0.1, 1.0);
    ContrastNP.fill(getDeviceName(), "CCD_CONTRAST", "Contrast",
                    "Camera Controls", IP_RW, 60, IPS_IDLE);

    // --- Saturation ---
    SaturationNP[0].fill("SATURATION", "Saturation", "%.2f",
                         0.0, 32.0, 0.1, 1.0);
    SaturationNP.fill(getDeviceName(), "CCD_SATURATION", "Saturation",
                      "Camera Controls", IP_RW, 60, IPS_IDLE);

    // --- Sharpness ---
    SharpnessNP[0].fill("SHARPNESS", "Sharpness", "%.2f",
                        0.0, 16.0, 0.1, 1.0);
    SharpnessNP.fill(getDeviceName(), "CCD_SHARPNESS", "Sharpness",
                     "Camera Controls", IP_RW, 60, IPS_IDLE);

    // --- Noise Reduction ---
    NoiseReductionSP[NR_OFF ].fill("NR_OFF",  "Off",          ISS_ON);
    NoiseReductionSP[NR_FAST].fill("NR_FAST", "Fast",         ISS_OFF);
    NoiseReductionSP[NR_HQ  ].fill("NR_HQ",   "High Quality", ISS_OFF);
    NoiseReductionSP.fill(getDeviceName(), "NOISE_REDUCTION", "Noise Reduction",
                          "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // --- Autofocus (optional) ---
    m_HasAF = ctrlMap.find(&lc::controls::AfMode) != ctrlMap.end();
    if (m_HasAF)
    {
        AfModeSP[AF_MANUAL    ].fill("AF_MANUAL",     "Manual",     ISS_ON);
        AfModeSP[AF_AUTO      ].fill("AF_AUTO",       "Auto",       ISS_OFF);
        AfModeSP[AF_CONTINUOUS].fill("AF_CONTINUOUS",  "Continuous", ISS_OFF);
        AfModeSP.fill(getDeviceName(), "AF_MODE", "AF Mode",
                      "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

        AfTriggerSP[AF_TRIGGER_START ].fill("AF_START",  "Start",  ISS_OFF);
        AfTriggerSP[AF_TRIGGER_CANCEL].fill("AF_CANCEL", "Cancel", ISS_OFF);
        AfTriggerSP.fill(getDeviceName(), "AF_TRIGGER", "AF Trigger",
                         "Camera Controls", IP_RW, ISR_ATMOST1, 60, IPS_IDLE);
    }
}

// ============================================================
//  Exposure — StartExposure
// ============================================================

bool RPiCamera::StartExposure(float duration)
{
    if (m_InExposure)
    {
        LOG_ERROR("An exposure is already in progress.");
        return false;
    }

    if (duration < 0.001f)
        duration = 0.001f;

    m_ExposureRequest = duration;
    PrimaryCCD.setExposureDuration(duration);

    // ---- Determine capture format ----
    std::string capFmt = GetCaptureFormat();
    m_ActiveIsRaw = (capFmt == "INDI_RAW");

    // Stop any previous camera session
    if (m_CameraRunning)
        stopCamera();
    unmapBuffers();
    cleanupRequests();
    m_Allocator.reset();
    m_Config.reset();

    // ---- Configure and start ----
    bool ok = m_ActiveIsRaw ? configureForStill() : configureForStill();
    if (!ok)
    {
        LOG_ERROR("Failed to configure camera for exposure.");
        return false;
    }

    if (!startCamera())
    {
        LOG_ERROR("Failed to start camera.");
        return false;
    }

    // ---- Set controls on the first queued request ----
    {
        auto &req = m_Requests[0];
        int64_t expUs = static_cast<int64_t>(duration * 1e6);
        req->controls().set(lc::controls::ExposureTime, static_cast<int32_t>(expUs));
        req->controls().set(lc::controls::AnalogueGain,
                            static_cast<float>(GainNP[0].getValue()));
        applyCameraControls(req->controls());
    }

    // Queue just one request for a single-frame capture
    m_Camera->queueRequest(m_Requests[0].get());

    m_ExposureTimer.start();
    m_InExposure  = true;
    m_FrameReady  = false;

    LOGF_INFO("Starting %.3f s exposure (%s).",
              duration, m_ActiveIsRaw ? "RAW" : "RGB");
    return true;
}

// ============================================================
//  Exposure — AbortExposure
// ============================================================

bool RPiCamera::AbortExposure()
{
    if (!m_InExposure)
        return true;

    stopCamera();
    m_InExposure = false;
    m_FrameReady = false;
    LOG_INFO("Exposure aborted.");
    return true;
}

// ============================================================
//  TimerHit — Exposure countdown & polling
// ============================================================

void RPiCamera::TimerHit()
{
    if (!isConnected())
        return;

    if (m_InExposure)
    {
        double elapsed = m_ExposureTimer.elapsed() / 1000.0; // ms → s
        double timeLeft = m_ExposureRequest - elapsed;
        if (timeLeft < 0)
            timeLeft = 0;

        PrimaryCCD.setExposureLeft(timeLeft);

        if (m_FrameReady)
        {
            m_InExposure = false;
            downloadImage();
        }
    }

    SetTimer(getCurrentPollingPeriod());
}

// ============================================================
//  Exposure — downloadImage (copy DMA buffer → INDI frame buffer)
// ============================================================

int RPiCamera::downloadImage()
{
    std::lock_guard<std::mutex> lock(m_CompletedMutex);

    if (!m_CompletedRequest)
    {
        LOG_ERROR("No completed request available.");
        return -1;
    }

    // Find the buffer from the completed request
    const auto &buffers = m_CompletedRequest->buffers();
    if (buffers.empty())
    {
        LOG_ERROR("Completed request has no buffers.");
        m_CompletedRequest = nullptr;
        m_FrameReady = false;
        return -1;
    }

    const lc::FrameBuffer *fb = buffers.begin()->second;
    auto it = m_MappedBuffers.find(fb);
    if (it == m_MappedBuffers.end() || it->second.empty())
    {
        LOG_ERROR("Frame buffer not mapped.");
        m_CompletedRequest = nullptr;
        m_FrameReady = false;
        return -1;
    }

    const uint8_t *srcData = static_cast<const uint8_t *>(it->second[0].memory);
    size_t srcLen = it->second[0].length;

    int subX = PrimaryCCD.getSubX();
    int subY = PrimaryCCD.getSubY();
    int subW = PrimaryCCD.getSubW();
    int subH = PrimaryCCD.getSubH();

    if (m_ActiveIsRaw)
    {
        // RAW capture — Bayer data
        int bpp = 16;
        PrimaryCCD.setBPP(bpp);
        PrimaryCCD.setNAxis(2);

        size_t frameBytes = static_cast<size_t>(subW) * subH * (bpp / 8);
        PrimaryCCD.setFrameBufferSize(frameBytes);

        uint16_t *dstBuf = reinterpret_cast<uint16_t *>(
            PrimaryCCD.getFrameBuffer());

        int fullW = static_cast<int>(m_ActiveSize.width);

        if (isPackedCSI2(m_ActivePixelFormat))
        {
            // Unpack the full raw frame, then extract the subframe.
            unsigned int bd = bitDepthFromFormat(m_ActivePixelFormat);
            size_t totalPixels = static_cast<size_t>(m_ActiveSize.width) *
                                 m_ActiveSize.height;

            std::vector<uint16_t> unpacked(totalPixels);
            if (bd == 10)
                unpack10bitCSI2(srcData, unpacked.data(), totalPixels);
            else if (bd == 12)
                unpack12bitCSI2(srcData, unpacked.data(), totalPixels);
            else
                std::memcpy(unpacked.data(), srcData,
                            std::min(totalPixels * 2, srcLen));

            // Copy subframe from the unpacked full frame
            for (int row = 0; row < subH; row++)
            {
                const uint16_t *srcRow =
                    unpacked.data() + (subY + row) * fullW + subX;
                uint16_t *dstRow = dstBuf + row * subW;
                std::memcpy(dstRow, srcRow, subW * sizeof(uint16_t));
            }
        }
        else
        {
            // Unpacked 16-bit raw (or 10/12-bit in 16-bit container)
            const uint16_t *srcBuf =
                reinterpret_cast<const uint16_t *>(srcData);

            for (int row = 0; row < subH; row++)
            {
                const uint16_t *srcRow =
                    srcBuf + (subY + row) * fullW + subX;
                uint16_t *dstRow = dstBuf + row * subW;
                std::memcpy(dstRow, srcRow, subW * sizeof(uint16_t));
            }
        }
    }
    else
    {
        // RGB capture — ISP-processed output
        // Data from libcamera is BGR888 (R, G, B in memory with our format choice)
        int bpp = 8;
        PrimaryCCD.setBPP(bpp);
        PrimaryCCD.setNAxis(2);

        // Store as interleaved RGB: buffer size = W * H * 3
        size_t frameBytes = static_cast<size_t>(subW) * subH * 3;
        PrimaryCCD.setFrameBufferSize(frameBytes);

        uint8_t *dstBuf = PrimaryCCD.getFrameBuffer();

        int fullW = static_cast<int>(m_ActiveSize.width);
        int bytesPerPixel = 3;

        for (int row = 0; row < subH; row++)
        {
            const uint8_t *srcRow =
                srcData + ((subY + row) * fullW + subX) * bytesPerPixel;
            uint8_t *dstRow = dstBuf + row * subW * bytesPerPixel;
            std::memcpy(dstRow, srcRow, subW * bytesPerPixel);
        }
    }

    // Stop camera (single-shot capture)
    stopCamera();

    m_CompletedRequest = nullptr;
    m_FrameReady = false;

    LOGF_INFO("Download complete: %dx%d  offset (%d,%d)",
              subW, subH, subX, subY);

    ExposureComplete(&PrimaryCCD);
    return 0;
}

// ============================================================
//  Frame Geometry
// ============================================================

bool RPiCamera::UpdateCCDFrame(int x, int y, int w, int h)
{
    // Clamp to sensor dimensions
    int maxW = PrimaryCCD.getXRes();
    int maxH = PrimaryCCD.getYRes();

    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (w <= 0) w = maxW;
    if (h <= 0) h = maxH;
    if (x + w > maxW) w = maxW - x;
    if (y + h > maxH) h = maxH - y;

    PrimaryCCD.setFrame(x, y, w, h);

    size_t frameBytes = static_cast<size_t>(w) * h *
        (PrimaryCCD.getBPP() / 8);
    if (!m_ActiveIsRaw)
        frameBytes = static_cast<size_t>(w) * h * 3; // RGB interleaved

    PrimaryCCD.setFrameBufferSize(frameBytes);

    LOGF_DEBUG("CCD frame: %dx%d at (%d,%d)", w, h, x, y);
    return true;
}

bool RPiCamera::UpdateCCDBin(int binx, int biny)
{
    // Find a sensor mode that matches the requested binning
    for (int i = 0; i < m_NumSensorModes; i++)
    {
        auto &mode = m_SensorModes[i];
        if (static_cast<int>(mode.hBin) == binx &&
            static_cast<int>(mode.vBin) == biny)
        {
            m_CurrentModeIndex = i;
            PrimaryCCD.setBin(binx, biny);

            // Update resolution to the binned mode
            SetCCDParams(mode.size.width, mode.size.height,
                         mode.bitDepth,
                         PrimaryCCD.getPixelSizeX() * binx,
                         PrimaryCCD.getPixelSizeY() * biny);

            // Update Bayer pattern
            std::string bayer = bayerPatternFromFormat(mode.format);
            if (!bayer.empty())
                IUSaveText(&BayerT[2], bayer.c_str());

            // Update raw format switch to match
            RawFormatSP.reset();
            RawFormatSP[i].setState(ISS_ON);
            RawFormatSP.apply();

            LOGF_INFO("Binning set to %dx%d → sensor mode: %s",
                      binx, biny, mode.label.c_str());
            return true;
        }
    }

    LOGF_ERROR("No sensor mode found for binning %dx%d", binx, biny);
    return false;
}

bool RPiCamera::UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType)
{
    PrimaryCCD.setFrameType(fType);
    return true;
}

// ============================================================
//  Streaming — Start / Stop
// ============================================================

bool RPiCamera::StartStreaming()
{
    if (m_InExposure)
    {
        LOG_ERROR("Cannot start streaming while an exposure is in progress.");
        return false;
    }

    if (m_CameraRunning)
        stopCamera();
    unmapBuffers();
    cleanupRequests();
    m_Allocator.reset();
    m_Config.reset();

    // Cap streaming resolution to 1280 on the longest side
    int maxDim = 1280;
    int sw = PrimaryCCD.getXRes();
    int sh = PrimaryCCD.getYRes();
    if (sw > maxDim || sh > maxDim)
    {
        double scale = static_cast<double>(maxDim) / std::max(sw, sh);
        sw = static_cast<int>(sw * scale) & ~1; // even dimensions
        sh = static_cast<int>(sh * scale) & ~1;
    }

    if (!configureForStreaming(sw, sh))
    {
        LOG_ERROR("Failed to configure camera for streaming.");
        return false;
    }

    // Tell StreamManager about the format
    Streamer->setPixelFormat(INDI_RGB, 8);
    Streamer->setSize(sw, sh);

    if (!startCamera())
    {
        LOG_ERROR("Failed to start camera for streaming.");
        return false;
    }

    // Set default streaming controls
    for (auto &req : m_Requests)
    {
        double streamExp = Streamer->getExposure();  // seconds
        int32_t expUs = static_cast<int32_t>(streamExp * 1e6);
        if (expUs < 100) expUs = 100;  // minimum

        req->controls().set(lc::controls::ExposureTime, expUs);
        req->controls().set(lc::controls::AnalogueGain,
                            static_cast<float>(GainNP[0].getValue()));
        applyCameraControls(req->controls());
    }

    // Queue all requests for continuous capture pipelining
    for (auto &req : m_Requests)
        m_Camera->queueRequest(req.get());

    m_IsStreaming = true;
    LOGF_INFO("Streaming started at %dx%d.", sw, sh);
    return true;
}

bool RPiCamera::StopStreaming()
{
    m_IsStreaming = false;

    if (m_CameraRunning)
        stopCamera();

    LOG_INFO("Streaming stopped.");
    return true;
}

// ============================================================
//  libcamera — Configuration helpers
// ============================================================

bool RPiCamera::configureForStill()
{
    if (!m_Camera)
        return false;

    std::string capFmt = GetCaptureFormat();

    if (capFmt == "INDI_RAW")
    {
        // Raw Bayer capture from the sensor
        auto &mode = m_SensorModes[m_CurrentModeIndex];
        m_Config = m_Camera->generateConfiguration({lc::StreamRole::Raw});
        if (!m_Config || m_Config->empty())
            return false;

        m_Config->at(0).pixelFormat = mode.format;
        m_Config->at(0).size = mode.size;
        m_ActivePixelFormat = mode.format;
        m_ActiveSize = mode.size;
        m_ActiveIsRaw = true;

        // Update CCD params if needed
        PrimaryCCD.setBPP(16);
        PrimaryCCD.setNAxis(2);
    }
    else
    {
        // ISP-processed RGB output
        m_Config = m_Camera->generateConfiguration(
            {lc::StreamRole::StillCapture});
        if (!m_Config || m_Config->empty())
            return false;

        int w = PrimaryCCD.getSubW();
        int h = PrimaryCCD.getSubH();
        if (w <= 0) w = PrimaryCCD.getXRes();
        if (h <= 0) h = PrimaryCCD.getYRes();

        m_Config->at(0).pixelFormat = lc::formats::BGR888;
        m_Config->at(0).size = {static_cast<unsigned>(w),
                                static_cast<unsigned>(h)};
        m_ActivePixelFormat = lc::formats::BGR888;
        m_ActiveSize = {static_cast<unsigned>(w),
                        static_cast<unsigned>(h)};
        m_ActiveIsRaw = false;

        PrimaryCCD.setBPP(8);
        PrimaryCCD.setNAxis(2);
    }

    auto status = m_Config->validate();
    if (status == lc::CameraConfiguration::Invalid)
    {
        LOG_ERROR("Camera configuration is invalid.");
        return false;
    }
    if (status == lc::CameraConfiguration::Adjusted)
        LOG_WARN("Camera configuration was adjusted by the driver.");

    if (m_Camera->configure(m_Config.get()))
    {
        LOG_ERROR("Failed to apply camera configuration.");
        return false;
    }

    // ---- Allocate frame buffers ----
    auto *stream = m_Config->at(0).stream();
    m_Allocator = std::make_unique<lc::FrameBufferAllocator>(m_Camera);
    int ret = m_Allocator->allocate(stream);
    if (ret < 0)
    {
        LOG_ERROR("Failed to allocate frame buffers.");
        return false;
    }

    LOGF_DEBUG("Allocated %d frame buffer(s).", ret);

    mapBuffers(stream);

    // ---- Create requests ----
    for (const auto &buffer : m_Allocator->buffers(stream))
    {
        auto request = m_Camera->createRequest();
        if (!request)
        {
            LOG_ERROR("Failed to create request.");
            return false;
        }
        if (request->addBuffer(stream, buffer.get()))
        {
            LOG_ERROR("Failed to add buffer to request.");
            return false;
        }
        m_Requests.push_back(std::move(request));
    }

    return true;
}

bool RPiCamera::configureForStreaming(int width, int height)
{
    if (!m_Camera)
        return false;

    m_Config = m_Camera->generateConfiguration(
        {lc::StreamRole::VideoRecording});
    if (!m_Config || m_Config->empty())
        return false;

    m_Config->at(0).pixelFormat = lc::formats::BGR888;
    m_Config->at(0).size = {static_cast<unsigned>(width),
                            static_cast<unsigned>(height)};
    m_ActivePixelFormat = lc::formats::BGR888;
    m_ActiveSize = {static_cast<unsigned>(width),
                    static_cast<unsigned>(height)};
    m_ActiveIsRaw = false;

    auto status = m_Config->validate();
    if (status == lc::CameraConfiguration::Invalid)
    {
        LOG_ERROR("Streaming configuration is invalid.");
        return false;
    }

    if (m_Camera->configure(m_Config.get()))
    {
        LOG_ERROR("Failed to apply streaming configuration.");
        return false;
    }

    auto *stream = m_Config->at(0).stream();
    m_Allocator = std::make_unique<lc::FrameBufferAllocator>(m_Camera);
    int ret = m_Allocator->allocate(stream);
    if (ret < 0)
    {
        LOG_ERROR("Failed to allocate streaming buffers.");
        return false;
    }

    mapBuffers(stream);

    for (const auto &buffer : m_Allocator->buffers(stream))
    {
        auto request = m_Camera->createRequest();
        if (!request)
            return false;
        if (request->addBuffer(stream, buffer.get()))
            return false;
        m_Requests.push_back(std::move(request));
    }

    return true;
}

// ============================================================
//  libcamera — Start / Stop / Request completion
// ============================================================

bool RPiCamera::startCamera()
{
    if (!m_Camera || m_CameraRunning)
        return false;

    m_Camera->requestCompleted.connect(this, &RPiCamera::requestComplete);

    if (m_Camera->start())
    {
        LOG_ERROR("Failed to start the camera.");
        m_Camera->requestCompleted.disconnect(this);
        return false;
    }

    m_CameraRunning = true;
    return true;
}

void RPiCamera::stopCamera()
{
    if (!m_Camera || !m_CameraRunning)
        return;

    m_Camera->stop();
    m_Camera->requestCompleted.disconnect(this);
    m_CameraRunning = false;
}

/**
 * @brief Called by libcamera from the CameraManager thread when a
 *        capture request completes.  Thread-safe access required.
 */
void RPiCamera::requestComplete(lc::Request *request)
{
    if (request->status() == lc::Request::RequestCancelled)
        return;

    if (m_IsStreaming)
    {
        // ---- Streaming: feed frame to INDI StreamManager ----
        const auto &buffers = request->buffers();
        if (!buffers.empty())
        {
            const lc::FrameBuffer *fb = buffers.begin()->second;
            auto it = m_MappedBuffers.find(fb);
            if (it != m_MappedBuffers.end() && !it->second.empty())
            {
                const uint8_t *data =
                    static_cast<const uint8_t *>(it->second[0].memory);
                size_t len = it->second[0].length;
                Streamer->newFrame(data, len);
            }
        }

        // Re-queue for continuous capture
        request->reuse(lc::Request::ReuseBuffers);

        // Update exposure if it changed
        double streamExp = Streamer->getExposure();
        int32_t expUs = static_cast<int32_t>(streamExp * 1e6);
        if (expUs < 100) expUs = 100;
        request->controls().set(lc::controls::ExposureTime, expUs);

        m_Camera->queueRequest(request);
    }
    else
    {
        // ---- Still capture: signal that frame is ready ----
        std::lock_guard<std::mutex> lock(m_CompletedMutex);
        m_CompletedRequest = request;
        m_FrameReady = true;
    }
}

// ============================================================
//  Buffer Management
// ============================================================

void RPiCamera::mapBuffers(lc::Stream *stream)
{
    for (const auto &buffer : m_Allocator->buffers(stream))
    {
        std::vector<MappedPlane> planes;
        for (const auto &plane : buffer->planes())
        {
            void *mem = mmap(nullptr, plane.length,
                             PROT_READ | PROT_WRITE, MAP_SHARED,
                             plane.fd.get(), plane.offset);
            if (mem == MAP_FAILED)
            {
                LOGF_ERROR("mmap failed for buffer plane (fd=%d, len=%u)",
                           plane.fd.get(), plane.length);
                continue;
            }
            planes.push_back({mem, plane.length});
        }
        m_MappedBuffers[buffer.get()] = std::move(planes);
    }
}

void RPiCamera::unmapBuffers()
{
    for (auto &[fb, planes] : m_MappedBuffers)
    {
        for (auto &p : planes)
        {
            if (p.memory && p.memory != MAP_FAILED)
                munmap(p.memory, p.length);
        }
    }
    m_MappedBuffers.clear();
}

void RPiCamera::cleanupRequests()
{
    m_Requests.clear();
}

// ============================================================
//  Property Handlers — ISNewNumber
// ============================================================

bool RPiCamera::ISNewNumber(const char *dev, const char *name,
                            double values[], char *names[], int n)
{
    if (dev && std::string(dev) != getDeviceName())
        return INDI::CCD::ISNewNumber(dev, name, values, names, n);

    // ---- Gain ----
    if (GainNP.isNameMatch(name))
    {
        GainNP.update(values, names, n);
        GainNP.setState(IPS_OK);
        GainNP.apply();
        LOGF_INFO("Gain set to %.1f", GainNP[0].getValue());
        return true;
    }

    // ---- Colour Gains ----
    if (ColourGainsNP.isNameMatch(name))
    {
        ColourGainsNP.update(values, names, n);
        ColourGainsNP.setState(IPS_OK);
        ColourGainsNP.apply();
        return true;
    }

    // ---- Brightness ----
    if (BrightnessNP.isNameMatch(name))
    {
        BrightnessNP.update(values, names, n);
        BrightnessNP.setState(IPS_OK);
        BrightnessNP.apply();
        return true;
    }

    // ---- Contrast ----
    if (ContrastNP.isNameMatch(name))
    {
        ContrastNP.update(values, names, n);
        ContrastNP.setState(IPS_OK);
        ContrastNP.apply();
        return true;
    }

    // ---- Saturation ----
    if (SaturationNP.isNameMatch(name))
    {
        SaturationNP.update(values, names, n);
        SaturationNP.setState(IPS_OK);
        SaturationNP.apply();
        return true;
    }

    // ---- Sharpness ----
    if (SharpnessNP.isNameMatch(name))
    {
        SharpnessNP.update(values, names, n);
        SharpnessNP.setState(IPS_OK);
        SharpnessNP.apply();
        return true;
    }

    return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

// ============================================================
//  Property Handlers — ISNewSwitch
// ============================================================

bool RPiCamera::ISNewSwitch(const char *dev, const char *name,
                            ISState *states, char *names[], int n)
{
    if (dev && std::string(dev) != getDeviceName())
        return INDI::CCD::ISNewSwitch(dev, name, states, names, n);

    // ---- Auto Exposure ----
    if (AutoExposureSP.isNameMatch(name))
    {
        AutoExposureSP.update(states, names, n);
        AutoExposureSP.setState(IPS_OK);
        AutoExposureSP.apply();
        return true;
    }

    // ---- Auto White Balance ----
    if (AutoWhiteBalanceSP.isNameMatch(name))
    {
        AutoWhiteBalanceSP.update(states, names, n);
        AutoWhiteBalanceSP.setState(IPS_OK);
        AutoWhiteBalanceSP.apply();
        return true;
    }

    // ---- AWB Mode ----
    if (AwbModeSP.isNameMatch(name))
    {
        AwbModeSP.update(states, names, n);
        AwbModeSP.setState(IPS_OK);
        AwbModeSP.apply();
        return true;
    }

    // ---- Noise Reduction ----
    if (NoiseReductionSP.isNameMatch(name))
    {
        NoiseReductionSP.update(states, names, n);
        NoiseReductionSP.setState(IPS_OK);
        NoiseReductionSP.apply();
        return true;
    }

    // ---- Raw Format (sensor mode) ----
    if (RawFormatSP.isNameMatch(name))
    {
        RawFormatSP.update(states, names, n);
        int sel = RawFormatSP.findOnSwitchIndex();
        if (sel >= 0 && sel < m_NumSensorModes)
        {
            m_CurrentModeIndex = sel;
            auto &mode = m_SensorModes[sel];

            // Update CCD params for the new mode
            float pixX = m_PixelSizeUm > 0 ? m_PixelSizeUm * mode.hBin
                                            : PrimaryCCD.getPixelSizeX();
            float pixY = m_PixelSizeUm > 0 ? m_PixelSizeUm * mode.vBin
                                            : PrimaryCCD.getPixelSizeY();
            SetCCDParams(mode.size.width, mode.size.height,
                         mode.bitDepth, pixX, pixY);

            PrimaryCCD.setBin(mode.hBin, mode.vBin);

            std::string bayer = bayerPatternFromFormat(mode.format);
            if (!bayer.empty())
                IUSaveText(&BayerT[2], bayer.c_str());

            RawFormatSP.setState(IPS_OK);
            RawFormatSP.apply();
            LOGF_INFO("Sensor mode: %s", mode.label.c_str());
        }
        return true;
    }

    // ---- AF Mode ----
    if (m_HasAF && AfModeSP.isNameMatch(name))
    {
        AfModeSP.update(states, names, n);
        AfModeSP.setState(IPS_OK);
        AfModeSP.apply();
        return true;
    }

    // ---- AF Trigger ----
    if (m_HasAF && AfTriggerSP.isNameMatch(name))
    {
        AfTriggerSP.update(states, names, n);
        AfTriggerSP.setState(IPS_OK);
        AfTriggerSP.apply();
        // Reset triggers to off after processing
        AfTriggerSP.reset();
        AfTriggerSP.apply();
        return true;
    }

    return INDI::CCD::ISNewSwitch(dev, name, states, names, n);
}

// ============================================================
//  Camera Controls — apply to a libcamera ControlList
// ============================================================

void RPiCamera::applyCameraControls(lc::ControlList &ctrlList)
{
    // Auto Exposure
    if (m_HasAE)
    {
        bool aeOn = (AutoExposureSP.findOnSwitchIndex() == 0);
        ctrlList.set(lc::controls::AeEnable, aeOn);
    }

    // Auto White Balance
    if (m_HasAWB)
    {
        bool awbOn = (AutoWhiteBalanceSP.findOnSwitchIndex() == 0);
        ctrlList.set(lc::controls::AwbEnable, awbOn);

        if (awbOn)
        {
            int awbIdx = AwbModeSP.findOnSwitchIndex();
            ctrlList.set(lc::controls::AwbMode,
                         static_cast<int32_t>(awbIdx));
        }
        else
        {
            // Manual colour gains
            float rGain = static_cast<float>(ColourGainsNP[0].getValue());
            float bGain = static_cast<float>(ColourGainsNP[1].getValue());
            ctrlList.set(lc::controls::ColourGains,
                         libcamera::Span<const float, 2>({rGain, bGain}));
        }
    }

    // ISP tuning
    ctrlList.set(lc::controls::Brightness,
                 static_cast<float>(BrightnessNP[0].getValue()));
    ctrlList.set(lc::controls::Contrast,
                 static_cast<float>(ContrastNP[0].getValue()));
    ctrlList.set(lc::controls::Saturation,
                 static_cast<float>(SaturationNP[0].getValue()));
    ctrlList.set(lc::controls::Sharpness,
                 static_cast<float>(SharpnessNP[0].getValue()));

    // Noise reduction
    int nrIdx = NoiseReductionSP.findOnSwitchIndex();
    ctrlList.set(lc::controls::draft::NoiseReductionMode,
                 static_cast<int32_t>(nrIdx));

    // Autofocus
    if (m_HasAF)
    {
        int afIdx = AfModeSP.findOnSwitchIndex();
        ctrlList.set(lc::controls::AfMode, static_cast<int32_t>(afIdx));

        int trigIdx = AfTriggerSP.findOnSwitchIndex();
        if (trigIdx == AF_TRIGGER_START)
            ctrlList.set(lc::controls::AfTrigger, 0);
        else if (trigIdx == AF_TRIGGER_CANCEL)
            ctrlList.set(lc::controls::AfTrigger, 1);
    }
}

// ============================================================
//  FITS Keywords
// ============================================================

void RPiCamera::addFITSKeywords(INDI::CCDChip *targetChip,
                                std::vector<INDI::FITSRecord> &fitsKeywords)
{
    // Let the base class add standard keywords: RA, DEC, SITELAT, SITELONG,
    // AIRMASS, PIERSIDE, FOCALLEN, APTDIA, FILTER, BAYERPAT, etc.
    INDI::CCD::addFITSKeywords(targetChip, fitsKeywords);

    // ---- Camera-specific keywords ----

    // Analog gain
    fitsKeywords.push_back({"GAIN", GainNP[0].getValue(), 3, "Analog Gain"});

    // Sensor model
    if (!m_SensorModel.empty())
        fitsKeywords.push_back({"SENSOR", m_SensorModel, "Camera Sensor Model"});

    // Sensor-specific friendly name
    if (!m_SensorAdj.friendlyName.empty())
        fitsKeywords.push_back({"CAMNAME", m_SensorAdj.friendlyName,
                                "Camera Module"});

    // Actual bit depth of the raw data (before promotion to 16-bit)
    if (m_ActiveIsRaw && m_CurrentModeIndex < m_NumSensorModes)
    {
        unsigned bd = m_SensorModes[m_CurrentModeIndex].bitDepth;
        fitsKeywords.push_back({"RAWBPP",
                                static_cast<int>(bd), "Native Sensor Bit Depth"});
    }
}

// ============================================================
//  Configuration Persistence
// ============================================================

bool RPiCamera::saveConfigItems(FILE *fp)
{
    INDI::CCD::saveConfigItems(fp);

    GainNP.save(fp);

    if (m_HasAE)
        AutoExposureSP.save(fp);
    if (m_HasAWB)
    {
        AutoWhiteBalanceSP.save(fp);
        AwbModeSP.save(fp);
        ColourGainsNP.save(fp);
    }

    BrightnessNP.save(fp);
    ContrastNP.save(fp);
    SaturationNP.save(fp);
    SharpnessNP.save(fp);
    NoiseReductionSP.save(fp);

    if (m_NumSensorModes > 0)
        RawFormatSP.save(fp);

    if (m_HasAF)
        AfModeSP.save(fp);

    return true;
}

// ============================================================
//  Utility — Bayer pattern from libcamera PixelFormat
// ============================================================

std::string RPiCamera::bayerPatternFromFormat(
    const lc::PixelFormat &fmt) const
{
    std::string s = fmt.toString();

    // libcamera format strings: "SBGGR10", "SGBRG12_CSI2P", etc.
    // The Bayer pattern is encoded in the first 5 characters after 'S'.
    if (s.find("BGGR") != std::string::npos) return "BGGR";
    if (s.find("GBRG") != std::string::npos) return "GBRG";
    if (s.find("GRBG") != std::string::npos) return "GRBG";
    if (s.find("RGGB") != std::string::npos) return "RGGB";

    return "";  // not a Bayer format
}

// ============================================================
//  Utility — Bit depth from libcamera PixelFormat
// ============================================================

unsigned int RPiCamera::bitDepthFromFormat(const lc::PixelFormat &fmt) const
{
    std::string s = fmt.toString();

    // Look for bit depth digits at the end of the base name
    // e.g. "SBGGR10_CSI2P" → 10,  "SBGGR12" → 12,  "SBGGR16" → 16
    // Remove any suffix first
    auto pos = s.find('_');
    std::string base = (pos != std::string::npos) ? s.substr(0, pos) : s;

    // Extract trailing digits
    std::string digits;
    for (auto it = base.rbegin(); it != base.rend() && std::isdigit(*it); ++it)
        digits.insert(digits.begin(), *it);

    if (!digits.empty())
        return static_cast<unsigned>(std::stoi(digits));

    return 8; // fallback
}

// ============================================================
//  Utility — Detect packed CSI-2 format
// ============================================================

bool RPiCamera::isPackedCSI2(const lc::PixelFormat &fmt) const
{
    std::string s = fmt.toString();
    return s.find("CSI2P") != std::string::npos;
}

// ============================================================
//  Utility — Unpack 10-bit MIPI CSI-2 packed → 16-bit
//
//  4 pixels in 5 bytes:
//    byte0: P0[9:2]    byte1: P1[9:2]
//    byte2: P2[9:2]    byte3: P3[9:2]
//    byte4: P0[1:0] | P1[1:0]<<2 | P2[1:0]<<4 | P3[1:0]<<6
// ============================================================

void RPiCamera::unpack10bitCSI2(const uint8_t *src, uint16_t *dst,
                                size_t numPixels)
{
    size_t groups = numPixels / 4;
    for (size_t g = 0; g < groups; g++)
    {
        uint8_t lsb = src[4];
        dst[0] = (static_cast<uint16_t>(src[0]) << 2) | ((lsb >> 0) & 0x03);
        dst[1] = (static_cast<uint16_t>(src[1]) << 2) | ((lsb >> 2) & 0x03);
        dst[2] = (static_cast<uint16_t>(src[2]) << 2) | ((lsb >> 4) & 0x03);
        dst[3] = (static_cast<uint16_t>(src[3]) << 2) | ((lsb >> 6) & 0x03);
        src += 5;
        dst += 4;
    }
}

// ============================================================
//  Utility — Unpack 12-bit MIPI CSI-2 packed → 16-bit
//
//  2 pixels in 3 bytes:
//    byte0: P0[11:4]
//    byte1: P0[3:0] | P1[3:0]<<4
//    byte2: P1[11:4]
// ============================================================

void RPiCamera::unpack12bitCSI2(const uint8_t *src, uint16_t *dst,
                                size_t numPixels)
{
    size_t groups = numPixels / 2;
    for (size_t g = 0; g < groups; g++)
    {
        dst[0] = (static_cast<uint16_t>(src[0]) << 4) | (src[1] & 0x0F);
        dst[1] = (static_cast<uint16_t>(src[2]) << 4) | ((src[1] >> 4) & 0x0F);
        src += 3;
        dst += 2;
    }
}
