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
#include <chrono>
#include <ctime>
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
    addCaptureFormat({"INDI_RAW_MONO", "RAW Mono", 16, false});
    addCaptureFormat({"INDI_MONO", "Mono", 8, false});

    // --- Raw Left-Shift (normalize to 16-bit) ---
    RawLeftShiftSP[0].fill("LEFTSHIFT_ON",  "On",  ISS_ON);
    RawLeftShiftSP[1].fill("LEFTSHIFT_OFF", "Off", ISS_OFF);
    RawLeftShiftSP.fill(getDeviceName(), "RAW_LEFT_SHIFT", "Raw Normalize",
                        IMAGE_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // --- Fast Exposure toggle ---
    FastExposureSP[0].fill("FAST_ON",  "On",  ISS_OFF);
    FastExposureSP[1].fill("FAST_OFF", "Off", ISS_ON);
    FastExposureSP.fill(getDeviceName(), "CCD_FAST_TOGGLE", "Fast Exposure",
                        MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    FastCountNP[0].fill("FAST_COUNT", "Frame Count (0=unlimited)", "%.0f",
                        0, 100000, 1, 0);
    FastCountNP.fill(getDeviceName(), "CCD_FAST_COUNT", "Fast Count",
                     MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // --- CCD_PROCFRAME (ISP output size for RGB/Mono) ---
    ProcFrameNP[0].fill("PROC_WIDTH",  "Width",  "%.0f", 64, 16384, 1, 0);
    ProcFrameNP[1].fill("PROC_HEIGHT", "Height", "%.0f", 64, 16384, 1, 0);
    ProcFrameNP.fill(getDeviceName(), "CCD_PROCFRAME", "ISP Output Size",
                     IMAGE_SETTINGS_TAB, IP_RW, 60, IPS_IDLE);

    // --- Streaming Resolution Preset ---
    StreamResSP[STREAM_720P  ].fill("STREAM_720P",   "720p",   ISS_ON);
    StreamResSP[STREAM_1080P ].fill("STREAM_1080P",  "1080p",  ISS_OFF);
    StreamResSP[STREAM_4K    ].fill("STREAM_4K",     "4K",     ISS_OFF);
    StreamResSP[STREAM_CUSTOM].fill("STREAM_CUSTOM", "Custom", ISS_OFF);
    StreamResSP.fill(getDeviceName(), "STREAM_RESOLUTION", "Stream Resolution",
                     "Streaming", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    StreamCustomResNP[0].fill("STREAM_WIDTH",  "Width",  "%.0f", 160, 8192, 1, 1280);
    StreamCustomResNP[1].fill("STREAM_HEIGHT", "Height", "%.0f", 120, 8192, 1, 720);
    StreamCustomResNP.fill(getDeviceName(), "STREAM_CUSTOM_RES", "Custom Resolution",
                           "Streaming", IP_RW, 60, IPS_IDLE);

    // --- Streaming Target FPS ---
    StreamFpsNP[0].fill("TARGET_FPS", "Target FPS", "%.1f", 1.0, 120.0, 1.0, 30.0);
    StreamFpsNP.fill(getDeviceName(), "STREAM_FPS", "Target FPS",
                     "Streaming", IP_RW, 60, IPS_IDLE);

    // --- Streaming Estimated FPS (read-only) ---
    StreamEstFpsNP[0].fill("EST_FPS", "Est. FPS", "%.1f", 0, 999, 0, 0);
    StreamEstFpsNP.fill(getDeviceName(), "STREAM_EST_FPS", "Actual FPS",
                        "Streaming", IP_RO, 60, IPS_IDLE);

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
        defineProperty(RawLeftShiftSP);
        defineProperty(FastExposureSP);
        defineProperty(FastCountNP);
        defineProperty(ProcFrameNP);
        defineProperty(StreamResSP);
        defineProperty(StreamCustomResNP);
        defineProperty(StreamFpsNP);
        defineProperty(StreamEstFpsNP);

        // Camera-control properties were created in Connect() →
        // createCameraControlProperties().  Define them now.
        if (m_HasAE)
        {
            defineProperty(AutoExposureSP);
            if (m_HasAeConstraintMode)
                defineProperty(AeConstraintModeSP);
            if (m_HasAeExposureMode)
                defineProperty(AeExposureModeSP);
            if (m_HasAeMeteringMode)
                defineProperty(AeMeteringModeSP);
        }
        if (m_HasExposureValue)
            defineProperty(ExposureValueNP);
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
            if (m_HasAfMetering)
                defineProperty(AfMeteringSP);
            if (m_HasAfPause)
                defineProperty(AfPauseSP);
            if (m_HasAfRange)
                defineProperty(AfRangeSP);
            if (m_HasAfSpeed)
                defineProperty(AfSpeedSP);
        }

        if (m_HasLensPosition)
            defineProperty(LensPositionNP);

        defineProperty(TemperatureNP);

        SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(GainNP);
        deleteProperty(RawLeftShiftSP);
        deleteProperty(FastExposureSP);
        deleteProperty(FastCountNP);
        deleteProperty(ProcFrameNP);
        deleteProperty(StreamResSP);
        deleteProperty(StreamCustomResNP);
        deleteProperty(StreamFpsNP);
        deleteProperty(StreamEstFpsNP);

        if (m_HasAE)
        {
            deleteProperty(AutoExposureSP);
            if (m_HasAeConstraintMode)
                deleteProperty(AeConstraintModeSP);
            if (m_HasAeExposureMode)
                deleteProperty(AeExposureModeSP);
            if (m_HasAeMeteringMode)
                deleteProperty(AeMeteringModeSP);
        }
        if (m_HasExposureValue)
            deleteProperty(ExposureValueNP);
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
            if (m_HasAfMetering)
                deleteProperty(AfMeteringSP);
            if (m_HasAfPause)
                deleteProperty(AfPauseSP);
            if (m_HasAfRange)
                deleteProperty(AfRangeSP);
            if (m_HasAfSpeed)
                deleteProperty(AfSpeedSP);
        }

        if (m_HasLensPosition)
            deleteProperty(LensPositionNP);

        deleteProperty(TemperatureNP);
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
            BayerTP[0].setText("0");              // X offset
            BayerTP[1].setText("0");              // Y offset
            BayerTP[2].setText(bayer.c_str());    // e.g. "RGGB"
        }
    }

    // ---- Create camera-control properties based on what the sensor supports ----
    createCameraControlProperties();

    // ---- Set default ProcFrame to sensor resolution ----
    if (!m_SensorModes.empty())
    {
        ProcFrameNP[0].setValue(m_SensorModes[0].size.width);
        ProcFrameNP[1].setValue(m_SensorModes[0].size.height);
    }

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
        std::string fmtName = pixFmt.toString();

        // Skip Pi 5 PISP compressed transport formats — we cannot
        // decompress these.  The unpacked variant should also be listed.
        if (fmtName.find("PISP") != std::string::npos ||
            fmtName.find("_COMP") != std::string::npos)
        {
            LOGF_DEBUG("Skipping unsupported format: %s", fmtName.c_str());
            continue;
        }

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
        RawFormatSP.resize(m_NumSensorModes);
    }

    LOGF_INFO("Enumerated %d raw sensor mode(s).", m_NumSensorModes);

    // ---- Pi 5 PiSP detection ----
    // On Pi 5 (PiSP backend), the Raw stream delivers PISP-compressed data
    // which we cannot use.  Detect this by validating the first raw mode and
    // checking if the format gets adjusted to a PISP/COMP format.
    if (m_NumSensorModes > 0)
    {
        auto testCfg = m_Camera->generateConfiguration({lc::StreamRole::Raw});
        if (testCfg && !testCfg->empty())
        {
            testCfg->at(0).pixelFormat = m_SensorModes[0].format;
            testCfg->at(0).size        = m_SensorModes[0].size;
            testCfg->validate();
            std::string valFmt = testCfg->at(0).pixelFormat.toString();
            if (valFmt.find("PISP") != std::string::npos ||
                valFmt.find("_COMP") != std::string::npos)
            {
                m_IsPiSP = true;
                // Record the native sensor bit depth before we overwrite modes
                m_NativeBitDepth = m_SensorModes[0].bitDepth;
                LOG_INFO("Pi 5 PiSP backend detected — raw Bayer will be "
                         "routed through the ISP (StillCapture + Bayer16).");
            }
        }
    }
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

    // --- Lens Position (manual focus — optional) ---
    m_HasLensPosition = ctrlMap.find(&lc::controls::LensPosition) != ctrlMap.end();
    if (m_HasLensPosition)
    {
        auto it = ctrlMap.find(&lc::controls::LensPosition);
        float minL = it->second.min().get<float>();
        float maxL = it->second.max().get<float>();
        float defL = it->second.def().get<float>();
        LensPositionNP[0].fill("LENS_POSITION", "Position (dioptres)", "%.2f",
                               minL, maxL, 0.01, defL);
        LensPositionNP.fill(getDeviceName(), "FOCUS_POSITION", "Focus",
                            "Camera Controls", IP_RW, 60, IPS_IDLE);
        LOGF_INFO("Lens focus available: %.2f to %.2f dioptres (0 = infinity)",
                  minL, maxL);
    }

    // --- AE Constraint Mode (optional) ---
    m_HasAeConstraintMode = ctrlMap.find(&lc::controls::AeConstraintMode) != ctrlMap.end();
    if (m_HasAeConstraintMode)
    {
        AeConstraintModeSP[AE_CONSTRAINT_NORMAL   ].fill("AEC_NORMAL",    "Normal",    ISS_ON);
        AeConstraintModeSP[AE_CONSTRAINT_HIGHLIGHT].fill("AEC_HIGHLIGHT", "Highlight", ISS_OFF);
        AeConstraintModeSP[AE_CONSTRAINT_SHADOWS  ].fill("AEC_SHADOWS",   "Shadows",   ISS_OFF);
        AeConstraintModeSP[AE_CONSTRAINT_CUSTOM   ].fill("AEC_CUSTOM",    "Custom",    ISS_OFF);
        AeConstraintModeSP.fill(getDeviceName(), "AE_CONSTRAINT_MODE", "AE Constraint",
                                "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    }

    // --- AE Exposure Mode (optional) ---
    m_HasAeExposureMode = ctrlMap.find(&lc::controls::AeExposureMode) != ctrlMap.end();
    if (m_HasAeExposureMode)
    {
        AeExposureModeSP[AE_EXPOSURE_NORMAL].fill("AEE_NORMAL", "Normal", ISS_ON);
        AeExposureModeSP[AE_EXPOSURE_SHORT ].fill("AEE_SHORT",  "Short",  ISS_OFF);
        AeExposureModeSP[AE_EXPOSURE_LONG  ].fill("AEE_LONG",   "Long",   ISS_OFF);
        AeExposureModeSP[AE_EXPOSURE_CUSTOM].fill("AEE_CUSTOM", "Custom", ISS_OFF);
        AeExposureModeSP.fill(getDeviceName(), "AE_EXPOSURE_MODE", "AE Exposure Mode",
                              "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    }

    // --- AE Metering Mode (optional) ---
    m_HasAeMeteringMode = ctrlMap.find(&lc::controls::AeMeteringMode) != ctrlMap.end();
    if (m_HasAeMeteringMode)
    {
        AeMeteringModeSP[AE_METERING_CENTRE].fill("AEM_CENTRE", "Centre Weighted", ISS_ON);
        AeMeteringModeSP[AE_METERING_SPOT  ].fill("AEM_SPOT",   "Spot",            ISS_OFF);
        AeMeteringModeSP[AE_METERING_MATRIX].fill("AEM_MATRIX", "Matrix",          ISS_OFF);
        AeMeteringModeSP[AE_METERING_CUSTOM].fill("AEM_CUSTOM", "Custom",          ISS_OFF);
        AeMeteringModeSP.fill(getDeviceName(), "AE_METERING_MODE", "AE Metering",
                              "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    }

    // --- Exposure Value (EV compensation, optional) ---
    m_HasExposureValue = ctrlMap.find(&lc::controls::ExposureValue) != ctrlMap.end();
    if (m_HasExposureValue)
    {
        ExposureValueNP[0].fill("EV", "EV", "%.1f", -8.0, 8.0, 0.5, 0.0);
        ExposureValueNP.fill(getDeviceName(), "EXPOSURE_VALUE", "Exposure Value",
                             "Camera Controls", IP_RW, 60, IPS_IDLE);
    }

    // --- Extended AF controls (optional) ---
    if (m_HasAF)
    {
        m_HasAfMetering = ctrlMap.find(&lc::controls::AfMetering) != ctrlMap.end();
        if (m_HasAfMetering)
        {
            AfMeteringSP[AF_METERING_AUTO   ].fill("AFM_AUTO",    "Auto",    ISS_ON);
            AfMeteringSP[AF_METERING_WINDOWS].fill("AFM_WINDOWS", "Windows", ISS_OFF);
            AfMeteringSP.fill(getDeviceName(), "AF_METERING", "AF Metering",
                              "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
        }

        m_HasAfPause = ctrlMap.find(&lc::controls::AfPause) != ctrlMap.end();
        if (m_HasAfPause)
        {
            AfPauseSP[AF_PAUSE_DEFERRED ].fill("AFP_DEFERRED",  "Deferred",  ISS_OFF);
            AfPauseSP[AF_PAUSE_IMMEDIATE].fill("AFP_IMMEDIATE", "Immediate", ISS_OFF);
            AfPauseSP[AF_PAUSE_RESUME   ].fill("AFP_RESUME",    "Resume",    ISS_OFF);
            AfPauseSP.fill(getDeviceName(), "AF_PAUSE", "AF Pause",
                           "Camera Controls", IP_RW, ISR_ATMOST1, 60, IPS_IDLE);
        }

        m_HasAfRange = ctrlMap.find(&lc::controls::AfRange) != ctrlMap.end();
        if (m_HasAfRange)
        {
            AfRangeSP[AF_RANGE_NORMAL].fill("AFR_NORMAL", "Normal", ISS_ON);
            AfRangeSP[AF_RANGE_MACRO ].fill("AFR_MACRO",  "Macro",  ISS_OFF);
            AfRangeSP[AF_RANGE_FULL  ].fill("AFR_FULL",   "Full",   ISS_OFF);
            AfRangeSP.fill(getDeviceName(), "AF_RANGE", "AF Range",
                           "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
        }

        m_HasAfSpeed = ctrlMap.find(&lc::controls::AfSpeed) != ctrlMap.end();
        if (m_HasAfSpeed)
        {
            AfSpeedSP[AF_SPEED_NORMAL].fill("AFS_NORMAL", "Normal", ISS_ON);
            AfSpeedSP[AF_SPEED_FAST  ].fill("AFS_FAST",   "Fast",   ISS_OFF);
            AfSpeedSP.fill(getDeviceName(), "AF_SPEED", "AF Speed",
                           "Camera Controls", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
        }
    }

    // --- Sensor Temperature (read-only) ---
    TemperatureNP[0].fill("CCD_TEMPERATURE_VALUE", "Temperature (\u00b0C)", "%.1f",
                          -40.0, 85.0, 0, 0);
    TemperatureNP.fill(getDeviceName(), "CCD_TEMPERATURE", "Temperature",
                       MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // --- Exposure Time limits from hardware ---
    {
        auto it = ctrlMap.find(&lc::controls::ExposureTime);
        if (it != ctrlMap.end())
        {
            m_ExposureMinS = it->second.min().get<int32_t>() / 1e6f;
            m_ExposureMaxS = it->second.max().get<int32_t>() / 1e6f;
            LOGF_INFO("Exposure range: %.6f s to %.1f s", m_ExposureMinS, m_ExposureMaxS);
        }
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
    std::string capFmt = "INDI_RAW";
    for (size_t i = 0; i < m_CaptureFormats.size(); i++)
    {
        if (CaptureFormatSP[i].getState() == ISS_ON)
        {
            capFmt = m_CaptureFormats[i].name;
            break;
        }
    }
    m_ActiveCaptureFmt = capFmt;
    m_ActiveIsRaw = (capFmt == "INDI_RAW" || capFmt == "INDI_RAW_MONO");

    // Fast exposure mode check
    m_FastMode = (FastExposureSP.findOnSwitchIndex() == 0);
    if (m_FastMode)
    {
        int count = static_cast<int>(FastCountNP[0].getValue());
        m_FastFramesRemaining = (count <= 0) ? -1 : count; // -1 = unlimited
    }
    else
    {
        m_FastFramesRemaining = 0;
    }

    // Stop any previous camera session
    if (m_CameraRunning)
        stopCamera();
    unmapBuffers();
    cleanupRequests();
    m_Allocator.reset();
    m_Config.reset();

    // ---- Configure and start ----
    bool ok = configureForStill();
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

        // Handle BIAS frame type — force minimum exposure
        float effectiveDuration = duration;
        auto frameType = PrimaryCCD.getFrameType();
        if (frameType == INDI::CCDChip::BIAS_FRAME)
        {
            effectiveDuration = m_ExposureMinS;
            LOGF_INFO("BIAS frame: overriding exposure to %.6f s", effectiveDuration);
        }

        int64_t expUs = static_cast<int64_t>(effectiveDuration * 1e6);
        if (expUs < 1) expUs = 1;

        req->controls().set(lc::controls::ExposureTime, static_cast<int32_t>(expUs));
        req->controls().set(lc::controls::AnalogueGain,
                            static_cast<float>(GainNP[0].getValue()));

        // Set FrameDurationLimits to allow the full requested exposure.
        // Without this, the sensor's default frame rate may clip long exposures.
        {
            int64_t frameDurMin = expUs;
            int64_t frameDurMax = expUs + 1000 > 100000 ? expUs + 1000 : 100000;
            req->controls().set(lc::controls::FrameDurationLimits,
                                libcamera::Span<const int64_t, 2>({frameDurMin, frameDurMax}));
        }

        // Apply user camera controls (AWB, ISP tuning, NR, AF, etc.)
        applyCameraControls(req->controls());

        // Force AE OFF for still captures — manual ExposureTime must not be
        // overridden by the auto-exposure algorithm.  The AE preference only
        // affects streaming/preview.
        if (m_HasAE)
            req->controls().set(lc::controls::AeEnable, false);
    }

    // Record wall-clock time for DATE-OBS FITS keyword
    {
        auto now = std::chrono::system_clock::now();
        auto tt  = std::chrono::system_clock::to_time_t(now);
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch()) % 1000;
        struct tm utc;
        gmtime_r(&tt, &utc);
        char buf[64];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03ld",
                 utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                 utc.tm_hour, utc.tm_min, utc.tm_sec,
                 static_cast<long>(ms.count()));
        m_ExposureDateObs = buf;
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

    m_FastFramesRemaining = 0;
    m_FastMode = false;
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
    (void)srcLen; // used only in debug logging

    int subX = PrimaryCCD.getSubX();
    int subY = PrimaryCCD.getSubY();
    int subW = PrimaryCCD.getSubW();
    int subH = PrimaryCCD.getSubH();

    // Garbage column removal: reduce effective width
    int garbageCols = garbageColumnsForCurrentMode();
    if (garbageCols > 0 && m_ActiveIsRaw)
    {
        int maxW = static_cast<int>(m_ActiveSize.width) - garbageCols;
        if (subX + subW > maxW)
        {
            subW = maxW - subX;
            if (subW < 1) subW = 1;
            LOGF_DEBUG("Garbage column removal: cropping to width=%d (removed %d cols)",
                       subW, garbageCols);
        }
    }

    bool isRawMono = (m_ActiveCaptureFmt == "INDI_RAW_MONO");
    bool isMono    = (m_ActiveCaptureFmt == "INDI_MONO");

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
        unsigned int stride = m_Config->at(0).stride;

        LOGF_DEBUG("downloadImage RAW: %dx%d  stride=%u  fmt=%s",
                   fullW, m_ActiveSize.height, stride,
                   m_ActivePixelFormat.toString().c_str());

        if (isPackedCSI2(m_ActivePixelFormat))
        {
            // CSI-2 packed: unpack row by row using stride for row offsets
            unsigned int bd = bitDepthFromFormat(m_ActivePixelFormat);
            size_t rowPixels = static_cast<size_t>(fullW);
            std::vector<uint16_t> unpackedRow(rowPixels);

            for (int row = 0; row < subH; row++)
            {
                const uint8_t *srcRow = srcData + (subY + row) * stride;
                if (bd == 10)
                    unpack10bitCSI2(srcRow, unpackedRow.data(), rowPixels);
                else if (bd == 12)
                    unpack12bitCSI2(srcRow, unpackedRow.data(), rowPixels);
                else
                    std::memcpy(unpackedRow.data(), srcRow,
                                std::min(rowPixels * 2, static_cast<size_t>(stride)));

                uint16_t *dstRow = dstBuf + row * subW;
                std::memcpy(dstRow, unpackedRow.data() + subX,
                            subW * sizeof(uint16_t));
            }
        }
        else
        {
            // Unpacked 16-bit raw, or Pi 5 PISP decompressed (data is in
            // 16-bit per pixel format in the DMA buffer). Use stride for
            // correct row-to-row offsets.
            for (int row = 0; row < subH; row++)
            {
                const uint8_t *srcRow = srcData + (subY + row) * stride;
                const uint16_t *srcPixels =
                    reinterpret_cast<const uint16_t *>(srcRow);
                uint16_t *dstRow = dstBuf + row * subW;
                std::memcpy(dstRow, srcPixels + subX,
                            subW * sizeof(uint16_t));
            }
        }

        // Normalize raw data to fill (or restore) the 16-bit range.
        //
        // PiSP (Pi 5):  The ISP already left-shifts sensor data to fill
        //   16 bits (e.g. 10-bit << 6).  An additional shift would cause
        //   overflow corruption, so we skip it.  When the user disables
        //   normalization, we right-shift to recover native-depth values.
        //
        // Non-PiSP:  The unpacked data is in its native bit depth (e.g.
        //   0–1023 for 10-bit).  Left-shifting promotes it to 16-bit.
        bool doLeftShift = (RawLeftShiftSP.findOnSwitchIndex() == 0);
        unsigned int bd = m_IsPiSP && m_NativeBitDepth > 0
                        ? m_NativeBitDepth
                        : m_SensorModes[m_CurrentModeIndex].bitDepth;
        size_t numPixels = static_cast<size_t>(subW) * subH;

        if (m_IsPiSP)
        {
            if (doLeftShift)
            {
                // Data is already 16-bit from the ISP — nothing to do.
                LOGF_DEBUG("PiSP: data already normalized to 16-bit by ISP "
                           "(native %u-bit, <<  %u) — no additional shift",
                           bd, 16 - bd);
            }
            else if (bd < 16)
            {
                // User wants native bit-depth values: undo the ISP shift.
                unsigned int shift = 16 - bd;
                LOGF_DEBUG("PiSP: right-shifting by %u bits to restore "
                           "%u-bit native values", shift, bd);
                for (size_t i = 0; i < numPixels; i++)
                    dstBuf[i] >>= shift;
            }
        }
        else
        {
            if (doLeftShift && bd < 16)
                applyRawLeftShift(dstBuf, numPixels, bd);
        }

        // Log pixel statistics for diagnostics
        {
            uint16_t pMin = 65535, pMax = 0;
            uint64_t pSum = 0;
            for (size_t i = 0; i < numPixels; i++)
            {
                if (dstBuf[i] < pMin) pMin = dstBuf[i];
                if (dstBuf[i] > pMax) pMax = dstBuf[i];
                pSum += dstBuf[i];
            }
            LOGF_DEBUG("RAW pixels: min=%u  max=%u  mean=%.0f  "
                       "(%u-bit, PiSP=%s, normalize=%s)",
                       pMin, pMax,
                       static_cast<double>(pSum) / numPixels,
                       bd, m_IsPiSP ? "yes" : "no",
                       doLeftShift ? "on" : "off");
        }

        // RAW Mono: convert Bayer to mono by summing 2×2 superpixels
        if (isRawMono)
        {
            int monoW = subW / 2;
            int monoH = subH / 2;
            // Convert in-place into the front of the buffer
            convertRawToMono(dstBuf, dstBuf, subW, subH);

            // Update frame geometry for the mono output
            size_t monoBytes = static_cast<size_t>(monoW) * monoH * 2;
            PrimaryCCD.setFrameBufferSize(monoBytes);
            PrimaryCCD.setFrame(subX / 2, subY / 2, monoW, monoH);

            // Remove Bayer flag for mono output
            SetCCDCapability(GetCCDCapability() & ~CCD_HAS_BAYER);
            LOGF_DEBUG("RAW Mono: %dx%d → %dx%d mono", subW, subH, monoW, monoH);
        }
    }
    else
    {
        // RGB or Mono ISP capture — ISP-processed output
        // libcamera may output BGR888 (3 bpp) or XBGR8888 (4 bpp)
        int srcBpp = 3; // bytes per pixel in DMA buffer
        std::string fmtStr = m_ActivePixelFormat.toString();
        if (fmtStr.find("XB") != std::string::npos ||
            fmtStr.find("XR") != std::string::npos ||
            fmtStr.find("BX") != std::string::npos ||
            fmtStr.find("RX") != std::string::npos ||
            fmtStr.find("AB") != std::string::npos ||
            fmtStr.find("AR") != std::string::npos)
            srcBpp = 4;

        if (isMono)
        {
            // Mono capture: extract single luminance channel (green)
            // Saturation was set to 0 in applyCameraControls, so
            // R≈G≈B.  We just take the green channel for NAXIS=2.
            int bpp = 8;
            PrimaryCCD.setBPP(bpp);
            PrimaryCCD.setNAxis(2);

            size_t frameBytes = static_cast<size_t>(subW) * subH;
            PrimaryCCD.setFrameBufferSize(frameBytes);

            uint8_t *dstBuf = PrimaryCCD.getFrameBuffer();
            unsigned int stride = m_Config->at(0).stride;

            for (int row = 0; row < subH; row++)
            {
                const uint8_t *srcRow = srcData + (subY + row) * stride
                                        + subX * srcBpp;
                size_t dstOffset = static_cast<size_t>(row) * subW;

                for (int col = 0; col < subW; col++)
                {
                    // Green channel (index 1 in both BGR888 and XBGR8888)
                    dstBuf[dstOffset + col] = srcRow[col * srcBpp + 1];
                }
            }

            // Remove Bayer flag for mono output
            SetCCDCapability(GetCCDCapability() & ~CCD_HAS_BAYER);
        }
        else
        {
            // Full RGB output
            int bpp = 8;
            PrimaryCCD.setBPP(bpp);
            PrimaryCCD.setNAxis(3);

            // FITS 3D cube: NAXIS1=W, NAXIS2=H, NAXIS3=3
            // Data must be plane-sequential: all R, all G, all B.
            size_t planeSize = static_cast<size_t>(subW) * subH;
            size_t frameBytes = planeSize * 3;
            PrimaryCCD.setFrameBufferSize(frameBytes);

            uint8_t *dstBuf = PrimaryCCD.getFrameBuffer();
            uint8_t *rPlane = dstBuf;
            uint8_t *gPlane = dstBuf + planeSize;
            uint8_t *bPlane = dstBuf + planeSize * 2;

            // Use stride from the config for correct row offsets in the DMA buffer.
            unsigned int stride = m_Config->at(0).stride;

            LOGF_DEBUG("downloadImage RGB: %dx%d  srcBpp=%d  stride=%u  subX=%d subY=%d",
                       m_ActiveSize.width, m_ActiveSize.height, srcBpp, stride, subX, subY);

            for (int row = 0; row < subH; row++)
            {
                const uint8_t *srcRow = srcData + (subY + row) * stride
                                        + subX * srcBpp;
                size_t dstOffset = static_cast<size_t>(row) * subW;

                if (srcBpp == 3)
                {
                    // BGR888 → plane-sequential RGB
                    for (int col = 0; col < subW; col++)
                    {
                        bPlane[dstOffset + col] = srcRow[col * 3 + 0]; // B
                        gPlane[dstOffset + col] = srcRow[col * 3 + 1]; // G
                        rPlane[dstOffset + col] = srcRow[col * 3 + 2]; // R
                    }
                }
                else
                {
                    // XBGR8888 (4 bpp) → plane-sequential RGB
                    for (int col = 0; col < subW; col++)
                    {
                        bPlane[dstOffset + col] = srcRow[col * 4 + 0]; // B
                        gPlane[dstOffset + col] = srcRow[col * 4 + 1]; // G
                        rPlane[dstOffset + col] = srcRow[col * 4 + 2]; // R
                    }
                }
            }
        }
    }

    // Extract metadata (temperature, black levels, actual exposure, etc.)
    readRequestMetadata(m_CompletedRequest);

    // Record DATE-END wall-clock time
    {
        auto now = std::chrono::system_clock::now();
        auto tt  = std::chrono::system_clock::to_time_t(now);
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch()) % 1000;
        struct tm utc;
        gmtime_r(&tt, &utc);
        char buf[64];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03ld",
                 utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                 utc.tm_hour, utc.tm_min, utc.tm_sec,
                 static_cast<long>(ms.count()));
        m_ExposureDateEnd = buf;
    }

    // In fast mode, re-queue for next frame instead of stopping
    if (m_FastMode && m_FastFramesRemaining != 0)
    {
        // Save the request pointer before clearing it
        lc::Request *reqToRequeue = m_CompletedRequest;

        m_CompletedRequest = nullptr;
        m_FrameReady = false;

        LOGF_INFO("Fast exposure: frame delivered (%d remaining)",
                  m_FastFramesRemaining.load());

        // Restore Bayer capability for next raw frame if we cleared it
        if (isRawMono || isMono)
            SetCCDCapability(GetCCDCapability() | CCD_HAS_BAYER);

        ExposureComplete(&PrimaryCCD);

        // Decrement counter (-1 means unlimited)
        if (m_FastFramesRemaining > 0)
            m_FastFramesRemaining--;

        // Re-queue the request for the next frame
        if (reqToRequeue && m_CameraRunning)
        {
            reqToRequeue->reuse(lc::Request::ReuseBuffers);

            // Re-apply controls for the next frame
            float effectiveDuration = m_ExposureRequest;
            auto frameType = PrimaryCCD.getFrameType();
            if (frameType == INDI::CCDChip::BIAS_FRAME)
                effectiveDuration = m_ExposureMinS;

            int64_t expUs = static_cast<int64_t>(effectiveDuration * 1e6);
            if (expUs < 1) expUs = 1;

            reqToRequeue->controls().set(lc::controls::ExposureTime,
                                         static_cast<int32_t>(expUs));
            reqToRequeue->controls().set(lc::controls::AnalogueGain,
                                         static_cast<float>(GainNP[0].getValue()));
            {
                int64_t frameDurMin = expUs;
                int64_t frameDurMax = expUs + 1000 > 100000 ? expUs + 1000 : 100000;
                reqToRequeue->controls().set(lc::controls::FrameDurationLimits,
                                             lc::Span<const int64_t, 2>({frameDurMin, frameDurMax}));
            }
            applyCameraControls(reqToRequeue->controls());
            if (m_HasAE)
                reqToRequeue->controls().set(lc::controls::AeEnable, false);

            m_Camera->queueRequest(reqToRequeue);
        }

        // Set up for the next frame
        handleFastExposureFrame();
        return 0;
    }

    // Stop camera (single-shot capture)
    stopCamera();

    // Restore Bayer capability if we cleared it for mono formats
    if (isRawMono || isMono)
        SetCCDCapability(GetCCDCapability() | CCD_HAS_BAYER);

    m_CompletedRequest = nullptr;
    m_FrameReady = false;

    LOGF_INFO("Download complete: %dx%d  offset (%d,%d)",
              subW, subH, subX, subY);

    ExposureComplete(&PrimaryCCD);
    return 0;
}

// ============================================================
//  Frame Metadata Extraction
// ============================================================

void RPiCamera::readRequestMetadata(lc::Request *request)
{
    if (!request)
        return;

    const auto &metadata = request->metadata();

    // Sensor temperature
    auto temp = metadata.get(lc::controls::SensorTemperature);
    if (temp)
    {
        m_SensorTemperature = static_cast<double>(*temp);
        TemperatureNP[0].setValue(m_SensorTemperature);
        TemperatureNP.setState(IPS_OK);
        TemperatureNP.apply();
        LOGF_DEBUG("Sensor temperature: %.1f °C", m_SensorTemperature);
    }

    // Actual exposure time (may differ from requested due to sensor quantisation)
    auto actualExp = metadata.get(lc::controls::ExposureTime);
    if (actualExp)
    {
        m_LastActualExposureUs = *actualExp;
        LOGF_DEBUG("Actual exposure: %lld µs (requested: %.0f µs)",
                   static_cast<long long>(m_LastActualExposureUs),
                   m_ExposureRequest * 1e6);
    }

    // Digital gain applied by ISP
    auto dgain = metadata.get(lc::controls::DigitalGain);
    if (dgain)
    {
        m_LastDigitalGain = *dgain;
        LOGF_DEBUG("Digital gain: %.3f", m_LastDigitalGain);
    }

    // Sensor black levels (R, Gr, Gb, B)
    auto blackLevels = metadata.get(lc::controls::SensorBlackLevels);
    if (blackLevels)
    {
        auto &bl = *blackLevels;
        for (int i = 0; i < 4; i++)
            m_LastBlackLevels[i] = bl[i];
        LOGF_DEBUG("Black levels: %d %d %d %d",
                   m_LastBlackLevels[0], m_LastBlackLevels[1],
                   m_LastBlackLevels[2], m_LastBlackLevels[3]);
    }
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
    if (PrimaryCCD.getNAxis() == 3)
        frameBytes = static_cast<size_t>(w) * h * 3; // RGB planar

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
                BayerTP[2].setText(bayer.c_str());

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

    switch (fType)
    {
    case INDI::CCDChip::LIGHT_FRAME:
        LOG_INFO("Frame type: LIGHT");
        break;
    case INDI::CCDChip::DARK_FRAME:
        LOG_INFO("Frame type: DARK (use same exposure/gain as LIGHT)");
        break;
    case INDI::CCDChip::BIAS_FRAME:
        LOGF_INFO("Frame type: BIAS (exposure will be forced to %.0f µs)",
                  m_ExposureMinS * 1e6);
        break;
    case INDI::CCDChip::FLAT_FRAME:
        LOG_INFO("Frame type: FLAT");
        break;
    }

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

    // Determine streaming resolution from preset
    int sw = 1280, sh = 720;
    int preset = StreamResSP.findOnSwitchIndex();
    switch (preset)
    {
    case STREAM_720P:
        sw = 1280; sh = 720;
        break;
    case STREAM_1080P:
        sw = 1920; sh = 1080;
        break;
    case STREAM_4K:
        sw = 3840; sh = 2160;
        break;
    case STREAM_CUSTOM:
    default:
        sw = static_cast<int>(StreamCustomResNP[0].getValue());
        sh = static_cast<int>(StreamCustomResNP[1].getValue());
        break;
    }
    // Ensure even dimensions
    sw &= ~1;
    sh &= ~1;
    if (sw < 160) sw = 160;
    if (sh < 120) sh = 120;

    if (!configureForStreaming(sw, sh))
    {
        LOG_ERROR("Failed to configure camera for streaming.");
        return false;
    }

    // Use the actual negotiated size (libcamera may have adjusted it)
    auto &streamCfg = m_Config->at(0);
    sw = streamCfg.size.width;
    sh = streamCfg.size.height;

    // Tell StreamManager about the format
    Streamer->setPixelFormat(INDI_RGB, 8);
    Streamer->setSize(sw, sh);

    // INDI 2.1.x StreamManager reads source frame dimensions from PrimaryCCD
    // (getSubW/getSubH), NOT from Streamer->setSize().  We must temporarily
    // shrink PrimaryCCD's subframe to the streaming resolution so the internal
    // size check in newFrame() passes.  Full resolution is restored in
    // StopStreaming().
    PrimaryCCD.setFrame(0, 0, sw, sh);

    LOGF_DEBUG("Streaming config: %dx%d  stride=%u  fmt=%s",
               sw, sh, streamCfg.stride,
               streamCfg.pixelFormat.toString().c_str());

    if (!startCamera())
    {
        LOG_ERROR("Failed to start camera for streaming.");
        return false;
    }

    // Compute FrameDurationLimits from target FPS
    double targetFps = StreamFpsNP[0].getValue();
    if (targetFps < 1.0) targetFps = 1.0;
    int64_t frameDurUs = static_cast<int64_t>(1e6 / targetFps);

    // Set default streaming controls
    for (auto &req : m_Requests)
    {
        double streamExp = Streamer->getTargetExposure();  // seconds
        int32_t expUs = static_cast<int32_t>(streamExp * 1e6);
        if (expUs < 100) expUs = 100;  // minimum

        req->controls().set(lc::controls::ExposureTime, expUs);
        req->controls().set(lc::controls::AnalogueGain,
                            static_cast<float>(GainNP[0].getValue()));

        // Set frame duration to achieve target FPS
        req->controls().set(lc::controls::FrameDurationLimits,
                            lc::Span<const int64_t, 2>({frameDurUs, frameDurUs}));

        applyCameraControls(req->controls());
    }

    // Streaming sends fully-debayered RGB — remove the Bayer property
    // so INDI clients (KStars/Ekos) don't try to debayer it again.
    // Just clearing the capability bit isn't enough because clients
    // cache the property at connect time.  We must actively delete it.
    SetCCDCapability(GetCCDCapability() & ~CCD_HAS_BAYER);
    deleteProperty(BayerTP);

    // Set streaming flag BEFORE queueing so requestComplete sees it
    m_IsStreaming = true;
    m_StreamFrameCount = 0;
    m_StreamFpsStart = std::chrono::steady_clock::now();
    m_StreamFpsFrames = 0;
    m_StreamEstFps = 0;

    // Reset FPS display
    StreamEstFpsNP[0].setValue(0);
    StreamEstFpsNP.setState(IPS_BUSY);
    StreamEstFpsNP.apply();

    // Queue all requests for continuous capture pipelining
    for (auto &req : m_Requests)
        m_Camera->queueRequest(req.get());

    LOGF_INFO("Streaming started at %dx%d, target %.0f FPS.", sw, sh, targetFps);
    return true;
}

bool RPiCamera::StopStreaming()
{
    m_IsStreaming = false;

    if (m_CameraRunning)
        stopCamera();

    // Restore Bayer capability + property so RAW still captures report
    // the correct Bayer pattern.
    SetCCDCapability(GetCCDCapability() | CCD_HAS_BAYER);
    defineProperty(BayerTP);

    // Restore PrimaryCCD to the full sensor resolution (it was temporarily
    // set to the streaming resolution in StartStreaming).
    PrimaryCCD.setFrame(0, 0, PrimaryCCD.getXRes(), PrimaryCCD.getYRes());

    // Reset FPS display
    StreamEstFpsNP[0].setValue(0);
    StreamEstFpsNP.setState(IPS_IDLE);
    StreamEstFpsNP.apply();

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

    std::string capFmt = "INDI_RAW";
    for (size_t i = 0; i < m_CaptureFormats.size(); i++)
    {
        if (CaptureFormatSP[i].getState() == ISS_ON)
        {
            capFmt = m_CaptureFormats[i].name;
            break;
        }
    }

    if (capFmt == "INDI_RAW" || capFmt == "INDI_RAW_MONO")
    {
        // Raw Bayer capture from the sensor
        auto &mode = m_SensorModes[m_CurrentModeIndex];

        if (m_IsPiSP)
        {
            // Pi 5 (PiSP): the Raw stream delivers PISP-compressed data
            // that we cannot decode.  Route through the ISP instead by
            // using StillCapture with an unpacked 16-bit Bayer format.
            // The ISP decompresses the data and outputs clean Bayer pixels.
            m_Config = m_Camera->generateConfiguration(
                {lc::StreamRole::StillCapture});
            if (!m_Config || m_Config->empty())
                return false;

            lc::PixelFormat bayer16 = matchingBayer16Format(mode.format);
            m_Config->at(0).pixelFormat = bayer16;
            m_Config->at(0).size = mode.size;
            m_ActivePixelFormat = bayer16;
            m_ActiveSize = mode.size;
            m_ActiveIsRaw = true;

            PrimaryCCD.setBPP(16);
            PrimaryCCD.setNAxis(2);

            LOGF_DEBUG("Pi 5 raw: requesting %s %dx%d via ISP",
                       bayer16.toString().c_str(),
                       mode.size.width, mode.size.height);
        }
        else
        {
            // Non-Pi 5: use the Raw stream directly
            m_Config = m_Camera->generateConfiguration({lc::StreamRole::Raw});
            if (!m_Config || m_Config->empty())
                return false;

            m_Config->at(0).pixelFormat = mode.format;
            m_Config->at(0).size = mode.size;
            m_ActivePixelFormat = mode.format;
            m_ActiveSize = mode.size;
            m_ActiveIsRaw = true;

            PrimaryCCD.setBPP(16);
            PrimaryCCD.setNAxis(2);
        }
    }
    else
    {
        // ISP-processed RGB or Mono output
        m_Config = m_Camera->generateConfiguration(
            {lc::StreamRole::StillCapture});
        if (!m_Config || m_Config->empty())
            return false;

        // Use ProcFrame dimensions if set, otherwise use sensor resolution
        int w = static_cast<int>(ProcFrameNP[0].getValue());
        int h = static_cast<int>(ProcFrameNP[1].getValue());
        if (w <= 0 || h <= 0)
        {
            w = PrimaryCCD.getXRes();
            h = PrimaryCCD.getYRes();
        }

        m_Config->at(0).pixelFormat = lc::formats::BGR888;
        m_Config->at(0).size = {static_cast<unsigned>(w),
                                static_cast<unsigned>(h)};
        m_ActivePixelFormat = lc::formats::BGR888;
        m_ActiveSize = {static_cast<unsigned>(w),
                        static_cast<unsigned>(h)};
        m_ActiveIsRaw = false;

        if (capFmt == "INDI_MONO")
        {
            PrimaryCCD.setBPP(8);
            PrimaryCCD.setNAxis(2);
        }
        else
        {
            PrimaryCCD.setBPP(8);
            PrimaryCCD.setNAxis(3);
        }
    }

    auto status = m_Config->validate();
    if (status == lc::CameraConfiguration::Invalid)
    {
        LOG_ERROR("Camera configuration is invalid.");
        return false;
    }
    if (status == lc::CameraConfiguration::Adjusted)
        LOG_WARN("Camera configuration was adjusted by the driver.");

    // Check for Pi 5 PISP compressed — we cannot handle this
    {
        std::string valFmt = m_Config->at(0).pixelFormat.toString();
        if (valFmt.find("PISP") != std::string::npos ||
            valFmt.find("_COMP") != std::string::npos)
        {
            LOGF_ERROR("Unsupported raw format after validation: %s. "
                       "No unpacked Bayer format available.", valFmt.c_str());
            return false;
        }
    }

    if (m_Camera->configure(m_Config.get()))
    {
        LOG_ERROR("Failed to apply camera configuration.");
        return false;
    }

    // Re-read the actual negotiated format/size/stride after configure()
    m_ActivePixelFormat = m_Config->at(0).pixelFormat;
    m_ActiveSize = m_Config->at(0).size;

    LOGF_INFO("Still config: %dx%d  fmt=%s  stride=%u",
              m_ActiveSize.width, m_ActiveSize.height,
              m_ActivePixelFormat.toString().c_str(),
              m_Config->at(0).stride);

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

    // Re-read actual negotiated format/size/stride after configure()
    m_ActivePixelFormat = m_Config->at(0).pixelFormat;
    m_ActiveSize = m_Config->at(0).size;

    LOGF_INFO("Streaming config: %dx%d  fmt=%s  stride=%u  srcBpp=%d",
              m_ActiveSize.width, m_ActiveSize.height,
              m_ActivePixelFormat.toString().c_str(),
              m_Config->at(0).stride,
              (m_ActivePixelFormat.toString().find("X") != std::string::npos) ? 4 : 3);

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

    if (m_IsStreaming && m_CameraRunning)
    {
        // ---- Streaming: feed frame to INDI StreamManager ----
        const auto &buffers = request->buffers();
        if (!buffers.empty())
        {
            const lc::FrameBuffer *fb = buffers.begin()->second;
            auto it = m_MappedBuffers.find(fb);
            if (it != m_MappedBuffers.end() && !it->second.empty())
            {
                const uint8_t *srcData =
                    static_cast<const uint8_t *>(it->second[0].memory);
                size_t srcLen = it->second[0].length;

                // The DMA buffer may have stride padding per row.
                // StreamManager expects exactly width*height*3 bytes (RGB).
                unsigned int stride = m_Config->at(0).stride;
                unsigned int width  = m_ActiveSize.width;
                unsigned int height = m_ActiveSize.height;

                // Detect 4-bpp formats (XBGR8888 etc.)
                int srcBpp = 3;
                std::string fmtStr = m_ActivePixelFormat.toString();
                if (fmtStr.find("XB") != std::string::npos ||
                    fmtStr.find("XR") != std::string::npos ||
                    fmtStr.find("BX") != std::string::npos ||
                    fmtStr.find("RX") != std::string::npos ||
                    fmtStr.find("AB") != std::string::npos ||
                    fmtStr.find("AR") != std::string::npos)
                    srcBpp = 4;

                unsigned int rowBytes = width * 3; // output is always RGB 3bpp
                size_t frameBytes = static_cast<size_t>(rowBytes) * height;

                // Log first frame diagnostics
                if (m_StreamFrameCount < 3)
                {
                    LOGF_INFO("Stream frame %u: %ux%u stride=%u srcBpp=%d "
                              "fmt=%s srcLen=%zu frameBytes=%zu",
                              m_StreamFrameCount, width, height, stride,
                              srcBpp, fmtStr.c_str(), srcLen, frameBytes);
                    // Sample a few pixels for sanity
                    if (srcLen >= 12)
                        LOGF_INFO("  First 12 bytes: %02x %02x %02x %02x  "
                                  "%02x %02x %02x %02x  %02x %02x %02x %02x",
                                  srcData[0], srcData[1], srcData[2], srcData[3],
                                  srcData[4], srcData[5], srcData[6], srcData[7],
                                  srcData[8], srcData[9], srcData[10], srcData[11]);
                }

                // Ensure pre-allocated streaming buffer is large enough
                if (m_StreamBuffer.size() < frameBytes)
                    m_StreamBuffer.resize(frameBytes);

                if (srcBpp == 4 || stride != rowBytes)
                {
                    // Per-row processing: stride padding and/or 4→3 bpp
                    for (unsigned int y = 0; y < height; y++)
                    {
                        const uint8_t *srcRow = srcData + y * stride;
                        uint8_t *dstRow = m_StreamBuffer.data() + y * rowBytes;

                        if (srcBpp == 4)
                        {
                            // XBGR8888 → RGB888: drop alpha, swap B↔R
                            for (unsigned int x = 0; x < width; x++)
                            {
                                dstRow[x * 3 + 0] = srcRow[x * 4 + 2]; // R
                                dstRow[x * 3 + 1] = srcRow[x * 4 + 1]; // G
                                dstRow[x * 3 + 2] = srcRow[x * 4 + 0]; // B
                            }
                        }
                        else
                        {
                            // BGR888 with stride padding — swap B↔R per pixel
                            for (unsigned int x = 0; x < width; x++)
                            {
                                dstRow[x * 3 + 0] = srcRow[x * 3 + 2]; // R
                                dstRow[x * 3 + 1] = srcRow[x * 3 + 1]; // G
                                dstRow[x * 3 + 2] = srcRow[x * 3 + 0]; // B
                            }
                        }
                    }
                }
                else
                {
                    // No stride padding — but still need BGR→RGB swap
                    for (unsigned int y = 0; y < height; y++)
                    {
                        const uint8_t *srcRow = srcData + y * rowBytes;
                        uint8_t *dstRow = m_StreamBuffer.data() + y * rowBytes;
                        for (unsigned int x = 0; x < width; x++)
                        {
                            dstRow[x * 3 + 0] = srcRow[x * 3 + 2]; // R
                            dstRow[x * 3 + 1] = srcRow[x * 3 + 1]; // G
                            dstRow[x * 3 + 2] = srcRow[x * 3 + 0]; // B
                        }
                    }
                }
                Streamer->newFrame(m_StreamBuffer.data(), frameBytes);

                m_StreamFrameCount++;

                // Read sensor temperature periodically during streaming
                if (m_StreamFrameCount % 30 == 0)
                {
                    const auto &metadata = request->metadata();
                    auto temp = metadata.get(lc::controls::SensorTemperature);
                    if (temp)
                    {
                        m_SensorTemperature = static_cast<double>(*temp);
                        TemperatureNP[0].setValue(m_SensorTemperature);
                        TemperatureNP.setState(IPS_OK);
                        TemperatureNP.apply();
                    }
                }
            }
        }

        // Re-queue for continuous capture
        request->reuse(lc::Request::ReuseBuffers);

        // Apply all current controls (exposure, gain, AWB, ISP tuning…)
        // so that property changes made during streaming take effect
        // on the very next frame.
        double streamExp = Streamer->getTargetExposure();
        int32_t expUs = static_cast<int32_t>(streamExp * 1e6);
        if (expUs < 100) expUs = 100;
        request->controls().set(lc::controls::ExposureTime, expUs);
        request->controls().set(lc::controls::AnalogueGain,
                                static_cast<float>(GainNP[0].getValue()));
        applyCameraControls(request->controls());

        // Enforce target FPS via FrameDurationLimits
        double targetFps = StreamFpsNP[0].getValue();
        if (targetFps < 1.0) targetFps = 1.0;
        int64_t frameDurUs = static_cast<int64_t>(1e6 / targetFps);
        request->controls().set(lc::controls::FrameDurationLimits,
                                lc::Span<const int64_t, 2>({frameDurUs, frameDurUs}));

        m_Camera->queueRequest(request);

        // Measure actual FPS (update every 30 frames)
        m_StreamFpsFrames++;
        if (m_StreamFpsFrames >= 30)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - m_StreamFpsStart).count();
            if (elapsed > 0)
                m_StreamEstFps = m_StreamFpsFrames / elapsed;
            m_StreamFpsStart = now;
            m_StreamFpsFrames = 0;

            StreamEstFpsNP[0].setValue(m_StreamEstFps);
            StreamEstFpsNP.setState(IPS_OK);
            StreamEstFpsNP.apply();
        }
    }
    else
    {
        // ---- Still capture: signal that frame is ready ----
        std::lock_guard<std::mutex> lock(m_CompletedMutex);
        m_CompletedRequest = request;
        m_FrameReady = true;

        // In fast mode, re-queue the request immediately for the next frame
        // (the request will be re-used after downloadImage processes it)
        if (m_FastMode && m_CameraRunning)
        {
            // We need to re-queue a request after download processes the buffer.
            // The actual re-queue happens in downloadImage → handleFastExposureFrame
            // after the buffer is copied.
        }
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

    // ---- Lens Position (manual focus) ----
    if (m_HasLensPosition && LensPositionNP.isNameMatch(name))
    {
        LensPositionNP.update(values, names, n);
        LensPositionNP.setState(IPS_OK);
        LensPositionNP.apply();
        LOGF_INFO("Focus position set to %.2f dioptres", LensPositionNP[0].getValue());
        return true;
    }

    // ---- Exposure Value ----
    if (m_HasExposureValue && ExposureValueNP.isNameMatch(name))
    {
        ExposureValueNP.update(values, names, n);
        ExposureValueNP.setState(IPS_OK);
        ExposureValueNP.apply();
        LOGF_INFO("Exposure Value set to %.1f", ExposureValueNP[0].getValue());
        return true;
    }

    // ---- Fast Count ----
    if (FastCountNP.isNameMatch(name))
    {
        FastCountNP.update(values, names, n);
        FastCountNP.setState(IPS_OK);
        FastCountNP.apply();
        LOGF_INFO("Fast count set to %.0f", FastCountNP[0].getValue());
        return true;
    }

    // ---- ProcFrame (ISP output size) ----
    if (ProcFrameNP.isNameMatch(name))
    {
        ProcFrameNP.update(values, names, n);
        ProcFrameNP.setState(IPS_OK);
        ProcFrameNP.apply();
        LOGF_INFO("ISP output size set to %.0fx%.0f",
                  ProcFrameNP[0].getValue(), ProcFrameNP[1].getValue());
        return true;
    }

    // ---- Stream Custom Resolution ----
    if (StreamCustomResNP.isNameMatch(name))
    {
        StreamCustomResNP.update(values, names, n);
        StreamCustomResNP.setState(IPS_OK);
        StreamCustomResNP.apply();
        LOGF_INFO("Stream custom resolution set to %.0fx%.0f",
                  StreamCustomResNP[0].getValue(), StreamCustomResNP[1].getValue());
        return true;
    }

    // ---- Stream Target FPS ----
    if (StreamFpsNP.isNameMatch(name))
    {
        StreamFpsNP.update(values, names, n);
        StreamFpsNP.setState(IPS_OK);
        StreamFpsNP.apply();
        LOGF_INFO("Stream target FPS set to %.1f", StreamFpsNP[0].getValue());
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
                BayerTP[2].setText(bayer.c_str());

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

    // ---- Raw Left Shift ----
    if (RawLeftShiftSP.isNameMatch(name))
    {
        RawLeftShiftSP.update(states, names, n);
        RawLeftShiftSP.setState(IPS_OK);
        RawLeftShiftSP.apply();
        LOGF_INFO("Raw normalize (left-shift): %s",
                  RawLeftShiftSP.findOnSwitchIndex() == 0 ? "ON" : "OFF");
        return true;
    }

    // ---- Fast Exposure Toggle ----
    if (FastExposureSP.isNameMatch(name))
    {
        FastExposureSP.update(states, names, n);
        FastExposureSP.setState(IPS_OK);
        FastExposureSP.apply();
        LOGF_INFO("Fast exposure: %s",
                  FastExposureSP.findOnSwitchIndex() == 0 ? "ON" : "OFF");
        return true;
    }

    // ---- AE Constraint Mode ----
    if (m_HasAeConstraintMode && AeConstraintModeSP.isNameMatch(name))
    {
        AeConstraintModeSP.update(states, names, n);
        AeConstraintModeSP.setState(IPS_OK);
        AeConstraintModeSP.apply();
        return true;
    }

    // ---- AE Exposure Mode ----
    if (m_HasAeExposureMode && AeExposureModeSP.isNameMatch(name))
    {
        AeExposureModeSP.update(states, names, n);
        AeExposureModeSP.setState(IPS_OK);
        AeExposureModeSP.apply();
        return true;
    }

    // ---- AE Metering Mode ----
    if (m_HasAeMeteringMode && AeMeteringModeSP.isNameMatch(name))
    {
        AeMeteringModeSP.update(states, names, n);
        AeMeteringModeSP.setState(IPS_OK);
        AeMeteringModeSP.apply();
        return true;
    }

    // ---- AF Metering ----
    if (m_HasAfMetering && AfMeteringSP.isNameMatch(name))
    {
        AfMeteringSP.update(states, names, n);
        AfMeteringSP.setState(IPS_OK);
        AfMeteringSP.apply();
        return true;
    }

    // ---- AF Pause ----
    if (m_HasAfPause && AfPauseSP.isNameMatch(name))
    {
        AfPauseSP.update(states, names, n);
        AfPauseSP.setState(IPS_OK);
        AfPauseSP.apply();
        AfPauseSP.reset();
        AfPauseSP.apply();
        return true;
    }

    // ---- AF Range ----
    if (m_HasAfRange && AfRangeSP.isNameMatch(name))
    {
        AfRangeSP.update(states, names, n);
        AfRangeSP.setState(IPS_OK);
        AfRangeSP.apply();
        return true;
    }

    // ---- AF Speed ----
    if (m_HasAfSpeed && AfSpeedSP.isNameMatch(name))
    {
        AfSpeedSP.update(states, names, n);
        AfSpeedSP.setState(IPS_OK);
        AfSpeedSP.apply();
        return true;
    }

    // ---- Stream Resolution Preset ----
    if (StreamResSP.isNameMatch(name))
    {
        StreamResSP.update(states, names, n);
        StreamResSP.setState(IPS_OK);
        StreamResSP.apply();
        int idx = StreamResSP.findOnSwitchIndex();
        const char *labels[] = {"720p", "1080p", "4K", "Custom"};
        LOGF_INFO("Stream resolution set to %s", labels[idx]);
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

        // AE sub-modes
        if (m_HasAeConstraintMode)
        {
            int idx = AeConstraintModeSP.findOnSwitchIndex();
            ctrlList.set(lc::controls::AeConstraintMode, static_cast<int32_t>(idx));
        }
        if (m_HasAeExposureMode)
        {
            int idx = AeExposureModeSP.findOnSwitchIndex();
            ctrlList.set(lc::controls::AeExposureMode, static_cast<int32_t>(idx));
        }
        if (m_HasAeMeteringMode)
        {
            int idx = AeMeteringModeSP.findOnSwitchIndex();
            ctrlList.set(lc::controls::AeMeteringMode, static_cast<int32_t>(idx));
        }
    }

    // Exposure Value (EV compensation)
    if (m_HasExposureValue)
    {
        ctrlList.set(lc::controls::ExposureValue,
                     static_cast<float>(ExposureValueNP[0].getValue()));
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

    // For INDI_MONO capture: force saturation to 0 so R≈G≈B
    if (m_ActiveCaptureFmt == "INDI_MONO")
        ctrlList.set(lc::controls::Saturation, 0.0f);
    else
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

        // Extended AF controls
        if (m_HasAfMetering)
        {
            int idx = AfMeteringSP.findOnSwitchIndex();
            ctrlList.set(lc::controls::AfMetering, static_cast<int32_t>(idx));
        }
        if (m_HasAfPause)
        {
            int idx = AfPauseSP.findOnSwitchIndex();
            if (idx >= 0)
                ctrlList.set(lc::controls::AfPause, static_cast<int32_t>(idx));
        }
        if (m_HasAfRange)
        {
            int idx = AfRangeSP.findOnSwitchIndex();
            ctrlList.set(lc::controls::AfRange, static_cast<int32_t>(idx));
        }
        if (m_HasAfSpeed)
        {
            int idx = AfSpeedSP.findOnSwitchIndex();
            ctrlList.set(lc::controls::AfSpeed, static_cast<int32_t>(idx));
        }
    }

    // Lens Position (manual focus)
    if (m_HasLensPosition)
    {
        int afIdx = m_HasAF ? AfModeSP.findOnSwitchIndex() : AF_MANUAL;
        if (afIdx == AF_MANUAL)
        {
            ctrlList.set(lc::controls::LensPosition,
                         static_cast<float>(LensPositionNP[0].getValue()));
        }
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
    fitsKeywords.push_back(INDI::FITSRecord("GAIN", GainNP[0].getValue(), 3, "Analog Gain"));

    // Digital gain (ISP)
    if (m_LastDigitalGain > 0)
        fitsKeywords.push_back(INDI::FITSRecord("DGAIN", m_LastDigitalGain, 3, "ISP Digital Gain"));

    // Sensor model
    if (!m_SensorModel.empty())
        fitsKeywords.push_back(INDI::FITSRecord("SENSOR", m_SensorModel.c_str(), "Camera Sensor Model"));

    // Sensor-specific friendly name
    if (!m_SensorAdj.friendlyName.empty())
        fitsKeywords.push_back(INDI::FITSRecord("CAMNAME", m_SensorAdj.friendlyName.c_str(),
                                "Camera Module"));

    // Sensor temperature
    if (m_SensorTemperature != 0)
        fitsKeywords.push_back(INDI::FITSRecord("CCD-TEMP", m_SensorTemperature, 1,
                                "Sensor Temperature (C)"));

    // Precise UTC timestamp of exposure start
    if (!m_ExposureDateObs.empty())
        fitsKeywords.push_back(INDI::FITSRecord("DATE-BEG", m_ExposureDateObs.c_str(),
                                "UTC Exposure Start"));

    // UTC timestamp of exposure end (frame download)
    if (!m_ExposureDateEnd.empty())
        fitsKeywords.push_back(INDI::FITSRecord("DATE-END", m_ExposureDateEnd.c_str(),
                                "UTC Exposure End"));

    // Actual exposure time from sensor metadata (vs requested)
    if (m_LastActualExposureUs > 0)
        fitsKeywords.push_back(INDI::FITSRecord("EXPTIME", m_LastActualExposureUs / 1e6, 6,
                                "Actual Exposure Time (s)"));

    // Image scale (arcsec/pixel) — computed from focal length and pixel size
    {
        double focalLen = ScopeInfoNP[FOCAL_LENGTH].getValue();  // mm
        double pixSizeUm = PrimaryCCD.getPixelSizeX();           // µm
        if (focalLen > 0 && pixSizeUm > 0)
        {
            double scale = (pixSizeUm / focalLen) * 206.265;    // arcsec/pixel
            fitsKeywords.push_back(INDI::FITSRecord("SCALE", scale, 4,
                                    "Image Scale (arcsec/pixel)"));
        }
    }

    // Sensor black levels per Bayer channel
    if (m_LastBlackLevels[0] != 0 || m_LastBlackLevels[1] != 0 ||
        m_LastBlackLevels[2] != 0 || m_LastBlackLevels[3] != 0)
    {
        fitsKeywords.push_back(INDI::FITSRecord("BLKLVL0", static_cast<int64_t>(m_LastBlackLevels[0]),
                                "Black Level Ch0"));
        fitsKeywords.push_back(INDI::FITSRecord("BLKLVL1", static_cast<int64_t>(m_LastBlackLevels[1]),
                                "Black Level Ch1"));
        fitsKeywords.push_back(INDI::FITSRecord("BLKLVL2", static_cast<int64_t>(m_LastBlackLevels[2]),
                                "Black Level Ch2"));
        fitsKeywords.push_back(INDI::FITSRecord("BLKLVL3", static_cast<int64_t>(m_LastBlackLevels[3]),
                                "Black Level Ch3"));
    }

    // Actual bit depth of the raw data (before promotion to 16-bit)
    if (m_ActiveIsRaw && m_CurrentModeIndex < m_NumSensorModes)
    {
        int64_t bd = m_IsPiSP && m_NativeBitDepth > 0
                   ? m_NativeBitDepth
                   : m_SensorModes[m_CurrentModeIndex].bitDepth;
        fitsKeywords.push_back(INDI::FITSRecord("RAWBPP",
                                bd, "Native Sensor Bit Depth"));
    }
}

// ============================================================
//  Configuration Persistence
// ============================================================

bool RPiCamera::saveConfigItems(FILE *fp)
{
    INDI::CCD::saveConfigItems(fp);

    GainNP.save(fp);
    RawLeftShiftSP.save(fp);
    FastExposureSP.save(fp);
    FastCountNP.save(fp);
    ProcFrameNP.save(fp);
    StreamResSP.save(fp);
    StreamCustomResNP.save(fp);
    StreamFpsNP.save(fp);

    if (m_HasAE)
    {
        AutoExposureSP.save(fp);
        if (m_HasAeConstraintMode)
            AeConstraintModeSP.save(fp);
        if (m_HasAeExposureMode)
            AeExposureModeSP.save(fp);
        if (m_HasAeMeteringMode)
            AeMeteringModeSP.save(fp);
    }
    if (m_HasExposureValue)
        ExposureValueNP.save(fp);
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
    {
        AfModeSP.save(fp);
        if (m_HasAfMetering)
            AfMeteringSP.save(fp);
        if (m_HasAfRange)
            AfRangeSP.save(fp);
        if (m_HasAfSpeed)
            AfSpeedSP.save(fp);
    }

    if (m_HasLensPosition)
        LensPositionNP.save(fp);

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
//  Utility — Find matching unpacked 16-bit Bayer format
//
//  Used on Pi 5 (PiSP) to request ISP-decompressed raw Bayer
//  output via StillCapture instead of the PISP-compressed Raw
//  stream.  Maps the sensor's native Bayer pattern to the
//  corresponding 16-bit unpacked libcamera pixel format.
// ============================================================

lc::PixelFormat RPiCamera::matchingBayer16Format(
    const lc::PixelFormat &rawFmt) const
{
    std::string bayer = bayerPatternFromFormat(rawFmt);

    if (bayer == "GBRG") return lc::formats::SGBRG16;
    if (bayer == "BGGR") return lc::formats::SBGGR16;
    if (bayer == "RGGB") return lc::formats::SRGGB16;
    if (bayer == "GRBG") return lc::formats::SGRBG16;

    // Fallback: if pattern detection failed, try the original format
    // name to infer.  Default to SGBRG16.
    LOGF_WARN("Could not determine Bayer pattern for %s — defaulting to SGBRG16",
              rawFmt.toString().c_str());
    return lc::formats::SGBRG16;
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

// ============================================================
//  Utility — Left-shift raw pixel data to fill 16-bit range
// ============================================================

void RPiCamera::applyRawLeftShift(uint16_t *data, size_t numPixels,
                                  unsigned int bitDepth)
{
    if (bitDepth >= 16 || bitDepth == 0)
        return;

    unsigned int shift = 16 - bitDepth;
    LOGF_DEBUG("Left-shifting raw data by %u bits (%u-bit → 16-bit)",
               shift, bitDepth);

    for (size_t i = 0; i < numPixels; i++)
        data[i] = static_cast<uint16_t>(data[i] << shift);
}

// ============================================================
//  Utility — Convert Bayer raw to mono by summing 2×2 superpixels
//
//  Each 2×2 block of Bayer pixels is summed and clamped to 16 bits.
//  Output is half the width and half the height.  Can work in-place
//  (dst == src) because output is always smaller.
// ============================================================

void RPiCamera::convertRawToMono(const uint16_t *src, uint16_t *dst,
                                 int width, int height)
{
    int monoW = width / 2;
    int monoH = height / 2;

    for (int y = 0; y < monoH; y++)
    {
        const uint16_t *row0 = src + (y * 2) * width;
        const uint16_t *row1 = src + (y * 2 + 1) * width;
        uint16_t *dstRow = dst + y * monoW;

        for (int x = 0; x < monoW; x++)
        {
            uint32_t sum = static_cast<uint32_t>(row0[x * 2])
                         + static_cast<uint32_t>(row0[x * 2 + 1])
                         + static_cast<uint32_t>(row1[x * 2])
                         + static_cast<uint32_t>(row1[x * 2 + 1]);
            // Clamp to 16-bit
            if (sum > 65535) sum = 65535;
            dstRow[x] = static_cast<uint16_t>(sum);
        }
    }
}

// ============================================================
//  Fast Exposure — re-queue request for the next frame
// ============================================================

void RPiCamera::handleFastExposureFrame()
{
    if (!m_CameraRunning || !m_Camera)
        return;

    // In fast mode, the camera is still running.  We need to
    // start a new "exposure" cycle by setting m_InExposure and
    // waiting for the next frame to arrive.

    // Record new DATE-OBS
    {
        auto now = std::chrono::system_clock::now();
        auto tt  = std::chrono::system_clock::to_time_t(now);
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch()) % 1000;
        struct tm utc;
        gmtime_r(&tt, &utc);
        char buf[64];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03ld",
                 utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                 utc.tm_hour, utc.tm_min, utc.tm_sec,
                 static_cast<long>(ms.count()));
        m_ExposureDateObs = buf;
    }

    m_ExposureTimer.start();
    m_InExposure = true;
    m_FrameReady = false;

    // The camera is still running and will deliver the next frame
    // via requestComplete → m_FrameReady.  TimerHit will pick it up.
}

// ============================================================
//  Utility — Garbage column count for current sensor mode
// ============================================================

int RPiCamera::garbageColumnsForCurrentMode() const
{
    if (m_SensorAdj.perModeGarbage.empty())
        return m_SensorAdj.garbageColumns;

    unsigned int modeWidth = m_ActiveSize.width;
    for (const auto &entry : m_SensorAdj.perModeGarbage)
    {
        if (entry.modeWidth == modeWidth || entry.modeWidth == 0)
            return entry.garbageColumns;
    }

    return m_SensorAdj.garbageColumns;
}
