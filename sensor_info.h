/*
 * sensor_info.h — Per-sensor adjustment table for known RPi camera modules
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#pragma once

#include <string>
#include <vector>

/**
 * @brief Per-sensor adjustment parameters.
 *
 * The libcamera camera ID string encodes the sensor model (e.g.
 * "/base/soc/i2c0mux/i2c@1/imx477@1a"). We match a substring to look up
 * hardware-specific quirks gathered from the Python indi_pylibcamera driver.
 */
/**
 * @brief Per-mode garbage column entry.
 *
 * Some sensors (e.g. IMX477) output padding columns in certain modes.
 * We store the mode width for matching, and how many garbage columns.
 */
struct GarbageColumnEntry
{
    unsigned int modeWidth{0};    ///< Sensor mode width to match (0 = any)
    int garbageColumns{0};        ///< Garbage pixel columns to crop from the right
};

struct SensorAdjustment
{
    std::string idMatch;          ///< Substring to match in camera->id()
    std::string friendlyName;     ///< e.g. "HQ Camera (IMX477)"
    int garbageColumns{0};        ///< Default garbage columns (if no per-mode match)
    bool forceRestart{false};     ///< Must stop/start camera between every frame
    std::vector<GarbageColumnEntry> perModeGarbage;  ///< Per-mode garbage column overrides
    // Per-mode binning info is detected dynamically from sensor mode sizes.
};

/**
 * @brief Built-in table of known Raspberry Pi camera modules.
 *
 * Data ported from indi_pylibcamera CameraControl.py `_camera_adjustments`.
 */
inline const std::vector<SensorAdjustment> &knownSensors()
{
    static const std::vector<SensorAdjustment> table = {
        // --- Official Raspberry Pi camera modules ---
        {
            "imx477",               // Raspberry Pi HQ Camera
            "HQ Camera (IMX477)",
            /* garbageColumns */ 0,
            /* forceRestart */  false,
            /* perModeGarbage */ {
                {4056, 8},   // Full resolution 4056x3040: 8 garbage columns
                {2028, 4},   // 2x2 binned: 4 garbage columns
            }
        },
        {
            "ov5647",               // Raspberry Pi Camera Module v1
            "Camera v1 (OV5647)",
            0,
            false,
            {}
        },
        {
            "imx219",               // Raspberry Pi Camera Module v2
            "Camera v2 (IMX219)",
            0,
            false,
            {}
        },
        {
            "imx708",               // Raspberry Pi Camera Module 3 (has AF)
            "Camera v3 (IMX708)",
            0,
            false,
            {}
        },
        {
            "imx296",               // Raspberry Pi Global Shutter Camera
            "GS Camera (IMX296)",
            0,
            false,
            {}
        },

        // --- Third-party / Arducam ---
        {
            "imx462",               // Arducam Pivariety IMX462
            "IMX462",
            0,
            false,
            {}
        },
        {
            "imx290",               // IMX290 — needs forced restart between frames
            "IMX290",
            0,
            /* forceRestart */ true,
            {}
        },
        {
            "imx415",               // IMX415
            "IMX415",
            0,
            false,
            {}
        },
        {
            "imx519",               // Arducam IMX519 — needs forced restart
            "IMX519",
            0,
            /* forceRestart */ true,
            {}
        },
    };
    return table;
}

/**
 * @brief Look up sensor adjustments for a camera ID string.
 *
 * @param cameraId  The libcamera camera->id() string.
 * @return Matching SensorAdjustment, or a default (no adjustments) if unknown.
 */
inline SensorAdjustment lookupSensor(const std::string &cameraId)
{
    for (const auto &s : knownSensors())
    {
        if (cameraId.find(s.idMatch) != std::string::npos)
            return s;
    }
    // Unknown sensor — no adjustments
    SensorAdjustment unknown;
    unknown.friendlyName = "Unknown Camera";
    return unknown;
}
