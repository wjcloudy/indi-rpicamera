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
struct SensorAdjustment
{
    std::string idMatch;          ///< Substring to match in camera->id()
    std::string friendlyName;     ///< e.g. "HQ Camera (IMX477)"
    int garbageColumns{0};        ///< Garbage pixel columns to crop from the right
    bool forceRestart{false};     ///< Must stop/start camera between every frame
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
            /* garbageColumns */ 0,  // 8 cols in some modes — handled per-mode below
            /* forceRestart */  false
        },
        {
            "ov5647",               // Raspberry Pi Camera Module v1
            "Camera v1 (OV5647)",
            0,
            false
        },
        {
            "imx708",               // Raspberry Pi Camera Module 3 (has AF)
            "Camera v3 (IMX708)",
            0,
            false
        },
        {
            "imx296",               // Raspberry Pi Global Shutter Camera
            "GS Camera (IMX296)",
            0,
            false
        },

        // --- Third-party / Arducam ---
        {
            "imx462",               // Arducam Pivariety IMX462
            "IMX462",
            0,
            false
        },
        {
            "imx290",               // IMX290 — needs forced restart between frames
            "IMX290",
            0,
            /* forceRestart */ true
        },
        {
            "imx415",               // IMX415
            "IMX415",
            0,
            false
        },
        {
            "imx519",               // Arducam IMX519 — needs forced restart
            "IMX519",
            0,
            /* forceRestart */ true
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
