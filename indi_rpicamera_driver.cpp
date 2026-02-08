/*
 * indi_rpicamera_driver.cpp — Static loader / entry point
 *
 * Enumerates all cameras via a single shared CameraManager, then creates
 * one RPiCamera INDI driver instance per detected camera.  If no cameras
 * are found, a single placeholder instance is created so the driver still
 * appears in the INDI control panel (it will report an error on connect).
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "indi_rpicamera.h"

#include <libcamera/camera_manager.h>
#include <memory>
#include <deque>

namespace lc = libcamera;

/**
 * @brief RAII loader — constructed once at process start.
 *
 * The CameraManager is shared (via shared_ptr) with every RPiCamera
 * instance so it stays alive for the lifetime of the process.
 */
static class Loader
{
  public:
    std::shared_ptr<lc::CameraManager> cameraManager;
    std::deque<std::unique_ptr<RPiCamera>> cameras;

    Loader()
    {
        cameraManager = std::make_shared<lc::CameraManager>();
        int ret = cameraManager->start();
        if (ret)
        {
            // CameraManager failed (permissions, no /dev/media*, etc.)
            // Create a single placeholder — Connect() will fail gracefully.
            cameras.push_back(
                std::make_unique<RPiCamera>(cameraManager, 0, ""));
            return;
        }

        auto cams = cameraManager->cameras();
        if (cams.empty())
        {
            // No cameras detected — create placeholder.
            cameras.push_back(
                std::make_unique<RPiCamera>(cameraManager, 0, ""));
        }
        else
        {
            for (size_t i = 0; i < cams.size(); i++)
            {
                cameras.push_back(
                    std::make_unique<RPiCamera>(
                        cameraManager,
                        static_cast<int>(i),
                        cams[i]->id()));
            }
        }
    }

    ~Loader()
    {
        cameras.clear();
        if (cameraManager)
            cameraManager->stop();
    }

} loader;
