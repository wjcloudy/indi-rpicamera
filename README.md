# indi-rpicamera

Native C++ INDI CCD driver for Raspberry Pi cameras using the **libcamera** C++ API directly.

Replaces the Python-based `indi_pylibcamera` driver with a proper INDI 3rd-party driver that subclasses `INDI::CCD`, giving free access to the framework's FITS engine, BLOB transport, StreamManager (live video + SER recording), telescope snooping, and configuration persistence.

## Features

| Feature | Details |
|---------|---------|
| **Raw Bayer capture** | Direct sensor readout via `StreamRole::Raw`; 10/12/16-bit with automatic CSI-2 unpacking |
| **RGB capture** | ISP-processed still frames via `StreamRole::StillCapture` |
| **Live streaming** | `StreamManager` integration — MJPEG live view with SER/native recording for free |
| **Subframing (ROI)** | `CCD_FRAME` property crops the frame buffer in software (raw) or via ISP (RGB) |
| **Hardware binning** | Sensor-mode based; `CCD_BINNING` selects the matching mode automatically |
| **Full camera controls** | Gain, AE, AWB (mode + manual colour gains), brightness, contrast, saturation, sharpness, noise reduction |
| **Autofocus** | AF mode / trigger exposed when the sensor supports it (e.g. IMX708 Camera Module 3) |
| **Multi-camera** | Single binary enumerates all connected cameras — one INDI device per camera |
| **Telescope snooping** | Built-in `INDI::CCD` snooping for RA/DEC, site coordinates, pier side, focal length, filter — all added to FITS headers automatically |
| **Per-sensor quirks** | Lookup table for known modules (IMX477, OV5647, IMX708, IMX296, IMX462, IMX290, IMX415, IMX519) with garbage-column and forced-restart handling |
| **Config persistence** | All camera settings saved/restored via standard INDI configuration system |

## Supported Cameras

Any camera supported by libcamera on Raspberry Pi:

- Raspberry Pi HQ Camera (IMX477)
- Raspberry Pi Camera Module v1 (OV5647)
- Raspberry Pi Camera Module 3 (IMX708) — with autofocus
- Raspberry Pi Global Shutter Camera (IMX296)
- Arducam IMX462 / IMX290 / IMX415 / IMX519
- Any other libcamera-compatible CSI camera

## Building

### Prerequisites (Raspberry Pi OS Bookworm+)

```bash
# INDI library + development headers
sudo apt install libindi-dev

# libcamera development headers (already installed on RPi OS)
sudo apt install libcamera-dev

# Build tools
sudo apt install cmake build-essential pkg-config

# FITS I/O
sudo apt install libcfitsio-dev

# Compression
sudo apt install zlib1g-dev
```

### Compile

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Install

```bash
sudo make install
```

This installs:
- `/usr/bin/indi_rpicamera` — the driver binary
- `/usr/share/indi/indi_rpicamera.xml` — the INDI driver descriptor

## Usage

### With indiserver

```bash
indiserver indi_rpicamera
```

The driver will automatically detect all connected cameras. Each camera appears as a separate INDI device ("RPi Camera" for a single camera, or "RPi Camera #0", "RPi Camera #1", etc. for multiple).

### With KStars/Ekos

1. Open KStars → Ekos → Profile Editor
2. Add a new CCD: select **RPi Camera** from the driver list
3. Start INDI and connect

### INDI Properties

| Group | Properties |
|-------|------------|
| **Main Control** | Connection, Exposure, Gain |
| **Image Settings** | Capture Format (RAW/RGB), Raw Sensor Mode, Frame, Binning |
| **Camera Controls** | Auto Exposure, Auto WB, AWB Mode, Colour Gains, Brightness, Contrast, Saturation, Sharpness, Noise Reduction, AF Mode/Trigger |
| **Streaming** | Video stream on/off, streaming exposure, FPS, recording (StreamManager) |
| **Options** | Upload mode, active devices (telescope snooping), debug, config |

## Architecture

```
indi_rpicamera_driver.cpp   Static Loader — enumerates cameras, creates instances
         │
         ▼
indi_rpicamera.h / .cpp     RPiCamera : INDI::CCD
         │                   ├── Connect/Disconnect (libcamera acquire/release)
         │                   ├── StartExposure/AbortExposure
         │                   ├── StartStreaming/StopStreaming (→ StreamManager)
         │                   ├── requestComplete() (libcamera async callback)
         │                   ├── downloadImage() (DMA buf → INDI frame buffer)
         │                   ├── Camera controls (ISNewNumber/ISNewSwitch)
         │                   └── addFITSKeywords() (gain, sensor model, etc.)
         │
sensor_info.h               Per-sensor adjustment table
```

## What INDI::CCD Provides for Free

By subclassing `INDI::CCD`, we inherit:

- **FITS creation** with standard headers (no manual astropy.io.fits)
- **BLOB encoding & transport** (no manual base64/zlib)
- **StreamManager** — MJPEG live view, SER/native recording, FPS tracking
- **Telescope snooping** — RA/DEC, site coords, pier side, airmass → FITS
- **Standard properties** — CCD_EXPOSURE, CCD_FRAME, CCD_BINNING, CCD_INFO, CCD_COMPRESSION, UPLOAD_MODE, ACTIVE_DEVICES
- **Configuration persistence** — save/load/default via INDI config system
- **Debug/logging framework**
- **Connection management** via indiserver

## License

LGPL-2.1-or-later
