# indi-rpicamera

Native C++ INDI CCD driver for Raspberry Pi cameras using the **libcamera** C++ API directly.

Replaces Python-based drivers (e.g. `indi_pylibcamera`) with a proper INDI 3rd-party driver that subclasses `INDI::CCD`, giving free access to the framework's FITS engine, BLOB transport, StreamManager (live video + SER recording), telescope snooping, and configuration persistence — all with zero Python overhead.

---

## Features

### Capture Modes

| Format | Description |
|--------|-------------|
| **RAW** | Direct Bayer sensor readout via `StreamRole::Raw`. 10/12/16-bit with automatic CSI-2 unpacking to 16-bit FITS. Default format for astrophotography. |
| **RGB** | ISP-processed 24-bit colour stills via `StreamRole::StillCapture`. Good for quick visual previews and EAA. |
| **RAW Mono** | 2×2 Bayer superpixel summation — produces a single-channel 16-bit monochrome image at half resolution with improved SNR. Ideal for luminance-only work with colour sensors. |
| **Mono (ISP)** | ISP-processed greyscale via desaturation (saturation = 0). 8-bit single-channel output at full resolution. |

### Raw Left-Shift Normalisation

Raw pixel data is normalised to fill the full 16-bit range, maximising dynamic range visibility in FITS viewers and stacking software.

- **Pi 5 (PiSP)** — The ISP already delivers raw Bayer data left-shifted to 16-bit (e.g. 10-bit sensor data × 64). With normalisation **ON** (default) the data is passed through as-is. With normalisation **OFF** the driver right-shifts back to native bit depth (e.g. 0–1023 for 10-bit).
- **Pi 4 / non-PiSP** — The raw stream delivers native-depth data. With normalisation **ON** the driver left-shifts to fill 16-bit (e.g. 10-bit << 6). With normalisation **OFF** data is passed through at native depth.
- **Configurable** — toggle via the `RAW_LEFT_SHIFT` property
- **Default ON** — matches the behaviour expected by most astrophotography stacking software
- The `RAWBPP` FITS keyword records the native sensor bit depth regardless of normalisation state

### Fast Exposure Mode

Keeps the camera running between sequential frames, eliminating the ~1–2 second pipeline restart overhead per frame. Particularly useful for:

- Lucky imaging (planetary, lunar)
- Rapid flat-frame acquisition
- Time-lapse sequences

Configure via `FAST_EXPOSURE` (ON/OFF) and `FAST_COUNT` (number of frames, 0 = unlimited until abort).

### Streaming & Live Video

| Feature | Details |
|---------|---------|
| **Resolution presets** | 720p, 1080p, 4K, or custom width×height — selectable via `STREAM_RESOLUTION` |
| **Target FPS** | Configurable 1–120 FPS via `STREAM_FPS`. Enforced via libcamera `FrameDurationLimits`. |
| **Actual FPS display** | `STREAM_EST_FPS` — real-time measured FPS updated every 30 frames |
| **Live view** | MJPEG streaming via INDI StreamManager |
| **Recording** | SER and native format recording via INDI RecordManager |
| **Format** | BGR888 → RGB conversion for maximum compatibility |

### Camera Controls

| Control | Property | Description |
|---------|----------|-------------|
| **Gain** | `CCD_GAIN` | Analogue gain (sensor-dependent range) |
| **Auto Exposure** | `AUTO_EXPOSURE` | AE on/off toggle |
| **AE Constraint Mode** | `AE_CONSTRAINT_MODE` | Normal / Highlight / Shadows / Custom |
| **AE Exposure Mode** | `AE_EXPOSURE_MODE` | Normal / Short / Long / Custom |
| **AE Metering Mode** | `AE_METERING_MODE` | Centre-weighted / Spot / Matrix / Custom |
| **Exposure Value** | `EXPOSURE_VALUE` | EV compensation (−8.0 to +8.0 stops) |
| **Auto White Balance** | `AUTO_WB` | AWB on/off toggle |
| **AWB Mode** | `AWB_MODE` | Auto / Tungsten / Fluorescent / Indoor / Daylight / Cloudy / Custom |
| **Colour Gains** | `COLOUR_GAINS` | Manual red/blue gains when AWB is off |
| **Brightness** | `BRIGHTNESS` | ISP brightness adjustment (−1.0 to +1.0) |
| **Contrast** | `CONTRAST` | ISP contrast (0.0 to 32.0, default 1.0) |
| **Saturation** | `SATURATION` | ISP colour saturation (0.0 to 32.0, default 1.0) |
| **Sharpness** | `SHARPNESS` | ISP sharpening (0.0 to 16.0, default 1.0) |
| **Noise Reduction** | `NOISE_REDUCTION` | Off / Fast / High Quality |

### Autofocus (sensor-dependent)

Available on cameras with motorised focus (e.g. Raspberry Pi Camera Module 3 / IMX708):

| Control | Description |
|---------|-------------|
| **AF Mode** | Manual / Auto / Continuous |
| **AF Trigger** | Start / Cancel one-shot AF |
| **AF Metering** | Auto / Windows |
| **AF Range** | Normal / Macro / Full |
| **AF Speed** | Normal / Fast |
| **AF Pause** | Deferred / Immediate / Resume |
| **Lens Position** | Direct manual focus position control |

> Controls are only exposed if the connected camera actually advertises them.

### ISP Output Size (CCD_PROCFRAME)

For RGB and Mono (ISP) captures, you can specify a custom ISP output resolution via `CCD_PROCFRAME`. The ISP will scale the image to the requested size. Useful for quick-look downsampled previews.

### Subframing & Binning

- **CCD_FRAME** — software ROI crop (raw) or ISP crop (RGB)
- **CCD_BINNING** — selects the matching hardware-binned sensor mode automatically. For example, 2×2 binning selects a half-resolution sensor mode if available.

### Garbage Column Removal

Some sensors (e.g. IMX477 HQ Camera) output extra non-image columns at the edge of the frame. The driver has a per-sensor, per-mode lookup table and automatically trims these columns from the delivered image.

### FITS Header Augmentation

In addition to all standard INDI FITS keywords (RA, DEC, AIRMASS, OBJECT, FILTER, etc. via telescope snooping), the driver adds:

| Keyword | Description |
|---------|-------------|
| `GAIN` | Analogue gain setting |
| `EGAIN` | Effective electronic gain (e⁻/ADU), accounting for digital gain |
| `OFFSET` | Black level (from `SensorBlackLevels` metadata, per-channel) |
| `SENSOR` | Sensor model string (e.g. "imx415") |
| `BAYERPAT` | Bayer pattern (RGGB, GBRG, etc.) — raw only |
| `XBAYOFF` / `YBAYOFF` | Bayer pattern offset for subframe alignment |
| `DATE-OBS` | UTC timestamp at exposure start |
| `DATE-END` | UTC timestamp at frame readout completion |
| `SCALE` | Image scale (arcsec/pixel) when telescope focal length is known |
| `EXPTIME` | Actual exposure time (from sensor metadata, not requested) |
| `RAWBPP` | Native sensor bit depth (e.g. 10, 12) — raw only |
| `CAMTEMP` | Sensor temperature in °C (from metadata) |

### Multi-Camera Support

The driver binary automatically enumerates all connected libcamera-compatible cameras. Each camera appears as a separate INDI device:
- Single camera: **RPi Camera**
- Multiple cameras: **RPi Camera #0**, **RPi Camera #1**, etc.

### Configuration Persistence

All camera settings (gain, controls, streaming presets, format selections, etc.) are saved and restored via the standard INDI configuration system.

---

## Supported Cameras

Any camera supported by libcamera on Raspberry Pi, including:

| Camera | Sensor | Notes |
|--------|--------|-------|
| Raspberry Pi HQ Camera | IMX477 | Garbage column removal included |
| Raspberry Pi Camera Module v1 | OV5647 | |
| Raspberry Pi Camera Module 3 | IMX708 | Autofocus supported |
| Raspberry Pi Global Shutter Camera | IMX296 | |
| Arducam IMX462 | IMX462 | |
| Arducam IMX290 | IMX290 | |
| Arducam IMX415 | IMX415 | Tested on Pi 5 (PiSP) |
| Arducam IMX519 | IMX519 | Autofocus supported |

The per-sensor adjustment table (`sensor_info.h`) contains pixel pitch, garbage column counts, and other quirks for known modules. Unknown sensors work with sensible defaults.

---

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

### Install (system-wide)

```bash
sudo make install
```

This installs:
- `/usr/bin/indi_rpicamera` — the driver binary
- `/usr/share/indi/indi_rpicamera.xml` — the INDI driver descriptor

---

## Usage

### With indiserver

```bash
indiserver indi_rpicamera
```

Or run directly from the build directory:

```bash
indiserver /path/to/build/indi_rpicamera
```

### With KStars / Ekos

1. Open KStars → Ekos → Profile Editor
2. Under CCD, select **RPi Camera** from the driver list
3. Start the INDI profile and connect

### With any INDI client

The driver listens on the default INDI port **7624**. Any INDI-compatible client (KStars, CCDciel, PHD2, N.I.N.A. via INDI, etc.) can connect.

---

## INDI Property Reference

| Group | Properties |
|-------|------------|
| **Main Control** | Connection, CCD_EXPOSURE, CCD_GAIN |
| **Image Settings** | Capture Format (RAW / RGB / RAW Mono / Mono), Raw Sensor Mode, CCD_FRAME, CCD_BINNING, CCD_PROCFRAME, RAW_LEFT_SHIFT |
| **Camera Controls** | Auto Exposure, AE Constraint/Exposure/Metering Modes, Exposure Value, Auto WB, AWB Mode, Colour Gains, Brightness, Contrast, Saturation, Sharpness, Noise Reduction |
| **Autofocus** | AF Mode, AF Trigger, AF Metering, AF Range, AF Speed, AF Pause, Lens Position *(sensor-dependent)* |
| **Fast Exposure** | FAST_EXPOSURE (on/off), FAST_COUNT |
| **Streaming** | Stream Resolution (720p/1080p/4K/Custom), Custom Resolution, Target FPS, Actual FPS, Video Stream on/off, Recording |
| **Sensor Info** | CCD_INFO (resolution, pixel size, bit depth), Sensor Temperature |
| **Options** | Upload mode, Active Devices (telescope snooping), Debug, Configuration |

---

## Standalone Capture Test

A standalone test program (`test_capture5.cpp`) is included to verify the raw capture pipeline independently of INDI. It uses libcamera auto-exposure to find good settings, then captures:

- **Phase 1 — RAW Bayer**: PGM (16-bit greyscale), PPM (demosaiced colour), FITS (2D, NAXIS=2 with Bayer keywords)
- **Phase 2 — RGB ISP**: PPM (24-bit colour), FITS (3D, plane-sequential)

```bash
# Build (no INDI dependency — just libcamera + cfitsio)
g++ -std=c++17 -O2 -o test_capture5 test_capture5.cpp \
    $(pkg-config --cflags --libs libcamera) -lcfitsio

# Run
./test_capture5
# Output: /tmp/raw_proof.{pgm,ppm,fits}  /tmp/rgb_proof.{ppm,fits}
```

Useful for diagnosing capture issues in isolation or verifying FITS output format.

---

## Architecture

```
indi_rpicamera_driver.cpp    Static Loader — enumerates cameras, creates instances
         │
         ▼
indi_rpicamera.h / .cpp      RPiCamera : INDI::CCD
         │                    ├── Connect / Disconnect (libcamera acquire/release)
         │                    ├── StartExposure / AbortExposure
         │                    │    ├── configureForStill()  (Raw or ISP pipeline)
         │                    │    ├── requestComplete()    (async DMA callback)
         │                    │    └── downloadImage()      (unpack → PiSP-aware normalize → FITS)
         │                    ├── Fast Exposure loop (camera stays running)
         │                    ├── StartStreaming / StopStreaming
         │                    │    ├── configureForStreaming()  (BGR888 VideoRecording)
         │                    │    ├── Resolution presets + FrameDurationLimits
         │                    │    └── FPS measurement
         │                    ├── Camera controls (ISNewNumber / ISNewSwitch)
         │                    │    └── applyCameraControls()  (→ libcamera ControlList)
         │                    ├── addFITSKeywords()  (gain, sensor, DATE-END, SCALE…)
         │                    └── saveConfigItems()  (persist all settings)
         │
sensor_info.h                Per-sensor adjustment table
         │                    ├── Pixel pitch (µm)
         │                    ├── Garbage column counts (per mode)
         │                    └── Force-restart flag
         │
cmake_modules/               CMake find-modules for INDI and CFITSIO
```

## What INDI::CCD Provides for Free

By subclassing `INDI::CCD`, we inherit:

- **FITS creation** with standard headers (no manual cfitsio calls)
- **BLOB encoding & transport** (zlib compression, base64)
- **StreamManager** — MJPEG live view, SER/native recording
- **RecordManager** — video recording in multiple formats
- **Telescope snooping** — RA/DEC, site coords, pier side, airmass → FITS
- **Standard properties** — CCD_EXPOSURE, CCD_FRAME, CCD_BINNING, CCD_INFO, CCD_COMPRESSION, UPLOAD_MODE, ACTIVE_DEVICES
- **Configuration persistence** — save/load/default via INDI config system
- **Debug/logging framework**
- **Connection management** via indiserver

---

## Platform Requirements

- **Hardware**: Raspberry Pi 4 or Pi 5 (Pi 5 recommended for PiSP backend)
- **OS**: Raspberry Pi OS Bookworm (Debian 12) or later
- **libcamera**: v0.1+ (v0.5+ recommended for Pi 5 / PiSP)
- **INDI**: v2.0+

---

## License

LGPL-2.1-or-later
