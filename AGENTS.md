# AGENTS.md — Notes for AI Agents Working on indi-rpicamera

> This file contains hard-won knowledge, architecture notes, gotchas, and
> testing procedures for anyone (human or AI) contributing to this driver.

---

## Project Overview

Native C++ INDI CCD driver for Raspberry Pi cameras using the **libcamera** C++ API.
Subclasses `INDI::CCD` to get free FITS creation, BLOB transport, streaming, telescope
snooping, and configuration persistence — with zero Python overhead.

**Current version**: 1.1  
**License**: LGPL-2.1-or-later  
**Target platform**: Raspberry Pi 4 / Pi 5 running Raspberry Pi OS Bookworm+  
**Primary test hardware**: Pi 5 with IMX415 (Arducam) via PiSP backend

---

## Repository Structure

```
indi_rpicamera.cpp          Main driver (~2950 lines). All the logic lives here.
indi_rpicamera.h            Class declaration, INDI properties, member variables.
indi_rpicamera_driver.cpp   Static loader — enumerates cameras, creates RPiCamera instances.
sensor_info.h               Per-sensor adjustment table (pixel pitch, garbage columns, quirks).
config.h.cmake              Template → build/config.h (version macros).
indi_rpicamera.xml.cmake    Template → build/indi_rpicamera.xml (INDI driver descriptor).
CMakeLists.txt              Build system. Version is defined here (MAJOR.MINOR).
install.sh                  One-command build + install script.
test_capture5.cpp           Standalone capture test (no INDI dependency).
cmake_modules/              FindINDI.cmake, FindCFITSIO.cmake.
build/                      Out-of-source build directory.
```

---

## Build & Install

```bash
# Quick: use the install script
./install.sh              # builds + sudo installs
./install.sh --build      # build only

# Manual:
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j$(nproc)
sudo make install
```

**Dependencies**: `libindi-dev`, `libcamera-dev`, `libcfitsio-dev`, `zlib1g-dev`, `cmake`, `build-essential`, `pkg-config`

**After editing code**: always rebuild with `cd build && make -j$(nproc)` and re-install
with `sudo make install` (or re-run `./install.sh`) so the system-wide binary is updated.
The desktop shortcut runs `/usr/bin/indi_rpicamera` — not the build directory copy.

---

## Running & Testing

### Start INDI Server

```bash
# From install (preferred — matches desktop shortcut)
indiserver indi_rpicamera

# From build dir (for quick iteration without install)
indiserver /home/pi/indi-rpicamera/build/indi_rpicamera
```

> **IMPORTANT**: `indiserver` uses `execlp` which searches `PATH`. Using
> `indiserver ./indi_rpicamera` with a **relative** path will fail with
> `execlp: No such file or directory` if the working directory changes.
> Always use an **absolute path** or ensure the binary is installed in PATH.

### INDI CLI Test (Python)

```python
import subprocess, time
DEVICE = "RPi Camera"
# Connect
subprocess.run(["indi_setprop", f"{DEVICE}.CONNECTION.CONNECT=On"])
time.sleep(3)
# Set RAW format
subprocess.run(["indi_setprop", f"{DEVICE}.CCD_CAPTURE_FORMAT.INDI_RAW=On"])
# Set local upload
subprocess.run(["indi_setprop", f"{DEVICE}.UPLOAD_MODE.UPLOAD_LOCAL=On"])
subprocess.run(["indi_setprop", f"{DEVICE}.UPLOAD_SETTINGS.UPLOAD_DIR=/tmp"])
# Take 1s exposure
subprocess.run(["indi_setprop", f"{DEVICE}.CCD_EXPOSURE.CCD_EXPOSURE_VALUE=1.0"])
time.sleep(8)
# Check result
from astropy.io import fits
hdul = fits.open("/tmp/<filename>.fits")
print(hdul[0].header['NAXIS'], hdul[0].data.shape)
```

### Standalone Capture Test (no INDI)

```bash
g++ -std=c++17 -O2 -o /tmp/test_capture5 test_capture5.cpp \
    $(pkg-config --cflags --libs libcamera) -lcfitsio
/tmp/test_capture5
# Outputs: /tmp/raw_proof.{pgm,ppm,fits}  /tmp/rgb_proof.{ppm,fits}
```

This is invaluable for isolating whether a bug is in the INDI layer or the
libcamera capture pipeline itself.

### Verifying FITS Output

```python
from astropy.io import fits
import numpy as np

with fits.open("/path/to/file.fits") as h:
    d = h[0].data
    hdr = h[0].header
    print(f"NAXIS={hdr['NAXIS']} shape={d.shape} dtype={d.dtype}")
    print(f"BAYERPAT={hdr.get('BAYERPAT','MISSING')}")
    print(f"min={d.min()} max={d.max()} mean={d.mean():.0f}")
    # Bayer channel check (GBRG pattern):
    G1 = d[0::2, 0::2].mean()
    B  = d[0::2, 1::2].mean()
    R  = d[1::2, 0::2].mean()
    G2 = d[1::2, 1::2].mean()
    print(f"G1={G1:.0f} B={B:.0f} R={R:.0f} G2={G2:.0f}")
```

**What correct RAW FITS looks like:**
- `NAXIS=2` (NOT 3)
- `BITPIX=16`
- Shape: `(height, width)` — e.g. `(2192, 3864)` for IMX415
- `BAYERPAT=GBRG` (or RGGB etc. depending on sensor)
- `RAWBPP=10` (native bit depth)
- G1 ≈ G2 (green channels should match within ~1%)
- Pixel values in range ~3000–50000 for a well-exposed PiSP frame

---

## Critical Architecture Knowledge

### PiSP (Pi 5) vs Non-PiSP (Pi 4) Raw Path

This is the **single most important** thing to understand about this driver.

**Pi 4 (non-PiSP)**:
- Raw stream (`StreamRole::Raw`) delivers native sensor data directly
- 10-bit CSI-2 packed format (e.g. `SGBRG10_CSI2P`)
- Driver must unpack CSI-2 → 16-bit, then optionally left-shift to fill 16 bits
- Pixel values are in native bit depth (0–1023 for 10-bit)

**Pi 5 (PiSP)**:
- Raw stream delivers **PISP-compressed** data that we cannot decode
- Instead, we route through the ISP: `StreamRole::StillCapture` + `SGBRG16`
- The ISP decompresses and outputs clean 16-bit Bayer
- **The ISP already left-shifts the data** (e.g. 10-bit value 48 → `48 << 6 = 3072`)
- Pixel values arrive pre-shifted to fill 16 bits

**Detection**: `m_IsPiSP` is set to `true` during `enumerateSensorModes()` when the
test config returns a PISP-compressed format. Also look for `BCM2712_C0` in libcamera logs.

### The Double Left-Shift Bug (Fixed in v1.1)

Before the fix, `applyRawLeftShift()` was called on PiSP data that was **already**
shifted to 16-bit by the ISP. This caused overflow:
- ISP delivers: `48 << 6 = 3072`
- Driver shifts again: `3072 << 6 = 196608` → wraps to garbage in `uint16_t`

**The fix** (in `downloadImage()`):
- PiSP + normalize ON → skip shift (data is already 16-bit)
- PiSP + normalize OFF → right-shift to restore native bit depth
- Non-PiSP → left-shift as before

### NAxis Must Be Set Correctly

The INDI base class `ExposureCompletePrivate` **reads** `getNAxis()` to decide the
FITS dimensionality. It never sets NAxis itself. The driver must explicitly call:
- `PrimaryCCD.setNAxis(2)` for RAW Bayer and Mono (2D image)
- `PrimaryCCD.setNAxis(3)` for RGB (3D plane-sequential cube)

`SetCCDParams()` does **not** touch NAxis. If you forget to set it, it defaults to 2
(from the CCDChip constructor), which is correct for raw but wrong for RGB if someone
previously set it to 3.

### Capture Format Flow

1. User selects format via `CCD_CAPTURE_FORMAT` (INDI_RAW / INDI_RGB / INDI_RAW_MONO / INDI_MONO)
2. `StartExposure()` → `configureForStill()` reads the active format
3. `configureForStill()` sets pixel format, NAxis, BPP, and configures the libcamera pipeline
4. `requestComplete()` callback fires when DMA buffer is ready → sets `m_FrameReady`
5. `TimerHit()` polls → sees `m_FrameReady` → calls `downloadImage()`
6. `downloadImage()` copies DMA buffer → INDI frame buffer, applies normalization
7. `ExposureComplete(&PrimaryCCD)` → INDI base class writes FITS, sends BLOB

### RGB Data Layout

libcamera delivers **BGR888** (or XBGR8888). The driver converts to **plane-sequential
RGB** for FITS 3D cubes: all R pixels, then all G, then all B. This is what INDI's
`ExposureCompletePrivate` expects when `NAxis=3`.

### Streaming Debayer Toggle

When streaming starts, the driver **deletes** the `BayerTP` property (removing the
Bayer pattern indicator) because the stream is debayered RGB. When streaming stops,
it **re-defines** BayerTP so that still captures get proper Bayer keywords. Use
`deleteProperty()` / `defineProperty()` for this — not just hiding it.

### AWB Warmup Frames (Added in v1.1)

**Problem**: RGB/ISP captures had a strong green hue on the first frame because
libcamera's AWB algorithm hadn't converged. Each `StartExposure()` call stops and
restarts the camera, resetting AWB state.

**Solution**: Before the real capture, the driver queues N short-exposure (30ms)
"warmup" frames with auto-exposure enabled, discarding each one. This gives the
ISP's AWB and AEC algorithms time to converge.

**Property**: `AWB_WARMUP` / `WARMUP_FRAMES` — configurable 0–20 frames (default 5).
Lives in the "Image Settings" tab.

**When warmup is skipped**:
- `warmup == 0` — user disabled it
- Fast Exposure mode — camera stays running between frames, AWB is already converged
- RAW captures — AWB doesn't affect raw Bayer data

**Implementation** (in `StartExposure()`, after `startCamera()`):
- Reuses `m_Requests[0]` with `ReuseBuffers`
- Sets 30ms exposure + `AeEnable=true` + `FrameDurationLimits`
- Applies user's AWB/ISP settings via `applyCameraControls()`
- Waits up to 5s per frame; on timeout, breaks and proceeds

---

## Sensor-Specific Quirks

Defined in `sensor_info.h`:

| Sensor | Notes |
|--------|-------|
| **IMX477** | Has garbage columns: 8 at 4056px, 4 at 2028px. Must be cropped. |
| **IMX290** | Needs `forceRestart=true` — camera must stop/start between frames. |
| **IMX708** | Has autofocus (AF controls exposed). |
| **IMX519** | Has autofocus. |
| **IMX415** | Primary test sensor. 10-bit native, 3864×2192, 1.45µm pitch. |

To add a new sensor: add an entry to the `knownSensors()` table in `sensor_info.h`.
The `idMatch` string is matched as a substring against `camera->id()`.

---

## Black Levels

libcamera reports `SensorBlackLevels` in metadata as 4 values (R, Gr, Gb, B).
On PiSP, these values are in **16-bit space** (already shifted), e.g. `3840` for
a 10-bit sensor with black level 60 (`60 << 6 = 3840`). They are stored in FITS
as `BLKLVL0`–`BLKLVL3`.

The IMX415 tuning file (`/usr/share/libcamera/ipa/rpi/pisp/imx415.json`) has
`black_level: 3840`.

---

## INDI Base Class Behaviour

Things the INDI `CCD` base class does that you should NOT reimplement:
- FITS file creation and writing (`ExposureCompletePrivate`)
- BLOB encoding, compression, and transport
- Standard properties (CCD_EXPOSURE, CCD_FRAME, CCD_BINNING, CCD_INFO, UPLOAD_MODE, etc.)
- StreamManager (MJPEG live view, SER recording)
- Telescope snooping (RA/DEC → FITS headers)
- Configuration save/load

Things you MUST do yourself:
- Set `NAxis`, `BPP`, `FrameBufferSize` before calling `ExposureComplete()`
- Fill the frame buffer (`PrimaryCCD.getFrameBuffer()`) with pixel data
- Call `SetCCDCapability()` with the right flags (BAYER, BINNING, STREAMING, etc.)
- Override `addFITSKeywords()` for driver-specific FITS headers
- Override `saveConfigItems()` to persist custom properties

---

## Common Pitfalls

1. **Relative paths with indiserver**: Always use absolute paths. `./indi_rpicamera`
   fails if the CWD changes.

2. **Camera busy**: Only one process can hold the camera. `pkill -f indi_rpicamera`
   or `pkill -f libcamera` before running tests.

3. **Port already in use**: If indiserver fails with `bind: Address already in use`,
   kill the old process: `pkill -9 indiserver; sleep 2`.

4. **Stride ≠ width×bpp**: Always use `m_Config->at(0).stride` for row offsets in
   the DMA buffer. The stride includes padding and alignment.

5. **Don't call SetCCDParams() after setting NAxis/Frame**: `SetCCDParams()` resets
   frame to (0,0,W,H) and binning to (1,1). Call it first, then set NAxis/Frame/BPP.

6. **FITS keywords only appear if NAxis=2**: The base class only adds `BAYERPAT`,
   `XBAYOFF`, `YBAYOFF` when `HasBayer() && getNAxis()==2`. So RGB captures
   (NAxis=3) won't have Bayer keywords — this is correct.

7. **libcamera controls are per-request**: Most controls go in the Request's
   ControlList, not the Camera's. The driver applies them in `applyCameraControls()`.

8. **BGR vs RGB**: libcamera outputs **BGR888**. Index 0 is Blue, 1 is Green, 2 is Red.
   Don't mix this up when converting to plane-sequential for FITS.

---

## Version Bumping

Version is defined in one place: [CMakeLists.txt](CMakeLists.txt) lines 11–12.
It propagates via `configure_file()` to:
- `build/config.h` → `RPICAMERA_VERSION_MAJOR` / `RPICAMERA_VERSION_MINOR` macros
- `build/indi_rpicamera.xml` → `<version>` tag
- The driver constructor calls `setVersion(RPICAMERA_VERSION_MAJOR, RPICAMERA_VERSION_MINOR)`

After changing the version, run `cmake ..` (or `./install.sh`) to regenerate config.h.

---

## Debugging

### INDI Server Verbosity

```bash
indiserver -v indi_rpicamera        # INFO messages
indiserver -vv indi_rpicamera       # + DEBUG messages (shows pixel stats, shift info)
indiserver -vvv indi_rpicamera      # + TRACE (very verbose)
```

`-vv` is most useful — shows `downloadImage` stats, PiSP shift decisions, pixel min/max/mean.

### Log Locations

- INDI server log: redirect with `indiserver ... > /tmp/indi.log 2>&1`
- libcamera messages appear on stderr (forwarded through indiserver)
- KStars INDI logs: `~/.local/share/kstars/logs/`

### Useful Grep Patterns

```bash
grep -iE "PiSP|left.shift|right.shift|RAW pixel|download|naxis|error" /tmp/indi.log
```

---

## Git Workflow

```bash
cd /home/pi/indi-rpicamera
# Edit code...
cd build && make -j$(nproc)          # Build
./install.sh                          # Or: build + install
# Test...
git add -A
git commit -m "Description of change"
git push origin main
```

Remote: `https://github.com/wjcloudy/indi-rpicamera.git`
