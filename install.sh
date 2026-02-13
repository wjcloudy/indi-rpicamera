#!/bin/bash
# ═══════════════════════════════════════════════════════════
#  install.sh — Build and install indi-rpicamera
# ═══════════════════════════════════════════════════════════
#  Usage:
#    ./install.sh          Build and install (sudo required)
#    ./install.sh --build  Build only (no install)
#    ./install.sh --help   Show this help
# ═══════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

# ── Parse arguments ────────────────────────────────────────
BUILD_ONLY=false
case "${1:-}" in
    --build)  BUILD_ONLY=true ;;
    --help|-h)
        echo "Usage: $0 [--build|--help]"
        echo "  (no args)  Build and install system-wide"
        echo "  --build    Build only, skip install"
        echo "  --help     Show this help"
        exit 0
        ;;
esac

echo "╔═══════════════════════════════════════════════════╗"
echo "║   indi-rpicamera — Build & Install               ║"
echo "╚═══════════════════════════════════════════════════╝"
echo ""

# ── Check dependencies ─────────────────────────────────────
echo "Checking dependencies..."
MISSING=()
pkg-config --exists libcamera 2>/dev/null || MISSING+=("libcamera-dev")
dpkg -s libindi-dev   >/dev/null 2>&1 || MISSING+=("libindi-dev")
dpkg -s libcfitsio-dev >/dev/null 2>&1 || MISSING+=("libcfitsio-dev")
dpkg -s zlib1g-dev    >/dev/null 2>&1 || MISSING+=("zlib1g-dev")
command -v cmake      >/dev/null 2>&1 || MISSING+=("cmake")
command -v g++        >/dev/null 2>&1 || MISSING+=("build-essential")

if [ ${#MISSING[@]} -gt 0 ]; then
    echo "  ✗ Missing packages: ${MISSING[*]}"
    echo ""
    echo "  Install with:"
    echo "    sudo apt install ${MISSING[*]}"
    echo ""
    exit 1
fi
echo "  ✓ All dependencies found"

# ── Build ──────────────────────────────────────────────────
echo ""
echo "Building..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j"$(nproc)"

# Show version
VERSION=$(grep '#define RPICAMERA_VERSION' config.h | awk '{print $3}' | paste -sd.)
echo ""
echo "  ✓ Built indi_rpicamera v${VERSION}"

if $BUILD_ONLY; then
    echo ""
    echo "Build-only mode — skipping install."
    echo "Binary: $BUILD_DIR/indi_rpicamera"
    exit 0
fi

# ── Install ────────────────────────────────────────────────
echo ""
echo "Installing (requires sudo)..."
sudo make install

# Verify installation
echo ""
if [ -x /usr/bin/indi_rpicamera ]; then
    echo "  ✓ /usr/bin/indi_rpicamera"
else
    echo "  ✗ /usr/bin/indi_rpicamera NOT FOUND"
fi

INDI_DATA_DIR=$(pkg-config --variable=datadir libindi 2>/dev/null || echo "/usr/share/indi")
XML_PATH="${INDI_DATA_DIR}/indi_rpicamera.xml"
if [ -f "$XML_PATH" ]; then
    echo "  ✓ ${XML_PATH}"
elif [ -f /usr/share/indi/indi_rpicamera.xml ]; then
    echo "  ✓ /usr/share/indi/indi_rpicamera.xml"
else
    echo "  ✗ Driver XML not found in INDI data dir"
    echo "    Copying manually..."
    sudo mkdir -p /usr/share/indi
    sudo cp "$BUILD_DIR/indi_rpicamera.xml" /usr/share/indi/
    echo "  ✓ /usr/share/indi/indi_rpicamera.xml (manual copy)"
fi

echo ""
echo "╔═══════════════════════════════════════════════════╗"
echo "║   ✓ indi_rpicamera v${VERSION} installed!            ║"
echo "╠═══════════════════════════════════════════════════╣"
echo "║                                                   ║"
echo "║   Run with:                                       ║"
echo "║     indiserver indi_rpicamera                     ║"
echo "║                                                   ║"
echo "║   Or use KStars / INDI Web Manager.               ║"
echo "║                                                   ║"
echo "╚═══════════════════════════════════════════════════╝"
