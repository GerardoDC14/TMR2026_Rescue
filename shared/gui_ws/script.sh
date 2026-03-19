#!/bin/bash
# start_robot.sh — Run on the NUC (robot)
# Launches GStreamer H.264 video streams + your ROS2 sensor nodes
#
# Usage: ./start_robot.sh <LAPTOP_IP>
# Example: ./start_robot.sh 192.168.1.100

set -e

LAPTOP_IP="${1:?Usage: $0 <LAPTOP_IP>}"

# --- Configuration ---
# Camera devices and corresponding UDP ports
# Add/remove entries as needed for your cameras
CAMERAS=(
    "/dev/video0:5000"
    #"/dev/video2:5001"
    # "/dev/video4:5002"
)

WIDTH=640
HEIGHT=480
FRAMERATE=30
BITRATE=2000        # kbps per stream — tune this (lower = less airtime)
KEYFRAME_PERIOD=30  # keyframe every N frames (lower = faster recovery from loss)
FEC_PERCENTAGE=25   # forward error correction overhead (0 to disable)

# --- Preflight checks ---
echo "Checking GStreamer and VAAPI availability..."

if ! command -v gst-launch-1.0 &>/dev/null; then
    echo "ERROR: gst-launch-1.0 not found. Install with:"
    echo "  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \\"
    echo "    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \\"
    echo "    gstreamer1.0-plugins-ugly gstreamer1.0-vaapi"
    exit 1
fi

# Check if VAAPI encoder is available (Intel Quick Sync)
if gst-inspect-1.0 vaapih264enc &>/dev/null; then
    ENCODER="vaapih264enc rate-control=cbr bitrate=${BITRATE} tune=low-latency keyframe-period=${KEYFRAME_PERIOD}"
    echo "Using VAAPI H.264 hardware encoder"
elif gst-inspect-1.0 vah264enc &>/dev/null; then
    # Newer GStreamer versions (1.22+) use va-based elements
    ENCODER="vah264enc rate-control=cbr bitrate=${BITRATE} key-int-max=${KEYFRAME_PERIOD}"
    echo "Using VA H.264 hardware encoder"
else
    echo "WARNING: No hardware encoder found, falling back to x264enc (CPU, higher latency)"
    ENCODER="x264enc tune=zerolatency bitrate=${BITRATE} speed-preset=ultrafast key-int-max=${KEYFRAME_PERIOD}"
fi

# Build FEC element string (or empty if disabled)
if [ "$FEC_PERCENTAGE" -gt 0 ]; then
    FEC_ENC="rtpulpfecenc percentage=${FEC_PERCENTAGE} pt=122 !"
    echo "FEC enabled at ${FEC_PERCENTAGE}%"
else
    FEC_ENC=""
    echo "FEC disabled"
fi

# --- Launch GStreamer pipelines ---
PIDS=()

for entry in "${CAMERAS[@]}"; do
    IFS=':' read -r DEVICE PORT <<< "$entry"

    if [ ! -e "$DEVICE" ]; then
        echo "WARNING: $DEVICE not found, skipping"
        continue
    fi

    echo "Streaming $DEVICE -> ${LAPTOP_IP}:${PORT} (H.264 @ ${WIDTH}x${HEIGHT}@${FRAMERATE}fps, ${BITRATE}kbps)"

    gst-launch-1.0 -e \
        v4l2src device="$DEVICE" ! \
        "video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FRAMERATE}/1" ! \
        videoconvert ! \
        ${ENCODER} ! \
        h264parse ! \
        rtph264pay config-interval=1 pt=96 ! \
        ${FEC_ENC} \
        udpsink host="${LAPTOP_IP}" port="${PORT}" sync=false async=false \
    &
    PIDS+=($!)
done

echo ""
echo "Video streams started (PIDs: ${PIDS[*]})"
echo ""

# --- Launch ROS2 nodes ---
# Uncomment / modify to match your actual launch file:
# echo "Launching ROS2 sensor nodes..."
# ros2 launch your_package sensors.launch.py &
# PIDS+=($!)

# --- Wait and cleanup ---
cleanup() {
    echo ""
    echo "Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait
    echo "Done."
}

trap cleanup SIGINT SIGTERM

echo "All streams running. Press Ctrl+C to stop."
wait
