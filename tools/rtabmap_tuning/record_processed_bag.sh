#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_DIR="${SCRIPT_DIR}/bags"
DEFAULT_LAUNCH_FILE="/workspace/launch/launch_all.py"

usage() {
    cat <<'EOF'
Usage:
  record_processed_bag.sh [options] <bag_name> [extra ros2 bag record args...]

Options:
  --launch-file <path>   Launch file that brings up the stereo/depth pipeline
                         (default: /workspace/launch/launch_all.py)
  --help                 Show this help
EOF
}

require_cmd() {
    local cmd="$1"
    if ! command -v "${cmd}" >/dev/null 2>&1; then
        echo "Required command not found: ${cmd}"
        exit 1
    fi
}

wait_for_topic() {
    local topic="$1"
    local attempts="${2:-30}"
    local delay="${3:-1}"
    local i

    for ((i = 0; i < attempts; ++i)); do
        if ros2 topic info "${topic}" >/dev/null 2>&1; then
            return 0
        fi
        sleep "${delay}"
    done

    echo "Timed out waiting for topic: ${topic}"
    return 1
}

cleanup() {
    local exit_code=$?
    set +e

    if [[ -n "${BAG_PID:-}" ]] && kill -0 "${BAG_PID}" >/dev/null 2>&1; then
        kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
        wait "${BAG_PID}" >/dev/null 2>&1 || true
    fi

    if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
        kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
        wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
    fi

    exit "${exit_code}"
}

trap cleanup EXIT INT TERM

require_cmd ros2

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

LAUNCH_FILE="${DEFAULT_LAUNCH_FILE}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --launch-file)
            LAUNCH_FILE="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
            ;;
        --*)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
        *)
            break
            ;;
    esac
done

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

mkdir -p "${BAGS_DIR}"

BAG_NAME="$1"
shift || true

OUTPUT_PATH="${BAGS_DIR}/${BAG_NAME}"

if [[ -e "${OUTPUT_PATH}" ]]; then
    echo "Output path already exists: ${OUTPUT_PATH}"
    exit 1
fi

if [[ ! -f "${LAUNCH_FILE}" ]]; then
    echo "Launch file does not exist: ${LAUNCH_FILE}"
    exit 1
fi

echo "Recording processed RTAB-Map tuning bag to:"
echo "  ${OUTPUT_PATH}"
echo
echo "Starting processing pipeline:"
echo "  ${LAUNCH_FILE}"
echo
echo "Topics:"
echo "  /stereo/left/image_rect"
echo "  /stereo/left/camera_info"
echo "  /stereo/depth/image_rect"
echo "  /tf"
echo "  /tf_static"
echo
echo "Stop with Ctrl-C when you have covered a full loop and the failure cases you care about."

ros2 launch "${LAUNCH_FILE}" &
LAUNCH_PID="$!"

echo
echo "Waiting for processed topics to appear..."
wait_for_topic /stereo/left/image_rect
wait_for_topic /stereo/left/camera_info
wait_for_topic /stereo/depth/image_rect

echo "Processed topics are live. Starting bag record."

ros2 bag record \
    --output "${OUTPUT_PATH}" \
    /stereo/left/image_rect \
    /stereo/left/camera_info \
    /stereo/depth/image_rect \
    /tf \
    /tf_static \
    "$@" &

BAG_PID="$!"
wait "${BAG_PID}"
