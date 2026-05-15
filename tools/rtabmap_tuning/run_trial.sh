#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_DIR="${SCRIPT_DIR}/results"
RUNS_DIR="${RESULTS_DIR}/runs"
SUMMARY_CSV="${RESULTS_DIR}/summary.csv"
DEFAULT_RVIZ="/workspace/launch/zed_stereo_rtabmap.rviz"

usage() {
    cat <<'EOF'
Usage:
  run_trial.sh --bag <bag_path> --preset <preset_yaml> [options]

Options:
  --bag <path>              Path to rosbag2 directory
  --preset <path>           YAML preset for rgbd_odometry + rtabmap
  --results-dir <path>      Override results root
  --rate <float>            Bag playback rate (default: 1.0)
  --rviz-config <path>      RViz config to open (default: /workspace/launch/zed_stereo_rtabmap.rviz)
  --no-rviz                 Do not open RViz
  --open-db-viewer          Open rtabmap-databaseViewer after playback
  --bag-extra "<args>"      Extra args forwarded to ros2 bag play
  --help                    Show this help
EOF
}

require_cmd() {
    local cmd="$1"
    if ! command -v "${cmd}" >/dev/null 2>&1; then
        echo "Required command not found: ${cmd}"
        exit 1
    fi
}

ensure_summary() {
    if [[ ! -f "${SUMMARY_CSV}" ]]; then
        mkdir -p "$(dirname "${SUMMARY_CSV}")"
        cat > "${SUMMARY_CSV}" <<'EOF'
run_id,date_utc,preset,bag,rate,odom_stability,loop_closure,map_quality,speed,failure_recovery,overall,db_path,notes_file
EOF
    fi
}

prompt_score() {
    local label="$1"
    local value
    while true; do
        read -r -p "${label} (0-5): " value
        if [[ "${value}" =~ ^[0-5]$ ]]; then
            printf '%s' "${value}"
            return
        fi
        echo "Enter a single digit from 0 to 5."
    done
}

cleanup() {
    local exit_code=$?
    set +e
    for pid in "${PIDS_TO_CLEAN[@]:-}"; do
        if kill -0 "${pid}" >/dev/null 2>&1; then
            kill "${pid}" >/dev/null 2>&1 || true
            wait "${pid}" >/dev/null 2>&1 || true
        fi
    done
    exit "${exit_code}"
}

PIDS_TO_CLEAN=()
trap cleanup EXIT INT TERM

require_cmd ros2

BAG_PATH=""
PRESET_PATH=""
PLAY_RATE="1.0"
OPEN_RVIZ="1"
OPEN_DB_VIEWER="0"
RVIZ_CONFIG="${DEFAULT_RVIZ}"
BAG_EXTRA=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag)
            BAG_PATH="$2"
            shift 2
            ;;
        --preset)
            PRESET_PATH="$2"
            shift 2
            ;;
        --results-dir)
            RESULTS_DIR="$2"
            RUNS_DIR="${RESULTS_DIR}/runs"
            SUMMARY_CSV="${RESULTS_DIR}/summary.csv"
            shift 2
            ;;
        --rate)
            PLAY_RATE="$2"
            shift 2
            ;;
        --rviz-config)
            RVIZ_CONFIG="$2"
            shift 2
            ;;
        --no-rviz)
            OPEN_RVIZ="0"
            shift
            ;;
        --open-db-viewer)
            OPEN_DB_VIEWER="1"
            shift
            ;;
        --bag-extra)
            BAG_EXTRA="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ -z "${BAG_PATH}" || -z "${PRESET_PATH}" ]]; then
    usage
    exit 1
fi

if [[ ! -d "${BAG_PATH}" ]]; then
    echo "Bag path does not exist or is not a directory: ${BAG_PATH}"
    exit 1
fi

if [[ ! -f "${PRESET_PATH}" ]]; then
    echo "Preset file does not exist: ${PRESET_PATH}"
    exit 1
fi

mkdir -p "${RUNS_DIR}"
ensure_summary

RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
PRESET_NAME="$(basename "${PRESET_PATH}" .yaml)"
RUN_ID="${RUN_STAMP}_${PRESET_NAME}"
RUN_DIR="${RUNS_DIR}/${RUN_ID}"
DB_PATH="${RUN_DIR}/rtabmap.db"
NOTES_FILE="${RUN_DIR}/notes.txt"
COMMANDS_FILE="${RUN_DIR}/commands.sh"
METADATA_FILE="${RUN_DIR}/metadata.txt"
PRESET_COPY="${RUN_DIR}/preset.yaml"

mkdir -p "${RUN_DIR}"
cp "${PRESET_PATH}" "${PRESET_COPY}"

cat > "${METADATA_FILE}" <<EOF
run_id=${RUN_ID}
date_utc=${RUN_STAMP}
preset=${PRESET_PATH}
bag=${BAG_PATH}
rate=${PLAY_RATE}
db_path=${DB_PATH}
rviz_config=${RVIZ_CONFIG}
EOF

cat > "${NOTES_FILE}" <<EOF
Run: ${RUN_ID}
Preset: ${PRESET_PATH}
Bag: ${BAG_PATH}

What improved?
- 

What got worse?
- 

Where did it fail?
- 

What should be tested next?
- 
EOF

cat > "${COMMANDS_FILE}" <<EOF
#!/usr/bin/env bash
ros2 run rtabmap_odom rgbd_odometry --ros-args \\
  -r __ns:=/rtabmap \\
  -r __node:=rgbd_odometry \\
  --params-file "${PRESET_COPY}" \\
  -r rgb/image:=/stereo/left/image_rect \\
  -r rgb/camera_info:=/stereo/left/camera_info \\
  -r depth/image:=/stereo/depth/image_rect \\
  -r depth/camera_info:=/stereo/left/camera_info \\
  -r odom:=odom

ros2 run rtabmap_slam rtabmap --ros-args \\
  -r __ns:=/rtabmap \\
  -r __node:=rtabmap \\
  --params-file "${PRESET_COPY}" \\
  -p database_path:="${DB_PATH}" \\
  -r rgb/image:=/stereo/left/image_rect \\
  -r rgb/camera_info:=/stereo/left/camera_info \\
  -r depth/image:=/stereo/depth/image_rect \\
  -r depth/camera_info:=/stereo/left/camera_info \\
  -r odom:=/rtabmap/odom

ros2 bag play "${BAG_PATH}" --clock --rate "${PLAY_RATE}" ${BAG_EXTRA}
EOF

chmod +x "${COMMANDS_FILE}"

echo "Starting trial ${RUN_ID}"
echo "Results folder: ${RUN_DIR}"
echo

ros2 run rtabmap_odom rgbd_odometry --ros-args \
    -r __ns:=/rtabmap \
    -r __node:=rgbd_odometry \
    --params-file "${PRESET_COPY}" \
    -r rgb/image:=/stereo/left/image_rect \
    -r rgb/camera_info:=/stereo/left/camera_info \
    -r depth/image:=/stereo/depth/image_rect \
    -r depth/camera_info:=/stereo/left/camera_info \
    -r odom:=odom \
    > "${RUN_DIR}/rgbd_odometry.log" 2>&1 &
PIDS_TO_CLEAN+=("$!")

sleep 2

ros2 run rtabmap_slam rtabmap --ros-args \
    -r __ns:=/rtabmap \
    -r __node:=rtabmap \
    --params-file "${PRESET_COPY}" \
    -p database_path:="${DB_PATH}" \
    -r rgb/image:=/stereo/left/image_rect \
    -r rgb/camera_info:=/stereo/left/camera_info \
    -r depth/image:=/stereo/depth/image_rect \
    -r depth/camera_info:=/stereo/left/camera_info \
    -r odom:=/rtabmap/odom \
    > "${RUN_DIR}/rtabmap.log" 2>&1 &
PIDS_TO_CLEAN+=("$!")

sleep 2

if [[ "${OPEN_RVIZ}" == "1" ]]; then
    if [[ -f "${RVIZ_CONFIG}" ]] && command -v rviz2 >/dev/null 2>&1; then
        rviz2 -d "${RVIZ_CONFIG}" > "${RUN_DIR}/rviz.log" 2>&1 &
        PIDS_TO_CLEAN+=("$!")
        sleep 2
    else
        echo "Skipping RViz: rviz2 or config file not available."
    fi
fi

echo "Playing bag..."
set +e
if [[ -n "${BAG_EXTRA}" ]]; then
    # shellcheck disable=SC2206
    BAG_EXTRA_ARGS=( ${BAG_EXTRA} )
else
    BAG_EXTRA_ARGS=()
fi
ros2 bag play "${BAG_PATH}" --clock --rate "${PLAY_RATE}" "${BAG_EXTRA_ARGS[@]}"
BAG_EXIT=$?
set -e

echo "Bag playback finished with exit code ${BAG_EXIT}."
echo "Stopping trial processes..."

for pid in "${PIDS_TO_CLEAN[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
        kill "${pid}" >/dev/null 2>&1 || true
        wait "${pid}" >/dev/null 2>&1 || true
    fi
done
PIDS_TO_CLEAN=()

echo
echo "Score this run:"
ODOM_STABILITY="$(prompt_score odom_stability)"
LOOP_CLOSURE="$(prompt_score loop_closure)"
MAP_QUALITY="$(prompt_score map_quality)"
SPEED="$(prompt_score speed)"
FAILURE_RECOVERY="$(prompt_score failure_recovery)"

OVERALL="$(awk "BEGIN { printf \"%.2f\", (${ODOM_STABILITY}+${LOOP_CLOSURE}+${MAP_QUALITY}+${SPEED}+${FAILURE_RECOVERY})/5.0 }")"

echo
echo "Add longer notes to:"
echo "  ${NOTES_FILE}"
if [[ -n "${EDITOR:-}" ]] && command -v "${EDITOR}" >/dev/null 2>&1; then
    "${EDITOR}" "${NOTES_FILE}" || true
else
    echo "Set \$EDITOR if you want this script to open the notes file automatically."
fi

printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${RUN_ID}" \
    "${RUN_STAMP}" \
    "$(basename "${PRESET_PATH}")" \
    "${BAG_PATH}" \
    "${PLAY_RATE}" \
    "${ODOM_STABILITY}" \
    "${LOOP_CLOSURE}" \
    "${MAP_QUALITY}" \
    "${SPEED}" \
    "${FAILURE_RECOVERY}" \
    "${OVERALL}" \
    "${DB_PATH}" \
    "${NOTES_FILE}" >> "${SUMMARY_CSV}"

echo
echo "Recorded run in:"
echo "  ${SUMMARY_CSV}"

if [[ "${OPEN_DB_VIEWER}" == "1" ]]; then
    if command -v rtabmap-databaseViewer >/dev/null 2>&1; then
        echo "Opening RTAB-Map database viewer..."
        rtabmap-databaseViewer "${DB_PATH}" || true
    else
        echo "rtabmap-databaseViewer not found on PATH."
        echo "Open manually with:"
        echo "  rtabmap-databaseViewer ${DB_PATH}"
    fi
fi

echo
echo "Run complete."
