#!/usr/bin/env bash
# =============================================================================
# run_sim.sh — Obstacle Avoidance Drone Simulation & Multi-Sensor Launch
# =============================================================================
set -e

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${SCRIPT_DIR}"
WS="${REPO}/ros2_ws"
ARDUPILOT="${HOME}/ardupilot"
PLUGIN="${ARDUPILOT}/ardupilot_gazebo/build"
MODELS="${REPO}/simulation/models"
WORLD="${REPO}/simulation/worlds/demo_world.sdf"
PARAMS="${REPO}/configs/sitl_params.parm"

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; MAGENTA='\033[0;35m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
die()   { echo -e "${RED}[FAIL]${NC}  $*"; exit 1; }
step()  { echo -e "${CYAN}[STEP]${NC}  $*"; }

# ── 1. Source ROS2 & Workspace ────────────────────────────────────────────────
set +u
if ! command -v ros2 &>/dev/null; then
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        source /opt/ros/humble/setup.bash
    fi
fi

if [[ -f "${WS}/install/setup.bash" ]]; then
    source "${WS}/install/setup.bash"
else
    info "Building ROS2 workspace..."
    (cd "${WS}" && colcon build --symlink-install)
    source "${WS}/install/setup.bash"
fi

if ! command -v ros2 &>/dev/null; then
    die "ROS2 not found. Please source ROS2 Humble: source /opt/ros/humble/setup.bash"
fi

# ── 2. Environment Variables for Gazebo ───────────────────────────────────────
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${PLUGIN}${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PLUGIN}${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export IGN_GAZEBO_RESOURCE_PATH="${MODELS}${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"
export GZ_SIM_RESOURCE_PATH="${MODELS}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export SDF_PATH="${MODELS}${SDF_PATH:+:${SDF_PATH}}"

# ── 3. Check ArduPilot & Display ──────────────────────────────────────────────
HAS_ARDUPILOT=true
if [[ ! -f "${PLUGIN}/libArduPilotPlugin.so" ]] || ! command -v sim_vehicle.py &>/dev/null; then
    HAS_ARDUPILOT=false
fi

GZ_FLAGS="-r"
if [[ -z "${DISPLAY:-}" ]]; then
    # Headless terminal environment
    GZ_FLAGS="-s -r"
fi

# ── PID tracking array & cleanup ──────────────────────────────────────────────
declare -a PIDS=()

cleanup() {
    echo ""
    warn "Stopping simulation processes..."
    for pid in "${PIDS[@]}"; do
        kill "${pid}" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    info "All simulation nodes stopped clean."
}
trap cleanup EXIT INT TERM

echo ""
step "=========================================================================="
step "      OBSTACLE AVOIDANCE DRONE: 3-SENSOR DETECTION & AVOIDANCE SYSTEM     "
step "=========================================================================="
info "LiDAR Sensor:   [ACTIVE] (3D PointCloud2 → 72 Horizontal Sectors)"
info "Radar Sensor:   [ACTIVE] (RF Range Noise, Beam Width & MAVLink Position)"
info "Camera Sensor:  [ACTIVE] (HSV Color Masking & Central ROI Edge Threat)"
info "Sensor Fusion:  [ACTIVE] (Multi-Modal Min Distance & Visual Weighting)"
info "Avoidance Mode: [ACTIVE] (BendyRuler Proximity Steering & Log Stream)"
step "=========================================================================="
echo ""

if [[ "${HAS_ARDUPILOT}" == "true" ]]; then
    info "ArduPilot SITL detected — Launching full SITL + Gazebo pipeline..."
    
    step "1. Starting ArduPilot SITL..."
    cd "${ARDUPILOT}"
    sim_vehicle.py \
        --vehicle=ArduCopter \
        --frame=gazebo-iris \
        --model=gazebo \
        --map \
        --console \
        --add-param-file="${PARAMS}" \
        --out=udp:127.0.0.1:14550 \
        --out=udp:127.0.0.1:14551 \
        --out=udp:127.0.0.1:14552 \
        -I0 &
    PIDS+=($!)
    sleep 6

    step "2. Starting Ignition Gazebo..."
    cd "${REPO}"
    ign gazebo --render-engine ogre ${GZ_FLAGS} "${WORLD}" &
    PIDS+=($!)
    sleep 6

    step "3. Launching ROS2 Avoidance Nodes..."
    ros2 launch obstacle_avoidance sim.launch.py &
    PIDS+=($!)

else
    info "Running ROS2 Multi-Sensor Detection & Gazebo Avoidance Pipeline..."
    
    step "1. Starting Ignition Gazebo..."
    cd "${REPO}"
    if command -v ign &>/dev/null; then
        ign gazebo --render-engine ogre ${GZ_FLAGS} "${WORLD}" &
        PIDS+=($!)
    elif command -v gz &>/dev/null; then
        gz sim ${GZ_FLAGS} "${WORLD}" &
        PIDS+=($!)
    else
        warn "Gazebo GUI not available — running ROS2 sensor nodes..."
    fi
    sleep 4

    step "2. Launching ROS2 3-Sensor Detection & Fusion Nodes..."
    ros2 launch obstacle_avoidance sim.launch.py &
    PIDS+=($!)
fi

echo ""
info "══════════════════════════════════════════════════════════════════════════"
info "  PROTOTYPE IS RUNNING! STREAMING LIVE SENSOR DETECTIONS BELOW:           "
info "══════════════════════════════════════════════════════════════════════════"
echo ""

sleep 4

# Stream live avoidance output to terminal for meeting demo
ros2 topic echo /avoidance/command &
PIDS+=($!)

wait
