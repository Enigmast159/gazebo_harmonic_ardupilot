#!/bin/bash
# Launch ArduPilot SITL for the canonical Gazebo Iris model.

set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${BASE_DIR}/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
SIM_VEHICLE="${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
SITL_PARAM_FILE="${SITL_PARAM_FILE:-${BASE_DIR}/config/ardupilot/terrain_gazebo_iris.parm}"
SITL_USE_EXTRA_PARAMS="${SITL_USE_EXTRA_PARAMS:-1}"
SITL_WIPE_EEPROM="${SITL_WIPE_EEPROM:-1}"
SITL_WITH_MAP="${SITL_WITH_MAP:-0}"
SITL_WITH_CONSOLE="${SITL_WITH_CONSOLE:-auto}"
SITL_USE_MAVPROXY="${SITL_USE_MAVPROXY:-0}"
SITL_REBUILD="${SITL_REBUILD:-0}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib}"

if [ ! -x "${SIM_VEHICLE}" ]; then
    echo "❌ sim_vehicle.py not found or not executable: ${SIM_VEHICLE}"
    echo "   Set ARDUPILOT_DIR to your ArduPilot source directory."
    exit 1
fi

cd "${ARDUPILOT_DIR}"

ARGS=(
    -v ArduCopter
    -f gazebo-iris
    --model JSON
)

case "${SITL_WITH_CONSOLE}" in
    1|true|TRUE|yes|YES)
        ARGS+=(--console)
        ;;
    0|false|FALSE|no|NO)
        ;;
    auto|AUTO)
        if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
            ARGS+=(--console)
        fi
        ;;
    *)
        echo "❌ Unsupported SITL_WITH_CONSOLE value: ${SITL_WITH_CONSOLE}"
        echo "   Use one of: auto, 0, 1"
        exit 1
        ;;
esac

if [ "${SITL_WITH_MAP}" = "1" ]; then
    echo "⚠️  SITL_WITH_MAP=1 enables MAVProxy map. If cv2/NumPy errors appear, run with SITL_WITH_MAP=0."
    ARGS+=(--map)
fi

if [ "${SITL_USE_MAVPROXY}" != "1" ]; then
    ARGS+=(--no-mavproxy)
fi

if [ "${SITL_REBUILD}" != "1" ]; then
    ARGS+=(--no-rebuild)
fi

if [ "${SITL_WIPE_EEPROM}" = "1" ]; then
    ARGS+=(--wipe-eeprom)
fi

if [ "${SITL_USE_EXTRA_PARAMS}" = "1" ] && [ -f "${SITL_PARAM_FILE}" ]; then
    echo "📋 Использую дополнительные параметры: ${SITL_PARAM_FILE}"
    ARGS+=(--add-param-file "${SITL_PARAM_FILE}")
else
    echo "📋 Использую штатные параметры ArduPilot: copter.parm + gazebo-iris.parm"
    echo "   Для запуска без terrain-specific параметров: SITL_USE_EXTRA_PARAMS=0 ./run_ardupilot_sitl.sh"
fi

if [ "${SITL_USE_MAVPROXY}" = "1" ]; then
    echo "📋 После готовности ArduPilot используйте в MAVProxy:"
    echo "   mode guided"
    echo "   arm throttle"
    echo "   takeoff 10"
    echo ""
else
    echo "📋 Режим по умолчанию: без MAVProxy и без rebuild."
    echo "   Это соответствует ROS 2 mission_controller и проектному quickstart."
    echo "   Для старого интерактивного режима запустите: SITL_USE_MAVPROXY=1 SITL_REBUILD=1 ./run_ardupilot_sitl.sh"
    echo ""
fi

exec "${SIM_VEHICLE}" "${ARGS[@]}" "$@"
