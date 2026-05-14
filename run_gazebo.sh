#!/bin/bash
# Базовый запуск Gazebo Harmonic для world/model smoke-проверок.

set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${BASE_DIR}/.." && pwd)"
USER_HOME="${HOME:-/home/amitryd}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${WORKSPACE_DIR}/ardupilot_gazebo}"
GZ_PHYSICS_ENGINE="${GZ_PHYSICS_ENGINE:-}"
CANONICAL_WORLD="${BASE_DIR}/worlds/random_terrain_ardupilot.sdf"

append_unique_dir() {
    local candidate="$1"
    shift
    local -n target_ref="$1"
    [ -d "${candidate}" ] || return 0
    local existing
    for existing in "${target_ref[@]}"; do
        [ "${existing}" = "${candidate}" ] && return 0
    done
    target_ref+=("${candidate}")
}

RESOURCE_PATHS=()
if [ -n "${GZ_SIM_RESOURCE_PATH:-}" ]; then
    while IFS= read -r path; do
        [ -n "${path}" ] && RESOURCE_PATHS+=("${path}")
    done < <(printf '%s' "${GZ_SIM_RESOURCE_PATH}" | tr ':' '\n')
fi
append_unique_dir "${BASE_DIR}/worlds" RESOURCE_PATHS
append_unique_dir "${BASE_DIR}/models" RESOURCE_PATHS
export GZ_SIM_RESOURCE_PATH="$(printf '%s\n' "${RESOURCE_PATHS[@]}" | paste -sd: -)"

PLUGIN_PATHS=()
if [ -n "${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" ]; then
    while IFS= read -r path; do
        [ -z "${path}" ] && continue
        case "${path}" in
            *build_local*) ;;
            *) PLUGIN_PATHS+=("${path}") ;;
        esac
    done < <(printf '%s' "${GZ_SIM_SYSTEM_PLUGIN_PATH}" | tr ':' '\n')
fi
append_unique_dir "${ARDUPILOT_GAZEBO_DIR}/build" PLUGIN_PATHS
append_unique_dir "${USER_HOME}/ardupilot_gazebo/build" PLUGIN_PATHS

if [ ${#PLUGIN_PATHS[@]} -gt 0 ]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$(printf '%s\n' "${PLUGIN_PATHS[@]}" | paste -sd: -)"
fi

if [ -n "${1:-}" ]; then
    WORLD_FILE="$1"
    echo "⚠️  Override: using world ${WORLD_FILE}"
else
    WORLD_FILE="${CANONICAL_WORLD}"
fi

if [ ! -f "${WORLD_FILE}" ]; then
    echo "❌ World file not found: ${WORLD_FILE}"
    exit 1
fi

echo "🔍 Gazebo Harmonic"
echo "📁 Repo directory: ${BASE_DIR}"
echo "🌍 World file    : ${WORLD_FILE}"
echo "⚙️  Physics      : ${GZ_PHYSICS_ENGINE:-world default}"
echo "🔧 GZ_SIM_RESOURCE_PATH     : ${GZ_SIM_RESOURCE_PATH}"
echo "🔧 GZ_SIM_SYSTEM_PLUGIN_PATH: ${GZ_SIM_SYSTEM_PLUGIN_PATH:-<not set>}"
echo ""
echo "🚀 Запуск Gazebo..."

if [ -n "${GZ_PHYSICS_ENGINE}" ]; then
    exec gz sim -v4 -r --physics-engine "${GZ_PHYSICS_ENGINE}" "${WORLD_FILE}"
fi

exec gz sim -v4 -r "${WORLD_FILE}"
