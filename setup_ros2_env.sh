#!/bin/bash
# Script pour configurer l'environnement ROS 2 après conan build

# Couleurs pour le terminal
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

usage() {
    cat <<EOF
Usage: source $(basename "$0") [--build <repertoire>] [--type <Release|Debug>]

Sans argument, le script sélectionne automatiquement le dernier build Conan.
EOF
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "⚠️  Merci d'utiliser 'source $0' pour conserver l'environnement."
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REQUESTED_BUILD_DIR=""
REQUESTED_TYPE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            shift
            REQUESTED_BUILD_DIR="${1:-}"
            ;;
        --type)
            shift
            REQUESTED_TYPE="${1:-}"
            ;;
        -h|--help)
            usage
            return 0 2>/dev/null || exit 0
            ;;
        *)
            echo "Option inconnue: $1"
            usage
            return 1 2>/dev/null || exit 1
            ;;
    esac
    shift
done

select_build_dir() {
    local root="$1"
    local prefix="${2:-}"
    local latest_dir=""
    local latest_mtime=0
    while IFS= read -r -d '' dir; do
        local name
        name="$(basename "$dir")"
        if [[ -n "$prefix" && "$name" != "${prefix}-"* ]]; then
            continue
        fi
        local mtime
        mtime="$(stat -c %Y "$dir")"
        if (( mtime > latest_mtime )); then
            latest_mtime=$mtime
            latest_dir="$dir"
        fi
    done < <(find "$root" -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)
    echo "$latest_dir"
}

BUILD_DIR=""
if [[ -n "$REQUESTED_BUILD_DIR" ]]; then
    BUILD_DIR="$REQUESTED_BUILD_DIR"
elif [[ -n "$REQUESTED_TYPE" ]]; then
    lower_type="$(echo "$REQUESTED_TYPE" | tr '[:upper:]' '[:lower:]')"
    BUILD_DIR="$(select_build_dir "${SCRIPT_DIR}/build" "$lower_type")"
fi

if [[ -z "$BUILD_DIR" ]]; then
    BUILD_DIR="$(select_build_dir "${SCRIPT_DIR}/build")"
fi

if [[ -z "$BUILD_DIR" || ! -d "$BUILD_DIR" ]]; then
    echo -e "${YELLOW}⚠️  Aucun build Conan trouvé. Lancez d'abord 'conan install' puis 'conan build'.${NC}"
    return 1 2>/dev/null || exit 1
fi

BUILD_DIR="$(cd "$BUILD_DIR" && pwd)"
GEN_DIR="$BUILD_DIR/generators"
CONAN_ENV_SCRIPT="$(ls "$GEN_DIR"/conanrunenv-*.sh 2>/dev/null | head -n1 || true)"

if [[ -z "$CONAN_ENV_SCRIPT" ]]; then
    echo -e "${YELLOW}⚠️  Impossible de trouver le script conanrunenv dans ${GEN_DIR}.${NC}"
    return 1 2>/dev/null || exit 1
fi

# Source l'environnement ROS 2
if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓ ROS 2 Jazzy sourcé${NC}"
else
    echo -e "${YELLOW}⚠️  /opt/ros/jazzy/setup.bash non trouvé${NC}"
fi

# Source l'environnement Conan généré
# shellcheck source=/dev/null
source "$CONAN_ENV_SCRIPT"
echo -e "${GREEN}✓ Environnement Conan sourcé (${CONAN_ENV_SCRIPT})${NC}"

# Configurer les variables d'environnement ROS 2
export AMENT_PREFIX_PATH="$BUILD_DIR/ament_cmake_index:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$BUILD_DIR/ament_cmake_index:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$BUILD_DIR:${LD_LIBRARY_PATH:-}"
export FAST_LIVO_BUILD_DIR="$BUILD_DIR"
export FAST_LIVO_SHARE_DIR="$BUILD_DIR/ament_cmake_index/share/fast_livo"

echo -e "${GREEN}✓ Variables d'environnement configurées pour:${NC} $BUILD_DIR"
echo ""
echo "Vous pouvez maintenant utiliser:"
echo "  • ros2 run fast_livo fastlivo_mapping"
echo "  • ros2 launch fast_livo mapping_ouster_ntu.launch"
echo ""
echo -e "${YELLOW}Note: Pour activer les core dumps en cas de crash:${NC}"
echo "  source enable_coredump.sh"
echo ""
