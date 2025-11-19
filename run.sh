#!/bin/bash
# Script pour lancer fast_livo avec ou sans gdb

set -euo pipefail

usage() {
    cat <<EOF
Usage: $0 <config> [--gdb]

Prérequis: source setup_ros2_env.sh pour définir FAST_LIVO_SHARE_DIR.

Options de configuration:
  avia       - Livox AVIA
  marslvig   - MARS LVIG dataset
  hilti22    - HILTI 2022 dataset
  ntu        - NTU VIRAL dataset

Options supplémentaires:
  --gdb      - Lancer fastlivo_mapping sous gdb
EOF
}

declare -A CONFIGS=(
    ["avia"]="avia.yaml camera_pinhole.yaml"
    ["marslvig"]="MARS_LVIG.yaml camera_MARS_LVIG.yaml"
    ["hilti22"]="HILTI22.yaml camera_fisheye_HILTI22.yaml"
    ["ntu"]="NTU_VIRAL.yaml camera_NTU_VIRAL.yaml"
)

if [[ $# -lt 1 ]]; then
    usage
    exit 0
fi

CONFIG_NAME=""
USE_GDB=0

for arg in "$@"; do
    case "$arg" in
        --gdb)
            USE_GDB=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            if [[ -z "$CONFIG_NAME" ]]; then
                CONFIG_NAME="$arg"
            else
                echo "Argument inconnu: $arg"
                usage
                exit 1
            fi
            ;;
    esac
done

if [[ -z "$CONFIG_NAME" ]]; then
    usage
    exit 1
fi

if [[ -z "${CONFIGS[$CONFIG_NAME]:-}" ]]; then
    echo "❌ Configuration '$CONFIG_NAME' inconnue"
    usage
    exit 1
fi

PACKAGE_SHARE_DIR="${FAST_LIVO_SHARE_DIR:-}"
if [[ -z "$PACKAGE_SHARE_DIR" || ! -d "$PACKAGE_SHARE_DIR" ]]; then
    echo "❌ Variable FAST_LIVO_SHARE_DIR introuvable."
    echo "   Sourcez d'abord l'environnement: source setup_ros2_env.sh"
    exit 1
fi

read -r MAIN_CONFIG CAM_CONFIG <<< "${CONFIGS[$CONFIG_NAME]}"

echo ""
echo "Lancement de fastlivo_mapping"
echo "Configuration: $CONFIG_NAME"
echo "Fichiers de paramètres:"
echo "  - $PACKAGE_SHARE_DIR/config/$MAIN_CONFIG"
echo "  - $PACKAGE_SHARE_DIR/config/$CAM_CONFIG"
echo ""

CMD=(
    ros2 run fast_livo fastlivo_mapping
    --ros-args
    --params-file "$PACKAGE_SHARE_DIR/config/$MAIN_CONFIG"
    --params-file "$PACKAGE_SHARE_DIR/config/$CAM_CONFIG"
)

if [[ $USE_GDB -eq 1 ]]; then
    echo "Mode gdb activé."
    exec gdb -ex run --args "${CMD[@]}"
else
    exec "${CMD[@]}"
fi
