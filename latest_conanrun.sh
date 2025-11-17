#!/bin/bash
# Source ce script pour activer l'environnement Conan du dernier build

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "⚠️  Utilisez 'source latest_conanrun.sh' afin d'exporter les variables."
fi

if [[ ! -d "${SCRIPT_DIR}/build" ]]; then
    echo "⚠️  Aucun dossier build/ trouvé. Lancez d'abord 'conan install' & 'conan build'."
    return 1 2>/dev/null || exit 1
fi

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/setup_ros2_env.sh"
