#!/usr/bin/env bash
# Installs the `mira_mivia` command system-wide.
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
ln -sf "$DIR/bin/mira_mivia" /usr/local/bin/mira_mivia
echo "Installed: /usr/local/bin/mira_mivia -> $DIR/bin/mira_mivia"
echo "Dependencies check:"
python3 -c "import rich" 2>/dev/null && echo "  rich       OK" || pip install rich
python3 -c "import dearpygui" 2>/dev/null && echo "  dearpygui  OK" || pip install dearpygui
python3 -c "import yaml" 2>/dev/null && echo "  pyyaml     OK" || pip install pyyaml
echo
echo "Done! Just type:  mira_mivia"
