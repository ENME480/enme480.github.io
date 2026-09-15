#!/usr/bin/env bash
# ENME480 setup check.
# Reads only - installs and changes nothing.
# Usage:  curl -fsSL https://enme480.github.io/assets/check_setup.sh | bash

ok=0; bad=0
chk() { if eval "$2" >/dev/null 2>&1; then printf '  \033[32mOK  \033[0m %s\n' "$1"; ok=$((ok+1));
        else printf '  \033[31mFAIL\033[0m %s\n' "$1"; bad=$((bad+1)); fi; }

echo
echo "ENME480 setup check"

echo
echo "Build tools"
chk "curl"                  "command -v curl"
chk "wget"                  "command -v wget"
chk "git"                   "command -v git"
chk "gcc, g++ and make"     "command -v gcc && command -v g++ && command -v make"
chk "cmake"                 "command -v cmake"

echo
echo "Python"
chk "python3"               "command -v python3"
chk "pip3"                  "command -v pip3"
chk "python runs python 3"  "command -v python && python --version 2>&1 | grep -q 'Python 3'"
chk "venv module"           "python3 -c 'import venv'"

echo
echo "Docker"
chk "docker is real Docker, not wmdocker" "docker --version 2>/dev/null | grep -q '^Docker version'"
chk "wmdocker is not installed"           "! dpkg -l docker wmdocker 2>/dev/null | grep -q '^ii'"
chk "docker compose v2"                   "docker compose version 2>/dev/null | grep -qi 'compose version'"
chk "containerd.io"                       "dpkg -l containerd.io 2>/dev/null | grep -q '^ii'"
chk "buildx plugin"                       "docker buildx version"
chk "your user is in the docker group"    "id -nG | tr ' ' '\n' | grep -qx docker"
chk "docker runs without sudo"            "docker info"

echo
echo "Package system"
chk "no half-installed packages"          "[ -z \"\$(dpkg --audit 2>/dev/null)\" ]"

echo
if [ "$bad" -eq 0 ]; then
  printf '\033[32mAll %s checks passed.\033[0m Your setup is complete.\n\n' "$ok"
else
  printf '\033[31m%s passed, %s FAILED.\033[0m\n' "$ok" "$bad"
  printf 'Fix these before going further:\n'
  printf '  https://enme480.github.io/ubuntu-setup/#repair-fixing-a-partly-broken-install\n\n'
fi
