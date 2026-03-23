#!/usr/bin/env bash
set -euo pipefail

cd /workdir

export WEST_TOPDIR="${WEST_TOPDIR:-/workdir}"
export ZEPHYR_BASE="${ZEPHYR_BASE:-/workdir/external/zephyr}"
VENV_DIR="/opt/zephyr-venv"

git config --global --add safe.directory '*' || true
git config --system --add safe.directory '*' || true
git config --global --add safe.directory /workdir || true
git config --global --add safe.directory /workdir/external/zephyr || true
git config --system --add safe.directory /workdir || true
git config --system --add safe.directory /workdir/external/zephyr || true

if [ ! -x "${VENV_DIR}/bin/python" ]; then
  rm -rf "${VENV_DIR}"
  python3 -m venv "${VENV_DIR}"
fi

"${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
if [ -f "${ZEPHYR_BASE}/scripts/requirements.txt" ]; then
  "${VENV_DIR}/bin/pip" install -r "${ZEPHYR_BASE}/scripts/requirements.txt"
fi
"${VENV_DIR}/bin/pip" install \
  west \
  dtsh \
  patool \
  requests \
  semver \
  tqdm \
  jsonschema \
  pyserial \
  catkin-pkg==1.0.0 \
  colcon-argcomplete==0.3.3 \
  colcon-bash==0.5.0 \
  colcon-cd==0.1.1 \
  colcon-cmake==0.2.29 \
  colcon-common-extensions==0.3.0 \
  colcon-core==0.19.0 \
  colcon-defaults==0.2.9 \
  colcon-devtools==0.3.0 \
  colcon-library-path==0.2.1 \
  colcon-metadata==0.2.5 \
  colcon-notification==0.3.0 \
  colcon-output==0.2.13 \
  colcon-package-information==0.4.0 \
  colcon-package-selection==0.2.10 \
  colcon-parallel-executor==0.3.0 \
  colcon-pkg-config==0.1.0 \
  colcon-powershell==0.4.0 \
  colcon-python-setup-py==0.2.9 \
  colcon-recursive-crawl==0.2.3 \
  colcon-ros==0.5.0 \
  colcon-test-result==0.3.8 \
  colcon-zsh==0.5.0 \
  empy==4.2 \
  lark==1.2.2

if [ ! -d .west ]; then
  "${VENV_DIR}/bin/west" init -l west-manifest
fi

"${VENV_DIR}/bin/west" update
"${VENV_DIR}/bin/west" zephyr-export || true
