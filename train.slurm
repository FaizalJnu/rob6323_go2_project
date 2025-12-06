#!/bin/bash

#SBATCH --requeue
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=10
#SBATCH --mem=20GB
#SBATCH --time=02:00:00
#SBATCH --gres=gpu:1
#SBATCH --mail-type=END
#SBATCH --account=rob_gy6323-2025fa
#SBATCH --partition=g2-standard-12
# -- CHANGED: Write logs to current directory so you can actually see them
#SBATCH --output=/home/fr2471/logs/slurm_%j.out
#SBATCH --error=/home/fr2471/logs/slurm_%j.err

set -euo pipefail

# -------------------------------
# Environment Setup
# -------------------------------
PROJECT_NAME="rob6323_go2_project"
# No need for remote host rsyncing on Greene (Shared Filesystem)
LOCAL_PROJECT="${HOME}/${PROJECT_NAME}"
ISAACLAB_DIR="/scratch/$USER/IsaacLab"

# -------------------------------
# Cluster Paths
# -------------------------------
SIF_IMAGE="/scratch/$USER/isaac-lab-base.sif"
RUN_DIR="${LOCAL_PROJECT}"
PERSISTENT_CACHE_DIR="/scratch/$USER/docker-isaac-sim"
PERSISTENT_LOGS_DIR="/scratch/$USER/isaaclab/logs/${SLURM_JOB_ID}"

# Isaac Sim container paths
DOCKER_ISAACSIM_ROOT_PATH="/isaac-sim"
DOCKER_USER_HOME="/root"

# Node-local cache (Faster IO)
NODE_TMP="${SLURM_TMPDIR:-${TMPDIR:-/tmp}}"
CACHE_ROOT="${NODE_TMP}/docker-isaac-sim"

# Create temp directories
mkdir -p \
  "${CACHE_ROOT}/cache/kit" \
  "${CACHE_ROOT}/cache/ov" \
  "${CACHE_ROOT}/cache/pip" \
  "${CACHE_ROOT}/cache/glcache" \
  "${CACHE_ROOT}/cache/computecache" \
  "${CACHE_ROOT}/logs" \
  "${CACHE_ROOT}/data" \
  "${CACHE_ROOT}/documents" \
  "${CACHE_ROOT}/kit-data"

mkdir -p "${PERSISTENT_LOGS_DIR}"

# Apptainer settings
export APPTAINER_TMPDIR="${NODE_TMP}"
export APPTAINER_CACHEDIR="${NODE_TMP}/apptainer-cache"

# Forward args
export ISAAC_ARGS="$*"
echo "Job ID: $SLURM_JOB_ID"
echo "Running with args: $ISAAC_ARGS"

# -------------------------------
# Execution
# -------------------------------
singularity exec \
  --nv --containall \
  -B "${CACHE_ROOT}/kit-data:${DOCKER_ISAACSIM_ROOT_PATH}/kit/data:rw" \
  -B "${CACHE_ROOT}/cache/kit:${DOCKER_ISAACSIM_ROOT_PATH}/kit/cache:rw" \
  -B "${CACHE_ROOT}/cache/ov:${DOCKER_USER_HOME}/.cache/ov:rw" \
  -B "${CACHE_ROOT}/cache/pip:${DOCKER_USER_HOME}/.cache/pip:rw" \
  -B "${CACHE_ROOT}/cache/glcache:${DOCKER_USER_HOME}/.cache/nvidia/GLCache:rw" \
  -B "${CACHE_ROOT}/cache/computecache:${DOCKER_USER_HOME}/.nv/ComputeCache:rw" \
  -B "${CACHE_ROOT}/logs:${DOCKER_USER_HOME}/.nvidia-omniverse/logs:rw" \
  -B "${CACHE_ROOT}/data:${DOCKER_USER_HOME}/.local/share/ov/data:rw" \
  -B "${CACHE_ROOT}/documents:${DOCKER_USER_HOME}/Documents:rw" \
  -B "${ISAACLAB_DIR}:/workspace/isaaclab:rw" \
  -B "${PERSISTENT_LOGS_DIR}:/workspace/isaaclab/logs:rw" \
  -B "${RUN_DIR}:/workspace/run:rw" \
  "${SIF_IMAGE}" bash -lc '
  
set -euo pipefail

# Move into Isaac Lab
cd /workspace/isaaclab
export ISAACLAB_PATH=/workspace/isaaclab

echo "Installing project..."
# Install your project in editable mode
/isaac-sim/python.sh -m pip install -e /workspace/run/source/rob6323_go2

echo "Starting training..."
/isaac-sim/python.sh /workspace/run/scripts/rsl_rl/train.py \
  --task=Template-Rob6323-Go2-Direct-v0 \
  --headless

echo "Training complete. Looking for checkpoints..."
# Identify the latest-created/modified subdirectory
LATEST_DIR=$(ls -td /workspace/isaaclab/logs/rsl_rl/go2_flat_direct/*/ 2>/dev/null | head -n 1 || true)

if [[ -z "${LATEST_DIR:-}" ]]; then
  echo "ERROR: No subdirectories found under /workspace/isaaclab/logs/rsl_rl/go2_flat_direct" >&2
  exit 1
fi

LATEST_DIR="${LATEST_DIR%/}"
echo "Found checkpoint dir: $LATEST_DIR"

# Check if the specific model exists before running play
if [[ ! -f "${LATEST_DIR}/model_499.pt" ]]; then
    echo "WARNING: model_499.pt not found. Using the latest model found."
    # Optional: Logic to find latest .pt file could go here
fi

echo "Starting evaluation/video..."
/isaac-sim/python.sh /workspace/run/scripts/rsl_rl/play.py \
  --task=Template-Rob6323-Go2-Direct-v0 \
  --checkpoint "${LATEST_DIR}/model_499.pt" \
  --video \
  --video_length 1000 \
  --headless
'

# Sync logs back (Optional, but safe if just copying from scratch to home)
# We can just copy safely since we are on the same filesystem
echo "Copying logs to project folder..."
cp -r "${PERSISTENT_LOGS_DIR}/" "${LOCAL_PROJECT}/logs/${SLURM_JOB_ID}/"