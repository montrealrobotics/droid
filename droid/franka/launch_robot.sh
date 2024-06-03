#!/bin/bash

add_to_path_if_conda_exists(){
        if [ -x "$1/bin/conda" ]; then
                export PATH="$1/bin:$PATH"
        fi
}

add_to_path_if_conda_exists "$HOME/anaconda3"
add_to_path_if_conda_exists "$HOME/miniconda3"

# Check if conda is installed
CONDA_PATH=$(which conda)

echo $CONDA_PATH
if [ -z "$CONDA_PATH" ]; then
    echo "Conda is not installed or not in the PATH."
    exit 1
fi

# Extract the base directory of the conda installation
CONDA_DIR=$(dirname $(dirname "$CONDA_PATH"))
echo $CONDA_DIR
# Construct the path to the 'activate' script
ACTIVATE_SCRIPT="$CONDA_DIR/etc/profile.d/conda.sh"
echo $ACTIVATE_SCRIPT
# Check if the 'activate' script exists
if [ ! -f "$ACTIVATE_SCRIPT" ]; then
    echo "Activate script not found at $ACTIVATE_SCRIPT"
    exit 1
fi

# Source the 'activate' script
source "$ACTIVATE_SCRIPT"

conda activate polymetis-local
pkill -9 run_server
pkill -9 franka_panda_cl
launch_robot.py robot_client=franka_hardware
