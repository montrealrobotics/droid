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
# Construct the path to the 'activate' script
ACTIVATE_SCRIPT="$CONDA_DIR/etc/profile.d/conda.sh"
# Check if the 'activate' script exists
if [ ! -f "$ACTIVATE_SCRIPT" ]; then
    echo "Activate script not found at $ACTIVATE_SCRIPT"
    exit 1
fi

# Source the 'activate' script
source "$ACTIVATE_SCRIPT"

conda activate polymetis-local
pkill -9 gripper
chmod a+rw /dev/ttyUSB0
launch_gripper.py gripper=robotiq_2f gripper.comport=/dev/ttyUSB0
