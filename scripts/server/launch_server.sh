#!/bin/bash

# Check if conda is installed
CONDA_PATH=$(which conda)
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
cd $( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
python run_server.py
