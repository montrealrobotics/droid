#!/bin/bash

set -e

# activate conda
source ~/miniconda3/bin/activate base
conda activate robot

# run user command
exec "$@"
