#!/bin/bash

# Default parameters
DEFAULT_LAUNCH="move_base_DWA.launch"
START_IDX=0
SPACING=6
REPEAT=10         # number of times to run each world
MAX_IDX=359
LAUNCH_FILES=()

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --launch)
        shift
        # Collect all non-option arguments as launch files
        while [[ $# -gt 0 && "$1" != --* ]]; do
            LAUNCH_FILES+=("$1")
            shift
        done
        ;;
        --start_idx)
        START_IDX="$2"
        shift
        shift
        ;;
        --spacing)
        SPACING="$2"
        shift
        shift
        ;;
        --repeat)
        REPEAT="$2"
        shift
        shift
        ;;
        *)
        echo "Unknown option: $1"
        shift
        ;;
    esac
done

# If no launch file provided, use default
if [ ${#LAUNCH_FILES[@]} -eq 0 ]; then
    echo "No launch file provided, using default: $DEFAULT_LAUNCH"
    LAUNCH_FILES+=("$DEFAULT_LAUNCH")
fi

# Loop over each launch file
for LAUNCH in "${LAUNCH_FILES[@]}"; do
    echo "Running tests with launch file: $LAUNCH"

    # Calculate number of worlds dynamically
    NUM_WORLDS=$(( (MAX_IDX - START_IDX) / SPACING + 1 ))

    for (( i=0; i<NUM_WORLDS; i++ )); do
        n=$(( START_IDX + i * SPACING ))

        for (( j=1; j<=REPEAT; j++ )); do
            echo "Running world index $n, test $j..."
            python3 run.py --world_idx $n --launch "$LAUNCH"
            sleep 5
        done
    done
done
