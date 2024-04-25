

SCENIC_FILE=$1
NUM_SCENARIOS=$2
RECORDING_FILE=`realpath $3`
MAX_TIME=$4
SCENIC_SEED=$5

scenic $SCENIC_FILE \
    --simulate \
    --2d \
    --model scenic.simulators.carla.model \
    --count $NUM_SCENARIOS \
    --seed $SCENIC_SEED \
    --param record $RECORDING_FILE
    # --time $MAX_TIME \