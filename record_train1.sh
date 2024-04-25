# capture data for 100 rounds for each scenario file

# CARLA_PATH="/home/tarang/Code/carla994" # path to the folder where you exracted carla - Update This
CARLA_PATH="F:/CARLA/CARLA_0.9.15/WindowsNoEditor" # path to the folder where you exracted carla - Update This
NUM_SCENARIOS=1 # number of scenarios to record
NUM_REAL_SCENARIOS=10
NUM_TL_SL_SCENARIOS=5

SCENARIO_MAX_TIME=20
SCENIC_SEED=50
# Initialize carla
$CARLA_PATH/CarlaUE4.sh -opengl &
PID=$!
echo "Carla PID=$PID"
sleep 4 # sleep to ensure Carla has opened up and is initialized

# Run Scenic Scenarios

echo "Running Scenic for $NUM_SCENARIOS Car Overtaking scenarios"
./scripts/run_scenic1.sh overtake.scenic $NUM_SCENARIOS car_overtake_recordings $SCENARIO_MAX_TIME $SCENIC_SEED

echo "Done Scenic task"

# convert Scenic's output (Carla logs to CSV/Parquet files)
python recorder2.py

# # Record Traffic Light and SpeedLimit violations
# echo "Running CARLA API task for $NUM_TL_SL_SCENARIOS TL and SL violation scenarios"
# python carla_python_api_recorder.py -s tl_sl -n $NUM_TL_SL_SCENARIOS

# echo "Running CARLA API task for $NUM_REAL_SCENARIOS nominal scenarios"
# python carla_python_api_recorder.py -s nominal -n $NUM_REAL_SCENARIOS

echo "Done Python task"

pkill -f "CarlaUE4" 
