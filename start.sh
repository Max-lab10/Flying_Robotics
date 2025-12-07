cp $(pwd)/simulator.launch ~/catkin_ws/src/clover/clover_simulation/launch/simulator.launch
python3 scripts/gen_world.py
roslaunch clover simulator.launch 
