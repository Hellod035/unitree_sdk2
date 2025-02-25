cd ~/unitree_sdk2/build/bin
setsid ./g1_deploy_onnx_recurrent_heightmap eth0

cd ~/ws_livox
conda activate dhm
source devel/setup.bash
setsid roslaunch fast_lio g1_3dmapping.launch

cd ~/projects/catkin_ws
conda activate dhm
source /home/unitree/projects/catkin_ws/devel/setup.bash
setsid roslaunch elevation_mapping_cupy g1_simple_lidar.launch

cd ~/unitree_sdk2/height_map
conda activate dhm
setsid python map_subscriber.py