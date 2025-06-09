#!/bin/bash

# Commands to run
commands=(
  "cd /home/rakshithram/aroi/ndt_localizer_vis && source devel/setup.bash && roslaunch ndt_localizer ndt_localizer.launch"
  "cd /home/rakshithram/aroi/lidar_obs_det_pca && source devel/setup.bash && roslaunch lidar_obstacle_detector obstacle_det.launch"
  "cd /home/rakshithram/aroi/pc_transform && source devel/setup.bash && roslaunch transform transform.launch"
  "cd /home/rakshithram/aroi/scripts && python3 waypoints_pub.py"
  "cd /home/rakshithram/aroi/scripts && python3 roi.py"
  "cd /home/rakshithram/aroi/scripts && python3 roi_visualizer.py"
  "cd /home/rakshithram/aroi/scripts && python3 obstacle_boxes.py"
  "cd /home/rakshithram && rosbag play testbed_out_filtered.bag"
)

# Sleep durations corresponding to the commands
sleep_durations=(2 2 0.5 0.5 0.5 2)


# Execute commands in new terminals
for i in "${!commands[@]}"; do
  gnome-terminal -- bash -c "${commands[i]}; exec bash"
  sleep "${sleep_durations[i]}"
done