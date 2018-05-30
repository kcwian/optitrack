# imu_low_cost
## driver for low cost imu

1. Clone repository to catkin_ws/src 

2. Open terminal
 - start "roscore"

3. Open new tab
 - go to catkin_ws type "catkin_make"
 - type "sudo su"
 - type "source devel/setup.bash"
 - type "rosrun low_cost_imu lowCostImu"

4. Open new tab 
 - type "source devel/setup.bash"
 - type "rostopic list"
 - type "rostopic echo lowCostImu/imu"  
