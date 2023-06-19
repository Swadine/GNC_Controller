A basic but apt Guided Navigation Controller I developed for UAV's in a known environment. The map of the environment is made available using lib_gazebo 2D map plugin that generates a 2D grid map of the environment. Then I implement Heuristic A-Star search on the map and make the UAV plan through the environment and reach the goal location.
All the low level control is handled using MAVROS.

Refer to [Ardupilot_SITL_20_04.md](https://github.com/Swadine/ROS_Gazebo/blob/main/Ardupilot_SITL_20_04.md) to read up about the entire setup of a ROS software stack for motion planning and controls.
