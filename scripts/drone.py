#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from gnc_controller import CONTROLLER
from create_grid import *

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = CONTROLLER()
    # Wait for FCU connection.
    drone.wait_for_connect()
    # Wait for the mode to be switched.
    drone.wait_for_start()

    # Request takeoff with an altitude of 3m.
    drone.takeoff(2)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(5)

    # Get Waypoints
    img_path = "/home/swadine/catkin_ws/src/a_star_gnc/worlds/test_world.pgm"

    start = (10, 7)
    goal = (14, 16)
    grid = A_star().create_grid(img_path)
    waypoints = A_star().prune_path(grid, start, goal)

    i = 0

    while i < len(waypoints):
        drone.set_waypoint(
            x=waypoints[i][0], y=waypoints[i][1], z = 2.0, psi=waypoints[i][2])
        rate.sleep()
        if drone.check_waypoint_reached(0.3):
            i += 1
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo("All waypoints reached landing now.")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()

#! /usr/bin/env python
# Import ROS.
# import rospy
# # Import the API.
# from gnc_controller import *
# # To print colours (optional).
# from PrintColours import *


# def main():
#     # Initializing ROS node.
#     rospy.init_node("drone_controller", anonymous=True)

#     # Create an object for the API.
#     drone = CONTROLLER()
#     # Wait for FCU connection.
#     drone.wait4connect()
#     # Wait for the mode to be switched.
#     drone.wait4start()

#     # Create local reference frame.
#     drone.initialize_local_frame()
#     # Request takeoff with an altitude of 3m.
#     drone.takeoff(3)
#     # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
#     rate = rospy.Rate(3)

#     # Specify some waypoints
#     waypoints = [[0, 0, 0], [-0.5, 0.5, 0], [-0.5, 1.5, 90], [-12.5, 1.5, 0], [-12.5, 9.5, 0]]
#     i = 0

#     while i < len(waypoints):
#         drone.set_destination(
#             x=waypoints[i][0], y=waypoints[i][1], psi=waypoints[i][2])
#         rate.sleep()
#         if drone.check_waypoint_reached():
#             i += 1
#     # Land after all waypoints is reached.
#     drone.land()
#     rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


# if __name__ == '__main__':
#     try:
#         main()
#     except KeyboardInterrupt:
#         exit()
