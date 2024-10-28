import os
import cv2
import yaml
import matplotlib.pyplot as plt

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from geometry_msgs.msg import PoseStamped

# Paths to map files
map_path = "/home/james/assignment.pgm"
details_path = "/home/james/assignment.yaml"

# Read the PGM image file
pgm_data = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

# Function to handle mouse click events and save waypoints
waypoints = []

# Load the YAML metadata
with open(details_path, 'r') as file:
    map_metadata = yaml.safe_load(file)

def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        # Get pixel coordinates of the click
        x_pixel, y_pixel = int(event.xdata), int(event.ydata)
        print(f"Clicked at pixel: ({x_pixel}, {y_pixel})")

        # Convert pixel coordinates to world coordinates
        resolution = map_metadata['resolution']
        origin = map_metadata['origin']
        waypoint = pixel_to_world(x_pixel, y_pixel, resolution, origin)
        waypoints.append({"x": waypoint[0], "y": waypoint[1]})
        print(f"Waypoint in world coordinates: {waypoint}")

        # Plot a point on the map to visualize the clicked location
        plt.plot(x_pixel, y_pixel, 'ro')
        plt.draw()


# Function to convert pixel coordinates to world coordinates
def pixel_to_world(x_pixel, y_pixel, resolution, origin):
    x_world = x_pixel * resolution + origin[0]
    y_world = (pgm_data.shape[0] - y_pixel) * resolution + origin[1]  # y inverted
    return (x_world, y_world)

# Function to plot the map and set up the interactive interface
def plot_map(pgm_data, map_metadata):
    plt.imshow(pgm_data, cmap='gray')
    plt.title('Click on the map to select waypoints')
    plt.xlabel(f"Resolution: {map_metadata['resolution']} meters/pixel")
    plt.ylabel("Map height (pixels)")
    plt.colorbar(label='Occupancy Value')

    # Connect the click event to the 'onclick' function
    cid = plt.gcf().canvas.mpl_connect('button_press_event', onclick)
    
    # Show the plot (interactive mode)
    plt.show()

# Function to convert waypoints to ROS PoseStamped messages
def create_pose(x, y, nav,frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = nav.get_clock().now().to_msg()

    # Set position (x, y)
    pose.pose.position.x = x
    pose.pose.position.y = y

    # Set orientation (quaternion), assuming flat (z-plane) navigation with no yaw rotation.
    pose.pose.orientation.w = 1.0  # No rotation
    return pose

def nav2task():
    # Create the navigator object
    navigator = BasicNavigator()

    #wait for navigator to activate
    navigator.waitUntilNav2Active()

    poses = [create_pose(wp['x'], wp['y'], nav=navigator) for wp in waypoints]

    while rclpy.ok():
        navigator.followWaypoints(poses)

        # Monitor navigation status
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(
                    'Executing current waypoint: '
                    + str(feedback.current_waypoint + 1)
                    + '/'
                    + str(len(poses))
                )

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        navigator.lifecycleShutdown()

    # Shutdown 
    rclpy.shutdown()

def preset_nav2task():
    # predefined_waypoints = [[4.7, 3.159], [11.2, 3.309], [9.2, 6.359], [1.65, 7.609]]
    predefined_waypoints = [[4.6, 3.059], [11.25, 2.96], [1.55, 7.46]]

    # Create the navigator object
    navigator = BasicNavigator()

    #wait for navigator to activate
    navigator.waitUntilNav2Active()

    poses = [create_pose(wp[0], wp[1], nav=navigator) for wp in predefined_waypoints]

    while rclpy.ok():
        navigator.followWaypoints(poses)

        # Monitor navigation status
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(
                    'Executing current waypoint: '
                    + str(feedback.current_waypoint + 1)
                    + '/'
                    + str(len(poses))
                )

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        navigator.lifecycleShutdown()

    # Shutdown 
    rclpy.shutdown()

# Main function
def main():
    rclpy.init()  # Initialize ROS 2 here

    # Prompt user for option
    option = input("Choose an option:\n1. Interactive waypoint selection\n2. Predefined waypoints\nEnter 1 or 2: ")

    if option == '1':
        plot_map(pgm_data, map_metadata)
        nav2task()
    elif option == '2':
        preset_nav2task()
    else:
        print("Invalid option. Please enter 1 or 2.")

# Entry point of the program
if __name__ == "__main__":
    main()

# import time
# from copy import deepcopy

# from geometry_msgs.msg import PoseStamped
# from rclpy.duration import Duration
# import rclpy

# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# def main():
#     rclpy.init()

#     navigator = BasicNavigator()

#     # Inspection route, probably read in from a file for a real application
#     # from either a map or drive and repeat.
#     inspection_route = [ # simulation points
#         [3.2, 1.5],
#         # [4.0, -1.2],
#         [9.0, 3.0]]


#     # Set our demo's initial pose
#     # initial_pose = PoseStamped()
#     # initial_pose.header.frame_id = 'map'
#     # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
#     # initial_pose.pose.position.x = 3.45
#     # initial_pose.pose.position.y = 2.15
#     # initial_pose.pose.orientation.z = 1.0
#     # initial_pose.pose.orientation.w = 0.0
#     # navigator.setInitialPose(initial_pose)

#     # Wait for navigation to fully activate
#     navigator.waitUntilNav2Active()

#     while rclpy.ok():

#         # Send our route
#         inspection_points = []
#         inspection_pose = PoseStamped()
#         inspection_pose.header.frame_id = 'map'
#         inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
#         inspection_pose.pose.orientation.z = 1.0
#         inspection_pose.pose.orientation.w = 0.0
#         for pt in inspection_route:
#             inspection_pose.pose.position.x = pt[0]
#             inspection_pose.pose.position.y = pt[1]
#             inspection_points.append(deepcopy(inspection_pose))
#         nav_start = navigator.get_clock().now()
#         navigator.followWaypoints(inspection_points)

#         # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
#         # Simply print the current waypoint ID for the demonstation
#         i = 0
#         while not navigator.isTaskComplete():
#             i = i + 1
#             feedback = navigator.getFeedback()
#             if feedback and i % 5 == 0:
#                 print('Executing current waypoint: ' +
#                     str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

#         result = navigator.getResult()
#         if result == TaskResult.SUCCEEDED:
#             print('Inspection of shelves complete! Returning to start...')
#         elif result == TaskResult.CANCELED:
#             print('Inspection of shelving was canceled. Returning to start...')
#             exit(1)
#         elif result == TaskResult.FAILED:
#             print('Inspection of shelving failed! Returning to start...')

#         # go back to start
#         # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
#         # navigator.goToPose(initial_pose)
#         while not navigator.isTaskComplete:
#             pass


# if __name__ == '__main__':
#     main()