import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # --- CONFIGURATION: EDIT YOUR POINTS HERE ---
    # Look at RViz grid to find X and Y (in meters)
    # Origin (Start) is usually 0.0, 0.0
    
    locations = {
        '1': {'x': 1.5, 'y': 0.0, 'w': 1.0},  # Point 1
        '2': {'x': 1.5, 'y': 1.5, 'w': 1.0},  # Point 2
        '3': {'x': 0.0, 'y': 1.5, 'w': 1.0}   # Point 3
    }
    
    home_pose = {'x': 0.0, 'y': 0.0, 'w': 1.0} 
    # --------------------------------------------

    # Set initial pose (Assuming we start at 0,0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to fully start
    navigator.waitUntilNav2Active()

    while True:
        user_input = input("Enter point number (1, 2, 3) or 'q' to quit: ")
        
        if user_input == 'q':
            break
            
        if user_input in locations:
            target = locations[user_input]
            
            # 1. Define the Goal
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = target['x']
            goal_pose.pose.position.y = target['y']
            goal_pose.pose.orientation.w = target['w']

            # 2. Go to Goal
            print(f"Moving to Point {user_input}...")
            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete():
                pass # Wait while moving

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("Goal Reached! Waiting 15 seconds...")
                time.sleep(15) # WAIT FOR 15 SECONDS
                
                # 3. Return Home
                print("Returning Home...")
                goal_home = PoseStamped()
                goal_home.header.frame_id = 'map'
                goal_home.header.stamp = navigator.get_clock().now().to_msg()
                goal_home.pose.position.x = home_pose['x']
                goal_home.pose.position.y = home_pose['y']
                goal_home.pose.orientation.w = home_pose['w']
                
                navigator.goToPose(goal_home)
                while not navigator.isTaskComplete():
                    pass
                print("Returned Home.")
                
            else:
                print("Failed to reach goal.")
        else:
            print("Unknown point.")

    exit(0)

if __name__ == '__main__':
    main()
