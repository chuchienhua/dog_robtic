#!/usr/bin/env python3
import csv
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

def demo():
    markerPub = rospy.Publisher('goal_point', Marker, queue_size=1)
    goalPub = rospy.Publisher('current_goal', Pose, queue_size=1)
    rospy.init_node('demo')

    
    # Read saved points
    fileName = rospy.get_param('~saved_points_file', '')
    goals = []

    try:
        with open(fileName) as csvfile:            
            rows = csv.reader(csvfile, delimiter=',')  # read with comma as separator
            next(rows)      # Skip first
            for row in rows:                
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = float(row[1])
                goal.target_pose.pose.position.y = float(row[2])
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation.x = float(row[3])
                goal.target_pose.pose.orientation.y = float(row[4])
                goal.target_pose.pose.orientation.z = float(row[5])
                goal.target_pose.pose.orientation.w = float(row[6])
                goals.append(goal)
    except Exception as e: 
        print(e)

    # Publish all goal points
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.POINTS
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.25
    marker.scale.y = 0.25

    for goal in goals:
        marker.points.append(goal.target_pose.pose)

    markerPub.publish(marker)

    # Wait for move_base to initialize
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    # Move to all goals
    for goal in goals:    
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        # while client.get_result() != 8:
        #     markerPub.publish(marker)
        #     goalPub.publish(goal.target_pose.pose)
        #     rospy.sleep(0.1)
        wait = client.wait_for_result()
        
        rospy.loginfo('Goal Arrived, moving to next goal.')
    rospy.loginfo('All goals arrived, going home.')    

    # # Go home
    # home = MoveBaseGoal()
    # home.target_pose.header.stamp = rospy.Time.now()
    # home.target_pose.header.frame_id = 'map'
    # home.target_pose.pose.position.x = 0
    # home.target_pose.pose.position.y = 0
    # home.target_pose.pose.position.z = 0
    # home.target_pose.pose.orientation.x = 0
    # home.target_pose.pose.orientation.y = 0
    # home.target_pose.pose.orientation.z = 0
    # home.target_pose.pose.orientation.w = 1
    # client.send_goal(home)
    
    # # while client.get_result() != 8:
    # #     markerPub.publish(marker)
    # #     goalPub.publish(home.target_pose.pose)
    # #     rospy.sleep(0.1)

    # wait = client.wait_for_result()


    # rospy.loginfo('Home arrived!')    


if __name__ == '__main__':
    try:
        demo()
    except rospy.ROSInterruptException:
        pass
