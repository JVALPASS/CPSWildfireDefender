#!/usr/bin/env python
# waypoint nuovi ma funzionali con >=1
import rospy
import time
import tf
from std_msgs.msg import *
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix
import geonav_transform.geonav_conversions as gc
from move_base_msgs.msg import MoveBaseActionResult
from visualization_msgs.msg import *
from darknet_ros_msgs.msg import ObjectCount
from actionlib_msgs.msg import GoalID
"""
user:~$ rosmsg show geometry_msgs/PoseWithCovarianceStamped                                                                                                               
std_msgs/Header header                                                                                                                                                    
  uint32 seq                                                                                                                                                              
  time stamp                                                                                                                                                              
  string frame_id                                                                                                                                                         
geometry_msgs/PoseWithCovariance pose                                                                                                                                     
  geometry_msgs/Pose pose                                                                                                                                                 
    geometry_msgs/Point position                                                                                                                                          
      float64 x                                                                                                                                                           
      float64 y                                                                                                                                                           
      float64 z                                                                                                                                                           
    geometry_msgs/Quaternion orientation                                                                                                                                  
      float64 x                                                                                                                                                           
      float64 y                                                                                                                                                           
      float64 z                                                                                                                                                           
      float64 w                                                                                                                                                           
  float64[36] covariance 
"""
class ProducePath(object):
  def __init__(self):
    self.nav_origin = NavSatFix()
    self.nav_goal = NavSatFix()
    self.odom = Odometry()
    self.position_container = Point()
    self.quaternion_container = Quaternion()
    self.detected =  False
    self.alert = False
    self.path_approach = Int8()
    self.path_approach.data = -1
    self.finish = False
    self.temperature = 50.0
    self.base_distance = 1.0
    self.attempts = 3
  def wait_for_gps_origin(self):
    global latitude_origin
    global longitude_orgin
    self.nav_origin = rospy.wait_for_message('/navsat/fix', NavSatFix)
    latitude_origin = self.nav_origin.latitude
    longitude_orgin = self.nav_origin.longitude
    rospy.loginfo("latitude " + str(latitude_origin))
    rospy.loginfo("longitude " + str(longitude_orgin))

  def wait_for_gps_goal(self):
    global latitude_goal
    global longitude_goal
    rospy.loginfo("Wait for GPS Goal Latitude and Longitude")
    self.nav_goal = rospy.wait_for_message('/gps_goal_fix', NavSatFix)
    latitude_goal = self.nav_goal.latitude
    longitude_goal = self.nav_goal.longitude
    rospy.loginfo("latitude " + str(latitude_goal))
    rospy.loginfo("longitude " + str(longitude_goal))
    self.get_xy_based_on_lat_long()

  def wait_for_gps_end(self):
    rospy.loginfo("Wait for GPS Goal end")
    result = rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
    rospy.loginfo("result " + str(result))


  def wait_for_detection(self):
    global waypoints
    pub_reset_wp = rospy.Publisher('/path_reset', Empty, queue_size=1)
    pub_cancel_wp = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    pub_temperature_update = rospy.Publisher('/update/temperature', Float32, queue_size=1)
    pub_fire_detected = rospy.Publisher('/detected_fire', String, queue_size=1)

    rospy.loginfo("Wait for Detection")
    count_detected = 0
    object_count = ObjectCount()
    i = 0
    while(not self.alert and i < len(waypoints)):
      rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
      object_count = rospy.wait_for_message('darknet_ros/found_object', ObjectCount)
      count_detected = object_count.count
      if count_detected >= 1:
        self.path_approach.data = i
        rospy.loginfo("ricevuto path to continue: " + str(self.path_approach))
        break
      i = i + 1

    if not (i < len(waypoints)):
      rospy.loginfo("No Detected Fire")
      self.finish = True

    if not self.finish:
      cancel_msg = GoalID()
      end_command = Empty()
      rospy.loginfo("Stop Path")
      pub_reset_wp.publish(end_command)
      pub_cancel_wp.publish(cancel_msg)
      pub_temperature_update.publish(self.temperature)
      pub_fire_detected.publish("True")
      self.detected = True

  def select_waypoint(self, val):
    #swithc case
    wp = PoseWithCovarianceStamped()
    rospy.loginfo(self.path_approach)
    if self.path_approach.data == 0:
      wp = self.produce_waypoint_low(val)
    elif self.path_approach.data == 1:
      wp = self.produce_waypoint_right(val)
    elif self.path_approach.data == 2:
      wp = self.produce_waypoint_middle_right_low(val)
    elif self.path_approach.data == 3:
      wp = self.produce_waypoint_left(val)
    elif self.path_approach.data == 4:
      wp = self.produce_waypoint_left_low(val)
    elif self.path_approach.data == 5:
      wp = self.produce_waypoint_up(val)
    elif self.path_approach.data == 6:
      wp = self.produce_waypoint_up_left(val)
    elif self.path_approach.data == 7:
      wp = self.produce_waypoint_up_right(val)
    elif self.path_approach.data == -1:
      rospy.loginfo("No fire detected")
    return wp

  def alert_approach(self):
      pub_reset_wp = rospy.Publisher('/path_reset', Empty, queue_size=1)
      pub_cancel_wp = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
      pub_temperature_update = rospy.Publisher('/update/temperature', Float32, queue_size=1)

      distance = 999999.99999 #indicate infinite value
      while(distance > 2.5):
        marker_array_detected = MarkerArray()
        try:
          marker_array_detected = rospy.wait_for_message('/darknet_ros_3d/markers', MarkerArray)
        except rospy.ROSException as e:
          rospy.loginfo("exception in alert_thread: "+ str(e))
          continue
        marker_array = marker_array_detected.markers
        for marker in marker_array:
          distance = marker.pose.position.x
        #rospy.sleep(1)
        time.sleep(0.5)
      cancel_msg = GoalID()
      end_command = Empty()

      rospy.loginfo("ALERT!!!!!!!!!!!!!!!!!!!!!!!!!")
      pub_reset_wp.publish(end_command)
      pub_cancel_wp.publish(cancel_msg)
      pub_temperature_update.publish(self.temperature + 100.0)
      self.detected = True
      self.alert = True
  def continue_approach(self):
      rate = rospy.Rate(10) # 10hz 
      pub_wp = rospy.Publisher('/my_jackal_waypoints', PoseWithCovarianceStamped, queue_size=1)
      pub_init_wp = rospy.Publisher('/path_ready', Empty, queue_size=1)
      pub_temperature_update = rospy.Publisher('/update/temperature', Float32, queue_size=1)
      i = 0.2
      while not self.alert:
        waypoint = self.select_waypoint(i)
        while not rospy.is_shutdown() and not self.alert:
          connections = pub_wp.get_num_connections()
          if connections > 0 and not self.alert:
            pub_wp.publish(waypoint)
            break
          rospy.loginfo("Waiting for /my_jackal_waypoints topiiiic")
          rate.sleep()
        start_command = Empty()
        while not rospy.is_shutdown() and not self.alert:
          connections = pub_init_wp.get_num_connections()
          if connections > 0 and not self.alert:
            pub_init_wp.publish(start_command)
            rospy.loginfo("Sent waypoint list execution command")
            break
          rospy.loginfo("Waiting for /path_ready topic")
          rate.sleep()
        if not self.alert:
          rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
          i = i + 0.2
          pub_temperature_update.publish(self.temperature + i * 100)
        
  def get_xy_based_on_lat_long(self):
    global latitude_origin
    global longitude_orgin
    global latitude_goal
    global longitude_goal
    latitude_origin = 49.900000001374615
    longitude_orgin = 8.900000056652528
    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin
    name = "/map"

    xg2, yg2 = gc.ll2xy(latitude_goal, longitude_goal, latitude_origin, longitude_orgin)

    rospy.loginfo("#########  "+name+"  ###########")
    rospy.loginfo("LAT COORDINATES ==>"+str(latitude_goal)+","+str(longitude_goal))
    rospy.loginfo("COORDINATES XYZ ==>"+str(xg2)+","+str(yg2))

    self.produce_gps_waypoint(xg2,yg2)

  def produce_gps_waypoint(self,xg2,yg2):
    global waypoints
    waypoints = []
    #original postion
    my_wp = PoseWithCovarianceStamped()
    my_wp.header.stamp = rospy.Time.now()
    my_wp.header.frame_id = "/map"   

    roll = 0
    pitch = 0
    yaw = 0.7
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw) 
    #type(pose) = geometry_msgs.msg.Pose
    my_wp.pose.pose.position.x = xg2
    my_wp.pose.pose.position.y = yg2
    my_wp.pose.pose.orientation.x = quaternion[0]
    my_wp.pose.pose.orientation.y = quaternion[1]
    my_wp.pose.pose.orientation.z = quaternion[2]
    my_wp.pose.pose.orientation.w = quaternion[3]
    waypoints.append(my_wp)

  def wait_for_position(self):
      self.odom = rospy.wait_for_message('/odometry/filtered/global', Odometry)
      self.position_container = self.odom.pose.pose.position
      rospy.loginfo("position.x " + str(self.position_container.x))
      rospy.loginfo("position.y " + str(self.position_container.y))
      rospy.loginfo("position.z " + str(self.position_container.z))
      self.quaternion_container = self.odom.pose.pose.orientation
      rospy.loginfo("orientation.x " + str(self.quaternion_container.x))
      rospy.loginfo("orientation.y " + str(self.quaternion_container.y))
      rospy.loginfo("orientation.z " + str(self.quaternion_container.z))
      rospy.loginfo("orientation.w " + str(self.quaternion_container.w))
      #rospy.loginfo(latitude)
      #rospy.loginfo(longitude)
  def produce_waypoint_low(self, move):
      # in basso
      my_wp = PoseWithCovarianceStamped()
      my_wp.header.stamp = rospy.Time.now()
      my_wp.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp.pose.pose.position.x = self.position_container.x + self.base_distance + move
      my_wp.pose.pose.position.y = self.position_container.y
      my_wp.pose.pose.position.z = self.position_container.z
      my_wp.pose.pose.orientation.x = 0.0
      my_wp.pose.pose.orientation.y = 0.0
      my_wp.pose.pose.orientation.z = 0.0
      my_wp.pose.pose.orientation.w = 1.0
      return my_wp
  def produce_waypoint_right(self, move):
      # verso destra
      my_wp_2 = PoseWithCovarianceStamped()
      my_wp_2.header.stamp = rospy.Time.now()
      my_wp_2.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp_2.pose.pose.position.x = self.position_container.x
      my_wp_2.pose.pose.position.y = self.position_container.y + self.base_distance + move
      my_wp_2.pose.pose.position.z = self.position_container.z
      my_wp_2.pose.pose.orientation.x = 0.0
      my_wp_2.pose.pose.orientation.y = 0.0
      my_wp_2.pose.pose.orientation.z = 0.7
      my_wp_2.pose.pose.orientation.w = 0.7
      return my_wp_2
  def produce_waypoint_middle_right_low(self, move):
      # verso diagonale destra in basso
      my_wp_3 = PoseWithCovarianceStamped()
      my_wp_3.header.stamp = rospy.Time.now()
      my_wp_3.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp_3.pose.pose.position.x = self.position_container.x + self.base_distance + move
      my_wp_3.pose.pose.position.y = self.position_container.y + self.base_distance + move
      my_wp_3.pose.pose.position.z = self.position_container.z
      my_wp_3.pose.pose.orientation.x = 0.0
      my_wp_3.pose.pose.orientation.y = 0.0
      my_wp_3.pose.pose.orientation.z = 0.35
      my_wp_3.pose.pose.orientation.w = 1.0
      return my_wp_3
  def produce_waypoint_left(self, move):
      my_wp_4 = PoseWithCovarianceStamped()
      my_wp_4.header.stamp = rospy.Time.now()
      my_wp_4.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp_4.pose.pose.position.x = self.position_container.x
      my_wp_4.pose.pose.position.y = self.position_container.y - self.base_distance - move
      my_wp_4.pose.pose.position.z = self.position_container.z
      my_wp_4.pose.pose.orientation.x = 0.0
      my_wp_4.pose.pose.orientation.y = 0.0
      my_wp_4.pose.pose.orientation.z = -0.7
      my_wp_4.pose.pose.orientation.w = 0.7
      return my_wp_4
  def produce_waypoint_left_low(self, move):
      # diagonale sinistra in basso
      my_wp_5 = PoseWithCovarianceStamped()
      my_wp_5.header.stamp = rospy.Time.now()
      my_wp_5.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp_5.pose.pose.position.x = self.position_container.x + self.base_distance + move
      my_wp_5.pose.pose.position.y = self.position_container.y - self.base_distance - move
      my_wp_5.pose.pose.position.z = self.position_container.z
      my_wp_5.pose.pose.orientation.x = 0.0
      my_wp_5.pose.pose.orientation.y = 0.0
      my_wp_5.pose.pose.orientation.z = -0.35
      my_wp_5.pose.pose.orientation.w = 1.0
      return my_wp_5
  def produce_waypoint_up(self, move):
      # in alto
      my_wp_6 = PoseWithCovarianceStamped()
      my_wp_6.header.stamp = rospy.Time.now()
      my_wp_6.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp_6.pose.pose.position.x = self.position_container.x - self.base_distance - move
      my_wp_6.pose.pose.position.y = self.position_container.y
      my_wp_6.pose.pose.position.z = self.position_container.z
      my_wp_6.pose.pose.orientation.x = 0.0
      my_wp_6.pose.pose.orientation.y = 0.0
      my_wp_6.pose.pose.orientation.z = 1.0
      my_wp_6.pose.pose.orientation.w = 0.0
      return my_wp_6
  def produce_waypoint_up_left(self, move):
      # verso diagonale sinistra in alto
      my_wp_7 = PoseWithCovarianceStamped()
      my_wp_7.header.stamp = rospy.Time.now()
      my_wp_7.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp_7.pose.pose.position.x = self.position_container.x - self.base_distance - move
      my_wp_7.pose.pose.position.y = self.position_container.y - self.base_distance - move
      my_wp_7.pose.pose.position.z = self.position_container.z
      my_wp_7.pose.pose.orientation.x = 0.0
      my_wp_7.pose.pose.orientation.y = 0.0
      my_wp_7.pose.pose.orientation.z = 1.0
      my_wp_7.pose.pose.orientation.w = -0.35
      return my_wp_7
  def produce_waypoint_up_right(self, move):
      # verso diagonale destra in alto
      my_wp_8 = PoseWithCovarianceStamped()
      my_wp_8.header.stamp = rospy.Time.now()
      my_wp_8.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp_8.pose.pose.position.x = self.position_container.x - self.base_distance - move
      my_wp_8.pose.pose.position.y = self.position_container.y + self.base_distance + move
      my_wp_8.pose.pose.position.z = self.position_container.z
      my_wp_8.pose.pose.orientation.x = 0.0
      my_wp_8.pose.pose.orientation.y = 0.0
      my_wp_8.pose.pose.orientation.z = 1.0
      my_wp_8.pose.pose.orientation.w = 0.35
      return my_wp_8
  def produce_waypoint_origin(self):
    my_wp_starting = PoseWithCovarianceStamped()
    my_wp_starting = PoseWithCovarianceStamped()
    my_wp_starting.header.stamp = rospy.Time.now()
    my_wp_starting.header.frame_id = "/map"      
    #type(pose) = geometry_msgs.msg.Pose
    my_wp_starting.pose.pose.position.x = 0.0
    my_wp_starting.pose.pose.position.y = 0.0
    my_wp_starting.pose.pose.position.z = 0.0
    roll = 0
    pitch = 0
    yaw = 0.7
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw) 
    #type(pose) = geometry_msgs.msg.Pose
    my_wp_starting.pose.pose.orientation.x = quaternion[0]
    my_wp_starting.pose.pose.orientation.y = quaternion[1]
    my_wp_starting.pose.pose.orientation.z = quaternion[2]
    my_wp_starting.pose.pose.orientation.w = quaternion[3]

    return my_wp_starting

  def go_to_origin(self):
    rate = rospy.Rate(10) # 10hz 
    pub_wp = rospy.Publisher('/my_jackal_waypoints', PoseWithCovarianceStamped, queue_size=1)
    pub_init_wp = rospy.Publisher('/path_ready', Empty, queue_size=1)

    waypoint = self.produce_waypoint_origin()
    while not rospy.is_shutdown():
      connections = pub_wp.get_num_connections()
      if connections > 0:
        pub_wp.publish(waypoint)
        break
      rospy.loginfo("Waiting for /my_jackal_waypoints topic")
      rate.sleep()
    rospy.loginfo("Published waypoint number ="+str())
    time.sleep(2)

    start_command = Empty()
    while not rospy.is_shutdown():
      connections = pub_init_wp.get_num_connections()
      if connections > 0:
        pub_init_wp.publish(start_command)
        rospy.loginfo("Sent waypoint list execution command")
        break
      rospy.loginfo("Waiting for /path_ready topic")
      rate.sleep()
    rospy.wait_for_message('/move_base/result', MoveBaseActionResult)

  def produce_waypoint(self):
      global waypoints
      waypoints = []
      #original postion
      my_wp_original = PoseWithCovarianceStamped()
      my_wp_original.header.stamp = rospy.Time.now()
      my_wp_original.header.frame_id = "/map"      
      #type(pose) = geometry_msgs.msg.Pose
      my_wp_original.pose.pose.position.x = self.position_container.x
      my_wp_original.pose.pose.position.y = self.position_container.y
      my_wp_original.pose.pose.position.z = self.position_container.z
      my_wp_original.pose.pose.orientation.x = self.quaternion_container.x
      my_wp_original.pose.pose.orientation.y = self.quaternion_container.y
      my_wp_original.pose.pose.orientation.z = self.quaternion_container.z
      my_wp_original.pose.pose.orientation.w = self.quaternion_container.w
      
      # in basso
      my_wp = self.produce_waypoint_low(0)

      # verso destra
      my_wp_2 = self.produce_waypoint_right(0)

      # verso diagonale destra in basso
      my_wp_3 = self.produce_waypoint_middle_right_low(0)#

      # verso sinistra
      my_wp_4 = self.produce_waypoint_left(0)

      # diagonale sinistra in basso
      my_wp_5 = self.produce_waypoint_left_low(0)


      # in alto
      my_wp_6 = self.produce_waypoint_up(0)#

      # verso diagonale sinistra in alto
      my_wp_7 = self.produce_waypoint_up_left(0)#

      # verso diagonale destra in alto
      my_wp_8 = self.produce_waypoint_up_right(0)#

      waypoints.append(my_wp)
      waypoints.append(my_wp_2)
      waypoints.append(my_wp_3)
      waypoints.append(my_wp_4)
      waypoints.append(my_wp_5)
      waypoints.append(my_wp_6)
      waypoints.append(my_wp_7)
      waypoints.append(my_wp_8)
      return waypoints
  def talker(self):
      rospy.init_node('waypoint_publisher', anonymous=True)
      global waypoints 
      pub_wp = rospy.Publisher('/my_jackal_waypoints', PoseWithCovarianceStamped, queue_size=1)
        
      pub_init_wp = rospy.Publisher('/path_ready', Empty, queue_size=1)

      pub_fire_estinguished = rospy.Publisher('/estinguished_fire', String, queue_size=1)
      pub_fire_estinguished.publish("False")
      pub_water = rospy.Publisher('/water/sensor', String, queue_size=1)
      pub_fire_estinguished.publish("False")

      pub_fire_detected = rospy.Publisher('/detected_fire', String, queue_size=1)
      pub_fire_detected.publish("False")

      rate = rospy.Rate(10) # 10hz 
      self.wait_for_gps_origin()
      self.wait_for_gps_goal()
      for waypoint in waypoints:
        while not rospy.is_shutdown():
          connections = pub_wp.get_num_connections()
          if connections > 0:
            pub_wp.publish(waypoint)
            break
          rospy.loginfo("Waiting for /my_jackal_waypoints topic")
          rate.sleep()
        rospy.loginfo("Published waypoint number ="+str())
        time.sleep(2)

      detection_alert = threading.Thread(target=self.alert_approach)
      detection_alert.start()
      start_command = Empty()
      while not rospy.is_shutdown():
        connections = pub_init_wp.get_num_connections()
        if connections > 0:
          pub_init_wp.publish(start_command)
          rospy.loginfo("Sent waypoint list execution command")
          break
        rospy.loginfo("Waiting for /path_ready topic")
        rate.sleep()

      self.wait_for_gps_end()

      self.wait_for_position()
      i = 0

      while i < self.attempts:
        rospy.loginfo("attempts : "+str(i+1))
        waypoints = self.produce_waypoint()
        rospy.loginfo("1 second to start")
        time.sleep(1)  

        for waypoint in waypoints:
          while not rospy.is_shutdown() and not self.alert:
            connections = pub_wp.get_num_connections()
            if connections > 0:
              pub_wp.publish(waypoint)
              break
            rospy.loginfo("Waiting for /my_jackal_waypoints topic")
            rate.sleep()
          rospy.loginfo("Published waypoint number ="+str())
          time.sleep(2)

        while not rospy.is_shutdown() and not self.alert:
          connections = pub_init_wp.get_num_connections()
          if connections > 0:
            pub_init_wp.publish(start_command)
            rospy.loginfo("Sent waypoint list execution command")
            break
          rospy.loginfo("Waiting for /path_ready topic")
          rate.sleep()

        self.wait_for_detection()
        if not self.detected and not self.alert:
          self.base_distance = self.base_distance + 1
          i = i + 1
        else:
          break

      rospy.loginfo("start continue approach")
      if self.detected and not self.alert:
        self.continue_approach()

      if self.alert:
        rospy.loginfo("Estinguish Fire")
        time.sleep(10)
        pub_fire_estinguished.publish("True")
        pub_water.publish("0")
        rospy.loginfo("Fire Estinguished")
        rospy.loginfo("Return Base")
        self.go_to_origin()
        time.sleep(10)
        rospy.loginfo("Restart mission")

if __name__ == '__main__':
  while not rospy.is_shutdown():
    produce = ProducePath()
    try:
        produce.talker()
    except rospy.ROSInterruptException:
      pass