#!/usr/bin/env python
import rospy 
import robot_skills
from robot_skills import get_robot
from robot_skills.robot import Robot
from ed.entity import Entity
from std_msgs.msg import String
import rosapi
from rosapi import srv
from laser_line_extraction.msg import LineSegmentList
from laser_line_extraction.msg import LineSegment

from robot_smach_states.util.designators import Designator
from pykdl_ros import FrameStamped, VectorStamped
import typing
from tue_msgs.msg import LocateDoorHandleGoal
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Twist
import actionlib
import numpy
import PyKDL as kdl
from sensor_msgs.msg import LaserScan
import math 
from cb_base_navigation_msgs.msg import LocalPlannerActionResult


from opening_door.srv import door_info


class Direction:
    INWARD = "inward"
    OUTWARD = "outward"
    
class Door(Entity):
    HANDLE_ID = "handle"
    FRAME_LEFT_POINT_ID = "frame_left_point"
    FRAME_RIGHT_POINT_ID = "frame_right_point"
    

    def __init__(self, entity: Entity):
        super().__init__(
            identifier=entity.uuid,
            object_type=entity.etype,
            frame_id=entity.frame_id,
            pose=entity.pose.frame,
            shape=entity.shape,
            volumes=entity.volumes,
            super_types=entity.super_types,
            last_update_time=entity.last_update_time,
        )

    @property
    def handle_pose(self) -> VectorStamped:
        """
        Returns the pose of the handle in map frame
        """
        #rospy.loginfo(self.pose)
        return self._get_volume_center_point_in_map(self.HANDLE_ID)

    @property
    def frame_points(self) -> typing.List[VectorStamped]:
        """
        Returns the ground points of the door frame in map frame
        """
        return [self._get_volume_center_point_in_map(self.FRAME_LEFT_POINT_ID),
                self._get_volume_center_point_in_map(self.FRAME_RIGHT_POINT_ID)]

    def _get_volume_center_point_in_map(self, volume_id: str) -> VectorStamped:
        """
        Gets the center point of a volume (typically defined w.r.t. the entity frame) and converts this to map frame.

        :param volume_id: id of the volume to get the center point from
        :return: center point converted to map frame
        """
        cp_entity = self.volumes[volume_id].center_point
        cp_map = self.pose.frame * cp_entity
        return VectorStamped.from_xyz(cp_map.x(), cp_map.y(), cp_map.z(), rospy.Time.now(),"map")

    def get_direction(self, base_pose: FrameStamped) -> str:
        """
        Determines whether the open the door inward or outward.

        In the current implementation, this is simply deduced from the fact if the robot is currently closer to the
        'inward' or 'outward' volume

        :param base_pose: pose of the base
        :return: inward or outward
        """
        base_pos = base_pose.frame.p
        inward_pos = self._get_volume_center_point_in_map(Direction.INWARD).vector
        outward_pos = self._get_volume_center_point_in_map(Direction.OUTWARD).vector
        delta_inward = (base_pos - inward_pos).Norm()
        delta_outward = (base_pos - outward_pos).Norm()
        return Direction.INWARD if delta_inward < delta_outward else Direction.OUTWARD

class Door2(Entity):
    HANDLE_ID = "handle_volume"
    HANDLE_BEHIND_ID = "handle_behind_volume"
    FRAME_LEFT_POINT_ID = "frame_left_point"
    FRAME_RIGHT_POINT_ID = "frame_right_point"
    HANDLE_POSE = None
    DIRECTION = None

    #constructor. 
    #no more init that a classic entity
    def __init__(self, entity: Entity):
        
        super().__init__(
            identifier=entity.uuid,
            object_type=entity.etype,
            frame_id=entity.frame_id,
            pose=entity.pose.frame,
            shape=entity.shape,
            volumes=entity.volumes,
            super_types=entity.super_types,
            last_update_time=entity.last_update_time,
        )

    @property
    def handle_pose(self) -> VectorStamped:
        """
        Returns the pose of the handle in map frame
        """
        
        return self._get_volume_center_point_in_map(self.HANDLE_ID)
    
    @property
    def handle_behind_pose(self) -> VectorStamped:
        """
        Returns the pose of the handle in map frame
        """
        
        return self._get_volume_center_point_in_map(self.HANDLE_BEHIND_ID)

    @property
    def frame_points(self) -> typing.List[VectorStamped]:
        """
        Returns the ground points of the door frame in map frame
        """
        return [self._get_volume_center_point_in_map(self.FRAME_LEFT_POINT_ID),
                self._get_volume_center_point_in_map(self.FRAME_RIGHT_POINT_ID)]

    def _get_volume_center_point_in_map(self, volume_id: str) -> VectorStamped:
        """
        Gets the center point of a volume (typically defined w.r.t. the entity frame) and converts this to map frame.

        :param volume_id: id of the volume to get the center point from
        :return: center point converted to map frame
        """
        cp_entity = self.volumes[volume_id].center_point
        cp_map = self.pose.frame * cp_entity
        return VectorStamped.from_xyz(cp_map.x(), cp_map.y(), cp_map.z(), rospy.Time.now(),"map")
    
    def update_pose(self,VectorStamped):
        # rospy.loginfo("before update, handle pose is on the type")
        # rospy.loginfo(type(self.HANDLE_POSE))
        # rospy.loginfo(self.HANDLE_POSE)
        self.HANDLE_POSE = VectorStamped
        # rospy.loginfo("atfer update, handle pose is on the type")
        # rospy.loginfo(type(self.HANDLE_POSE))
        # rospy.loginfo(self.HANDLE_POSE)

    def getPose(self):
        return self.HANDLE_POSE
    
    def setDirection(self, direction):
        self.DIRECTION = direction
        
    def getDirection(self):
        """
        for the moment the direction is only positif or negatif according to the axis it is perpendicular to
        if is positif, it mean you can open open the door by pushing it straight according to the axis it is perpendicular to
        In the door of impuls, the door is positif because you can open it by pushing it straight according to the x axis
        """
        return self.DIRECTION


class commandRobot:

    def __init__(self):
        self.robot = get_robot("hero")
        self.arm = self.robot.get_arm(force_sensor_required=True)
        self.frame_map = None

        #door2 
        self.door2=self.robot.ed.get_entity(uuid="door_inside")
        self.my_door2=Door2(self.door2)
        self.door_des2 = Designator(self.my_door2)
        self.my_door2.setDirection("positif")

        # #door
        self.door = self.robot.ed.get_entity(uuid="door")
        self.my_door = Door(self.door)
        self.door_des = Designator(self.my_door)
        
        #service
        self.door_info = rospy.ServiceProxy('door_info', door_info)
        self.rate = rospy.Rate(0.5)
        self.rate2 = rospy.Rate(0.1)

        rospy.loginfo("robot is load")

    def pushDoorOpen(self):
        end = False
        
        while (not end):
            
            door_frame_robot_left = self.robot.tf_buffer.transform(self.my_door2.frame_points[0], self.robot.base_link_frame, rospy.Duration(1.0)) #frame left
            door_frame_robot_right = self.robot.tf_buffer.transform(self.my_door2.frame_points[1], self.robot.base_link_frame, rospy.Duration(1.0)) #frame right
            
            #get the coordinate
            x1 = door_frame_robot_left.vector.x()
            y1 = door_frame_robot_left.vector.y()
            x2 = door_frame_robot_right.vector.x()
            y2 = door_frame_robot_right.vector.y()

            #mean of right and left
            x = (x1 + x2) / 2.0
            y = (y1 + y2) / 2.0
            
            rospy.loginfo("middle of the door, robot frame")
            rospy.loginfo(x)
            rospy.loginfo(y)
            
            
            #we have to check the position to be sure that hero is IFO the door
            position = self.robot.base.get_location()
            rot_y = position.frame.M.GetRot()[2]
            
            rospy.loginfo("rot_y = ")
            rospy.loginfo(rot_y)
            rospy.loginfo(" ")
            
            if rot_y > 0.1:
               publish_twist(0,0,-0.05,0,0,0) 
            elif rot_y < -0.1:
                publish_twist(0,0,0.05,0,0,0) 
            
            elif y < -0.1:
                publish_twist(0,0,0,0,-0.05,0)
            elif y > 0.1:
                publish_twist(0,0,0,0,+0.05,0)
            elif x > 0.1:
                publish_twist(0,0,0,0.1,0,0)
            else:
                end = True
            rospy.sleep(0.5)
          
    def checkrotation(self):
        location_map = self.robot.base.get_location() 
        rospy.loginfo("location")
        rospy.loginfo(location_map)
        
        location_door = self.robot.tf_buffer.transform(location_map, "door_inside", rospy.Duration(1.0))
        rospy.loginfo("location door")
        rospy.loginfo(location_door)
    
    def serviceCall(self, commande):
        self.door_info.call(commande,0)
        self.rate.sleep()

    def move_arm(self, a, b ,c , d, e ):
        list_send = [a,b,c,d,e]
        self.arm._arm._send_joint_trajectory([list_send])
        rospy.loginfo("harm has moved")

    def gripper(self, open):
        if open:
            self.arm._arm.gripper.send_goal("open")
            rospy.loginfo("gripper open")
        else:
            self.arm._arm.gripper.send_goal("close")
            rospy.loginfo("gripper close")

    def Locate_handle(self):
        
        # self.serviceCall("goIFOdoor")
        # self.rate.sleep()
        
        # position_achieve()
        handle_estimate = self.my_door2.handle_pose #call handle pose to know where is the handle. Renvoie le milieu de l'endoit ou est la poignee

        # rospy.loginfo("about handle pose")
        # rospy.loginfo(handle_estimate)

        #self.robot.head.look_at_point(handle_estimate, timeout=0.0)
        #self.robot.head.wait_for_motion_done()

        goal = LocateDoorHandleGoal() #don't really know what this fct does but it give a goal. goal is nothing here
        # rospy.loginfo("goal info")
        # rospy.loginfo(type(goal))

        goal_estimate = PointStamped() #don't really know but it gives an estimation goal according to what the robot know. goal estimate is also nothing here
        # rospy.loginfo("goal estimate info")
        # rospy.loginfo(type(goal_estimate))

        #goal estimate is useless because at the and we use handle_estimate
        goal_estimate.header.frame_id = "map"
        goal_estimate.point.x = handle_estimate.vector.x()
        goal_estimate.point.y = handle_estimate.vector.y()
        goal_estimate.point.z = handle_estimate.vector.z()

        #now goal estimate is something

        # rospy.loginfo("goal estimate info2")
        # rospy.loginfo(goal_estimate)

        #same for goal
        goal.handle_location_estimate = goal_estimate
        # rospy.loginfo("goal info2")
        # rospy.loginfo(goal)

        #to use the perception we have to use goal and not goal_estimate.
        #it seems goal_estimate is more a part of goal, only for the handle
        #to use the 3 following fct, hero has to be IFO the door (TT RVIZ or TT service)
        self.robot.perception.locate_handle_client.send_goal(goal) #on lui dit ou regarder
        self.robot.perception.locate_handle_client.wait_for_result(rospy.Duration.from_sec(5.0)) #on attend gentiment le resultat
        state = self.robot.perception.locate_handle_client.get_state() #pour savoir si c'est mission accompli ou non.


        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("detecting handle is a success")
            #dans le cas ou c'est un succès, il faut faire des trucs:
            #on recupere le resultat
            result = self.robot.perception.locate_handle_client.get_result()
            # rospy.loginfo("info about the result")
            # rospy.loginfo(result)
            #on va traiter les données en faisant la moyenne de edge 1 et 2
            x = numpy.average([result.handle_edge_point1.point.x, result.handle_edge_point2.point.x])
            y = numpy.average([result.handle_edge_point1.point.y, result.handle_edge_point2.point.y])
            z = numpy.average([result.handle_edge_point1.point.z, result.handle_edge_point2.point.z])
            
            #on va se rapproher de la partie gauche en refaisant une moyenne
            #y = numpy.average([y1,result.handle_edge_point2.point.y])
            
            #on va maintenant ecrire une location fixe pour la poignee
            #pour ca on utilise un vector stamped
            handle_loc = VectorStamped.from_xyz(x,y,z,rospy.Time.now(),result.handle_edge_point1.header.frame_id)

            self.my_door2.update_pose(self.robot.tf_buffer.transform(handle_loc, "map",rospy.Duration(1.0))) #1 seconde is enough for the extrapolation tot the futur.
        else:
            rospy.loginfo("detecting handle is not a success")
     
    def locateBehindHandle(self):
            
        self.serviceCall("goIFOdoor")
        self.rate.sleep()

        handle_estimate = self.my_door2.handle_behind_pose #call handle pose to know where is the handle. Renvoie le milieu de l'endoit ou est la poignee        
        rospy.loginfo("about handle behind pose")
        rospy.loginfo(handle_estimate)
        
        goal = LocateDoorHandleGoal() #don't really know what this fct does but it give a goal. goal is nothing here
        # rospy.loginfo("goal info")
        # rospy.loginfo(type(goal))

        goal_estimate = PointStamped() #don't really know but it gives an estimation goal according to what the robot know. goal estimate is also nothing here
        # rospy.loginfo("goal estimate info")
        # rospy.loginfo(type(goal_estimate))

        #goal estimate is useless because at the and we use handle_estimate
        goal_estimate.header.frame_id = "map"
        goal_estimate.point.x = handle_estimate.vector.x()
        goal_estimate.point.y = handle_estimate.vector.y()
        goal_estimate.point.z = handle_estimate.vector.z()

        #same for goal
        goal.handle_location_estimate = goal_estimate
        # rospy.loginfo("goal info2")
        # rospy.loginfo(goal)
        
        self.robot.perception.locate_handle_client.send_goal(goal) #on lui dit ou regarder
        self.robot.perception.locate_handle_client.wait_for_result(rospy.Duration.from_sec(5.0)) #on attend gentiment le resultat
        state = self.robot.perception.locate_handle_client.get_state() #pour savoir si c'est mission accompli ou non.

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("detecting handle is a success")
            #dans le cas ou c'est un succès, il faut faire des trucs:
            #on recupere le resultat
            result = self.robot.perception.locate_handle_client.get_result()
            # rospy.loginfo("info about the result")
            # rospy.loginfo(result)
            #on va traiter les données en faisant la moyenne de edge 1 et 2
            x = numpy.average([result.handle_edge_point1.point.x, result.handle_edge_point2.point.x])
            y = numpy.average([result.handle_edge_point1.point.y, result.handle_edge_point2.point.y])
            z = numpy.average([result.handle_edge_point1.point.z, result.handle_edge_point2.point.z])
            
            #on va se rapproher de la partie gauche en refaisant une moyenne
            #y = numpy.average([y1,result.handle_edge_point2.point.y])
            
            #on va maintenant ecrire une location fixe pour la poignee
            #pour ca on utilise un vector stamped
            handle_loc = VectorStamped.from_xyz(x,y,z,rospy.Time.now(),result.handle_edge_point1.header.frame_id)

            self.my_door2.update_pose(self.robot.tf_buffer.transform(handle_loc, "map",rospy.Duration(1.0))) #1 seconde is enough for the extrapolation tot the futur.
        else:
            rospy.loginfo("detecting handle is not a success")
            
    def GraspHandle(self):
        # self.serviceCall("is_door_open")
        self.serviceCall("go_treshold")
        #self.move_arm(0.01,0.0,-1.57,-1.2,1.5)
        self.arm.gripper.send_goal("open") #open gripper
        handle_vector = self.my_door2.getPose() #get the pose of the handle (vector)
        
        #let's transform this vector into a FrameStamped
        kdl_rotation = kdl.Rotation()
        kdl_vector = handle_vector.vector
        # rospy.loginfo("info about kdl vector")
        # rospy.loginfo(kdl_vector)

        #hange value of kdl_vector
        kdl_vector[0] = kdl_vector[0] - 0.045#in order to not touch the handle
        kdl_vector[1]=kdl_vector[1]+0.02
        kdl_frame = kdl.Frame(kdl_rotation, kdl_vector)
        handle_frame = FrameStamped(kdl_frame,rospy.Time.now(), frame_id = "map") #map is hard coded but it must change) #get the frame of the handle from the vector
        
        #create a goal
        goal_handle = self.robot.tf_buffer.transform(handle_frame, self.robot.base_link_frame, rospy.Duration(1.0))
        
        #move the arm on the handle
            #move the robot

        # print("goal_handle information")
        # print(type(goal_handle))
        # print(goal_handle)

        # need to move at the right place in order to be able to grasp the handle. Being not too far away
        #regarder la position actuelle puis 
        # trouver une limite jusqu'a laquelle aller
        #en attendant, ceci devrait faire l'affaire
        # self.serviceCall("go_forward")
        # rospy.loginfo("end of going forward")
        
        goal_handle.frame.M = kdl.Rotation.RPY(-1.57, 0.0, 0.0) #rotation of the gripper in order to be able to grasp the handle
         
        result = self.arm.send_goal(goal_handle,timeout=10.0)
        self.arm.wait_for_motion_done()

        if result:
            self.rate2.sleep()
            rospy.loginfo('arm is in the good position: handle is ready to be grasped')
        else:
            rospy.loginfo('grasping handle is not a success')
    
    def GraspBehindHandle(self):
        self.serviceCall("is_door_open")
        self.serviceCall("go_treshold_behind")
        #self.move_arm(0.01,0.0,-1.57,-1.2,1.5)
        self.arm.gripper.send_goal("open") #open gripper
        handle_vector = self.my_door2.getPose() #get the pose of the handle (vector)
        
        #let's transform this vector into a FrameStamped
        kdl_rotation = kdl.Rotation()
        kdl_vector = handle_vector.vector
        # rospy.loginfo("info about kdl vector")
        # rospy.loginfo(kdl_vector)

        #change value of kdl_vector
        kdl_vector[0] = kdl_vector[0] #+ 0.045 #in order to not touch the handle
        kdl_vector[1]=kdl_vector[1]+0.04
        kdl_frame = kdl.Frame(kdl_rotation, kdl_vector)
        handle_frame = FrameStamped(kdl_frame,rospy.Time.now(), frame_id = "map") #map is hard coded but it must change) #get the frame of the handle from the vector
        
        #create a goal
        goal_handle = self.robot.tf_buffer.transform(handle_frame, self.robot.base_link_frame, rospy.Duration(1.0))
        
        #move the arm on the handle
            #move the robot

        # print("goal_handle information")
        # print(type(goal_handle))
        # print(goal_handle)

        # need to move at the right place in order to be able to grasp the handle. Being not too far away
        #regarder la position actuelle puis 
        # trouver une limite jusqu'a laquelle aller
        #en attendant, ceci devrait faire l'affaire
        # self.serviceCall("go_forward")
        # rospy.loginfo("end of going forward")
        
        goal_handle.frame.M = kdl.Rotation.RPY(-1.57, 0.0, 0.0) #rotation of the gripper in order to be able to grasp the handle
         
        result = self.arm.send_goal(goal_handle,timeout=10.0)
        self.arm.wait_for_motion_done()

        if result:
            self.rate2.sleep()
            rospy.loginfo('arm is in the good position: handle is ready to be grasped')
        else:
            rospy.loginfo('grasping handle is not a success')
      
    def PushDoor (self):
        
        #get some frame
        #these are frame of the door according to the robot point of view (TT transform)
        door_frame_robot_left = self.robot.tf_buffer.transform(self.my_door2.frame_points[0], self.robot.base_link_frame, rospy.Duration(1.0)) #frame left
        door_frame_robot_right = self.robot.tf_buffer.transform(self.my_door2.frame_points[1], self.robot.base_link_frame, rospy.Duration(1.0)) #frame right
        
        #rospy.loginfo("info about door_frame_robot")
        
        # rospy.loginfo(type(door_frame_robot0))
        # rospy.loginfo(door_frame_robot0)
        
        #get the coordinate
        x1 = door_frame_robot_left.vector.x()
        y1 = door_frame_robot_left.vector.y()
        x2 = door_frame_robot_right.vector.x()
        y2 = door_frame_robot_right.vector.y()
        
        #mean of right and left
        x = (x1 + x2) / 2.0
        y = (y1 + y2) / 2.0
        
        self.robot.base.force_drive(0.1, y / (x / 0.1), 0, 0.5 ) #self.robot.base.force_drive(0.1, y / (x / 0.1), 0, x )
        
        rospy.loginfo("robot has moved")
        
    def unlatchHandle(self):
        
        #get the joint position
        joint_states = self.arm.get_joint_states()
        joint1, joint2, joint3, joint4, joint5 = joint_states['arm_lift_joint'], joint_states['arm_flex_joint'], joint_states['arm_roll_joint'], joint_states['wrist_flex_joint'], joint_states['wrist_roll_joint']
        
        rospy.loginfo("info about usefull joint state")
        rospy.loginfo(joint1)
        rospy.loginfo(joint2)
        rospy.loginfo(joint3)
        rospy.loginfo(joint4)
        rospy.loginfo(joint5)
        
        joint1_new_position = joint1 - 0.03
        
        list_trajectory = [joint1_new_position, joint2, joint3, joint4, joint5]
        self.arm._arm._send_joint_trajectory([list_trajectory])
   
    def pullDoorOpen(self):
        
        #force_drive(self, vx, vy, vth, timeout)
        #self.robot.base.force_drive(-0.1, 0, 0, 0.5)
        
        #get some frame
        #these are frame of the door according to the robot point of view (TT transform)
        door_frame_robot_left = self.robot.tf_buffer.transform(self.my_door2.frame_points[0], self.robot.base_link_frame, rospy.Duration(1.0)) #frame left
        door_frame_robot_right = self.robot.tf_buffer.transform(self.my_door2.frame_points[1], self.robot.base_link_frame, rospy.Duration(1.0)) #frame right
        
        #get the coordinate
        x1 = door_frame_robot_left.vector.x()
        y1 = door_frame_robot_left.vector.y()
        x2 = door_frame_robot_right.vector.x()
        y2 = door_frame_robot_right.vector.y()
        
        #mean of right and left
        x = (x1 + x2) / 2.0
        y = (y1 + y2) / 2.0
        
        #info about x and y
        rospy.loginfo("info about x and y when we have to drive")
        rospy.loginfo(x)
        rospy.loginfo(y)
        
        rospy.loginfo("info about what is send to force drive")
        
        
        self.robot.base.force_drive(-0.1, y / (x / 0.1), 0, 0.5 )
  
    def try_detecting_opening(self):
        self.frame_map = self.robot.base.get_location()
        rospy.loginfo("location is ")
        rospy.loginfo(self.frame_map)
        self.convert_frame()
        
        rospy.sleep(3)
        
        self.Locate_handle()
        
        rospy.sleep(3)
        
        self.GraspHandle()
        
    def convert_frame(self):
        pose_message_convert_in_frame_door = self.robot.tf_buffer.transform(self.frame_map, "door_inside", rospy.Duration(1.0))
        rospy.loginfo("position frame door")
        rospy.loginfo(pose_message_convert_in_frame_door)    
  
    def laserData_callback(self):
        msg = rospy.wait_for_message("/hero/base_laser/scan", LaserScan)
        
        count = 1 
        minimum = msg.ranges[0]
        minimum_count = 0
        
        length = len(msg.ranges)
        
        while not rospy.is_shutdown() and count < length: # length -1 maybe
            if minimum > msg.ranges[count]: #check all the ranges list
                minimum = msg.ranges[count]
                minimum_count = count
                
            count +=1
            
        #get the angle 
        min_angle = msg.angle_min + msg.angle_increment * minimum_count 
        
        rospy.loginfo("info about min")
        rospy.loginfo(minimum)
        return minimum, min_angle 
          
    def move_away_obstacle(self):
        dmin = 0.7 # minimum away distance from obstacle
        dget = 0.5
        
        while dget < dmin:
            dget, angle_min = self.laserData_callback()
            rospy.loginfo("info about angle min")
            rospy.loginfo(angle_min)
            x_direction = - math.cos(angle_min)
            y_direction = - math.sin(angle_min)
            
            rospy.loginfo("info about x and y ")
            rospy.loginfo(x_direction)
            rospy.loginfo(y_direction)
            rospy.loginfo(" ")
            
            publish_twist(0,0,0, x_direction/15, y_direction/15, 0)
            rospy.sleep(0.5)     
            
        return 'away'   
    
    def publish_marker(self):
        retour = self.door_info.call("write_marker",0)
        #response = retour()
        rospy.loginfo("info about write marker")
        rospy.loginfo(retour)
        rospy.sleep(5)
        self.serviceCall("publish_marker")
    
        
    
    
#function use to create geometry_msgs
def create_pose_stamped(x, y, z, qx, qy, qz, qw):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def publisher():
    #rospy.init_node('talker', anonymous=True)
    
    pub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=2)
    
    pose_stamped_IFOdoor = create_pose_stamped(6, 0.450, 0, 0, 0, 0.01, 0.99)
    rospy.sleep(2)
    pub.publish(pose_stamped_IFOdoor)
    
    rospy.loginfo(pose_stamped_IFOdoor)
    rospy.loginfo(pub)
    while not rospy.is_shutdown():
        rospy.loginfo("hey")
        rospy.sleep(2)
 
 
def create_twist(a,b,c,d,e,f):
    pub = Twist()
    pub.angular.x = a 
    pub.angular.y = b
    pub.angular.z = c
    pub.linear.x = d
    pub.linear.y = e
    pub.linear.z = f
    
    return pub
    
def publish_twist(a,b,c,x,y,z):    
    pub = rospy.Publisher('/hero/base/references',Twist, queue_size=2)
    twist_message = create_twist(a,b,c,x,y,z)
    #rospy.sleep(1)
    pub.publish(twist_message)
    rospy.sleep(0.5)
    
#fonction that allow us to know when we arrive at destination 
def position_achieve():
    shared_planner_message = rospy.wait_for_message("/hero/local_planner/action_server/result", LocalPlannerActionResult)
    while(shared_planner_message is None):
        rospy.sleep(2)

    return True    
   
def read_line_segment():
    msg_line_segments = rospy.wait_for_message("/line_segments", LineSegmentList)     
    rospy.loginfo("line segment msg")
    rospy.loginfo(msg_line_segments)
    
    # rospy.loginfo("line segments list")
    segments = msg_line_segments.line_segments
    # rospy.loginfo(segments)
    
    # rospy.loginfo("line segments list [0]")
    # rospy.loginfo(segments[0])
    
    # rospy.loginfo("line segments list [1]")
    # rospy.loginfo(segments[1])
    
    for segment in segments:
        if segment.start[1] < 0 and segment.end[1] > 0:
            start = segment.start
            end = segment.end
            rospy.loginfo("start and end = ")
            rospy.loginfo(start)
            rospy.loginfo(end)
    
if __name__ == "__main__":
    # #main of the code
    rospy.init_node('open_door_node',anonymous=True)
    rate = rospy.Rate(0.5)
    commandrobot = commandRobot()
    # # rospy.wait_for_service('SetParam')
    # # Setparameter = rospy.ServiceProxy('SetParam', srv.SetParam)
    # # rospy.loginfo("command robot is created and service client is created")
    # # rate.sleep()

    # # Setparameter.call("push_door","1")
    # # rospy.loginfo("door is opened")
    # rate.sleep()

    # commandrobot.publish_marker()

    # rospy.sleep(2)

    # commandrobot.GraspHandle()
    
    # rospy.sleep(3)
    
    # commandrobot.gripper(False)
    
    # #commandrobot.pullDoorOpen()
    
    # commandrobot.gripper(True)
    
    # # commandrobot.unlatchHandle()
    
    # # commandrobot.PushDoor()
    
    #commandrobot.pushDoorOpen()
    
    commandrobot.checkrotation()
    
    rospy.sleep(4)
