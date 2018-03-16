#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
from segmentation import *
from classification_svm import *
import numpy as np

class ObjectDetection(object):
    def __init__(self, pub1, pub2, pub3, pub4, pub5):
        self._pub1 = pub1
        self._pub2 = pub2
        self._pub3 = pub3
        self._pub4 = pub4
        self._pub5 = pub5

    def Arr2poseArray(self, obj_poses):
        obj_PoseArray = PoseArray()
        for obj_pose in obj_poses:
            pose = Pose()
            pose.position.x = obj_pose[0]
            pose.position.y = obj_pose[1]
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            obj_PoseArray.poses.append(pose)
        return obj_PoseArray

    def callback_cloud(self, msg):
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        bbox_3d, obj_points = segmentation(cloud_points)
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 16, PointField.FLOAT32, 1)]
        if obj_points:
            obj_types, obj_classified = classification(obj_points)
            rospy.loginfo('obj_classified: {}'.format([obj_types[i] for i in obj_classified]))

            if sum(obj_classified == 0) > 0:
                pedestrian_points = [obj for indx, obj in enumerate(obj_points) if obj_classified[indx] == 0]
                pedestrian_cloud = np.row_stack(pedestrian_points)
                pc_pedestrian = pc2.create_cloud(msg.header, fields, pedestrian_cloud)
            else:
                pc_pedestrian = pc2.create_cloud(msg.header, fields, [])
            if sum(obj_classified == 1) > 0:
                car_points = [obj for indx, obj in enumerate(obj_points) if obj_classified[indx] == 1]
                car_cloud = np.row_stack(car_points)
                pc_car = pc2.create_cloud(msg.header, fields, car_cloud)
                # publish the centers of the vehicle object as geometry_msgs.PoseArray
                car_poses = [np.mean(bbox, axis=0) for indx, bbox in enumerate(bbox_3d) if obj_classified[indx] == 1]
                car_PoseArray = self.Arr2poseArray(car_poses)
                car_PoseArray.header = msg.header
            else:
                pc_car = pc2.create_cloud(msg.header, fields, [])
                car_PoseArray = PoseArray()
                car_PoseArray.header = msg.header
            if sum(obj_classified == 2) > 0:
                cyclist_points = [obj for indx, obj in enumerate(obj_points) if obj_classified[indx] == 2]
                cyclist_cloud = np.row_stack(cyclist_points)
                pc_cyclist = pc2.create_cloud(msg.header, fields, cyclist_cloud)
            else:
                pc_cyclist = pc2.create_cloud(msg.header, fields, [])
            if sum(obj_classified == 3) > 0:
                misc_points = [obj for indx, obj in enumerate(obj_points) if obj_classified[indx] == 3]
                misc_cloud = np.row_stack(misc_points)
                pc_misc = pc2.create_cloud(msg.header, fields, misc_cloud)
            else:
                pc_misc = pc2.create_cloud(msg.header, fields, [])
        else:
            pc_pedestrian = pc2.create_cloud(msg.header, fields, [])
            pc_car = pc2.create_cloud(msg.header, fields, [])
            pc_cyclist = pc2.create_cloud(msg.header, fields, [])
            pc_misc = pc2.create_cloud(msg.header, fields, [])
        self._pub1.publish(pc_pedestrian)
        self._pub2.publish(pc_car)
        self._pub3.publish(pc_cyclist)
        self._pub4.publish(pc_misc)
        self._pub5.publish(car_PoseArray) 


def main ():
    rospy.init_node('object_detection')

    pub1 = rospy.Publisher('pedestrian_points', PointCloud2, queue_size=10)
    pub2 = rospy.Publisher('car_points', PointCloud2, queue_size=10)
    pub3 = rospy.Publisher('cyclist_points', PointCloud2, queue_size=10)
    pub4 = rospy.Publisher('misc_points', PointCloud2, queue_size=10)
    pub5 = rospy.Publisher('car_posearr', PoseArray, queue_size=10)
    detect = ObjectDetection(pub1, pub2, pub3, pub4, pub5)

    rospy.Subscriber("/velodyne_points", PointCloud2, detect.callback_cloud)
    rospy.spin()

if __name__ == '__main__':
    main()