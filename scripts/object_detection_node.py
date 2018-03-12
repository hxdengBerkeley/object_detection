#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from segmentation import *
from classification_svm import *
import numpy as np

class ObjectDetection(object):
    def __init__(self, pub1, pub2, pub3, pub4):
        self._pub1 = pub1
        self._pub2 = pub2
        self._pub3 = pub3
        self._pub4 = pub4

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
            else:
                pc_car = pc2.create_cloud(msg.header, fields, [])
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


def main ():
    rospy.init_node('object_detection')

    pub1 = rospy.Publisher('pedestrian_points', PointCloud2, queue_size=10)
    pub2 = rospy.Publisher('car_points', PointCloud2, queue_size=10)
    pub3 = rospy.Publisher('cyclist_points', PointCloud2, queue_size=10)
    pub4 = rospy.Publisher('misc_points', PointCloud2, queue_size=10)
    detect = ObjectDetection(pub1, pub2, pub3, pub4)

    rospy.Subscriber("/velodyne_points", PointCloud2, detect.callback_cloud)
    rospy.spin()

if __name__ == '__main__':
    main()