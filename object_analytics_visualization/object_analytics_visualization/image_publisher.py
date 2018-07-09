#!/usr/bin/env python
"""
 Copyright (c) 2017 Intel Corporation

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""
import cv2
import message_filters

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from object_msgs.msg import ObjectsInBoxes
from object_analytics_msgs.msg import TrackedObjects
from object_analytics_msgs.msg import ObjectsInBoxes3D

import rclpy

from rclpy.node import Node

"""@package image_publisher
Draw detection as well as tracking results on original RGB image, and republish so that could be shown on RViz image
display
"""


class Roi(object):
    """Wrapper of sensor_msgs.msg.RegionOfInterest for hashing

    when roi is used to be as dict key, it's required to have its own hash implementation which needs to implement two
    special methods __hash__ and __eq__
    """

    def __init__(self, roi):
        """Construct a Roi from sensor_msgs.msg.RegionOfInterest instance

        Args:
            roi (RegionOfInterest): Roi of a detected object or tracked object
        """
        self._hash = hash((roi.x_offset, roi.y_offset, roi.width, roi.height))

    def __hash__(self):
        return self._hash

    def __eq__(self, other):
        return hash(self) == hash(other)


class ObjectItem(object):
    """An ObjectItem represents a merged result of detection and tracking who have the same roi.x_offset

    The main usage of this class is to draw(attach) rectangle and text with the original image
    """


    def __init__(self, cv_image, roi, track_id, detected_object):
        """Build an instance from cv Mat, RegionOfInterest, tracking id and detected object

        Args:
            cv_image (cv2.Mat):         Mat convereted from rgb image
            roi (RegionOfInterest):     Region of interest
            track_id (int):             Tracking id
            detected_object (Object):   Instance of object_msgs.msg.Object
        """
        self._font = cv2.FONT_HERSHEY_SIMPLEX
        self._font_size = 0.5
        self._yellow = (255, 255, 0)
        self._red = (255, 0, 0)
        self._green= (0, 255, 0)
        self._cv_image = cv_image
        self._roi = roi
        self._track_id = track_id
        self._object = detected_object
        self._top_left = (roi.x_offset, roi.y_offset)
        self._down_right = (roi.x_offset + roi.width, roi.y_offset + roi.height)

    def draw(self):
        """Draw rectangle, name, tracking id and roi onto cv Mat.
        """
        self._draw_rectangle()
        self._draw_roi_text()
        self._draw_name_id()

    def _draw_rectangle(self):
        """Draw rectangle same size and position as roi.
        """
        cv2.rectangle(self._cv_image, self._top_left, self._down_right, self._yellow, 0)

    def _draw_roi_text(self):
        """Draw roi text on the top left position.
        """
        text = "ROI[{},{},{},{}]".format(self._roi.x_offset, self._roi.y_offset, self._roi.width, self._roi.height)
        cv2.putText(self._cv_image, text, self._top_left, self._font, self._font_size, self._green, 0, cv2.LINE_AA)

    def _draw_name_id(self):
        """Draw object name and tracking id together in the middle left of the rectangle
        """
        text = "{} #{}".format(self._object.object_name, self._track_id)
        pos = (self._roi.x_offset, self._roi.y_offset + self._roi.height / 2)
        cv2.putText(self._cv_image, text, pos, self._font, self._font_size, self._green, 1, cv2.LINE_AA)


class SynchronizedSubscriber(Node):
    """Time synchronizer wrapper for tracking, detection and original rgb image.

    Create a TimeSynchronizer to synchronize rgb image, detection and tracking topics published by object analytics,
    draw approprate rectangle and text onto original rgb image and republish the updated rgb image.

    """

    def __init__(self):
        """Create TimeSynchronizer to listen image, detection and tracking."""
        super().__init__('object_analytics_tracking_demo')

        self.get_logger().info('Start image_publisher')
        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, '/object_analytics/tracking_2d_rviz')

        img_sub = message_filters.Subscriber(self, Image,
                                             '/object_analytics/rgb')
        det_sub = message_filters.Subscriber(self, ObjectsInBoxes,
                                             '/movidius_ncs_stream/detected_objects')
        tra_sub = message_filters.Subscriber(self, TrackedObjects,
                                             '/object_analytics/tracking')

        ts = message_filters.ApproximateTimeSynchronizer([img_sub, det_sub, tra_sub], 10, 0.01)
        ts.registerCallback(self._callback)


    def _callback(self, image, detected_objects, tracked_objects):
        """TimeSynchronizer callback.

        Args:
            image (sensor_msgs.msg.Image):                              Rgb image of cuurent processing frame
            detected_objects (object_msgs.msg.ObjectsInBoxes):          Detection results of current frame
            tracked_objects (object_analytics_msgs.msg.TrackedObjects): Tracking results of current frame

        """
        print('test1')
        self.get_logger().info('****************TimeSynchromizer: callback**************')
        print('test0')
        header = image.header
        print('width %d, %d' % (image.width, image.height))
        #print('data %s' % image.data)
        print('test-1')
        #image.data = ('[157, 96, 77, 157, 96, 198, 157, 96, 99, 100]').encode()
        cv_image = self._bridge.imgmsg_to_cv2(image, "bgr8")

        print('test2')
        objects = self._merge_results(cv_image, detected_objects, tracked_objects)
        print('test3')
        for obj in objects:
            obj.draw()
        print('test4')
        self._draw_measures(cv_image)

        print('test5')
        msg = self._bridge.cv2_to_imgmsg(cv_image, "bgr8")
        msg.header = header
        self._pub.publish(msg)

    def _draw_measures(self, cv_image):
        """Draw measure result on the left up"""
        text = "\n".join(" ".join((str(m) for m in self._measures)))
        for idx, m in enumerate(self._measures):
            cv2.putText(cv_image, str(m), (2, 15+idx*15), ObjectItem.FONT,
                        ObjectItem.FONT_SIZE, ObjectItem._red, 1, cv2.LINE_AA)

    @staticmethod
    def _merge_results(cv_image, detected_objects, tracked_objects):
        """Merge detection and tracking result which have the same roi, roi should must be included in tracking results.

        Args:
            cv_image (cv2.Mat):                                         Rgb image of cuurent processing frame
            detected_objects (object_msgs.msg.ObjectsInBoxes):          Detection results of current frame
            tracked_objects (object_analytics_msgs.msg.TrackedObjects): Tracking results of current frame

        Returns:
            ObjectItem list: list of ObjectItem in which each item is merged from detction and tracking
        """
        tracking_map = {}
        for obj in tracked_objects.tracked_objects:
            tracking_map[Roi(obj.roi)] = obj

        merged = []
        for obj in detected_objects.objects_vector:
            key = Roi(obj.roi)
            if key not in tracking_map:
                continue
            merged.append(ObjectItem(cv_image, obj.roi, tracking_map[key].id, obj.object))

        return merged


def main(args=None):
    rclpy.init(args=args)

    synchronized_subscriber = SynchronizedSubscriber()

    rclpy.spin(synchronized_subscriber)

    synchronized_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
