# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from copy import deepcopy


from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

import message_filters

from object_analytics_msgs.msg import ObjectsInBoxes3D
from object_analytics_msgs.msg import TrackedObjects
from object_msgs.msg import ObjectsInBoxes

import rclpy

from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class Roi(object):
    """
    Wrapper of sensor_msgs.msg.RegionOfInterest for hashing.

    when roi is used to be as dict key, it's required to have its own hash implementation
    which needs to implement two special methods __hash__ and __eq__

    """

    def __init__(self, roi):
        """
        Construct a Roi from sensor_msgs.msg.RegionOfInterest instance.

        Args:
            roi (RegionOfInterest): Roi of a detected object or tracked object

        """
        self._hash = hash((roi.x_offset, roi.y_offset, roi.height, roi.width))

    def __hash__(self):
        return self._hash

    def __eq__(self, other):
        return hash(self) == hash(other)


class ObjectItem(object):
    """
    An ObjectItem represents a merged result.

    Merge localization, detection and tracking which has the same roi.
    The main usage of this class is to build Marker based on the merged information.

    """

    def __init__(self, header, roi, track_id, detected_object, min_3d, max_3d):
        """
        Build an instance from messages.

        Args:
            header (std_msgs.msg.Header):      Message header
            roi (RegionOfInterest):            Region of interest
            track_id (int):                    Tracking id
            detected_object (Object):          Instance of object_msgs.msg.Object
            min_3d (geometry_msgs.msg.Point32):   Min position in 3d space
            max (geometry_msgs.msg.Point32):   Max position in 3d space

        """
        self.YELLOW = ColorRGBA()
        self.YELLOW.r = 1.0
        self.YELLOW.g = 1.0
        self.YELLOW.b = 0.0
        self.YELLOW.a = 1.0

        self.GREEN = ColorRGBA()
        self.GREEN.r = 0.0
        self.GREEN.g = 1.0
        self.GREEN.b = 0.0
        self.GREEN.a = 1.0

        self.POSE = Pose()
        self._header = header
        self._roi = roi
        self._track_id = track_id
        self._object = detected_object

        self._p1 = Point()
        self._p1.x = min_3d.x
        self._p1.y = min_3d.y
        self._p1.z = min_3d.z

        self._p2 = deepcopy(self._p1)
        self._p2.x = max_3d.x

        self._p3 = deepcopy(self._p2)
        self._p3.z = max_3d.z

        self._p4 = deepcopy(self._p3)
        self._p4.x = min_3d.x

        self._p5 = deepcopy(self._p1)
        self._p5.y = max_3d.y

        self._p6 = deepcopy(self._p5)
        self._p6.x = max_3d.x

        self._p7 = deepcopy(self._p6)
        self._p7.z = max_3d.z

        self._p8 = deepcopy(self._p7)
        self._p8.x = min_3d.x

    def linelist(self):
        """Build line list marker from 8 points in 3d space."""
        line_list = Marker()
        line_list.header = self._header
        line_list.type = Marker.LINE_LIST
        line_list.action = Marker.ADD
        line_list.scale.x = 0.005
        line_list.color = self.YELLOW
        line_list.pose = deepcopy(self.POSE)

        line_list.pose.position.x = self._p1.x
        line_list.pose.position.y = self._p1.y
        line_list.pose.position.z = self._p1.z

        line_list.points.extend((self._p1, self._p2))
        line_list.points.extend((self._p2, self._p3))
        line_list.points.extend((self._p3, self._p4))
        line_list.points.extend((self._p4, self._p1))
        line_list.points.extend((self._p5, self._p6))
        line_list.points.extend((self._p6, self._p7))
        line_list.points.extend((self._p7, self._p8))
        line_list.points.extend((self._p8, self._p5))
        line_list.points.extend((self._p1, self._p5))
        line_list.points.extend((self._p2, self._p6))
        line_list.points.extend((self._p3, self._p7))
        line_list.points.extend((self._p4, self._p8))

        return line_list

    def min_text(self):
        """Build text marker to hold min_3d text."""
        text = Marker()
        text.header = self._header
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.05
        text.color = self.GREEN
        text.pose = deepcopy(self.POSE)
        text.pose.position.x = self._p1.x
        text.pose.position.y = self._p1.y
        text.pose.position.z = self._p1.z
        text.text = 'Min[{:.2f} {:.2f} {:.2f}]'.format(self._p1.x, self._p1.y, self._p1.z)
        return text

    def max_text(self):
        """Build text marker to hold max_3d text."""
        text = Marker()
        text.header = self._header
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.05
        text.color = self.GREEN
        text.pose = deepcopy(self.POSE)
        text.pose.position.x = self._p7.x
        text.pose.position.y = self._p7.y
        text.pose.position.z = self._p7.z
        text.text = 'Max[{:.2f} {:.2f} {:.2f}]'.format(self._p7.x, self._p7.y, self._p7.z)
        return text

    def name_id_text(self):
        """Build text marker to hold name and tracking id."""
        text = Marker()
        text.header = self._header
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.05
        text.color = self.GREEN
        text.pose = deepcopy(self.POSE)
        text.pose.position.x = self._p1.x
        text.pose.position.y = (self._p1.y + self._p5.y) / 2
        text.pose.position.z = self._p1.z
        text.text = '{} #{}'.format(self._object.object_name, self._track_id)
        return text


class SynchronizedSubscriber(Node):
    """Time synchronizer wrapper for localization."""

    def __init__(self):
        """Time synchronizer for Localization, detection and tracking."""
        super().__init__('object_analytics_visualization3d')

        self.get_logger().info('Start marker_publisher')
        self._pub = self.create_publisher(MarkerArray, '/object_analytics/localization_rviz')

        loc_sub = message_filters.Subscriber(self, ObjectsInBoxes3D,
                                             '/object_analytics/localization')
        det_sub = message_filters.Subscriber(self, ObjectsInBoxes,
                                             '/movidius_ncs_stream/detected_objects')
        tra_sub = message_filters.Subscriber(self, TrackedObjects,
                                             '/object_analytics/tracking')

        ts = message_filters.ApproximateTimeSynchronizer([det_sub, loc_sub, tra_sub], 10, 0.01)

        ts.registerCallback(self._callback)

    def _callback(self, detected_objects, localized_objects, tracked_objects):
        """
        Time synchronizer callback.

        Args:
            detected_objects (object_msgs.msg.ObjectsInBoxes):
                              Detection results of current frame
            localized_object (object_analytics_msgs.msg.ObjectsInBoxes3D):
                              Localization results of current frame
            tracked_objects (object_analytics_msgs.msg.TrackedObjects):
                              Tracking results of current frame

        """
        self.get_logger().info('****************TimeSynchromizer: callback**************')
        objects = self._merge_results(detected_objects, localized_objects, tracked_objects)
        marker_array = self._build_marker_array(detected_objects.header, objects)
        self.get_logger().info('[publish]: "%s"' % marker_array)
        self._pub.publish(marker_array)

    # @staticmethod
    # def _merge_results(self, detected_objects, localized_objects):
    def _merge_results(self, detected_objects, localized_objects, tracked_objects):
        """
        Merge detection, localization and tracking which the same roi.

        roi should be include in localizaton result
        Args:
            detected_objects (object_msgs.msg.ObjectsInBoxes):
                              Detection results of current frame
            localized_object (object_analytics_msgs.msg.ObjectsInBoxes3D):
                              Localization results of current frame
            tracked_objects (object_analytics_msgs.msg.TrackedObjects):
                             Tracking results of current frame
        Returns:
            ObjectItem list: list of ObjectItem in which each item is merged from
                             detction, tracking and localization.

        """
        key = []
        localized_map = {}
        for obj in localized_objects.objects_in_boxes:
            localized_map[Roi(obj.roi)] = obj

        detected_map = {}
        for obj in detected_objects.objects_vector:
            key = Roi(obj.roi)
            if key not in localized_map:
                continue
            detected_map[Roi(obj.roi)] = obj

        if key is None:
            return

        merged = []
        for obj in tracked_objects.tracked_objects:
            # TODO: later to fix track issue and replace with roi check, not only x_offset check.
            # for obj in tracked_objects.tracked_objects:
                # key = Roi(obj.roi)
            if obj.roi.x_offset != localized_map[key].roi.x_offset:
                continue
            merged.append(ObjectItem(tracked_objects.header, obj.roi, obj.id,
                                     detected_map[key].object,
                                     localized_map[key].min, localized_map[key].max))
        return merged

    # @staticmethod
    def _build_marker_array(self, header, objects):
        """
        Build MarkerArray from given header and list of ObjectItem.

        Args:
            header (std_msgs.msg.Header):
                     Header of current processing frame
            objects (list):
                     List of ObjectItem in which each item represents a merged result
        Returns:
            MarkerArray: Includes marker for cleaning preivous markers,
                         marker for bouding box and marker for text.

        """
        marker_array = MarkerArray()

        clean_all_marker = Marker()
        clean_all_marker.header = header
        clean_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(clean_all_marker)

        markers = []
        for obj in objects:
            markers.extend([obj.linelist()])
            markers.extend([obj.min_text()])
            markers.extend([obj.max_text()])
            markers.extend([obj.name_id_text()])
        for idx, marker in enumerate(markers):
            marker.id = idx
        marker_array.markers.extend(markers)

        return marker_array


def main(args=None):
    rclpy.init(args=args)

    synchronized_subscriber = SynchronizedSubscriber()

    rclpy.spin(synchronized_subscriber)

    synchronized_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
