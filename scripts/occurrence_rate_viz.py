#!/usr/bin/env python

import sys
import numpy as np

import rospy
from geometry_msgs.msg import Point
from soma_msgs.msg import SOMAROIObject
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from mongodb_store.message_store import MessageStoreProxy
from occurrence_learning.trajectory_occurrence_freq import TrajectoryOccurrenceFrequencies
from occurrence_learning.occurrence_learning_util import point_inside_polygon


def colour_func(x, y):
    if x == 0:
        x = 0.00001
    return float(x ** 2) / float(x ** 2 + (y * 5) * x)


class TOFViz(object):

    def __init__(self, soma_map, soma_config, minute_interval=1, window_time=10):
        self.tof = TrajectoryOccurrenceFrequencies(soma_map, soma_config, minute_interval, window_time)
        self.max_periodic_day = self.tof.periodic_days[-1]
        self.tof.load_tof()
        self.tof = self.tof.tof

        self.window_time = window_time
        self.pub = rospy.Publisher("tof_roi", MarkerArray, queue_size=10)
        self.region_markers = self.init_region_markers(soma_map, soma_config)
        self.text_markers = self.init_text_markers(soma_map, soma_config)

    def init_region_markers(self, soma_map, soma_config):
        soma_roi = MessageStoreProxy(collection="soma_roi").query(
            SOMAROIObject._type,
            message_query={"map": soma_map, "config": soma_config}
        )
        soma_roi = [i[0] for i in soma_roi]
        soma_roi_points = dict()
        for i in soma_roi:
            if i.roi_id not in soma_roi_points:
                soma_roi_points[i.roi_id] = list()
            soma_roi_points[i.roi_id].append(
                [i.pose.position.x, i.pose.position.y]
            )

        region_markers = list()
        for roi_id, roi in soma_roi_points.iteritems():
            region_marker = Marker()
            region_marker.ns = "/region_markers"
            region_marker.id = int(roi_id)
            region_marker.type = Marker.SPHERE_LIST
            # region_marker.action = Marker.ADD
            region_marker.header = Header(0, rospy.Time(), "/map")
            region_marker.lifetime = rospy.Duration(5)

            region_marker.pose.position = Point(0, 0, 0.1)
            region_marker.scale.x = 0.1
            region_marker.scale.y = 0.1
            region_marker.scale.z = 0.1

            [pmin, pmax] = self.get_region_border(roi_id, roi)
            x_max = (pmax[0] - pmin[0]) / 0.2
            y_max = (pmax[1] - pmin[1]) / 0.2
            for x in np.linspace(pmin[0], pmax[0], x_max):
                for y in np.linspace(pmin[1], pmax[1], y_max):
                    if point_inside_polygon(x, y, roi):
                        clr = ColorRGBA(0, 0, 0, 1)
                        region_marker.points.append(Point(x, y, 0))
                        region_marker.colors.append(clr)

            region_markers.append(region_marker)

        return region_markers

    def init_text_markers(self, soma_map, soma_config):
        soma_roi = MessageStoreProxy(collection="soma_roi").query(
            SOMAROIObject._type,
            message_query={"map": soma_map, "config": soma_config}
        )
        soma_roi = [i[0] for i in soma_roi]
        soma_roi_points = dict()
        for i in soma_roi:
            if i.roi_id not in soma_roi_points:
                soma_roi_points[i.roi_id] = list()
            soma_roi_points[i.roi_id].append(
                [i.pose.position.x, i.pose.position.y]
            )

        text_markers = list()
        for roi_id, roi in soma_roi_points.iteritems():
            text_marker = Marker()
            text_marker.ns = "/text_markers"
            text_marker.id = int(roi_id)
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.header = Header(0, rospy.Time(), "/map")
            text_marker.lifetime = rospy.Duration(5)
            text_marker.color.a = 1
            text_marker.scale.z = 0.5
            text_markers.append(text_marker)

            [pmin, pmax] = self.get_region_border(roi_id, roi)
            x_mid = pmin[0] + ((pmax[0] - pmin[0]) / 2.0)
            y_mid = pmin[1] + ((pmax[1] - pmin[1]) / 2.0)
            if point_inside_polygon(x_mid, y_mid, roi):
                text_marker.pose.position = Point(x_mid, y_mid, 0.15)
            else:
                for x in np.linspace(x_mid - 5.0, x_mid + 5.0, 25):
                    for y in np.linspace(y_mid - 5.0, y_mid + 5.0, 25):
                        if point_inside_polygon(x, y, roi):
                            text_marker.pose.position = Point(x, y, 0.15)
                            break
                # text_marker.pose.position = Point(11.3, 13, 0.15)

        return text_markers

    def get_region_border(self, roi_id, region):
        res = None
        if len(region) > 1:
            x_min = region[0][0]
            x_max = region[0][0]
            y_min = region[0][1]
            y_max = region[0][1]

            for i in region[1:]:
                if i[0] < x_min:
                    x_min = i[0]
                if i[0] > x_max:
                    x_max = i[0]
                if i[1] < y_min:
                    y_min = i[1]
                if i[1] > y_max:
                    y_max = i[1]

            if x_min == x_max and y_min == y_max:
                rospy.logwarn("Region %s is a dot, this region is ignored" % roi_id)
            else:
                res = [(x_min, y_min), (x_max, y_max)]
        else:
            rospy.logwarn("Region %s has only one border value, this region is ignored" % roi_id)
        return res

    def visualize_tof(self):
        seq = 0
        max_counter = [self.max_periodic_day, 23, 60]
        days = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
        counter = [0, 0, 0]
        ma = MarkerArray()

        rospy.loginfo(days[counter[0]] + "...")
        rospy.loginfo("Hour: %d" % counter[1])
        while not rospy.is_shutdown():
            if counter[2] < max_counter[2]:
                counter[2] += self.window_time
            else:
                counter[2] = self.window_time
                if counter[1] < max_counter[1]:
                    counter[1] += 1
                    rospy.loginfo("Hour: %d" % counter[1])
                else:
                    counter[1] = 0
                    if counter[0] < max_counter[0]:
                        counter[0] += 1
                    else:
                        counter[0] = 0
                    rospy.loginfo(days[counter[0]] + "...")
                    rospy.loginfo("Hour: %d" % counter[1])

            for region_marker in self.region_markers:
                ma.markers.append(self.change_marker_color(seq, counter, region_marker))

            for text_marker in self.text_markers:
                ma.markers.append(self.change_marker_text(seq, counter, text_marker))

            self.pub.publish(ma)
            rospy.sleep(0.3)
            seq += 1

    def change_marker_color(self, seq, counter, region_marker):
        region_marker.header = Header(seq, rospy.Time(), "/map")
        color_value = self.tof[str(region_marker.id)][counter[0]][counter[1]][counter[2]].get_occurrence_rate()
        # rospy.loginfo("Region %s: %d" % (str(region_marker.id), color_value))

        for i, color in enumerate(region_marker.colors):
            region_marker.colors[i].r = colour_func(color_value, 2)
            region_marker.colors[i].g = colour_func(color_value, 2)
            region_marker.colors[i].a = colour_func(color_value, 1)

        return region_marker

    def change_marker_text(self, seq, counter, text_marker):
        text_marker.header = Header(seq, rospy.Time(), "/map")
        text_marker.text = str(
            round(self.tof[str(text_marker.id)][counter[0]][counter[1]][counter[2]].get_occurrence_rate(), 3)
        )

        return text_marker

    def show_specific_region(self, region, day, hour, mins):
        ma = MarkerArray()
        for region_marker in self.region_markers:
            if region_marker.id == region:
                ma.markers.append(self.change_marker_color(0, [day, hour, mins], region_marker))

        self.pub.publish(ma)
        rospy.sleep(0.3)


if __name__ == "__main__":
    rospy.init_node("tof_visualization")
    if len(sys.argv) < 4:
        rospy.logerr("usage: visualization map map_config minute_interval window_time")
        sys.exit(2)

    test = TOFViz(sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
    test.visualize_tof()
    # test.show_specific_region(12, 0, 7, 40)
    rospy.spin()
