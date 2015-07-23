#!/usr/bin/env python

import time
import math
import datetime

import rospy
import pymongo
from occurrence_learning.msg import RegionObservationTime
from tf.transformations import euler_from_quaternion
from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy


def robot_view_cone(Px, Py, yaw):
    """
        let's call the triangle PLR, where P is the robot pose,
        L the left vertex, R the right vertex
    """
    d = 4       # max monitored distance: reasonably not more than 3.5-4m
    alpha = 1   # field of view: 57 deg kinect, 58 xtion, we can use exactly 1 rad (=57.3 deg)
    Lx = Px + d * (math.cos((yaw-alpha)/2))
    Ly = Py + d * (math.cos((yaw-alpha)/2))
    Rx = Px + d * (math.cos((yaw+alpha)/2))
    Ry = Py + d * (math.cos((yaw+alpha)/2))
    return [[Lx, Ly], [Rx, Ry], [Px, Py]]


class RegionObservationTimeManager(object):

    def __init__(self, soma_map, soma_config):
        self.soma_map = soma_map
        self.soma_config = soma_config
        self.ms = MessageStoreProxy(collection="region_observation_time")
        self.gs = GeoSpatialStoreProxy('geospatial_store', 'soma')
        self.roslog = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        ).roslog.robot_pose
        self.region_observation_duration = dict()
        self._month_year_observation_taken = dict()

    def load_from_mongo(self, days, minute_interval=60):
        """
            Load robot-region observation time from database (mongodb) and store them in
            self.region_observation_duration.
            Returning (region observation time, total duration of observation)
        """
        roi_region_daily = dict()
        total_duration = 0
        for i in days:
            end_date = i + datetime.timedelta(hours=24)
            rospy.loginfo(
                "Load region observation time from %s to %s..." % (str(i), str(end_date))
            )
            query = {
                "soma": self.soma_map, "soma_config": self.soma_config,
                "start_from.secs": {
                    "$gte": time.mktime(i.timetuple()),
                    "$lt": time.mktime(end_date.timetuple())
                }
            }
            roi_reg_hourly = dict()
            for log in self.ms.query(RegionObservationTime._type, message_query=query):
                end = datetime.datetime.fromtimestamp(log[0].until.secs)
                if log[0].region_id not in roi_reg_hourly:
                    temp = list()
                    start = datetime.datetime.fromtimestamp(log[0].start_from.secs)
                    interval = (end.minute + 1) - start.minute
                    if interval != minute_interval:
                        continue
                    for j in range(24):
                        group_mins = {k*interval: 0 for k in range(1, (60/interval)+1)}
                        temp.append(group_mins)
                        roi_reg_hourly.update({log[0].region_id: temp})
                roi_reg_hourly[log[0].region_id][end.hour][end.minute+1] = log[0].duration.secs
                total_duration += log[0].duration.secs
            roi_region_daily.update({i.day: roi_reg_hourly})
        return roi_region_daily, total_duration

    def store_to_mongo(self, data=dict()):
        """
            Store region observation time from self.region_observation_duration to mongodb.
            It will store soma map, soma configuration, region_id, the starting and end time where
            robot sees a region in some interval, and the duration of how long the robot during
            the interval.
        """
        rospy.loginfo("Storing region observation time to region_observation_time collection...")
        if data == dict():
            data = self.region_observation_duration
        minute_interval = 60 / len(data.values()[0].values()[0][0])
        for day, daily_data in data.iteritems():
            for reg, reg_data in daily_data.iteritems():
                for hour, hourly_data in enumerate(reg_data):
                    for minute, duration in hourly_data.iteritems():
                        date_until = datetime.datetime(
                            self._month_year_observation_taken[day][1],
                            self._month_year_observation_taken[day][0],
                            day, hour, minute-1,
                            59
                        )
                        until = time.mktime(date_until.timetuple())
                        start_from = until - (60 * minute_interval) + 1
                        msg = RegionObservationTime(
                            self.soma_map, self.soma_config, reg,
                            rospy.Time(start_from), rospy.Time(until),
                            rospy.Duration(duration)
                        )
                        self._store_to_mongo(msg)

    def _store_to_mongo(self, msg):
        query = {
            "soma": msg.soma, "soma_config": msg.soma_config,
            "region_id": msg.region_id, "start_from.secs": msg.start_from.secs,
            "until.secs": msg.until.secs
        }
        if msg.duration.secs > 0:
            if len(self.ms.query(RegionObservationTime._type, message_query=query)) > 0:
                self.ms.update(msg, message_query=query)
            else:
                self.ms.insert(msg)

    def calculate_region_observation_duration(self, days, minute_interval=60):
        """
            Calculating the region observation duration for particular days, splitted by
            minute_interval.
            Returns the ROIs the robot has monitored at each logged robot pose
            for particular days specified up to minutes interval.
        """
        rospy.loginfo('Calculation region observation duration...')
        roi_region_daily = dict()

        for i in days:
            end_date = i + datetime.timedelta(hours=24)
            roi_region_hourly = self._get_robot_region_stay_duration(i, end_date, minute_interval)
            roi_region_daily.update({i.day: roi_region_hourly})
            self._month_year_observation_taken.update({i.day: (i.month, i.year)})
        self.region_observation_duration = roi_region_daily
        return roi_region_daily

    # hidden function for get_robot_region_stay_duration
    def _get_robot_region_stay_duration(self, start_date, end_date, minute_interval=60):
        sampling_rate = 10
        roi_temp_list = dict()
        rospy.loginfo("Querying from " + str(start_date) + " to " + str(end_date))

        query = {
            "_id": {"$exists": "true"},
            "_meta.inserted_at": {"$gte": start_date, "$lt": end_date}
        }

        for i, pose in enumerate(self.roslog.find(query)):
            if i % sampling_rate == 0:
                _, _, yaw = euler_from_quaternion(
                    [0, 0, pose['orientation']['z'], pose['orientation']['w']]
                )
                coords = robot_view_cone(pose['position']['x'], pose['position']['y'], yaw)
                langitude_latitude = list()
                for pt in coords:
                    langitude_latitude.append(self.gs.coords_to_lnglat(pt[0], pt[1]))
                langitude_latitude.append(self.gs.coords_to_lnglat(coords[0][0], coords[0][1]))

                for j in self.gs.observed_roi(langitude_latitude, self.soma_map, self.soma_config):
                    region = str(j['soma_roi_id'])
                    hour = pose['_meta']['inserted_at'].time().hour
                    minute = pose['_meta']['inserted_at'].time().minute

                    # Region Knowledge per hour. Bin them by hour and minute_interval.
                    if region not in roi_temp_list:
                        temp = list()
                        for i in range(24):
                            group_mins = {
                                i*minute_interval: 0 for i in range(1, (60/minute_interval)+1)
                            }
                            temp.append(group_mins)
                        roi_temp_list[region] = temp
                    index = ((minute / minute_interval) + 1) * minute_interval
                    roi_temp_list[region][hour][index] += 1

        roi_temp_list = self._normalizing(roi_temp_list, sampling_rate)
        rospy.loginfo("Region observation duration for the query is " + str(roi_temp_list))
        return roi_temp_list

    def _normalizing(self, roi_temp_list, sampling_rate):
        """
            Checking if all robot region relation based on its stay duration is capped
            by minute_interval * 60 (total seconds). If it is not, then normalization is applied
        """
        for reg, hourly_poses in roi_temp_list.iteritems():
            _hourly_poses = list()
            for minute_poses in hourly_poses:
                _hourly_poses.append(sum([i for i in minute_poses.itervalues()]))
            max_hourly_poses = max(_hourly_poses)
            normalizing = False
            for ind, seconds in enumerate(_hourly_poses):
                if seconds > 3601:
                    normalizing = True
                    rospy.logwarn(
                        "The number of seconds robot being in %s in hour %d is greater than 3600 with sampling %d" %
                        (reg, ind, sampling_rate)
                    )
                    rospy.logwarn("Normalizing the number of seconds")
                    break
            if normalizing:
                for ind, val in enumerate(hourly_poses):
                    for minute, seconds in val.iteritems():
                        roi_temp_list[reg][ind][minute] = 3600 / float(max_hourly_poses) * seconds
        return roi_temp_list


if __name__ == '__main__':
    rospy.init_node("region_observation_time_test")
    days = [
        # datetime.datetime(2015, 6, 27, 0, 0),
        datetime.datetime(2015, 6, 28, 0, 0),
        datetime.datetime(2015, 6, 29, 0, 0)
        # datetime.datetime(2015, 5, 6, 0, 0)
    ]
    interval = 20
    rsd = RegionObservationTimeManager("rwth", "rwth_novelty")
    rsd.calculate_region_observation_duration(days, interval)
    rsd.store_to_mongo()
    print rsd.load_from_mongo(days, interval)
