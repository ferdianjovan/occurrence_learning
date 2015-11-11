#!/usr/bin/env python

import sys
import time
import math
import datetime
import calendar

import rospy
import pymongo
from occurrence_learning.msg import RegionObservationTime
from tf.transformations import euler_from_quaternion
from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy
from occurrence_learning.occurrence_learning_util import robot_view_cone


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
        self.minute_interval = minute_interval
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
        self.region_observation_duration = roi_region_daily
        return roi_region_daily, total_duration

    def store_to_mongo(self):
        """
            Store region observation time from self.region_observation_duration to mongodb.
            It will store soma map, soma configuration, region_id, the starting and end time where
            robot sees a region in some interval, and the duration of how long the robot during
            the interval.
        """
        rospy.loginfo("Storing region observation time to region_observation_time collection...")
        data = self.region_observation_duration
        try:
            minute_interval = self.minute_interval
        except:
            rospy.logwarn("Minute interval is not defined, please call either load_from_mongo or calculate_region_observation_duration first")
            return

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
        self.minute_interval = minute_interval

        for i in days:
            loaded_roi_reg_day = self.load_from_mongo([i], minute_interval)
            if loaded_roi_reg_day[1] == 0:
                end_date = i + datetime.timedelta(hours=24)
                roi_region_hourly = self._get_robot_region_stay_duration(i, end_date, minute_interval)
                roi_region_daily.update({i.day: roi_region_hourly})
            else:
                roi_region_daily.update({i.day: loaded_roi_reg_day[0][i.day]})
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
        regions = roi_temp_list.keys()
        _hourly_poses = [0] * 24
        for i in range(24):
            for region in regions:
                _hourly_poses[i] += sum(roi_temp_list[region][i].values())

        normalizing = sum([1 for i in _hourly_poses if i > 3601]) > 0
        max_hourly_poses = max(_hourly_poses)

        for reg, hourly_poses in roi_temp_list.iteritems():
            if normalizing:
                for ind, val in enumerate(hourly_poses):
                    for minute, seconds in val.iteritems():
                        roi_temp_list[reg][ind][minute] = 3600 / float(max_hourly_poses) * seconds
        return roi_temp_list


if __name__ == '__main__':
    rospy.init_node("region_observation_time")

    if len(sys.argv) < 6:
        rospy.logerr("usage: region_ob_time soma config month year minute_interval")
        sys.exit(2)

    year = int(sys.argv[4])
    month = int(sys.argv[3])
    days = [
        datetime.datetime(year, month, i, 0, 0) for i in range(
            1, calendar.monthrange(year, month)[1]+1
        )
    ]
    interval = int(sys.argv[5])
    rsd = RegionObservationTimeManager(sys.argv[1], sys.argv[2])
    rsd.calculate_region_observation_duration(days, interval)
    rsd.store_to_mongo()
    print rsd.load_from_mongo(days, interval)
