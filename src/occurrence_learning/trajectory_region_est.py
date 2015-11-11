#!/usr/bin/env python

import math
import datetime
import calendar

import rospy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy
from occurrence_learning.region_observation_time import RegionObservationTimeManager


class TrajectoryRegionEstimate(object):

    def __init__(self, soma_map, soma_config, minute_interval=60):
        self.soma_map = soma_map
        self.soma_config = soma_config
        self.minute_interval = minute_interval
        self.region_knowledge = RegionObservationTimeManager(soma_map, soma_config)
        # Service and Proxy
        rospy.loginfo("Connecting to geospatial_store and roslog database...")
        self.gs = GeoSpatialStoreProxy('geospatial_store', 'soma')

    def estimate_trajectories_daily(self, dates, month=1, year=2015):
        """
            Estimate the number of trajectories in a region hourly each day for particular dates
            assuming that the robot sees the complete region when it sees the region
        """
        days = [date for date in dates if date <= calendar.monthrange(year, month)[1]]
        days = [
            datetime.datetime(year, month, i, 0, 0) for i in days
        ]
        return self.estimate_trajectories(days)

    def estimate_trajectories(self, days):
        """
            Estimate the number of trajectories in a region for days,
            assuming that the robot sees the complete region when it sees the region
        """
        days_trajectories = dict()
        roi_region_daily = self.region_knowledge.calculate_region_observation_duration(
            days, self.minute_interval
        )
        trajectory_region_daily = self.obtain_number_of_trajectories(days)

        query = {
            "soma_map": self.soma_map, "soma_config": self.soma_config,
            "soma_roi_id": {"$exists": True}
        }
        regions = self.gs.find_projection(query, {"soma_roi_id": 1})
        minutes = [
            i * self.minute_interval for i in range(1, (60/self.minute_interval) + 1)
        ]
        for region in regions:
            reg = region['soma_roi_id']
            for day in days:
                day = day.day
                if not(day in roi_region_daily and day in trajectory_region_daily):
                    continue
                if reg not in days_trajectories:
                    days_trajectories[reg] = dict()
                if day not in days_trajectories[reg]:
                    temp = list()
                    for i in range(24):
                        temp.append({i: -1 for i in minutes})
                    days_trajectories[reg][day] = temp
                for hour in range(24):
                    for minute in minutes:
                        if reg in roi_region_daily[day]:
                            days_trajectories = self._estimate_trajectories_by_minutes(
                                reg, day, hour, minute, days_trajectories,
                                roi_region_daily[day], trajectory_region_daily[day]
                            )

        return days_trajectories

    def _extrapolate_trajectory_estimate(self, len_min_interval, seconds_stay_duration, num_traj):
        """ extrapolate the number of trajectories with specific upper_threshold.
            upper_threshold is to ceil how long the robot was in an area
            for one minute interval, if the robot was there for less than 20
            seconds, then it will be boosted to 20 seconds.
        """
        upper_threshold_duration = 0
        while seconds_stay_duration > upper_threshold_duration:
            upper_threshold_duration += self.minute_interval * 20

        multiplier_estimator = 3600 / float(
            len_min_interval * upper_threshold_duration
        )
        return math.ceil(multiplier_estimator * num_traj)

    def _hard_trajectory_estimate(self, seconds_stay_duration, num_traj):
        """ Estimating the number of trajectories with hard threshold.
            This forces the seconds_stay_duration to be 60 seconds for num_traj to be registered.
        """
        temp_num_traj = -1
        if seconds_stay_duration >= self.minute_interval * 60:
            temp_num_traj = num_traj
        return temp_num_traj

    def _estimate_trajectories_by_minutes(
        self, region, day, hour, minute, days_trajectories, rois_day, trajectories_day
    ):
        minutes = [
            i * self.minute_interval for i in range(1, (60/self.minute_interval) + 1)
        ]
        if len(trajectories_day) > hour:
            if region in trajectories_day[hour]:
                if minute in trajectories_day[hour][region]:
                    traj = trajectories_day[hour][region][minute]
                    if rois_day[region][hour][minute] > 0:
                        days_trajectories[region][day][hour][minute] = self._extrapolate_trajectory_estimate(
                            len(minutes), rois_day[region][hour][minute], traj
                        )
                        # days_trajectories[region][day][hour][minute] = self._hard_trajectory_estimate(
                        #     rois_day[region][hour][minute], traj
                        # )
                    else:
                        if traj > 0:
                            rospy.logwarn(
                                "%d trajectories are detected in region %s at day %d hour %d minute %d, but no info about robot being there" % (
                                    traj, region, day, hour, minute
                                )
                            )
                else:
                    if rois_day[region][hour][minute] >= self.minute_interval * 60:
                        days_trajectories[region][day][hour][minute] = 0
            else:
                for i in minutes:
                    if rois_day[region][hour][i] >= self.minute_interval * 60:
                        days_trajectories[region][day][hour][i] = 0
        else:
            query = {
                "soma_map": self.soma_map, "soma_config": self.soma_config,
                "soma_roi_id": {"$exists": True}
            }
            regions = self.gs.find_projection(query, {"soma_roi_id": 1})
            for j in regions:
                reg = j['soma_roi_id']
                if reg in rois_day and hour in rois_day[reg]:
                    for i in minutes:
                        if i in rois_day[reg][hour] and rois_day[reg][hour][i] >= self.minute_interval * 60:
                            days_trajectories[reg][day][hour][i] = 0

        return days_trajectories

    def obtain_number_of_trajectories(self, days):
        """
            Obtaining the number of trajectories for each day in argument days,
            The number of Trajectories will be split into day, hour, region, and minutes.
            Trajectories = [day[hour[region[minutes]]]]
        """
        rospy.loginfo('Updating frequency of trajectories in each region...')
        trajectory_region_daily = dict()
        for i in days:
            daily_trajectories = self._trajectories_daily(
                calendar.timegm(i.timetuple())
            )
            trajectory_region_daily.update({i.day: daily_trajectories})
        return trajectory_region_daily

    # count trajectories in regions for the whole day
    # day is in the epoch, so in default it refers to 1 Jan 1970
    def _trajectories_daily(self, day=0):
        daily_trajectories = list()
        for i in range(24):
            hourly_trajectories = self._trajectories_hourly_for_day(day, i)
            daily_trajectories.append(hourly_trajectories)
        return daily_trajectories

    # count trajectories in the region per hour for a certain day
    # day is in the epoch, so in default it refers to 1 Jan 1970
    def _trajectories_hourly_for_day(self, day=0, hour=0):
        date = datetime.date.fromtimestamp(day)
        rospy.loginfo(
            'Getting trajectories for day %s hour %d...' % (
                date.strftime("%d-%m-%Y"), hour
            )
        )
        start_hour = day + (hour * 3600)
        end_hour = day + ((hour + 1) * 3600)
        trajectories_hourly = self._get_num_traj_by_epoch(
            start_hour, end_hour, self.minute_interval*60
        )
        return trajectories_hourly

    def _get_num_traj_by_epoch(self, start, end, min_inter=3600):
        """
            Count the number of trajectories in areas defined by soma map and soma
            config that is restricted by the query.
        """
        start_hour = start
        end_hour = start + min_inter
        trajectories_hourly = dict()
        while end_hour <= end:
            query = {
                "$or": [
                    {"start": {"$gte": start_hour, "$lt": end_hour}},
                    {"end": {"$gte": start_hour, "$lt": end_hour}}
                ]
            }
            for geo_traj in self.gs.find(query):
                query = {
                    "soma_map":  self.soma_map,
                    "soma_config": self.soma_config,
                    "soma_roi_id": {"$exists": "true"},
                    "loc": {
                        "$geoIntersects": {
                            "$geometry": {
                                "type": "LineString",
                                "coordinates": geo_traj['loc']['coordinates']
                            }
                        }
                    }
                }
                roi_traj = self.gs.find_projection(query, {"soma_roi_id": 1})
                if roi_traj.count() < 1:
                    rospy.logwarn("No region covers the movement of uuid %s" % (geo_traj["uuid"]))
                else:
                    for region in roi_traj:
                        if region['soma_roi_id'] not in trajectories_hourly:
                            trajectories_hourly[region['soma_roi_id']] = dict()
                        index = (end_hour - start) / 60
                        if index not in trajectories_hourly[region['soma_roi_id']]:
                            trajectories_hourly[region['soma_roi_id']][index] = 0
                        trajectories_hourly[region['soma_roi_id']][index] += 1
            start_hour += min_inter
            end_hour += min_inter

        return trajectories_hourly
