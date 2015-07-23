#!/usr/bin/env python

import math
import datetime
import calendar

import rospy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy
from occurrence_learning.region_observation_time import RegionObservationTimeManager


def week_of_month(tgtdate):
    """
        Assuming the first week of the month starts with the first monday of the month.
        If the month starts on any other day, then the first week will start from 0.
    """
    days_this_month = calendar.mdays[tgtdate.month]
    for i in range(1, days_this_month):
        d = datetime.date(tgtdate.year, tgtdate.month, i)
        if d.day - d.weekday() > 0:
            startdate = d
            break
    # now we can use the modulo 7 appraoch
    return (tgtdate - startdate).days // 7 + 1


class TrajectoryRegionKnowledge(object):

    def __init__(self, soma_map, soma_config, minute_interval=60):
        self.soma_map = soma_map
        self.soma_config = soma_config
        self.minute_interval = minute_interval
        self.region_knowledge = RegionObservationTimeManager(soma_map, soma_config)
        # Service and Proxy
        rospy.loginfo("Connecting to geospatial_store and roslog database...")
        self.gs = GeoSpatialStoreProxy('geospatial_store', 'soma')

    def estimate_trajectories_monthly(self, month=1, year=2015):
        """
            Estimate the number of trajectories in a region hourly each day for a
            month assuming that the robot sees the complete region when it sees the region
        """
        days_of_month = [
            datetime.datetime(year, month, i, 0, 0) for i in range(
                1, calendar.monthrange(year, month)[1]+1
            )
        ]
        monthly_trajectories = self.estimate_trajectories(days_of_month)
        return monthly_trajectories

    def estimate_trajectories_weekly(self, month=1, year=2015):
        """
            Estimate the number of trajectories in a region for a month, split by
            weeks assuming that the robot sees the complete region when it sees the
            region
        """
        monthly_traj = self.estimate_trajectories_monthly(month, year)
        month_weekly_traj = list()

        # construct the whole month weekly trajectory template
        first_week = week_of_month(datetime.date(year, month, 1))
        end_week = week_of_month(
            datetime.date(year, month, calendar.monthrange(year, month)[1])
        )
        if first_week == 0:
            end_week += 1
        for i in range(end_week):
            # create a template of weekly trajectory in the form {region{day[hour{mins}]}}
            weekly_traj = {
                i: {
                    j: [
                        {
                            k*self.minute_interval: -1 for k in range(1, (60/self.minute_interval)+1)
                        }
                    ]*24 for j in range(7)
                } for i in monthly_traj.keys()
            }
            month_weekly_traj.append(weekly_traj)

        for reg, dly_traj in monthly_traj.iteritems():
            for day, hourly_traj in dly_traj.iteritems():
                th_week = week_of_month(datetime.date(year, month, day))
                if first_week == 0:
                    th_week -= 1
                week_day = datetime.date(year, month, day).weekday()
                month_weekly_traj[th_week][reg][week_day] = hourly_traj

        return month_weekly_traj

    def estimate_trajectories(self, days):
        """
            Estimate the number of trajectories in a region for days,
            assuming that the robot sees the complete region when it sees the region
        """
        days_trajectories = dict()
        roi_region_daily = self.region_knowledge.load_from_mongo(
            days, self.minute_interval
        )
        if roi_region_daily[1] == 0:
            roi_region_daily = self.region_knowledge.calculate_region_observation_duration(
                days, self.minute_interval
            )
        else:
            roi_region_daily = roi_region_daily[0]
        trajectory_region_daily = self.obtain_number_of_trajectories(days)

        for day, dly_traj in trajectory_region_daily.iteritems():
            for hour, reg_traj in enumerate(dly_traj):
                for reg, mins_traj in reg_traj.iteritems():
                    days_trajectories = self._estimate_trajectories_by_minutes(
                        mins_traj, day, hour, reg, days_trajectories, roi_region_daily
                    )
        return days_trajectories

    # helper function for estimate_trajectories, because that func is too complex
    def _estimate_trajectories_by_minutes(
        self, mins_traj, day, hour, reg, days_trajectories, roi_region_daily
    ):
        for mins, traj in mins_traj.iteritems():
            if reg not in days_trajectories:
                days_trajectories[reg] = dict()
            if day not in days_trajectories[reg]:
                min_keys = mins_traj.keys()
                temp = list()
                for i in range(24):
                    temp.append({i: -1 for i in min_keys})
                days_trajectories[reg][day] = temp

            try:
                if roi_region_daily[day][reg][hour][mins] != 0:
                    multi_est = 3600 / float(len(mins_traj) * roi_region_daily[day][reg][hour][mins])
                    days_trajectories[reg][day][hour][mins] = math.ceil(multi_est * traj)
                else:
                    if traj > 0:
                        rospy.logwarn(
                            "%d trajectories are detected in region %s at day %d hour %d, but no info about robot being there" % (traj, reg, day, hour)
                        )
            except:
                rospy.logwarn("No info about robot on day %d" % day)

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


if __name__ == '__main__':
    rospy.init_node("trk_test")
    days = [
        # datetime.datetime(2015, 6, 27, 0, 0),
        datetime.datetime(2015, 6, 28, 0, 0),
        datetime.datetime(2015, 6, 29, 0, 0)
        # datetime.datetime(2015, 5, 6, 0, 0)
    ]
    # dk = TrajectoryRegionKnowledge("g4s", "g4s_test")
    dk = TrajectoryRegionKnowledge("rwth", "rwth_novelty", 20)
    # print dk.obtain_number_of_trajectories(days)
    print dk.estimate_trajectories(days)
