#!/usr/bin/env python

import math
import datetime
import calendar

import rospy
import pymongo
from tf.transformations import euler_from_quaternion
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


# assuming the first week of the month starts with the first monday of the month
# if the month starts on any other day, then the first week will start from 0
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
        self.monthly_trajectory_est = dict()
        self.soma_map = soma_map
        self.soma_config = soma_config
        self.minute_interval = minute_interval
        # Service and Proxy
        rospy.loginfo("Connecting to geospatial_store and roslog database...")
        self.gs = GeoSpatialStoreProxy('geospatial_store', 'soma')
        # self.roslog = GeoSpatialStoreProxy("roslog", "robot_pose")
        self.roslog = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        ).roslog.robot_pose

    # get regional knowledge for each hour every day
    def obtain_regional_knowledge(self, days):
        """
            Returns the ROIs the robot has monitor at each logged robot pose
            for particular days
        """
        rospy.loginfo('Getting region knowledge from roslog...')
        roi_region_daily = dict()
        for i in days:
            end_date = i + datetime.timedelta(hours=24)
            roi_region_hourly = self._obtain_regional_knowledge(i, end_date, self.minute_interval)
            roi_region_daily.update({i.day: roi_region_hourly})
        return roi_region_daily

    # hidden function for the obtain_regional_knowledge function
    def _obtain_regional_knowledge(self, start_date, end_date, minute_interval=60):
        sampling_rate = 10
        min_in_hour = 60
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
                                i*minute_interval: 0 for i in range(1, (min_in_hour/minute_interval)+1)
                            }
                            temp.append(group_mins)
                        roi_temp_list[region] = temp
                    group_min = minute_interval
                    while minute >= group_min:
                        group_min += minute_interval
                    roi_temp_list[region][hour][group_min] += 1

        roi_temp_list = self._normalizing_region_knowledge(roi_temp_list, sampling_rate)
        rospy.loginfo("Region knowledge for the query is " + str(roi_temp_list))
        return roi_temp_list

    # checking if all the region knowledge is capped by 3600 (total seconds).
    # if it is not, then normalizing is applied
    def _normalizing_region_knowledge(self, roi_temp_list, sampling_rate):
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

    # estimate the number of trajectories in a region hourly each day for a
    # month assuming that the robot sees the complete region when it sees the region
    def estimate_trajectories_monthly(self, month=1, year=2015):
        days_of_month = [
            datetime.datetime(year, month, i, 0, 0) for i in range(
                1, calendar.monthrange(year, month)[1]+1
            )
        ]
        monthly_trajectories = self.estimate_trajectories(days_of_month)
        self.monthly_trajectory_est = monthly_trajectories
        return monthly_trajectories

    # estimate the number of trajectories in a region for a month, split by
    # weeks assuming that the robot sees the complete region when it sees the
    # region
    def estimate_trajectories_weekly(self, month=1, year=2015):
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
        days_trajectories = dict()
        roi_region_daily = self.obtain_regional_knowledge(days)
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
                    # days_trajectories[reg][day][hour][mins] = -1
            except:
                rospy.logwarn("No info about robot on day %d" % day)

        return days_trajectories

    # trajectories = [day[hour[region]]]
    def obtain_number_of_trajectories(self, days):
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

    # count the number of trajectories in areas defined by soma map and soma
    # config that is restricted by the query.
    # this function aims to reduce the processing time if we compare with using
    # trajectory query service.
    def _get_num_traj_by_epoch(self, start, end, min_inter=3600):
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
        # datetime.datetime(2015, 6, 29, 0, 0)
        # datetime.datetime(2015, 5, 6, 0, 0)
    ]
    # dk = TrajectoryRegionKnowledge("g4s", "g4s_test")
    dk = TrajectoryRegionKnowledge("rwth", "rwth_novelty", 20)
    # print dk.obtain_regional_knowledge(days)
    # print dk.obtain_number_of_trajectories(days)
    print dk.estimate_trajectories(days)
