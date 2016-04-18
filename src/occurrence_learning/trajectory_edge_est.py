#!/usr/bin/env python

import math
import numpy
import pymongo
import calendar
import datetime
import scipy.spatial.distance as distance_calc

import rospy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy
from occurrence_learning.region_observation_time import RegionObservationTimeManager
from occurrence_learning.occurrence_learning_util import rotation_180_quaternion


class TrajectoryEstimate(object):

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

    def obtain_number_of_trajectories(self, days):
        """
            Obtaining the number of trajectories for each day in argument days,
            The number of Trajectories will be split into day, hour, region, and minutes.
            Trajectories = [day[hour[region[minutes]]]]
        """
        rospy.loginfo('Calculating total trajectories each day...')
        trajectory_several_days = dict()
        for i in days:
            daily_trajectories = self._trajectories_daily(
                calendar.timegm(i.timetuple())
            )
            trajectory_several_days.update({i.day: daily_trajectories})
        return trajectory_several_days

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


class TrajectoryEdgeEstimate(TrajectoryEstimate):

    def __init__(self, soma_map, soma_config, minute_interval=60):
        TrajectoryEstimate.__init__(self, soma_map, soma_config, minute_interval)
        self.mongo_client = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        )
        self.direction_coeff = 0.5
        self.uuid_in_edges = dict()

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
                    "soma_roi_id": {"$exists": "true"}
                }
                edges = self.gs.find_projection(query, {"soma_roi_id": 1, "loc": 1})
                edge = self._find_nearest_edge(geo_traj, edges)
                if edge not in trajectories_hourly and edge is not None:
                    trajectories_hourly[edge] = dict()
                if edge not in self.uuid_in_edges and edge is not None:
                    self.uuid_in_edges[edge] = list()
                self.uuid_in_edges[edge].append(geo_traj['uuid'])
                index = (end_hour - start) / 60
                if index not in trajectories_hourly[edge]:
                    trajectories_hourly[edge][index] = 0
                trajectories_hourly[edge][index] += 1
            start_hour += min_inter
            end_hour += min_inter
        return trajectories_hourly

    def _find_nearest_edge(self, geo_trajectory, geo_edges):
        poses = self._get_poses_from_message_store(geo_trajectory)
        edges = self._get_edges_from_message_store(geo_edges)

        distance_measures = list()
        result = None
        for edge in edges:
            line = self._get_sliced_edge(edge[1])
            dist_measure = self._get_distance_measure(poses, line)
            distance_measures.append((edge[0], dist_measure))
        distance_measures = sorted(distance_measures, key=lambda dist: dist[1])
        print geo_trajectory['uuid'], distance_measures
        if len(distance_measures) > 0:
            result = distance_measures[0][0]
        return result

    # motion prediction for moving objects
    def _get_distance_measure3(self, poses, edge):
        length = max([len(poses), len(edge)])
        sum_up = 0
        sum_up2 = 0
        for i in range(length):
            if i >= len(poses):
                ipose = -1
            else:
                ipose = i
            if i >= len(edge):
                iedge = -1
            else:
                iedge = i
            dist = (distance_calc.euclidean(poses[ipose][0], edge[iedge][0]))**2
            dir_measure = (
                numpy.dot(poses[ipose][1], edge[iedge][1]) / (
                    distance_calc.euclidean((0, 0, 0, 1), poses[ipose][1]) *
                    distance_calc.euclidean((0, 0, 0, 1), edge[iedge][1])
                )
            )
            temp = rotation_180_quaternion(edge[iedge][1])
            dir_measure2 = (
                numpy.dot(poses[ipose][1], temp) / (
                    distance_calc.euclidean((0, 0, 0, 1), poses[ipose][1]) *
                    distance_calc.euclidean((0, 0, 0, 1), temp)
                )
            )
            sum_up += dist + (self.direction_coeff * dir_measure)
            sum_up2 += dist + (self.direction_coeff * dir_measure2)
        return min([math.sqrt(sum_up / float(length)), math.sqrt(sum_up2 / float(length))])

    # from online trajectory clustering for anomalous events detection with
    # variance 1.
    def _get_distance_measure2(self, poses, edge):
        nearest_pos_edge_dists = list()
        nearest_pos_edge_dists2 = list()
        for pind, pval in enumerate(poses):
            temp = list()
            temp2 = list()
            for eind, edgval in enumerate(edge):
                dist = distance_calc.euclidean(pval[0], edgval[0])
                dir_measure = (
                    numpy.dot(pval[1], edgval[1]) / (
                        distance_calc.euclidean((0, 0, 0, 1), pval[1]) *
                        distance_calc.euclidean((0, 0, 0, 1), edgval[1])
                    )
                )
                rotation = rotation_180_quaternion(edgval[1])
                dir_measure2 = (
                    numpy.dot(pval[1], rotation) / (
                        distance_calc.euclidean((0, 0, 0, 1), pval[1]) *
                        distance_calc.euclidean((0, 0, 0, 1), rotation)
                    )
                )
                temp.append(dist + (self.direction_coeff * dir_measure))
                temp2.append(dist + (self.direction_coeff * dir_measure2))
            nearest_pos_edge_dists.append(sorted(temp)[0])
            nearest_pos_edge_dists2.append(sorted(temp2)[0])

        return min([
            sum(nearest_pos_edge_dists) / float(len(nearest_pos_edge_dists)),
            sum(nearest_pos_edge_dists2) / float(len(nearest_pos_edge_dists2))
        ])

    def _get_increment_step(self, poses, edge):
        average_step_distance = 0
        for ind, pose in enumerate(poses[:-1]):
            average_step_distance += distance_calc.euclidean(pose[0], poses[ind+1][0])
        average_step_distance /= (len(poses) - 1)
        average_edge_distance = 0
        for ind, edg in enumerate(edge[:-1]):
            average_edge_distance += distance_calc.euclidean(edg[0], edge[ind+1][0])
        # average_edge_distance = distance_calc.euclidean(edge[0][0], edge[1][0])
        average_edge_distance /= (len(edge) - 1)
        if average_step_distance >= average_edge_distance:
            increment = math.floor(average_step_distance / average_edge_distance)
            temp = average_step_distance - (increment * average_edge_distance)
            temp2 = ((increment + 1) * average_edge_distance) - average_step_distance
            if temp >= temp2:
                increment += 1
        else:
            increment = math.floor(average_edge_distance / average_step_distance)
            temp = average_edge_distance - (increment * average_step_distance)
            temp2 = ((increment + 1) * average_step_distance) - average_edge_distance
            if temp >= temp2:
                increment += 1
        return int(increment)

    # def _get_distance_measure(self, poses, edge):
    #     if len(poses) < len(edge):
    #         shorter = poses
    #         longer = edge
    #     else:
    #         shorter = edge
    #         longer = poses

    #     distance_measures = list()
    #     for ind, val in enumerate(longer):
    #         dist = 0
    #         dir_measure = 0
    #         dir_measure2 = 0
    #         for ind, val in enumerate(shorter):
    #             dist += (distance_calc.euclidean(val[0], longer[init_ind+ind][0]))**2
    #             dir_measure += (
    #                 numpy.dot(val[1], longer[init_ind+ind][1]) / (
    #                     distance_calc.euclidean((0, 0, 0, 1), val[1]) *
    #                     distance_calc.euclidean((0, 0, 0, 1), longer[init_ind+ind][1])
    #                 )
    #             )
    #             if len(shorter) == len(edge):
    #                 rotation = rotation_180_quaternion(val[1])
    #                 dir_measure2 += (
    #                     numpy.dot(longer[init_ind+ind][1], rotation) / (
    #                         distance_calc.euclidean((0, 0, 0, 1), longer[init_ind+ind][1]) *
    #                         distance_calc.euclidean((0, 0, 0, 1), rotation)
    #                     )
    #                 )
    #             else:
    #                 rotation = rotation_180_quaternion(longer[init_ind+ind][1])
    #                 dir_measure2 += (
    #                     numpy.dot(val[1], rotation) / (
    #                         distance_calc.euclidean((0, 0, 0, 1), val[1]) *
    #                         distance_calc.euclidean((0, 0, 0, 1), rotation)
    #                     )
    #                 )
    #         distance_measures.append(dist + (self.direction_coeff * dir_measure))
    #         distance_measures.append(dist + (self.direction_coeff * dir_measure2))
    #     return min(distance_measures) / len(shorter)

    # Jeremy's idea
    def _get_distance_measure(self, poses, edge):
        if len(poses) < len(edge):
            shorter = poses
            longer = edge
        else:
            shorter = edge
            longer = poses

        init_ind = 0
        distance_measures = list()
        while init_ind + len(shorter) <= len(longer):
            dist = 0
            dir_measure = 0
            dir_measure2 = 0
            for ind, val in enumerate(shorter):
                dist += (distance_calc.euclidean(val[0], longer[init_ind+ind][0]))**2
                dir_measure += (
                    numpy.dot(val[1], longer[init_ind+ind][1]) / (
                        distance_calc.euclidean((0, 0, 0, 1), val[1]) *
                        distance_calc.euclidean((0, 0, 0, 1), longer[init_ind+ind][1])
                    )
                )
                if len(shorter) == len(edge):
                    rotation = rotation_180_quaternion(val[1])
                    dir_measure2 += (
                        numpy.dot(longer[init_ind+ind][1], rotation) / (
                            distance_calc.euclidean((0, 0, 0, 1), longer[init_ind+ind][1]) *
                            distance_calc.euclidean((0, 0, 0, 1), rotation)
                        )
                    )
                else:
                    rotation = rotation_180_quaternion(longer[init_ind+ind][1])
                    dir_measure2 += (
                        numpy.dot(val[1], rotation) / (
                            distance_calc.euclidean((0, 0, 0, 1), val[1]) *
                            distance_calc.euclidean((0, 0, 0, 1), rotation)
                        )
                    )
            distance_measures.append(dist + (self.direction_coeff * dir_measure))
            distance_measures.append(dist + (self.direction_coeff * dir_measure2))
            init_ind += 1
        return min(distance_measures) / len(shorter)

    def _get_sliced_edge(self, edge):
        distance = distance_calc.euclidean(
            [edge[0][0][0], edge[0][0][1]], [edge[-1][0][0], edge[-1][0][1]]
        )
        # 1 foot = 30.48 cm, 1/10 foot = 3.048 cm
        total_chunks = math.ceil(distance / 0.03048)
        xs = numpy.linspace(edge[0][0][0], edge[-1][0][0], total_chunks)
        ys = numpy.linspace(edge[0][0][1], edge[-1][0][1], total_chunks)
        position = zip(xs, ys)
        orientation = int(total_chunks) * [
            (edge[0][1][0], edge[0][1][1], edge[0][1][2], edge[0][1][3])
        ]
        return zip(position, orientation)

    def _get_poses_from_message_store(self, trajectory):
        poses = list()
        total = self.mongo_client.message_store.people_trajectory.find(
            {"uuid": trajectory['uuid']}
        ).count()
        if int(total) < 1:
            rospy.logerr(
                "Trajectory %s can not be found in people_trajectory collection" % trajectory['uuid']
            )
            return None
        elif int(total) > 1:
            rospy.logwarn("Duplicate trajectory %s" % trajectory['uuid'])
        mongo_trajs = self.mongo_client.message_store.people_trajectory.find(
            {"uuid": trajectory['uuid']}
        ).limit(1)
        for traj in mongo_trajs:
            poses = [
                (
                    (j['pose']['position']['x'], j['pose']['position']['y']),  # position
                    (
                        j['pose']['orientation']['x'], j['pose']['orientation']['y'],
                        j['pose']['orientation']['z'], j['pose']['orientation']['w']
                    )  # orientation
                )
                for j in traj['trajectory']
            ]
        return poses

    def _get_edges_from_message_store(self, geo_edges):
        edges = list()
        for geo_edge in geo_edges:
            total = self.mongo_client.message_store.soma_roi.find(
                {"roi_id": geo_edge["soma_roi_id"], "config": self.soma_config, "map": self.soma_map}
            ).count()
            if total > 2:
                rospy.logerr("SOMA %s is not a line, this SOMA is ignored" % geo_edge["soma_roi_id"])
                continue
            message_edge = self.mongo_client.message_store.soma_roi.find(
                {"roi_id": geo_edge["soma_roi_id"], "config": self.soma_config, "map": self.soma_map}
            )
            edge = list()
            for i in message_edge:
                edge.append(
                    (
                        (i["pose"]["position"]["x"], i["pose"]["position"]["y"]),
                        (
                            i["pose"]["orientation"]['x'], i['pose']['orientation']['y'],
                            i['pose']['orientation']['z'], i['pose']['orientation']['w']
                        )
                    )
                )
            edges.append((geo_edge["soma_roi_id"], edge))
        return edges
