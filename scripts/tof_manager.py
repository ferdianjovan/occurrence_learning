#!/usr/bin/env python

import rospy
import datetime
import argparse
import actionlib
from occurrence_learning.msg import OccurrenceRate
from mongodb_store.message_store import MessageStoreProxy
from occurrence_learning.trajectory_region_est import TrajectoryRegionEstimate
from occurrence_learning.region_observation_time import RegionObservationTimeManager
from occurrence_learning.occurrence_learning_util import trajectory_estimate_for_date
from occurrence_learning.trajectory_occurrence_freq import TrajectoryOccurrenceFrequencies
from occurrence_learning.msg import OccurrenceRateLearningResult, OccurrenceRateLearningAction
from occurrence_learning.srv import TrajectoryOccurrenceRate, TrajectoryOccurrenceRateResponse


class TrajectoryOccurrenceRateManager(object):

    def __init__(
        self, name, soma_map, soma_config, minute_interval,
        window_interval, collection="occurrence_rates"
    ):
        self.soma_map = soma_map
        self.soma_config = soma_config
        self.minute_interval = minute_interval
        self.window_interval = window_interval
        self.rotm = RegionObservationTimeManager(soma_map, soma_config)
        self.tre = TrajectoryRegionEstimate(soma_map, soma_config, minute_interval)
        self.tof = TrajectoryOccurrenceFrequencies(
            soma_map, soma_config, minute_interval=minute_interval,
            window_interval=window_interval
        )

        rospy.loginfo("Connect to database collection %s..." % collection)
        self._ms = MessageStoreProxy(collection=collection)
        rospy.loginfo("Create a service %s/service..." % name)
        self.service = rospy.Service(name+'/service', TrajectoryOccurrenceRate, self.srv_cb)
        rospy.loginfo("Create an action server %s..." % name)
        self._as = actionlib.SimpleActionServer(
            name, OccurrenceRateLearningAction, execute_cb=self.execute, auto_start=False
        )
        self._as.start()

    def execute(self, goal):
        temp_start_time = rospy.Time.now()
        curr_date = datetime.date.fromtimestamp(rospy.Time.now().secs)
        # curr_date = datetime.date(2016, 3, 9)
        curr_date = datetime.datetime(curr_date.year, curr_date.month, curr_date.day, 0, 0)
        prev_date = curr_date - datetime.timedelta(hours=24)
        self.rotm.calculate_region_observation_duration([prev_date, curr_date], self.minute_interval)
        self.rotm.store_to_mongo()
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            return

        curr_traj_est = self.tre.estimate_trajectories_daily(
            [curr_date.day], curr_date.month, curr_date.year
        )
        curr_traj_est = trajectory_estimate_for_date(curr_traj_est, curr_date)
        prev_traj_est = self.tre.estimate_trajectories_daily(
            [prev_date.day], prev_date.month, prev_date.year
        )
        prev_traj_est = trajectory_estimate_for_date(prev_traj_est, prev_date)
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            return

        self.tof.update_tof_daily(curr_traj_est, prev_traj_est, curr_date)
        self.tof.store_tof()
        temp_end_time = rospy.Time.now()
        rospy.loginfo("Time needed to complete this is %d" % (temp_end_time - temp_start_time).secs)
        self._as.set_succeeded(OccurrenceRateLearningResult())

    def srv_cb(self, msg):
        rospy.loginfo("Retrieving trajectory occurrence frequencies from database...")
        print msg
        query = {
            "soma": self.soma_map, "soma_config": self.soma_config,
            "region_id": msg.region_id, "day": msg.day,
            "end_hour": msg.hour, "end_minute": msg.minute
        }
        logs = self._ms.query(OccurrenceRate._type, message_query=query)
        occurrence_rate = 0.0
        counter = 0
        for log in logs:
            occurrence_rate += log[0].occurrence_rate
            counter += 1
        if counter == 0:
            counter += 1
        rospy.loginfo(
            "Occurrence Rate is obtained, which has value %.2f" % (occurrence_rate / float(counter))
        )
        return TrajectoryOccurrenceRateResponse(occurrence_rate / float(counter))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog="occurrence_rate_server")
    parser.add_argument('soma_map', help="Soma Map")
    parser.add_argument('soma_config', help="Soma Config")
    parser.add_argument('minute_interval', help="Minute Interval")
    parser.add_argument('window_interval', help="Window Interval")
    args = parser.parse_args()

    rospy.init_node("occurrence_rate_server")
    TrajectoryOccurrenceRateManager(
        rospy.get_name(), args.soma_map, args.soma_config,
        int(args.minute_interval), int(args.window_interval)
    )
    rospy.spin()
