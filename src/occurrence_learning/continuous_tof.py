#!/usr/bin/env python

import sys
import math
import string
import datetime

import rospy
from occurrence_learning.msg import OccurrenceRate
from mongodb_store.message_store import MessageStoreProxy
from occurrence_learning.trajectory_occurrence_freq import Lambda
from occurrence_learning.trajectory_region_knowledge import TrajectoryRegionKnowledge


class ContinuousTOF(object):

    def __init__(
        self, soma_map, soma_config, minute_interval=60,
        window_interval=10, periodic_type="weekly"
    ):
        """
            Initialize a set of trajectory occurrence frequency classified by regions, days, hours, minute intervals respectively.
            Hours will be set by default to [0-23]. Days are set as days of a week [0-6] where Monday is 0.
            Argument periodic_type can be set either 'weekly' or 'monthly'.
        """
        self.soma = soma_map
        self.soma_config = soma_config
        self.periodic_type = periodic_type
        if self.periodic_type == "weekly":
            self.periodic_days = [i for i in range(7)]
        else:
            self.periodic_days = [i for i in range(31)]
        self.minute_interval = minute_interval
        self.window_interval = window_interval
        self.ms = MessageStoreProxy(collection="occurrence_rates")
        self.tof = dict()

    def load_tof(self):
        """
            Load trajectory occurrence frequency from mongodb occurrence_rates collection.
        """
        rospy.loginfo("Retrieving continuous tof from database...")
        query = {
            "soma": self.soma, "soma_config": self.soma_config,
            "periodic_type": self.periodic_type
        }
        logs = self.ms.query(OccurrenceRate._type, message_query=query)
        if len(logs) == 0:
            rospy.logwarn(
                "No data for %s with config %s and periodicity type %s" % (
                    self.soma, self.soma_config, self.periodic_type
                )
            )
            return

        for i in logs:
            same_hour = (i[0].end_hour == i[0].start_hour)
            within_interval = (i[0].end_hour == i[0].start_hour+1) and (i[0].end_minute - i[0].start_minute) % 60 == self.window_interval
            if same_hour or within_interval:
                if i[0].region_id not in self.tof:
                    self.init_region_tof(i[0].region_id)
                key = "%s-%s" % (
                    datetime.time(i[0].start_hour, i[0].start_minute).isoformat()[:-3],
                    datetime.time(i[0].end_hour, i[0].end_minute).isoformat()[:-3]
                )
                if key in self.tof[i[0].region_id][i[0].day]:
                    self.tof[i[0].region_id][i[0].day][key].alpha = i[0].occurrence_shape
                    self.tof[i[0].region_id][i[0].day][key].beta = i[0].occurrence_scale
                    self.tof[i[0].region_id][i[0].day][key].gamma_map = i[0].occurrence_rate
        rospy.loginfo("Retrieving is complete...")

    def update_tof(self, reg_traj):
        """
            Update continuous tof based on a dictionary consisting regions with daily trajectories in
            the form of dictionary as well. In each daily trajectories, trajectories are classified
            based on hours in the form of lists. Each hour is splitted into minute interval
            i.e. {20, 40, 60} with the number of trajectories during that interval
        """
        rospy.loginfo("Updating continuous tof...")
        for reg, daily_traj in reg_traj.iteritems():
            for day, hourly_traj in daily_traj.iteritems():
                prev_day_n_min_traj = self._get_previous_n_minutes_trajs(
                    day, daily_traj
                )
                self._update_tof(reg, day, hourly_traj, prev_day_n_min_traj)
        rospy.loginfo("Updating is complete...")

    def _update_tof(self, reg, day, hourly_traj, prev_n_min_traj):
        length = (self.window_interval / self.minute_interval)
        temp_data = prev_n_min_traj + [-1]
        pointer = length - 1
        for hour, mins_traj in enumerate(hourly_traj):
            minutes = sorted(mins_traj)
            for mins in minutes:
                traj = mins_traj[mins]
            # for mins, traj in mins_traj.iteritems():
                temp_data[pointer % length] = traj
                pointer += 1
                # if pointer <= length:
                #     continue
                if reg not in self.tof:
                    self.init_region_tof(reg)
                if sum(temp_data) == (-1 * length):
                    continue
                else:
                    total_traj = length / float(length + sum([i for i in temp_data if i == -1]))
                    total_traj = math.ceil(total_traj * sum([i for i in temp_data if i != -1]))
                temp = [
                    hour + (mins - self.window_interval) / 60,
                    (mins - self.window_interval) % 60
                ]
                hour = (hour + (mins/60)) % 24
                key = "%s-%s" % (
                    datetime.time(temp[0] % 24, temp[1]).isoformat()[:-3],
                    datetime.time(hour, mins % 60).isoformat()[:-3]
                )
                self.tof[reg][day][key].update_lambda([total_traj])

    def _get_previous_n_minutes_trajs(self, day, daily_traj):
        """
           get (n-1) minutes window interval of trajectories from the previous day.
           If the day is zero, then the previous day is the last day within the set periodic days.
        """
        temp_n_min_trajs = list()
        prev_day_trajs = daily_traj[(day - 1) % len(self.periodic_days)]

        for i in range(
            60 - self.window_interval + self.minute_interval,
            60, self.minute_interval
        ):
            temp_n_min_trajs.append(prev_day_trajs[23][i])

        return temp_n_min_trajs

    def init_region_tof(self, reg):
        """
            Initialize trajectory occurrence frequency for one whole region.
            {region: daily_tof}
        """
        daily_tof = dict()
        for j in self.periodic_days:
            hourly_tof = dict()
            # for i in range(24 * (60 / self.minute_interval)):
            for i in range(24 * (60 / self.minute_interval) + 1):
                hour = i / (60 / self.minute_interval)
                minute = (self.minute_interval * i) % 60
                temp = [
                    # hour + (minute + self.window_interval) / 60,
                    # (minute + self.window_interval) % 60
                    hour + (minute - self.window_interval) / 60,
                    (minute - self.window_interval) % 60
                ]
                key = "%s-%s" % (
                    datetime.time(temp[0] % 24, temp[1]).isoformat()[:-3],
                    datetime.time(hour % 24, minute).isoformat()[:-3]
                )
                hourly_tof.update(
                    {key: Lambda(self.window_interval / float(60))}
                )
                # if temp[0] > 23:
                #     break

            daily_tof.update({j: hourly_tof})
        self.tof.update({reg: daily_tof})

    def store_tof(self):
        """
            Store self.tof into mongodb in occurrence_rates collection
        """
        rospy.loginfo("Storing to database...")
        for reg, daily_tof in self.tof.iteritems():
            for day, hourly_tof in daily_tof.iteritems():
                for window_time, lmbd in hourly_tof.iteritems():
                        self._store_tof(reg, day, window_time, lmbd)
        rospy.loginfo("Storing is complete...")

    # helper function of store_tof
    def _store_tof(self, reg, day, window_time, lmbd):
        start_time, end_time = string.split(window_time, "-")
        start_hour, start_min = string.split(start_time, ":")
        end_hour, end_min = string.split(end_time, ":")
        occu_msg = OccurrenceRate(
            self.soma, self.soma_config, reg.encode("ascii"), day,
            int(start_hour), int(end_hour), int(start_min), int(end_min),
            lmbd.alpha, lmbd.beta, lmbd.gamma_map, self.periodic_type
        )
        query = {
            "soma": self.soma, "soma_config": self.soma_config,
            "region_id": reg, "day": day, "start_hour": int(start_hour),
            "end_hour": int(end_hour), "start_minute": int(start_min), "end_minute": int(end_min),
            "periodic_type": self.periodic_type
        }
        if lmbd.gamma_map > 0:
            if len(self.ms.query(OccurrenceRate._type, message_query=query)) > 0:
                self.ms.update(occu_msg, message_query=query)
            else:
                self.ms.insert(occu_msg)


if __name__ == '__main__':
    rospy.init_node("continuous_tof")

    if len(sys.argv) < 7:
        rospy.logerr("usage: continuous_tof soma config month year minute_interval window_time")
        sys.exit(2)

    # the interval minute must be the same for Trajectory Region Knowledge and
    # Continuous TOF
    interval = int(sys.argv[5])
    window = int(sys.argv[6])
    trk = TrajectoryRegionKnowledge(sys.argv[1], sys.argv[2], interval)
    reg_trajs = trk.estimate_trajectories_weekly(int(sys.argv[3]), int(sys.argv[4]))

    tof = ContinuousTOF(sys.argv[1], sys.argv[2], minute_interval=interval, window_interval=window)
    tof.load_tof()
    for i in reg_trajs:
        tof.update_tof(i)
    tof.store_tof()
