#!/usr/bin/env python

import sys
from scipy.stats import gamma

import rospy
from occurrence_learning.msg import OccurrenceRate
from mongodb_store.message_store import MessageStoreProxy
from occurrence_learning.trajectory_region_knowledge import TrajectoryRegionKnowledge


class Lambda(object):

    def __init__(self, interval=1):
        self.reset()
        self.interval = interval

    def reset(self):
        self.beta = 1.0
        self.alpha = 0.00001
        self.gamma_map = 0.0
        self.gamma_mean = 0.0

    def update_lambda(self, data):
        self.alpha += sum(data)
        self.beta += len(data) * self.interval
        self.gamma_map = self._gamma_mode(self.alpha, self.beta)
        self.gamma_mean = gamma.mean(self.alpha, scale=1/float(self.beta))

    def _gamma_mode(self, alpha, beta):
        if alpha >= 1:
            return (alpha - 1) / float(beta)
        else:
            return -1.0


class TrajectoryOccurrenceFrequencies(object):

    def __init__(self, soma_map, soma_config, minute_interval=60, periodic_type="weekly"):
        """
            Initialize a set of trajectory occurrence frequency classified by regions, days, hours, minute intervals respectively.
            Hours will be set by default to [0-23]. Days are set as days of a week [0-6] where Monday is 0.
            Argument periodic_type can be set either 'weekly' or 'monthly'
        """
        self.soma = soma_map
        self.soma_config = soma_config
        self.periodic_type = periodic_type
        if self.periodic_type == "weekly":
            self.periodic_days = [i for i in range(7)]
        else:
            self.periodic_days = [i for i in range(31)]
        self.minute_interval = minute_interval
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
            within_interval = (i[0].end_hour == i[0].start_hour+1) and (i[0].end_minute - i[0].start_minute) % 60 == self.minute_interval
            if same_hour or within_interval:
                if i[0].region_id not in self.tof:
                    self.init_region_tof(i[0].region_id)
                minute = (i[0].start_minute + self.minute_interval)
                if minute in self.tof[i[0].region_id][i[0].day][i[0].start_hour]:
                    self.tof[i[0].region_id][i[0].day][i[0].start_hour][minute].alpha = i[0].occurrence_shape
                    self.tof[i[0].region_id][i[0].day][i[0].start_hour][minute].beta = i[0].occurrence_scale
                    self.tof[i[0].region_id][i[0].day][i[0].start_hour][minute].gamma_map = i[0].occurrence_rate
        rospy.loginfo("Loading is complete...")

    def update_tof(self, reg_traj):
        """
            Update tof based on a dictionary consisting regions with daily trajectories in
            the form of dictionary as well. In each daily trajectories, trajectories are classified
            based on hours in the form of lists. Each hour is splitted into minute interval
            i.e. {20, 40, 60} with the number of trajectories during that interval
        """
        rospy.loginfo("Updating tof...")
        for reg, daily_traj in reg_traj.iteritems():
            for day, hourly_traj in daily_traj.iteritems():
                for hour, mins_traj in enumerate(hourly_traj):
                    for mins, traj in mins_traj.iteritems():
                        if traj != -1:
                            if reg not in self.tof:
                                self.init_region_tof(reg)
                            self.tof[reg][day][hour][mins].update_lambda([traj])
        rospy.loginfo("Updating is complete...")

    def init_region_tof(self, reg):
        """
            Initialize trajectory occurrence frequency for one whole region.
            {region: daily_tof}
        """
        daily_tof = dict()
        for j in self.periodic_days:
            hourly_tof = list()
            for i in range(24):
                temp = {
                    i*self.minute_interval: Lambda(
                        self.minute_interval/float(60)
                    ) for i in range(1, (60/self.minute_interval)+1)
                }
                hourly_tof.append(temp)
            daily_tof.update({j: hourly_tof})
        self.tof.update({reg: daily_tof})

    def store_tof(self):
        """
            Store self.tof into mongodb in occurrence_rates collection
        """
        rospy.loginfo("Storing to database...")
        for reg, daily_tof in self.tof.iteritems():
            for day, hourly_tof in daily_tof.iteritems():
                for hour, minutely_tof in enumerate(hourly_tof):
                    for mins, lmbd in minutely_tof.iteritems():
                        self._store_tof(reg, day, hour, mins, lmbd)
        rospy.loginfo("Storing is complete...")

    # helper function of store_tof
    def _store_tof(self, reg, day, hour, mins, lmbd):
        start_minute = mins - self.minute_interval
        occu_msg = OccurrenceRate(
            self.soma, self.soma_config, reg.encode("ascii"), day, hour, hour, start_minute,
            mins, lmbd.alpha, lmbd.beta, lmbd.gamma_map, self.periodic_type
        )
        query = {
            "soma": self.soma, "soma_config": self.soma_config,
            "region_id": reg.encode("ascii"), "day": day, "start_hour": hour,
            "end_hour": hour, "start_minute": start_minute, "end_minute": mins,
            "periodic_type": self.periodic_type
        }
        if lmbd.gamma_map > 0:
            if len(self.ms.query(OccurrenceRate._type, message_query=query)) > 0:
                self.ms.update(occu_msg, message_query=query)
            else:
                self.ms.insert(occu_msg)


if __name__ == '__main__':
    rospy.init_node("occurrence_rate_learning")

    if len(sys.argv) < 6:
        rospy.logerr("usage: tof soma config month year minute_interval")
        sys.exit(2)

    interval = int(sys.argv[5])
    trk = TrajectoryRegionKnowledge(sys.argv[1], sys.argv[2], interval)
    reg_trajs = trk.estimate_trajectories_weekly(int(sys.argv[3]), int(sys.argv[4]))

    tof = TrajectoryOccurrenceFrequencies(sys.argv[1], sys.argv[2], minute_interval=interval)
    tof.load_tof()
    for i in reg_trajs:
        tof.update_tof(i)

    tof.store_tof()
