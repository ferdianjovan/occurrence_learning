#!/usr/bin/env python

import datetime
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


class Lambdas(object):

    def __init__(self, periodic_type="weekly", minute_interval=60):
        """
            Initialize a set of lambdas classified by regions, days, and hours respectively.
            Hours will be set by default to [0-23]. Days are set as days of a week [0-6] where
            Monday is 0.
        """
        self.periodic_type = periodic_type
        if self.periodic_type == "weekly":
            self.periodic_days = [i for i in range(7)]
        else:
            self.periodic_days = [i for i in range(31)]
        self.minute_interval = interval
        self.ms = MessageStoreProxy(collection="occurrence_rates")
        self.lambdas = dict()

    # load lambdas from mongo occurrence_rates
    def load_lambdas(self, soma, soma_config):
        query = {
            "soma": soma, "soma_config": soma_config,
            "periodic_type": self.periodic_type
        }
        logs = self.ms.query(OccurrenceRate._type, message_query=query)
        if len(logs) == 0:
            rospy.logwarn(
                "No data for %s with config %s and periodicity type %s" % (
                    soma, soma_config, self.periodic_type
                )
            )
            return

        for i in logs:
            if i[0].region_id not in self.lambdas:
                self.init_region_lambdas(i[0].region_id)
            self.lambdas[i[0].region_id][i[0].day][i[0].hour][i[0].end_minute].alpha = i[0].occurrence_shape
            self.lambdas[i[0].region_id][i[0].day][i[0].hour][i[0].end_minute].beta = i[0].occurrence_scale
            self.lambdas[i[0].region_id][i[0].day][i[0].hour][i[0].end_minute].gamma_map = i[0].occurrence_rate
            rospy.loginfo(
                "Reg: %s, Day: %d, Hour: %d, Mins: %d, Lambda: %f" % (
                    i[0].region_id, i[0].day, i[0].hour, i[0].end_minute,
                    i[0].occurrence_rate
                )
            )

    # update lambdas from regional trajectories
    def update_lambdas(self, reg_traj):
        """
            Update lambdas based on a dictionary consisting regions with daily trajectories in
            the form of dictionary as well. In each daily trajectories, trajectories are classified
            based on hours in the form of lists
        """
        for reg, daily_traj in reg_traj.iteritems():
            for day, hourly_traj in daily_traj.iteritems():
                for hour, mins_traj in enumerate(hourly_traj):
                    for mins, traj in mins_traj.iteritems():
                        if traj != -1:
                            if reg not in self.lambdas:
                                self.init_region_lambdas(reg)
                            self.lambdas[reg][day][hour][mins].update_lambda([traj])

    # initialize {region: daily_lambda} for region reg
    def init_region_lambdas(self, reg):
        daily_lambda = dict()
        for j in self.periodic_days:
            hourly_lambda = list()
            for i in range(24):
                temp = {
                    i*self.minute_interval: Lambda(
                        self.minute_interval/float(60)
                    ) for i in range(1, (60/self.minute_interval)+1)
                }
                hourly_lambda.append(temp)
            daily_lambda.update({j: hourly_lambda})
        self.lambdas.update({reg: daily_lambda})

    # store the occurrence rate in database
    def store_lambdas(self, soma, soma_config):
        for reg, daily_lambda in lambdas.lambdas.iteritems():
            for day, hourly_lambda in daily_lambda.iteritems():
                for hour, mins_lmbd in enumerate(hourly_lambda):
                    for mins, lmbd in mins_lmbd.iteritems():
                        rospy.loginfo(
                            "Reg: %s, Day: %d, Hour: %d, Mins: %d, Lambda: %f" % (reg, day, hour, mins, lmbd.gamma_map)
                        )
                        self._store_lambdas(
                            soma, soma_config, reg, day, hour, mins, lmbd
                        )

    # helper function of store_lambdas
    def _store_lambdas(self, soma, soma_config, reg, day, hour, mins, lmbd):
        start_minute = mins - self.minute_interval
        occu_msg = OccurrenceRate(
            soma, soma_config, reg.encode("ascii"), day, hour, start_minute,
            mins, lmbd.alpha, lmbd.beta, lmbd.gamma_map, self.periodic_type
        )
        query = {
            "soma": soma, "soma_config": soma_config,
            "region_id": reg.encode("ascii"), "day": day, "hour": hour,
            "start_minute": start_minute, "end_minute": mins,
            "periodic_type": self.periodic_type
        }
        if lmbd.gamma_map > 0:
            meta = {'inserted_at': datetime.datetime.now()}
            if len(self.ms.query(OccurrenceRate._type, message_query=query)) > 0:
                self.ms.update(occu_msg, message_query=query, meta=meta)
            else:
                self.ms.insert(occu_msg, meta)


if __name__ == '__main__':
    rospy.init_node("occurrence_rate_learning")
    interval = 20
    trk = TrajectoryRegionKnowledge("rwth", "rwth_novelty", interval)
    # trk = TrajectoryRegionKnowledge("g4s", "g4s_test", interval)
    reg_trajs = trk.estimate_trajectories_weekly(6, 2015)
    # reg_trajs = trk.estimate_trajectories_weekly(5, 2015)

    lambdas = Lambdas(minute_interval=interval)
    lambdas.load_lambdas("rwth", "rwth_novelty")
    for i in reg_trajs:
        lambdas.update_lambdas(i)

    lambdas.store_lambdas("rwth", "rwth_novelty")
