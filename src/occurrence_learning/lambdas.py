#!/usr/bin/env python

# import datetime
from scipy.stats import gamma

import rospy
from occurrence_learning.trajectory_region_knowledge import TrajectoryRegionKnowledge


class Lambda(object):

    def __init__(self, interval=1):
        self.reset()
        self.interval = interval

    def reset(self):
        self.beta = 0.01
        self.alpha = 0.01
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

    def __init__(self, regions, days=[0, 1, 2, 3, 4, 5, 6], interval=1):
        """
            Initialize a set of lambdas classified by regions, days, and hours respectively.
            Hours will be set by default to [0-23]. Days are set as days of a week [0-6] where
            Monday is 0.
        """
        self.lambdas = {i: {j: list() for j in days} for i in regions}
        for reg, daily_lambda in self.lambdas.iteritems():
            for day, hourly_lambda in daily_lambda.iteritems():
                for i in range(24):
                    hourly_lambda.append(Lambda(interval))

    def update_lambdas(self, reg_traj):
        """
            Update lambdas based on a dictionary consisting regions with daily trajectories in
            the form of dictionary as well. In each daily trajectories, trajectories are classified
            based on hours in the form of lists
        """
        for reg, daily_traj in reg_traj.iteritems():
            for day, hourly_traj in daily_traj.iteritems():
                for hour, traj in enumerate(hourly_traj):
                    if traj != -1:
                        self.lambdas[reg][day][hour].update_lambda([traj])


if __name__ == '__main__':
    rospy.init_node("lambdas_test")
    # trk = TrajectoryRegionKnowledge("rwth", "rwth_novelty")
    trk = TrajectoryRegionKnowledge("g4s", "g4s_test")
    # reg_trajs = trk.estimate_trajectories_weekly(6, 2015)
    reg_trajs = trk.estimate_trajectories_weekly(5, 2015)
    # print reg_trajs[-1]

    lambdas = Lambdas(reg_trajs[0].keys())
    for i in reg_trajs:
        lambdas.update_lambdas(i)
    for reg, daily_lambda in lambdas.lambdas.iteritems():
        for day, hourly_lambda in daily_lambda.iteritems():
            for hour, lmbd in enumerate(hourly_lambda):
                print "Reg: %s, Day: %d, Hour: %d, Lambda: %f" % (reg, day, hour, lmbd.gamma_map)
