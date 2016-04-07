#!/usr/bin/env python

import sys
import numpy
import random
import datetime

import rospy
from occurrence_learning.occurrence_learning_util import trajectory_estimate_for_date
from occurrence_learning.trajectory_periodicity import TrajectoryPeriodicity
from occurrence_learning.trajectory_occurrence_freq import TrajectoryOccurrenceFrequencies as TOF


class SyntheticWave(object):

    def __init__(self, minute_interval):
        self.frequencies = [
            1, 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43,
            47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109
        ]
        self.length = 7 * 24 * 60
        self.regions = ["1"]
        self.minutes = [
            i * minute_interval for i in range(1, (60/minute_interval) + 1)
        ]

    def create_wave(self, frequencies, length, gauss=False):
        random.seed()
        amplitudes = sorted(
            [i*0.1 for i in range(1, len(self.frequencies)+1)], reverse=True
        )
        x = numpy.linspace(0.0, length, length)
        wave = 50.0 + x * 0
        for ind, freq in enumerate(frequencies):
            wave += amplitudes[ind] * numpy.cos(freq * 2.0 * numpy.pi * x)
        if gauss:
            wave += numpy.random.normal(0, 40, length)
        return wave

    def add_noise_to(self, series, mu=1, sigma=1):
        return [i*random.gauss(mu, sigma) for i in series]

    def _get_one_week_wave(self, dates, week_wave=dict(), noise=False):
        # First value in dates must be monday
        # Dates that are already in the data ({reg}) will be ignored
        wave = self.create_wave(self.frequencies, self.length, noise)
        for region in self.regions:
            if region not in week_wave:
                week_wave[region] = dict()
            for ind, date in enumerate(dates[:7]):
                if date not in week_wave[region]:
                    temp = list()
                    for hour in range(24):
                        list_minutes = dict()
                        for minute in self.minutes:
                            list_minutes.update(
                                {
                                    minute: wave[(ind * 24 * 60) + (hour * 60) + (minute-1)]
                                }
                            )
                        temp.append(list_minutes)
                    week_wave[region][date] = temp
        return week_wave

    def get_one_month_synthetic_wave(self, noise=False):
        dates = [i for i in range(4, 32)]
        result = self._get_one_week_wave([27, 28, 29, 30, 1, 2, 3], noise=noise)
        temp = {
            region: {
                date: daily_data for date, daily_data in val.iteritems() if date in [1, 2, 3]
            } for region, val in result.iteritems()
        }
        result = temp
        for i in [4, 11, 18, 25]:
            week_dates = dates[dates.index(i):dates.index(i)+7]
            result = self._get_one_week_wave([j for j in week_dates], result, noise)
        return result

    def get_one_week_synthetic_wave(self, noise=False):
        result = self._get_one_week_wave([i for i in range(4, 11)], noise=noise)
        temp = {
            region: {
                date: daily_data for date, daily_data in val.iteritems() if date in [10]
            } for region, val in result.iteritems()
        }
        result = temp
        return self._get_one_week_wave([i for i in range(11, 18)], result, noise)


if __name__ == '__main__':
    rospy.init_node("synthetic_tof")

    if len(sys.argv) < 4:
        rospy.logerr("usage: synthetic minute_interval window_interval store(0)/test(1)")
        sys.exit(2)

    interval = int(sys.argv[1])
    window = int(sys.argv[2])

    sw = SyntheticWave(interval)
    if not int(sys.argv[3]):
        waves = sw.get_one_month_synthetic_wave(True)
        tof = TOF("synthetic", "synthetic_config", interval, window)
        tof.load_tof()
        for i in range(4, 31+1):
            prev_traj_est = trajectory_estimate_for_date(
                waves, datetime.date(2015, 5, i-1)
            )
            curr_traj_est = trajectory_estimate_for_date(
                waves, datetime.date(2015, 5, i)
            )
            tof.update_tof_daily(
                curr_traj_est, prev_traj_est, datetime.date(2015, 5, i)
            )
        tof.store_tof()
    else:
        waves = sw.get_one_week_synthetic_wave(True)
        tp = TrajectoryPeriodicity("synthetic", "synthetic_config", interval, window)
        inp = raw_input("MSE(0) or Prediction MSE(1): ")
        if int(inp) == 0:
            rospy.loginfo("Start model selection...")
            tp.model_selection(waves, 5, 2015, True)
            rospy.loginfo("End model selection...")
            # tp.addition_technique = True
            for region in tp.regions:
                tp.calculate_mse("1", waves["1"], 5, 2015)

            rospy.loginfo("Start model selection...")
            tp.model_selection(waves, 5, 2015, False)
            rospy.loginfo("End model selection...")
            # tp.addition_technique = False
            for region in tp.regions:
                tp.calculate_mse("1", waves["1"], 5, 2015)
        else:
            rospy.loginfo("Start model selection...")
            tp.model_selection(waves, 5, 2015, True)
            rospy.loginfo("End model selection...")
            tp.addition_technique = True
            for region in tp.regions:
                tp.prediction_accuracy("1", waves["1"], 5, 2015)

            rospy.loginfo("Start model selection...")
            tp.model_selection(waves, 5, 2015, False)
            rospy.loginfo("End model selection...")
            tp.addition_technique = False
            for region in tp.regions:
                tp.prediction_accuracy("1", waves["1"], 5, 2015)
            # tp.plot_region_idft("1")
