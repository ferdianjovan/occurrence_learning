#!/usr/bin/env python

import sys
import string
import datetime
import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import gamma
from occurrence_learning.occurrence_learning_util import time_ticks
from occurrence_learning.trajectory_occurrence_freq import TrajectoryOccurrenceFrequencies


class TOFPlot(object):
    """
        This class is for plotting the trajectory occurrence frequency every n minute with
        m window minute interval (as its bin) that is continuous starting from Monday 00:00
        to Sunday 23:59
    """

    def __init__(self, soma_map, soma_config, minute_interval=1, window_interval=10):
        """
            Initialize plotting, it needs the name of the soma map, specific soma configuration,
            the minute interval between two different occurrence rate, and the window interval that
            acts as bins.
        """
        self.tof = TrajectoryOccurrenceFrequencies(soma_map, soma_config, minute_interval, window_interval)
        self.xticks = time_ticks(minute_interval, window_interval, self.tof.periodic_type)
        self.x = np.arange(len(self.xticks))
        self.minute_interval = minute_interval
        self.window_interval = window_interval
        self.tof.load_tof()
        self.tof = self.tof.tof
        self.regions = self.tof.keys()
        self.colors = [
            (0., 0., 0.), (0., 0., 1.), (0., 1., 0.), (0., 1., 1.),
            (1., 0., 0.), (1., 0., 1.), (1., 1., 0.), (.75, .75, .75),
            (0., 0., 0.), (0., 0., .5), (0., .5, 0.), (0., .5, .5),
            (.5, 0., 0.), (.5, 0., .5), (.5, .5, 0.), (.25, .25, .25)

        ]

    def get_y_yerr_per_region(self, region):
        """
            Obtain the occurrence rate value, the mode of gamma distribution, for each region including
            the lower percentile 0.025, and the upper percentile of the value 0.975 showing
            the wide of the gamma distribution.
        """
        y = list()
        lower_percentile = list()
        upper_percentile = list()
        region_tof = self.tof[region]
        for day, hourly_tof in region_tof.iteritems():
            mins = sorted(hourly_tof)
            for i in mins:
                y.append(hourly_tof[i].get_occurrence_rate())
                lower_percentile.append(
                    abs(hourly_tof[i].get_occurrence_rate() - gamma.ppf(0.025, hourly_tof[i].occurrence_shape, scale=1/float(hourly_tof[i].occurrence_scale)))
                    # gamma.ppf(0.025, mins_tof[i].occurrence_shape, scale=1/float(mins_tof[i].occurrence_scale))
                )
                upper_percentile.append(
                    abs(hourly_tof[i].get_occurrence_rate() - gamma.ppf(0.975, hourly_tof[i].occurrence_shape, scale=1/float(hourly_tof[i].occurrence_scale)))
                    # gamma.ppf(0.975, mins_tof[i].occurrence_shape, scale=1/float(mins_tof[i].occurrence_scale))
                )
        return y, lower_percentile, upper_percentile

    def show_tof_per_region(self, region):
        """
            Show the occurrence rate over a week/month for each region available from soma map
        """
        y, low_err, up_err = self.get_y_yerr_per_region(region)
        plt.errorbar(
            self.x, y, yerr=[low_err, up_err], color='b', ecolor='r',
            fmt="-o", label="Region " + region
        )

        plt.title("Occurrence Rate for Region %s" % region)
        plt.xticks(self.x, self.xticks, rotation="vertical")
        plt.xlabel(
            "One Week Period with %d minutes interval and %d window time" % (
                self.minute_interval, self.window_interval
            )
        )
        plt.ylabel("Occurrence rate value")
        plt.ylim(ymin=-1)

        plt.legend()
        plt.show()

    def show_tof(self):
        """
            Show occurrence rate for all regions over a week/month
        """
        for region in self.regions:
            try:
                color = self.colors[int(region) % len(self.colors)]
                ecolor = self.colors[(int(region)**2 + 4) % len(self.colors)]
            except:
                color = (0., 0., 1.)
                ecolor = (1., 0., 0.)

            y, low_err, up_err = self.get_y_yerr_per_region(region)
            plt.errorbar(
                self.x, y, yerr=[low_err, up_err], color=color, ecolor=ecolor,
                fmt="-o", label="Region " + region
            )

        plt.title("Occurrence Rate for All Regions")
        plt.xticks(self.x, self.xticks, rotation="vertical")
        plt.xlabel(
            "One Week Period with %d minutes interval and %d window time" % (
                self.minute_interval, self.window_interval
            )
        )
        plt.ylabel("Occurrence rate value")
        plt.ylim(ymin=-1)

        plt.legend()
        plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("usage: visualization map map_config minute_interval window_interval")
        sys.exit(2)

    tofplot = TOFPlot(sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
    print "Available regions: %s" % str(tofplot.regions)
    region = raw_input("Chosen region (or type 'all'): ")

    if region in tofplot.regions:
        tofplot.show_tof_per_region(region)
    else:
        tofplot.show_tof()
