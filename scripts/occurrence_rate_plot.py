#!/usr/bin/env python

import sys
import string
import numpy as np
from scipy.stats import gamma
import matplotlib.pyplot as plt
from occurrence_learning.trajectory_occurrence_freq import TrajectoryOccurrenceFrequencies


class TOFPlot(object):

    def __init__(self, soma_map, soma_config, minute_interval=60):
        self.tof = TrajectoryOccurrenceFrequencies(soma_map, soma_config, minute_interval)
        self.set_x_ticks(minute_interval)
        self.minute_interval = minute_interval
        self.tof.load_tof()
        self.tof = self.tof.tof
        self.regions = self.tof.keys()
        self.colors = [
            (0., 0., 0.), (0., 0., 1.), (0., 1., 0.), (0., 1., 1.),
            (1., 0., 0.), (1., 0., 1.), (1., 1., 0.), (.75, .75, .75),
            (0., 0., 0.), (0., 0., .5), (0., .5, 0.), (0., .5, .5),
            (.5, 0., 0.), (.5, 0., .5), (.5, .5, 0.), (.25, .25, .25)

        ]

    def set_x_ticks(self, minute_interval):
        if len(self.tof.periodic_days) == 7:
            days = [
                "Monday", "Tuesday", "Wednesday", "Thursday", "Friday",
                "Saturday", "Sunday"
            ]
        else:
            days = [i+1 for i in range(31)]
        hours = [
            "%d:%d" % (i/(60/minute_interval), (minute_interval*i)%60)
            for i in range(24 * (60/minute_interval))
        ]
        for i in range(len(hours)):
            if string.find(hours[i], ":0") > -1:
                hours[i] = hours[i] + "0"
        days_hours = list()
        for j in days:
            for i in hours:
                days_hours.append("%s\n%s" % (j, i))
        self.xticks = days_hours

    def get_y_yerr_per_region(self, region):
        y = list()
        lower_percentile = list()
        upper_percentile = list()
        region_tof = self.tof[region]
        for hourly_tof in region_tof.itervalues():
            for mins_tof in hourly_tof:
                mins = sorted(mins_tof)
                for i in mins:
                    y.append(mins_tof[i].gamma_map)
                    lower_percentile.append(
                        abs(mins_tof[i].gamma_map - gamma.ppf(0.025, mins_tof[i].alpha, scale=1/float(mins_tof[i].beta)))
                        # gamma.ppf(0.025, mins_tof[i].alpha, scale=1/float(mins_tof[i].beta))
                    )
                    upper_percentile.append(
                        abs(mins_tof[i].gamma_map - gamma.ppf(0.975, mins_tof[i].alpha, scale=1/float(mins_tof[i].beta)))
                        # gamma.ppf(0.975, mins_tof[i].alpha, scale=1/float(mins_tof[i].beta))
                    )
        return y, lower_percentile, upper_percentile

    def show_tof_per_region(self, region):
        x = np.arange(len(self.xticks))
        xticks = list()
        for i in self.xticks:
            if string.find(i, ":00") > -1:
                hour = string.split(i, "\n")[1]
                if hour == "0:00":
                    xticks.append(i)
                elif hour in ["3:00", "6:00", "9:00", "12:00", "15:00", "18:00", "21:00"]:
                    xticks.append(hour)
                else:
                    xticks.append(" ")
            else:
                xticks.append(" ")

        y, low_err, up_err = self.get_y_yerr_per_region(region)
        plt.errorbar(x, y, yerr=[low_err, up_err], color='b', ecolor='r', fmt="-o", label="Region " + region)
        # plt.bar(x, up_err, 0.35, bottom=low_err, color=color)

        plt.title("Occurrence Rate for Region %s" % region)
        plt.xticks(x, xticks, rotation="vertical")
        plt.xlabel("One Week Period with %d minutes interval" % (self.minute_interval))
        plt.ylabel("Occurrence rate value")
        plt.ylim(ymin=-5)

        plt.legend()
        plt.show()

    def show_tof(self):
        x = np.arange(len(self.xticks))
        xticks = list()
        for i in self.xticks:
            if string.find(i, ":00") > -1:
                hour = string.split(i, "\n")[1]
                if hour == "0:00":
                    xticks.append(i)
                elif hour in ["3:00", "6:00", "9:00", "12:00", "15:00", "18:00", "21:00"]:
                    xticks.append(hour)
                else:
                    xticks.append(" ")
            else:
                xticks.append(" ")

        for region in self.regions:
            try:
                color = self.colors[int(region) % len(self.colors)]
                ecolor = self.colors[(int(region)**2 + 4) % len(self.colors)]
            except:
                color = (0., 0., 1.)
                ecolor = (1., 0., 0.)

            y, low_err, up_err = self.get_y_yerr_per_region(region)
            plt.errorbar(x, y, yerr=[low_err, up_err], color=color, ecolor=ecolor, fmt="-o", label="Region " + region)

        plt.title("Occurrence Rate for All Regions")
        plt.xticks(x, xticks, rotation="vertical")
        plt.xlabel("One Week Period with %d minutes interval" % (self.minute_interval))
        plt.ylabel("Occurrence rate value")
        plt.ylim(ymin=-5)

        plt.legend()
        plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("usage: visualization map map_config minute_interval [all|region]")
        sys.exit(2)

    tofplot = TOFPlot(sys.argv[1], sys.argv[2], int(sys.argv[3]))
    if sys.argv[4] == "all":
        tofplot.show_tof()
    else:
        print "Available regions: %s" % str(tofplot.regions)
        region = raw_input("Chosen region: ")
        tofplot.show_tof_per_region(region)
