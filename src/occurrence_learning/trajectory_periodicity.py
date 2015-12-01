#!/usr/bin/env python

import sys
import rospy
import string
import numpy as np
import matplotlib.pyplot as plt

from scipy.fftpack import fft
from scipy.stats import poisson
from sklearn.metrics import mean_squared_error as mse
from occurrence_learning.occurrence_learning_util import time_ticks
from occurrence_learning.occurrence_rate import OccurrenceRate as Lambda
from occurrence_learning.trajectory_region_est import TrajectoryRegionEstimate
from occurrence_learning.occurrence_learning_util import trajectories_full_dates_periodic
from occurrence_learning.trajectory_occurrence_freq import TrajectoryOccurrenceFrequencies


class TrajectoryPeriodicity(object):

    def __init__(self, soma_map, soma_config, minute_interval=1, window_interval=10):
        """
            Initialise analysis for trajectory periodicity.
            Initialise trajectory occurrence frequencies from database.
        """
        self.tof = TrajectoryOccurrenceFrequencies(
            soma_map, soma_config, minute_interval, window_interval
        )
        self.periodic_type = self.tof.periodic_type
        self.length_of_periodicity = len(self.tof.periodic_days)
        self.minute_interval = minute_interval
        self.window_interval = window_interval
        self.tof.load_tof()
        self.tof = self.tof.tof
        self.regions = self.tof.keys()

    def get_tof_values(self, region):
        """
            Obtain trajectory occurrence frequency values for a specific region.
        """
        length = (self.window_interval/self.minute_interval) - 1
        y = list()
        region_tof = self.tof[region]
        for day, hourly_tof in region_tof.iteritems():
            mins = sorted(hourly_tof)
            daily_y = list()
            for i in mins:
                daily_y.append(hourly_tof[i].get_occurrence_rate())
            y.extend(daily_y[-length:] + daily_y[:-length])
        return y

    def reconstruct_tof_from_spectrum(self, region, addition_method=True):
        """
            Reconstruct trajectory occurrence frequency values after they are transformed
            into spectral dimensions.
        """
        original = self.get_tof_values(region)
        if addition_method:
            spectrums, _ = self.get_significant_frequencies(original, 60)
            spectrums = spectrums[0:30]
        else:
            spectrums = self.get_highest_n_freq(fft(original), 30)
        reconstruction = 0
        for spectrum in spectrums:
            reconstruction += self.rectify_wave(
                spectrum[2], spectrum[0], spectrum[1], len(original)
            )
        reconstruction = map(
            lambda x: Lambda().get_occurrence_rate() if x <= 0.0 else x, reconstruction
        )
        return reconstruction, original

    def rectify_wave(self, freq, amp, phs, num_of_points, up_thres=None, low_thres=None):
        xf = np.linspace(0.0, num_of_points, num_of_points)  # frequency varations
        wave = amp * np.cos((freq * 2.0 * np.pi * xf) + phs)
        for ind, val in enumerate(wave):
            if low_thres is not None and val < low_thres:
                wave[ind] = low_thres
            if up_thres is not None and val > up_thres:
                wave[ind] = up_thres
        return wave

    def get_significant_frequencies(self, data, total_freq=15):
        N = len(data)
        xf = np.linspace(0.0, N, N)
        # initialise significant frequencies by taking frequency 0
        spectrum_data = fft(data)
        [amp, phs, freq] = self.get_highest_n_freq(spectrum_data, 1)[0]
        frequencies = [[amp, phs, freq]]
        freq_occur_counter = dict()

        while len(frequencies) < total_freq:
            spectrum_data = fft(data)
            # recreate wave of the highest frequency
            [amp, phs, freq] = self.get_highest_n_freq(spectrum_data, 2)[1]
            wave = amp * np.cos((freq * 2.0 * np.pi * xf) + phs)
            # substracting data with the wave
            data -= wave
            if freq not in zip(*frequencies)[2]:
                frequencies.append([amp, phs, freq])
                freq_occur_counter.update({freq: 1})
            else:
                for ind, val in enumerate(frequencies):
                    if frequencies[ind][2] == freq:
                        frequencies[ind][0] += amp
                        frequencies[ind][1] = ((
                            freq_occur_counter[freq] * frequencies[ind][1]
                        ) + phs) / (freq_occur_counter[freq] + 1)
                        freq_occur_counter[freq] += 1
        return frequencies, data

    def get_highest_n_freq(self, freqs, n=15):
        N = len(freqs)
        freqs = freqs[0:N/2]
        indices = [i for i in range(len(freqs))]
        angles = np.angle(freqs)
        amplitudes = np.abs(freqs) / float(N)
        sorted_result = sorted(zip(amplitudes, angles, indices), reverse=True)
        n_freqs = sorted_result[:n]
        return n_freqs

    def calculate_mse(self, region, data, month, year, addition_method=True):
        reconstruction_tof, original_tof = self.reconstruct_tof_from_spectrum(
            region, addition_method
        )
        test_data = trajectories_full_dates_periodic(
            data, month, year, self.length_of_periodicity,
            self.window_interval, self.minute_interval
        )
        # Dropping -1 in test_data together with corresponding tof
        test_data, reconstruction_tof, original_tof, _ = self._remove_unobserved_data(
            test_data, reconstruction_tof, original_tof
        )
        mse_recon = -1
        mse_origin = -1
        if len(reconstruction_tof) > 0 and len(original_tof) > 0:
            mse_recon = mse(test_data, reconstruction_tof)
            mse_origin = mse(test_data, original_tof)
        rospy.loginfo("Calculated MSE for original tof Region %s: %.2f" % (region, mse_origin))
        rospy.loginfo("Calculated MSE for reconstruction tof Region %s: %.2f" % (region, mse_recon))
        return mse_recon, mse_origin

    def _remove_unobserved_data(self, data1, data2, data3):
        # Dropping -1 in data1, the corresponding indices in data2 and data3 are
        # also dropped
        deleted_indices = list()
        for ind, trajs in enumerate(data1):
            if trajs == -1:
                deleted_indices.append(ind)
        data1 = [j for i, j in enumerate(data1) if i not in deleted_indices]
        data2 = [j for i, j in enumerate(data2) if i not in deleted_indices]
        data3 = [j for i, j in enumerate(data3) if i not in deleted_indices]
        return data1, data2, data3, deleted_indices

    def prediction_accuracy(self, region, data, month, year, percentile=0.1, plot=False, addition_method=True):
        reconstruction_tof, original_tof = self.reconstruct_tof_from_spectrum(
            region, addition_method
        )
        test_data = trajectories_full_dates_periodic(
            data, month, year, self.length_of_periodicity,
            self.window_interval, self.minute_interval
        )
        original_predict = self._get_prediction(original_tof, test_data, percentile)
        reconstr_predict = self._get_prediction(reconstruction_tof, test_data, percentile)
        _, clean_ori_pred, clean_recon_pred, indices = self._remove_unobserved_data(
            test_data, reconstr_predict, original_predict
        )
        if len(clean_ori_pred) and len(clean_recon_pred):
            mse_predict = mse(clean_ori_pred, clean_recon_pred)
        else:
            mse_predict = -1
        rospy.loginfo(
            "Calculated MSE for prediction between original and reconstruction for Region %s: %.2f" % (region, mse_predict)
        )
        if plot:
            for index in indices:
                original_predict[index] = -1
                reconstr_predict[index] = -1
            x = np.linspace(0, len(test_data), len(test_data))
            xticks = time_ticks(self.minute_interval, self.window_interval, self.periodic_type)
            plt.plot(
                x, original_predict, "-o", label="Prediction Original TOF"
            )
            plt.plot(
                x, reconstr_predict, "-^", label="Prediction Reconstruction TOF"
            )
            plt.title("Prediction for Region %s" % region)
            plt.xticks(x, xticks, rotation="vertical")
            plt.xlabel("One Week Period with %d minutes interval and %d window time" % (self.minute_interval, self.window_interval))
            plt.ylabel("Prediction (1=Anomalous, 0=Normal, -1=Unobserved)")
            plt.ylim(ymin=-2, ymax=2)

            plt.legend()
            plt.show()

    def _get_prediction(self, rates, data, percentile):
        flag = list()
        for ind, rate in enumerate(rates):
            if rate > 0.0:
                lower = poisson.ppf(percentile, rate)
                upper = poisson.ppf(1-percentile, rate)
                if data[ind] < lower or data[ind] > upper:
                    flag.append(1)
                else:
                    flag.append(0)
            else:
                rospy.logwarn("Occurrence rate is %.2f" % rate)
                flag.append(-1)
        return flag

    def plot_region_idft(self, region):
        data = self.get_tof_values(region)
        spectrums, _ = self.get_significant_frequencies(data, 60)
        spectrums = spectrums[:30]
        spectrums2 = self.get_highest_n_freq(fft(data))

        y = 0
        x = np.linspace(0, len(data), len(data))
        xticks = time_ticks(self.minute_interval, self.window_interval, self.periodic_type)
        for spectrum in spectrums:
            y += self.rectify_wave(spectrum[2], spectrum[0], spectrum[1], len(data))

        y2 = 0
        for spectrum in spectrums2:
            y2 += self.rectify_wave(spectrum[2], spectrum[0], spectrum[1], len(data))

        plt.plot(
            x, y, "-o", color="red", label="Region " + region + " Amplitude Addition Reconstruction"
        )
        plt.plot(
            x, y2, "-x", color="blue", label="Region " + region + " Best Amplitude Reconstruction"
        )
        plt.plot(x, data, "-", color="green", label="Region " + region + " Original")

        plt.title("Reconstruction Occurrence Rate for Region %s" % region)
        plt.xticks(x, xticks, rotation="vertical")
        plt.xlabel("One Week Period with %d minutes interval and %d window time" % (self.minute_interval, self.window_interval))
        plt.ylabel("Occurrence rate value")
        plt.ylim(ymin=-5)

        plt.legend()
        plt.show()


if __name__ == "__main__":
    rospy.init_node("trajectory_periodicity")
    if len(sys.argv) < 5:
        rospy.logerr("usage: periodicity map map_config minute_interval window_interval")
        sys.exit(2)

    interval = int(sys.argv[3])
    window = int(sys.argv[4])
    inp = raw_input("start_date end_date month year: ")

    [start_date, end_date, month, year] = string.split(inp)
    start_date = int(start_date)
    end_date = int(end_date)
    month = int(month)
    year = int(year)
    tre = TrajectoryRegionEstimate(sys.argv[1], sys.argv[2], interval)
    if start_date > 1:
        # if start_date is not the first date of the month, then add the
        # previous date
        trajectory_estimate = tre.estimate_trajectories_daily(
            range(start_date-1, end_date+1), month, year
        )
    else:
        trajectory_estimate = tre.estimate_trajectories_daily(
            range(start_date, end_date+1), month, year
        )

    tp = TrajectoryPeriodicity(sys.argv[1], sys.argv[2], interval, window)
    inp = raw_input("MSE(0) or Prediction MSE(1): ")
    if int(inp) == 0:
        rospy.loginfo("Addition Best Amplitude Frequency Method")
        for region in tp.regions:
            tp.calculate_mse(region, trajectory_estimate[region], month, year)
        rospy.loginfo("Best Amplitude Frequency Method")
        for region in tp.regions:
            tp.calculate_mse(region, trajectory_estimate[region], month, year, False)
    else:
        rospy.loginfo("Addition Best Amplitude Frequency Method")
        for region in tp.regions:
            tp.prediction_accuracy(region, trajectory_estimate[region], month, year)
        rospy.loginfo("Best Amplitude Frequency Method")
        for region in tp.regions:
            tp.prediction_accuracy(region, trajectory_estimate[region], month, year, addition_method=False)
            # tp.plot_region_idft(region)
