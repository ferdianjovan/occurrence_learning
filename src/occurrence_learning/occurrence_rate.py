#!/usr/bin/env python

from scipy.stats import gamma


class OccurrenceRate(object):

    def __init__(self, interval=1):
        self.reset()
        self.interval = interval

    def reset(self):
        self.occurrence_scale = 1.0
        self.occurrence_shape = 1.1
        self._gamma_map = self._gamma_mode(self.occurrence_shape, self.occurrence_scale)
        self._gamma_mean = gamma.mean(self.occurrence_shape, scale=1/float(self.occurrence_scale))

    def update_lambda(self, data):
        self.occurrence_shape += sum(data)
        self.occurrence_scale += len(data) * self.interval
        self._gamma_map = self._gamma_mode(self.occurrence_shape, self.occurrence_scale)
        self._gamma_mean = gamma.mean(self.occurrence_shape, scale=1/float(self.occurrence_scale))

    def _gamma_mode(self, occurrence_shape, occurrence_scale):
        if occurrence_shape >= 1:
            return (occurrence_shape - 1) / float(occurrence_scale)
        else:
            return -1.0

    def get_occurrence_rate(self):
        return self._gamma_map

    def set_occurrence_rate(self, value):
        self._gamma_map = value
