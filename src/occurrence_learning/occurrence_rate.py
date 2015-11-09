#!/usr/bin/env python

from scipy.stats import gamma


class OccurrenceRate(object):

    def __init__(self, interval=1):
        self.reset()
        self.interval = interval

    def reset(self):
        self.occurrence_scale = 1.0
        self.occurrence_shape = 0.00001
        self.gamma_map = 0.0
        self.gamma_mean = 0.0

    def update_lambda(self, data):
        self.occurrence_shape += sum(data)
        self.occurrence_scale += len(data) * self.interval
        self.gamma_map = self._gamma_mode(self.occurrence_shape, self.occurrence_scale)
        self.gamma_mean = gamma.mean(self.occurrence_shape, scale=1/float(self.occurrence_scale))

    def _gamma_mode(self, occurrence_shape, occurrence_scale):
        if occurrence_shape >= 1:
            return (occurrence_shape - 1) / float(occurrence_scale)
        else:
            return -1.0
