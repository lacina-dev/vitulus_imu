"""T35 I1 (2026-11-16), from the 18 night drives dock -> TEST -> dock:
  the samples note in the status says what is on disk now
"""
import json
import os

import numpy as np

import test_mag_heading as T
from test_mag_heading import calibrated  # noqa: F401 - fixture
from vitulus_imu import mag_heading as mh

assert json and os and np            # used by some of the tests below


def test_samples_note_follows_the_file(tmp_path, calibrated):  # noqa: F811
    cal_file = str(tmp_path / 'mag_calibration.yaml')
    mh.save_calibration(cal_file, calibrated[0].cal)
    node, clock, _ = T.samples_node({'~calibration_file': cal_file})
    assert 'no samples file' in node.samples_note and node.samples_saved_t is None      # true at the start
    T.simulate(node.est, 9000.0, 30.0, T.to_and_fro, seed=50)
    clock['t'] = 9030.0
    assert node.save_samples() is True
    n, bins = node.est.bins.total(), node.est.bins.coverage(node.est.p['min_per_bin'])
    note = node.samples_note
    assert 'no samples file' not in note and node.samples_file in note
    assert '%d samples, %d bins, written' % (n, bins) in note and node.samples_saved_t == 9030.0
    clock['t'] = 9050.0
    assert node.save_samples() is False and node.samples_note == note                   # nothing new: note kept
    T.simulate(node.est, 9030.0, 10.0, T.to_and_fro, seed=51)
    clock['t'] = 9060.0
    assert node.save_samples() is True and node.samples_note != note
    assert '%d samples' % node.est.bins.total() in node.samples_note and node.samples_saved_t == 9060.0
    # after a restart: what was loaded
    node2, _, _ = T.samples_node({'~calibration_file': cal_file}, t0=9100.0)
    assert 'samples from' in node2.samples_note and '%d samples' % node.est.bins.total() in node2.samples_note


def test_samples_note_when_the_disk_cannot_be_written(tmp_path, calibrated):  # noqa: F811
    cal_file = str(tmp_path / 'mag_calibration.yaml')
    mh.save_calibration(cal_file, calibrated[0].cal)
    blocker = tmp_path / 'file'
    blocker.write_text('x')
    node, clock, _ = T.samples_node({'~calibration_file': cal_file, '~samples_file': str(blocker / 's.npz')})
    T.simulate(node.est, 9000.0, 10.0, T.to_and_fro, seed=51)
    assert node.save_samples() is False
    assert 'cannot be written' in node.samples_note and 'never written' in node.samples_note
    node5, _, _ = T.samples_node({'~calibration_file': cal_file, '~samples_file': ''})
    assert node5.samples_note == 'samples kept in memory only' and node5.save_samples() is False
