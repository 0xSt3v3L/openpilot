import os
import unittest
import numpy as np
from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.controls.lib.low_speed_lat import low_speed_lat_accel, LowSpeedLatShaper, DEADBAND, DEADBAND_RAMP, FULL_SPEED, MAX_SPEED

MODELD_V2 = os.path.join(BASEDIR, "openpilot/sunnypilot/modeld_v2/modeld.py")


def read_modeld_v2() -> str:
  # modeld_v2 cannot be imported without its model runtime, so check its source
  with open(MODELD_V2) as f:
    return f.read()


class TestLowSpeedLatAccel(unittest.TestCase):
  def test_stop_bias_is_zeroed(self):
    # routes 000000a3-a5: the model asks for +0.015 to +0.022 m/s^2 while stopping, which is 31-48 deg of wheel below 1 m/s
    for a in (0.015, 0.018, 0.022, -0.02):
      assert low_speed_lat_accel(a, 0.4) == 0.

  def test_stop_bias_with_a_blinker_is_zeroed(self):
    # routes 000000a8-a9: with either blinker on, +0.005 to +0.011 m/s^2 while stopping turned the wheel 22-38 deg right
    for a in (0.005, 0.009, 0.011):
      assert low_speed_lat_accel(a, 0.4) == 0.

  def test_real_low_speed_turns_are_unchanged(self):
    for a in (DEADBAND_RAMP, 0.15, 0.41, -1.27):
      assert low_speed_lat_accel(a, 1.0) == a

  def test_ramp_between_deadband_and_ramp_end(self):
    mid = (DEADBAND + DEADBAND_RAMP) / 2
    assert abs(low_speed_lat_accel(mid, 1.0) - mid * 0.5) < 1e-9
    assert low_speed_lat_accel(DEADBAND, 1.0) == 0.

  def test_fades_out_above_full_speed(self):
    halfway = (FULL_SPEED + MAX_SPEED) / 2
    out = low_speed_lat_accel(0.018, halfway)
    assert 0. < out < 0.018
    for v in (MAX_SPEED, 5., 30.):
      assert low_speed_lat_accel(0.018, v) == 0.018

  def test_never_grows_or_flips(self):
    for v in (0., 0.5, 1., 2., 2.5, 3., 10.):
      for i in range(-100, 101):
        a = i * 0.001
        out = low_speed_lat_accel(a, v)
        assert abs(out) <= abs(a) + 1e-12 and out * a >= 0.


class TestLowSpeedLatShaper(unittest.TestCase):
  DT = 0.05

  def _run(self, requests, v_ego):
    shaper = LowSpeedLatShaper(self.DT)
    return np.array([shaper.update(a, v_ego) for a in requests])

  def test_steady_small_requests_pass(self):
    # gentle low-speed bends need a steady small request: keep all of it
    out = self._run(np.full(100, 0.08), 4.0)
    assert abs(out[-1] - 0.08) < 1e-3

  def test_small_oscillation_is_attenuated(self):
    # 10-20 km/h swings: the request itself oscillates at about 1.3 s with a few hundredths of a m/s^2
    t = np.arange(400) * self.DT
    a = 0.06 * np.sin(2 * np.pi * t / 1.3)
    out = self._run(a, 4.0)
    assert np.max(np.abs(out[200:])) < 0.6 * 0.06

  def test_real_turns_are_unchanged(self):
    a = np.r_[np.zeros(20), np.full(60, 0.3), np.full(60, -0.5)]
    out = self._run(a, 4.0)
    assert np.allclose(out[20:], a[20:])

  def test_off_outside_the_speed_range(self):
    t = np.arange(200) * self.DT
    a = 0.06 * np.sin(2 * np.pi * t / 1.3)
    for v in (0.5, 7.0, 15.0):
      assert np.allclose(self._run(a, v), a)


class TestModeldV2Wiring(unittest.TestCase):
  def test_shapes_before_the_deadband(self):
    modeld = read_modeld_v2()
    assert "low_speed_lat_accel(self.lat_shaper.update(model_output['action'][0, 0], v_ego), v_ego) / (max(1.0, v_ego))**2" in modeld
    assert "self.lat_shaper = LowSpeedLatShaper(DT_MDL)" in modeld

  def test_deadband_ignores_the_blinker(self):
    assert "blinker" not in read_modeld_v2()


if __name__ == "__main__":
  unittest.main()
