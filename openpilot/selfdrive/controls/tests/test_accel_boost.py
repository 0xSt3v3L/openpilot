import os
import unittest
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.basedir import BASEDIR
from openpilot.common.constants import CV

# op-chestnut-mads keeps upstream's gas override boost (removed upstream in #39306)
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost, ACCEL_BOOST_INITIAL, ACCEL_BOOST_MAX, ACCEL_BOOST_PER_OVERRIDE

AGGRESSIVE = log.LongitudinalPersonality.aggressive
STANDARD = log.LongitudinalPersonality.standard
FAST = 25 * CV.MPH_TO_MS
SLOW = 5 * CV.MPH_TO_MS


def make_sm(enabled=True, personality=AGGRESSIVE, gas=False, v_ego=FAST):
  return {
    'selfdriveState': SimpleNamespace(enabled=enabled, personality=personality),
    'carState': SimpleNamespace(gasPressed=gas, vEgo=v_ego),
  }


class TestAccelBoost(unittest.TestCase):
  def test_aggressive_starts_without_boost(self):
    # Fairy Tale road test: no initial boost, so red-light braking onset (0 to -0.5 m/s^2) is the model's own
    self.assertEqual(ACCEL_BOOST_INITIAL, 0.0)
    boost = AccelBoost()
    self.assertAlmostEqual(boost.update(make_sm(), 0.5, 2.0, 2.0), 0.5)
    self.assertAlmostEqual(boost.update(make_sm(), -0.3, 2.0, 2.0), -0.3)

  def test_standard_has_no_initial_boost(self):
    boost = AccelBoost()
    self.assertAlmostEqual(boost.update(make_sm(personality=STANDARD), 0.5, 2.0, 2.0), 0.5)

  def test_no_boost_at_low_speed(self):
    boost = AccelBoost()
    self.assertAlmostEqual(boost.update(make_sm(v_ego=SLOW), 0.5, 2.0, 2.0), 0.5)

  def test_no_boost_while_braking(self):
    boost = AccelBoost()
    self.assertAlmostEqual(boost.update(make_sm(), -1.5, 2.0, 2.0), -1.5)

  def test_gas_override_while_model_limited_builds_boost(self):
    boost = AccelBoost()
    boost.update(make_sm(), 0.5, 2.0, 2.0)  # model limited before the override: eligible
    for _ in range(200):
      boost.update(make_sm(gas=True), 0.5, 2.0, 2.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_INITIAL + ACCEL_BOOST_PER_OVERRIDE)

  def test_boost_is_capped(self):
    boost = AccelBoost()
    for _ in range(10):
      boost.update(make_sm(), 0.5, 2.0, 2.0)
      for _ in range(200):
        boost.update(make_sm(gas=True), 0.5, 2.0, 2.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_MAX)

  def test_gas_override_without_model_limit_adds_nothing(self):
    boost = AccelBoost()
    boost.update(make_sm(), 2.0, 2.0, 2.0)  # the model is not the limit
    for _ in range(200):
      boost.update(make_sm(gas=True), 2.0, 2.0, 2.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_INITIAL)

  def test_disengage_resets_to_initial(self):
    boost = AccelBoost()
    boost.update(make_sm(), 0.5, 2.0, 2.0)
    for _ in range(200):
      boost.update(make_sm(gas=True), 0.5, 2.0, 2.0)
    boost.update(make_sm(enabled=False), 0.5, 2.0, 2.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_INITIAL)


class TestPlannerUsesAccelBoost(unittest.TestCase):
  # the planner needs a full SubMaster to run, so check its source
  def test_planner_applies_and_publishes_boost(self):
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/controls/lib/longitudinal_planner.py")) as f:
      src = f.read()
    self.assertIn("self.accel_boost = AccelBoost()", src)
    self.assertIn("output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)", src)
    self.assertIn("longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)", src)

  def test_no_second_aggressive_boost(self):
    # upstream's #39430 adds a fixed aggressive boost; AccelBoost already starts at ACCEL_BOOST_INITIAL for aggressive
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/controls/lib/longitudinal_planner.py")) as f:
      src = f.read()
    self.assertNotIn("AGGRESSIVE_ACCEL_BOOST", src)

  def test_accel_boost_is_a_live_field(self):
    self.assertIn("accelBoost", log.LongitudinalPlan.schema.fields)


if __name__ == "__main__":
  unittest.main()
