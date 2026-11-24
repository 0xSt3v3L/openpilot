import os
import unittest
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.basedir import BASEDIR
from openpilot.common.constants import CV

# op-chestnut-mads keeps upstream's gas override boost (removed upstream in #39306)
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost, ACCEL_BOOST_INITIAL, ACCEL_BOOST_MAX, ACCEL_BOOST_PER_OVERRIDE
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW

AGGRESSIVE = log.LongitudinalPersonality.aggressive
STANDARD = log.LongitudinalPersonality.standard
RELAXED = log.LongitudinalPersonality.relaxed
INITIAL = ACCEL_BOOST_INITIAL[AGGRESSIVE]
FAST = 25 * CV.MPH_TO_MS
SLOW = 5 * CV.MPH_TO_MS


def make_sm(enabled=True, personality=AGGRESSIVE, gas=False, v_ego=FAST):
  return {
    'selfdriveState': log.SelfdriveState.new_message(enabled=enabled, personality=personality),
    'carState': SimpleNamespace(gasPressed=gas, vEgo=v_ego),
  }


class TestAccelBoost(unittest.TestCase):
  def test_initial_boost_follows_upstream_personality_table(self):
    # upstream #39440: relaxed 0, standard 0.05, aggressive 0.15
    self.assertEqual(ACCEL_BOOST_INITIAL, {RELAXED: 0.0, STANDARD: 0.05, AGGRESSIVE: 0.15})
    for personality, initial in ACCEL_BOOST_INITIAL.items():
      boost = AccelBoost()
      self.assertAlmostEqual(boost.update(make_sm(personality=personality), 0.5, 2.0, 2.0), 0.5 + initial)
      self.assertAlmostEqual(boost.update(make_sm(personality=personality), -0.3, 2.0, 2.0), -0.3 + initial)

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
    self.assertAlmostEqual(boost.total_boost, INITIAL + ACCEL_BOOST_PER_OVERRIDE)

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
    self.assertAlmostEqual(boost.total_boost, INITIAL)

  def test_disengage_resets_to_initial(self):
    boost = AccelBoost()
    boost.update(make_sm(), 0.5, 2.0, 2.0)
    for _ in range(200):
      boost.update(make_sm(gas=True), 0.5, 2.0, 2.0)
    boost.update(make_sm(enabled=False), 0.5, 2.0, 2.0)
    self.assertAlmostEqual(boost.total_boost, INITIAL)


class TestPlannerUsesAccelBoost(unittest.TestCase):
  # the planner needs a full SubMaster to run, so check its source
  def test_planner_applies_and_publishes_boost(self):
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/controls/lib/longitudinal_planner.py")) as f:
      src = f.read()
    self.assertIn("self.accel_boost = AccelBoost()", src)
    self.assertIn("output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)", src)
    self.assertIn("longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)", src)

  def test_no_second_personality_boost(self):
    # upstream adds its own personality boost (#39430 fixed aggressive, #39440 a table); AccelBoost already starts there
    with open(os.path.join(BASEDIR, "openpilot/selfdrive/controls/lib/longitudinal_planner.py")) as f:
      src = f.read()
    self.assertNotIn("AGGRESSIVE_ACCEL_BOOST", src)
    self.assertNotIn("ACCEL_BOOST[", src)

  def test_accel_boost_is_a_live_field(self):
    self.assertIn("accelBoost", log.LongitudinalPlan.schema.fields)


class TestFollowTime(unittest.TestCase):
  def test_relaxed_follows_like_standard(self):
    # upstream #39440
    self.assertEqual(get_T_FOLLOW(RELAXED), 1.45)
    self.assertEqual(get_T_FOLLOW(STANDARD), 1.45)
    self.assertEqual(get_T_FOLLOW(AGGRESSIVE), 1.25)


if __name__ == "__main__":
  unittest.main()
