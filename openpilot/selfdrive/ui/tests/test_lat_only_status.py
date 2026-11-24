import ast
import glob
import os
import unittest

from openpilot.common.basedir import BASEDIR

ONROAD = os.path.join(BASEDIR, "openpilot/selfdrive/ui/mici/onroad")


def _is_lat_only(node: ast.AST | None) -> bool:
  return isinstance(node, ast.Attribute) and node.attr == "LAT_ONLY"


def _dict_keys(tree: ast.Module, arg: ast.AST) -> list[ast.AST | None] | None:
  if isinstance(arg, ast.Dict):
    return arg.keys
  if isinstance(arg, ast.Name):
    for node in tree.body:
      if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == arg.id for t in node.targets):
        if isinstance(node.value, ast.Dict):
          return node.value.keys
  return None


def _calls(tree: ast.Module, method: str):
  for node in ast.walk(tree):
    if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == method and
        isinstance(node.func.value, ast.Name) and node.func.value.id == "status_fade"):
      yield node


# mici UI modules cannot be imported without a display, so these read the source.
# StatusFade.color() indexes its dict by every UIStatus, so a call without LAT_ONLY raises a KeyError in MADS.
class TestLatOnlyStatus(unittest.TestCase):
  def _parse(self, path):
    with open(path) as f:
      return ast.parse(f.read())

  def test_every_status_color_has_lat_only(self):
    calls = 0
    for path in sorted(glob.glob(os.path.join(ONROAD, "*.py"))):
      tree = self._parse(path)
      for call in _calls(tree, "color"):
        calls += 1
        keys = _dict_keys(tree, call.args[0])
        assert keys is not None, f"{os.path.basename(path)}:{call.lineno} status_fade.color() argument is not a dict literal or module dict"
        assert any(_is_lat_only(k) for k in keys), f"{os.path.basename(path)}:{call.lineno} status_fade.color() has no LAT_ONLY color"
    assert calls >= 3

  def test_steering_weight_covers_lat_only(self):
    tree = self._parse(os.path.join(ONROAD, "status.py"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "steering_weight")
    names = {n.attr for n in ast.walk(fn) if isinstance(n, ast.Attribute)}
    assert {"ENGAGED", "LAT_ONLY"} <= names

  def test_steering_widgets_use_steering_weight(self):
    # the torque bar and the wheel show openpilot steering, also while MADS steers alone
    for name in ("torque_bar.py", "hud_renderer.py"):
      tree = self._parse(os.path.join(ONROAD, name))
      assert any(_calls(tree, "steering_weight")), f"{name} does not use status_fade.steering_weight()"
      engaged_weights = [c for c in _calls(tree, "weight") if c.args and isinstance(c.args[0], ast.Attribute) and c.args[0].attr == "ENGAGED"]
      assert not engaged_weights, f"{name} weighs ENGAGED alone"

  def test_speed_dial_is_off_while_lat_only(self):
    # MADS lateral-only does not control speed: the speed dial must not turn green or show the set speed / AUTO
    tree = self._parse(os.path.join(ONROAD, "long_indicator.py"))
    weighed = {c.args[0].attr for c in _calls(tree, "weight") if c.args and isinstance(c.args[0], ast.Attribute)}
    assert {"DISENGAGED", "LAT_ONLY"} <= weighed, f"long_indicator.py weighs {weighed}, not DISENGAGED and LAT_ONLY"


if __name__ == "__main__":
  unittest.main()
