import glob
import os
import unittest

from openpilot.common.basedir import BASEDIR

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


# The fork's tree comes from release builds, which hold real asset files. Commits cherry-picked from master carry
# git-lfs pointers instead (master stores assets in LFS), and the UI cannot load a pointer as an image.
class TestAssetsNotLfs(unittest.TestCase):
  def test_pngs_are_real_images(self):
    paths = glob.glob(os.path.join(BASEDIR, "openpilot/selfdrive/assets/**/*.png"), recursive=True)
    assert len(paths) > 50
    bad = []
    for path in paths:
      with open(path, "rb") as f:
        if f.read(len(PNG_SIGNATURE)) != PNG_SIGNATURE:
          bad.append(os.path.relpath(path, BASEDIR))
    assert not bad, f"not PNG files (git-lfs pointers?): {bad}"


if __name__ == "__main__":
  unittest.main()
