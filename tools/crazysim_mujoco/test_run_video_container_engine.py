import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


class VideoContainerEngineTest(unittest.TestCase):
    def test_video_render_reuses_selected_container_engine(self):
        source = (ROOT / "tools/crazysim_mujoco/run.sh").read_text()
        render = source.split(
            'if [[ "$RENDER_VIDEO" == 1 && -f "$OUT/state.csv" ]]; then',
            1)[1]
        render = render.split('\necho "Results: $OUT"', 1)[0]
        self.assertIn('"${container_command[@]}"', render)
        self.assertNotIn("docker run", render)


if __name__ == "__main__":
    unittest.main()
