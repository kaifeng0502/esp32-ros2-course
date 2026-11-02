"""Help must never start provisioning, open hardware, or launch a long experiment."""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EntryTests(unittest.TestCase):
    def test_offline_lesson_help(self):
        for path in sorted((ROOT/'lessons').glob('*/code/*.py')):
            if path.parts[-3] == '12-topics':
                continue  # rclpy belongs to the real ROS job, not this Python environment.
            with self.subTest(entry=path.relative_to(ROOT)):
                result = subprocess.run([sys.executable,str(path),'--help'],cwd=ROOT,
                                        capture_output=True,text=True,timeout=15)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertIn('usage:',result.stdout.lower())


if __name__ == '__main__': unittest.main()
