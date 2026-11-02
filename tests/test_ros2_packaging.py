"""Verify a second lesson bundle actually replaces CMake-installed scripts."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1]/'ros2/package_action.py'
spec = importlib.util.spec_from_file_location('package_action_test',SOURCE)
packager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(packager)


class PackagingTests(unittest.TestCase):
    def test_source_timestamp_survives_archive(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            src = root/'client.py'
            src.write_text('new --configured')
            os.utime(src,(1700000000,1700000000))
            bundle = root/'lesson.tar.gz'
            with tarfile.open(bundle,'w:gz') as archive:
                packager.add_source(archive,src,'src/client.py')
            with tarfile.open(bundle) as archive:
                self.assertEqual(archive.getmember('src/client.py').mtime,1700000000)

    @unittest.skipUnless(shutil.which('cmake'),'CMake required for install regression')
    def test_updated_bundle_replaces_installed_script(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            src = root/'client.py'
            extracted = root/'unpacked'
            installed = root/'installed'
            script = root/'install.cmake'
            script.write_text(f'file(INSTALL DESTINATION "{installed}" TYPE PROGRAM FILES "{extracted}/client.py")\n')
            for stamp, contents in [(1700000000,'old'),(1700000002,'new --configured')]:
                src.write_text(contents)
                os.utime(src,(stamp,stamp))
                bundle = root/'lesson.tar.gz'
                with tarfile.open(bundle,'w:gz') as archive:
                    packager.add_source(archive,src,'client.py')
                with tarfile.open(bundle) as archive:
                    archive.extractall(extracted,filter='data')
                subprocess.run(['cmake','-P',str(script)],check=True,capture_output=True)
                self.assertEqual((installed/'client.py').read_text(),contents)


if __name__ == '__main__':
    unittest.main()
