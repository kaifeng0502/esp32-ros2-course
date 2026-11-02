"""Verify the portable public example still matches the board wire format."""
from pathlib import Path
import sys
import unittest
import zlib
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'host'))
from PIL import Image
import display
from oled_demo import packet


class PublicDisplayTests(unittest.TestCase):
    def test_page_layout_and_wire_checksum(self):
        image = Image.new('1', (128,64))
        image.putpixel((0,0),1); image.putpixel((127,63),1)
        frame = display.pack(image)
        self.assertEqual(len(frame),1024)
        self.assertEqual(frame[0],1)
        self.assertEqual(frame[-1],128)
        fields = packet(frame,123).decode().split()
        self.assertEqual(fields[:2],['OLED1','123'])
        self.assertEqual(int(fields[2],16),zlib.crc32(bytes.fromhex(fields[3])))
        self.assertEqual(bytes.fromhex(fields[3]),frame)

    def test_usage_semantics_and_offline_render(self):
        self.assertEqual(display.remaining_percent(27),73)
        self.assertEqual(display.remaining_percent(130),0)
        self.assertIsNone(display.remaining_percent(None))
        with self.assertRaises(ValueError): display.remaining_percent(float('nan'))
        self.assertEqual(len(display.pack(display.render(offline=True,animation_time=0))),1024)


if __name__ == '__main__': unittest.main()
