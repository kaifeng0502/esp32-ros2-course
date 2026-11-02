#!/usr/bin/env python3
"""Package the already-built ESP32-S3 image for flashing from Ubuntu. Never flashes."""
import hashlib
import argparse
import io
import json
from pathlib import Path
import tarfile


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    options=parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    build = root/'build'
    args = json.loads((build/'flasher_args.json').read_text())
    if args['extra_esptool_args']['chip'] != 'esp32s3':
        raise ValueError('Expected an ESP32-S3 build')
    expected = {0:'bootloader.bin',0x8000:'partition-table.bin',0x10000:'codex_oled.bin'}
    images = []
    for address,relative in args['flash_files'].items():
        offset = int(address,16)
        source = (build/relative).resolve()
        if offset not in expected or source.name != expected[offset] or not source.is_relative_to(build.resolve()):
            raise ValueError('Unexpected image/offset; review flasher_args.json before packaging')
        images.append((offset,source,expected[offset]))
    images.sort()
    if {offset for offset,_,_ in images} != set(expected):
        raise ValueError('Incomplete flash image set')
    checksums = ''.join(f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {name}\n' for _,path,name in images)
    settings = args['flash_settings']
    if settings != dict(flash_mode='dio',flash_size='16MB',flash_freq='80m'):
        raise ValueError('Flash settings changed; review before packaging')
    commands = ' '.join(f'0x{offset:x} {name}' for offset,_,name in images)
    flash = ('#!/usr/bin/env bash\nset -e\ncd "$(dirname "$0")"\n'
             'esp_port="${1:-/dev/ttyACM0}"\nsha256sum -c SHA256SUMS\n'
             'python3 -m esptool --chip esp32s3 --port "$esp_port" --baud 115200 '
             '--before default_reset --after hard_reset --no-stub write_flash '
             '--flash_mode dio --flash_size 16MB --flash_freq 80m '+commands+'\n')
    destination = options.output or root/'.run/ros2_action_download/wheel_firmware.tar.gz'
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tarfile.open(destination,'w:gz') as archive:
        for _,source,name in images:
            archive.add(source,arcname=name)
        for name,data in [('SHA256SUMS',checksums.encode()),('flash.sh',flash.encode()),
                          ('README.txt',b'ESP32 motor / ENC2 firmware. Keep motor 10V off while flashing.\n'
                            b'Ubuntu: bash flash.sh /dev/ttyACM0\n'
                            b'ROM mode (--no-stub) avoids missing Ubuntu packaged stub JSON files.\n'
                            b'Only bootloader, partition table and application are written.\n')]:
            info = tarfile.TarInfo(name)
            info.size,info.mode = len(data),0o644
            info.mtime = int((root/'main/codex_oled.c').stat().st_mtime)
            archive.addfile(info,io.BytesIO(data))
    print(destination)
    print('SHA256',hashlib.sha256(destination.read_bytes()).hexdigest())


if __name__ == '__main__':
    main()
