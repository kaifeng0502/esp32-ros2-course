#!/usr/bin/env python3
"""Optionally bundle the ROS package for transfer; preserve mtimes for CMake install."""
import io
from pathlib import Path
import tarfile


def add_source(archive,source,name):
    data=source.read_bytes();info=tarfile.TarInfo(name)
    info.size=len(data);info.mode=0o644;info.mtime=int(source.stat().st_mtime)
    archive.addfile(info,io.BytesIO(data))


def package(root,destination):
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tarfile.open(destination,'w:gz') as archive:
        for source in sorted((root/'motor_action_interfaces').rglob('*')):
            if source.is_file() and '__pycache__' not in source.parts:
                add_source(archive,source,'src/motor_action_interfaces/'+str(source.relative_to(root/'motor_action_interfaces')))


if __name__=='__main__':
    root=Path(__file__).resolve().parent
    destination=root.parent/'dist/ros2-source.tar.gz';package(root,destination);print(destination)
