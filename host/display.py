"""Portable OLED rendering with an original procedural pixel cat; no bundled font/sprite."""
from datetime import datetime
from pathlib import Path
import math
import time
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]


def remaining_percent(used):
    if used is None:return None
    if not math.isfinite(used):raise ValueError('Usage percentage must be finite')
    return max(0,min(100,100-used))


def pack(image):
    if image.size!=(128,64):raise ValueError('Expected a 128x64 image')
    return bytes(sum((1<<bit) if image.getpixel((x,page*8+bit)) else 0
                     for bit in range(8)) for page in range(8) for x in range(128))


def render(snapshot=None,*,now=None,offline=False,animation_time=None,text=None):
    now=now or datetime.now().astimezone()
    animation_time=time.monotonic() if animation_time is None else animation_time
    image=Image.new('1',(128,64),0);draw=ImageDraw.Draw(image)
    font=ImageFont.load_default(size=10)
    # Geometry drawn here is the public example's own mascot, not an app sprite.
    bob=int(round(math.sin(animation_time*3)))
    draw.polygon([(3,10+bob),(3,2+bob),(10,7+bob),(20,7+bob),(27,2+bob),(27,10+bob)],outline=1)
    draw.rounded_rectangle((3,7+bob,27,28+bob),radius=5,outline=1)
    blink=int(animation_time*4)%12==0
    if blink:
        draw.line((8,16+bob,11,16+bob),fill=1);draw.line((19,16+bob,22,16+bob),fill=1)
    else:
        draw.rectangle((9,14+bob,10,17+bob),fill=1);draw.rectangle((20,14+bob,21,17+bob),fill=1)
    draw.point((15,20+bob),fill=1)
    draw.text((35,1),'ESP32 READY' if offline else now.strftime('%H:%M:%S'),font=font,fill=1)
    draw.text((35,15),'ESP32 + ROS2',font=font,fill=1)
    if offline:
        draw.text((4,39),'READY / WAITING',font=font,fill=1)
    elif text:
        draw.text((3,38),text,font=font,fill=1)
    else:
        snapshot=snapshot or {};percent=remaining_percent(snapshot.get('used_percent'))
        caption='REMAINING --' if percent is None else f'REMAINING {percent:.0f}%'
        draw.text((3,38),caption,font=font,fill=1)
        draw.rectangle((2,53,125,62),outline=1)
        if percent is not None and percent>0:
            draw.rectangle((4,55,3+round(120*percent/100),60),fill=1)
    return image
