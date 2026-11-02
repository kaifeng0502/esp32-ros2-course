"""One-time local Wi-Fi setup. Password stays in memory and is sent only by USB."""
from http.server import BaseHTTPRequestHandler, HTTPServer
import argparse
import html
import ipaddress
import json
import os
from pathlib import Path
import secrets
import subprocess
import sys
import threading
import time
import urllib.parse
import webbrowser
import serial
from encoder_monitor import choose_port
from display import ROOT
from wifi_transport import CONFIG, WiFiTransport

TOKEN=secrets.token_urlsafe(24)
RESULT=ROOT/'.run/setup-result.json'
INFO=ROOT/'.run/setup-info.json'

def provision(ssid,password,device_port=None):
    sb=ssid.encode('utf-8');pb=password.encode('utf-8')
    if not 1<=len(sb)<=32 or b'\x00' in sb:raise ValueError('Wi-Fi 名称需要为 1–32 字节。')
    if (pb and not 8<=len(pb)<=63) or b'\x00' in pb:raise ValueError('密码需要为 8–63 字节；无密码网络可留空。')
    key=secrets.token_bytes(32)
    port=None
    def connect():
        p=serial.Serial(port=None,baudrate=115200,timeout=.2,write_timeout=2,exclusive=True)
        p.dtr=False;p.rts=False;p.port=choose_port(device_port);p.open();p.reset_input_buffer()
        return p
    try:
        port=connect()
        command=b'WIFI1 '+sb.hex().encode()+b' '+(pb.hex().encode() if pb else b'-')+b' '+key.hex().encode()+b'\n'
        port.write(command)
        command=None;password=None;pb=None
        deadline=time.monotonic()+5
        saved=False
        while time.monotonic()<deadline:
            line=port.readline().strip()
            if line==b'NET SAVED':saved=True;break
            if line.startswith(b'NET ERROR'):raise RuntimeError('开发板未能保存配置，请重试。')
        if not saved:raise RuntimeError('开发板没有确认配置，请确认 USB 连接和固件。')
        port.close();port=None
        # Persist only the display pairing key, never the Wi-Fi password.
        config={'key':key.hex(),'ip':None}
        fd=os.open(CONFIG,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
        with os.fdopen(fd,'w') as file:json.dump(config,file)
        os.chmod(CONFIG,0o600)
        time.sleep(2)
        deadline=time.monotonic()+60
        ip=None
        while time.monotonic()<deadline:
            try:
                if port is None:port=connect()
                port.write(b'NET?\n')
                end=time.monotonic()+1
                while time.monotonic()<end:
                    line=port.readline().strip()
                    if line.startswith(b'NET IP '):
                        ip=str(ipaddress.IPv4Address(line[7:].decode()));break
                if ip:break
            except (OSError,serial.SerialException,RuntimeError):
                if port:port.close();port=None
            time.sleep(1)
        if not ip:raise RuntimeError('尚未连上网络。请确认名称、密码和 2.4 GHz 网络，再提交一次。')
        port.close();port=None
        config['ip']=ip
        temp=CONFIG.with_suffix('.tmp')
        fd=os.open(temp,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
        with os.fdopen(fd,'w') as file:json.dump(config,file)
        temp.replace(CONFIG)
        wireless=WiFiTransport(key,ip)
        try:
            from display import render,pack
            wireless.send_frame(pack(render({})),1)
        finally:wireless.close()
        RESULT.write_text(json.dumps({'success':True,'ip':ip,'transport':'wifi'}))
        return '已连接 Wi-Fi，无线画面传输也已通过验证！现在可以把 USB 从电脑移到充电器上供电。'
    finally:
        if port:port.close()

PAGE='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>OLED Wi-Fi 设置</title>
<style>body{background:#111318;color:#f5f5f5;font:16px system-ui;max-width:520px;margin:55px auto;padding:24px}h1{font-size:30px}p{color:#bdc2cb;line-height:1.7}label{display:block;margin:20px 0 8px}input{box-sizing:border-box;width:100%;background:#242831;color:white;border:1px solid #505765;border-radius:9px;padding:13px;font-size:16px}button{background:#b4e6ca;border:0;border-radius:9px;font-size:17px;padding:14px 22px;margin-top:26px;cursor:pointer;width:100%}#status{margin-top:24px;line-height:1.8}</style>
<h1>给小屏幕连接 Wi-Fi</h1><p>先保持 ESP32 通过 USB 连接 电脑。输入一个 <b>2.4 GHz</b> 网络，电脑 也需在同一局域网。配置后会自动测试无线传输。</p>
<form><label>Wi-Fi 名称</label><input name="ssid" autocomplete="off" required maxlength="32"><label>Wi-Fi 密码</label><input name="password" type="password" autocomplete="off" maxlength="63"><button>连接并测试</button></form>
<p>密码只经 USB 写入开发板，不写入项目代码、日志或聊天。此页面仅在这台 电脑 上运行。</p><div id="status"></div>
<script>document.querySelector('form').onsubmit=async(e)=>{e.preventDefault();let f=e.target,b=f.querySelector('button'),s=document.querySelector('#status');b.disabled=true;s.textContent='正在配置并测试，请保持 USB 连接，可能需要约一分钟…';try{let r=await fetch(location.pathname,{method:'POST',body:new URLSearchParams(new FormData(f)),headers:{'Content-Type':'application/x-www-form-urlencoded'}});let d=await r.json();s.textContent=d.message;if(d.success){f.querySelector('[name=password]').value='';b.textContent='连接成功';return}}catch(err){s.textContent='连接中断，请重新打开配置页面重试。'}b.disabled=false;};</script></html>'''

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def valid(self):
        return self.path=='/'+TOKEN and self.headers.get('Host')==f'127.0.0.1:{self.server.server_port}'
    def send(self,code,content,kind):
        self.send_response(code);self.send_header('Content-Type',kind)
        self.send_header('Cache-Control','no-store');self.send_header('X-Frame-Options','DENY')
        self.end_headers();self.wfile.write(content)
    def do_GET(self):
        if not self.valid():self.send(404,b'Not found','text/plain');return
        self.send(200,PAGE.encode(),'text/html; charset=utf-8')
    def do_POST(self):
        if not self.valid() or self.headers.get('Origin')!=f'http://127.0.0.1:{self.server.server_port}':
            self.send(403,b'Forbidden','text/plain');return
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=2048:raise ValueError('输入过长，请检查网络名称。')
            body=urllib.parse.parse_qs(self.rfile.read(size).decode(),keep_blank_values=True)
            message=provision(body.get('ssid',[''])[0],body.get('password',[''])[0],self.server.device_port)
            result={'success':True,'message':message}
        except Exception as exc:
            # Only controlled human-readable errors are returned, never raw request data.
            message=str(exc) if isinstance(exc,(ValueError,RuntimeError)) else '配置未成功，请检查 USB 连接、网络信息后重试。'
            result={'success':False,'message':message}
            RESULT.write_text(json.dumps({'success':False,'message':message},ensure_ascii=False))
        self.send(200,json.dumps(result,ensure_ascii=False).encode(),'application/json; charset=utf-8')
        if result['success']:threading.Thread(target=self.server.shutdown,daemon=True).start()

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',help='Explicit ESP32 serial device, otherwise auto-detect')
    parser.add_argument('--no-browser',action='store_true',help='Print the local URL without opening a browser')
    args=parser.parse_args()
    ROOT.joinpath('.run').mkdir(exist_ok=True)
    RESULT.unlink(missing_ok=True)
    server=HTTPServer(('127.0.0.1',0),Handler)
    server.device_port=args.port
    url=f'http://127.0.0.1:{server.server_port}/{TOKEN}'
    INFO.write_text(json.dumps({'url':url,'pid':os.getpid()}))
    print('Wi-Fi setup page: '+url,flush=True)
    if not args.no_browser:webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close();INFO.unlink(missing_ok=True)
