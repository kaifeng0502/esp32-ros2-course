"""Authenticated local UDP display transport. No account credentials go to ESP32."""
import hashlib
import hmac
import json
import re
import secrets
import socket
import struct
import subprocess
import time
from display import ROOT

PORT=43821
CONFIG=ROOT/'.run/wifi.json'

def signed(key,payload):
    return payload+hmac.new(key,payload,hashlib.sha256).digest()

def verified(key,message):
    return len(message)>=32 and hmac.compare_digest(message[-32:],hmac.new(key,message[:-32],hashlib.sha256).digest())

def broadcast_addresses():
    addresses={'255.255.255.255'}
    try:
        result=subprocess.run(['/sbin/ifconfig'],capture_output=True,text=True,timeout=3)
        addresses.update(re.findall(r'\bbroadcast (\d+\.\d+\.\d+\.\d+)',result.stdout))
    except (OSError,subprocess.TimeoutExpired): pass
    return addresses

class WiFiTransport:
    def __init__(self,key,last_ip=None):
        self.key=key
        if len(key)!=32: raise ValueError('Invalid Wi-Fi pairing key')
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        self.sock.settimeout(.25)
        try:
            self.ip=self.discover(last_ip)
            self.port='wifi:'+self.ip
        except Exception:
            self.sock.close()
            raise
    @classmethod
    def from_config(cls):
        try:
            config=json.loads(CONFIG.read_text())
            return cls(bytes.fromhex(config['key']),config.get('ip'))
        except (KeyError,ValueError,FileNotFoundError) as exc:
            raise RuntimeError('Configure Wi-Fi with python host/setup_wifi.py first') from exc
    def discover(self,last_ip):
        nonce=secrets.token_bytes(16)
        request=signed(self.key,b'COD1'+nonce)
        targets=[last_ip] if last_ip else []
        # A direct probe avoids relying on broadcast on networks that filter it.
        for target_set in (targets, list(broadcast_addresses()),list(broadcast_addresses())):
            for ip in target_set:
                try:self.sock.sendto(request,(ip,PORT))
                except OSError:continue
            if not target_set:continue
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                try:reply,source=self.sock.recvfrom(1500)
                except socket.timeout:continue
                if len(reply)==52 and reply[:20]==b'COR1'+nonce and verified(self.key,reply):
                    return source[0]
        raise TimeoutError('ESP32 Wi-Fi not found; check power and same LAN')
    def send_frame(self,frame,sequence):
        seq=struct.pack('<I',sequence)
        message=signed(self.key,b'COF1'+seq+frame)
        for _ in range(2):
            self.sock.sendto(message,(self.ip,PORT))
            deadline=time.monotonic()+.8
            while time.monotonic()<deadline:
                try:response,source=self.sock.recvfrom(1500)
                except socket.timeout:continue
                if source[0]==self.ip and len(response)==40 and response[:8]==b'COA1'+seq and verified(self.key,response):
                    return
        raise TimeoutError('No Wi-Fi display acknowledgement')
    def close(self):self.sock.close()
