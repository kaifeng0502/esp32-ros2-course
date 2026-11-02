from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"host"))
import socket
import struct
import threading
import unittest
from unittest.mock import patch
from wifi_transport import WiFiTransport, signed, verified, PORT

class WiFiTests(unittest.TestCase):
    def test_wrong_key_and_changed_payload_rejected(self):
        key=b'x'*32
        message=signed(key,b'COD1'+b'n'*16)
        self.assertTrue(verified(key,message))
        self.assertFalse(verified(b'y'*32,message))
        self.assertFalse(verified(key,b'X'+message[1:]))
        self.assertFalse(verified(key,b'bad'))
    def test_discovery_frame_and_ack_on_local_udp(self):
        key=b't'*32
        received=[]
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server.bind(('127.0.0.1',PORT));server.settimeout(4)
        def device():
            try:
                request,source=server.recvfrom(1500)
                if verified(key,request):
                    server.sendto(signed(key,b'COR1'+request[4:20]),source)
                request,source=server.recvfrom(1500)
                received.append(request)
                # Ignore an acknowledgement forged with a different key.
                server.sendto(signed(b'z'*32,b'COA1'+request[4:8]),source)
                server.sendto(signed(key,b'COA1'+request[4:8]),source)
            finally:server.close()
        worker=threading.Thread(target=device);worker.start()
        transport=WiFiTransport(key,'127.0.0.1')
        try:transport.send_frame(bytes([0xAA])*1024,123)
        finally:transport.close();worker.join()
        self.assertEqual(received[0][:8],b'COF1'+struct.pack('<I',123))
        self.assertEqual(received[0][8:-32],bytes([0xAA])*1024)
        self.assertTrue(verified(key,received[0]))

if __name__=='__main__':unittest.main()
