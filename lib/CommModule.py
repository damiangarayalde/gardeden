import socket
import json


class CommModule:
    def __init__(self, udp_address: str, udp_port: int, bind_ip="0.0.0.0"):
        """
        udp_address: destination address (ESP32 or PC)
        udp_port: destination port
        bind_ip: local bind address (default all interfaces)
        """
        self.udp_address = udp_address
        self.udp_port = udp_port

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, udp_port))
        self.sock.settimeout(0.5)  # non-blocking-ish

        print(f"[CommModule] Listening on {bind_ip}:{udp_port}")

    def send(self, kv: dict):
        """
        Send dictionary as JSON over UDP.
        """
        try:
            msg = json.dumps(kv)
            self.sock.sendto(msg.encode(), (self.udp_address, self.udp_port))
            print(f"[CommModule] Sent: {msg}")
        except OSError as e:
            print(f"[CommModule] Send error: {e}")

    def receive(self) -> dict:
        """
        Non-blocking receive.
        Returns dict if valid JSON, else {}.
        """
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().strip()
            kv = json.loads(msg)
            print(f"[CommModule] Received from {addr}: {kv}")
            return kv
        except socket.timeout:
            return {}
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            print(f"[CommModule] Invalid JSON received: {e}")
            return {}
        except OSError as e:
            print(f"[CommModule] Receive error: {e}")
            return {}
