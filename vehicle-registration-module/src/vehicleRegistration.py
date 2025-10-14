import json
import socket
import time
import argparse
import struct
import logging

SEND_RATE = 10  # Hz
CONNECT_TIMEOUT = 5  # seconds
SEND_TIMEOUT = 5

class VehicleRegistrationSender:

    def __init__(self):
        parser = argparse.ArgumentParser(description="Vehicle Registration Handshake Sender")
        parser.add_argument('--vehicleId', type=str, default="carma_1", help='Unique identifier for the vehicle')
        parser.add_argument('--roleId', type=str, default="carma_1", help='Role identifier for the vehicle')
        parser.add_argument('--rxMessageIpAddress', type=str, default="172.2.0.7", help='IP address of the message receiver')
        parser.add_argument('--rxMessagePort', type=int, default=2500, help='Port number for message receiver')
        parser.add_argument('--rxTimeSyncPort', type=int, default=2501, help='Port number for time synchronization')
        parser.add_argument('--receiverPort', type=int, default=1515, help='Port number for the receiver')
        args = parser.parse_args()

        self.vehicleId = args.vehicleId
        self.roleId = args.roleId
        self.rxMessageIpAddress = args.rxMessageIpAddress
        self.rxMessagePort = args.rxMessagePort
        self.rxTimeSyncPort = args.rxTimeSyncPort
        self.receiverPort = args.receiverPort

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def compose_json_handshake_payload(self):
        output_payload = {
            'vehicleId': self.vehicleId,
            'roleId': self.roleId,
            'rxMessageIpAddress': self.rxMessageIpAddress,
            'rxMessagePort': self.rxMessagePort,
            'rxTimeSyncPort': self.rxTimeSyncPort
        }
        return json.dumps(output_payload)

    def send(self):
        handshake_json = self.compose_json_handshake_payload()
        message = handshake_json.encode('utf-8')

        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(CONNECT_TIMEOUT)
                    self.logger.info(f"Attempting to connect to {self.rxMessageIpAddress}:{self.receiverPort}")
                    s.connect((self.rxMessageIpAddress, self.receiverPort))
                    s.settimeout(SEND_TIMEOUT)
                    self.logger.info("Connected. Starting to send handshakes.")

                    while True:
                        try:
                            # Send length prefix + message (reliable framing)
                            payload = struct.pack('!I', len(message)) + message
                            s.sendall(payload)
                            self.logger.debug(f"Handshake sent: {handshake_json}")
                            time.sleep(1.0 / SEND_RATE)
                        except (BrokenPipeError, ConnectionResetError, OSError) as e:
                            self.logger.error(f"Send failed: {e}. Reconnecting immediately...")
                            break  # Break to reconnect
                # Immediate retry on connection close
                self.logger.info("Connection closed. Retrying immediately...")
                time.sleep(0.1)  # Minimal pause to avoid tight loop; set to 0 for true immediate
            except socket.timeout:
                self.logger.error("Connection timeout. Retrying immediately...")
                time.sleep(0.1)
                continue
            except ConnectionError as e:
                self.logger.error(f"Connection error: {e}. Retrying immediately...")
                time.sleep(0.1)
                continue
            except KeyboardInterrupt:
                self.logger.info("Shutting down.")
                break
            except Exception as e:
                self.logger.error(f"Unexpected error: {e}. Retrying...")
                time.sleep(0.1)
                continue

if __name__ == "__main__":
    sender = VehicleRegistrationSender()
    sender.send()