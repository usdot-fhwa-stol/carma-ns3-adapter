#   Copyright (C) 2025 LEIDOS.
#
#   Licensed under the Apache License, Version 2.0 (the "License"); you may not
#   use this file except in compliance with the License. You may obtain a copy of
#   the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#   WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#   License for the specific language governing permissions and limitations under
#   the License.
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
        parser.add_argument('--receiverPort', type=int, default=1515, help='Port number for the mosaic receiver')
        parser.add_argument('--receiverIpAddress', type=str, default="172.2.0.2", help="Ip address for the mosaic receiver")
        args = parser.parse_args()

        self.vehicleId = args.vehicleId
        self.roleId = args.roleId
        self.rxMessageIpAddress = args.rxMessageIpAddress
        self.rxMessagePort = args.rxMessagePort
        self.rxTimeSyncPort = args.rxTimeSyncPort
        self.receiverPort = args.receiverPort
        self.receiverIpAddress = args.receiverIpAddress

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def compose_json_handshake_payload(self):
        output_payload = {
            'vehicleId': self.vehicleId,
            'vehicleRole': self.roleId,
            'rxMessageIpAddress': self.rxMessageIpAddress,
            'rxMessagePort': self.rxMessagePort,
            'rxTimeSyncPort': self.rxTimeSyncPort
        }
        return json.dumps(output_payload)

    def send(self):
        handshake_json = self.compose_json_handshake_payload()
        message = handshake_json.encode('utf-8')

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(SEND_TIMEOUT)

        self.logger.info(
            f"Sending UDP handshakes to {self.receiverIpAddress}:{self.receiverPort}"
        )

        try:
            while True:
                sock.sendto(message, (self.receiverIpAddress, self.receiverPort))
                self.logger.info(f"Handshake sent: {handshake_json}")
                time.sleep(1.0 / SEND_RATE)

        except KeyboardInterrupt:
            self.logger.info("Shutting down sender.")

        finally:
            sock.close()

if __name__ == "__main__":
    sender = VehicleRegistrationSender()
    sender.send()
