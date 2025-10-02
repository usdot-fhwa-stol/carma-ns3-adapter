import json
import socket
import time
import argparse

SEND_RATE = 10

class Registration:

    def __init__(self):
        parser = argparse.ArgumentParser(description="Vehicle Registration")
        parser.add_argument('--vehicleId', type=str, required=False, help='Unique identifier for the vehicle')
        parser.add_argument('--roleId', type=str, required=False, help='Role identifier for the vehicle')
        parser.add_argument('--rxMessageIpAddress', type=str, required=False, help='IP address of the message receiver')
        parser.add_argument('--rxMessagePort', type=int, required=False, help='Port number for message receiver')
        parser.add_argument('--rxTimeSyncPort', type=int, required=False, help='Port number for time synchronization')
        parser.add_argument('--receiverPort', type=int, required=False, help='Port number for the receiver')
        args = parser.parse_args()

        self.vehicleId = args.vehicleId if args.vehicleId else "carma_1"
        self.roleId = args.roleId if args.roleId else "carma_1"
        self.rxMessageIpAddress = args.rxMessageIpAddress if args.rxMessageIpAddress else "172.2.0.7"
        self.rxMessagePort = args.rxMessagePort if args.rxMessagePort else 2500
        self.rxTimeSyncPort = args.rxTimeSyncPort if args.rxTimeSyncPort else 2501
        self.receiverPort = args.receiverPort if args.receiverPort else 1515

    def compose_json_handshake_payload(self):
        output_payload = {}
        output_payload['vehicleId'] = self.vehicleId
        output_payload['roleId'] = self.roleId
        output_payload['rxMessageIpAddress'] = self.rxMessageIpAddress
        output_payload['rxMessagePort'] = self.rxMessagePort
        output_payload['rxTimeSyncPort'] = self.rxTimeSyncPort
        output_json = json.dumps(output_payload)

        return output_json

    def send(self):
        #package to json
        handshake_json = self.compose_json_handshake_payload()

        #send data with the rate of 10Hz
        while True:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                try:
                    s.connect((self.rxMessageIpAddress, self.receiverPort))
                    
                    while True:
                        s.sendall(handshake_json.encode('utf-8'))
                        print(f"Handshake sent, message: {handshake_json}")
                        time.sleep(1.0/SEND_RATE)
                except ConnectionError as e:
                    print(f"Connection error: {e}")
                    continue

if __name__ == "__main__":
    sender = Registration()
    sender.send()