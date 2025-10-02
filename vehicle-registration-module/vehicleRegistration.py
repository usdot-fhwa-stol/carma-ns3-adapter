import os
import json
import socket
import time

REGISTRATION_PORT = 1515
SEND_RATE = 10

class Registration:

    def __init__(self):
        self.vehicleId = os.environ.get('VEHICLEID', 'carma_1')
        self.roleId = os.environ.get('ROLEID', 'carma_1')
        self.rxMessageIpAddress = os.environ.get('RXMESSAGEIPADDRESS', '172.2.0.7')
        self.rxMessagePort = os.environ.get('RXMESSAGEPORT', 2500)
        self.rxTimeSyncPort = os.environ.get('RXTIMESYNCPORT', 2501)

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
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((self.rxMessageIpAddress, REGISTRATION_PORT))
                
                while True:
                    s.sendall(handshake_json.encode('utf-8'))
                    print(f"Handshake sent, message: {handshake_json}")
                    time.sleep(1.0/SEND_RATE)
            except ConnectionError as e:
                print(f"Connection error: {e}")

if __name__ == "__main__":
    sender = Registration()
    sender.send()