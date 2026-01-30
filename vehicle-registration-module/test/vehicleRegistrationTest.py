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
import unittest
from unittest.mock import patch, Mock, call, ANY
import json
import socket
import logging
import sys
from io import StringIO

from src.vehicleRegistration import VehicleRegistrationSender, CONNECT_TIMEOUT, SEND_TIMEOUT

class TestVehicleRegistrationSender(unittest.TestCase):

    def setUp(self):
        self.patcher = patch('src.vehicleRegistration.logging.basicConfig')
        self.mock_basic_config = self.patcher.start()
        self.log_capture = StringIO()
        self.handler = logging.StreamHandler(self.log_capture)
        self.logger_patcher = patch('src.vehicleRegistration.logging.getLogger', return_value=logging.getLogger('mock_logger'))
        self.mock_get_logger = self.logger_patcher.start()
        self.mock_logger = logging.getLogger('mock_logger')
        self.mock_logger.addHandler(self.handler)
        self.mock_logger.setLevel(logging.DEBUG)

    def tearDown(self):
        self.patcher.stop()
        self.logger_patcher.stop()
        self.mock_logger.removeHandler(self.handler)

    def test_init_default_args(self):
        with patch('sys.argv', ['script.py']):
            sender = VehicleRegistrationSender()
        
        self.assertEqual(sender.vehicleId, "carma_1")
        self.assertEqual(sender.roleId, "carma_1")
        self.assertEqual(sender.rxMessageIpAddress, "172.2.0.7")
        self.assertEqual(sender.rxMessagePort, 2500)
        self.assertEqual(sender.rxTimeSyncPort, 2501)
        self.assertEqual(sender.receiverPort, 1515)
        self.assertEqual(sender.receiverIpAddress, "172.2.0.2")
        self.mock_basic_config.assert_called_with(level=logging.INFO)

    def test_init_custom_args(self):
        custom_args = [
            'script.py',
            '--vehicleId', 'test_vehicle',
            '--roleId', 'test_role',
            '--rxMessageIpAddress', '192.168.1.100',
            '--rxMessagePort', '3000',
            '--rxTimeSyncPort', '3001',
            '--receiverPort', '2000',
            '--receiverIpAddress', '192.168.1.200'
        ]
        with patch('sys.argv', custom_args):
            sender = VehicleRegistrationSender()
        
        self.assertEqual(sender.vehicleId, "test_vehicle")
        self.assertEqual(sender.roleId, "test_role")
        self.assertEqual(sender.rxMessageIpAddress, "192.168.1.100")
        self.assertEqual(sender.rxMessagePort, 3000)
        self.assertEqual(sender.rxTimeSyncPort, 3001)
        self.assertEqual(sender.receiverPort, 2000)
        self.assertEqual(sender.receiverIpAddress, "192.168.1.200")

    def test_compose_json_handshake_payload(self):
        with patch('sys.argv', ['script.py']):
            sender = VehicleRegistrationSender()
        
        payload = json.loads(sender.compose_json_handshake_payload())
        
        expected = {
            'vehicleId': 'carma_1',
            'roleId': 'carma_1',
            'rxMessageIpAddress': '172.2.0.7',
            'rxMessagePort': 2500,
            'rxTimeSyncPort': 2501
        }
        self.assertDictEqual(payload, expected)

    @patch('time.sleep', return_value=None)
    @patch('socket.socket')
    def test_send_successful_connection_and_send(self, mock_socket_class, mock_sleep):
        with patch('sys.argv', ['script.py']):
            sender = VehicleRegistrationSender()
        
        mock_socket = Mock()
        mock_socket_class.return_value.__enter__.return_value = mock_socket
        
        mock_socket.connect.return_value = None
        mock_socket.sendall.side_effect = [None, KeyboardInterrupt()]
        
        with patch.object(sender, 'compose_json_handshake_payload', return_value='{"test": "payload"}'):
            sender.send()
        
        mock_socket_class.assert_called_with(socket.AF_INET, socket.SOCK_STREAM)
        mock_socket.settimeout.assert_any_call(CONNECT_TIMEOUT)
        mock_socket.connect.assert_called_with(('172.2.0.2', 1515))
        mock_socket.settimeout.assert_any_call(SEND_TIMEOUT)
        
        log_output = self.log_capture.getvalue()
        self.assertIn("Attempting to connect to 172.2.0.2:1515", log_output)
        self.assertIn("Connected. Starting to send handshakes.", log_output)
        self.assertIn('Handshake sent: {"test": "payload"}', log_output)
        self.assertIn("Shutting down.", log_output)

    @patch('time.sleep', return_value=None)
    @patch('socket.socket')
    def test_send_connection_timeout(self, mock_socket_class, mock_sleep):
        with patch('sys.argv', ['script.py']):
            sender = VehicleRegistrationSender()
        
        mock_socket = Mock()
        mock_socket_class.return_value.__enter__.return_value = mock_socket
        
        connect_calls = []
        def connect_side_effect(*args):
            connect_calls.append(1)
            if len(connect_calls) >= 2:
                raise KeyboardInterrupt()
            raise socket.timeout()
        
        mock_socket.connect.side_effect = connect_side_effect
        
        sender.send()
        
        log_output = self.log_capture.getvalue()
        self.assertIn("Attempting to connect to 172.2.0.2:1515", log_output)
        self.assertIn("Connection timeout. Retrying immediately...", log_output)
        self.assertIn("Shutting down.", log_output)
        self.assertGreaterEqual(len(connect_calls), 2)

    @patch('time.sleep', return_value=None)
    @patch('socket.socket')
    def test_send_broken_pipe_during_send(self, mock_socket_class, mock_sleep):
        with patch('sys.argv', ['script.py']):
            sender = VehicleRegistrationSender()
        
        mock_socket = Mock()
        mock_socket_class.return_value.__enter__.return_value = mock_socket
        
        send_calls = []
        def send_side_effect(data):
            send_calls.append(1)
            if len(send_calls) == 1:
                return None
            else:
                raise BrokenPipeError()
        
        mock_socket.sendall.side_effect = send_side_effect
        
        reconnect_attempts = 0
        def connect_side_effect(*args):
            nonlocal reconnect_attempts
            reconnect_attempts += 1
            if reconnect_attempts >= 2:
                raise KeyboardInterrupt()
            return None
        
        mock_socket.connect.side_effect = connect_side_effect
        
        with patch.object(sender, 'compose_json_handshake_payload', return_value='{"test": "payload"}'):
            sender.send()
        
        log_output = self.log_capture.getvalue()
        self.assertIn("Attempting to connect to 172.2.0.2:1515", log_output)
        self.assertIn("Connected. Starting to send handshakes.", log_output)
        self.assertIn('Handshake sent: {"test": "payload"}', log_output)
        self.assertIn("Send failed: ", log_output)
        self.assertIn("Reconnecting immediately...", log_output)
        self.assertIn("Connection closed. Retrying immediately...", log_output)
        self.assertIn("Shutting down.", log_output)
        self.assertEqual(len(send_calls), 2)
        self.assertEqual(reconnect_attempts, 2)

    @patch('time.sleep', return_value=None)
    @patch('socket.socket')
    def test_send_connection_reset(self, mock_socket_class, mock_sleep):
        with patch('sys.argv', ['script.py']):
            sender = VehicleRegistrationSender()
        
        mock_socket = Mock()
        mock_socket_class.return_value.__enter__.return_value = mock_socket
        
        mock_socket.sendall.side_effect = ConnectionResetError()
        
        reconnect_attempts = 0
        def connect_side_effect(*args):
            nonlocal reconnect_attempts
            reconnect_attempts += 1
            if reconnect_attempts >= 2:
                raise KeyboardInterrupt()
            return None
        
        mock_socket.connect.side_effect = connect_side_effect
        
        with patch.object(sender, 'compose_json_handshake_payload', return_value='{"test": "payload"}'):
            sender.send()
        
        log_output = self.log_capture.getvalue()
        self.assertIn("Attempting to connect to 172.2.0.2:1515", log_output)
        self.assertIn("Connected. Starting to send handshakes.", log_output)
        self.assertIn("Send failed: ", log_output)
        self.assertIn("Reconnecting immediately...", log_output)
        self.assertIn("Connection closed. Retrying immediately...", log_output)
        self.assertIn("Shutting down.", log_output)

    @patch('time.sleep', return_value=None)
    @patch('socket.socket')
    def test_send_unexpected_exception(self, mock_socket_class, mock_sleep):
        with patch('sys.argv', ['script.py']):
            sender = VehicleRegistrationSender()
        
        mock_socket = Mock()
        mock_socket_class.return_value.__enter__.return_value = mock_socket
        
        attempts = 0
        def connect_side_effect(*args):
            nonlocal attempts
            attempts += 1
            if attempts >= 2:
                raise KeyboardInterrupt()
            raise Exception("Unexpected")
        
        mock_socket.connect.side_effect = connect_side_effect
        
        sender.send()
        
        log_output = self.log_capture.getvalue()
        self.assertIn("Attempting to connect to 172.2.0.2:1515", log_output)
        self.assertIn("Unexpected error: Unexpected. Retrying...", log_output)
        self.assertIn("Shutting down.", log_output)

if __name__ == '__main__':
    unittest.main()