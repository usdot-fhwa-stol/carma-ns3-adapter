import unittest
from unittest.mock import patch, MagicMock, call
import json
import socket
import struct
import sys
import os

# Add project root if running directly
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.vehicleRegistration import VehicleRegistrationSender

class TestVehicleRegistrationSender(unittest.TestCase):

    def setUp(self):
        self.argv_patch = patch.object(sys, 'argv', ['script.py'])
        self.argv_patch.start()

    def tearDown(self):
        self.argv_patch.stop()

    def test_init_defaults(self):
        sender = VehicleRegistrationSender()
        self.assertEqual(sender.vehicleId, "carma_1")
        self.assertEqual(sender.roleId, "carma_1")
        self.assertEqual(sender.rxMessageIpAddress, "172.2.0.7")
        self.assertEqual(sender.rxMessagePort, 2500)
        self.assertEqual(sender.rxTimeSyncPort, 2501)
        self.assertEqual(sender.receiverPort, 1515)

    @patch.object(sys, 'argv', ['script.py', '--vehicleId', 'test_car', '--receiverPort', '9999'])
    def test_init_overrides(self):
        sender = VehicleRegistrationSender()
        self.assertEqual(sender.vehicleId, "test_car")
        self.assertEqual(sender.receiverPort, 9999)
        self.assertEqual(sender.roleId, "carma_1")

    def test_compose_json_handshake_payload(self):
        sender = VehicleRegistrationSender()
        sender.vehicleId = "test_id"
        sender.roleId = "test_role"
        sender.rxMessageIpAddress = "192.168.1.1"
        sender.rxMessagePort = 1234
        sender.rxTimeSyncPort = 5678
        expected = json.dumps({
            'vehicleId': "test_id",
            'roleId': "test_role",
            'rxMessageIpAddress': "192.168.1.1",
            'rxMessagePort': 1234,
            'rxTimeSyncPort': 5678
        })
        self.assertEqual(sender.compose_json_handshake_payload(), expected)

    @patch('time.sleep', return_value=None)
    @patch('src.vehicleRegistration.logging')
    @patch('socket.socket', autospec=True)
    def test_send_successful_sends(self, mock_socket_class, mock_logging, mock_sleep):
        mock_logger = MagicMock()
        mock_logging.getLogger.return_value = mock_logger

        mock_sock = MagicMock()
        mock_socket_class.return_value.__enter__.return_value = mock_sock

        sender = VehicleRegistrationSender()
        payload_json = '{"test": "payload"}'
        message_bytes = payload_json.encode('utf-8')
        expected_prefix = struct.pack('!I', len(message_bytes))

        send_count = [0]
        def side_effect_sendall(data):
            send_count[0] += 1
            self.assertTrue(data.startswith(expected_prefix))
            if send_count[0] == 2:
                raise OSError("Simulated disconnect")

        mock_sock.sendall.side_effect = side_effect_sendall

        connect_count = [0]
        def side_effect_connect(addr):
            connect_count[0] += 1
            if connect_count[0] == 2:
                raise KeyboardInterrupt()

        mock_sock.connect.side_effect = side_effect_connect

        with patch.object(sender, 'compose_json_handshake_payload', return_value=payload_json):
            sender.send()

        mock_socket_class.assert_called_with(socket.AF_INET, socket.SOCK_STREAM)
        mock_sock.settimeout.assert_has_calls([call(5), call(5)])
        mock_sock.connect.assert_called_with(('172.2.0.7', 1515))
        self.assertEqual(mock_sock.sendall.call_count, 2)
        mock_sleep.assert_has_calls([call(0.1)] * 2)
        mock_logger.info.assert_any_call(f"Attempting to connect to 172.2.0.7:1515")
        mock_logger.info.assert_any_call("Connected. Starting to send handshakes.")
        mock_logger.debug.assert_has_calls([call(f"Handshake sent: {payload_json}")])
        mock_logger.info.assert_any_call("Connection closed. Retrying immediately...")
        mock_logger.info.assert_any_call("Shutting down.")

    @patch('time.sleep', return_value=None)
    @patch('src.vehicleRegistration.logging')
    @patch('socket.socket', autospec=True)
    def test_send_connection_timeout_retry(self, mock_socket_class, mock_logging, mock_sleep):
        mock_logger = MagicMock()
        mock_logging.getLogger.return_value = mock_logger

        mock_sock = MagicMock()
        mock_socket_class.return_value.__enter__.return_value = mock_sock

        sender = VehicleRegistrationSender()

        connect_count = [0]
        def side_effect_connect(addr):
            connect_count[0] += 1
            if connect_count[0] == 1:
                raise socket.timeout()
            raise KeyboardInterrupt()

        mock_sock.connect.side_effect = side_effect_connect

        with patch.object(sender, 'compose_json_handshake_payload', return_value='{"test": "payload"}'):
            sender.send()

        mock_logger.error.assert_any_call("Connection timeout. Retrying immediately...")
        mock_sleep.assert_called_with(0.1)

    @patch('time.sleep', return_value=None)
    @patch('src.vehicleRegistration.logging')
    @patch('socket.socket', autospec=True)
    def test_send_broken_pipe_retry(self, mock_socket_class, mock_logging, mock_sleep):
        mock_logger = MagicMock()
        mock_logging.getLogger.return_value = mock_logger

        mock_sock = MagicMock()
        mock_socket_class.return_value.__enter__.return_value = mock_sock

        send_count = [0]
        def side_effect_sendall(data):
            send_count[0] += 1
            if send_count[0] == 1:
                raise OSError("Broken pipe")
            raise KeyboardInterrupt()

        mock_sock.sendall.side_effect = side_effect_sendall

        connect_count = [0]
        def side_effect_connect(addr):
            connect_count[0] += 1
            if connect_count[0] == 2:
                raise KeyboardInterrupt()
            return

        mock_sock.connect.side_effect = side_effect_connect

        sender = VehicleRegistrationSender()

        with patch.object(sender, 'compose_json_handshake_payload', return_value='{" тест": "payload"}'):
            sender.send()

        mock_sock.connect.assert_called()
        mock_logger.error.assert_any_call("Send failed: Broken pipe. Reconnecting immediately...")
        mock_logger.info.assert_any_call("Connection closed. Retrying immediately...")

    @patch('time.sleep', return_value=None)
    @patch('src.vehicleRegistration.logging')
    @patch('socket.socket', autospec=True)
    def test_send_keyboard_interrupt(self, mock_socket_class, mock_logging, mock_sleep):
        mock_logger = MagicMock()
        mock_logging.getLogger.return_value = mock_logger

        mock_sock = MagicMock()
        mock_socket_class.return_value.__enter__.return_value = mock_sock

        sender = VehicleRegistrationSender()

        def side_effect_connect(addr):
            raise KeyboardInterrupt()

        mock_sock.connect.side_effect = side_effect_connect

        with patch.object(sender, 'compose_json_handshake_payload'):
            sender.send()

        mock_logger.info.assert_called_with("Shutting down.")

if __name__ == '__main__':
    unittest.main()