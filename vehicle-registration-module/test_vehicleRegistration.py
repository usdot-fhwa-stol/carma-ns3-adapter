import unittest
import json
import os
from unittest.mock import patch, MagicMock
from vehicleRegistration import Registration  # Assuming the original script is in registration.py

class TestRegistration(unittest.TestCase):
    def setUp(self):
        # Reset environment variables before each test
        for var in ['VEHICLEID', 'ROLEID', 'RXMESSAGEIPADDRESS', 'RXMESSAGEPORT', 'RXTIMESYNCPORT', 'REGISTRATIONPORT']:
            if var in os.environ:
                del os.environ[var]
        self.registration = Registration()

    def test_init_with_default_values(self):
        """Test initialization with default environment variables."""
        self.assertEqual(self.registration.vehicleId, 'carma_1')
        self.assertEqual(self.registration.roleId, 'carma_1')
        self.assertEqual(self.registration.rxMessageIpAddress, '172.2.0.7')
        self.assertEqual(self.registration.rxMessagePort, 2500)
        self.assertEqual(self.registration.rxTimeSyncPort, 2501)
        self.assertEqual(self.registration.registrationPort, 1515)

    def test_init_with_custom_env_vars(self):
        """Test initialization with custom environment variables."""
        os.environ['VEHICLEID'] = 'test_vehicle'
        os.environ['ROLEID'] = 'test_role'
        os.environ['RXMESSAGEIPADDRESS'] = '192.168.1.1'
        os.environ['RXMESSAGEPORT'] = '3000'
        os.environ['RXTIMESYNCPORT'] = '3001'
        os.environ['REGISTRATIONPORT'] = '1516'
        
        registration = Registration()
        
        self.assertEqual(registration.vehicleId, 'test_vehicle')
        self.assertEqual(registration.roleId, 'test_role')
        self.assertEqual(registration.rxMessageIpAddress, '192.168.1.1')
        self.assertEqual(registration.rxMessagePort, 3000)
        self.assertEqual(registration.rxTimeSyncPort, 3001)
        self.assertEqual(registration.registrationPort, 1516)

    def test_compose_json_handshake_payload(self):
        """Test JSON handshake payload composition."""
        expected_payload = {
            'vehicleId': 'carma_1',
            'roleId': 'carma_1',
            'rxMessageIpAddress': '172.2.0.7',
            'rxMessagePort': 2500,
            'rxTimeSyncPort': 2501
        }
        result = self.registration.compose_json_handshake_payload()
        self.assertEqual(json.loads(result), expected_payload)

    @patch('socket.socket')
    def test_send_successful_connection(self, mock_socket):
        """Test send method with successful connection."""
        mock_instance = MagicMock()
        mock_socket.return_value.__enter__.return_value = mock_instance
        
        # Run send in a controlled way to avoid infinite loop
        with patch('time.sleep', return_value=None):
            self.registration.send()
            
        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_STREAM)
        mock_instance.connect.assert_called_with(('172.2.0.7', 1515))
        mock_instance.sendall.assert_called()
        self.assertTrue(mock_instance.sendall.called)

    @patch('socket.socket')
    def test_send_connection_error(self, mock_socket):
        """Test send method with connection error."""
        mock_instance = MagicMock()
        mock_instance.connect.side_effect = ConnectionError("Connection failed")
        mock_socket.return_value.__enter__.return_value = mock_instance
        
        # Run send in a controlled way to avoid infinite loop
        with patch('time.sleep', return_value=None):
            with self.assertRaises(ConnectionError):
                self.registration.send()
                
        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_STREAM)
        mock_instance.connect.assert_called_with(('172.2.0.7', 1515))

if __name__ == '__main__':
    unittest.main()