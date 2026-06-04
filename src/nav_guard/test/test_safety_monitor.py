import unittest
from nav_guard.safety_monitor import SafetyMonitor
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

class MockLogger:
    def warn(self, msg):
        pass

class TestSafetyMonitor(unittest.TestCase):
    def test_pc_cb_emergency(self):
        class MockNode:
            def __init__(self):
                self.emergency_published = None
                self.pub = self.MockPub(self)
                self.logger = MockLogger()
            
            def get_parameter(self, name):
                class Param:
                    value = 100
                return Param()
            
            def get_logger(self):
                return self.logger

            class MockPub:
                def __init__(self, parent):
                    self.parent = parent
                def publish(self, msg):
                    self.parent.emergency_published = msg.data
        
        node = MockNode()
        msg = PointCloud2()
        msg.width = 10
        msg.height = 5  # 50 points < 100 threshold
        
        SafetyMonitor.pc_cb(node, msg)
        self.assertTrue(node.emergency_published)

    def test_pc_cb_safe(self):
        class MockNode:
            def __init__(self):
                self.emergency_published = None
                self.pub = self.MockPub(self)
                self.logger = MockLogger()
            
            def get_parameter(self, name):
                class Param:
                    value = 10
                return Param()
            
            def get_logger(self):
                return self.logger

            class MockPub:
                def __init__(self, parent):
                    self.parent = parent
                def publish(self, msg):
                    self.parent.emergency_published = msg.data
        
        node = MockNode()
        msg = PointCloud2()
        msg.width = 10
        msg.height = 5  # 50 points > 10 threshold
        
        SafetyMonitor.pc_cb(node, msg)
        self.assertFalse(node.emergency_published)

if __name__ == '__main__':
    unittest.main()
