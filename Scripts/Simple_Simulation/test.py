import unittest
import math
from Planar_manipulator import Planar3DOFRobotArm

class TestPlanar3DOFRobotArm(unittest.TestCase):
    def setUp(self):
        self.robot = Planar3DOFRobotArm()
    
    def test_forward_kinematics_geometric_solution(self):
        # Test case 1
        theta1 = 0.0
        theta2 = 0.0
        theta3 = 0.0
        expected_result = [self.robot.x_base + self.robot.l1 + self.robot.l2 + self.robot.l3,
                           self.robot.y_base,
                           theta1 + theta2 + theta3]
        result = self.robot.forward_kinematics_geometric_solution(theta1, theta2, theta3)
        self.assertEqual(result, expected_result)
        
        # Test case 2
        theta1 = math.pi / 2
        theta2 = math.pi / 4
        theta3 = -math.pi / 6
        expected_result = [-0.5243241690468861, 1.451100864343523, 1.8325957145940461]    #calculated by hand
        result = self.robot.forward_kinematics_geometric_solution(theta1, theta2, theta3)
        self.assertEqual(result, expected_result)
     
    def test_inverse_kinematics_geometric_solution(self):
        # Test case 1
        x_e = 0.5
        y_e = 0.8
        phi_e = -math.pi / 6
        expected_result = [2.008972864729709, -1.6210804386583237, -0.9114912016696841]  #calculated by hand
        result = self.robot.inverse_kinematics_geometric_solution(x_e, y_e, phi_e)
        self.assertEqual(result, expected_result)
        
     

if __name__ == '__main__':
    unittest.main()

