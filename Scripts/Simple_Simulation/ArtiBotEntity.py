#!/usr/bin/env python3
import math
import numpy as np 

class ArtiBotEntity():
    def __init__(self):
        self.x_base = 0.0
        self.y_base = 0.0
        self.z_base = 0.0
        self.l1 = 1
        self.l2 = 1 
        self.l3 = 1
        self.robot_xdata =[] #save the xdata of each joint
        self.robot_ydata =[] #save the ydata of each joint
        self.robot_zdata =[] #save the ydata of each joint
        self.d_h_table =  self.UpdateDHTable(input_theta1=0,input_theta2=0,input_theta3=0) #Initalize the dh Table 

    def UpdateDHTable(self,input_theta1,input_theta2,input_theta3):
        #This the DH paramters calculated on paper and hardcoded for ArtiBot
        return np.array([[input_theta1, math.pi/2, 0, self.l1],  #Read about the difference between np.array and np.matrix and which should I choose
                      [input_theta2, 0, self.l2, 0],
                      [input_theta3, 0, self.l3, 0]])
    
    def ComputeTF(self,input_link_number,input_d_h_table):
        #This function calculate the TF for each link
        return np.array([[np.cos(input_d_h_table[input_link_number,0]), -np.sin(input_d_h_table[input_link_number,0]) * np.cos(input_d_h_table[input_link_number,1]), np.sin(input_d_h_table[input_link_number,0]) * np.sin(input_d_h_table[input_link_number,1]), input_d_h_table[input_link_number,2] * np.cos(input_d_h_table[input_link_number,0])],
                      [np.sin(input_d_h_table[input_link_number,0]), np.cos(input_d_h_table[input_link_number,0]) * np.cos(input_d_h_table[input_link_number,1]), -np.cos(input_d_h_table[input_link_number,0]) * np.sin(input_d_h_table[input_link_number,1]), input_d_h_table[input_link_number,2] * np.sin(input_d_h_table[input_link_number,0])],
                      [0, np.sin(input_d_h_table[input_link_number,1]), np.cos(input_d_h_table[input_link_number,1]), input_d_h_table[input_link_number,3]],
                      [0, 0, 0, 1]])
   
    def update_robot_data(self,input_xdata,input_ydata, input_zdata):
        # update the robot data
        self.robot_xdata,self.robot_ydata,self.robot_zdata =input_xdata,input_ydata,input_zdata
    
    def get_robot_xyzdata(self):
        # get the xy data of the robot of each joint
        return [self.robot_xdata,self.robot_ydata,self.robot_zdata]
    
    def ForwardKinematicsSolution(self, input_theta1, input_theta2, input_theta3):
        d_h_table = self.UpdateDHTable(input_theta1, input_theta2, input_theta3)

        TF_0_1 = self.ComputeTF(0, d_h_table)
        TF_1_2 = self.ComputeTF(1, d_h_table)
        TF_2_3 = self.ComputeTF(2, d_h_table)

        # Cumulative Transformations
        TF_0_2 = TF_0_1 @ TF_1_2
        TF_0_3 = TF_0_2 @ TF_2_3

        # Update robot data based on transformation matrices
        self.update_robot_data([self.x_base, TF_0_1[0, 3], TF_0_2[0, 3], TF_0_3[0, 3]],
                               [self.y_base, TF_0_1[1, 3], TF_0_2[1, 3], TF_0_3[1, 3]],
                               [self.z_base, TF_0_1[2, 3], TF_0_2[2, 3], TF_0_3[2, 3]])
        return [TF_0_3[0, 3], TF_0_3[1, 3], TF_0_3[2, 3]]
 
    def inverse_kinematics(self,x_goal, y_goal, z_goal):
        # Please Revise theta2 I need more test
        
        theta1 = math.atan2(y_goal, x_goal)
        
        # Correcting calculations for theta2 and theta3
        # First, find the distance to the goal in the XY plane
        r_xy = math.sqrt(x_goal**2 + y_goal**2)
        
        # Projected goal distance considering offset by l1
        r = math.sqrt(r_xy**2 + (z_goal - self.l1)**2)
        
        # Law of cosines for theta2 and theta3 
        cos_theta3 = (r**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_theta3 = max(min(cos_theta3, 1), -1)  # Clamp to [-1, 1] to ensure it's within the valid range
        theta3 = -math.acos(cos_theta3)  # Use the clamped value to avoid math domain error
        # Angle to the projected point
        alpha = math.atan2(z_goal - self.l1, r_xy)
        # Law of cosines for the inner angle at l2
        cos_beta = (self.l2**2 + r**2 - self.l3**2) / (2 * self.l2 * r)
        cos_beta = max(min(cos_beta, 1), -1)  # Clamp to [-1, 1] to ensure it's within the valid range
        beta = math.acos(cos_beta)
        
        theta2 = alpha + beta
        return (theta1, theta2, theta3)

    def get_arm_length(self):
        return abs(self.l1 + self.l2+self.l3)
