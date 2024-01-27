#!/usr/bin/env python3
import math
import numpy as np 

class ArtiBotEntity():
    def __init__(self):
        self.x_base = 0.0
        self.y_base = 0.0
        self.z_base = 0.0
        self.l1 = 3 
        self.l2 = 2 
        self.l3 = 1
        self.robot_xdata =[] #save the xdata of each joint
        self.robot_ydata =[] #save the ydata of each joint
        self.robot_zdata =[] #save the ydata of each joint
        #self.d_h_table =  self.UpdateDHTable(input_theta1=0,input_theta2=0,input_theta3=0) #Initalize the dh Table 

    def UpdateDHTable(self,input_theta1,input_theta2,input_theta3):
        #This the DH paramters calculated on paper and hardcoded for ArtiBot
        return np.array([[input_theta1, np.pi/2, 0, self.l1],  #Read about the difference between np.array and np.matrix and which should I choose
                      [input_theta2, 0, self.l2, 0],
                      [input_theta3, 0, self.l3, 0],])
    
    def ComputeTF(self,input_link_number,input_d_h_table):
        #This function calculate the TF for each link
        return np.array([[np.cos(input_d_h_table[input_link_number,0]), -np.sin(input_d_h_table[input_link_number,0]) * np.cos(input_d_h_table[input_link_number,1]), np.sin(input_d_h_table[input_link_number,0]) * np.sin(input_d_h_table[input_link_number,1]), input_d_h_table[input_link_number,2] * np.cos(input_d_h_table[input_link_number,0])],
                      [np.sin(input_d_h_table[input_link_number,0]), np.cos(input_d_h_table[input_link_number,0]) * np.cos(input_d_h_table[input_link_number,1]), -np.cos(input_d_h_table[input_link_number,0]) * np.sin(input_d_h_table[input_link_number,1]), input_d_h_table[input_link_number,2] * np.sin(input_d_h_table[input_link_number,0])],
                      [0, np.sin(input_d_h_table[input_link_number,1]), np.cos(input_d_h_table[input_link_number,1]), input_d_h_table[input_link_number,3]],
                      [0, 0, 0, 1]])
   
    def update_robot_data(self,input_xdata,input_ydata, input_zdata):
        # update the robot data
        self.robot_xdata,self.robot_ydata,self.robot_zdata=input_xdata,input_ydata,input_zdata
    
    def get_robot_xyzdata(self):
        # get the xy data of the robot of each joint
        return [self.robot_xdata,self.robot_ydata,self.robot_zdata]
    
    def ForwardKinematicsSolution(self,input_theta1,input_theta2,input_theta3):
        #Geometric solution for the forward_kinematics problem

        self.d_h_table =  self.UpdateDHTable(input_theta1,input_theta2,input_theta3)
        #Loop over number of links
        TF_0_1 = self.ComputeTF(0,self.d_h_table)
        TF_1_2 = self.ComputeTF(1,self.d_h_table)
        TF_2_3 = self.ComputeTF(2,self.d_h_table)
        
        TF_0_2 =  TF_0_1 @ TF_1_2
        TF_0_3 =  TF_0_1 @ TF_1_2 @ TF_2_3

        x0,y0,z0 = self.x_base,self.y_base,self.z_base
        x1,y1,z1 = x0 + TF_0_1[0,3],y0 + TF_0_1[1,3],z0 + TF_0_1[2,3]
        x2,y2,z2 = x1 + TF_1_2[0,3],y1 + TF_1_2[1,3],z1 + TF_1_2[2,3]
        x3,y3,z3 = x2 + TF_2_3[0,3],y2 + TF_2_3[1,3],z2 + TF_2_3[2,3]

        #Do I need the Phi End Effector??
        phi_end_effector = input_theta1 + input_theta2 + input_theta3

        self.update_robot_data([x0,x1, x2,  x3, TF_0_3[0,3]],
                               [y0,y1, y2, y3, TF_0_3[1,3]],
                               [z0,z1, z2, z3, TF_0_3[2,3]])

 
        # self.update_robot_data([x0,TF_0_1[0,3], TF_1_2[0,3], TF_0_2[0,3], TF_0_3[0,3]],
        #                        [y0,TF_0_1[1,3], TF_1_2[1,3], TF_0_2[1,3], TF_0_3[1,3]],
        #                        [z0,TF_0_1[2,3], TF_1_2[2,3], TF_0_2[2,3], TF_0_3[2,3]])
        return [TF_0_3[0,3],TF_0_3[1,3],TF_0_3[2,3],phi_end_effector]
  
    # def inverse_kinematics_geometric_solution(self,x_e,y_e,phi_e,elbow_up=True): 
    #    #Geometric solution for the Inverse_kinematics problem        
    #     x_w = x_e - self.l3 * math.cos(phi_e)
    #     y_w = y_e - self.l3 * math.sin(phi_e)
    #     beta = math.atan2(y_w,x_w)

    #     if elbow_up:
    #         # ELBOW UP
    #         theta1 = beta + math.acos(max(min((math.pow(x_w,2) + math.pow(y_w,2) + math.pow(self.l1,2) - math.pow(self.l2,2)) / (2 * self.l1 * math.sqrt(math.pow(x_w,2) + math.pow(y_w,2))), 1), -1)) 
    #         theta2 = -(math.pi - math.acos(max(min((math.pow(self.l1,2) + math.pow(self.l2,2) - math.pow(x_w,2) - math.pow(y_w,2)) / (2 * self.l1 * self.l2), 1), -1)))
    #         theta3 =  phi_e - theta1 -theta2
            
    #     else:
    #         # ELBOW DOWN
    #         theta1 = beta - math.acos(max(min((math.pow(x_w,2) + math.pow(y_w,2) + math.pow(self.l1,2) - math.pow(self.l2,2)) / (2 * self.l1 * math.sqrt(math.pow(x_w,2) + math.pow(y_w,2))), 1), -1))
    #         theta2 = math.pi - math.acos(max(min((math.pow(self.l1,2) + math.pow(self.l2,2) - math.pow(x_w,2) - math.pow(y_w,2)) / (2 * self.l1 * self.l2), 1), -1))
    #         theta3 = phi_e - theta1 - theta2    
    #     return [theta1,theta2,theta3]
    
    def get_arm_length(self):
        return abs(self.l1 + self.l2+self.l3)
