import numpy as np # Scientific computing library
from ArtiBotEntity import ArtiBotEntity
# Link lengths in centimeters
a1 = 1 # Length of link 1
a2 = 1 # Length of link 2
a3 = 1 # Length of link 3


# Initialize values for the joint angles (degrees)
theta_1 = 90 # Joint 1
theta_2 = 0 # Joint 2
theta_3 = 0 # Joint 3

d_h_table = np.array([[np.deg2rad(theta_1), np.deg2rad(90), 0, a1],  #Read about the difference between np.array and np.matrix and which should I choose
                      [np.deg2rad(theta_2), 0, a2, 0],
                      [np.deg2rad(theta_3), 0, a3, 0],]) 
 

def ComputeTF(input_link,input_d_h_table):
    return np.array([[np.cos(input_d_h_table[input_link,0]), -np.sin(input_d_h_table[input_link,0]) * np.cos(input_d_h_table[input_link,1]), np.sin(input_d_h_table[input_link,0]) * np.sin(input_d_h_table[input_link,1]), input_d_h_table[input_link,2] * np.cos(input_d_h_table[input_link,0])],
                      [np.sin(input_d_h_table[input_link,0]), np.cos(input_d_h_table[input_link,0]) * np.cos(input_d_h_table[input_link,1]), -np.cos(input_d_h_table[input_link,0]) * np.sin(input_d_h_table[input_link,1]), input_d_h_table[input_link,2] * np.sin(input_d_h_table[input_link,0])],
                      [0, np.sin(input_d_h_table[input_link,1]), np.cos(input_d_h_table[input_link,1]), input_d_h_table[input_link,3]],
                      [0, 0, 0, 1]]) 
    

TF_0_1 = ComputeTF(0,d_h_table)
TF_1_2 = ComputeTF(1,d_h_table)
TF_2_3 = ComputeTF(2,d_h_table)

TF_0_3 =  TF_0_1 @ TF_1_2 @ TF_2_3

# Print the homogeneous transformation matrices


artibot = ArtiBotEntity()

end_effector = artibot.ForwardKinematicsSolution(np.pi/2,0,0)

print(end_effector)


