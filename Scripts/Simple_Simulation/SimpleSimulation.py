import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation 
from ArtiBotEntity import ArtiBotEntity
from matplotlib.backend_bases import MouseButton
import math

from mpl_toolkits.mplot3d import Axes3D



# TODO: Handle The singularity

artibot = ArtiBotEntity()

# This function move the robot on the plot
def move_robot(theta1,theta2,theta3):
    artibot.ForwardKinematicsSolution(theta1,theta2,theta3)
    xyz_data = artibot.get_robot_xyzdata()
    line.set_data_3d(xyz_data[0],xyz_data[1],xyz_data[2])
    fig.canvas.draw_idle() 


# Define initial thetas
init_theta1 = math.pi/2
init_theta2 = math.pi/2
init_theta3 =  math.pi/2
# Create the figure and the line that we will manipulate
#fig, ax = plt.subplots()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

line, = ax.plot( artibot.robot_xdata,artibot.robot_ydata,artibot.robot_zdata,marker='o', lw=2)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')  # Z-label for 3D plot

# adjust the main plot to make room for the sliders
#fig.subplots_adjust(left=0.25, bottom=0.25)

# Determine maximum reach of arm.
max_reach = artibot.get_arm_length()

# Set axis limits based on reach from base joint.
ax.set_xlim(artibot.x_base - 1.3 * max_reach, artibot.x_base + 1.3 * max_reach)
ax.set_ylim(artibot.y_base - 1.3 * max_reach, artibot.y_base + 1.3 * max_reach)
ax.set_zlim(0, 5)

# Add dashed circle to plot indicating reach.
# circle = plt.Circle((artibot.x_base, artibot.y_base), max_reach, ls='dashed', fill=False)
# ax.add_artist(circle)

# # move robot to initial pose
move_robot(init_theta1,init_theta2,init_theta3)


# # Make a vertically oriented slider to control the thetas
# theta1_bar_location = fig.add_axes([0.05, 0.25, 0.0225, 0.63])
# theta1_slider = Slider(ax=theta1_bar_location,label="θ1",valmin=0,valmax=360,valinit=init_theta1,orientation="vertical")
# theta2_bar_location = fig.add_axes([0.1, 0.25, 0.0225, 0.63])
# theta2_slider = Slider(ax=theta2_bar_location,label="θ2",valmin=0,valmax=360,valinit=init_theta2,orientation="vertical")
# theta3_bar_location = fig.add_axes([0.15, 0.25, 0.0225, 0.63])
# theta3_slider = Slider(ax=theta3_bar_location,label="θ3", valmin=0,valmax=360,valinit=init_theta3,orientation="vertical")


# def calculate_mouse_orientation(x, y):
#     # Calculate the slope angle
#     robotdataxy =artibot.get_robot_xydata()
#     robotdatax,robotdatay =robotdataxy[0],robotdataxy[1]
#     dx = x - robotdatax[-1]
#     dy = y - robotdatay[-1]
#     slope_angle = math.atan2(dy, dx)

#     # Normalize the angle within 0 to 2π range
#     normalized_angle = slope_angle % (2 * math.pi)    
#     return normalized_angle


# def on_click(event):
#     if event.button is MouseButton.LEFT and isinstance(event.xdata, float)and isinstance(event.ydata, float):
#         if math.sqrt(math.pow(event.xdata,2) + math.pow(event.ydata,2)) <= max_reach:
#             phi=calculate_mouse_orientation(event.xdata,event.ydata)
#             ik = robot.inverse_kinematics_geometric_solution(event.xdata,event.ydata,phi)
#             move_robot(ik[0],ik[1],ik[2])


# # The function to be called anytime a slider's value changes
# def update(val):
#     move_robot(math.radians(theta1_slider.val),math.radians(theta2_slider.val),math.radians(theta3_slider.val))

# # register the update function with each slider
# theta1_slider.on_changed(update)
# theta2_slider.on_changed(update)
# theta3_slider.on_changed(update)

# # plt.connect('button_press_event', on_click)
plt.show()