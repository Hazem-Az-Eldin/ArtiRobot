import matplotlib.pyplot as plt
from ArtiBotEntity import ArtiBotEntity
from matplotlib.widgets import Slider, Button
import math
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

artibot = ArtiBotEntity()


def generate_circle_waypoints(x_center, y_center, z_center, r_circle, N):
    waypoints = []
    for i in range(N):
        theta = 2 * math.pi * i / N  # Angle for the current waypoint
        x_goal = x_center + r_circle * math.cos(theta)
        y_goal = y_center + r_circle * math.sin(theta)
        waypoints.append((x_goal, y_goal, z_center))  # Z remains constant for a flat circle
    return waypoints


def move_robot(theta1, theta2, theta3):
    artibot.ForwardKinematicsSolution(theta1, theta2, theta3)
    xyz_data = artibot.get_robot_xyzdata()
    line.set_data_3d(xyz_data[0], xyz_data[1], xyz_data[2])
    fig.canvas.draw_idle()
    plt.pause(0.05)  # Pause briefly to show update


init_theta1 = 0
init_theta2 = 0
init_theta3 = 0

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

line, = ax.plot(artibot.robot_xdata, artibot.robot_ydata, artibot.robot_zdata, marker='o', lw=2)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

max_reach = artibot.get_arm_length()
ax.set_xlim(artibot.x_base - 1.3 * max_reach, artibot.x_base + 1.3 * max_reach)
ax.set_ylim(artibot.y_base - 1.3 * max_reach, artibot.y_base + 1.3 * max_reach)
ax.set_zlim(0, 5)


# Make a vertically oriented slider to control the thetas
# theta1_bar_location = fig.add_axes([0.05, 0.25, 0.0225, 0.63])
# theta1_slider = Slider(ax=theta1_bar_location,label="θ1",valmin=0,valmax=360,valinit=init_theta1,orientation="vertical")
# theta2_bar_location = fig.add_axes([0.1, 0.25, 0.0225, 0.63])
# theta2_slider = Slider(ax=theta2_bar_location,label="θ2",valmin=0,valmax=360,valinit=init_theta2,orientation="vertical")
# theta3_bar_location = fig.add_axes([0.15, 0.25, 0.0225, 0.63])
# theta3_slider = Slider(ax=theta3_bar_location,label="θ3", valmin=0,valmax=360,valinit=init_theta3,orientation="vertical")

# # The function to be called anytime a slider's value changes
# def update(val):
#     move_robot(math.radians(theta1_slider.val),math.radians(theta2_slider.val),math.radians(theta3_slider.val))
#     x_goal = artibot.get_robot_xyzdata()[0][3]
#     y_goal = artibot.get_robot_xyzdata()[1][3]
#     z_goal = artibot.get_robot_xyzdata()[2][3]
#     print(x_goal,y_goal,z_goal)
#     theta1,theta2,theta3 = artibot.inverse_kinematics(x_goal,y_goal,z_goal)
#     print(math.degrees(theta1),math.degrees(theta2),math.degrees(theta3))



# theta1_slider.on_changed(update)
# theta2_slider.on_changed(update)
# theta3_slider.on_changed(update)



x_center =  1.2
y_center =  0.5
z_center =  0.0
r_circle =  0.5
N        =  100

circle_waypoints = generate_circle_waypoints(x_center, y_center, z_center, r_circle, N)
# Extract X, Y, Z coordinates from waypoints
x_coords, y_coords, z_coords = zip(*circle_waypoints)


# Plot waypoints as dots
ax.scatter(x_coords, y_coords, z_coords, color='r', marker='o', label='Waypoints')

for waypoint in circle_waypoints:
    theta1, theta2, theta3 = artibot.inverse_kinematics(waypoint[0],waypoint[1],waypoint[2])
    print(theta1,theta2,theta3)
    move_robot(theta1, theta2, theta3)

plt.show()