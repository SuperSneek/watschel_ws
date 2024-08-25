import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Header
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider, TextBox
import numpy as np
from numpy import pi, sin, cos, tan, arctan, arccos
import math
from typing import List

# import os
# import pickle

"""
source /opt/ros/foxy/setup.bash
ros2 topic echo /hadabot/log/info
ros2 topic pub -t 1 /hadabot/servos std_msgs/msg/Int32MultiArray
"data: [1000, 2000, 2000, 1000]"

colcon build
source install/setup.bash
ros2 run m01_pkg_py m01_node_py
export DISPLAY=172.20.10.3:0
export DISPLAY=100.124.167.36:0
"""

# [0.0,   -0.5, [0, 2.5], 8],     # 3 go to center


ik_valss = [[0, -0.5, [0, 0.5], 8.3],  # 0 stand up
            [0.3, -0.5, [0, 0.5], 8.3],  # 1 lean right
            [0.3, 0, [0, 2.0], 7.8],  # 3 slide back
            [-0.3, 1.5, [0, 0.5], 8.3],  # 5 lean left
            [-0.3, -2.0, [0, -2.], 7.8],  # 7 slide back
            [0.3, 0, [0, -2.], 7.8],  # 9 lean right
            [0.3, 0, [0, 0.0], 7.8],  # 11 slide back
            [0, -0.5, [0, 0.5], 8.3]]  # 12 stand up

times_adaption = [1, 1, 2, 1, 2, 1, 2, 1]


# [0.0,   -0.5, [2.5, 0], 8],     # 6 go to center


def map_to_servo(x, in_min, in_max, out_min, out_max):
    mapped_val = ((x - in_min) * (out_max - out_min) /
                  (in_max - in_min) + out_min)
    return min(out_max, max(out_min, mapped_val))


class MyROSNode(Node):
    def __init__(self):
        super().__init__('intro_ros2_node_py')
        self.seq = 0
        self.servo_pub_ = self.create_publisher(
            Int32MultiArray, '/hadabot/servos', 10)
        self.trajectory_pub = self.create_publisher(
            Float32MultiArray, '/hadabot/servos_traj', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

    def publish_deltas(self, deltas):
        d = [float(round(map_to_servo(deltas[0][0], -pi / 4, 0, 1050, 1650), 0)),
             float(round(map_to_servo(deltas[1][0], 0, -pi / 4, 1450, 2000), 0)),
             float(round(map_to_servo(deltas[0][1], 0, pi / 4, 1450, 2000), 0)),
             float(round(map_to_servo(deltas[1][1], pi / 4, 0, 1050, 1450), 0))]
        d.append(.4)
        print("dat   ", d)
        msg = Float32MultiArray()
        msg.data = d
        self.trajectory_pub.publish(msg)

    def publish_joint_state(self, state):
        msg = JointState()


        msg.header = Header()
        msg.header.stamp = Time()
        msg.header.stamp.sec = self.get_clock().now().seconds_nanoseconds()[0]
        msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()[1]
        msg.header.frame_id = "watschel"

        msg.name = ["hip_right", "hip_left", "leg_left", "leg_right"]

        state = list(map(lambda x:float(x), state))
        msg.position = state
        print(state)
        msg.velocity = [float(0),float(0),float(0),float(0)]
        msg.effort = [float(0),float(0),float(0),float(0)]


        self.joint_state_pub.publish(msg)

    def publish_trajectories(self, delts, ts):
        traj_mess = []
        for i_d, delt in enumerate(delts):
            dat = [
                float(map_to_servo(delt[0][0], -pi / 4, 0, 1050, 1650)),
                float(map_to_servo(delt[1][0], 0, -pi / 4, 1450, 2000)),
                float(map_to_servo(delt[0][1], 0, pi / 4, 1450, 2000)),
                float(map_to_servo(delt[1][1], pi / 4, 0, 1050, 1450))]
            dat.append(ts[i_d])
            traj_mess.extend(dat)
        print("traj_mess", traj_mess)
        msg = Float32MultiArray()
        msg.data = traj_mess
        self.trajectory_pub.publish(msg)


leg_length = [[5, 5], [5, 5]]  # cm

l_3 = 7  # cm


class watschelApprox():
    def __init__(self, d, ax1, ax2, ax3):
        l_w = []
        l_w.append(leg_length[0][0] * cos(d[1][0]) +
                   leg_length[0][1] * cos(d[1][1]))
        l_w.append(leg_length[1][0] * cos(d[0][0]) +
                   leg_length[1][1] * cos(d[0][1]))
        self.alpha = arctan((l_w[1] - l_w[0]) / l_3)
        print("legs", l_w)
        print("alpha", self.alpha)

        self.z = ((leg_length[0][0] * cos(d[1][0]) +
                   leg_length[0][1] * cos(d[1][1])) * cos(self.alpha) +
                  sin(self.alpha) * l_3 / 2)

        self.l_2_y = l_w[1] * sin(self.alpha)
        self.l_2_z = l_w[1] * cos(self.alpha)
        self.l_3_y = self.l_2_y + l_3 * cos(self.alpha)
        self.l_3_z = self.l_2_z - l_3 * sin(self.alpha)
        self.l_1_y = self.l_3_y - l_w[0] * sin(self.alpha)
        self.l_1_z = self.l_3_z - l_w[0] * cos(self.alpha)

        self.l_0, = ax2.plot([self.l_3_y, self.l_1_y],
                             [self.l_3_z, self.l_1_z], label="l_3", c="r")
        self.l_1, = ax2.plot([self.l_2_y, self.l_3_y],
                             [self.l_2_z, self.l_3_z], label="l_1", c="g")
        self.l_2, = ax2.plot([0, self.l_2_y], [0, self.l_2_z],
                             label="l_2", c="b")

        self.z_plot, = ax2.plot([(self.l_3_y + self.l_2_y) / 2] * 2,
                                [0, self.z], label="z")

        self.l_1_2_x = sin(d[0][1]) * leg_length[0][1]
        self.l_1_2_z = cos(d[0][1]) * leg_length[0][1]
        self.l_1_1_x = self.l_1_2_x + sin(d[0][0]) * leg_length[0][0]
        self.l_1_1_z = self.l_1_2_z + cos(d[0][0]) * leg_length[0][0]
        self.l_2_2_x = sin(d[1][1]) * leg_length[1][1]
        self.l_2_2_z = cos(d[1][1]) * leg_length[1][1]
        self.l_2_1_x = self.l_1_2_x + sin(d[1][0]) * leg_length[1][0]
        self.l_2_1_z = self.l_1_2_z + cos(d[1][0]) * leg_length[1][0]

        self.current_moving_foot = 0
        self.update_step_diff()

        self.l_1_1, = ax3.plot([self.step_diff[0] + self.l_1_2_x,
                                self.step_diff[0] + self.l_1_1_x],
                               [self.l_1_2_z, self.l_1_1_z], label="l_1_1",
                               c="r")
        self.l_1_2, = ax3.plot([self.step_diff[0],
                                self.step_diff[0] + self.l_1_2_x],
                               [0, self.l_1_2_z], label="l_1_2", c="r")
        self.l_2_1, = ax1.plot([self.step_diff[1] + self.l_2_2_x,
                                self.step_diff[1] + self.l_2_1_x],
                               [self.l_2_2_z, self.l_2_1_z], label="l_2_1",
                               c="b")
        self.l_2_2, = ax1.plot([self.step_diff[1] + self.step_diff[0],
                                self.step_diff[1] + self.l_2_2_x],
                               [0, self.l_2_2_z], label="l_2_2", c="b")

        ax1.set_title("Left Leg")
        ax3.set_title("Right Leg")

        self.x_diff_old = [self.l_1_1_x, self.l_2_1_x]

        self.update_ik_vals = True
        self.update_fk_vals = True
        self.keypoint_index = 0
        self.t_keypoints_first = .30  # second
        self.t_keypoints_second = .10  # second

    @property
    def ik_vals(self):
        return [self.alpha, self.x_com, self.step_diff, self.z]

    def set_ik_vals(self, ik_vals__):
        self.alpha = ik_vals__[0]
        self.x_com = ik_vals__[1]
        self.step_diff = ik_vals__[2]
        if ik_vals__[2][0] == 0:
            self.current_moving_foot = 1
        else:
            self.current_moving_foot = 0
        self.z = ik_vals__[3]

    def update_step_diff(self):
        if self.current_moving_foot == 0:
            self.x_com = self.l_2_1_x
            self.step_diff = [self.x_com - self.l_1_1_x, 0]
        else:
            self.x_com = self.l_1_1_x
            self.step_diff = [0, self.x_com - self.l_2_1_x]


def get_deg(x_vals, y_vals):
    vector1 = [x_vals[1] - x_vals[0], y_vals[1] - y_vals[0]]
    vector2 = [x_vals[2] - x_vals[1], y_vals[2] - y_vals[1]]
    dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]

    magnitude1 = math.sqrt(vector1[0] ** 2 + vector1[1] ** 2)
    magnitude2 = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2)

    angle_rad = arccos(dot_product / (magnitude1 * magnitude2))
    angle_deg = math.degrees(angle_rad)

    return angle_deg


def verify_90_bend(x_vals, y_vals):
    angle_deg = get_deg(x_vals, y_vals)
    if angle_deg % 90 == 0:  # check for 90° bedn
        return True
    return False


def update_front(l_uf: List, alpha, d: List, watschel: watschelApprox):
    z = ((leg_length[0][0] * cos(d[1][0]) +
          leg_length[0][1] * cos(d[1][1])) * cos(alpha) +
         sin(alpha) * l_3 / 2)
    watschel.z = z
    watschel.z_plot.set_ydata([0, z])

    l_2_y = l_uf[1] * sin(alpha)
    l_2_z = l_uf[1] * cos(alpha)
    watschel.l_2_y = l_2_y
    watschel.l_2_z = l_2_z
    watschel.l_2.set_xdata([0, l_2_y])
    watschel.l_2.set_ydata([0, l_2_z])

    l_3_y = l_2_y + l_3 * cos(alpha)
    l_3_z = l_2_z - l_3 * sin(alpha)
    watschel.l_3_y = l_3_y
    watschel.l_3_z = l_3_z
    watschel.l_1.set_xdata([l_2_y, l_3_y])
    watschel.l_1.set_ydata([l_2_z, l_3_z])
    watschel.z_plot.set_xdata([(l_3_y + l_2_y) / 2] * 2)

    l_1_y = l_3_y - l_uf[0] * sin(alpha)
    l_1_z = l_3_z - l_uf[0] * cos(alpha)
    watschel.l_1_y = l_1_y
    watschel.l_1_z = l_1_z
    watschel.l_0.set_xdata([l_3_y, l_1_y])
    watschel.l_0.set_ydata([l_3_z, l_1_z])
    assert abs(l_1_z) < 2 * np.finfo(np.float32).eps, ("should be 0 but" +
                                                       f"is f{l_1_z}")


def update_side(i, d, watschel):
    l_i_2_x = sin(d[1]) * leg_length[i][1]
    l_i_2_z = cos(d[1]) * leg_length[i][1]
    l_i_1_x = l_i_2_x + sin(d[0]) * leg_length[i][0]
    l_i_1_z = l_i_2_z + cos(d[0]) * leg_length[i][0]

    if i == 0:
        watschel.l_1_2_x = l_i_2_x
        watschel.l_1_2_z = l_i_2_z
        watschel.l_1_1_x = l_i_1_x
        watschel.l_1_1_z = l_i_1_z
    else:
        watschel.l_2_2_x = l_i_2_x
        watschel.l_2_2_z = l_i_2_z
        watschel.l_2_1_x = l_i_1_x
        watschel.l_2_1_z = l_i_1_z
    watschel.update_step_diff()
    watschel.l_1_1.set_xdata([watschel.step_diff[0] + watschel.l_1_2_x,
                              watschel.step_diff[0] + watschel.l_1_1_x])
    watschel.l_1_1.set_ydata([watschel.l_1_2_z, watschel.l_1_1_z])
    watschel.l_1_2.set_xdata([watschel.step_diff[0],
                              watschel.step_diff[0] + watschel.l_1_2_x])
    watschel.l_1_2.set_ydata([0, watschel.l_1_2_z])
    watschel.l_2_1.set_xdata([watschel.step_diff[1] + watschel.l_2_2_x,
                              watschel.step_diff[1] + watschel.l_2_1_x])
    watschel.l_2_1.set_ydata([watschel.l_2_2_z, watschel.l_2_1_z])
    watschel.l_2_2.set_xdata([watschel.step_diff[1],
                              watschel.step_diff[1] + watschel.l_2_2_x])
    watschel.l_2_2.set_ydata([0, watschel.l_2_2_z])


def fk_plot(d: List, ax1, ax2, ax3, watschel):
    l_fk = []
    l_fk.append(leg_length[0][0] * cos(d[1][0]) +
                leg_length[0][1] * cos(d[1][1]))
    l_fk.append(leg_length[1][0] * cos(d[0][0]) +
                leg_length[1][1] * cos(d[0][1]))
    alpha = arctan((l_fk[1] - l_fk[0]) / l_3)
    watschel.alpha = alpha
    update_side(1, d[0], watschel)
    update_front(l_fk, alpha, d, watschel)
    update_side(0, d[1], watschel)

    ax1.legend()
    ax2.legend()
    ax3.legend()
    ax2.set_box_aspect(1)
    ax1.set_xlim(ax2.get_xbound())
    ax1.set_box_aspect(1)
    ax3.set_xlim(ax2.get_xbound())
    ax3.set_box_aspect(1)


def ik(watschel):
    print(watschel.current_moving_foot,
          watschel.step_diff)
    if watschel.current_moving_foot == 0:
        c0 = watschel.step_diff[0]
        c1 = watschel.x_com + watschel.step_diff[1]
    else:
        c0 = watschel.x_com + watschel.step_diff[0]
        c1 = watschel.step_diff[1]

    d = (watschel.z - sin(watschel.alpha) * l_3 / 2) / cos(watschel.alpha)
    print(watschel.ik_vals)

    def find_in_bounds(y_1, y_2, bounds, i, j):
        x = np.linspace(bounds[0], bounds[1], 10 ** 4)
        sqrt_diff = (leg_length[i][j] ** 2 -
                     (y_2 - leg_length[i][1 - j] * cos(x)) ** 2)
        sqrt_diff_true = np.array([sqrt_diff_ for sqrt_diff_ in sqrt_diff
                                   if sqrt_diff_ >= 0])
        assert len(sqrt_diff_true) > 0, ("No solution was found. " +
                                         "Try less critical values")
        x_true = np.array([x_ for i_x, x_ in enumerate(x)
                           if sqrt_diff[i_x] >= 0])
        errors = abs(y_1 - np.sqrt(sqrt_diff_true) -
                     leg_length[i][1 - j] * sin(x_true))
        min_error_index = np.argmin(errors)
        return x_true[min_error_index]

    d_00 = find_in_bounds(c0, d, [-pi / 4, 0], 0, 1)
    d_01 = arccos((d - leg_length[0][0] * cos(d_00)) / leg_length[0][1])
    d_01 = min(pi / 4, max(0, d_01))

    l_1 = leg_length[0][0] * cos(d_00) + leg_length[0][1] * cos(d_01)
    e = tan(watschel.alpha) * l_3 + l_1
    d_11 = -find_in_bounds(c1, e, [-pi / 4, 0], 1, 0)
    d_10 = -abs(arccos((e - leg_length[1][1] * cos(d_11)) / leg_length[1][0]))
    d_10 = min(0, max(-pi / 4, d_10))

    return [[d_10, d_11], [d_00, d_01]]


def init_plot(ros_node):
    fig, (ax1, ax2, ax3) = plt.subplots(ncols=3, sharey=True)

    deltas = [[-0 / 360 * 2 * pi, 0 / 360 * 2 * pi], [-0 / 360 * 2 * pi, 0 / 360 * 2 * pi]]
    watschel = watschelApprox(deltas, ax1, ax2, ax3)

    axdelta11 = fig.add_axes([0.1, 0.1, 0.3, 0.03])
    delta11_slider = Slider(ax=axdelta11, label='d_1_1', valmin=-pi / 4,
                            valmax=0, valinit=deltas[0][0])
    axdelta12 = fig.add_axes([0.1, 0.05, 0.3, 0.03])
    delta12_slider = Slider(ax=axdelta12, label='d_1_2', valmin=0,
                            valmax=pi / 4, valinit=deltas[0][1])
    axdelta21 = fig.add_axes([0.6, 0.1, 0.3, 0.03])
    delta21_slider = Slider(ax=axdelta21, label='d_2_1', valmin=-pi / 4,
                            valmax=0, valinit=deltas[1][0])
    axdelta22 = fig.add_axes([0.6, 0.05, 0.3, 0.03])
    delta22_slider = Slider(ax=axdelta22, label='d_2_2', valmin=0,
                            valmax=pi / 4, valinit=deltas[1][1])

    axalpha = fig.add_axes([0.1, 0.9, 0.3, 0.03])
    alpha_slider = Slider(ax=axalpha, label='alpha', valmin=-pi / 4,
                          valmax=pi / 4, valinit=watschel.alpha)
    ax_x_com = fig.add_axes([0.1, 0.95, 0.3, 0.03])
    x_com_slider = Slider(ax=ax_x_com, label='x_com', valmin=-3,
                          valmax=3, valinit=watschel.x_com)
    axstep_diff_1 = fig.add_axes([0.6, 0.9, 0.3, 0.03])
    step_diff_slider = Slider(ax=axstep_diff_1, label=('step_diff ' +
                                                       f'{watschel.current_moving_foot}'),
                              valmin=-3, valmax=3,
                              valinit=watschel.step_diff[
                                  watschel.current_moving_foot])
    axz = fig.add_axes([0.6, 0.95, 0.3, 0.03])
    z_slider = Slider(ax=axz, label='z', valmin=0, valmax=10,
                      valinit=watschel.z)

    def read_ik_vals(except_ind, except_val):
        if except_ind != 0:
            watschel.alpha = alpha_slider.val
        else:
            watschel.alpha = except_val
        if except_ind != 1:
            watschel.x_com = x_com_slider.val
        else:
            watschel.x_com = except_val
        if except_ind != 2:
            wcmf = watschel.current_moving_foot
            watschel.step_diff[wcmf] = step_diff_slider.val
            watschel.step_diff[wcmf - 1] = 0
        else:
            watschel.step_diff[watschel.current_moving_foot] = except_val
            watschel.step_diff[watschel.current_moving_foot - 1] = 0
        if except_ind != 3:
            watschel.z = z_slider.val
        else:
            watschel.z = except_val

    def update_11(val):
        if watschel.update_fk_vals:
            deltas[0][0] = val
            publish_print_data(deltas)
            watschel.update_ik_vals = False
            set_ik_sliders(watschel.ik_vals)
            watschel.update_ik_vals = True

    def update_12(val):
        if watschel.update_fk_vals:
            deltas[0][1] = val
            publish_print_data(deltas)
            watschel.update_ik_vals = False
            set_ik_sliders(watschel.ik_vals)
            watschel.update_ik_vals = True

    def update_21(val):
        if watschel.update_fk_vals:
            deltas[1][0] = val
            publish_print_data(deltas)
            watschel.update_ik_vals = False
            set_ik_sliders(watschel.ik_vals)
            watschel.update_ik_vals = True

    def update_22(val):
        if watschel.update_fk_vals:
            deltas[1][1] = val
            publish_print_data(deltas)
            watschel.update_ik_vals = False
            set_ik_sliders(watschel.ik_vals)
            watschel.update_ik_vals = True

    def update_alpha(val):
        if watschel.update_ik_vals:
            read_ik_vals(0, val)
            deltas = ik(watschel)
            watschel.update_fk_vals = False
            set_delta_sliders(deltas)
            watschel.update_fk_vals = True
            publish_print_data(deltas)

    def update_x_com(val):
        if watschel.update_ik_vals:
            read_ik_vals(1, val)
            deltas = ik(watschel)
            watschel.update_fk_vals = False
            set_delta_sliders(deltas)
            watschel.update_fk_vals = True
            publish_print_data(deltas)

    def update_step_diff(val):
        if watschel.update_ik_vals:
            read_ik_vals(2, val)
            deltas = ik(watschel)
            watschel.update_fk_vals = False
            set_delta_sliders(deltas)
            watschel.update_fk_vals = True
            publish_print_data(deltas)

    def update_z(val):
        if watschel.update_ik_vals:
            read_ik_vals(3, val)
            deltas = ik(watschel)
            watschel.update_fk_vals = False
            set_delta_sliders(deltas)
            watschel.update_fk_vals = True
            publish_print_data(deltas)

    def update_duration_between_keypoints_first(val):
        print("first", val)
        watschel.t_keypoints_first = float(val)  # second

    def update_duration_between_keypoints_second(val):
        print("second", val)
        watschel.t_keypoints_second = float(val)  # second

    delta11_slider.on_changed(update_11)
    delta12_slider.on_changed(update_12)
    delta21_slider.on_changed(update_21)
    delta22_slider.on_changed(update_22)

    alpha_slider.on_changed(update_alpha)
    x_com_slider.on_changed(update_x_com)
    step_diff_slider.on_changed(update_step_diff)
    z_slider.on_changed(update_z)
    lakj = ["r", "l"]
    change_step_ax = fig.add_axes([.9, .85, .03, .03])
    change_step_button = Button(change_step_ax,
                                lakj[watschel.current_moving_foot],
                                hovercolor='0.975')

    def change_step(event):
        watschel.current_moving_foot = 1 - watschel.current_moving_foot
        print(watschel.current_moving_foot)
        change_step_button.label.set_text(str(watschel.current_moving_foot))
        step_diff_slider.label.set_text('step_diff ' +
                                        lakj[watschel.current_moving_foot])
        fig.canvas.draw_idle()

    change_step_button.on_clicked(change_step)

    keypoint_ax = fig.add_axes([0.1, 0.8, 0.04, 0.04])
    TextBox(keypoint_ax, "Keypoint")

    prev_ax = fig.add_axes([0.2, 0.8, 0.04, 0.04])
    prev_button = Button(prev_ax, "<", hovercolor='.975')

    index_ax = fig.add_axes([0.25, 0.8, 0.04, 0.04])
    ind_tex = TextBox(index_ax, str(watschel.keypoint_index))
    second_speed_ax = fig.add_axes([0.25, 0.75, 0.04, 0.04])
    second_speed = TextBox(second_speed_ax, "")

    next_ax = fig.add_axes([0.3, 0.8, 0.04, 0.04])
    next_button = Button(next_ax, ">", hovercolor='.975')

    def prev(event):
        # ik_valss[watschel.keypoint_index] = get_ik_vals(deltas, watschel)
        # with open("ik_valss", "wb") as filename:
        #     pickle.dump(ik_valss, filename)
        watschel.keypoint_index = max(0, watschel.keypoint_index - 1)
        ind_tex.label.set_text(str(watschel.keypoint_index))
        watschel.update_ik_vals = False
        watschel.update_fk_vals = False
        # set_ik_sliders(ik_valss[watschel.keypoint_index])
        watschel.set_ik_vals(ik_valss[watschel.keypoint_index])
        deltas_ = ik(watschel)
        set_delta_sliders(deltas_)
        watschel.update_ik_vals = True
        watschel.update_fk_vals = True
        publish_print_data(deltas_)

    def next(event):
        # ik_valss[watschel.keypoint_index] = get_ik_vals(deltas, watschel)
        # with open("ik_valss", "wb") as filename:
        #     pickle.dump(ik_valss, filename)
        watschel.keypoint_index = min(len(ik_valss) - 1,
                                      watschel.keypoint_index + 1)
        ind_tex.label.set_text(str(watschel.keypoint_index))
        watschel.update_ik_vals = False
        watschel.update_fk_vals = False
        # set_ik_sliders(ik_valss[watschel.keypoint_index])
        watschel.set_ik_vals(ik_valss[watschel.keypoint_index])
        deltas_ = ik(watschel)
        set_delta_sliders(deltas_)
        watschel.update_ik_vals = True
        watschel.update_fk_vals = True
        publish_print_data(deltas_)

    prev_button.on_clicked(prev)
    next_button.on_clicked(next)

    ind_tex.on_submit(update_duration_between_keypoints_first)
    second_speed.on_submit(update_duration_between_keypoints_second)

    # add_ax = fig.add_axes([0.4, 0.8, 0.04, 0.04])
    # add_button = Button(add_ax, 'Add', hovercolor='0.975')

    # del_ax = fig.add_axes([0.5, 0.8, 0.04, 0.04])
    # del_button = Button(del_ax, 'Delete', hovercolor='0.975')

    # def add_(event):
    #     ik_v = get_ik_vals(deltas, watschel)
    #     ik_valss.insert(watschel.keypoint_index + 1, ik_v)
    #     next(None)

    # def del_(event):
    #     del ik_valss[watschel.keypoint_index]
    #     prev(None)

    # add_button.on_clicked(add_)
    # del_button.on_clicked(del_)

    playax = fig.add_axes([0.8, 0.8, 0.1, 0.04])
    play_button = Button(playax, 'Play', hovercolor='0.975')

    def get_time_for_ind(ind_):
        if ind_ == 1:
            return watschel.t_keypoints_first
        else:
            return watschel.t_keypoints_second

    def play(event):
        publish_traj(ik_valss,
                     [get_time_for_ind(ind__) for ind__ in times_adaption])
        start_time = time.time()
        curr_time_delta = 0
        i__ = 0
        # print("start_time", start_time)
        while (int(np.ceil(curr_time_delta /
                           get_time_for_ind(times_adaption[i__]))) < len(ik_valss)):
            # print("t:", curr_time_delta / duration_between_keypoints)
            i__ = int(np.floor(curr_time_delta /
                               get_time_for_ind(times_adaption[i__])))
            curr_vals = (np.array(ik_valss[i__][0:2] + ik_valss[i__][2] +
                                  [ik_valss[i__][3]]))
            next_vals = (np.array(ik_valss[i__ + 1][0:2] + ik_valss[i__ + 1][2]
                                  + [ik_valss[i__ + 1][3]]))
            t = (curr_time_delta - get_time_for_ind(times_adaption[i__]) *
                 np.floor(curr_time_delta /
                          get_time_for_ind(times_adaption[i__])))
            liv_t = (1 - t) * curr_vals + t * next_vals
            lin_interpols_vals = (liv_t[0:2].tolist() + [liv_t[2:4].tolist()]
                                  + [liv_t[4].tolist()])

            
            watschel.set_ik_vals(lin_interpols_vals)
            lin_interpols_deltas = ik(watschel)
            print("SUS")

            print_data(lin_interpols_deltas)
            states = [lin_interpols_deltas[0][0],lin_interpols_deltas[1][0],lin_interpols_deltas[0][1],lin_interpols_deltas[1][1]]
            ros_node.publish_joint_state(states)
            plt.pause(.001)
            curr_time_delta = time.time() - start_time

    play_button.on_clicked(play)

    def set_delta_sliders(delt):
        delta11_slider.set_val(delt[0][0])
        delta12_slider.set_val(delt[0][1])
        delta21_slider.set_val(delt[1][0])
        delta22_slider.set_val(delt[1][1])

    def set_ik_sliders(ik_val):
        alpha_slider.set_val(ik_val[0])
        x_com_slider.set_val(ik_val[1])
        step_diff_slider.set_val(ik_val[2][watschel.current_moving_foot])
        z_slider.set_val(ik_val[3])

    def print_data(delt_):
        fk_plot(delt_, ax1, ax2, ax3, watschel)
        watschel.update_ik_vals = False
        set_ik_sliders(watschel.ik_vals)
        watschel.update_ik_vals = True
        fig.canvas.draw_idle()

    def publish_print_data(delt):
        print("deltas", delt)
        ros_node.publish_deltas(delt)
        print_data(delt)

    def publish_traj(ik_vals_, ts):
        assert len(ik_vals_) == len(ts), ("The length of ik_v and the times" +
                                          f"does not match: {ik_vals_}, {ts}")
        delts = []
        for ik_v in ik_vals_:
            watschel.set_ik_vals(ik_v)
            delts.append(ik(watschel))
        print(delts)
        ros_node.publish_trajectories(delts, ts)

    # deltas = [[-45/360*2*pi, 30/360*2*pi], [-30/360*2*pi, 30/360*2*pi]]
    watschel.set_ik_vals(ik_valss[0])

    try:
        deltas = ik(watschel)
        print("deltas", deltas)
    except Exception:
        pass
    publish_print_data(deltas)

    plt.show()


def main2(args=None):
    rclpy.init(args=args)
    my_ros_node = MyROSNode()
    # rclpy.spin(my_ros_node)
    my_ros_node.publish_servo_degees()
    my_ros_node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    my_ros_node = MyROSNode()
    init_plot(my_ros_node)
    my_ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
