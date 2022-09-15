from __future__ import division
import os
import rospkg

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget
from rqt_gui_py.plugin import Plugin
from ctr_hardware_interface.CTR_Python.Tube import Tube
import numpy as np


# TODO: Big stop sign to cut the controllers incase something goes wrong
# TODO: Add tube lengths here from ros parameter server


class CTRPlugin(Plugin):
    def __init__(self, context):
        super(CTRPlugin, self).__init__(context)
        self.setObjectName('CTRPlugin')

        self.tr_scale = 10.0
        self.rot_scale = 1.0

        # Load ctr robot parameters
        # Get parameters from ROS params
        tube_0 = Tube(**rospy.get_param("/tube_0"))
        tube_1 = Tube(**rospy.get_param("/tube_1"))
        tube_2 = Tube(**rospy.get_param("/tube_2"))
        self.ctr_parameters = [tube_0, tube_1, tube_2]

        beta_1_range = [-tube_0.L * 1000.0, -0.001 * 1000.0]
        beta_1_init_value = -11.0 * self.tr_scale
        beta_2_range = [-tube_1.L * 1000.0, -0.001 * 1000.0]
        beta_2_init_value = -7.0 * self.tr_scale
        beta_3_range = [-tube_2.L * 1000.0, -0.001 * 1000.0]
        beta_3_init_value = -3.0 * self.tr_scale
        alpha_1_range = [-180.0, 180.0]
        alpha_1_init_value = 0.0 * self.rot_scale
        alpha_2_range = [-180.0, 180.0]
        alpha_2_init_value = 0.0 * self.rot_scale
        alpha_3_range = [-180.0, 180.0]
        alpha_3_init_value = 0.0 * self.rot_scale

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_ctr'), 'resource', 'CTRPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('CTRPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Top topic change and stop
        topic = self._widget.topic_line_edit.text()
        self._publisher = rospy.Publisher(topic, JointState, queue_size=10)

        # Subscribe to actual joint values to update the values shown
        self._subscriber = rospy.Subscriber("/joint_state", JointState, self._jointstate_callback)

        # rospy.wait_for_service("/read_joint_states")
        # read_joints = rospy.ServiceProxy("/read_joint_states", Trigger)
        # print("Service found, reading for initialization.")
        # try:
        #    resp = read_joints()
        # except rospy.ServiceException as exc:
        #    print("Service did not process request" + str(exc))

        self._widget.topic_line_edit.textChanged.connect(
            self._on_topic_changed)
        self._widget.stop_push_button.pressed.connect(self._on_stop_pressed)

        # Update min and max alphas and betas for label and sliders
        # beta1
        self._initialize_slider(self._widget.beta_1_slider, self._widget.min_beta_1_label,
                                self._widget.max_beta_1_label, self._widget.current_beta_1_label, beta_1_range, 1.0,
                                beta_1_init_value, 'mm', self.tr_scale)
        # beta2
        self._initialize_slider(self._widget.beta_2_slider, self._widget.min_beta_2_label,
                                self._widget.max_beta_2_label, self._widget.current_beta_2_label, beta_2_range, 1.0,
                                beta_2_init_value, 'mm', self.tr_scale)
        # beta3
        self._initialize_slider(self._widget.beta_3_slider, self._widget.min_beta_3_label,
                                self._widget.max_beta_3_label, self._widget.current_beta_3_label, beta_3_range, 1.0,
                                beta_3_init_value, 'mm', self.tr_scale)
        # alpha1
        self._initialize_slider(self._widget.alpha_1_slider, self._widget.min_alpha_1, self._widget.max_alpha_1,
                                self._widget.current_alpha_1_label, alpha_1_range, 1.0, alpha_1_init_value, 'deg',
                                self.rot_scale)
        # alpha2
        self._initialize_slider(self._widget.alpha_2_slider, self._widget.min_alpha_2, self._widget.max_alpha_2,
                                self._widget.current_alpha_2_label, alpha_2_range, 1.0, alpha_2_init_value, 'deg',
                                self.rot_scale)
        # alpha3
        self._initialize_slider(self._widget.alpha_3_slider, self._widget.min_alpha_3, self._widget.max_alpha_3,
                                self._widget.current_alpha_3_label, alpha_3_range, 1.0, alpha_3_init_value, 'deg',
                                self.rot_scale)

        # When moving the sliders for beta_1 and alpha_1 update the value printed and sent a msg to publish
        self._widget.beta_1_slider.valueChanged.connect(
            self._on_beta_1_slider_changed)
        self._widget.alpha_1_slider.valueChanged.connect(
            self._on_alpha_1_slider_changed)
        # When moving the sliders for beta_2 and alpha_2 update the value printed and sent a msg to publish
        self._widget.beta_2_slider.valueChanged.connect(
            self._on_beta_2_slider_changed)
        self._widget.alpha_2_slider.valueChanged.connect(
            self._on_alpha_2_slider_changed)
        # When moving the sliders for beta_3 and alpha_3 update the value printed and sent a msg to publish
        self._widget.beta_3_slider.valueChanged.connect(
            self._on_beta_3_slider_changed)
        self._widget.alpha_3_slider.valueChanged.connect(
            self._on_alpha_3_slider_changed)

        # Beta_1 and alpha_1 push buttons to increase and decrease values
        self._widget.increase_beta_1_push_button.pressed.connect(
            self._on_increase_beta_1_pressed)
        self._widget.decrease_beta_1_push_button.pressed.connect(
            self._on_decrease_beta_1_pressed)
        self._widget.increase_alpha_1_push_button.pressed.connect(
            self._on_increase_alpha_1_pressed)
        self._widget.decrease_alpha_1_push_button.pressed.connect(
            self._on_decrease_alpha_1_pressed)
        # Beta_2 and alpha_2 push buttons to increase and decrease values
        self._widget.increase_beta_2_push_button.pressed.connect(
            self._on_increase_beta_2_pressed)
        self._widget.decrease_beta_2_push_button.pressed.connect(
            self._on_decrease_beta_2_pressed)
        self._widget.increase_alpha_2_push_button.pressed.connect(
            self._on_increase_alpha_2_pressed)
        self._widget.decrease_alpha_2_push_button.pressed.connect(
            self._on_decrease_alpha_2_pressed)
        # Beta_3 and alpha_3 push buttons to increase and decrease values
        self._widget.increase_beta_3_push_button.pressed.connect(
            self._on_increase_beta_3_pressed)
        self._widget.decrease_beta_3_push_button.pressed.connect(
            self._on_decrease_beta_3_pressed)
        self._widget.increase_alpha_3_push_button.pressed.connect(
            self._on_increase_alpha_3_pressed)
        self._widget.decrease_alpha_3_push_button.pressed.connect(
            self._on_decrease_alpha_3_pressed)

        # Joint space sampler
        self._widget.sample_joint_position_button.pressed.connect(self._on_sample_joint_position_button_pressed)
        # Home Position button
        self._widget.home_position_button.pressed.connect(self._on_home_position_button_pressed)

    @Slot(str)
    def _on_topic_changed(self, topic):
        topic = str(topic)
        self._unregister_publisher()
        if topic == '':
            return
        try:
            self._publisher = rospy.Publisher(topic, JointState, queue_size=10)
        except TypeError:
            print("Topic not found to publish.")

    def _on_stop_pressed(self):
        # Unregister topic
        self._unregister_publisher()

    # Conversion between normalized and un-normalized joints
    @staticmethod
    def B_U_to_B(B_U, L_1, L_2, L_3):
        B_U = np.append(B_U, 1)
        M_B = np.array([[-L_1, 0, 0],
                        [-L_1, L_1 - L_2, 0],
                        [-L_1, L_1 - L_2, L_2 - L_3]])
        normalized_B = np.block([[0.5 * M_B, 0.5 * np.matmul(M_B, np.ones((3, 1)))],
                                 [np.zeros((1, 3)), 1]])
        B = np.matmul(normalized_B, B_U)
        return B[:3]

    @staticmethod
    def alpha_U_to_alpha(alpha_U, alpha_max):
        return alpha_max * alpha_U

    def _on_sample_joint_position_button_pressed(self):
        # Length of tubes, considering a margin of 5mm for innermost tube. Ordered outermost to innermost
        L_star = np.array([47.5, 77.0 - 5.0, 136.5 - 15.0])
        alpha_max = np.rad2deg(np.pi / 3)
        # Sample a joint position
        alphas_U = np.random.uniform(low=-np.ones((1, 3)), high=np.ones((1, 3)))
        alphas = np.squeeze(self.alpha_U_to_alpha(alphas_U, alpha_max))
        # Sample betas
        B_U = np.random.uniform(low=-np.ones((1, 3)), high=np.ones((1, 3)))
        Betas = np.flip(self.B_U_to_B(B_U, L_star[0], L_star[1], L_star[2]))
        # Set the slider values
        self._set_slider_value(self._widget.beta_1_slider, self._widget.current_beta_1_label,
                               np.round(Betas[0] * self.tr_scale, 2), 'mm', self.tr_scale)
        self._set_slider_value(self._widget.beta_2_slider, self._widget.current_beta_2_label,
                               np.round(Betas[1] * self.tr_scale, 2), 'mm', self.tr_scale)
        self._set_slider_value(self._widget.beta_3_slider, self._widget.current_beta_3_label,
                               np.round(Betas[2] * self.tr_scale, 2), 'mm', self.tr_scale)
        self._set_slider_value(self._widget.alpha_1_slider, self._widget.current_alpha_1_label,
                               np.round(alphas[0] * self.rot_scale, 0), 'deg', self.rot_scale)
        self._set_slider_value(self._widget.alpha_2_slider, self._widget.current_alpha_2_label,
                               np.round(alphas[1] * self.rot_scale, 0), 'deg', self.rot_scale)
        self._set_slider_value(self._widget.alpha_3_slider, self._widget.current_alpha_3_label,
                               np.round(alphas[2] * self.rot_scale, 0), 'deg', self.rot_scale)
        self._on_parameter_changed()

    def _on_home_position_button_pressed(self):
        Betas = [-2.0, -2.0, -2.0]
        alphas = [0.0, 0.0, 0.0]
        # Set the slider values
        self._set_slider_value(self._widget.beta_1_slider, self._widget.current_beta_1_label,
                               np.round(Betas[0] * self.tr_scale, 2), 'mm', self.tr_scale)
        self._set_slider_value(self._widget.beta_2_slider, self._widget.current_beta_2_label,
                               np.round(Betas[1] * self.tr_scale, 2), 'mm', self.tr_scale)
        self._set_slider_value(self._widget.beta_3_slider, self._widget.current_beta_3_label,
                               np.round(Betas[2] * self.tr_scale, 2), 'mm', self.tr_scale)
        self._set_slider_value(self._widget.alpha_1_slider, self._widget.current_alpha_1_label,
                               np.round(alphas[0] * self.rot_scale, 0), 'deg', self.rot_scale)
        self._set_slider_value(self._widget.alpha_2_slider, self._widget.current_alpha_2_label,
                               np.round(alphas[1] * self.rot_scale, 0), 'deg', self.rot_scale)
        self._set_slider_value(self._widget.alpha_3_slider, self._widget.current_alpha_3_label,
                               np.round(alphas[2] * self.rot_scale, 0), 'deg', self.rot_scale)
        self._on_parameter_changed()

    def _on_beta_1_slider_changed(self):
        self._widget.current_beta_1_label.setText(
            '%0.2f mm' % (self._widget.beta_1_slider.value() / self.tr_scale))
        self._on_parameter_changed()

    def _on_beta_2_slider_changed(self):
        self._widget.current_beta_2_label.setText(
            '%0.2f mm' % (self._widget.beta_2_slider.value() / self.tr_scale))
        self._on_parameter_changed()

    def _on_beta_3_slider_changed(self):
        self._widget.current_beta_3_label.setText(
            '%0.2f mm' % (self._widget.beta_3_slider.value() / self.tr_scale))
        self._on_parameter_changed()

    def _on_alpha_1_slider_changed(self):
        self._widget.current_alpha_1_label.setText(
            '%0.2f deg' % (self._widget.alpha_1_slider.value() / self.rot_scale))
        self._on_parameter_changed()

    def _on_alpha_2_slider_changed(self):
        self._widget.current_alpha_2_label.setText(
            '%0.2f deg' % (self._widget.alpha_2_slider.value() / self.rot_scale))
        self._on_parameter_changed()

    def _on_alpha_3_slider_changed(self):
        self._widget.current_alpha_3_label.setText(
            '%0.2f deg' % (self._widget.alpha_3_slider.value() / self.rot_scale))
        self._on_parameter_changed()

    def _on_increase_beta_1_pressed(self):
        val = self._widget.beta_1_slider.value() + self._widget.beta_1_slider.singleStep()
        self._set_slider_value(self._widget.beta_1_slider, self._widget.current_beta_1_label, val, 'mm',
                               self.tr_scale)

    def _on_increase_beta_2_pressed(self):
        val = self._widget.beta_2_slider.value() + self._widget.beta_2_slider.singleStep()
        self._set_slider_value(self._widget.beta_2_slider, self._widget.current_beta_2_label, val, 'mm',
                               self.tr_scale)

    def _on_increase_beta_3_pressed(self):
        val = self._widget.beta_3_slider.value() + self._widget.beta_3_slider.singleStep()
        self._set_slider_value(self._widget.beta_3_slider, self._widget.current_beta_3_label, val, 'mm',
                               self.tr_scale)

    def _on_decrease_beta_1_pressed(self):
        val = self._widget.beta_1_slider.value() - self._widget.beta_1_slider.singleStep()
        self._set_slider_value(self._widget.beta_1_slider, self._widget.current_beta_1_label, val, 'mm',
                               self.tr_scale)

    def _on_decrease_beta_2_pressed(self):
        val = self._widget.beta_2_slider.value() - self._widget.beta_2_slider.singleStep()
        self._set_slider_value(self._widget.beta_2_slider, self._widget.current_beta_2_label, val, 'mm',
                               self.tr_scale)

    def _on_decrease_beta_3_pressed(self):
        val = self._widget.beta_3_slider.value() - self._widget.beta_3_slider.singleStep()
        self._set_slider_value(self._widget.beta_3_slider, self._widget.current_beta_3_label, val, 'mm',
                               self.tr_scale)

    def _on_increase_alpha_1_pressed(self):
        val = self._widget.alpha_1_slider.value() + self._widget.alpha_1_slider.singleStep()
        self._set_slider_value(self._widget.alpha_1_slider, self._widget.current_alpha_1_label, val, 'deg',
                               self.rot_scale)

    def _on_increase_alpha_2_pressed(self):
        val = self._widget.alpha_2_slider.value() + self._widget.alpha_2_slider.singleStep()
        self._set_slider_value(self._widget.alpha_2_slider, self._widget.current_alpha_2_label, val, 'deg',
                               self.rot_scale)

    def _on_increase_alpha_3_pressed(self):
        val = self._widget.alpha_3_slider.value() + self._widget.alpha_3_slider.singleStep()
        self._set_slider_value(self._widget.alpha_3_slider, self._widget.current_alpha_3_label, val, 'deg',
                               self.rot_scale)

    def _on_decrease_alpha_1_pressed(self):
        val = self._widget.alpha_1_slider.value() - self._widget.alpha_1_slider.singleStep()
        self._set_slider_value(self._widget.alpha_1_slider, self._widget.current_alpha_1_label, val, 'deg',
                               self.rot_scale)

    def _on_decrease_alpha_2_pressed(self):
        val = self._widget.alpha_2_slider.value() - self._widget.alpha_2_slider.singleStep()
        self._set_slider_value(self._widget.alpha_2_slider, self._widget.current_alpha_2_label, val, 'deg',
                               self.rot_scale)

    def _on_decrease_alpha_3_pressed(self):
        val = self._widget.alpha_3_slider.value() - self._widget.alpha_3_slider.singleStep()
        self._set_slider_value(self._widget.alpha_3_slider, self._widget.current_alpha_3_label, val, 'deg',
                               self.rot_scale)

    def _on_parameter_changed(self):
        self._send_jointstate(
            self._widget.beta_1_slider.value() / self.tr_scale,
            self._widget.beta_2_slider.value() / self.tr_scale,
            self._widget.beta_3_slider.value() / self.tr_scale,
            self._widget.alpha_1_slider.value() / self.rot_scale,
            self._widget.alpha_2_slider.value() / self.rot_scale,
            self._widget.alpha_3_slider.value() / self.rot_scale)

    def _send_jointstate(self, beta_0, beta_1, beta_2, alpha_0, alpha_1, alpha_2):
        if self._publisher is None:
            return
        # publish JointState as (beta_0,beta_1,beta_2,alpha_0,alpha_1,alpha_2),
        # reading from each slider the new linear or angular position and updating it to the correct tube parameter
        jointstate = JointState()
        jointstate.position = np.zeros(6)
        jointstate.position[0] = beta_0
        jointstate.position[1] = beta_1
        jointstate.position[2] = beta_2
        jointstate.position[3] = alpha_0
        jointstate.position[4] = alpha_1
        jointstate.position[5] = alpha_2

        self.zero_cmd_sent = False
        self._publisher.publish(jointstate)

    def _jointstate_callback(self, msg):
        # self._set_slider_value(self._widget.beta_1_slider, self._widget.current_beta_1_label,
        #                       msg.position[0] * self.tr_scale, 'mm',
        #                       self.tr_scale)
        # self._set_slider_value(self._widget.beta_2_slider, self._widget.current_beta_2_label,
        #                       msg.position[1] * self.tr_scale, 'mm',
        #                       self.tr_scale)
        # self._set_slider_value(self._widget.beta_3_slider, self._widget.current_beta_3_label,
        #                       msg.position[2] * self.tr_scale, 'mm',
        #                       self.tr_scale)
        self._set_slider_value(self._widget.alpha_1_slider, self._widget.current_alpha_1_label,
                               msg.position[3] * self.rot_scale, 'deg',
                               self.rot_scale)
        self._set_slider_value(self._widget.alpha_2_slider, self._widget.current_alpha_2_label,
                               msg.position[4] * self.rot_scale, 'deg',
                               self.rot_scale)
        self._set_slider_value(self._widget.alpha_3_slider, self._widget.current_alpha_3_label,
                               msg.position[5] * self.rot_scale, 'deg',
                               self.rot_scale)

    def _initialize_slider(self, slider, min_label, max_label, current_label, slider_range, single_step, value, unit,
                           factor):
        slider.setMinimum(slider_range[0] * factor)
        slider.setMaximum(slider_range[1] * factor)
        slider.setSingleStep(single_step)
        min_label.setText(str(slider_range[0]) + ' ' + unit)
        max_label.setText(str(slider_range[1]) + ' ' + unit)
        self._set_slider_value(slider, current_label, value, unit, factor)

    def _set_slider_value(self, slider, current_label, value, unit, factor):
        slider.setValue(value)
        current_label.setText(str(value / factor) + ' ' + unit)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._unregister_publisher()
