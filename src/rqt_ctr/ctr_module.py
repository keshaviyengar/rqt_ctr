from __future__ import division
import os
import rospkg

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget
from rqt_gui_py.plugin import Plugin

import numpy as np

# TODO: Big stop sign to cut the controllers incase something goes wrong


class CTRPlugin(Plugin):
    def __init__(self, context):
        super(CTRPlugin, self).__init__(context)
        self.setObjectName('CTRPlugin')

        self.tr_scale = 10
        self.rot_scale = 10

        beta_1_range = [-100, -6]
        beta_1_init_value = -6.0
        beta_2_range = [-90, -4]
        beta_2_init_value = -4.0
        beta_3_range = [-50, -2]
        beta_3_init_value = -2.0
        alpha_1_range = [-3.0, 3.0]
        alpha_1_init_value = 0
        alpha_2_range = [-3.0, 3.0]
        alpha_2_init_value = 0
        alpha_3_range = [-3.0, 3.0]
        alpha_3_init_value = 0

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

        self._widget.topic_line_edit.textChanged.connect(
            self._on_topic_changed)
        self._widget.stop_push_button.pressed.connect(self._on_stop_pressed)

        # Update min and max alphas and betas for label and sliders
        # beta1
        self._initialize_slider(self._widget.beta_1_slider, self._widget.min_beta_1_label,
                                self._widget.max_beta_1_label, self._widget.current_beta_1_label, beta_1_range, 1.0,
                                beta_1_init_value, 'mm')
        # beta2
        self._initialize_slider(self._widget.beta_2_slider, self._widget.min_beta_2_label,
                                self._widget.max_beta_2_label, self._widget.current_beta_2_label, beta_2_range, 1.0,
                                beta_2_init_value, 'mm')
        # beta3
        self._initialize_slider(self._widget.beta_3_slider, self._widget.min_beta_3_label,
                                self._widget.max_beta_3_label, self._widget.current_beta_3_label, beta_3_range, 1.0,
                                beta_3_init_value, 'mm')
        # alpha1
        self._initialize_slider(self._widget.alpha_1_slider, self._widget.min_alpha_1, self._widget.max_alpha_1,
                                self._widget.current_alpha_1_label, alpha_1_range, 1.0, alpha_1_init_value, 'rad')
        # alpha2
        self._initialize_slider(self._widget.alpha_2_slider, self._widget.min_alpha_2, self._widget.max_alpha_2,
                                self._widget.current_alpha_2_label, alpha_2_range, 1.0, alpha_2_init_value, 'rad')
        # alpha3
        self._initialize_slider(self._widget.alpha_3_slider, self._widget.min_alpha_3, self._widget.max_alpha_3,
                                self._widget.current_alpha_3_label, alpha_3_range, 1.0, alpha_3_init_value, 'rad')

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

    def _on_beta_1_slider_changed(self):
        self._widget.current_beta_1_label.setText(
            '%0.2f mm' % (self._widget.beta_1_slider.value()))
        self._on_parameter_changed()

    def _on_beta_2_slider_changed(self):
        self._widget.current_beta_2_label.setText(
            '%0.2f mm' % (self._widget.beta_2_slider.value()))
        self._on_parameter_changed()

    def _on_beta_3_slider_changed(self):
        self._widget.current_beta_3_label.setText(
            '%0.2f mm' % (self._widget.beta_3_slider.value()))
        self._on_parameter_changed()

    def _on_alpha_1_slider_changed(self):
        self._widget.current_alpha_1_label.setText(
            '%0.2f rad' % (self._widget.alpha_1_slider.value()))
        self._on_parameter_changed()

    def _on_alpha_2_slider_changed(self):
        self._widget.current_alpha_2_label.setText(
            '%0.2f rad' % (self._widget.alpha_2_slider.value()))
        self._on_parameter_changed()

    def _on_alpha_3_slider_changed(self):
        self._widget.current_alpha_3_label.setText(
            '%0.2f rad' % (self._widget.alpha_3_slider.value()))
        self._on_parameter_changed()

    def _on_increase_beta_1_pressed(self):
        self._widget.beta_1_slider.setValue(
            self._widget.beta_1_slider.value() + self._widget.beta_1_slider.pageStep())

    def _on_increase_beta_2_pressed(self):
        self._widget.beta_2_slider.setValue(
            self._widget.beta_2_slider.value() + self._widget.beta_2_slider.pageStep())

    def _on_increase_beta_3_pressed(self):
        self._widget.beta_3_slider.setValue(
            self._widget.beta_3_slider.value() + self._widget.beta_3_slider.pageStep())

    def _on_decrease_beta_1_pressed(self):
        self._widget.beta_1_slider.setValue(
            self._widget.beta_1_slider.value() - self._widget.beta_1_slider.pageStep())

    def _on_decrease_beta_2_pressed(self):
        self._widget.beta_2_slider.setValue(
            self._widget.beta_2_slider.value() - self._widget.beta_2_slider.pageStep())

    def _on_decrease_beta_3_pressed(self):
        self._widget.beta_3_slider.setValue(
            self._widget.beta_3_slider.value() - self._widget.beta_3_slider.pageStep())

    def _on_increase_alpha_1_pressed(self):
        self._widget.alpha_1_slider.setValue(
            self._widget.alpha_1_slider.value() + self._widget.alpha_1_slider.pageStep())

    def _on_increase_alpha_2_pressed(self):
        self._widget.alpha_2_slider.setValue(
            self._widget.alpha_2_slider.value() + self._widget.alpha_2_slider.pageStep())

    def _on_increase_alpha_3_pressed(self):
        self._widget.alpha_3_slider.setValue(
            self._widget.alpha_3_slider.value() + self._widget.alpha_3_slider.pageStep())

    def _on_decrease_alpha_1_pressed(self):
        self._widget.alpha_1_slider.setValue(
            self._widget.alpha_1_slider.value() - self._widget.alpha_1_slider.pageStep())

    def _on_decrease_alpha_2_pressed(self):
        self._widget.alpha_2_slider.setValue(
            self._widget.alpha_2_slider.value() - self._widget.alpha_2_slider.pageStep())

    def _on_decrease_alpha_3_pressed(self):
        self._widget.alpha_3_slider.setValue(
            self._widget.alpha_3_slider.value() - self._widget.alpha_3_slider.pageStep())

    def _on_strong_increase_beta_1_pressed(self):
        self._widget.beta_1_slider.setValue(
            self._widget.beta_1_slider.value() + self._widget.beta_1_slider.pageStep())

    def _on_strong_decrease_beta_1_pressed(self):
        self._widget.beta_1_slider.setValue(
            self._widget.beta_1_slider.value() - self._widget.beta_1_slider.pageStep())

    def _on_strong_increase_alpha_1_pressed(self):
        self._widget.alpha_1_slider.setValue(
            self._widget.alpha_1_slider.value() + self._widget.alpha_1_slider.pageStep())

    def _on_strong_decrease_alpha_1_pressed(self):
        self._widget.alpha_1_slider.setValue(
            self._widget.alpha_1_slider.value() - self._widget.alpha_1_slider.pageStep())

    def _on_parameter_changed(self):
        self._send_jointstate(
            self._widget.beta_1_slider.value(),
            self._widget.beta_2_slider.value(),
            self._widget.beta_3_slider.value(),
            self._widget.alpha_1_slider.value(),
            self._widget.alpha_2_slider.value(),
            self._widget.alpha_3_slider.value(), )

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
        self._set_slider_value(self._widget.beta_1_slider, self._widget.current_beta_1_label, msg.position[0], 'mm')
        self._set_slider_value(self._widget.beta_2_slider, self._widget.current_beta_2_label, msg.position[1], 'mm')
        self._set_slider_value(self._widget.beta_3_slider, self._widget.current_beta_3_label, msg.position[2], 'mm')
        self._set_slider_value(self._widget.alpha_1_slider, self._widget.current_alpha_1_label, msg.position[0], 'rad')
        self._set_slider_value(self._widget.alpha_2_slider, self._widget.current_alpha_2_label, msg.position[1], 'rad')
        self._set_slider_value(self._widget.alpha_3_slider, self._widget.current_alpha_3_label, msg.position[2], 'rad')

    def _initialize_slider(self, slider, min_label, max_label, current_label, slider_range, single_step, value, unit):
        slider.setMinimum(slider_range[0])
        slider.setMaximum(slider_range[1])
        slider.setSingleStep(single_step)
        min_label.setText(str(slider_range[0]) + ' ' + unit)
        max_label.setText(str(slider_range[1]) + ' ' + unit)
        self._set_slider_value(slider, current_label, value, unit)

    def _set_slider_value(self, slider, current_label, value, unit):
        slider.setValue(value)
        current_label.setText(str(value) + ' ' + unit)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._unregister_publisher()
