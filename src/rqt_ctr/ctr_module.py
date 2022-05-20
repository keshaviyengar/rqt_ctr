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


class CTRPlugin(Plugin):
    def __init__(self, context):
        super(CTRPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CTRPlugin')

        beta_1_range = [-100, -6]
        beta_2_range = [-100, 0]
        beta_3_range = [-150, 2]
        alpha_1_range = [-3.0, 3.0]
        alpha_2_range = [-3.0, 3.0]
        alpha_3_range = [-3.0, 3.0]

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

        self._widget.topic_line_edit.textChanged.connect(
            self._on_topic_changed)
        self._widget.stop_push_button.pressed.connect(self._on_stop_pressed)

        # Update min and max alphas and betas for label and sliders
        self._widget.beta_1_slider.setMinimum(beta_1_range[0])
        self._widget.beta_1_slider.setMaximum(beta_1_range[1])
        self._widget.beta_1_slider.setSingleStep(0.5)
        self._widget.beta_1_slider.setValue(-15.0)
        self._widget.current_beta_1_label.setText(str(-15.0) + ' mm')
        self._widget.min_beta_1_label.setText(str(beta_1_range[0]) + ' mm')
        self._widget.max_beta_1_label.setText(str(beta_1_range[1]) + ' mm')

        # When moving the sliders for beta_1 and alpha_1 update the value printed and sent a msg to publish
        self._widget.beta_1_slider.valueChanged.connect(
            self._on_beta_1_slider_changed)
        self._widget.alpha_1_slider.valueChanged.connect(
            self._on_alpha_1_slider_changed)

        # Beta_1 and alpha_1 push buttons to increase and decrease values
        self._widget.increase_beta_1_push_button.pressed.connect(
            self._on_increase_beta_1_pressed)
        self._widget.decrease_beta_1_push_button.pressed.connect(
            self._on_decrease_beta_1_pressed)
        self._widget.increase_alpha_1_push_button.pressed.connect(
            self._on_increase_alpha_1_pressed)
        self._widget.decrease_alpha_1_push_button.pressed.connect(
            self._on_decrease_alpha_1_pressed)

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

    def _on_alpha_1_slider_changed(self):
        self._widget.current_alpha_1_label.setText(
            '%0.2f rad' % (self._widget.alpha_1_slider.value()))

    def _on_increase_beta_1_pressed(self):
        self._widget.beta_1_slider.setValue(
            self._widget.beta_1_slider.value() + self._widget.beta_1_slider.singleStep())

    def _on_decrease_beta_1_pressed(self):
        self._widget.beta_1_slider.setValue(
            self._widget.beta_1_slider.value() - self._widget.beta_1_slider.singleStep())

    def _on_increase_alpha_1_pressed(self):
        self._widget.alpha_1_slider.setValue(
            self._widget.alpha_1_slider.value() + self._widget.alpha_1_slider.singleStep())

    def _on_decrease_alpha_1_pressed(self):
        self._widget.alpha_1_slider.setValue(
            self._widget.alpha_1_slider.value() - self._widget.alpha_1_slider.singleStep())

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
            self._widget.alpha_1_slider.value())

    def _send_jointstate(self, beta_1, alpha_1):
        if self._publisher is None:
            return
        # publish JointState as (beta1,beta2,beta3,alpha1,alpha2,alpha3), reading from each slider the new linear or
        # angular position and updating it to the correct tube parameter
        jointstate = JointState()
        jointstate.position = np.zeros(6)
        jointstate.position[2] = beta_1

        self.zero_cmd_sent = False
        self._publisher.publish(jointstate)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._unregister_publisher()
