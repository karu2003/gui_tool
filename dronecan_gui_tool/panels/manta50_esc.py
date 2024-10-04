#
# Copyright (C) 2023  UAVCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Andrew Tridgell
#

import dronecan
from functools import partial

from PyQt5.QtWidgets import (
    QVBoxLayout,
    QWidget,
    QLabel,
    QDialog,
    QPlainTextEdit,
    QPushButton,
    QLineEdit,
    QFileDialog,
    QComboBox,
    QHBoxLayout,
    QSpinBox,
    QTableWidgetItem,
    QDoubleSpinBox,
    QTableWidget,
    QHeaderView,
    QCheckBox,
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QTextOption
from logging import getLogger
from ..widgets import make_icon_button, get_icon, get_monospace_font
from ..widgets import table_display
import random
import base64
import struct

__all__ = "PANEL_NAME", "spawn", "get_icon"

PANEL_NAME = "Manta50 ESC Panel"

logger = getLogger(__name__)

_singleton = None

DEBUG = 0
INFO = 1


class Manta50Panel(QDialog):
    DEFAULT_INTERVAL = 0.1

    def __init__(self, parent, node):
        super(Manta50Panel, self).__init__(parent)
        self.setWindowTitle("Manta ESC Panel")
        self.setAttribute(
            Qt.WA_DeleteOnClose
        )  # This is required to stop background timers!

        self.param_index = {
            "Node ID": (0, int),
            "ESC Index": (1, int),
            "Arming": (2, int),
            "Telemetry Rate": (3, int),
            "CAN Speed": (4, int),
            "Max Speed": (5, float),
            "Control Word": (6, int),
            "Midle Point": (7, int),
            "Acceleration": (8, float),
            "Motor Poles": (9, int),
            "KP": (10, float),
            "KI": (11, float),
        }
        self.bmap = {
            "1MHz->12": 12,
            "500KHz->11": 11,
            "250KHz->10": 10,
            "200KHz->9": 9,
            "125KHz->8": 8,
            "100KHz->7": 7,
            "80KHz->6": 6,
            "50KHz->5": 5,
        }

        self._node = node
        self.fetch = 0
        self.current_index = 0
        self.messages = {}

        self.integer_params_key = [
            key
            for key, (index, param_type) in self.param_index.items()
            if param_type is int
        ]
        self.integer_params_index = [
            index
            for key, (index, param_type) in self.param_index.items()
            if param_type is int
        ]

        layout = QVBoxLayout()

        self.node_select = QComboBox()
        self.table = QTableWidget()

        layout.addLayout(self.labelWidget("Node", self.node_select))

        fetch = QHBoxLayout()
        self.fetch_label = QLabel("Fetch all parameters by index about 10s")
        self.fetch_set = QPushButton("Fetch All", self)
        self.fetch_set.clicked.connect(self.send_param_requests)

        fetch.addWidget(self.fetch_label, stretch=5)
        fetch.addWidget(self.fetch_set, stretch=1)

        layout.addLayout(fetch)
        layout.addWidget(self.table)

        # Log message display
        self.log_display = QPlainTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setFont(get_monospace_font())
        self.log_display.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.log_display.setWordWrapMode(QTextOption.NoWrap)
        self.log_display.setUndoRedoEnabled(False)
        self.log_display.setContextMenuPolicy(Qt.NoContextMenu)
        self.log_display.setTabStopWidth(20)
        self.log_display.setPlaceholderText("Log messages")
        layout.addWidget(self.log_display)

        # 0
        self.node_id = QSpinBox(self)
        self.node_id.setMinimum(1)
        self.node_id.setMaximum(127)
        self.node_id.setValue(1)
        self.node_id_set = QPushButton("Set", self)
        self.node_id_set.clicked.connect(self.on_nodeid_set)
        layout.addLayout(self.labelWidget("Node ID:", [self.node_id, self.node_id_set]))

        # 1
        self.esc_index = QSpinBox(self)
        self.esc_index.setMinimum(0)
        self.esc_index.setMaximum(32)
        self.esc_index.setValue(0)
        self.esc_index_set = QPushButton("Set", self)
        self.esc_index_set.clicked.connect(self.on_esc_index_set)
        layout.addLayout(
            self.labelWidget("ESC Index:", [self.esc_index, self.esc_index_set])
        )

        # 2
        self.arming = QCheckBox(self)
        self.arming_set = QPushButton("Set", self)
        self.arming_set.clicked.connect(self.on_arming_set)
        layout.addLayout(self.labelWidget("Arming", [self.arming, self.arming_set]))

        # 3
        self.tele_rate = QComboBox(self)
        for r in [0, 1, 5, 10, 20, 25, 50, 100]:
            self.tele_rate.addItem(str(r))
        self.tele_rate.setCurrentText("25")
        self.tele_rate_set = QPushButton("Set", self)
        self.tele_rate_set.clicked.connect(self.on_tele_rate_set)
        layout.addLayout(
            self.labelWidget("Telemetry Rate Hz:", [self.tele_rate, self.tele_rate_set])
        )

        # 4
        self.baudrate = QComboBox(self)
        for b in self.bmap.keys():
            self.baudrate.addItem(b)
        self.baudrate.setCurrentText(list(self.bmap.keys())[1])
        self.baudrate_set = QPushButton("Set", self)
        self.baudrate_set.clicked.connect(self.on_baudrate_set)
        layout.addLayout(
            self.labelWidget("CAN Speed:", [self.baudrate, self.baudrate_set])
        )
        # 5
        self.max_speed = QDoubleSpinBox(self)
        self.max_speed.setMinimum(0.0)
        self.max_speed.setMaximum(100.0)
        self.max_speed.setValue(5.0)
        self.max_speed_set = QPushButton("Set", self)
        self.max_speed_set.clicked.connect(self.on_max_speed_set)
        layout.addLayout(
            self.labelWidget(
                "Motor Max Speed KRPM:", [self.max_speed, self.max_speed_set]
            )
        )
        # 6
        self.checkboxes = {
            "enableSys": QCheckBox("Enable System", self),
            "runIdentify": QCheckBox("Run Identify", self),
            "FieldWeakening": QCheckBox("Field Weakening", self),
            "ForceAngle": QCheckBox("Force Angle", self),
            "RsRecalc": QCheckBox("Rs Recalc", self),
            "PowerWarp": QCheckBox("Power Warp", self),
            "UserParams": QCheckBox("User Params", self),
        }
        self.checkbox_layout = QHBoxLayout()
        for checkbox in self.checkboxes.values():
            self.checkbox_layout.addWidget(checkbox)
        self.checkbox_set_button = QPushButton("Set", self)
        self.checkbox_set_button.clicked.connect(self.on_checkbox_set)
        self.checkbox_layout.addWidget(self.checkbox_set_button)
        layout.addLayout(self.checkbox_layout)

        # 7
        self.midle_point = QCheckBox(self)
        self.midle_point_set = QPushButton("Set", self)
        self.midle_point_set.clicked.connect(self.on_midle_point_set)
        layout.addLayout(
            self.labelWidget("Midle Point", [self.midle_point, self.midle_point_set])
        )

        # 8
        self.acceleration = QDoubleSpinBox(self)
        self.acceleration.setMinimum(0.0)
        self.acceleration.setMaximum(100.0)
        self.acceleration.setValue(2.0)
        self.acceleration_set = QPushButton("Set", self)
        self.acceleration_set.clicked.connect(self.on_acceleration_set)
        layout.addLayout(
            self.labelWidget(
                "Acceleration KRPM:", [self.acceleration, self.acceleration_set]
            )
        )

        # 9
        self.motor_poles = QSpinBox(self)
        self.motor_poles.setMinimum(0)
        self.motor_poles.setMaximum(32)
        self.motor_poles.setValue(14)
        self.motor_poles_set = QPushButton("Set", self)
        self.motor_poles_set.clicked.connect(self.on_motor_poles_set)
        layout.addLayout(
            self.labelWidget("Motor Poles:", [self.motor_poles, self.motor_poles_set])
        )

        # 10
        self.kp = QDoubleSpinBox(self)
        self.kp.setMinimum(0.0)
        self.kp.setMaximum(100.0)
        self.kp.setDecimals(3)
        self.kp.setValue(3.0)
        self.kp_set = QPushButton("Set", self)
        self.kp_set.clicked.connect(self.on_kp_set)
        layout.addLayout(self.labelWidget("KP:", [self.kp, self.kp_set]))

        # 11
        self.ki = QDoubleSpinBox(self)
        self.ki.setMinimum(0.0)
        self.ki.setMaximum(100.0)
        self.ki.setDecimals(3)
        self.ki.setValue(0.059)
        self.ki_set = QPushButton("Set", self)
        self.ki_set.clicked.connect(self.on_ki_set)
        layout.addLayout(self.labelWidget("KI:", [self.ki, self.ki_set]))

        self.setLayout(layout)
        self.resize(1200, 450)

        self.handlers = [
            node.add_handler(
                dronecan.uavcan.protocol.debug.LogMessage, self.handle_debug_log_message
            ),
        ]

        QTimer.singleShot(767, self.update_nodes)

    def send_param_requests(self):
        self.fetch_label.setText("Fetch all parameters by index about 10s")
        self.messages = {}
        self.nodeid = int(self.node_select.currentText().split(":")[0])
        self.current_index = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_next_request)
        self.timer.start(987)

    def send_next_request(self):
        if self.current_index < len(self.param_index):
            request = dronecan.uavcan.protocol.param.GetSet.Request(
                index=self.current_index
            )
            self._node.request(request, self.nodeid, self.empty_callback)
            self.current_index += 1
        else:
            self.timer.stop()
            self.current_index = 0

    def send_value(self, index, value):
        self.nodeid = int(self.node_select.currentText().split(":")[0])
        if isinstance(value, float):
            print(value)
            request = dronecan.uavcan.protocol.param.GetSet.Request(
                index=index,
                value=dronecan.uavcan.protocol.param.Value(real_value=value),
            )
        else:
            request = dronecan.uavcan.protocol.param.GetSet.Request(
                index=index,
                value=dronecan.uavcan.protocol.param.Value(integer_value=value),
            )

        self._node.request(request, self.nodeid, self.empty_callback)

    def empty_callback(self, event):
        pass

    def update_table(self):
        self.table.clearContents()
        self.table.setRowCount(2)
        self.table.setColumnCount(len(self.messages))
        self.table.setHorizontalHeaderLabels(self.messages.keys())
        self.table.setVerticalHeaderLabels(["Index", "Value"])

        for col, (key, (index, value)) in enumerate(self.messages.items()):
            index_item = QTableWidgetItem(str(index))
            index_item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(0, col, index_item)

            value_item = QTableWidgetItem(value)
            value_item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(1, col, value_item)

        self.table.resizeColumnsToContents()
        self.table.resizeRowsToContents()


    def handle_debug_log_message(self, event):

        message = event.transfer.payload

        if message.level.value != INFO:
            self.log_display.appendPlainText(message.text.decode("utf-8"))
        
        if message.level.value == INFO:
            self.set_checkboxes_from_byte(0)
            self.log_display.clear()
            parts = message.text.decode("utf-8").split(" ")
            if len(parts) >= 2:
                key, value = parts[0], " ".join(parts[1:])
                if self.current_index - 1 in self.integer_params_index:
                    value = str(int(float(value)))
                self.messages[key] = (self.current_index - 1, value)
                self.update_table()

            if len(self.messages) == len(self.param_index):
                self.fetch_label.setText("All parameters fetched")
                self.table.resizeColumnsToContents()
                self.table.resizeRowsToContents()

                self.update_ui_from_params()

    def get_param_value(self, param_name, convert_func=str):
        """
        Returns the value of a parameter from the table, if it exists.
        """
        column_index = self.param_index.get(param_name, [None])[0]
        if column_index is not None:
            item = self.table.item(1, column_index)
            return convert_func(item.text())
        return None

    def update_ui_from_params(self):
        """
        Updates all UI elements based on parameter values.
        """
        # Update Node ID
        node_id = self.get_param_value("Node ID", int)
        if node_id is not None:
            self.node_id.setValue(node_id)

        # Update ESC Index
        esc_index = self.get_param_value("ESC Index", int)
        if esc_index is not None:
            self.esc_index.setValue(esc_index)

        # Update Arming status
        arming = self.get_param_value("Arming", int)
        if arming is not None:
            self.arming.setChecked(bool(arming))

        # Update Telemetry Rate
        tele_rate = self.get_param_value("Telemetry Rate")
        if tele_rate is not None:
            self.tele_rate.setCurrentText(tele_rate)

        # Update CAN Speed
        baudrate_value = self.get_param_value("CAN Speed", int)
        if baudrate_value is not None:
            baudrate_key = next(
                (key for key, value in self.bmap.items() if value == baudrate_value),
                None,
            )
            if baudrate_key:
                self.baudrate.setCurrentText(baudrate_key)

        # Update Max Speed
        max_speed = self.get_param_value("Max Speed", float)
        if max_speed is not None:
            self.max_speed.setValue(max_speed)

        # Update Control Word
        control_word = self.get_param_value("Control Word", int)
        if control_word is not None:
            self.set_checkboxes_from_byte(control_word)

        # Update Midle Point
        midle_point = self.get_param_value("Midle Point", int)
        if midle_point is not None:
            self.midle_point.setChecked(bool(midle_point))

        # Update Acceleration
        acceleration = self.get_param_value("Acceleration", float)
        if acceleration is not None:
            self.acceleration.setValue(acceleration)

        # Update Motor Poles
        motor_poles = self.get_param_value("Motor Poles", int)
        if motor_poles is not None:
            self.motor_poles.setValue(motor_poles)

        # Update KP
        kp = self.get_param_value("KP", float)
        if kp is not None:
            self.kp.setValue(kp)

        # Update KI
        ki = self.get_param_value("KI", float)
        if ki is not None:
            self.ki.setValue(ki)

    def update_nodes(self):
        """update list of available nodes"""
        QTimer.singleShot(767, self.update_nodes)
        from ..widgets.node_monitor import app_node_monitor

        if app_node_monitor is None:
            return
        node_list = []
        for nid in app_node_monitor._registry.keys():
            r = app_node_monitor._registry[nid]
            if r.info is not None:
                node_list.append("%u: %s" % (nid, r.info.name.decode()))
            else:
                node_list.append("%u" % nid)
        node_list = sorted(node_list)
        current_node = sorted(
            [self.node_select.itemText(i) for i in range(self.node_select.count())]
        )
        for n in node_list:
            if not n in current_node:
                self.node_select.addItem(n)

    def on_baudrate_set(self):
        """set baudrate"""
        com_index = self.param_index["CAN Speed"][0]
        self.current_index = com_index + 1
        baudrate = self.baudrate.currentText()
        baud = self.bmap[baudrate]
        QTimer.singleShot(767, lambda: self.send_value(com_index, int(baud)))

    def on_esc_index_set(self):
        """set esc index"""
        com_index = self.param_index["ESC Index"][0]
        self.current_index = com_index + 1
        esc_index = int(self.esc_index.value())
        QTimer.singleShot(767, lambda: self.send_value(com_index, esc_index))

    def on_nodeid_set(self):
        """set node ID"""
        com_index = self.param_index["Node ID"][0]
        self.current_index = com_index + 1
        node_iid = int(self.node_id.value())
        QTimer.singleShot(767, lambda: self.send_value(com_index, node_iid))

    def on_tele_rate_set(self):
        """set tele rate"""
        com_index = self.param_index["Telemetry Rate"][0]
        self.current_index = com_index + 1
        t_rate = int(self.tele_rate.currentText())
        QTimer.singleShot(767, lambda: self.send_value(com_index, t_rate))

    def on_max_speed_set(self):
        """set motor max speed"""
        com_index = self.param_index["Max Speed"][0]
        self.current_index = com_index + 1
        max_speed = self.max_speed.value()
        QTimer.singleShot(767, lambda: self.send_value(com_index, max_speed))

    def on_acceleration_set(self):
        """set motor max speed"""
        com_index = self.param_index["Acceleration"][0]
        self.current_index = com_index + 1
        acceleration = self.acceleration.value()
        QTimer.singleShot(767, lambda: self.send_value(com_index, acceleration))

    def on_arming_set(self):
        """set arming"""
        com_index = self.param_index["Arming"][0]
        self.current_index = com_index + 1
        arming = 1 if self.arming.isChecked() else 0
        print("Arming", arming)
        QTimer.singleShot(767, lambda: self.send_value(com_index, arming))

    def on_midle_point_set(self):
        """set midle point"""
        com_index = self.param_index["Midle Point"][0]
        self.current_index = com_index + 1
        midle_point = 1 if self.midle_point.isChecked() else 0
        QTimer.singleShot(767, lambda: self.send_value(com_index, midle_point))

    def on_motor_poles_set(self):
        """set motor poles"""
        com_index = self.param_index["Motor Poles"][0]
        self.current_index = com_index + 1
        motor_poles = int(self.motor_poles.value())
        QTimer.singleShot(767, lambda: self.send_value(com_index, motor_poles))

    def on_kp_set(self):
        """set kp"""
        com_index = self.param_index["KP"][0]
        self.current_index = com_index + 1
        kp = self.kp.value()
        QTimer.singleShot(767, lambda: self.send_value(com_index, kp))

    def on_ki_set(self):
        """set ki"""
        com_index = self.param_index["KI"][0]
        self.current_index = com_index + 1
        ki = self.ki.value()
        QTimer.singleShot(767, lambda: self.send_value(com_index, ki))

    def get_byte_from_checkboxes(self):
        byte = 0
        for i, checkbox in enumerate(self.checkboxes.values()):
            if checkbox.isChecked():
                byte |= 1 << i
        return byte

    def set_checkboxes_from_byte(self, byte):
        for i, checkbox in enumerate(self.checkboxes.values()):
            checkbox.setChecked(byte & (1 << i))

    def on_checkbox_set(self):
        com_index = self.param_index["Control Word"][0]
        self.current_index = com_index + 1
        control_word = self.get_byte_from_checkboxes()
        QTimer.singleShot(767, lambda: self.send_value(com_index, control_word))

    def handle_GetEscID(self, msg):
        """handle GetEscID"""
        nodeid = msg.transfer.source_node_id
        if len(msg.message.payload) != 2:
            return
        data = self.table.get(nodeid)
        if data is None:
            data = [nodeid, 0, 0, 0, 0, 0, 0]
        data[1] = msg.message.payload[1]
        self.table.update(nodeid, data)

    def labelWidget(self, label, widgets):
        """a widget with a label"""
        hlayout = QHBoxLayout()
        hlayout.addWidget(QLabel(label, self))
        if not isinstance(widgets, list):
            widgets = [widgets]
        for w in widgets:
            hlayout.addWidget(w)
        return hlayout

    def __del__(self):
        for h in self.handlers:
            h.remove()
        global _singleton
        _singleton = None

    def closeEvent(self, event):
        super(Manta50Panel, self).closeEvent(event)
        self.__del__()


def spawn(parent, node):
    global _singleton
    if _singleton is None:
        try:
            _singleton = Manta50Panel(parent, node)
        except Exception as ex:
            print(ex)

    _singleton.show()
    _singleton.raise_()
    _singleton.activateWindow()

    return _singleton


get_icon = partial(get_icon, "asterisk")
