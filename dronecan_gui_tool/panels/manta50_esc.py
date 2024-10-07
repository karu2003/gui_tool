#
# Copyright (C) 2024  IPM Group
#
# This software is distributed under the terms of the MIT License.
#
# Author: Andrew Buckin
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
from enum import Enum
import re

__all__ = "PANEL_NAME", "spawn", "get_icon"

PANEL_NAME = "Manta50 ESC Panel"

logger = getLogger(__name__)

_singleton = None

DEBUG = 0
INFO = 1


class CTRL_State(Enum):
    CTRL_State_Error = 0  # the controller error state
    CTRL_State_Idle = 1  # the controller idle state
    CTRL_State_OffLine = 2  # the controller offline state
    CTRL_State_OnLine = 3  # the controller online state
    CTRL_numStates = 4  # the number of controller states


class EST_State(Enum):
    EST_State_Error = 0  # error
    EST_State_Idle = 1  # idle
    EST_State_RoverL = 2  # R/L estimation
    EST_State_Rs = 3  # Rs estimation state
    EST_State_RampUp = 4  # ramp up the speed
    EST_State_IdRated = 5  # control Id and estimate the rated flux
    EST_State_RatedFlux_OL = 6  # estimate the open loop rated flux
    EST_State_RatedFlux = 7  # estimate the rated flux
    EST_State_RampDown = 8  # ramp down the speed
    EST_State_LockRotor = 9  # lock the rotor
    EST_State_Ls = 10  # stator inductance estimation state
    EST_State_Rr = 11  # rotor resistance estimation state
    EST_State_MotorIdentified = 12  # motor identified state
    EST_State_OnLine = 13  # online parameter estimation
    EST_numStates = 14  # the number of estimator states


class USER_ErrorCode(Enum):
    USER_ErrorCode_NoError = 0  # no error error code
    USER_ErrorCode_iqFullScaleCurrent_A_High = (
        1  # iqFullScaleCurrent_A too high error code
    )
    USER_ErrorCode_iqFullScaleCurrent_A_Low = (
        2  # iqFullScaleCurrent_A too low error code
    )
    USER_ErrorCode_iqFullScaleVoltage_V_High = (
        3  # iqFullScaleVoltage_V too high error code
    )
    USER_ErrorCode_iqFullScaleVoltage_V_Low = (
        4  # iqFullScaleVoltage_V too low error code
    )
    USER_ErrorCode_iqFullScaleFreq_Hz_High = 5  # iqFullScaleFreq_Hz too high error code
    USER_ErrorCode_iqFullScaleFreq_Hz_Low = 6  # iqFullScaleFreq_Hz too low error code
    USER_ErrorCode_numPwmTicksPerIsrTick_High = (
        7  # numPwmTicksPerIsrTick too high error code
    )
    USER_ErrorCode_numPwmTicksPerIsrTick_Low = (
        8  # numPwmTicksPerIsrTick too low error code
    )
    USER_ErrorCode_numIsrTicksPerCtrlTick_High = (
        9  # numIsrTicksPerCtrlTick too high error code
    )
    USER_ErrorCode_numIsrTicksPerCtrlTick_Low = (
        10  # numIsrTicksPerCtrlTick too low error code
    )
    USER_ErrorCode_numCtrlTicksPerCurrentTick_High = (
        11  # numCtrlTicksPerCurrentTick too high error code
    )
    USER_ErrorCode_numCtrlTicksPerCurrentTick_Low = (
        12  # numCtrlTicksPerCurrentTick too low error code
    )
    USER_ErrorCode_numCtrlTicksPerEstTick_High = (
        13  # numCtrlTicksPerEstTick too high error code
    )
    USER_ErrorCode_numCtrlTicksPerEstTick_Low = (
        14  # numCtrlTicksPerEstTick too low error code
    )
    USER_ErrorCode_numCtrlTicksPerSpeedTick_High = (
        15  # numCtrlTicksPerSpeedTick too high error code
    )
    USER_ErrorCode_numCtrlTicksPerSpeedTick_Low = (
        16  # numCtrlTicksPerSpeedTick too low error code
    )
    USER_ErrorCode_numCtrlTicksPerTrajTick_High = (
        17  # numCtrlTicksPerTrajTick too high error code
    )
    USER_ErrorCode_numCtrlTicksPerTrajTick_Low = (
        18  # numCtrlTicksPerTrajTick too low error code
    )
    USER_ErrorCode_numCurrentSensors_High = 19  # numCurrentSensors too high error code
    USER_ErrorCode_numCurrentSensors_Low = 20  # numCurrentSensors too low error code
    USER_ErrorCode_numVoltageSensors_High = 21  # numVoltageSensors too high error code
    USER_ErrorCode_numVoltageSensors_Low = 22  # numVoltageSensors too low error code
    USER_ErrorCode_offsetPole_rps_High = 23  # offsetPole_rps too high error code
    USER_ErrorCode_offsetPole_rps_Low = 24  # offsetPole_rps too low error code
    USER_ErrorCode_fluxPole_rps_High = 25  # fluxPole_rps too high error code
    USER_ErrorCode_fluxPole_rps_Low = 26  # fluxPole_rps too low error code
    USER_ErrorCode_zeroSpeedLimit_High = 27  # zeroSpeedLimit too high error code
    USER_ErrorCode_zeroSpeedLimit_Low = 28  # zeroSpeedLimit too low error code
    USER_ErrorCode_forceAngleFreq_Hz_High = 29  # forceAngleFreq_Hz too high error code
    USER_ErrorCode_forceAngleFreq_Hz_Low = 30  # forceAngleFreq_Hz too low error code
    USER_ErrorCode_maxAccel_Hzps_High = 31  # maxAccel_Hzps too high error code
    USER_ErrorCode_maxAccel_Hzps_Low = 32  # maxAccel_Hzps too low error code
    USER_ErrorCode_maxAccel_est_Hzps_High = 33  # maxAccel_est_Hzps too high error code
    USER_ErrorCode_maxAccel_est_Hzps_Low = 34  # maxAccel_est_Hzps too low error code
    USER_ErrorCode_directionPole_rps_High = 35  # directionPole_rps too high error code
    USER_ErrorCode_directionPole_rps_Low = 36  # directionPole_rps too low error code
    USER_ErrorCode_speedPole_rps_High = 37  # speedPole_rps too high error code
    USER_ErrorCode_speedPole_rps_Low = 38  # speedPole_rps too low error code
    USER_ErrorCode_dcBusPole_rps_High = 39  # dcBusPole_rps too high error code
    USER_ErrorCode_dcBusPole_rps_Low = 40  # dcBusPole_rps too low error code
    USER_ErrorCode_fluxFraction_High = 41  # fluxFraction too high error code
    USER_ErrorCode_fluxFraction_Low = 42  # fluxFraction too low error code
    USER_ErrorCode_indEst_speedMaxFraction_High = (
        43  # indEst_speedMaxFraction too high error code
    )
    USER_ErrorCode_indEst_speedMaxFraction_Low = (
        44  # indEst_speedMaxFraction too low error code
    )
    USER_ErrorCode_powerWarpGain_High = 45  # powerWarpGain too high error code
    USER_ErrorCode_powerWarpGain_Low = 46  # powerWarpGain too low error code
    USER_ErrorCode_systemFreq_MHz_High = 47  # systemFreq_MHz too high error code
    USER_ErrorCode_systemFreq_MHz_Low = 48  # systemFreq_MHz too low error code
    USER_ErrorCode_pwmFreq_kHz_High = 49  # pwmFreq_kHz too high error code
    USER_ErrorCode_pwmFreq_kHz_Low = 50  # pwmFreq_kHz too low error code
    USER_ErrorCode_voltage_sf_High = 51  # voltage_sf too high error code
    USER_ErrorCode_voltage_sf_Low = 52  # voltage_sf too low error code
    USER_ErrorCode_current_sf_High = 53  # current_sf too high error code
    USER_ErrorCode_current_sf_Low = 54  # current_sf too low error code
    USER_ErrorCode_voltageFilterPole_Hz_High = (
        55  # voltageFilterPole_Hz too high error code
    )
    USER_ErrorCode_voltageFilterPole_Hz_Low = (
        56  # voltageFilterPole_Hz too low error code
    )
    USER_ErrorCode_maxVsMag_pu_High = 57  # maxVsMag_pu too high error code
    USER_ErrorCode_maxVsMag_pu_Low = 58  # maxVsMag_pu too low error code
    USER_ErrorCode_estKappa_High = 59  # estKappa too high error code
    USER_ErrorCode_estKappa_Low = 60  # estKappa too low error code
    USER_ErrorCode_motor_type_Unknown = 61  # motor type unknown error code
    USER_ErrorCode_motor_numPolePairs_High = (
        62  # motor_numPolePairs too high error code
    )
    USER_ErrorCode_motor_numPolePairs_Low = 63  # motor_numPolePairs too low error code
    USER_ErrorCode_motor_ratedFlux_High = 64  # motor_ratedFlux too high error code
    USER_ErrorCode_motor_ratedFlux_Low = 65  # motor_ratedFlux too low error code
    USER_ErrorCode_motor_Rr_High = 66  # motor_Rr too high error code
    USER_ErrorCode_motor_Rr_Low = 67  # motor_Rr too low error code
    USER_ErrorCode_motor_Rs_High = 68  # motor_Rs too high error code
    USER_ErrorCode_motor_Rs_Low = 69  # motor_Rs too low error code
    USER_ErrorCode_motor_Ls_d_High = 70  # motor_Ls_d too high error code
    USER_ErrorCode_motor_Ls_d_Low = 71  # motor_Ls_d too low error code
    USER_ErrorCode_motor_Ls_q_High = 72  # motor_Ls_q too high error code
    USER_ErrorCode_motor_Ls_q_Low = 73  # motor_Ls_q too low error code
    USER_ErrorCode_maxCurrent_High = 74  # maxCurrent too high error code
    USER_ErrorCode_maxCurrent_Low = 75  # maxCurrent too low error code
    USER_ErrorCode_maxCurrent_resEst_High = 76  # maxCurrent_resEst too high error code
    USER_ErrorCode_maxCurrent_resEst_Low = 77  # maxCurrent_resEst too low error code
    USER_ErrorCode_maxCurrent_indEst_High = 78  # maxCurrent_indEst too high error code
    USER_ErrorCode_maxCurrent_indEst_Low = 79  # maxCurrent_indEst too low error code
    USER_ErrorCode_maxCurrentSlope_High = 80  # maxCurrentSlope too high error code
    USER_ErrorCode_maxCurrentSlope_Low = 81  # maxCurrentSlope too low error code
    USER_ErrorCode_maxCurrentSlope_powerWarp_High = (
        82  # maxCurrentSlope_powerWarp too high error code
    )
    USER_ErrorCode_maxCurrentSlope_powerWarp_Low = (
        83  # maxCurrentSlope_powerWarp too low error code
    )
    USER_ErrorCode_IdRated_High = 84  # IdRated too high error code
    USER_ErrorCode_IdRated_Low = 85  # IdRated too low error code
    USER_ErrorCode_IdRatedFraction_indEst_High = (
        86  # IdRatedFraction_indEst too high error code
    )
    USER_ErrorCode_IdRatedFraction_indEst_Low = (
        87  # IdRatedFraction_indEst too low error code
    )
    USER_ErrorCode_IdRatedFraction_ratedFlux_High = (
        88  # IdRatedFraction_ratedFlux too high error code
    )
    USER_ErrorCode_IdRatedFraction_ratedFlux_Low = (
        89  # IdRatedFraction_ratedFlux too low error code
    )
    USER_ErrorCode_IdRated_delta_High = 90  # IdRated_delta too high error code
    USER_ErrorCode_IdRated_delta_Low = 91  # IdRated_delta too low error code
    USER_ErrorCode_fluxEstFreq_Hz_High = 92  # fluxEstFreq_Hz too high error code
    USER_ErrorCode_fluxEstFreq_Hz_Low = 93  # fluxEstFreq_Hz too low error code
    USER_ErrorCode_ctrlFreq_Hz_High = 94  # ctrlFreq_Hz too high error code
    USER_ErrorCode_ctrlFreq_Hz_Low = 95  # ctrlFreq_Hz too low error code
    USER_ErrorCode_estFreq_Hz_High = 96  # estFreq_Hz too high error code
    USER_ErrorCode_estFreq_Hz_Low = 97  # estFreq_Hz too low error code
    USER_ErrorCode_RoverL_estFreq_Hz_High = 98  # RoverL_estFreq_Hz too high error code
    USER_ErrorCode_RoverL_estFreq_Hz_Low = 99  # RoverL_estFreq_Hz too low error code
    USER_ErrorCode_trajFreq_Hz_High = 100  # trajFreq_Hz too high error code
    USER_ErrorCode_trajFreq_Hz_Low = 101  # trajFreq_Hz too low error code
    USER_ErrorCode_ctrlPeriod_sec_High = 102  # ctrlPeriod_sec too high error code
    USER_ErrorCode_ctrlPeriod_sec_Low = 103  # ctrlPeriod_sec too low error code
    USER_ErrorCode_maxNegativeIdCurrent_a_High = (
        104  # maxNegativeIdCurrent_a too high error code
    )
    USER_ErrorCode_maxNegativeIdCurrent_a_Low = (
        105  # maxNegativeIdCurrent_a too low error code
    )
    USER_numErrorCodes = 106  # the number of user error codes


def replace_enum_values(message):
    def replace_match(match):
        prefix = match.group(1)
        number_str = match.group(2)
        try:
            number = int(number_str)
            if prefix == "UserErrorCode:":
                return f"{prefix} {USER_ErrorCode(number).name}"
            elif prefix == "CtrlState:":
                return f"{prefix} {CTRL_State(number).name}"
            elif prefix == "EstState:":
                return f"{prefix} {EST_State(number).name}"
        except (ValueError, KeyError):
            return match.group(0)
        return match.group(0)

    pattern = r"(UserErrorCode:|CtrlState:|EstState:)\s*(\d+)"
    return re.sub(pattern, replace_match, message)


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
        self.setLayout(layout)
        self.setStyleSheet("Manta50Panel { border: 2px solid darkblue; }")

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

        # CtrlState message display
        self.CtrlState_display = QPlainTextEdit()
        self.CtrlState_display.setReadOnly(True)
        self.CtrlState_display.setFont(get_monospace_font())
        self.CtrlState_display.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.CtrlState_display.setWordWrapMode(QTextOption.NoWrap)
        self.CtrlState_display.setUndoRedoEnabled(False)
        self.CtrlState_display.setContextMenuPolicy(Qt.NoContextMenu)
        self.CtrlState_display.setTabStopWidth(20)
        self.CtrlState_display.setPlaceholderText("CtrlState messages")

        # EstState message display
        self.EstState_display = QPlainTextEdit()
        self.EstState_display.setReadOnly(True)
        self.EstState_display.setFont(get_monospace_font())
        self.EstState_display.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.EstState_display.setWordWrapMode(QTextOption.NoWrap)
        self.EstState_display.setUndoRedoEnabled(False)
        self.EstState_display.setContextMenuPolicy(Qt.NoContextMenu)
        self.EstState_display.setTabStopWidth(20)
        self.EstState_display.setPlaceholderText("EstState messages")

        # UserErrorCode message display
        self.UserErrorCode_display = QPlainTextEdit()
        self.UserErrorCode_display.setReadOnly(True)
        self.UserErrorCode_display.setFont(get_monospace_font())
        self.UserErrorCode_display.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.UserErrorCode_display.setWordWrapMode(QTextOption.NoWrap)
        self.UserErrorCode_display.setUndoRedoEnabled(False)
        self.UserErrorCode_display.setContextMenuPolicy(Qt.NoContextMenu)
        self.UserErrorCode_display.setTabStopWidth(20)
        self.UserErrorCode_display.setPlaceholderText("UserErrorCode messages")

        message_layout = QHBoxLayout()
        message_layout.addWidget(self.CtrlState_display)
        message_layout.addWidget(self.EstState_display)
        message_layout.addWidget(self.UserErrorCode_display)

        layout.addLayout(message_layout)

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
            decoded_message = message.text.decode("utf-8")
            updated_message = replace_enum_values(decoded_message)

            user_error_code_match = re.search(
                r"(UserErrorCode:)\s*(\d+)", decoded_message
            )
            if user_error_code_match:
                updated_message = replace_enum_values(user_error_code_match.group(0))
                self.UserErrorCode_display.appendPlainText(updated_message)

            ctrl_state_match = re.search(r"(CtrlState:)\s*(\d+)", decoded_message)
            if ctrl_state_match:
                updated_message = replace_enum_values(ctrl_state_match.group(0))
                self.CtrlState_display.appendPlainText(updated_message)

            est_state_match = re.search(r"(EstState:)\s*(\d+)", decoded_message)
            if est_state_match:
                updated_message = replace_enum_values(est_state_match.group(0))
                self.EstState_display.appendPlainText(updated_message)

        if message.level.value == INFO:
            self.set_checkboxes_from_byte(0)
            self.CtrlState_display.clear()
            self.EstState_display.clear()
            self.UserErrorCode_display.clear()
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
