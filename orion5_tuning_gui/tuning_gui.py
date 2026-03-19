import json
import math
import sys
from pathlib import Path
from typing import List

import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtWidgets import (
        QApplication,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QPushButton,
        QSlider,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    print("PyQt5 import failed. Install python3-pyqt5.", exc)
    sys.exit(1)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class RosBridge(Node):
    def __init__(self) -> None:
        super().__init__("orion5_tuning_gui")
        self.joint_names = [f"joint_{i}" for i in range(1, 7)]
        self.last_q = [0.0] * 6
        self.last_qd = [0.0] * 6
        self.last_cmd = [0.0] * 6
        self.last_bias = [0.0] * 6

        self.get_params_client = self.create_client(
            GetParameters, "/gravity_comp_relay_node/get_parameters")
        self.set_params_client = self.create_client(
            SetParameters, "/gravity_comp_relay_node/set_parameters")

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self.create_subscription(Float64MultiArray, "/effort_controller/commands", self._on_cmd, 50)
        self.create_subscription(Float64MultiArray, "/mujoco/bias_torque", self._on_bias, 50)

    def _on_joint_state(self, msg: JointState) -> None:
        for i, joint_name in enumerate(self.joint_names):
            try:
                idx = msg.name.index(joint_name)
            except ValueError:
                continue
            if idx < len(msg.position):
                self.last_q[i] = msg.position[idx]
            if idx < len(msg.velocity):
                self.last_qd[i] = msg.velocity[idx]

    def _on_cmd(self, msg: Float64MultiArray) -> None:
        n = min(6, len(msg.data))
        for i in range(n):
            self.last_cmd[i] = msg.data[i]

    def _on_bias(self, msg: Float64MultiArray) -> None:
        n = min(6, len(msg.data))
        for i in range(n):
            self.last_bias[i] = msg.data[i]

    def request_params(self, callback) -> None:
        if not self.get_params_client.service_is_ready():
            callback(None)
            return

        req = GetParameters.Request()
        req.names = ["q_ref", "kp", "kd", "gain_vector"]
        future = self.get_params_client.call_async(req)

        def on_done(fut):
            try:
                result = fut.result()
            except Exception:
                callback(None)
                return
            callback(result.values if result else None)

        future.add_done_callback(on_done)

    def set_arrays(self, q_ref: List[float], kp: List[float], kd: List[float], gain: List[float]) -> None:
        if not self.set_params_client.service_is_ready():
            self.get_logger().warn("/gravity_comp_relay_node parameter service not ready")
            return

        param_objs = [
            Parameter("q_ref", Parameter.Type.DOUBLE_ARRAY, q_ref),
            Parameter("kp", Parameter.Type.DOUBLE_ARRAY, kp),
            Parameter("kd", Parameter.Type.DOUBLE_ARRAY, kd),
            Parameter("gain_vector", Parameter.Type.DOUBLE_ARRAY, gain),
        ]
        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg() for p in param_objs]
        self.set_params_client.call_async(req)


class SliderRow:
    def __init__(self, joint_idx: int):
        self.joint_idx = joint_idx
        self.qref_slider = QSlider(Qt.Horizontal)
        self.kp_slider = QSlider(Qt.Horizontal)
        self.kd_slider = QSlider(Qt.Horizontal)
        self.gain_slider = QSlider(Qt.Horizontal)

        self.qref_value = QLabel("0.0 deg")
        self.kp_value = QLabel("0.0")
        self.kd_value = QLabel("0.0")
        self.gain_value = QLabel("1.00")

        self.q_value = QLabel("q=0.000")
        self.qd_value = QLabel("qd=0.000")
        self.bias_value = QLabel("bias=0.000")
        self.cmd_value = QLabel("cmd=0.000")

        self._configure_sliders()

    def _configure_sliders(self) -> None:
        self.qref_slider.setRange(-1800, 1800)  # 0.1 deg
        self.kp_slider.setRange(0, 1500)        # 0.1
        self.kd_slider.setRange(0, 600)         # 0.1
        self.gain_slider.setRange(0, 300)       # 0.01

        self.qref_slider.valueChanged.connect(self._update_labels)
        self.kp_slider.valueChanged.connect(self._update_labels)
        self.kd_slider.valueChanged.connect(self._update_labels)
        self.gain_slider.valueChanged.connect(self._update_labels)

        self.gain_slider.setValue(100)
        self._update_labels()

    def _update_labels(self) -> None:
        self.qref_value.setText(f"{self.qref_deg():.1f} deg")
        self.kp_value.setText(f"{self.kp():.1f}")
        self.kd_value.setText(f"{self.kd():.1f}")
        self.gain_value.setText(f"{self.gain():.2f}")

    def qref_deg(self) -> float:
        return self.qref_slider.value() / 10.0

    def qref_rad(self) -> float:
        return math.radians(self.qref_deg())

    def kp(self) -> float:
        return self.kp_slider.value() / 10.0

    def kd(self) -> float:
        return self.kd_slider.value() / 10.0

    def gain(self) -> float:
        return self.gain_slider.value() / 100.0

    def set_qref_rad(self, value: float) -> None:
        deg = clamp(math.degrees(value), -180.0, 180.0)
        self.qref_slider.setValue(int(round(deg * 10.0)))

    def set_kp(self, value: float) -> None:
        self.kp_slider.setValue(int(round(clamp(value, 0.0, 150.0) * 10.0)))

    def set_kd(self, value: float) -> None:
        self.kd_slider.setValue(int(round(clamp(value, 0.0, 60.0) * 10.0)))

    def set_gain(self, value: float) -> None:
        self.gain_slider.setValue(int(round(clamp(value, 0.0, 3.0) * 100.0)))


class TuningWindow(QWidget):
    def __init__(self, ros: RosBridge):
        super().__init__()
        self.ros = ros
        self.setWindowTitle("Orion5 Gravity/Hold Tuning")
        self.rows = [SliderRow(i) for i in range(6)]
        self.local_profile_path = Path.home() / ".ros" / "orion5_tuning_last.json"

        self.status_label = QLabel("Param service bekleniyor...")

        self._build_ui()
        self._connect_buttons()

        self.load_local_profile()
        self.refresh_from_node()

    def _build_ui(self) -> None:
        root = QVBoxLayout()

        slider_group = QGroupBox("Joint Tuning (q_ref, kp, kd, gain)")
        grid = QGridLayout()
        headers = ["Joint", "q_ref", "q_ref val", "kp", "kp val", "kd", "kd val", "gain", "gain val"]
        for c, title in enumerate(headers):
            grid.addWidget(QLabel(title), 0, c)

        for r, row in enumerate(self.rows, start=1):
            grid.addWidget(QLabel(f"joint_{r}"), r, 0)
            grid.addWidget(row.qref_slider, r, 1)
            grid.addWidget(row.qref_value, r, 2)
            grid.addWidget(row.kp_slider, r, 3)
            grid.addWidget(row.kp_value, r, 4)
            grid.addWidget(row.kd_slider, r, 5)
            grid.addWidget(row.kd_value, r, 6)
            grid.addWidget(row.gain_slider, r, 7)
            grid.addWidget(row.gain_value, r, 8)

        slider_group.setLayout(grid)
        root.addWidget(slider_group)

        live_group = QGroupBox("Live State / Torque")
        live_grid = QGridLayout()
        live_headers = ["Joint", "q(rad)", "qd(rad/s)", "bias_tau", "cmd_tau"]
        for c, title in enumerate(live_headers):
            live_grid.addWidget(QLabel(title), 0, c)

        for r, row in enumerate(self.rows, start=1):
            live_grid.addWidget(QLabel(f"joint_{r}"), r, 0)
            live_grid.addWidget(row.q_value, r, 1)
            live_grid.addWidget(row.qd_value, r, 2)
            live_grid.addWidget(row.bias_value, r, 3)
            live_grid.addWidget(row.cmd_value, r, 4)

        live_group.setLayout(live_grid)
        root.addWidget(live_group)

        btn_layout = QHBoxLayout()
        self.btn_apply = QPushButton("Apply Arrays")
        self.btn_refresh = QPushButton("Refresh From Node")
        self.btn_load_last = QPushButton("Load Last Local")
        self.btn_home = QPushButton("Preset Home 0,0,-90,0,-90,0")
        btn_layout.addWidget(self.btn_apply)
        btn_layout.addWidget(self.btn_refresh)
        btn_layout.addWidget(self.btn_load_last)
        btn_layout.addWidget(self.btn_home)
        root.addLayout(btn_layout)

        root.addWidget(self.status_label)
        self.setLayout(root)

    def _connect_buttons(self) -> None:
        self.btn_apply.clicked.connect(self.apply_to_node)
        self.btn_refresh.clicked.connect(self.refresh_from_node)
        self.btn_load_last.clicked.connect(self.load_local_profile)
        self.btn_home.clicked.connect(self.set_home_pose)

    def set_home_pose(self) -> None:
        home = [0.0, 0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0]
        for i, rad in enumerate(home):
            self.rows[i].set_qref_rad(rad)

    def collect_arrays(self):
        q_ref = [row.qref_rad() for row in self.rows]
        kp = [row.kp() for row in self.rows]
        kd = [row.kd() for row in self.rows]
        gain = [row.gain() for row in self.rows]
        return q_ref, kp, kd, gain

    def save_local_profile(self, q_ref, kp, kd, gain) -> None:
        payload = {
            "q_ref": q_ref,
            "kp": kp,
            "kd": kd,
            "gain_vector": gain,
        }
        self.local_profile_path.parent.mkdir(parents=True, exist_ok=True)
        self.local_profile_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def load_local_profile(self) -> None:
        if not self.local_profile_path.exists():
            return
        try:
            payload = json.loads(self.local_profile_path.read_text(encoding="utf-8"))
        except Exception:
            self.status_label.setText("Local profil okunamadı")
            return

        q_ref = payload.get("q_ref", [])
        kp = payload.get("kp", [])
        kd = payload.get("kd", [])
        gain = payload.get("gain_vector", [])

        for i, row in enumerate(self.rows):
            if i < len(q_ref):
                row.set_qref_rad(float(q_ref[i]))
            if i < len(kp):
                row.set_kp(float(kp[i]))
            if i < len(kd):
                row.set_kd(float(kd[i]))
            if i < len(gain):
                row.set_gain(float(gain[i]))
        self.status_label.setText("Local profil yüklendi")

    def apply_to_node(self) -> None:
        q_ref, kp, kd, gain = self.collect_arrays()
        self.ros.set_arrays(q_ref, kp, kd, gain)
        self.save_local_profile(q_ref, kp, kd, gain)
        self.status_label.setText("Parametreler gönderildi (q_ref/kp/kd/gain_vector)")

    def refresh_from_node(self) -> None:
        def on_params(values):
            if values is None:
                self.status_label.setText("Parametre servisi hazır değil veya okunamadı")
                return
            if len(values) != 4:
                self.status_label.setText("Beklenmeyen parametre cevabı")
                return

            q_ref = list(values[0].double_array_value)
            kp = list(values[1].double_array_value)
            kd = list(values[2].double_array_value)
            gain = list(values[3].double_array_value)

            for i, row in enumerate(self.rows):
                if i < len(q_ref):
                    row.set_qref_rad(q_ref[i])
                if i < len(kp):
                    row.set_kp(kp[i])
                if i < len(kd):
                    row.set_kd(kd[i])
                if i < len(gain):
                    row.set_gain(gain[i])
            self.status_label.setText("Parametreler node'dan okundu")

        self.ros.request_params(on_params)

    def update_live(self) -> None:
        for i, row in enumerate(self.rows):
            row.q_value.setText(f"{self.ros.last_q[i]: .3f}")
            row.qd_value.setText(f"{self.ros.last_qd[i]: .3f}")
            row.bias_value.setText(f"{self.ros.last_bias[i]: .3f}")
            row.cmd_value.setText(f"{self.ros.last_cmd[i]: .3f}")


def main() -> None:
    rclpy.init()
    ros = RosBridge()

    app = QApplication(sys.argv)
    window = TuningWindow(ros)
    window.resize(1380, 700)
    window.show()

    def spin_ros_once() -> None:
        if not rclpy.ok():
            return
        try:
            rclpy.spin_once(ros, timeout_sec=0.0)
        except Exception:
            return

    ros_timer = QTimer()
    ros_timer.timeout.connect(spin_ros_once)
    ros_timer.start(10)

    ui_timer = QTimer()
    ui_timer.timeout.connect(window.update_live)
    ui_timer.start(100)

    exit_code = app.exec_()
    ros_timer.stop()
    ui_timer.stop()
    ros.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
