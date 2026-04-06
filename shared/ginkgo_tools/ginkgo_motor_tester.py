#!/usr/bin/env python3

from __future__ import annotations

import sys
import time
from pathlib import Path

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from odrive_ginkgo import ODriveGinkgoClient, TelemetrySnapshot


def _format_metric(value: float, unit: str, precision: int = 3) -> str:
    if value != value:
        return "n/a"
    return f"{value:.{precision}f} {unit}".strip()


class MotorTesterWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Ginkgo ODrive Motor Tester - Gerardo Escobar")
        self.resize(1100, 760)

        self.client: ODriveGinkgoClient | None = None

        self.channel_edit = QLineEdit("0")
        self.kbps_edit = QLineEdit("500")
        self.node_id_edit = QLineEdit("0x10")
        self.timeout_edit = QLineEdit("0.15")
        self.refresh_ms_edit = QLineEdit("750")
        self.target_edit = QLineEdit("0.0")
        self.vel_ff_edit = QLineEdit("0")
        self.torque_ff_edit = QLineEdit("0")
        self.auto_refresh_checkbox = QCheckBox("Auto refresh")
        self.auto_refresh_checkbox.setChecked(True)
        self.status_label = QLabel("Disconnected")

        self.telemetry_labels: dict[str, QLabel] = {
            "position": QLabel("n/a"),
            "velocity": QLabel("n/a"),
            "iq_setpoint": QLabel("n/a"),
            "iq_measured": QLabel("n/a"),
            "fet_temp": QLabel("n/a"),
            "motor_temp": QLabel("n/a"),
            "bus_voltage": QLabel("n/a"),
            "bus_current": QLabel("n/a"),
            "torque_target": QLabel("n/a"),
            "torque_estimate": QLabel("n/a"),
            "electrical_power": QLabel("n/a"),
            "mechanical_power": QLabel("n/a"),
        }

        self.log_widget = QPlainTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_widget.setLineWrapMode(QPlainTextEdit.LineWrapMode.WidgetWidth)

        self._build_ui()

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._auto_refresh_tick)
        self.refresh_timer.start(self._refresh_interval_ms())

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)

        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        title = QLabel("Standalone Ginkgo + ODrive Bench Tester")
        title.setStyleSheet("font-size: 20px; font-weight: 700;")
        subtitle = QLabel("Use small moves first. Position commands are in ODrive turns.")
        subtitle.setStyleSheet("color: #4b5563;")

        layout.addWidget(title)
        layout.addWidget(subtitle)

        top_row = QHBoxLayout()
        top_row.setSpacing(12)
        top_row.addWidget(self._build_connection_group(), 1)
        top_row.addWidget(self._build_axis_group(), 1)
        layout.addLayout(top_row)

        layout.addWidget(self._build_telemetry_group())
        layout.addWidget(self._build_log_group(), 1)

        signature = QLabel("Gerardo Escobar")
        signature.setAlignment(Qt.AlignmentFlag.AlignRight)
        signature.setStyleSheet("font-style: italic; color: #6b7280;")
        layout.addWidget(signature)

        self.statusBar().showMessage("Disconnected")

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)

        fields = [
            ("Channel", self.channel_edit),
            ("Bitrate (kbps)", self.kbps_edit),
            ("Node ID", self.node_id_edit),
            ("Timeout (s)", self.timeout_edit),
            ("Refresh (ms)", self.refresh_ms_edit),
        ]

        for row, (label_text, widget) in enumerate(fields):
            layout.addWidget(QLabel(label_text), row, 0)
            layout.addWidget(widget, row, 1, 1, 3)

        layout.addWidget(self.auto_refresh_checkbox, 5, 0, 1, 2)
        layout.addWidget(self.status_label, 5, 2, 1, 2, Qt.AlignmentFlag.AlignRight)

        button_row = [
            ("Connect", self._connect),
            ("Disconnect", self._disconnect),
            ("Clear Errors", self._clear_errors),
            ("Read Once", lambda: self._refresh_telemetry(log_result=True)),
        ]
        for column, (text, handler) in enumerate(button_row):
            button = QPushButton(text)
            button.clicked.connect(handler)
            layout.addWidget(button, 6, column)

        return group

    def _build_axis_group(self) -> QGroupBox:
        group = QGroupBox("Axis Control")
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(3, 1)

        controls = [
            ("Closed Loop", self._enter_closed_loop),
            ("Idle", self._idle),
            ("Move Zero", lambda: self._move_to(0.0)),
            ("Read Encoder", self._read_encoder_only),
        ]
        for column, (text, handler) in enumerate(controls):
            button = QPushButton(text)
            button.clicked.connect(handler)
            layout.addWidget(button, 0, column)

        layout.addWidget(QLabel("Target (turns)"), 1, 0)
        layout.addWidget(self.target_edit, 1, 1)
        layout.addWidget(QLabel("Vel FF"), 1, 2)
        layout.addWidget(self.vel_ff_edit, 1, 3)

        layout.addWidget(QLabel("Torque FF"), 2, 0)
        layout.addWidget(self.torque_ff_edit, 2, 1)
        send_button = QPushButton("Send Position")
        send_button.clicked.connect(self._move_to_target)
        layout.addWidget(send_button, 2, 2, 1, 2)

        step_buttons = [("-0.10", -0.10), ("-0.01", -0.01), ("+0.01", 0.01), ("+0.10", 0.10)]
        for column, (label, delta) in enumerate(step_buttons):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, step=delta: self._offset_target(step))
            layout.addWidget(button, 3, column)

        return group

    def _build_telemetry_group(self) -> QGroupBox:
        group = QGroupBox("Telemetry")
        form = QGridLayout(group)
        form.setColumnStretch(1, 1)
        form.setColumnStretch(3, 1)

        rows = [
            ("Position", "position", "Velocity", "velocity"),
            ("Iq Setpoint", "iq_setpoint", "Iq Measured", "iq_measured"),
            ("FET Temp", "fet_temp", "Motor Temp", "motor_temp"),
            ("Bus Voltage", "bus_voltage", "Bus Current", "bus_current"),
            ("Torque Target", "torque_target", "Torque Estimate", "torque_estimate"),
            ("Electrical Power", "electrical_power", "Mechanical Power", "mechanical_power"),
        ]

        for row_index, (left_name, left_key, right_name, right_key) in enumerate(rows):
            form.addWidget(QLabel(left_name), row_index, 0)
            form.addWidget(self.telemetry_labels[left_key], row_index, 1)
            form.addWidget(QLabel(right_name), row_index, 2)
            form.addWidget(self.telemetry_labels[right_key], row_index, 3)

        return group

    def _build_log_group(self) -> QGroupBox:
        group = QGroupBox("Log")
        layout = QVBoxLayout(group)
        self.log_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout.addWidget(self.log_widget)
        return group

    def _append_log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_widget.appendPlainText(f"[{timestamp}] {message}")

    def _set_status(self, text: str) -> None:
        self.status_label.setText(text)
        self.statusBar().showMessage(text)

    def _parse_connection_values(self) -> tuple[int, int, int, float]:
        channel = int(self.channel_edit.text().strip(), 0)
        kbps = int(self.kbps_edit.text().strip(), 0)
        node_id = int(self.node_id_edit.text().strip(), 0)
        timeout_s = float(self.timeout_edit.text().strip())
        return channel, kbps, node_id, timeout_s

    def _refresh_interval_ms(self) -> int:
        try:
            return max(100, int(float(self.refresh_ms_edit.text().strip())))
        except ValueError:
            return 750

    def _require_client(self) -> ODriveGinkgoClient:
        if self.client is None:
            raise RuntimeError("Not connected to the Ginkgo adapter.")
        return self.client

    def _connect(self) -> None:
        try:
            if self.client is not None:
                self._append_log("Already connected.")
                return

            channel, kbps, node_id, _timeout_s = self._parse_connection_values()
            self.client = ODriveGinkgoClient(node_id=node_id, channel=channel, kbps=kbps)
            self.client.open()
        except Exception as exc:
            self.client = None
            self._show_error("Connection error", str(exc))
            self._set_status("Connection failed")
            return

        self._set_status(f"Connected to node 0x{self.client.node_id:02X}")
        self._append_log(
            f"Connected on channel={self.client.channel}, bitrate={self.client.kbps} kbps, "
            f"node=0x{self.client.node_id:02X}"
        )
        self.refresh_timer.start(self._refresh_interval_ms())
        self._refresh_telemetry(log_result=True)

    def _disconnect(self) -> None:
        if self.client is None:
            self._append_log("Disconnect requested while already disconnected.")
            return

        try:
            self.client.close()
        finally:
            self.client = None
            self._set_status("Disconnected")
            self._append_log("Disconnected from Ginkgo adapter.")

    def _clear_errors(self) -> None:
        try:
            self._require_client().clear_errors()
            self._append_log("Sent Clear_Errors.")
        except Exception as exc:
            self._handle_error("Clear_Errors failed", exc)

    def _enter_closed_loop(self) -> None:
        try:
            client = self._require_client()
            target = float(self.target_edit.text().strip())
            vel_ff = int(self.vel_ff_edit.text().strip(), 0)
            torque_ff = int(self.torque_ff_edit.text().strip(), 0)
            client.set_input_pos(target, vel_ff=vel_ff, torque_ff=torque_ff)
            time.sleep(0.05)
            client.enter_closed_loop()
            self._append_log(f"Entered closed loop with primed target {target:.4f} turns.")
        except Exception as exc:
            self._handle_error("Closed loop request failed", exc)

    def _idle(self) -> None:
        try:
            self._require_client().idle()
            self._append_log("Axis set to IDLE.")
        except Exception as exc:
            self._handle_error("Idle request failed", exc)

    def _move_to_target(self) -> None:
        try:
            target = float(self.target_edit.text().strip())
            self._move_to(target)
        except ValueError as exc:
            self._handle_error("Invalid target value", exc)

    def _move_to(self, target: float) -> None:
        try:
            client = self._require_client()
            vel_ff = int(self.vel_ff_edit.text().strip(), 0)
            torque_ff = int(self.torque_ff_edit.text().strip(), 0)
            client.set_input_pos(target, vel_ff=vel_ff, torque_ff=torque_ff)
            self.target_edit.setText(f"{target:.4f}")
            self._append_log(
                f"Sent position target {target:.4f} turns "
                f"(vel_ff={vel_ff}, torque_ff={torque_ff})."
            )
        except Exception as exc:
            self._handle_error("Position command failed", exc)

    def _offset_target(self, delta: float) -> None:
        try:
            current = float(self.target_edit.text().strip())
            self._move_to(current + delta)
        except ValueError as exc:
            self._handle_error("Invalid target value", exc)

    def _read_encoder_only(self) -> None:
        try:
            client = self._require_client()
            _, _, _, timeout_s = self._parse_connection_values()
            position_turns, velocity_turns_s = client.read_encoder_estimates(timeout_s=timeout_s)
            self.telemetry_labels["position"].setText(_format_metric(position_turns, "turns", 4))
            self.telemetry_labels["velocity"].setText(_format_metric(velocity_turns_s, "turns/s", 4))
            self._append_log(
                f"Encoder read: pos={position_turns:.4f} turns, "
                f"vel={velocity_turns_s:.4f} turns/s"
            )
        except Exception as exc:
            self._handle_error("Encoder read failed", exc)

    def _refresh_telemetry(self, *, log_result: bool = False) -> None:
        if self.client is None:
            return

        try:
            _, _, _, timeout_s = self._parse_connection_values()
            snapshot = self.client.read_telemetry(timeout_s=timeout_s, best_effort=True)
            self._apply_snapshot(snapshot)
            if log_result:
                self._append_log("Telemetry refreshed.")
        except Exception as exc:
            self._handle_error("Telemetry refresh failed", exc)

    def _apply_snapshot(self, snapshot: TelemetrySnapshot) -> None:
        self.telemetry_labels["position"].setText(_format_metric(snapshot.position_turns, "turns", 4))
        self.telemetry_labels["velocity"].setText(
            _format_metric(snapshot.velocity_turns_s, "turns/s", 4)
        )
        self.telemetry_labels["iq_setpoint"].setText(_format_metric(snapshot.iq_setpoint_a, "A"))
        self.telemetry_labels["iq_measured"].setText(_format_metric(snapshot.iq_measured_a, "A"))
        self.telemetry_labels["fet_temp"].setText(_format_metric(snapshot.fet_temp_c, "C"))
        self.telemetry_labels["motor_temp"].setText(_format_metric(snapshot.motor_temp_c, "C"))
        self.telemetry_labels["bus_voltage"].setText(_format_metric(snapshot.bus_voltage_v, "V"))
        self.telemetry_labels["bus_current"].setText(_format_metric(snapshot.bus_current_a, "A"))
        self.telemetry_labels["torque_target"].setText(
            _format_metric(snapshot.torque_target_nm, "Nm")
        )
        self.telemetry_labels["torque_estimate"].setText(
            _format_metric(snapshot.torque_estimate_nm, "Nm")
        )
        self.telemetry_labels["electrical_power"].setText(
            _format_metric(snapshot.electrical_power_w, "W")
        )
        self.telemetry_labels["mechanical_power"].setText(
            _format_metric(snapshot.mechanical_power_w, "W")
        )

    def _auto_refresh_tick(self) -> None:
        self.refresh_timer.start(self._refresh_interval_ms())
        if self.auto_refresh_checkbox.isChecked() and self.client is not None:
            self._refresh_telemetry(log_result=False)

    def _handle_error(self, title: str, exc: Exception) -> None:
        self._set_status(title)
        self._append_log(f"{title}: {exc}")

    def _show_error(self, title: str, text: str) -> None:
        dialog = QMessageBox(self)
        dialog.setIcon(QMessageBox.Icon.Critical)
        dialog.setWindowTitle(title)
        dialog.setText(text)
        dialog.exec()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    window = MotorTesterWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
