import sys
import random
import datetime
import subprocess
import os
import socket
import json
import threading
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QPushButton,
                             QVBoxLayout, QHBoxLayout, QDialog, QFrame,
                             QStackedWidget, QGridLayout, QProgressBar, QGraphicsOpacityEffect,
                             QButtonGroup)
from PyQt5.QtGui import (QPixmap, QFont, QPainter, QPainterPath, QLinearGradient,
                         QColor, QIcon, QBrush, QPen, QPolygonF)
from PyQt5.QtCore import (Qt, QTimer, QPropertyAnimation, QEasingCurve, pyqtProperty,
                          QPointF, QSequentialAnimationGroup, QObject, pyqtSignal)

def get_destination_number(destination_name):
    destination_mapping = {
        "백화점 본관 입구": 0,
        "영화관 입구": 1,
        "문화시설 입구": 2
    }
    return destination_mapping.get(destination_name, 0)

def get_destination_name(destination_number):
    number_mapping = {
        0: "백화점 본관 입구",
        1: "영화관 입구", 
        2: "문화시설 입구"
    }
    return number_mapping.get(destination_number, "백화점 본관 입구")

WIFI_CONFIG = {
    'port': 7777
}

class WifiSender(QObject):
    send_finished = pyqtSignal()
    send_error = pyqtSignal(str)

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        print(f"📡 WifiSender 초기화 -> 대상: {self.host}:{self.port}")

    def send_data(self, data):
        thread = threading.Thread(target=self._send_in_background, args=(data,))
        thread.daemon = True
        thread.start()

    def _send_in_background(self, data):
        error_message = None
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                print(f"연결 시도 중... -> {self.host}:{self.port}")
                s.connect((self.host, self.port))
                data['timestamp'] = datetime.datetime.now().isoformat()
                message = json.dumps(data)

                print("\n" + "="*50)
                print("📩 전송할 데이터:")
                print(json.dumps(data, indent=2, ensure_ascii=False))
                print("="*50 + "\n")

                s.sendall(message.encode('utf-8'))
                print(f"🚀 데이터 전송 성공: {message}")
                response = s.recv(1024)
                print(f"📬 서버 응답: {response.decode('utf-8')}")

        except socket.timeout:
            error_message = f"❌ 전송 실패: 연결 시간 초과. {self.host} 기기가 켜져 있고 같은 네트워크에 있는지 확인하세요."
            print(error_message)
        except ConnectionRefusedError:
            error_message = "❌ 전송 실패: 연결이 거부되었습니다. 대상 기기에서 수신 프로그램이 실행 중인지 확인하세요."
            print(error_message)
        except Exception as e:
            error_message = f"❌ 전송 중 알 수 없는 오류 발생: {e}"
            print(error_message)
        finally:
            if error_message:
                self.send_error.emit(error_message)
            else:
                self.send_finished.emit()


HYUNDAI_COLORS = {
    'primary': '#002C5F',
    'secondary': '#007FA3',
    'accent': '#00AAD2',
    'success': '#00C851',
    'warning': '#FFB300',
    'background': '#0A0E1A',
    'surface': '#1A1E2E',
    'text_primary': '#FFFFFF',
    'text_secondary': '#B0BEC5',
    'glass': 'rgba(255, 255, 255, 0.1)'
}

FONT_SIZES = {
    'status_bar_location': 12,
    'status_bar_date': 11,
    'status_bar_time': 28,
    'status_bar_weather': 12,
    'status_bar_radio': 11,
    'main_title': 32,
    'main_subtitle': 16,
    'button': 16,
    'toggle_button': 14,
    'scenario_title': 26,
    'scenario_subtitle': 16,
    'scenario_info': 14,
    'scanner_text': 18,
    'timer': 14,
    'progress_bar': 12,
    'transition_text': 28,
}

class HyundaiBackground(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setAutoFillBackground(True)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        gradient = QLinearGradient(0, 0, w, h)
        gradient.setColorAt(0, QColor('#0A0E1A'))
        gradient.setColorAt(0.5, QColor('#1A1E2E'))
        gradient.setColorAt(1, QColor('#0F1419'))
        painter.fillRect(self.rect(), QBrush(gradient))
        painter.setPen(QPen(QColor(0, 170, 210, 30), 1))
        for i in range(0, w, 50): painter.drawLine(i, 0, i, h)
        for i in range(0, h, 50): painter.drawLine(0, i, w, i)
        painter.setPen(QPen(QColor(0, 170, 210, 80), 2))
        painter.setBrush(QBrush(QColor(0, 170, 210, 20)))
        painter.drawEllipse(QPointF(w * 0.15, h * 0.15), w * 0.12, h * 0.1)
        painter.drawEllipse(QPointF(w * 0.27, h * 0.25), w * 0.1, h * 0.08)
        path = QPainterPath()
        path.moveTo(w * 0.7, h)
        path.quadTo(w * 0.9, h * 0.7, w, h * 0.85)
        path.lineTo(w, h)
        path.closeSubpath()
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 123, 163, 40)))
        painter.drawPath(path)

class StatusBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.initUI()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)
        self.update_time()

    def initUI(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(30, 15, 30, 15)
        location_layout = QHBoxLayout()
        location_icon = QLabel("📍")
        location_icon.setStyleSheet(f"font-size: {FONT_SIZES['status_bar_location']}pt; color: white;")
        self.location_label = QLabel("Seocho-gu, Seoul")
        self.location_label.setStyleSheet(f"color: {HYUNDAI_COLORS['text_primary']}; font-size: {FONT_SIZES['status_bar_location']}pt; font-weight: bold;")
        location_layout.addWidget(location_icon)
        location_layout.addWidget(self.location_label)
        location_layout.addStretch()
        time_layout = QVBoxLayout()
        time_layout.setSpacing(0)
        self.date_label = QLabel()
        self.date_label.setAlignment(Qt.AlignCenter)
        self.date_label.setStyleSheet(f"color: {HYUNDAI_COLORS['text_secondary']}; font-size: {FONT_SIZES['status_bar_date']}pt;")
        self.time_label = QLabel()
        self.time_label.setAlignment(Qt.AlignCenter)
        self.time_label.setStyleSheet(f"color: {HYUNDAI_COLORS['text_primary']}; font-size: {FONT_SIZES['status_bar_time']}pt; font-weight: bold;")
        time_layout.addWidget(self.date_label)
        time_layout.addWidget(self.time_label)
        right_layout = QVBoxLayout()
        right_layout.setSpacing(5)
        weather_layout = QHBoxLayout()
        weather_layout.addStretch()
        temp_label = QLabel("26°C")
        temp_label.setStyleSheet(f"color: {HYUNDAI_COLORS['text_primary']}; font-size: {FONT_SIZES['status_bar_weather']}pt; font-weight: bold;")
        weather_icon = QLabel("🌙")
        weather_icon.setStyleSheet(f"font-size: {FONT_SIZES['status_bar_weather']}pt;")
        weather_layout.addWidget(temp_label)
        weather_layout.addWidget(weather_icon)
        radio_layout = QHBoxLayout()
        radio_layout.addStretch()
        radio_label = QLabel("FM 87.5")
        radio_label.setStyleSheet(f"color: {HYUNDAI_COLORS['text_secondary']}; font-size: {FONT_SIZES['status_bar_radio']}pt;")
        radio_layout.addWidget(radio_label)
        right_layout.addLayout(weather_layout)
        right_layout.addLayout(radio_layout)
        layout.addLayout(location_layout, 1)
        layout.addLayout(time_layout, 2)
        layout.addLayout(right_layout, 1)
        self.setLayout(layout)
        self.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 rgba(26, 30, 46, 0.9), stop:1 rgba(10, 14, 26, 0.7)); border-bottom: 1px solid rgba(0, 170, 210, 0.3);")

    def update_time(self):
        now = datetime.datetime.now()
        weekdays = ['월', '화', '수', '목', '금', '토', '일']
        date_str = f"{now.month}월 {now.day}일 ({weekdays[now.weekday()]})"
        self.date_label.setText(date_str)
        time_str = now.strftime("%H:%M")
        self.time_label.setText(time_str)

class AnimatedButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setMinimumHeight(70)
        self.setCursor(Qt.PointingHandCursor)
        self.default_style = f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(0, 44, 95, 0.8), stop:1 rgba(0, 127, 163, 0.8));
                color: white; border: 2px solid rgba(0, 170, 210, 0.5);
                border-radius: 25px; font-size: {FONT_SIZES['button']}pt;
                font-weight: bold; padding: 15px 30px;
            }}
            QPushButton:disabled {{
                background: rgba(40, 50, 70, 0.8);
                color: rgba(255, 255, 255, 0.4);
                border: 2px solid rgba(0, 170, 210, 0.2);
            }}
        """
        self.hover_style = f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(0, 170, 210, 0.9), stop:1 rgba(0, 127, 163, 0.9));
                color: white; border: 2px solid rgba(0, 170, 210, 0.8);
                border-radius: 25px; font-size: {FONT_SIZES['button']}pt;
                font-weight: bold; padding: 15px 30px;
            }}
        """
        self.setStyleSheet(self.default_style)

    def enterEvent(self, event):
        if self.isEnabled():
            self.setStyleSheet(self.hover_style)
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.setStyleSheet(self.default_style)
        super().leaveEvent(event)

class ToggleButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setCheckable(True)
        self.setMinimumHeight(60)
        self.setCursor(Qt.PointingHandCursor)
        self.setStyleSheet(f"""
            QPushButton {{
                background: rgba(0, 44, 95, 0.7);
                color: {HYUNDAI_COLORS['text_secondary']};
                border: 2px solid rgba(0, 170, 210, 0.4);
                border-radius: 15px;
                font-size: {FONT_SIZES['toggle_button']}pt;
                font-weight: bold;
                padding: 10px;
            }}
            QPushButton:hover {{
                background: rgba(0, 127, 163, 0.7);
            }}
            QPushButton:checked {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 {HYUNDAI_COLORS['accent']}, stop:1 {HYUNDAI_COLORS['secondary']});
                color: white;
                border: 2px solid {HYUNDAI_COLORS['accent']};
            }}
        """)


class BaseScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.background = HyundaiBackground(self)
        self.main_widget = QWidget(self)
        self.main_widget.setStyleSheet("background: transparent;")
        self.content_layout = QVBoxLayout(self.main_widget)
        self.content_layout.setSpacing(20)
        self.content_layout.setContentsMargins(80, 40, 80, 40)
        full_layout = QVBoxLayout(self)
        full_layout.setContentsMargins(0, 0, 0, 0)
        full_layout.addWidget(self.main_widget)
        self.setLayout(full_layout)

    def resizeEvent(self, event):
        self.background.resize(event.size())
        super().resizeEvent(event)

class SimulationSetupScreen(BaseScreen):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.vehicle_type = None
        self.is_handicapped = None
        self.initUI()

    def initUI(self):
        title = QLabel("Smart Parking System")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: {FONT_SIZES['main_title']}pt; font-weight: bold; color: {HYUNDAI_COLORS['text_primary']}; text-shadow: 2px 2px 4px rgba(0,0,0,0.3);")

        subtitle = QLabel("시뮬레이션 설정을 선택해주세요")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(f"font-size: {FONT_SIZES['main_subtitle']}pt; color: {HYUNDAI_COLORS['text_secondary']};")

        vehicle_label = QLabel("1. 차량 유형")
        vehicle_label.setStyleSheet(f"font-size: {FONT_SIZES['scenario_subtitle']}pt; color: {HYUNDAI_COLORS['text_primary']}; margin-top: 15px; margin-bottom: 5px;")

        self.vehicle_btn_group = QButtonGroup(self)
        self.vehicle_btn_group.setExclusive(True)

        vehicle_buttons_layout = QHBoxLayout()
        vehicle_buttons_layout.setSpacing(20)
        self.regular_car_btn = ToggleButton("🚗 일반 차량")
        self.ev_car_btn = ToggleButton("🔋 전기차")
        self.vehicle_btn_group.addButton(self.regular_car_btn)
        self.vehicle_btn_group.addButton(self.ev_car_btn)
        vehicle_buttons_layout.addWidget(self.regular_car_btn)
        vehicle_buttons_layout.addWidget(self.ev_car_btn)

        handicap_label = QLabel("2. 장애인 차량 여부")
        handicap_label.setStyleSheet(f"font-size: {FONT_SIZES['scenario_subtitle']}pt; color: {HYUNDAI_COLORS['text_primary']}; margin-top: 15px; margin-bottom: 5px;")

        self.handicap_btn_group = QButtonGroup(self)
        self.handicap_btn_group.setExclusive(True)

        handicap_buttons_layout = QHBoxLayout()
        handicap_buttons_layout.setSpacing(20)
        self.handicapped_btn = ToggleButton("♿ 예")
        self.non_handicapped_btn = ToggleButton("🅿️ 아니오")
        self.handicap_btn_group.addButton(self.handicapped_btn)
        self.handicap_btn_group.addButton(self.non_handicapped_btn)
        handicap_buttons_layout.addWidget(self.handicapped_btn)
        handicap_buttons_layout.addWidget(self.non_handicapped_btn)

        self.start_btn = AnimatedButton("시뮬레이션 시작")
        self.start_btn.clicked.connect(self.start_simulation)
        self.start_btn.setEnabled(False)

        self.vehicle_btn_group.buttonClicked.connect(self.check_selections)
        self.handicap_btn_group.buttonClicked.connect(self.check_selections)

        self.content_layout.addStretch(2)
        self.content_layout.addWidget(title)
        self.content_layout.addWidget(subtitle)
        self.content_layout.addSpacing(20)
        self.content_layout.addWidget(vehicle_label)
        self.content_layout.addLayout(vehicle_buttons_layout)
        self.content_layout.addSpacing(20)
        self.content_layout.addWidget(handicap_label)
        self.content_layout.addLayout(handicap_buttons_layout)
        self.content_layout.addStretch(3)
        self.content_layout.addWidget(self.start_btn)
        self.content_layout.addStretch(1)

    def check_selections(self):
        if (self.vehicle_btn_group.checkedButton() and 
            self.handicap_btn_group.checkedButton()):
            self.start_btn.setEnabled(True)
        else:
            self.start_btn.setEnabled(False)

    def start_simulation(self):
        if self.regular_car_btn.isChecked():
            self.vehicle_type = 'regular'
        elif self.ev_car_btn.isChecked():
            self.vehicle_type = 'electric'

        if self.handicapped_btn.isChecked():
            self.is_handicapped = True
        elif self.non_handicapped_btn.isChecked():
            self.is_handicapped = False

        if self.vehicle_type is not None and self.is_handicapped is not None:
            if hasattr(self.parent_window, 'show_transition'):
                self.parent_window.show_transition(self.vehicle_type, self.is_handicapped)

class TransitionScreen(BaseScreen):
    def __init__(self, vehicle_type, is_handicapped, parent=None):
        super().__init__(parent)
        self.vehicle_type = vehicle_type
        self.is_handicapped = is_handicapped
        self.initUI()
        self.start_animation()

    def initUI(self):
        self.message_label = QLabel("시뮬레이션을 시작합니다")
        self.message_label.setAlignment(Qt.AlignCenter)
        self.message_label.setStyleSheet(f"font-size: {FONT_SIZES['transition_text']}pt; font-weight: bold; color: {HYUNDAI_COLORS['text_primary']};")
        self.content_layout.addStretch(1)
        self.content_layout.addWidget(self.message_label)
        self.content_layout.addStretch(1)
        self.opacity_effect = QGraphicsOpacityEffect(self)
        self.message_label.setGraphicsEffect(self.opacity_effect)
        self.opacity_effect.setOpacity(0.0)

    def start_animation(self):
        fade_in = QPropertyAnimation(self.opacity_effect, b"opacity")
        fade_in.setDuration(1000); fade_in.setStartValue(0.0); fade_in.setEndValue(1.0)
        fade_out = QPropertyAnimation(self.opacity_effect, b"opacity")
        fade_out.setDuration(1000); fade_out.setStartValue(1.0); fade_out.setEndValue(0.0)
        self.anim_group = QSequentialAnimationGroup(self)
        self.anim_group.addAnimation(fade_in)
        self.anim_group.addPause(1000)
        self.anim_group.addAnimation(fade_out)
        self.anim_group.finished.connect(self.on_animation_finished)
        self.anim_group.start()

    def on_animation_finished(self):
        if hasattr(self.parent_window, 'show_scenario'):
            self.parent_window.show_scenario(self.vehicle_type, self.is_handicapped)

class DestinationSelectionScreen(BaseScreen):
    def __init__(self, vehicle_type, is_handicapped, parent=None, preferred_spot=None):
        super().__init__(parent)
        self.vehicle_type = vehicle_type
        self.is_handicapped = is_handicapped
        self.preferred_spot = preferred_spot
        self.initUI()

    def initUI(self):
        title = QLabel("목적지 선택")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: {FONT_SIZES['main_title']}pt; font-weight: bold; color: {HYUNDAI_COLORS['text_primary']}; text-shadow: 2px 2px 4px rgba(0,0,0,0.3);")

        subtitle = QLabel("어디로 가시나요?")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(f"font-size: {FONT_SIZES['main_subtitle']}pt; color: {HYUNDAI_COLORS['text_secondary']};")

        self.destination_btn_group = QButtonGroup(self)
        self.destination_btn_group.setExclusive(True)

        destination_buttons_layout = QVBoxLayout()
        destination_buttons_layout.setSpacing(20)
        
        self.beauty_btn = AnimatedButton("🏬 백화점 본관 입구")
        self.mart_btn = AnimatedButton("🎬 영화관 입구")
        self.restaurant_btn = AnimatedButton("🎨 문화시설 입구")
        
        self.destination_btn_group.addButton(self.beauty_btn)
        self.destination_btn_group.addButton(self.mart_btn)
        self.destination_btn_group.addButton(self.restaurant_btn)
        
        destination_buttons_layout.addWidget(self.beauty_btn)
        destination_buttons_layout.addWidget(self.mart_btn)
        destination_buttons_layout.addWidget(self.restaurant_btn)

        self.beauty_btn.clicked.connect(lambda: self.select_destination('백화점 본관 입구'))
        self.mart_btn.clicked.connect(lambda: self.select_destination('영화관 입구'))
        self.restaurant_btn.clicked.connect(lambda: self.select_destination('문화시설 입구'))

        self.content_layout.addStretch(1)
        self.content_layout.addWidget(title)
        self.content_layout.addWidget(subtitle)
        self.content_layout.addSpacing(40)
        self.content_layout.addLayout(destination_buttons_layout)
        self.content_layout.addStretch(2)

    def select_destination(self, destination):
        if hasattr(self.parent_window, 'send_final_choice'):
            destination_number = get_destination_number(destination)
            if self.preferred_spot:
                preferred_spot = self.preferred_spot
            else:
                preferred_spot = 'electric' if self.vehicle_type == 'electric' else 'regular'
            self.parent_window.send_final_choice(
                self.vehicle_type,
                self.is_handicapped,
                destination_number,
                preferred_spot
            )

class FingerprintAuthentication(BaseScreen):
    def __init__(self, vehicle_type, is_handicapped, fallback_scenario, parent=None):
        super().__init__(parent)
        self.vehicle_type = vehicle_type
        self.is_handicapped = is_handicapped
        self.fallback_scenario = fallback_scenario
        self.authentication_timer = None
        self.initUI()
        self.start_authentication()

    def initUI(self):
        fingerprint_label = QLabel("👆"); fingerprint_label.setAlignment(Qt.AlignCenter); fingerprint_label.setStyleSheet("font-size: 100pt; margin-bottom: 20px;")
        message = QLabel("장애인 주차구역 이용 안내"); message.setAlignment(Qt.AlignCenter); message.setStyleSheet(f"font-size: {FONT_SIZES['scenario_title']}pt; color: {HYUNDAI_COLORS['text_primary']}; font-weight: bold;")
        fingerprint_info = QLabel("본인 확인을 위해 지문 인식을 진행해주세요"); fingerprint_info.setAlignment(Qt.AlignCenter); fingerprint_info.setStyleSheet(f"font-size: {FONT_SIZES['scenario_subtitle']}pt; color: {HYUNDAI_COLORS['text_secondary']};")
        self.timer_label = QLabel("5초 후 일반 구역으로 자동 배정됩니다"); self.timer_label.setAlignment(Qt.AlignCenter); self.timer_label.setStyleSheet(f"font-size: {FONT_SIZES['timer']}pt; color: {HYUNDAI_COLORS['warning']};")
        fingerprint_scanner = QFrame(); fingerprint_scanner.setMinimumHeight(140); fingerprint_scanner.setStyleSheet("border: 3px solid rgba(0, 170, 210, 0.6); border-radius: 25px; background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 rgba(26, 30, 46, 0.9), stop:1 rgba(10, 14, 26, 0.7)); margin: 20px;")
        scanner_layout = QVBoxLayout(); scanner_text = QLabel("지문을 스캐너에 올려주세요"); scanner_text.setAlignment(Qt.AlignCenter); scanner_text.setStyleSheet(f"font-size: {FONT_SIZES['scanner_text']}pt; color: {HYUNDAI_COLORS['text_primary']}; font-weight: bold;"); scanner_layout.addWidget(scanner_text); fingerprint_scanner.setLayout(scanner_layout)
        button_layout = QHBoxLayout(); button_layout.setSpacing(20); success_btn = AnimatedButton("인증 성공"); success_btn.clicked.connect(self.authentication_success); fallback_btn = AnimatedButton("일반 구역으로"); fallback_btn.clicked.connect(self.authentication_timeout); button_layout.addWidget(success_btn); button_layout.addWidget(fallback_btn)
        self.content_layout.addStretch(1); self.content_layout.addWidget(fingerprint_label); self.content_layout.addWidget(message); self.content_layout.addWidget(fingerprint_info); self.content_layout.addWidget(self.timer_label); self.content_layout.addWidget(fingerprint_scanner); self.content_layout.addLayout(button_layout); self.content_layout.addStretch(1)

    def start_authentication(self):
        self.remaining_time = 5
        self.authentication_timer = QTimer(self)
        self.authentication_timer.timeout.connect(self.update_timer)
        self.authentication_timer.start(1000)

    def update_timer(self):
        self.remaining_time -= 1
        if self.remaining_time > 0:
            self.timer_label.setText(f"{self.remaining_time}초 후 일반 구역으로 자동 배정됩니다")
        else:
            self.authentication_timer.stop()
            self.authentication_timeout()

    def authentication_success(self):
        if self.authentication_timer: self.authentication_timer.stop()
        self.send_final_choice('disabled')

    def authentication_timeout(self):
        if self.authentication_timer: self.authentication_timer.stop()
        self.send_final_choice(self.fallback_scenario)

    def send_final_choice(self, preferred_spot):
        self.preferred_spot = preferred_spot
        if hasattr(self.parent_window, 'show_destination_selection'):
            self.parent_window.show_destination_selection(self.vehicle_type, self.is_handicapped, preferred_spot)

class ElectricVehicleOptions(BaseScreen):
    def __init__(self, vehicle_type, is_handicapped, parent=None):
        super().__init__(parent)
        self.vehicle_type = vehicle_type
        self.is_handicapped = is_handicapped
        self.initUI()

    def initUI(self):
        icon_label = QLabel("🔋"); icon_label.setAlignment(Qt.AlignCenter); icon_label.setStyleSheet("font-size: 80pt; margin-bottom: 20px;")
        message = QLabel("전기차 옵션 선택"); message.setAlignment(Qt.AlignCenter); message.setStyleSheet(f"font-size: {FONT_SIZES['scenario_title']}pt; color: {HYUNDAI_COLORS['text_primary']}; font-weight: bold;")
        option_info = QLabel("원하시는 주차구역을 선택해주세요"); option_info.setAlignment(Qt.AlignCenter); option_info.setStyleSheet(f"font-size: {FONT_SIZES['scenario_subtitle']}pt; color: {HYUNDAI_COLORS['text_secondary']};")

        button_layout = QVBoxLayout(); button_layout.setSpacing(20)
        normal_btn = AnimatedButton("🅿️ 일반 주차구역"); normal_btn.clicked.connect(self.select_normal_parking); button_layout.addWidget(normal_btn)
        charging_btn = AnimatedButton("⚡ 전기차 충전구역"); charging_btn.clicked.connect(self.select_charging); button_layout.addWidget(charging_btn)

        if self.is_handicapped:
            handicapped_btn = AnimatedButton("♿ 장애인 전용 주차구역"); handicapped_btn.clicked.connect(self.select_handicapped_parking); button_layout.addWidget(handicapped_btn)

        self.content_layout.addStretch(1); self.content_layout.addWidget(icon_label); self.content_layout.addWidget(message); self.content_layout.addWidget(option_info); self.content_layout.addSpacing(30); self.content_layout.addLayout(button_layout); self.content_layout.addStretch(1)

    def select_charging(self):
        self.send_final_choice('electric')

    def select_normal_parking(self):
        self.send_final_choice('regular')

    def select_handicapped_parking(self):
        if hasattr(self.parent_window, 'show_fingerprint_auth'):
            self.parent_window.show_fingerprint_auth(self.vehicle_type, self.is_handicapped, 'regular')

    def send_final_choice(self, preferred_spot):
        if hasattr(self.parent_window, 'show_destination_selection'):
            self.parent_window.show_destination_selection(self.vehicle_type, self.is_handicapped, preferred_spot)

class RegularVehicleResult(BaseScreen):
    def __init__(self, vehicle_type, is_handicapped, parent=None):
        super().__init__(parent)
        self.vehicle_type = vehicle_type
        self.is_handicapped = is_handicapped
        self.initUI()

    def initUI(self):
        success_label = QLabel("✅"); success_label.setAlignment(Qt.AlignCenter); success_label.setStyleSheet("font-size: 80pt; margin-bottom: 20px;")
        message = QLabel("일반 주차구역 배정 완료"); message.setAlignment(Qt.AlignCenter); message.setStyleSheet(f"font-size: {FONT_SIZES['scenario_title']}pt; color: {HYUNDAI_COLORS['text_primary']}; font-weight: bold;")
        info = QLabel("주차장 입구가 곧 열립니다"); info.setAlignment(Qt.AlignCenter); info.setStyleSheet(f"font-size: {FONT_SIZES['scenario_subtitle']}pt; color: {HYUNDAI_COLORS['text_secondary']};")
        confirm_btn = AnimatedButton("확인"); confirm_btn.clicked.connect(self.confirm_and_launch)
        self.content_layout.addStretch(1); self.content_layout.addWidget(success_label); self.content_layout.addWidget(message); self.content_layout.addWidget(info); self.content_layout.addSpacing(30); self.content_layout.addWidget(confirm_btn); self.content_layout.addStretch(1)

    def confirm_and_launch(self):
        if hasattr(self.parent_window, 'show_destination_selection'):
            self.parent_window.show_destination_selection(self.vehicle_type, self.is_handicapped, 'regular')

class HyundaiStyleUI(QWidget):
    def __init__(self, vehicle_ip=None):
        super().__init__()
        if not vehicle_ip:
            print("⚠️ 경고: ESP32 IP 주소 없이 HyundaiStyleUI가 생성되었습니다. (단독 테스트용)")
            vehicle_ip = '127.0.0.1'
        else:
            print(f"🎯 ESP32 IP 주소 수신: {vehicle_ip}")

        self.wifi_sender = WifiSender(vehicle_ip, WIFI_CONFIG['port'])
        
        self.wifi_sender.send_finished.connect(self.launch_parking_ui)
        self.wifi_sender.send_error.connect(self.handle_send_error)
        
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Smart Parking System')
        self.resize(1280, 800)
        self.setMinimumSize(1000, 700)
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        self.status_bar = StatusBar()
        main_layout.addWidget(self.status_bar)
        self.stacked_widget = QStackedWidget()
        self.home_screen = SimulationSetupScreen(self)
        self.stacked_widget.addWidget(self.home_screen)
        main_layout.addWidget(self.stacked_widget)
        self.setLayout(main_layout)
        self.setStyleSheet(f"background-color: {HYUNDAI_COLORS['background']};")
        self.showMaximized()

    def send_final_choice(self, vehicle_type, is_handicapped, destination, preferred_spot):
        elec_val = "true" if vehicle_type == 'electric' else "false"
        disabled_val = "true" if is_handicapped else "false"
        
        preferred_map = {'regular': 'normal', 'electric': 'elec', 'disabled': 'disabled'}
        preferred_val = preferred_map.get(preferred_spot, 'normal')

        final_data = {
            "elec": elec_val, 
            "disabled": disabled_val, 
            "preferred": preferred_val, 
            "destination": destination
        }
        
        self.wifi_sender.send_data(final_data)

    def launch_parking_ui(self):
        try:
            script_name = 'UI_testing.py'
            print(f"\n✅ 전송 성공! 다음 UI 실행 시도: {script_name}")
            subprocess.Popen([sys.executable, script_name])
            QApplication.quit()
        except FileNotFoundError:
            print(f"❌ 실행 실패: {script_name} 파일을 찾을 수 없습니다.")
            self.show_home()
        except Exception as e:
            print(f"❌ 실행 실패: {script_name} 실행 중 오류 발생: {e}")
            self.show_home()

    def handle_send_error(self, error_message):
        print("데이터 전송 실패로 인해 홈 화면으로 돌아갑니다.")
        self.show_home()

    def show_transition(self, vehicle_type, is_handicapped):
        transition_screen = TransitionScreen(vehicle_type, is_handicapped, self)
        self.switch_screen(transition_screen)

    def show_scenario(self, vehicle_type, is_handicapped):
        if vehicle_type == 'regular':
            if is_handicapped:
                self.show_fingerprint_auth(vehicle_type, is_handicapped, 'regular')
            else:
                self.show_regular_result(vehicle_type, is_handicapped)
        elif vehicle_type == 'electric':
            self.show_electric_options(vehicle_type, is_handicapped)

    def show_fingerprint_auth(self, vehicle_type, is_handicapped, fallback_scenario):
        fingerprint_screen = FingerprintAuthentication(vehicle_type, is_handicapped, fallback_scenario, self)
        self.switch_screen(fingerprint_screen)

    def show_electric_options(self, vehicle_type, is_handicapped):
        electric_screen = ElectricVehicleOptions(vehicle_type, is_handicapped, self)
        self.switch_screen(electric_screen)

    def show_regular_result(self, vehicle_type, is_handicapped):
        result_screen = RegularVehicleResult(vehicle_type, is_handicapped, self)
        self.switch_screen(result_screen)

    def show_destination_selection(self, vehicle_type, is_handicapped, preferred_spot=None):
        destination_screen = DestinationSelectionScreen(vehicle_type, is_handicapped, self, preferred_spot)
        self.switch_screen(destination_screen)

    def switch_screen(self, new_screen):
        while self.stacked_widget.count() > 1:
            widget = self.stacked_widget.widget(1)
            self.stacked_widget.removeWidget(widget)
            widget.deleteLater()

        self.stacked_widget.addWidget(new_screen)
        self.stacked_widget.setCurrentWidget(new_screen)

    def show_home(self):
        while self.stacked_widget.count() > 1:
            widget_to_remove = self.stacked_widget.widget(1)
            self.stacked_widget.removeWidget(widget_to_remove)
            widget_to_remove.deleteLater()
        self.stacked_widget.setCurrentIndex(0)


if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)

    app = QApplication(sys.argv)

    font = QFont("Malgun Gothic")
    font.setPointSize(11)
    app.setFont(font)
    app.setStyle('Fusion')

    ex = HyundaiStyleUI()
    sys.exit(app.exec_())