import sys
import socket
import json
import threading
from heapq import heappush, heappop
from math import sqrt, atan2, degrees, sin, cos, radians
import random
from datetime import datetime
from typing import List, Tuple, Optional
from PyQt5.QtWidgets import (
    QApplication, QGraphicsScene, QGraphicsView, QGraphicsRectItem,
    QGraphicsSimpleTextItem, QGraphicsEllipseItem, QGraphicsPolygonItem,
    QPushButton, QWidget, QVBoxLayout, QHBoxLayout, QGraphicsItem,
    QLineEdit, QLabel, QMessageBox, QGraphicsItemGroup, QFrame, QGraphicsObject
)
from PyQt5.QtGui import (
    QBrush, QPainter, QPen, QColor, QPainterPath, QFont, QPolygonF,
    QLinearGradient, QRadialGradient, QTransform, QFontMetrics
)
from PyQt5.QtCore import (
    Qt, QPointF, QRectF, pyqtSignal, QTimer, QPropertyAnimation,
    pyqtProperty, QEasingCurve, QParallelAnimationGroup
)
class WaypointReceiver:
    def __init__(self, host='0.0.0.0', port=9999):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.waypoint_callback = None
        self.position_callback = None
        print(f"📡 Waypoint 및 위치 수신기 초기화됨. 수신 대기 주소: {self.host}:{self.port}")
    def set_waypoint_callback(self, callback_function):
        self.waypoint_callback = callback_function
    def set_position_callback(self, callback_function):
        self.position_callback = callback_function
    def start_receiver(self):
        def server_thread():
            try:
                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server_socket.bind((self.host, self.port))
                self.server_socket.listen(5)
                print(f"✅ 서버가 {self.host}:{self.port}에서 대기 중...")
                self.running = True
                while self.running:
                    try:
                        client_socket, addr = self.server_socket.accept()
                        print(f"🔗 클라이언트 연결됨: {addr}")
                        self.handle_connection(client_socket)
                    except Exception as e:
                        if self.running:
                            print(f"❌ 연결 오류: {e}")
                        break
            except Exception as e:
                print(f"❌ 서버 시작 오류: {e}")
        threading.Thread(target=server_thread, daemon=True).start()
    def handle_connection(self, client_socket):
        try:
            while self.running:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                try:
                    for chunk in data.strip().split('}{'):
                        if not chunk.startswith('{'): chunk = '{' + chunk
                        if not chunk.endswith('}'): chunk = chunk + '}'
                        message = json.loads(chunk)
                        self.process_waypoint_data(message)
                        response = {"status": "received", "timestamp": datetime.now().isoformat()}
                        client_socket.send(json.dumps(response).encode('utf-8'))
                except json.JSONDecodeError:
                    print(f"❌ 잘못된 JSON 데이터: {data}")
        except Exception as e:
            print(f"❌ 데이터 수신 오류: {e}")
        finally:
            client_socket.close()
            print("📱 클라이언트 연결 종료")
    def process_waypoint_data(self, data):
        msg_type = data.get('type')
        if msg_type == 'waypoint_assignment':
            waypoints = data.get('waypoints', [])
            print(f"\n🎯 새로운 waypoint 수신: {waypoints}")
            if self.waypoint_callback:
                self.waypoint_callback(waypoints)
            print("=" * 50)
        elif msg_type == 'real_time_position':
            x = data.get('x')
            y = data.get('y')
            tag_id = data.get('tag_id')
            print(f"📍 실시간 위치 수신 - Tag {tag_id}: ({x}, {y})")
            if x is not None and y is not None:
                position = [float(x), float(y)]
                if self.position_callback:
                    self.position_callback(position)
            else:
                print(f"❌ 잘못된 위치 데이터: x={x}, y={y}")
    def stop(self):
        print("🛑 Waypoint 수신기를 종료합니다...")
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.server_socket.close()
HYUNDAI_COLORS = {
    'primary': '#1a1a1a',
    'secondary': "#2d2d2d",
    'accent': '#4a9eff',
    'success': '#00d084',
    'warning': '#ffa726',
    'danger': '#ef5350',
    'background': '#0f0f0f',
    'surface': '#1e1e1e',
    'text_primary': '#ffffff',
    'text_secondary': '#9e9e9e',
    'glass': 'rgba(255, 255, 255, 0.08)',
    'blue_soft': '#6bb6ff',
    'blue_muted': '#4285f4',
    'white_soft': '#f5f5f5',
    'gray_light': '#757575',
    'gray_medium': '#424242'
}
FONT_SIZES = {
    'hud_distance': 42, 'hud_direction': 12, 'hud_speed': 28, 'hud_speed_unit': 10,
    'hud_progress': 14, 'hud_next_label': 10, 'hud_next_direction': 14,
    'map_label': 10, 'map_io_label': 12, 'map_waypoint_label': 12,
    'controls_title': 16, 'controls_info': 12, 'controls_button': 16, 'msgbox_button': 10
}
class PremiumHudWidget(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.NoFrame)
        self.setMinimumSize(450, 700)
        self.setStyleSheet(f"""
            PremiumHudWidget {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 {HYUNDAI_COLORS['primary']}, 
                    stop:1 {HYUNDAI_COLORS['background']});
                border: 2px solid {HYUNDAI_COLORS['gray_medium']};
                border-radius: 25px;
            }}
        """)
        self.current_direction = "경로 설정 대기"
        self.current_distance = 0.0
        self.next_direction = ""
        self.speed = 0
        self.progress = 0
        self.animation_timer = QTimer(self)
        self.animation_timer.timeout.connect(self.update_animation)
        self.animation_timer.start(50)
        self.rotation_angle = 0
        self.pulse_scale = 1.0
        self.pulse_growing = True
        self.glow_opacity = 0.3
        self.glow_increasing = True
        self.particle_positions = []
        self.init_particles()
        self.direction_transition = 0.0
        self.target_direction = "직진"
        self.previous_direction = "직진"
        self.exit_scenario_button = QPushButton("출차 시나리오 시작", self)
        self.exit_scenario_button.setGeometry(50, 650, 350, 40)
        self.exit_scenario_button.setStyleSheet(f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 {HYUNDAI_COLORS['accent']}, 
                    stop:1 {HYUNDAI_COLORS['blue_muted']});
                color: {HYUNDAI_COLORS['text_primary']};
                border: 2px solid {HYUNDAI_COLORS['blue_soft']};
                border-radius: 20px;
                font-size: 16px;
                font-weight: bold;
                font-family: 'Malgun Gothic';
            }}
            QPushButton:hover {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 {HYUNDAI_COLORS['blue_soft']}, 
                    stop:1 {HYUNDAI_COLORS['accent']});
                border: 2px solid {HYUNDAI_COLORS['text_primary']};
            }}
            QPushButton:pressed {{
                background: {HYUNDAI_COLORS['blue_muted']};
                border: 2px solid {HYUNDAI_COLORS['warning']};
            }}
        """)
        self.exit_scenario_button.clicked.connect(self.start_exit_scenario)
    def start_exit_scenario(self):
        if hasattr(self.parent(), 'start_exit_scenario'):
            self.parent().start_exit_scenario()
    def init_particles(self):
        self.particle_positions = []
        for _ in range(8):
            self.particle_positions.append({
                'x': random.randint(0, 450), 'y': random.randint(0, 700),
                'speed': random.uniform(0.3, 1.0), 'size': random.randint(1, 3),
                'opacity': random.uniform(0.05, 0.15)
            })
    def update_animation(self):
        self.rotation_angle = (self.rotation_angle + 1) % 360
        if self.pulse_growing:
            self.pulse_scale += 0.01
            if self.pulse_scale >= 1.05: self.pulse_growing = False
        else:
            self.pulse_scale -= 0.01
            if self.pulse_scale <= 1.0: self.pulse_growing = True
        if self.glow_increasing:
            self.glow_opacity += 0.02
            if self.glow_opacity >= 0.4: self.glow_increasing = False
        else:
            self.glow_opacity -= 0.02
            if self.glow_opacity <= 0.2: self.glow_increasing = True
        for particle in self.particle_positions:
            particle['y'] -= particle['speed']
            if particle['y'] < 0:
                particle['y'] = 700
                particle['x'] = random.randint(0, 450)
        if self.direction_transition < 1.0:
            self.direction_transition = min(1.0, self.direction_transition + 0.1)
        self.update()
    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        rect, center_x = self.rect(), self.rect().width() // 2
        self.draw_background_effects(painter, rect)
        self.draw_3d_direction_display(painter, center_x, 120)
        self.draw_distance_panel(painter, center_x, 280)
        self.draw_speed_gauge(painter, center_x, 400)
        self.draw_progress_bar(painter, center_x, 500)
        self.draw_next_instruction_card(painter, center_x, 580)
        self.draw_decorative_elements(painter, rect)
    def draw_background_effects(self, painter, rect):
        painter.save()
        for particle in self.particle_positions:
            color = QColor(HYUNDAI_COLORS['blue_soft'])
            color.setAlphaF(particle['opacity'])
            painter.setBrush(QBrush(color))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(particle['x'], particle['y']), particle['size'], particle['size'])
        painter.restore()
    def draw_3d_direction_display(self, painter, center_x, y):
        painter.save()
        painter.translate(center_x, y)
        painter.rotate(self.rotation_angle)
        gradient = QRadialGradient(0, 0, 90)
        gradient.setColorAt(0, QColor(0, 0, 0, 0))
        gradient.setColorAt(0.8, QColor(HYUNDAI_COLORS['gray_light']).darker(200))
        gradient.setColorAt(1, QColor(HYUNDAI_COLORS['gray_medium']))
        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['gray_medium']), 1))
        painter.drawEllipse(QPointF(0, 0), 85, 85)
        painter.rotate(-self.rotation_angle)
        painter.scale(self.pulse_scale, self.pulse_scale)
        is_warning = self.current_distance <= 0 and ("좌회전" in self.current_direction or "우회전" in self.current_direction)
        is_exit_complete = "출차 완료" in self.current_direction
        is_destination_arrival = "목적지 도착" in self.current_direction
        if is_warning or is_exit_complete or is_destination_arrival:
            main_color = QColor(HYUNDAI_COLORS['warning'])
            bg_color = main_color.darker(150)
        else:
            main_color = QColor(HYUNDAI_COLORS['blue_soft'])
            bg_color = main_color.darker(150)
        painter.setBrush(QBrush(bg_color))
        painter.setPen(QPen(main_color.lighter(120), 3))
        painter.drawEllipse(QPointF(0, 0), 65, 65)
        inner_gradient = QRadialGradient(0, 0, 30)
        inner_gradient.setColorAt(0, QColor(255, 255, 255, 20))
        inner_gradient.setColorAt(1, QColor(255, 255, 255, 0))
        painter.setBrush(QBrush(inner_gradient))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(0, 0), 30, 30)
        painter.scale(1 / self.pulse_scale, 1 / self.pulse_scale)
        self.draw_3d_direction_icon(painter)
        painter.restore()
    def draw_3d_direction_icon(self, painter):
        painter.save()
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 30)))
        action = None
        if "좌회전" in self.current_direction:
            action = self.draw_3d_left_arrow
        elif "우회전" in self.current_direction:
            action = self.draw_3d_right_arrow
        elif "목적지" in self.current_direction:
            action = self.draw_3d_destination_icon
        elif "출차 완료" in self.current_direction:
            action = self.draw_3d_exit_complete_icon
        if action:
            painter.translate(2, 2)
            action(painter, 0, 0, shadow=True)
            painter.translate(-2, -2)
            painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['white_soft'])))
            painter.setPen(QPen(QColor(HYUNDAI_COLORS['white_soft']), 2))
            action(painter, 0, 0)
        else:
            self.draw_3d_straight_arrow(painter, 0, 0)
        painter.restore()
    def draw_3d_left_arrow(self, painter, x, y, shadow=False):
        if not shadow: painter.drawPolygon(QPolygonF([QPointF(x-35,y),QPointF(x-15,y-20),QPointF(x-15,y-10),QPointF(x+20,y-10),QPointF(x+20,y+10),QPointF(x-15,y+10),QPointF(x-15,y+20)]))
        else: painter.drawPolygon(QPolygonF([QPointF(x-35,y),QPointF(x-15,y-20),QPointF(x-15,y-10),QPointF(x+20,y-10),QPointF(x+20,y+10),QPointF(x-15,y+10),QPointF(x-15,y+20)]))
    def draw_3d_right_arrow(self, painter, x, y, shadow=False):
        if not shadow: painter.drawPolygon(QPolygonF([QPointF(x+35,y),QPointF(x+15,y-20),QPointF(x+15,y-10),QPointF(x-20,y-10),QPointF(x-20,y+10),QPointF(x+15,y+10),QPointF(x+15,y+20)]))
        else: painter.drawPolygon(QPolygonF([QPointF(x+35,y),QPointF(x+15,y-20),QPointF(x+15,y-10),QPointF(x-20,y-10),QPointF(x-20,y+10),QPointF(x+15,y+10),QPointF(x+15,y+20)]))
    def draw_3d_straight_arrow(self, painter, x, y):
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0,0,0,30)))
        painter.drawPolygon(QPolygonF([QPointF(x+2,y-32),QPointF(x-15,y-7),QPointF(x-7,y-7),QPointF(x-7,y+28),QPointF(x+13,y+28),QPointF(x+13,y-7),QPointF(x+21,y-7)]))
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['white_soft'])))
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['white_soft']), 2))
        painter.drawPolygon(QPolygonF([QPointF(x,y-35),QPointF(x-18,y-10),QPointF(x-10,y-10),QPointF(x-10,y+25),QPointF(x+10,y+25),QPointF(x+10,y-10),QPointF(x+18,y-10)]))
    def draw_3d_destination_icon(self, painter, x, y, shadow=False):
        if shadow:
            painter.drawEllipse(QPointF(x, y), 25, 25)
        else:
            painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['warning'])))
            painter.setPen(QPen(QColor(HYUNDAI_COLORS['white_soft']), 2))
            painter.drawEllipse(QPointF(x, y), 25, 25)
            painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['white_soft'])))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(x, y), 8, 8)
    def draw_3d_exit_complete_icon(self, painter, x, y, shadow=False):
        if shadow:
            painter.drawEllipse(QPointF(x, y), 25, 25)
        else:
            painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['warning'])))
            painter.setPen(QPen(QColor(HYUNDAI_COLORS['white_soft']), 2))
            painter.drawEllipse(QPointF(x, y), 25, 25)
            painter.setPen(QPen(QColor(HYUNDAI_COLORS['white_soft']), 3))
            painter.drawLine(QPointF(x-8, y), QPointF(x-2, y+6))
            painter.drawLine(QPointF(x-2, y+6), QPointF(x+8, y-6))
    def draw_distance_panel(self, painter, center_x, y):
        painter.save()
        panel_rect = QRectF(center_x - 150, y - 50, 300, 100)
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['surface'])))
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['gray_medium']), 1))
        painter.drawRoundedRect(panel_rect, 20, 20)
        distance_text = f"{self.current_distance:.0f}m" if self.current_distance < 1000 else f"{self.current_distance/1000:.1f}km"
        font = QFont("Segoe UI", FONT_SIZES['hud_distance'], QFont.Bold)
        painter.setFont(font)
        if self.current_distance <= 5:
            text_color = QColor(HYUNDAI_COLORS['warning'])
        elif self.current_distance <= 20:
            text_color = QColor(HYUNDAI_COLORS['success'])
        elif "출차 완료" in self.current_direction or "목적지 도착" in self.current_direction:
            text_color = QColor(HYUNDAI_COLORS['warning'])
        else:
            text_color = QColor(HYUNDAI_COLORS['blue_soft'])
        painter.setPen(QPen(text_color))
        painter.drawText(QRectF(center_x-150, y-30, 300, 60), Qt.AlignCenter, distance_text)
        font = QFont("Malgun Gothic", FONT_SIZES['hud_direction'])
        painter.setFont(font)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['text_secondary'])))
        direction_text = self.current_direction[:20] + "..." if len(self.current_direction)>20 else self.current_direction
        painter.drawText(QRectF(center_x-150, y+10, 300, 40), Qt.AlignCenter, direction_text)
        painter.restore()
    def draw_speed_gauge(self, painter, center_x, y):
        painter.save()
        gauge_rect = QRectF(center_x - 80, y - 40, 160, 80)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['gray_medium']), 6))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(gauge_rect, 0, 180 * 16)
        speed_angle = min(180, (self.speed / 100) * 180)
        if "출차 완료" in self.current_direction or "목적지 도착" in self.current_direction:
            painter.setPen(QPen(QColor(HYUNDAI_COLORS['warning']), 6))
        else:
            painter.setPen(QPen(QColor(HYUNDAI_COLORS['blue_soft']), 6))
        painter.drawArc(gauge_rect, 0, int(speed_angle * 16))
        font = QFont("Segoe UI", FONT_SIZES['hud_speed'], QFont.Bold)
        painter.setFont(font)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['text_primary'])))
        painter.drawText(QRectF(center_x-80, y-20, 160, 40), Qt.AlignCenter, f"{self.speed}")
        font = QFont("Malgun Gothic", FONT_SIZES['hud_speed_unit'])
        painter.setFont(font)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['text_secondary'])))
        painter.drawText(QRectF(center_x-80, y+10, 160, 20), Qt.AlignCenter, "km/h")
        painter.restore()
    def draw_progress_bar(self, painter, center_x, y):
        painter.save()
        bar_width, bar_height = 350, 12
        bar_rect = QRectF(center_x - bar_width / 2, y - bar_height / 2, bar_width, bar_height)
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['gray_medium'])))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(bar_rect, 6, 6)
        if self.progress > 0:
            progress_rect = QRectF(bar_rect.x(), bar_rect.y(), (self.progress / 100) * bar_width, bar_height)
            if "출차 완료" in self.current_direction or "목적지 도착" in self.current_direction:
                painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['warning'])))
            else:
                painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['blue_soft'])))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(progress_rect, 6, 6)
        font = QFont("Segoe UI", FONT_SIZES['hud_progress'], QFont.Bold)
        painter.setFont(font)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['text_primary'])))
        painter.drawText(QRectF(center_x-175, y+10, 350, 30), Qt.AlignCenter, f"{self.progress:.0f}%")
        painter.restore()
    def draw_next_instruction_card(self, painter, center_x, y):
        if not self.next_direction: return
        painter.save()
        card_rect = QRectF(center_x-200, y-40, 400, 80)
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['surface'])))
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['gray_medium']), 1))
        painter.drawRoundedRect(card_rect, 20, 20)
        font = QFont("Malgun Gothic", FONT_SIZES['hud_next_label'], QFont.Bold)
        painter.setFont(font)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['blue_soft'])))
        painter.drawText(QPointF(center_x-190, y-15), "다음")
        icon_x, icon_y = center_x - 140, y + 10
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['gray_medium'])))
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['blue_soft']), 1))
        painter.drawEllipse(QPointF(icon_x, icon_y), 25, 25)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['white_soft']), 2))
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['white_soft'])))
        if "좌회전" in self.next_direction:
            self.draw_mini_left_arrow(painter, icon_x, icon_y)
        elif "우회전" in self.next_direction:
            self.draw_mini_right_arrow(painter, icon_x, icon_y)
        elif "목적지" in self.next_direction or "도착" in self.next_direction:
            self.draw_mini_destination(painter, icon_x, icon_y)
        elif "출차 완료" in self.next_direction:
            self.draw_mini_exit_complete(painter, icon_x, icon_y)
        else:
            self.draw_mini_straight(painter, icon_x, icon_y)
        font.setPointSize(FONT_SIZES['hud_next_direction'])
        painter.setFont(font)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['text_secondary'])))
        painter.drawText(QRectF(icon_x+30, y-20, 200, 60), Qt.AlignVCenter, 
                        self.next_direction[:20]+"..." if len(self.next_direction)>20 else self.next_direction)
        painter.restore()
    def draw_mini_left_arrow(self, painter, x, y): 
        painter.drawPolygon(QPolygonF([QPointF(x-12,y),QPointF(x-5,y-7),QPointF(x-5,y-3),QPointF(x+8,y-3),QPointF(x+8,y+3),QPointF(x-5,y+3),QPointF(x-5,y+7)]))
    def draw_mini_right_arrow(self, painter, x, y): 
        painter.drawPolygon(QPolygonF([QPointF(x+12,y),QPointF(x+5,y-7),QPointF(x+5,y-3),QPointF(x-8,y-3),QPointF(x-8,y+3),QPointF(x+5,y+3),QPointF(x+5,y+7)]))
    def draw_mini_straight(self, painter, x, y): 
        painter.drawPolygon(QPolygonF([QPointF(x,y-12),QPointF(x-6,y-4),QPointF(x-3,y-4),QPointF(x-3,y+8),QPointF(x+3,y+8),QPointF(x+3,y-4),QPointF(x+6,y-4)]))
    def draw_mini_destination(self, painter, x, y):
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['warning'])))
        painter.drawEllipse(QPointF(x,y), 8, 8)
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['white_soft'])))
        painter.drawEllipse(QPointF(x,y), 3, 3)
    def draw_mini_exit_complete(self, painter, x, y):
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['warning'])))
        painter.drawEllipse(QPointF(x,y), 8, 8)
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['white_soft']), 2))
        painter.drawLine(QPointF(x-3, y), QPointF(x-1, y+2))
        painter.drawLine(QPointF(x-1, y+2), QPointF(x+3, y-2))
    def draw_decorative_elements(self, painter, rect):
        painter.save()
        painter.setBrush(QBrush(QColor(HYUNDAI_COLORS['gray_medium'])))
        painter.setPen(Qt.NoPen)
        painter.drawRect(0, 20, rect.width(), 2)
        painter.drawRect(0, rect.height()-22, rect.width(), 2)
        corner_size = 20
        painter.setPen(QPen(QColor(HYUNDAI_COLORS['blue_soft']), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(15, 15, corner_size, corner_size, 90*16, 90*16)
        painter.drawArc(rect.width()-35, 15, corner_size, corner_size, 0*16, 90*16)
        painter.drawArc(15, rect.height()-35, corner_size, corner_size, 180*16, 90*16)
        painter.drawArc(rect.width()-35, rect.height()-35, corner_size, corner_size, 270*16, 90*16)
        painter.restore()
    def update_navigation_info(self, instructions, current_speed=0, route_progress=0):
        self.speed, self.progress = current_speed, route_progress
        if not instructions:
            self.current_direction, self.current_distance, self.next_direction = "경로를 생성하세요", 0.0, ""
            self.update()
            return
        direction, distance = instructions[0]
        is_turn_complete = ("좌회전" in direction or "우회전" in direction) and distance <= 1
        if is_turn_complete and len(instructions) > 1:
            next_dir, next_dist = instructions[1]
            if "목적지" in next_dir and next_dist > 5:
                self.current_direction = "직진"
                self.current_distance = next_dist
                self.next_direction = next_dir
            else:
                self.current_direction = next_dir
                self.current_distance = next_dist
                if len(instructions) > 2:
                    self.next_direction = instructions[2][0]
                else:
                    self.next_direction = ""
        else:
            if distance > 5:
                self.current_direction = "직진"
                self.current_distance = distance
                self.next_direction = direction
            else:
                self.current_direction = direction
                self.current_distance = distance
                if len(instructions) > 1:
                    next_dir, next_dist = instructions[1]
                    if "목적지" in next_dir and next_dist <= 5:
                        self.next_direction = next_dir
                    else:
                        self.next_direction = "직진"
                else:
                    self.next_direction = ""
        new_direction = self.current_direction
        if new_direction != self.target_direction:
            self.previous_direction, self.target_direction, self.direction_transition = self.target_direction, new_direction, 0.0
        self.update()
class CarItem(QGraphicsObject):
    positionChanged = pyqtSignal(QPointF)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.car_body = QPolygonF([
            QPointF(-45, -45), QPointF(45, -45), QPointF(40, 15), QPointF(-40, 15)
        ])
        self.car_cabin = QPolygonF([
            QPointF(-30, 15), QPointF(30, 15), QPointF(25, 45), QPointF(-25, 45)
        ])
        self.headlight_left = QRectF(-35, -10, 15, 10)
        self.headlight_right = QRectF(20, -10, 15, 10)
        self.grille = QRectF(-15, -15, 30, 10)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)
        self.setZValue(100)
        self.setRotation(0)
    def boundingRect(self):
        return self.car_body.boundingRect().united(self.car_cabin.boundingRect()).adjusted(-5, -5, 5, 5)
    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.Antialiasing)
        painter.save()
        painter.translate(4, 4)
        painter.setBrush(QBrush(QColor(0, 0, 0, 70)))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(self.car_body)
        painter.drawPolygon(self.car_cabin)
        painter.restore()
        body_gradient = QLinearGradient(0, 15, 0, -45)
        body_gradient.setColorAt(0, QColor(220, 30, 30))
        body_gradient.setColorAt(1, QColor(150, 20, 20))
        painter.setBrush(QBrush(body_gradient))
        painter.setPen(QPen(QColor(255, 200, 200, 150), 2))
        painter.drawPolygon(self.car_body)
        cabin_gradient = QLinearGradient(0, 45, 0, 15)
        cabin_gradient.setColorAt(0, QColor(50, 60, 80))
        cabin_gradient.setColorAt(1, QColor(20, 30, 50))
        painter.setBrush(QBrush(cabin_gradient))
        painter.setPen(QPen(QColor(150, 180, 200, 100), 1))
        painter.drawPolygon(self.car_cabin)
        headlight_gradient = QRadialGradient(0, 0, 15)
        headlight_gradient.setColorAt(0, QColor(255, 255, 220))
        headlight_gradient.setColorAt(1, QColor(200, 200, 150, 100))
        painter.save()
        painter.translate(self.headlight_left.center())
        painter.setBrush(QBrush(headlight_gradient))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QRectF(-7.5, -5, 15, 10))
        painter.restore()
        painter.save()
        painter.translate(self.headlight_right.center())
        painter.setBrush(QBrush(headlight_gradient))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QRectF(-7.5, -5, 15, 10))
        painter.restore()
        painter.setBrush(QBrush(QColor(50, 60, 70)))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(self.grille, 3, 3)
        painter.setPen(QPen(QColor(100, 110, 120), 1.5))
        painter.drawLine(int(self.grille.left()), int(self.grille.center().y()), int(self.grille.right()), int(self.grille.center().y()))
    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            self.positionChanged.emit(value)
        return super().itemChange(change, value)
class ParkingLotUI(QWidget):
    SCENE_W, SCENE_H = 2000, 2000
    CELL, MARGIN, PATH_WIDTH = 30, 10, 50
    PIXELS_PER_METER = 50
    ENTRANCE = QPointF(200, 200)
    newWaypointsReceived = pyqtSignal(list)
    carPositionReceived = pyqtSignal(list)
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SmartParking Navigation System")
        self.initial_fit = False
        self.received_waypoints = []
        self.setup_styles()
        self.init_ui()
        self.init_map()
        self.init_wifi()
    def setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{ background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 {HYUNDAI_COLORS['background']}, stop:1 {HYUNDAI_COLORS['surface']}); color: {HYUNDAI_COLORS['text_primary']}; font-family: 'Malgun Gothic'; }}
            QGraphicsView {{ border: 3px solid {HYUNDAI_COLORS['accent']}; border-radius: 15px; background: '#303030'; }}
        """)
    def init_ui(self):
        main_layout = QHBoxLayout(self)
        self.scene = QGraphicsScene(0, 0, self.SCENE_W, self.SCENE_H)
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.scale(1, -1)
        self.view.translate(0, -self.SCENE_H)
        self.hud = PremiumHudWidget()
        main_layout.addWidget(self.view, 3)
        main_layout.addWidget(self.hud, 1)
    def init_map(self):
        self.layer_static = QGraphicsItemGroup()
        self.layer_path = QGraphicsItemGroup()
        self.scene.addItem(self.layer_static)
        self.scene.addItem(self.layer_path)
        self.full_path_points = []
        self.snapped_waypoints = []
        self.current_path_segment_index = 0
        self.is_exit_scenario = False
        self.car = CarItem()
        self.car.positionChanged.connect(self.update_hud_from_car_position)
        self.scene.addItem(self.car)
        self.car.hide()
        self.parking_spots = {}
        self.build_static_layout()
        self.build_occupancy()
        self.hud.update_navigation_info([])
    def init_wifi(self):
        self.newWaypointsReceived.connect(self.update_ui_with_waypoints)
        self.carPositionReceived.connect(self.update_car_position_from_wifi)
        self.waypoint_receiver = WaypointReceiver()
        self.waypoint_receiver.set_waypoint_callback(self.handle_new_waypoints_from_thread)
        self.waypoint_receiver.set_position_callback(self.handle_new_position_from_thread)
        self.waypoint_receiver.start_receiver()
        QMessageBox.information(self, "WiFi 수신기", f"서버가 {self.waypoint_receiver.host}:{self.waypoint_receiver.port}에서 시작되었습니다.\n관제 시스템의 연결을 기다립니다.")
    def handle_new_waypoints_from_thread(self, waypoints):
        self.newWaypointsReceived.emit(waypoints)
    def handle_new_position_from_thread(self, position):
        self.carPositionReceived.emit(position)
    def update_ui_with_waypoints(self, waypoints):
        if not waypoints or not isinstance(waypoints, list):
            QMessageBox.warning(self, "수신 오류", "잘못된 형식의 웨이포인트 데이터가 수신되었습니다.")
            return
        self.received_waypoints = waypoints
        QMessageBox.information(self, "경로 자동 설정", f"새로운 경로가 수신되었습니다:\n{waypoints}\n\n자동으로 경로 안내를 시작합니다.")
        self.calculate_and_display_route()
    def update_car_position_from_wifi(self, position: List[float]):
        if not (isinstance(position, list) and len(position) == 2):
            return
        new_pos = QPointF(position[0], position[1])
        self.car.setPos(new_pos)
    def detect_parking_spot_from_waypoint(self, waypoint):
        x, y = waypoint[0], waypoint[1]
        parking_waypoints = {
            1: [200, 1475], 2: [550, 1475], 3: [850, 1475], 4: [1150, 1475],
            5: [1450, 1475],
            6: [1475, 1400], 7: [1475, 1000],
            8: [1475, 925], 9: [1150, 925], 10: [850, 925], 11: [550, 925]
        }
        tolerance = 50
        for spot_num, coord in parking_waypoints.items():
            if abs(x - coord[0]) <= tolerance and abs(y - coord[1]) <= tolerance:
                return spot_num
        return None
    def change_parking_spot_color(self, parking_spot_num, color):
        if parking_spot_num in self.parking_spots:
            rect_item = self.parking_spots[parking_spot_num]
            if color == "orange":
                gradient = QLinearGradient(rect_item.rect().x(), rect_item.rect().y(),
                                        rect_item.rect().x() + rect_item.rect().width(),
                                        rect_item.rect().y() + rect_item.rect().height())
                gradient.setColorAt(0, QColor(255, 165, 0, 250))
                gradient.setColorAt(1, QColor(255, 140, 0, 200))
                rect_item.setBrush(QBrush(gradient))
                rect_item.setPen(QPen(QColor("white"), 20))
                print(f"🎯 주차구역 {parking_spot_num}번 색상을 주황색으로 변경 (테두리는 흰색 유지)")
            else:
                self.restore_parking_spot_color(parking_spot_num)
    def restore_parking_spot_color(self, parking_spot_num):
        if parking_spot_num in self.parking_spots:
            rect_item = self.parking_spots[parking_spot_num]
            if parking_spot_num in [1, 6, 7]:
                gradient = QLinearGradient(rect_item.rect().x(), rect_item.rect().y(),
                                        rect_item.rect().x() + rect_item.rect().width(),
                                        rect_item.rect().y() + rect_item.rect().height())
                gradient.setColorAt(0, QColor(135, 206, 250, 200))
                gradient.setColorAt(1, QColor(70, 130, 180, 150))
                rect_item.setBrush(QBrush(gradient))
            elif parking_spot_num in [4, 5, 10, 11]:
                gradient = QLinearGradient(rect_item.rect().x(), rect_item.rect().y(),
                                        rect_item.rect().x() + rect_item.rect().width(),
                                        rect_item.rect().y() + rect_item.rect().height())
                gradient.setColorAt(0, QColor(0, 200, 130, 200))
                gradient.setColorAt(1, QColor(0, 150, 100, 150))
                rect_item.setBrush(QBrush(gradient))
            else:
                gradient = QLinearGradient(rect_item.rect().x(), rect_item.rect().y(),
                                        rect_item.rect().x() + rect_item.rect().width(),
                                        rect_item.rect().y() + rect_item.rect().height())
                gradient.setColorAt(0, QColor("#303030"))
                gradient.setColorAt(1, QColor("#303030"))
                rect_item.setBrush(QBrush(gradient))
            rect_item.setPen(QPen(QColor("white"), 20))
            print(f"🎯 주차구역 {parking_spot_num}번 색상을 원래 색상으로 복원")
    def calculate_and_display_route(self):
        if not self.received_waypoints:
            QMessageBox.warning(self, "경로 오류", "경로를 계산할 웨이포인트가 없습니다.")
            return
        print(f"🗺️ 웨이포인트 경로 생성: {self.received_waypoints}")
        start_point = QPointF(200, 200)
        waypoints_qpoints = [QPointF(p[0], p[1]) for p in self.received_waypoints]
        self.full_path_points = [start_point] + waypoints_qpoints
        if self.received_waypoints:
            last_waypoint = self.received_waypoints[-1]
            destination_parking_spot = self.detect_parking_spot_from_waypoint(last_waypoint)
            if destination_parking_spot:
                print(f"🎯 마지막 웨이포인트는 주차구역 {destination_parking_spot}번 입니다. 색상을 주황색으로 변경합니다.")
                self.change_parking_spot_color(destination_parking_spot, "orange")
            else:
                print(f"📍 마지막 웨이포인트 ({last_waypoint})는 주차구역이 아닙니다.")
        print(f"✅ 최종 경로: {len(self.full_path_points)}개 포인트")
        for i, point in enumerate(self.full_path_points):
            print(f"  {i+1}. ({point.x():.1f}, {point.y():.1f})")
        self.clear_path_layer()
        self.draw_straight_path(self.full_path_points)
        self.current_path_segment_index = 0
        self.is_exit_scenario = False
        if not self.car.isVisible():
            self.car.setPos(start_point)
            self.car.show()
        self.update_hud_from_car_position(self.car.pos())
    def showEvent(self, event):
        super().showEvent(event)
        if not self.initial_fit:
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
            self.initial_fit = True
    def closeEvent(self, event):
        self.waypoint_receiver.stop()
        super().closeEvent(event)
    def add_block(self, x, y, w, h, color, label=""):
        r = QGraphicsRectItem(QRectF(x, y, w, h))
        if "장애인" in label:
            gradient = QLinearGradient(x,y,x+w,y+h)
            gradient.setColorAt(0,QColor(135, 206, 250, 200))
            gradient.setColorAt(1,QColor(70, 130, 180,150))
            r.setBrush(QBrush(gradient))
        elif "전기차" in label:
            gradient = QLinearGradient(x,y,x+w,y+h)
            gradient.setColorAt(0,QColor(0,200,130,200))
            gradient.setColorAt(1,QColor(0,150,100,150))
            r.setBrush(QBrush(gradient))
        elif "일반" in label:
            gradient = QLinearGradient(x,y,x+w,y+h)
            gradient.setColorAt(0,QColor("#303030"))
            gradient.setColorAt(1,QColor("#303030"))
            r.setBrush(QBrush(gradient))
        else:
            r.setBrush(QBrush(color))
        if "장애인" in label or "전기" in label or "일반" in label:
            pen = QPen(QColor("white"), 20)
            r.setPen(pen)
        elif label in ["백화점 본관 입구", "영화관 입구", "문화시설 입구"]:
            pen = QPen(QColor(255, 255, 0), 20)
            r.setPen(pen)
        elif "입출차" in label:
            r.setPen(QPen(Qt.NoPen))
        else:
            r.setPen(QPen(QColor(255,255,255,100), 2))
        r.setParentItem(self.layer_static)
        if label:
            t = QGraphicsSimpleTextItem(label)
            t.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            t.setBrush(QColor(255,255,255))
            if label in ["백화점 본관 입구", "영화관 입구", "문화시설 입구"]:
                font = QFont("Malgun Gothic", int(FONT_SIZES['map_label'] * 2.25), QFont.Bold)
                if label == "백화점 본관 입구":
                    t.setPos(x+w//2-50-310, y-20)
                elif label == "영화관 입구":
                    t.setPos(x+w+20, y+h-40)
                elif label == "문화시설 입구":
                    t.setPos(x+w+20, y+h-60)
            elif label in ["장애인", "전기", "일반"]:
                font = QFont("Malgun Gothic", int(FONT_SIZES['map_label'] * 1.5), QFont.Bold)
                t.setPos(x+5,y+h-25)
            else:
                font = QFont("Malgun Gothic", FONT_SIZES['map_label'], QFont.Bold)
                t.setPos(x+5,y+h-25)
            t.setFont(font)
            t.setParentItem(self.layer_static)
        return r
    def add_hatched(self, x, y, w, h, edge=QColor("black"), fill=QColor(220, 20, 60, 90)):
        r = QGraphicsRectItem(QRectF(x,y,w,h)); b = QBrush(fill); b.setStyle(Qt.BDiagPattern); r.setBrush(b); r.setPen(QPen(edge,3)); r.setParentItem(self.layer_static)
        t = QGraphicsSimpleTextItem("통행 불가"); t.setFlag(QGraphicsItem.ItemIgnoresTransformations, True); t.setBrush(QColor(255,100,100))
        font = QFont("Malgun Gothic", int(FONT_SIZES['map_label'] * 1.5), QFont.Bold); t.setFont(font); t.setPos(x+10,y+h-30); t.setParentItem(self.layer_static)
    def add_dot_label_static(self, p: QPointF, text: str, color=QColor("blue")):
        t = QGraphicsSimpleTextItem(text); t.setFlag(QGraphicsItem.ItemIgnoresTransformations, True); t.setBrush(QColor(0,200,255))
        font = QFont("Malgun Gothic", FONT_SIZES['map_io_label'], QFont.Bold); t.setFont(font); t.setPos(p.x()-20,p.y()+25); t.setParentItem(self.layer_static)
    def build_static_layout(self):
        c_dis, c_ele, c_gen, c_obs, c_emp, c_io = QColor(135, 206, 250), QColor(0, 200, 130), QColor("#303030"), QColor(108, 117, 125), QColor(206, 212, 218), QColor("#303030")
        border = QGraphicsRectItem(0, 0, self.SCENE_W, self.SCENE_H); border.setPen(QPen(QColor(0, 170, 210), 12)); border.setBrush(QBrush(Qt.NoBrush)); border.setParentItem(self.layer_static)
        self.add_hatched(400, 0, 1600, 400)
        self.add_block(0, 0, 400, 400, c_io, "입출차")
        base = [
            (-400, 1600, 400, 400, c_emp, "백화점 본관 입구"),
            (1600, 1600, 400, 400, c_emp, "영화관 입구"),
            (550, 1050, 800, 300, c_obs, "장애물")
        ]
        parking_spots = [
            (0, 1600, 400, 400, c_dis, "장애인"),
            (400, 1600, 300, 400, c_gen, "일반"),
            (700, 1600, 300, 400, c_gen, "일반"),
            (1000, 1600, 300, 400, c_ele, "전기"),
            (1300, 1600, 300, 400, c_ele, "전기"),
            (1600, 1200, 400, 400, c_dis, "장애인"),
            (1000, 400, 300, 400, c_gen, "일반"),
            (700, 400, 300, 400, c_ele, "전기"),
            (400, 400, 300, 400, c_ele, "전기")
        ]
        for x, y, w, h, c, l in base: self.add_block(x, y, w, h, c, l)
        self.add_dot_label_static(self.ENTRANCE, "입구", QColor(0, 170, 210))
        spot_numbers = [1, 2, 3, 4, 5, 6, 9, 10, 11]
        for i, (x, y, w, h, c, l) in enumerate(parking_spots):
            rect_item = self.add_block(x, y, w, h, c, l)
            if rect_item:
                self.parking_spots[spot_numbers[i]] = rect_item
        self.add_block(1600, 400, 400, 400, c_emp, "문화시설 입구")
        rect_item = self.add_block(1600, 800, 400, 400, c_dis, "장애인")
        self.parking_spots[7] = rect_item
        rect_item = self.add_block(1300, 400, 300, 400, c_gen, "일반")
        self.parking_spots[8] = rect_item
    def build_occupancy(self):
        W, H, C = self.SCENE_W, self.SCENE_H, self.CELL; gx, gy = (W + C - 1) // C, (H + C - 1) // C
        self.grid_w, self.grid_h = gx, gy; self.occ = bytearray(gx * gy)
        def idx(cx, cy): return cy * gx + cx
        def block_rect(x, y, w, h):
            x0,y0,x1,y1 = max(0,x-self.MARGIN), max(0,y-self.MARGIN), min(W,x+w+self.MARGIN), min(H,y+h+self.MARGIN)
            cx0,cy0,cx1,cy1 = int(x0//C), int(y0//C), int((x1-1)//C), int((y1-1)//C)
            for cy in range(cy0,cy1+1):
                for cx in range(cx0,cx1+1):
                    if 0<=cx<gx and 0<=cy<gy: self.occ[cy*gx+cx] = 1
        for x,y,w,h,c,l in [
            (550,1050,800,300,0,""),
            (400,0,1600,400,0,""),
            (1600,400,400,400,0,""),
            (1600,1600,400,400,0,""),
            (-400,1600,400,400,0,""),
            (0,0,400,400,0,"")
        ]: 
            block_rect(x,y,w,h)
        parking_blocks = [
            (0, 1600, 400, 400, 0, ""),
            (400, 1600, 300, 400, 0, ""),
            (700, 1600, 300, 400, 0, ""),
            (1000, 1600, 300, 400, 0, ""),
            (1300, 1600, 300, 400, 0, ""),
            (1600, 1200, 400, 400, 0, ""),
            (1600, 800, 400, 400, 0, ""),
            (1300, 400, 300, 400, 0, ""),
            (1000, 400, 300, 400, 0, ""),
            (700, 400, 300, 400, 0, ""),
            (400, 400, 300, 400, 0, "")
        ]
        for x,y,w,h,c,l in parking_blocks: 
            block_rect(x,y,w,h)
        self._occ_idx = idx
    def clamp_point(self, p: QPointF): return QPointF(min(self.SCENE_W-1.,max(0.,p.x())), min(self.SCENE_H-1.,max(0.,p.y())))
    def pt_to_cell(self, p: QPointF): return int(p.x()//self.CELL), int(p.y()//self.CELL)
    def cell_to_pt_center(self, c): return QPointF(c[0]*self.CELL+self.CELL/2., c[1]*self.CELL+self.CELL/2.)
    def is_cell_free(self, cx, cy): return 0<=cx<self.grid_w and 0<=cy<self.grid_h and self.occ[self._occ_idx(cx,cy)]==0
    def find_nearest_free_cell_from_point(self, p: QPointF, max_radius_cells=100):
        sx, sy = self.pt_to_cell(p)
        print(f"🔍 자유 셀 검색: 원본 좌표 ({p.x():.1f}, {p.y():.1f}) -> 셀 ({sx}, {sy})")
        if self.is_cell_free(sx, sy): 
            result = self.cell_to_pt_center((sx, sy))
            print(f"✅ 원본 셀이 자유함: {result}")
            return result
        for r in range(1, max_radius_cells + 1):
            for dx in range(-r, r+1):
                for dy in [-r, r]:
                    if self.is_cell_free(sx+dx, sy+dy): 
                        result = self.cell_to_pt_center((sx+dx, sy+dy))
                        print(f"✅ 자유 셀 발견 (반경 {r}): ({sx+dx}, {sy+dy}) -> {result}")
                        return result
            for dy in range(-r+1, r):
                for dx in [-r, r]:
                    if self.is_cell_free(sx+dx, sy+dy): 
                        result = self.cell_to_pt_center((sx+dx, sy+dy))
                        print(f"✅ 자유 셀 발견 (반경 {r}): ({sx+dx}, {sy+dy}) -> {result}")
                        return result
        result = self.cell_to_pt_center((sx, sy))
        print(f"⚠️ 자유 셀을 찾지 못함, 원본 셀 반환: {result}")
        return result
    def astar(self, start_pt: QPointF, goal_pt: QPointF):
        sx, sy = self.pt_to_cell(start_pt)
        gx, gy = self.pt_to_cell(goal_pt)
        W, H = self.grid_w, self.grid_h
        occ, idx = self.occ, self._occ_idx
        print(f"🗺️ A* 경로 검색: ({start_pt.x():.1f}, {start_pt.y():.1f}) -> ({goal_pt.x():.1f}, {goal_pt.y():.1f})")
        print(f"   셀 좌표: ({sx}, {sy}) -> ({gx}, {gy})")
        if not (0 <= sx < W and 0 <= sy < H and 0 <= gx < W and 0 <= gy < H):
            print(f"❌ 경계 밖 좌표: 시작({sx}, {sy}) 목적지({gx}, {gy}), 그리드 크기({W}, {H})")
            return None
        if occ[idx(sx, sy)]:
            print(f"⚠️ 시작점 ({sx}, {sy})이 점유됨, 자유 셀 검색 중...")
            free_start = self.find_nearest_free_cell_from_point(start_pt)
            sx, sy = self.pt_to_cell(free_start)
            print(f"   새로운 시작점: ({sx}, {sy})")
        if occ[idx(gx, gy)]:
            print(f"⚠️ 목적지 ({gx}, {gy})이 점유됨, 자유 셀 검색 중...")
            free_goal = self.find_nearest_free_cell_from_point(goal_pt)
            gx, gy = self.pt_to_cell(free_goal)
            print(f"   새로운 목적지: ({gx}, {gy})")
        if occ[idx(sx, sy)] or occ[idx(gx, gy)]:
            print(f"❌ 시작점 또는 목적지가 여전히 점유됨: 시작({sx}, {sy})={occ[idx(sx, sy)]}, 목적지({gx}, {gy})={occ[idx(gx, gy)]}")
            return None
        openh = [(abs(sx - gx) + abs(sy - gy), 0, (sx, sy))]
        came, g = {}, {(sx, sy): 0}
        iterations = 0
        max_iterations = 10000
        while openh and iterations < max_iterations:
            iterations += 1
            _, gc, (x, y) = heappop(openh)
            if (x, y) == (gx, gy):
                path = []
                curr = (x, y)
                while curr in came:
                    path.append(curr)
                    curr = came[curr]
                path.append((sx, sy))
                path.reverse()
                print(f"✅ 경로 발견! {len(path)}개 셀, {iterations}회 반복")
                return path
            for dx, dy, cst in [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1)]:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < W and 0 <= ny < H) or occ[idx(nx, ny)]:
                    continue
                ng = gc + cst
                if (nx, ny) not in g or ng < g[(nx, ny)]:
                    g[(nx, ny)] = ng
                    came[(nx, ny)] = (x, y)
                    heappush(openh, (ng + abs(nx - gx) + abs(ny - gy), ng, (nx, ny)))
        if iterations >= max_iterations:
            print(f"❌ 최대 반복 횟수 초과: {max_iterations}회")
        else:
            print(f"❌ 경로를 찾을 수 없음: {iterations}회 반복 후 종료")
        return None
    def simplify_cells(self, cells):
        if not cells: return []
        simp = [cells[0]]
        norm = lambda vx,vy: ((0 if vx==0 else (1 if vx>0 else -1)), (0 if vy==0 else (1 if vy>0 else -1)))
        for i in range(1, len(cells)-1):
            if norm(cells[i][0]-simp[-1][0], cells[i][1]-simp[-1][1]) != norm(cells[i+1][0]-cells[i][0], cells[i+1][1]-cells[i][1]): simp.append(cells[i])
        if len(cells)>1 and cells[-1]!=simp[-1]: simp.append(cells[-1])
        return simp
    def draw_straight_path(self, pts):
        if len(pts) < 2: return
        for i in range(len(pts) - 1):
            start = pts[i]
            end = pts[i + 1]
            for width, alpha in [(self.PATH_WIDTH + 12, 60), (self.PATH_WIDTH + 6, 100)]:
                glow_pen = QPen(QColor(0,170,210,alpha), width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
                self.scene.addLine(start.x(), start.y(), end.x(), end.y(), glow_pen).setParentItem(self.layer_path)
            main_pen = QPen(QColor(0,200,255), self.PATH_WIDTH, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
            self.scene.addLine(start.x(), start.y(), end.x(), end.y(), main_pen).setParentItem(self.layer_path)
            center_pen = QPen(QColor(255,255,255,150), 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
            self.scene.addLine(start.x(), start.y(), end.x(), end.y(), center_pen).setParentItem(self.layer_path)
    def generate_hud_instructions(self, pts, is_exit_scenario=False):
        if len(pts) < 2: return []
        instructions, total_dist = [], 0
        for i in range(len(pts) - 1):
            p1, p2 = pts[i], pts[i+1]
            dist_m = sqrt((p2.x()-p1.x())**2 + (p2.y()-p1.y())**2) / self.PIXELS_PER_METER
            total_dist += dist_m
            if i < len(pts) - 2:
                p3 = pts[i+2]
                angle = (degrees(atan2(p3.y()-p2.y(),p3.x()-p2.x()))-degrees(atan2(p2.y()-p1.y(),p2.x()-p1.x()))+180)%360-180
                direction = "좌회전" if angle>45 else ("우회전" if angle<-45 else "")
                if direction: 
                    if is_exit_scenario:
                        direction = f"출차 {direction}"
                    instructions.append((direction, total_dist)); total_dist = 0
        if is_exit_scenario:
            instructions.append(("출차 완료", total_dist))
        else:
            instructions.append(("목적지 도착", total_dist))
        return instructions
    def calculate_route_progress(self, car_pos):
        if not self.full_path_points or len(self.full_path_points)<2: return 0
        total_len = sum(sqrt((self.full_path_points[i+1].x()-p.x())**2 + (self.full_path_points[i+1].y()-p.y())**2) for i,p in enumerate(self.full_path_points[:-1]))
        if total_len==0: return 0
        min_dist, closest_seg, proj_ratio = float('inf'), 0, 0
        for i,p1 in enumerate(self.full_path_points[:-1]):
            p2 = self.full_path_points[i+1]; seg_vec, car_vec = p2-p1, car_pos-p1
            seg_len_sq = QPointF.dotProduct(seg_vec, seg_vec)
            if seg_len_sq==0: continue
            t = max(0, min(1, QPointF.dotProduct(car_vec, seg_vec)/seg_len_sq))
            proj = p1 + t * seg_vec
            dist = sqrt((car_pos.x()-proj.x())**2 + (car_pos.y()-proj.y())**2)
            if dist < min_dist: min_dist, closest_seg, proj_ratio = dist, i, t
        traveled = sum(sqrt((self.full_path_points[i+1].x()-p.x())**2+(self.full_path_points[i+1].y()-p.y())**2) for i,p in enumerate(self.full_path_points[:closest_seg]))
        if closest_seg < len(self.full_path_points)-1:
            p1, p2 = self.full_path_points[closest_seg], self.full_path_points[closest_seg+1]
            traveled += sqrt((p2.x()-p1.x())**2+(p2.y()-p1.y())**2) * proj_ratio
        return min(100, (traveled / total_len) * 100)
    def clear_path_layer(self):
        for child in self.layer_path.childItems(): self.scene.removeItem(child)
    def _update_current_segment(self, car_pos):
        if not self.full_path_points or len(self.full_path_points) < 2:
            return
        while self.current_path_segment_index < len(self.full_path_points) - 1:
            p_curr = self.full_path_points[self.current_path_segment_index]
            p_next = self.full_path_points[self.current_path_segment_index + 1]
            dist_to_next = sqrt((car_pos.x() - p_next.x())**2 + (car_pos.y() - p_next.y())**2)
            v_seg = p_next - p_curr
            v_car = car_pos - p_curr
            seg_len_sq = QPointF.dotProduct(v_seg, v_seg)
            proj_ratio = 1.0
            if seg_len_sq > 0:
                proj_ratio = QPointF.dotProduct(v_car, v_seg) / seg_len_sq
            if dist_to_next < 50 or proj_ratio > 1.0:
                self.current_path_segment_index += 1
            else:
                break
    def update_hud_from_car_position(self, car_pos):
        if not self.full_path_points: return
        self._update_current_segment(car_pos)
        remaining_pts = self.full_path_points[self.current_path_segment_index+1:]
        path_for_hud = [car_pos] + remaining_pts
        if len(path_for_hud) < 2:
            if self.is_exit_scenario:
                self.hud.update_navigation_info([("출차 완료", 0)], current_speed=0, route_progress=100)
            else:
                self.hud.update_navigation_info([("목적지 도착", 0)], current_speed=0, route_progress=100)
            return
        instructions = self.generate_hud_instructions(path_for_hud, self.is_exit_scenario)
        progress = self.calculate_route_progress(car_pos)
        speed = self.calculate_realistic_speed(instructions, progress, car_pos)
        self.hud.update_navigation_info(instructions, current_speed=speed, route_progress=progress)
    def calculate_realistic_speed(self, instructions, progress, car_pos):
        if not instructions:
            return 0
        direction, distance = instructions[0]
        base_speed = 20
        if distance <= 5:
            speed = 5 + (distance / 5) * 10
        elif distance <= 20:
            speed = 15 + (distance / 20) * 10
        else:
            speed = 20 + min(10, (distance - 20) / 50 * 10)
        if "좌회전" in direction or "우회전" in direction:
            speed = min(speed, 15)
        elif "목적지" in direction or "도착" in direction:
            speed = min(speed, 15)
        elif "출차" in direction:
            speed = min(speed, 20)
        if progress < 20:
            speed *= 0.8
        elif progress > 80:
            speed *= 0.7
        if self.is_exit_scenario:
            speed *= 0.75
        speed = max(0, min(30, int(speed)))
        return speed
    def start_exit_scenario(self):
        if not self.car.isVisible():
            QMessageBox.warning(self, "출차 오류", "차량이 지도에 표시되지 않았습니다. 먼저 경로를 설정해주세요.")
            return
        car_pos = self.car.pos()
        parking_spot = self.detect_parking_spot(car_pos)
        if parking_spot is None:
            QMessageBox.warning(self, "출차 오류", "현재 위치에서 주차 구역을 찾을 수 없습니다.")
            return
        exit_waypoints = self.generate_exit_waypoints(parking_spot)
        if not exit_waypoints:
            QMessageBox.warning(self, "출차 오류", "출차 경로를 생성할 수 없습니다.")
            return
        self.is_exit_scenario = True
        self.clear_path_layer()
        self.calculate_and_display_exit_route(exit_waypoints, parking_spot)
        QMessageBox.information(self, "출차 시나리오", f"주차 구역 {parking_spot}번에서 출차 경로를 시작합니다.\n입차 경로의 역순으로 안전하게 출차하세요.")
    def detect_parking_spot(self, car_pos):
        x, y = car_pos.x(), car_pos.y()
        parking_spots = {
            1: (0, 1600, 400, 400),
            2: (400, 1600, 300, 400),
            3: (700, 1600, 300, 400),
            4: (1000, 1600, 300, 400),
            5: (1300, 1600, 300, 400),
            6: (1600, 1200, 400, 400),
            7: (1600, 800, 400, 400),
            8: (1300, 400, 300, 400),
            9: (1000, 400, 300, 400),
            10: (700, 400, 300, 400),
            11: (400, 400, 300, 400),
        }
        for spot_num, (spot_x, spot_y, spot_w, spot_h) in parking_spots.items():
            if spot_x <= x <= spot_x + spot_w and spot_y <= y <= spot_y + spot_h:
                return spot_num
        return None
    def generate_exit_waypoints(self, parking_spot):
        MANDATORY_WAYPOINT = [200, 925]
        parking_waypoints = {
            1: [200, 1475], 2: [550, 1475], 3: [850, 1475], 4: [1150, 1475],
            5: [1450, 1475],
            6: [1475, 1400], 7: [1475, 1000],
            8: [1475, 925], 9: [1150, 925], 10: [850, 925], 11: [550, 925]
        }
        current_waypoint = parking_waypoints.get(parking_spot)
        if not current_waypoint:
            return None
        FINAL_DESTINATION = [200, 200]
        exit_waypoints = []
        if parking_spot == 1:
            exit_waypoints.append(MANDATORY_WAYPOINT)
            exit_waypoints.append(FINAL_DESTINATION)
        elif parking_spot in [2, 3, 4, 5]:
            exit_waypoints.append([200, 1475])
            exit_waypoints.append(MANDATORY_WAYPOINT)
            exit_waypoints.append(FINAL_DESTINATION)
        elif parking_spot == 6:
            exit_waypoints.append([1475, 1475])
            exit_waypoints.append([200, 1475])
            exit_waypoints.append(MANDATORY_WAYPOINT)
            exit_waypoints.append(FINAL_DESTINATION)
        elif parking_spot == 7:
            exit_waypoints.append([1475, 925])
            exit_waypoints.append(MANDATORY_WAYPOINT)
            exit_waypoints.append(FINAL_DESTINATION)
        elif parking_spot in [8, 9, 10, 11]:
            exit_waypoints.append(MANDATORY_WAYPOINT)
            exit_waypoints.append(FINAL_DESTINATION)
        print(f"🚗 출차 경로 생성 - 주차구역 {parking_spot}번")
        print(f"   최종 목적지: {FINAL_DESTINATION}")
        print(f"   경로 포인트: {exit_waypoints}")
        return exit_waypoints
    def get_parking_spot_start_waypoint(self, parking_spot):
        parking_waypoints = {
            1: [200, 1475], 2: [550, 1475], 3: [850, 1475], 4: [1150, 1475],
            5: [1450, 1475],
            6: [1475, 1400], 7: [1475, 1000],
            8: [1475, 925], 9: [1150, 925], 10: [850, 925], 11: [550, 925]
        }
        if parking_spot in parking_waypoints:
            return parking_waypoints[parking_spot]
        return None
    def calculate_and_display_exit_route(self, exit_waypoints, parking_spot):
        assigned_waypoint = self.get_parking_spot_start_waypoint(parking_spot)
        if not assigned_waypoint:
            QMessageBox.warning(self, "출차 경로 실패", "배정 웨이포인트를 찾을 수 없습니다.")
            return
        start_point = QPointF(assigned_waypoint[0], assigned_waypoint[1])
        waypoints_qpoints = [QPointF(p[0], p[1]) for p in exit_waypoints]
        self.full_path_points = [start_point] + waypoints_qpoints
        print(f"✅ 출차 경로: {len(self.full_path_points)}개 포인트")
        for i, point in enumerate(self.full_path_points):
            print(f"  {i+1}. ({point.x():.1f}, {point.y():.1f})")
        self.clear_path_layer()
        self.draw_exit_path(self.full_path_points)
        self.car.show()
        self.current_path_segment_index = 0
        self.update_hud_from_car_position(self.car.pos())
    def draw_exit_path(self, pts):
        if len(pts) < 2: 
            print(f"출차 경로 포인트가 부족합니다: {len(pts)}개")
            return
        print(f"출차 경로 그리기 시작: {len(pts)}개 포인트")
        for i in range(len(pts) - 1):
            start, end = pts[i], pts[i + 1]
            print(f"경로 구간 {i+1}: ({start.x():.0f}, {start.y():.0f}) -> ({end.x():.0f}, {end.y():.0f})")
            for width, alpha in [(self.PATH_WIDTH + 12, 60), (self.PATH_WIDTH + 6, 100)]:
                glow_pen = QPen(QColor(255, 165, 0, alpha), width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
                line_item = self.scene.addLine(start.x(), start.y(), end.x(), end.y(), glow_pen)
                line_item.setParentItem(self.layer_path)
            main_pen = QPen(QColor(255, 140, 0), self.PATH_WIDTH, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
            line_item = self.scene.addLine(start.x(), start.y(), end.x(), end.y(), main_pen)
            line_item.setParentItem(self.layer_path)
            center_pen = QPen(QColor(255, 255, 255, 150), 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
            line_item = self.scene.addLine(start.x(), start.y(), end.x(), end.y(), center_pen)
            line_item.setParentItem(self.layer_path)
            self.draw_clockwise_arrow(start, end)
        print("출차 경로 그리기 완료")
    def draw_clockwise_arrow(self, start, end):
        mid_x = (start.x() + end.x()) / 2
        mid_y = (start.y() + end.y()) / 2
        dx = end.x() - start.x()
        dy = end.y() - start.y()
        length = sqrt(dx*dx + dy*dy)
        if length == 0:
            return
        nx = dx / length
        ny = dy / length
        arrow_size = 20
        arrow_head_x = mid_x + nx * arrow_size
        arrow_head_y = mid_y + ny * arrow_size
        angle = radians(30)
        cos_angle = cos(angle)
        sin_angle = sin(angle)
        left_wing_x = mid_x + (nx * cos_angle - ny * sin_angle) * arrow_size * 0.6
        left_wing_y = mid_y + (nx * sin_angle + ny * cos_angle) * arrow_size * 0.6
        right_wing_x = mid_x + (nx * cos_angle + ny * sin_angle) * arrow_size * 0.6
        right_wing_y = mid_y + (-nx * sin_angle + ny * cos_angle) * arrow_size * 0.6
        arrow_points = [
            QPointF(arrow_head_x, arrow_head_y),
            QPointF(left_wing_x, left_wing_y),
            QPointF(right_wing_x, right_wing_y)
        ]
        arrow_polygon = QPolygonF(arrow_points)
        arrow_item = QGraphicsPolygonItem(arrow_polygon)
        arrow_item.setBrush(QBrush(QColor(255, 140, 0)))
        arrow_item.setPen(QPen(QColor(255, 255, 255), 2))
        arrow_item.setParentItem(self.layer_path)
        self.scene.addItem(arrow_item)
if __name__ == "__main__":
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    font = QFont("Malgun Gothic"); font.setPointSize(10); app.setFont(font)
    app.setStyleSheet(f"""
        QApplication {{ background-color: '#303030'; }}
        QMessageBox {{ background: {HYUNDAI_COLORS['surface']}; color: {HYUNDAI_COLORS['text_primary']}; border: 1px solid {HYUNDAI_COLORS['accent']}; border-radius: 10px; }}
        QMessageBox QPushButton {{ background: {HYUNDAI_COLORS['primary']}; border: 1px solid {HYUNDAI_COLORS['secondary']}; border-radius: 5px; color: white; padding: 8px 16px; min-width: 60px; font-size: {FONT_SIZES['msgbox_button']}pt; }}
    """)
    ui = ParkingLotUI()
    ui.showMaximized()
    sys.exit(app.exec_())