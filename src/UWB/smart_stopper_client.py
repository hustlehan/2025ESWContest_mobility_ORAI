#!/usr/bin/env python3

import socket
import time
import threading
from enum import Enum
from typing import Optional
import sys

class Command(Enum):
    """제어 명령 열거형"""
    STOP = 0
    FORWARD = 1
    BACKWARD = 2

class StopperController:
    """ESP32 스토퍼 제어 클래스"""
    
    def __init__(self, host='192.168.225.99', port=8888):
        """
        초기화
        Args:
            host: ESP32 IP 주소
            port: ESP32 TCP 포트
        """
        self.host = host
        self.port = port
        self.client_socket = None
        self.connected = False
        self.receive_thread = None
        self.running = False
        
    def connect(self) -> bool:
        """ESP32 스토퍼에 연결"""
        try:
            # 소켓 생성
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(3.0)
            
            # ESP32 서버에 연결
            print(f"[STOPPER] Connecting to {self.host}:{self.port}...")
            self.client_socket.connect((self.host, self.port))
            self.connected = True
            self.running = True
            
            print(f"[STOPPER] Connected to ESP32 at {self.host}:{self.port}")
            
            # 수신 스레드 시작
            self.receive_thread = threading.Thread(target=self._receive_handler)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            return True
            
        except socket.timeout:
            print("[STOPPER] Connection timeout")
            self.connected = False
            return False
            
        except Exception as e:
            print(f"[STOPPER] Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """연결 해제"""
        self.running = False
        self.connected = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
            
        print("[STOPPER] Disconnected")
    
    def _receive_handler(self):
        """ESP32로부터 메시지 수신 처리 (별도 스레드)"""
        buffer = ""
        
        while self.running and self.connected:
            try:
                # 타임아웃 설정으로 주기적 체크
                self.client_socket.settimeout(1.0)
                data = self.client_socket.recv(1024)
                
                if not data:
                    print("[STOPPER] Connection closed by ESP32")
                    self.connected = False
                    break
                    
                # 수신 데이터 처리
                buffer += data.decode('utf-8')
                
                # 줄바꿈 기준으로 메시지 분리
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if line:
                        self._process_response(line)
                        
            except socket.timeout:
                continue
                
            except Exception as e:
                if self.running:
                    print(f"[STOPPER] Receive error: {e}")
                break
    
    def _process_response(self, response: str):
        """ESP32 응답 처리"""
        timestamp = time.strftime("%H:%M:%S")
        
        if response == "ESP32_CONNECTED":
            print(f"[{timestamp}] ESP32 confirmed connection")
        elif response == "STATUS:FORWARD":
            print(f"[{timestamp}] ▶ Moving FORWARD (2 seconds)")
        elif response == "STATUS:BACKWARD":
            print(f"[{timestamp}] ◀ Moving BACKWARD (2 seconds)")
        elif response == "STATUS:STOPPED":
            print(f"[{timestamp}] ■ Motor STOPPED (manual)")
        elif response == "STATUS:TIMER_STOP":
            print(f"[{timestamp}] ⏱ Motor STOPPED (timer)")
        elif response == "STATUS:TIMEOUT":
            print(f"[{timestamp}] ⚠ Motor STOPPED (safety timeout)")
        elif response == "HEARTBEAT":
            # 하트비트는 디버그 모드에서만 표시
            pass
        else:
            print(f"[{timestamp}] ESP32: {response}")
    
    def send_command(self, command: Command) -> bool:
        """
        스토퍼에 명령 전송
        Args:
            command: Command 열거형 값
        Returns:
            성공 여부
        """
        if not self.connected:
            print("[STOPPER] Not connected. Attempting to connect...")
            if not self.connect():
                return False
        
        try:
            message = f"CMD:{command.value}\n"
            self.client_socket.send(message.encode('utf-8'))
            
            cmd_name = command.name
            print(f"[STOPPER] Sent: {cmd_name} command")
            return True
            
        except Exception as e:
            print(f"[STOPPER] Send error: {e}")
            self.disconnect()
            return False
    
    def move_forward(self) -> bool:
        """전진 명령 (2초 동작)"""
        return self.send_command(Command.FORWARD)
    
    def move_backward(self) -> bool:
        """후진 명령 (2초 동작)"""
        return self.send_command(Command.BACKWARD)
    
    def stop(self) -> bool:
        """즉시 정지 명령"""
        return self.send_command(Command.STOP)
    
    def get_status(self) -> str:
        """연결 상태 확인"""
        if self.connected:
            return f"Connected to {self.host}:{self.port}"
        else:
            return "Disconnected"


def print_menu():
    """메뉴 출력"""
    print("\n" + "="*40)
    print("ESP32 스토퍼 제어 메뉴")
    print("="*40)
    print("1. 전진 (Forward) - 2초 동작")
    print("2. 후진 (Backward) - 2초 동작")
    print("3. 정지 (Stop) - 즉시 정지")
    print("4. 연결 상태 확인")
    print("5. 재연결")
    print("6. 자동 테스트 (전진-후진 반복)")
    print("0. 종료")
    print("-"*40)


def auto_test(controller: StopperController, cycles: int = 3):
    """자동 테스트 모드"""
    print(f"\n[TEST] 자동 테스트 시작 ({cycles}회 반복)")
    
    for i in range(cycles):
        print(f"\n[TEST] Cycle {i+1}/{cycles}")
        
        # 전진
        print("[TEST] Moving forward...")
        if not controller.move_forward():
            print("[TEST] Failed to move forward")
            return
        time.sleep(3)  # 2초 동작 + 1초 대기
        
        # 후진
        print("[TEST] Moving backward...")
        if not controller.move_backward():
            print("[TEST] Failed to move backward")
            return
        time.sleep(3)  # 2초 동작 + 1초 대기
        
    print("[TEST] 자동 테스트 완료")


def main():
    """메인 함수"""
    print("ESP32 스토퍼 제어 프로그램 v1.0")
    print("="*40)
    
    # ESP32 IP 입력 (기본값 사용)
    esp_ip = input("ESP32 IP 주소 [192.168.225.99]: ").strip()
    if not esp_ip:
        esp_ip = "192.168.0.90"
    
    # 컨트롤러 생성
    controller = StopperController(host=esp_ip, port=8888)
    
    # 초기 연결
    print("\n[SYSTEM] ESP32에 연결 중...")
    controller.connect()
    time.sleep(1)
    
    # 메인 루프
    try:
        while True:
            print_menu()
            
            try:
                choice = input("선택> ").strip()
                
                if choice == '1':
                    controller.move_forward()
                    
                elif choice == '2':
                    controller.move_backward()
                    
                elif choice == '3':
                    controller.stop()
                    
                elif choice == '4':
                    status = controller.get_status()
                    print(f"[STATUS] {status}")
                    
                elif choice == '5':
                    print("[SYSTEM] 재연결 시도...")
                    controller.disconnect()
                    time.sleep(1)
                    controller.connect()
                    
                elif choice == '6':
                    cycles = input("반복 횟수 [3]: ").strip()
                    cycles = int(cycles) if cycles else 3
                    auto_test(controller, cycles)
                    
                elif choice == '0':
                    print("[SYSTEM] 프로그램 종료")
                    break
                    
                else:
                    print("[ERROR] 잘못된 선택입니다")
                    
            except KeyboardInterrupt:
                print("\n[SYSTEM] Ctrl+C 감지")
                break
                
            except Exception as e:
                print(f"[ERROR] {e}")
                
    except Exception as e:
        print(f"[FATAL] {e}")
        
    finally:
        # 정리
        print("\n[SYSTEM] 연결 해제 중...")
        controller.stop()  # 안전을 위해 정지 명령
        time.sleep(0.5)
        controller.disconnect()
        print("[SYSTEM] 프로그램 종료")


if __name__ == "__main__":
    main()