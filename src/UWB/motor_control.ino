#include <Arduino.h>
#include <WiFi.h>

// ====== 사용자 네트워크 ======
#define WIFI_SSID   "aaaa"
#define WIFI_PASS   "00000906"
#define SERVER_IP   "192.168.0.90"  // 라즈베리파이 IP 안쓰는 변수임 무시해도됨.
#define SERVER_PORT 8888

// ====== 고정 IP 설정 ======
IPAddress local_IP(192, 168, 225, 99);     // ESP32가 사용할 고정 IP (무선 LAN 어댑터 IPv4 주소 수정!!!!!!!!!)
IPAddress gateway(192, 168, 225, 250);        // 게이트웨이 (공유기 IP) (무선lan어댑터 게이트웨이주소 수정!!!!!!!!!)
IPAddress subnet(255, 255, 255, 0);       // 서브넷 마스크  
IPAddress primaryDNS(8, 8, 8, 8);         // 기본 DNS (구글 DNS)
IPAddress secondaryDNS(8, 8, 4, 4);       // 보조 DNS (구글 DNS)

// ====== 타이머 설정 ======
#define TIMER_DURATION_MS 800  

// ====== L298N 핀 설정 ======
#define PWM_PIN  14  // ENA와 ENB 둘 다 연결 (공통 PWM)
#define IN1      32  // 모터 A 방향 1
#define IN2      33  // 모터 A 방향 2
#define IN3      25  // 모터 B 방향 1
#define IN4      27  // 모터 B 방향 2

// ====== PWM 설정 ======
#define PWM_FREQ 1000   // 1kHz
#define PWM_RES  8      // 8비트 해상도 (0-255)
#define PWM_MAX  255
#define PWM_CH   0      // PWM 채널

// ====== TCP 통신 ======
WiFiServer server(SERVER_PORT);
WiFiClient client;
bool client_connected = false;

// ====== 로직 ======
enum Mode { IDLE, RUN_FWD, RUN_REV };
volatile Mode mode = IDLE;
const int SPEED = 160;  // 속도 (0-255)
unsigned long started_ms = 0;
const unsigned long MAX_RUN_MS = 10000;  // 안전 타임아웃 10초

#define LOGI(...) do { Serial.printf(__VA_ARGS__); Serial.println(); } while(0)

// ====== 모터 제어 함수들 ======
void stopMotors() {
  // 모든 방향 핀 LOW로 설정하여 완전 정지
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CH, 0);
  LOGI("[MOTOR] Stopped");
}

void driveForward() {
  LOGI("[MOTOR] Forward - speed=%d", SPEED);
  // 모터 A 전진
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // 모터 B 전진
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  // 속도 설정
  ledcWrite(PWM_CH, SPEED);
}

void driveBackward() {
  LOGI("[MOTOR] Backward - speed=%d", SPEED);
  // 모터 A 후진
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // 모터 B 후진
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  // 속도 설정
  ledcWrite(PWM_CH, SPEED);
}

// ====== 명령 처리 ======
void processCommand(String cmd) {
  cmd.trim();
  LOGI("[TCP] Received: %s", cmd.c_str());
  
  if (cmd.startsWith("CMD:")) {
    int command = cmd.substring(4).toInt();
    
    switch(command) {
      case 1:  // FORWARD
        mode = RUN_FWD;
        started_ms = millis();
        driveForward();
        LOGI("[CMD] FORWARD - will stop after %d ms", TIMER_DURATION_MS);
        if (client_connected && client) {
          client.println("STATUS:FORWARD");
        }
        break;
        
      case 2:  // BACKWARD
        mode = RUN_REV;
        started_ms = millis();
        driveBackward();
        LOGI("[CMD] BACKWARD - will stop after %d ms", TIMER_DURATION_MS);
        if (client_connected && client) {
          client.println("STATUS:BACKWARD");
        }
        break;
        
      case 0:  // STOP (수동 정지)
      default:
        mode = IDLE;
        stopMotors();
        LOGI("[CMD] Manual STOP");
        if (client_connected && client) {
          client.println("STATUS:STOPPED");
        }
        break;
    }
  }
}

// ====== TCP 서버 처리 ======
void handleTCPServer() {
  // 새로운 클라이언트 연결 확인
  if (server.hasClient()) {
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = server.available();
      client_connected = true;
      LOGI("[TCP] Client connected from %s", client.remoteIP().toString().c_str());
      client.println("ESP32_CONNECTED");
    } else {
      // 이미 클라이언트가 연결되어 있으면 새 연결 거부
      WiFiClient rejected = server.available();
      rejected.stop();
      LOGI("[TCP] Client rejected - already connected");
    }
  }
  
  // 연결된 클라이언트로부터 데이터 수신
  if (client && client.connected()) {
    if (client.available()) {
      String received = client.readStringUntil('\n');
      processCommand(received);
    }
  } else if (client_connected) {
    // 클라이언트 연결이 끊어짐
    client_connected = false;
    LOGI("[TCP] Client disconnected");
    // 안전을 위해 모터 정지
    mode = IDLE;
    stopMotors();
  }
}

void checkStopConditions() {
  // 동작 중이 아니면 체크 불필요
  if (mode == IDLE) {
    return;
  }
  
  // 안전 타임아웃 체크
  if (millis() - started_ms > MAX_RUN_MS) {
    LOGI("[SAFE] Safety timeout -> STOP");
    mode = IDLE;
    stopMotors();
    if (client_connected && client) {
      client.println("STATUS:TIMEOUT");
    }
    return;
  }
  
  if (millis() - started_ms >= TIMER_DURATION_MS) {
    LOGI("[TIMER] %d ms reached -> STOP", TIMER_DURATION_MS);
    mode = IDLE;
    stopMotors();
    if (client_connected && client) {
      client.println("STATUS:TIMER_STOP");
    }
  }
}

// ====== Arduino Setup ======
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  LOGI("===== ESP32 + L298N Timer Control =====");
  LOGI("[MODE] Timer Stop: %d ms", TIMER_DURATION_MS);
  LOGI("[PINS] PWM=%d, IN1=%d, IN2=%d, IN3=%d, IN4=%d", 
       PWM_PIN, IN1, IN2, IN3, IN4);
  
  // L298N 핀 초기화
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // PWM 채널 설정 (ENA와 ENB 공통)
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CH);
  
  // 초기 상태: 모터 정지
  stopMotors();
  
  // WiFi 연결
  WiFi.mode(WIFI_STA);
  
  // 고정 IP 설정
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    LOGI("[WIFI] Failed to configure static IP");
  } else {
    LOGI("[WIFI] Static IP configured: %s", local_IP.toString().c_str());
  }
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  LOGI("[WIFI] Connecting to %s ...", WIFI_SSID);
  
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    LOGI("[WIFI] Connected");
    LOGI("[WIFI] IP: %s", WiFi.localIP().toString().c_str());
    LOGI("[WIFI] Gateway: %s", WiFi.gatewayIP().toString().c_str());
    LOGI("[WIFI] Subnet: %s", WiFi.subnetMask().toString().c_str());
    
    // TCP 서버 시작
    server.begin();
    LOGI("[TCP] Server started on port %d", SERVER_PORT);
    LOGI("[TCP] Waiting for client connection...");
  } else {
    LOGI("[WIFI] Connection failed - restarting...");
    delay(2000);
    ESP.restart();
  }
  
  LOGI("[INIT] System ready");
  LOGI("=====================================");
}

void loop() {
  // TCP 서버 처리
  handleTCPServer();
  checkStopConditions();
  
  // 상태 출력 (디버그용)
  static unsigned long last_status = 0;
  if (millis() - last_status > 5000) {  // 5초마다
    last_status = millis();
    if (mode == IDLE) {
      LOGI("[STATUS] IDLE - Waiting for command");
    } else if (mode == RUN_FWD) {
      unsigned long elapsed = millis() - started_ms;
      LOGI("[STATUS] FORWARD - %lu ms elapsed", elapsed);
    } else if (mode == RUN_REV) {
      unsigned long elapsed = millis() - started_ms;
      LOGI("[STATUS] BACKWARD - %lu ms elapsed", elapsed);
    }
  }
  
  delay(10);  // CPU 부하 감소
}