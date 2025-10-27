

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/point_stamped.h>

#define MEGA_SERIAL Serial2
#define MEGA_BAUD 115200
#define RX2 16  // ESP32 RX2 pin
#define TX2 17  // ESP32 TX2 pin

#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ROS2 객체들
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Publishers
rcl_publisher_t pos_publisher;
rcl_publisher_t debug_publisher;

// Subscribers
rcl_subscription_t track_start_sub;
rcl_subscription_t track_stop_sub;

// Messages
geometry_msgs__msg__PointStamped pos_msg;
std_msgs__msg__String debug_msg;
std_msgs__msg__String track_start_msg;
std_msgs__msg__String track_stop_msg;

// 메시지 버퍼
char debug_buffer[256];
char track_buffer[32];

// ---------- 에러 처리 ----------
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// 메시지 형식: "vehicle_id,tag_id"
void track_start_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  // 파싱: vehicle_id,tag_id
  char* str = (char*)msg->data.data;
  char* comma = strchr(str, ',');
  
  if (comma != NULL) {
    *comma = '\0';
    // vehicle_id는 일단 무시하고 tag_id만 사용
    int tag_id = atoi(comma + 1);
    
    // Arduino Mega로 TS 명령 전송
    MEGA_SERIAL.print("TS,");
    MEGA_SERIAL.println(tag_id);
    
    // 디버깅
    snprintf(debug_buffer, sizeof(debug_buffer), 
             "[ROS_BRIDGE] Received track_start for tag_id=%d", tag_id);
    debug_msg.data.data = debug_buffer;
    debug_msg.data.size = strlen(debug_buffer);
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
  }
}

// 메시지 형식: "vehicle_id,tag_id"
void track_stop_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  // 파싱: vehicle_id,tag_id
  char* str = (char*)msg->data.data;
  char* comma = strchr(str, ',');
  
  if (comma != NULL) {
    *comma = '\0';
    // vehicle_id는 일단 무시하고 tag_id만 사용
    int tag_id = atoi(comma + 1);
    
    // Arduino Mega로 TE 명령 전송
    MEGA_SERIAL.print("TE,");
    MEGA_SERIAL.println(tag_id);
    
    // 디버깅
    snprintf(debug_buffer, sizeof(debug_buffer), 
             "[ROS_BRIDGE] Received track_stop for tag_id=%d", tag_id);
    debug_msg.data.data = debug_buffer;
    debug_msg.data.size = strlen(debug_buffer);
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
  }
}

void processMegaSerial() {
  static char serialBuf[256];
  static uint8_t bufLen = 0;
  
  while (MEGA_SERIAL.available()) {
    char c = MEGA_SERIAL.read();
    
    if (c == '\n' || c == '\r') {
      if (bufLen > 0) {
        serialBuf[bufLen] = '\0';
        
        // P,tag_id,x_cm,y_cm,z_cm - 좌표 데이터
        if (serialBuf[0] == 'P' && serialBuf[1] == ',') {
          // 파싱
          int tag_id = 0, x_cm = 0, y_cm = 0, z_cm = 0;
          if (sscanf(serialBuf + 2, "%d,%d,%d,%d", &tag_id, &x_cm, &y_cm, &z_cm) == 4) {
            // frame_id에 태그 ID 포함
            char frame_id_buf[32];
            snprintf(frame_id_buf, sizeof(frame_id_buf), "tag_%d", tag_id);
            
            // PointStamped 메시지 생성
            pos_msg.header.frame_id.data = frame_id_buf;
            pos_msg.header.frame_id.size = strlen(frame_id_buf);
            pos_msg.header.stamp.sec = 0;  // 시간은 ROS2에서 자동 설정
            pos_msg.header.stamp.nanosec = 0;
            
            // cm를 m로 변환
            pos_msg.point.x = x_cm / 100.0;
            pos_msg.point.y = y_cm / 100.0;
            pos_msg.point.z = z_cm / 100.0;
            
            // 발행
            RCSOFTCHECK(rcl_publish(&pos_publisher, &pos_msg, NULL));
            
            // 디버깅
            snprintf(debug_buffer, sizeof(debug_buffer), 
                     "[ROS_BRIDGE] Published position - tag:%d x:%.2fm y:%.2fm z:%.2fm", 
                     tag_id, pos_msg.point.x, pos_msg.point.y, pos_msg.point.z);
            debug_msg.data.data = debug_buffer;
            debug_msg.data.size = strlen(debug_buffer);
            RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
          }
        }
        // D,message - 디버깅 메시지
        else if (serialBuf[0] == 'D' && serialBuf[1] == ',') {
          // 디버깅 메시지 그대로 전달
          snprintf(debug_buffer, sizeof(debug_buffer), 
                   "[MEGA] %s", serialBuf + 2);
          debug_msg.data.data = debug_buffer;
          debug_msg.data.size = strlen(debug_buffer);
          RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
        }
        
        bufLen = 0;
      }
    } else {
      if (bufLen < sizeof(serialBuf) - 1) {
        serialBuf[bufLen++] = c;
      }
    }
  }
}

void setup() {
  // LED 초기화
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // UART 초기화 (Arduino Mega와 통신)
  MEGA_SERIAL.begin(MEGA_BAUD, SERIAL_8N1, RX2, TX2);
  
  // Micro-ROS 초기화
  set_microros_transports();
  
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  
  // init_options 생성
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // 노드 생성
  RCCHECK(rclc_node_init_default(&node, "uwb_bridge_node", "", &support));
  
  // Publisher 생성 - /uwb/pos
  RCCHECK(rclc_publisher_init_default(
    &pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped),
    "/uwb/pos"));
  
  // Publisher 생성 - /uwb/debug
  RCCHECK(rclc_publisher_init_default(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/uwb/debug"));
  
  // Subscriber 생성 - /uwb/track_start
  RCCHECK(rclc_subscription_init_default(
    &track_start_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/uwb/track_start"));
  
  // Subscriber 생성 - /uwb/track_stop
  RCCHECK(rclc_subscription_init_default(
    &track_stop_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/uwb/track_stop"));
  
  // Executor 생성 (2개 subscription)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &track_start_sub, &track_start_msg, 
                                         &track_start_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &track_stop_sub, &track_stop_msg, 
                                         &track_stop_callback, ON_NEW_DATA));
  
  // 메시지 초기화
  debug_msg.data.data = debug_buffer;
  debug_msg.data.capacity = sizeof(debug_buffer);
  debug_msg.data.size = 0;
  
  track_start_msg.data.data = track_buffer;
  track_start_msg.data.capacity = sizeof(track_buffer);
  track_start_msg.data.size = 0;
  
  track_stop_msg.data.data = track_buffer;
  track_stop_msg.data.capacity = sizeof(track_buffer);
  track_stop_msg.data.size = 0;
  
  pos_msg.header.frame_id.data = (char*)malloc(32);
  pos_msg.header.frame_id.capacity = 32;
  
  // 시작 메시지
  snprintf(debug_buffer, sizeof(debug_buffer), 
           "[ROS_BRIDGE] ESP32 UWB ROS Bridge initialized");
  debug_msg.data.data = debug_buffer;
  debug_msg.data.size = strlen(debug_buffer);
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
  
  digitalWrite(LED_PIN, LOW);  // LED OFF - 정상 동작
}

// ---------- Loop ----------
void loop() {
  // Arduino Mega로부터 시리얼 데이터 처리
  processMegaSerial();
  
  // ROS2 executor 실행
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  delay(5);  // CPU 부하 감소
}