#include <Arduino.h>
#include <math.h>

#define BAUD_ROS_BRIDGE 115200
#define BAUD_ANCHOR     115200

#define MEDIAN_WINDOW_SIZE 5      
#define KALMAN_PROCESS_NOISE 0.01 
#define KALMAN_MEASURE_NOISE 0.5 
#define OUTLIER_THRESHOLD 30.0    
#define RANSAC_INLIER_THRESH 15.0 

const uint8_t STX = 20;
const uint8_t ETX = 21;

enum : uint8_t {
  CMD_SET_ANCHOR_NUM   = 0x01,
  CMD_SET_TAG_NUM      = 0x02,
  CMD_GET_INFO         = 0x03,
  CMD_MEASURE_TAG_ONCE = 0x04,
  CMD_MEASURE_TAG_CONT = 0x05,
  CMD_MEASURE_ALL_ONCE = 0x06,
  CMD_MEASURE_ALL_CONT = 0x07,
  CMD_SET_ANCHOR1_XY   = 0x08,
  CMD_SET_ANCHOR2_XY   = 0x09,
  CMD_SET_ANCHOR3_XY   = 0x0A,
  CMD_MEASURE_AX_TAG   = 0x0B
};

enum SegType : uint8_t { SEG_VERT, SEG_HORIZ };

struct AxisSeg {
  SegType type;
  float fixed; 
  float r1;    
  float r2;   
};

static AxisSeg CORRIDORS[] = {
  {SEG_VERT,  20.0f,  0.0f,  180.0f},   
  {SEG_VERT,  55.0f,  140.0f,  180.0f},  
  {SEG_VERT,  85.0f,  140.0f,  180.0f},   
  {SEG_VERT,  115.0f,  140.0f,  180.0f},   
  {SEG_VERT,  145.0f,  140.0f,  180.0f},  
  {SEG_VERT, 55.0f,  60.0f,  100.0f},
  {SEG_VERT, 85.0f,  60.0f,  100.0f},
  {SEG_VERT, 115.0f,  60.0f,  100.0f},
  {SEG_VERT, 145.0f,  60.0f,  100.0f},
  {SEG_HORIZ, 147.0f,  20.0f, 175.0f},  
  {SEG_HORIZ, 100.0f,  20.0f, 175.0f},   
};
static const size_t N_CORRIDORS = sizeof(CORRIDORS) / sizeof(CORRIDORS[0]);

static inline float fclamp(float v, float a, float b) {
  if (a > b) { float t=a; a=b; b=t; }
  return (v < a) ? a : (v > b) ? b : v;
}

static inline void snapToCorridors(float x, float y, float &sx, float &sy) {
  if (N_CORRIDORS == 0) { sx = x; sy = y; return; }

  float best_dx = 0, best_dy = 0;
  float best_d2 = INFINITY;
  float cand_x, cand_y;

  for (size_t i = 0; i < N_CORRIDORS; ++i) {
    const AxisSeg &s = CORRIDORS[i];
    if (s.type == SEG_VERT) {
      cand_x = s.fixed;
      cand_y = fclamp(y, s.r1, s.r2);
    } else { // SEG_HORIZ
      cand_y = s.fixed;
      cand_x = fclamp(x, s.r1, s.r2);
    }
    float dx = x - cand_x;
    float dy = y - cand_y;
    float d2 = dx*dx + dy*dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_dx = cand_x;
      best_dy = cand_y;
    }
  }
  sx = best_dx; sy = best_dy;
}

struct Vec3 { float x, y, z; };

float anchor_H = 100.0f;  // 앵커 높이 (cm)
float TAG_Z_CM = 13.0f;   // 태그 높이 (cm)

// 앵커 좌표 (cm) 시연 환경에맞게 수정 필요
const Vec3 ANCHOR[3] = {
  { -99.0f,  -9.0f, anchor_H },  // Anchor#1 on Serial1
  {  89.0f,   226.0f, anchor_H },  // Anchor#2 on Serial2
  {277.0f,   -11.0f, anchor_H }   // Anchor#3 on Serial3
};

// ---------- 칼만 필터 구조체 ----------
struct KalmanFilter2D {
  float x[4];
  float P[4][4];
  float Q;
  float R;
  float dt;
  
  void init(float init_x, float init_y, float process_noise, float measure_noise) {
    x[0] = init_x; x[1] = init_y; x[2] = 0; x[3] = 0;
    Q = process_noise;
    R = measure_noise;
    dt = 0.1; 
    
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        P[i][j] = (i == j) ? 1000.0 : 0.0;
      }
    }
  }
  
  void predict() {
    // 상태 예측: x = F * x
    float new_x[4];
    new_x[0] = x[0] + x[2] * dt;
    new_x[1] = x[1] + x[3] * dt;
    new_x[2] = x[2];
    new_x[3] = x[3];
    
    for(int i = 0; i < 4; i++) x[i] = new_x[i];
    
    // 공분산 예측: P = F * P * F' + Q
    float F[4][4] = {
      {1, 0, dt, 0},
      {0, 1, 0, dt},
      {0, 0, 1, 0},
      {0, 0, 0, 1}
    };
    
    float temp[4][4];
    // temp = F * P
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        temp[i][j] = 0;
        for(int k = 0; k < 4; k++) {
          temp[i][j] += F[i][k] * P[k][j];
        }
      }
    }
    
    // P = temp * F' + Q
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        P[i][j] = 0;
        for(int k = 0; k < 4; k++) {
          P[i][j] += temp[i][k] * F[j][k];
        }
        if(i == j) P[i][j] += Q;
      }
    }
  }
  
  void update(float meas_x, float meas_y) {
    // 칼만 이득 계산: K = P * H' * (H * P * H' + R)^-1
    // H = [1 0 0 0; 0 1 0 0]
    float S[2][2];  // H * P * H' + R
    S[0][0] = P[0][0] + R;
    S[0][1] = P[0][1];
    S[1][0] = P[1][0];
    S[1][1] = P[1][1] + R;
    
    // S의 역행렬
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if(fabs(det) < 1e-6) return;
    
    float S_inv[2][2];
    S_inv[0][0] = S[1][1] / det;
    S_inv[0][1] = -S[0][1] / det;
    S_inv[1][0] = -S[1][0] / det;
    S_inv[1][1] = S[0][0] / det;
    
    // 칼만 이득 K
    float K[4][2];
    K[0][0] = P[0][0] * S_inv[0][0] + P[0][1] * S_inv[1][0];
    K[0][1] = P[0][0] * S_inv[0][1] + P[0][1] * S_inv[1][1];
    K[1][0] = P[1][0] * S_inv[0][0] + P[1][1] * S_inv[1][0];
    K[1][1] = P[1][0] * S_inv[0][1] + P[1][1] * S_inv[1][1];
    K[2][0] = P[2][0] * S_inv[0][0] + P[2][1] * S_inv[1][0];
    K[2][1] = P[2][0] * S_inv[0][1] + P[2][1] * S_inv[1][1];
    K[3][0] = P[3][0] * S_inv[0][0] + P[3][1] * S_inv[1][0];
    K[3][1] = P[3][0] * S_inv[0][1] + P[3][1] * S_inv[1][1];
    
    // 상태 업데이트: x = x + K * (z - H * x)
    float y_err[2];
    y_err[0] = meas_x - x[0];
    y_err[1] = meas_y - x[1];
    
    for(int i = 0; i < 4; i++) {
      x[i] += K[i][0] * y_err[0] + K[i][1] * y_err[1];
    }
    
    // 공분산 업데이트: P = (I - K * H) * P
    float I_KH[4][4];
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        I_KH[i][j] = (i == j) ? 1.0 : 0.0;
        if(i < 2 && j < 2) {
          I_KH[i][j] -= K[i][j];
        }
      }
    }
    
    float new_P[4][4];
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        new_P[i][j] = 0;
        for(int k = 0; k < 4; k++) {
          new_P[i][j] += I_KH[i][k] * P[k][j];
        }
      }
    }
    
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        P[i][j] = new_P[i][j];
      }
    }
  }
  
  void getPosition(float& out_x, float& out_y) {
    out_x = x[0];
    out_y = x[1];
  }
};

// ---------- 중간값 필터 ----------
class MedianFilter {
  float buffer[MEDIAN_WINDOW_SIZE];
  int index;
  int count;
  
public:
  MedianFilter() : index(0), count(0) {}
  
  float filter(float value) {
    buffer[index] = value;
    index = (index + 1) % MEDIAN_WINDOW_SIZE;
    if(count < MEDIAN_WINDOW_SIZE) count++;
    
    float sorted[MEDIAN_WINDOW_SIZE];
    for(int i = 0; i < count; i++) {
      sorted[i] = buffer[i];
    }
    
    // 버블 정렬
    for(int i = 0; i < count - 1; i++) {
      for(int j = 0; j < count - i - 1; j++) {
        if(sorted[j] > sorted[j + 1]) {
          float temp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = temp;
        }
      }
    }
    
    // 중간값 반환
    if(count > 0) {
      return sorted[count / 2];
    }
    return value;
  }
  
  void reset() {
    index = 0;
    count = 0;
  }
};


struct TagTracker {
  bool active;
  uint8_t tag_id;
  uint32_t last_measure_time;
  uint16_t measure_interval_ms;
  uint16_t last_distances[3];
  bool last_valid[3];
  
  // 필터링 추가
  KalmanFilter2D kalman;
  MedianFilter median_filters[3];  // 각 앵커별 중간값 필터
  float last_x, last_y;             // 마지막 위치
  bool position_initialized;
  float distance_variance[3];       // 거리 측정 분산 (적응형 필터용)
  float distance_history[3][10];    // 거리 히스토리
  int history_index;
  
    // >>> 추가: raw≤50000일 때 마지막으로 보낸 좌표를 저장
  bool has_last_good = false;
  float last_good_x = 0.0f;
  float last_good_y = 0.0f;

  void init(uint8_t id) {
    tag_id = id;
    last_x = 70.0;  // 초기 위치 추정값
    last_y = 50.0;
    position_initialized = false;
    history_index = 0;
    kalman.init(last_x, last_y, KALMAN_PROCESS_NOISE, KALMAN_MEASURE_NOISE);
    
    for(int i = 0; i < 3; i++) {
      distance_variance[i] = 0;
      for(int j = 0; j < 10; j++) {
        distance_history[i][j] = 0;
      }
    }
        // >>> 추가: 초기화 시 저장 좌표 리셋
    has_last_good = false;
    last_good_x = last_x;
    last_good_y = last_y;
  
  }
  
  void updateDistanceHistory(int anchor_idx, float distance) {
    distance_history[anchor_idx][history_index % 10] = distance;
    
    // 분산 계산 (10개 샘플 이상일 때)
    if(history_index >= 9) {
      float mean = 0;
      for(int i = 0; i < 10; i++) {
        mean += distance_history[anchor_idx][i];
      }
      mean /= 10.0;
      
      float variance = 0;
      for(int i = 0; i < 10; i++) {
        float diff = distance_history[anchor_idx][i] - mean;
        variance += diff * diff;
      }
      distance_variance[anchor_idx] = variance / 10.0;
    }
  }
  
  bool isOutlier(int anchor_idx, float distance) {
    // 적응형 이상치 검출
    if(history_index < 9) return false;
    
    float mean = 0;
    for(int i = 0; i < 10; i++) {
      mean += distance_history[anchor_idx][i];
    }
    mean /= 10.0;
    
    // 동적 임계값: 분산 기반
    float dynamic_threshold = max(OUTLIER_THRESHOLD, 3.0 * sqrt(distance_variance[anchor_idx]));
    
    return fabs(distance - mean) > dynamic_threshold;
  }
};

#define MAX_TAGS 10
TagTracker trackers[MAX_TAGS];

static bool horizRadius(float d, float anchorZ, float tagZ, float &r);
static bool trilat2D(const Vec3 a[3], const float r[3], float &x, float &y);

void sendDebug(const String& msg) {
  Serial.print("D,");
  Serial.println(msg);
}

void sendPosition(uint8_t tag_id, float x, float y, float z) {
  Serial.print("P,");
  Serial.print(tag_id);
  Serial.print(",");
  Serial.print((int)x);
  Serial.print(",");
  Serial.print((int)y);
  Serial.print(",");
  Serial.println((int)z);
}

static inline void writeS16(HardwareSerial& s, int16_t v) {
  s.write((uint8_t)(v & 0xFF));
  s.write((uint8_t)((uint16_t)v >> 8));
}

static inline void sendFrame(HardwareSerial& s, uint8_t cmd) {
  s.write(STX); s.write(cmd); s.write(ETX);
}

static inline void sendFrame1(HardwareSerial& s, uint8_t cmd, uint8_t v) {
  s.write(STX); s.write(cmd); s.write(v); s.write(ETX);
}

static inline void sendSetXY(HardwareSerial& s, uint8_t cmd, uint8_t anchorId, int16_t x, int16_t y) {
  s.write(STX);
  s.write(cmd);
  s.write(anchorId);
  writeS16(s, x);   // 부호 있는 16비트 그대로 전송
  writeS16(s, y);
  s.write(ETX);
}

static bool readDistanceFrame(HardwareSerial& s, uint16_t& out_cm, uint32_t timeout_ms=250) {
  uint32_t t0 = millis();
  bool inFrame = false;
  uint8_t buf[8]; 
  uint8_t idx = 0;

  while ((millis() - t0) < timeout_ms) {
    if (!s.available()) { 
      delay(1); 
      continue;
    }
    
    uint8_t ch = (uint8_t)s.read();

    if (!inFrame) {
      if (ch == STX) { 
        inFrame = true; 
        idx = 0; 
        buf[idx++] = ch; 
      }
      continue;
    }

    buf[idx++] = ch;
    if (idx >= sizeof(buf)) { 
      inFrame = false; 
      idx = 0; 
      continue;
    }
    
    if (ch == ETX) {
      if (idx == 5 && buf[1] == CMD_MEASURE_TAG_ONCE) {
        out_cm = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        if (out_cm < anchor_H) { 
          out_cm = anchor_H + 10;
        }
        return true;
      }
      inFrame = false; 
      idx = 0;
    }
  }
  return false;
}

static bool requestDistanceOnce(HardwareSerial& s, uint8_t tagId, uint16_t& dist_cm) {
  while (s.available()) (void)s.read();
  sendFrame1(s, CMD_MEASURE_TAG_ONCE, tagId);
  return readDistanceFrame(s, dist_cm, 400);
}

// 3D→수평(r) 보정: r = sqrt(d^2 - dz^2)
static bool horizRadius(float d, float anchorZ, float tagZ, float &r) {
  float dz = anchorZ - tagZ;
  float h2 = d*d - dz*dz;
  
  if (h2 <= 0.0f) { 
    r = 0.0f; 
    return false; 
  }
  
  r = sqrtf(h2);
  return true;
}

// 2D 삼변측량
static bool trilat2D(const Vec3 a[3], const float r[3], float &x, float &y) {
  float A11 = 2.0f*(a[1].x - a[0].x), A12 = 2.0f*(a[1].y - a[0].y);
  float A21 = 2.0f*(a[2].x - a[0].x), A22 = 2.0f*(a[2].y - a[0].y);
  float b1  = (r[0]*r[0] - r[1]*r[1])
            + (a[1].x*a[1].x - a[0].x*a[0].x)
            + (a[1].y*a[1].y - a[0].y*a[0].y);
  float b2  = (r[0]*r[0] - r[2]*r[2])
            + (a[2].x*a[2].x - a[0].x*a[0].x)
            + (a[2].y*a[2].y - a[0].y*a[0].y);
  float det = A11*A22 - A12*A21;
  
  if (fabsf(det) < 1e-6f) return false;
  
  x = ( b1*A22 - A12*b2) / det;
  y = (-b1*A21 + A11*b2) / det;
  return true;
}

static void configureAnchors() {
  sendDebug("Starting anchor configuration");
 
  sendFrame1(Serial1, CMD_SET_ANCHOR_NUM, 1);
  sendFrame1(Serial2, CMD_SET_ANCHOR_NUM, 2);
  sendFrame1(Serial3, CMD_SET_ANCHOR_NUM, 3);
  delay(30);
  sendDebug("Anchor mode and ID set completed");

  sendSetXY(Serial1, CMD_SET_ANCHOR1_XY, 1, (int16_t)lroundf(ANCHOR[0].x), (int16_t)lroundf(ANCHOR[0].y));
  sendSetXY(Serial2, CMD_SET_ANCHOR2_XY, 2, (int16_t)lroundf(ANCHOR[1].x), (int16_t)lroundf(ANCHOR[1].y));
  sendSetXY(Serial3, CMD_SET_ANCHOR3_XY, 3, (int16_t)lroundf(ANCHOR[2].x), (int16_t)lroundf(ANCHOR[2].y));
  delay(30);
  sendDebug("Anchor XY coordinates set completed");
}

void measureTag(uint8_t tag_id) {
  uint16_t raw_d_cm[3] = {0, 0, 0};
  float filtered_d_cm[3] = {0, 0, 0};
  bool ok[3] = {false, false, false};
  
  int tracker_idx = tag_id - 10;
  if (tracker_idx < 0 || tracker_idx >= MAX_TAGS) return;
  
  TagTracker& tracker = trackers[tracker_idx];
  
// 각 앵커에서 거리 측정
  ok[0] = requestDistanceOnce(Serial1, tag_id, raw_d_cm[0]);
  ok[1] = requestDistanceOnce(Serial2, tag_id, raw_d_cm[1]);
  ok[2] = requestDistanceOnce(Serial3, tag_id, raw_d_cm[2]);

  // raw 중 하나라도 50000 이상이면 마지막 정상 좌표 전송 후 반환
  bool raw_overflow = false;
  for (int i = 0; i < 3; i++) {
    if (ok[i] && raw_d_cm[i] >= 50000) { raw_overflow = true; break; }
  }
  if (raw_overflow) {
    if (tracker.has_last_good) {
      sendPosition(tag_id, tracker.last_good_x, tracker.last_good_y, TAG_Z_CM);
      sendDebug("TAG" + String(tag_id) + " raw>=50000 detected; sending last good position");
    } else if (tracker.position_initialized) {
      // 마지막 정상 좌표가 아직 없으면 기존 예측값으로라도 전달(통신 포맷 유지)
      tracker.kalman.predict();
      float px, py; tracker.kalman.getPosition(px, py);
      sendPosition(tag_id, px, py, TAG_Z_CM);
      sendDebug("TAG" + String(tag_id) + " raw>=50000 and no last-good; using predicted position");
    } else {
      sendDebug("TAG" + String(tag_id) + " raw>=50000 and no position available yet");
      
    }
    return;
  }
  
  // 중간값 필터 적용 및 이상치 검출
  int valid_count = 0;
  for(int i = 0; i < 3; i++) {
    if(ok[i]) {
      // 중간값 필터
      filtered_d_cm[i] = tracker.median_filters[i].filter((float)raw_d_cm[i]);
      
      // 히스토리 업데이트
      tracker.updateDistanceHistory(i, filtered_d_cm[i]);
      
      // 이상치 검출
      if(tracker.isOutlier(i, filtered_d_cm[i])) {
        sendDebug("TAG" + String(tag_id) + " A" + String(i+1) + 
                  " outlier detected: " + String(filtered_d_cm[i]) + "cm");
        ok[i] = false;
      } else {
        valid_count++;
      }
    }
  }
  
  // 트래커 업데이트
  for (int i = 0; i < 3; i++) {
    tracker.last_distances[i] = filtered_d_cm[i];
    tracker.last_valid[i] = ok[i];
  }
  tracker.history_index++;
  

  String dist_msg = "TAG" + String(tag_id) + " filtered: ";
  for (int i = 0; i < 3; i++) {
    if (ok[i]) {
      dist_msg += "A" + String(i+1) + "=" + String((int)filtered_d_cm[i]) + 
                  "cm(raw:" + String(raw_d_cm[i]) + ") ";
    } else {
      dist_msg += "A" + String(i+1) + "=INVALID ";
    }
  }
  sendDebug(dist_msg);
  
  // 최소 3개의 유효한 측정값 필요
  if (valid_count < 3) {
    sendDebug("TAG" + String(tag_id) + " insufficient valid measurements");
    
    // 칼만 필터 예측만 수행
    if(tracker.position_initialized) {
      tracker.kalman.predict();
      float pred_x, pred_y;
      tracker.kalman.getPosition(pred_x, pred_y);
      sendPosition(tag_id, pred_x, pred_y, TAG_Z_CM);
      sendDebug("TAG" + String(tag_id) + " using predicted position");
    }
    return;
  }
  
  // 3D→수평 반지름 보정
  float r[3];
  bool good = true;
  
  for (int i = 0; i < 3; i++) {
    if(ok[i]) {
      if (!horizRadius(filtered_d_cm[i], ANCHOR[i].z, TAG_Z_CM, r[i])) {
        sendDebug("TAG" + String(tag_id) + " negative h2 at anchor" + String(i+1));
        good = false;
        break;
      }
    }
  }
  
  if (!good) {
    sendDebug("TAG" + String(tag_id) + " horizontal radius calculation failed");
    return;
  }
  
  // 2D Trilateration
  float x, y;
  if (!trilat2D(ANCHOR, r, x, y)) {
    sendDebug("TAG" + String(tag_id) + " trilateration failed (det≈0)");
    return;
  }
  
  // 칼만 필터 업데이트
  if(!tracker.position_initialized) {
    // 첫 번째 측정: 칼만 필터 초기화
    tracker.kalman.init(x, y, KALMAN_PROCESS_NOISE, KALMAN_MEASURE_NOISE);
    tracker.position_initialized = true;
    tracker.last_x = x;
    tracker.last_y = y;
  } else {
    // 위치 점프 체크 (너무 큰 변화는 무시)
    float jump = sqrt((x - tracker.last_x) * (x - tracker.last_x) + 
                     (y - tracker.last_y) * (y - tracker.last_y));
    
    if(jump > 100.0) {  // 100cm 이상 점프는 이상치로 간주
      sendDebug("TAG" + String(tag_id) + " position jump detected: " + String(jump) + "cm");
      
      // 예측만 수행
      tracker.kalman.predict();
    } else {
      // 정상적인 업데이트
      tracker.kalman.predict();
      tracker.kalman.update(x, y);
    }
  }
  
// 필터링된 최종 위치
  float final_x, final_y;
  tracker.kalman.getPosition(final_x, final_y);

  // 위치 업데이트(필터 내부 상태 추적용, 스냅 전 좌표 유지)
  tracker.last_x = final_x;
  tracker.last_y = final_y;

  //세그먼트로 스냅
  float snapped_x = final_x, snapped_y = final_y;
  snapToCorridors(final_x, final_y, snapped_x, snapped_y);

  //last_good은 스냅된 좌표로 저장
  bool all_raw_ok =
      (ok[0] && ok[1] && ok[2]) &&
      (raw_d_cm[0] < 50000 && raw_d_cm[1] < 50000 && raw_d_cm[2] < 50000);
  if (all_raw_ok) {
    tracker.last_good_x = snapped_x;
    tracker.last_good_y = snapped_y;
    tracker.has_last_good = true;
  }

  // 스냅된 좌표 사용 
  sendPosition(tag_id, snapped_x, snapped_y, TAG_Z_CM);

  // 디버깅: 최종 좌표(스냅 전/후 비교)
  sendDebug("TAG" + String(tag_id) + 
            " final position: x=" + String((int)final_x) + "cm, y=" + String((int)final_y) + 
            "cm -> snapped: x=" + String((int)snapped_x) + "cm, y=" + String((int)snapped_y) + "cm");
}

// ---------- ROS Bridge 명령 처리 ----------
void processRosBridgeCommand() {
  static char cmdBuf[64];
  static uint8_t cmdLen = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        
        // TS,tag_id - 추적 시작
        if (strncmp(cmdBuf, "TS,", 3) == 0) {
          int tag_id = atoi(cmdBuf + 3);
          if (tag_id >= 10 && tag_id <= 19) {
            int idx = tag_id - 10;
            trackers[idx].init(tag_id);  // 필터 초기화 추가
            trackers[idx].active = true;
            trackers[idx].tag_id = tag_id;
            trackers[idx].last_measure_time = 0;
            trackers[idx].measure_interval_ms = 100;  // 100ms 주기
            sendDebug("TAG" + String(tag_id) + " tracking started with filters initialized");
          } else {
            sendDebug("Invalid tag_id for TS command: " + String(tag_id));
          }
        }
        // TE,tag_id - 추적 종료
        else if (strncmp(cmdBuf, "TE,", 3) == 0) {
          int tag_id = atoi(cmdBuf + 3);
          if (tag_id >= 10 && tag_id <= 19) {
            int idx = tag_id - 10;
            trackers[idx].active = false;
            // 필터 리셋
            for(int i = 0; i < 3; i++) {
              trackers[idx].median_filters[i].reset();
            }
            sendDebug("TAG" + String(tag_id) + " tracking stopped and filters reset");
          } else {
            sendDebug("Invalid tag_id for TE command: " + String(tag_id));
          }
        }
        // 알 수 없는 명령
        else {
          sendDebug("Unknown command: " + String(cmdBuf));
        }
        
        cmdLen = 0;
      }
    } else {
      if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = c;
      }
    }
  }
}

// ---------- Arduino 메인 ----------
void setup() {
  // 시리얼 초기화
  Serial.begin(BAUD_ROS_BRIDGE);   // ESP32 ROS Bridge
  Serial1.begin(BAUD_ANCHOR);      // Anchor 1
  Serial2.begin(BAUD_ANCHOR);      // Anchor 2
  Serial3.begin(BAUD_ANCHOR);      // Anchor 3
  
  delay(500);
  
  // 트래커 초기화 (필터 포함)
  for (int i = 0; i < MAX_TAGS; i++) {
    trackers[i].active = false;
    trackers[i].tag_id = 10 + i;
    trackers[i].measure_interval_ms = 100;
    trackers[i].init(10 + i);  // 필터 초기화
  }
  
  // 앵커 설정
  configureAnchors();
  
  sendDebug("Arduino Mega UWB System Ready with Advanced Filtering");
  sendDebug("Filters: Kalman + Median + Adaptive Outlier Detection");
  sendDebug("Anchors configured at A1(" + String(ANCHOR[0].x) + "," + String(ANCHOR[0].y) + "," + String(ANCHOR[0].z) + 
            ") A2(" + String(ANCHOR[1].x) + "," + String(ANCHOR[1].y) + "," + String(ANCHOR[1].z) + 
            ") A3(" + String(ANCHOR[2].x) + "," + String(ANCHOR[2].y) + "," + String(ANCHOR[2].z) + ") cm");
}

void loop() {
  // ROS Bridge 명령 처리
  processRosBridgeCommand();
  
  // 활성 태그들 주기적 측정
  uint32_t now = millis();
  for (int i = 0; i < MAX_TAGS; i++) {
    if (trackers[i].active) {
      if (now - trackers[i].last_measure_time >= trackers[i].measure_interval_ms) {
        measureTag(trackers[i].tag_id);
        trackers[i].last_measure_time = now;
      }
    }
  }
  
  delay(5);  // CPU 부하 감소
}
