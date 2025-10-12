// ===== Arduino NANO -> DE2405C =====
// Non-Blocking Stepper (Trapezoid + proper SOFTSTOP + ENA) + W5500 UDP Control + STATUS TX
//
// UDP command mapping (ASCII):
//   -1 : close drawer (CLOSE_STEPS, -dir)
//    0 : hard STOP immediately
//    1 : open  drawer (OPEN_STEPS, +dir)
//    2 : SOFTSTOP (decelerate smoothly to zero)
// others: ignored
//
// STATUS (Int32 as ASCII) mapping:
//   0 IDLE_DISABLED, 1 RUNNING, 2 SOFTSTOPPING, 3 HARD_STOPPED, 4 TARGET_REACHED, 6 ENABLED_IDLE, 5 ERROR (reserved)
//
// Serial (manual fallback):
//   STEP <n> | SPEED <val> | ACCEL <val> | SOFTSTOP | STOP | ZERO | STATUS
//
// Wiring (common-anode):
//   PUL+ -> +5V, PUL- -> D2 (STEP)
//   DIR+ -> +5V, DIR- -> D3 (DIR)
//   ENA+ -> +5V, ENA- -> D4 (ENA)   // LOW=Enable, HIGH=Disable
//   W5500 CS -> D10 (Ethernet.init(10))
// DIP: SW9=OFF, SW10=OFF
// Baud: 115200

#include <Arduino.h>
#include <math.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ---------- Network config ----------
byte mac[] = { 0xDE,0xAD,0xBE,0xEF,0xFE,0xED };
IPAddress ip(10, 1, 100, 222);          // Nano IP
const uint16_t UDP_LOCAL_PORT = 30001;   // รับคำสั่ง

// ที่อยู่ปลายทางสำหรับ "สถานะ"
IPAddress STATUS_DST_IP(10, 1, 100, 88); // <-- ใส่ IP ของเครื่อง ROS
const uint16_t STATUS_DST_PORT = 30002;  // พอร์ตที่ ROS listen สถานะ

EthernetUDP Udp;        // คำสั่ง
EthernetUDP UdpStatus;  // สถานะ (TX)
char udpBuf[32];

// ---------- Pins ----------
#define STEP_PIN  2
#define DIR_PIN   3
#define EN_PIN    4   // ENA- (LOW=Enable, HIGH=Disable)
#define LED_1     5
#define LED_2     6
#define LED_3     7
#define LED_4     8

// ---------- Motion parameters ----------
float DEFAULT_SPEED_SPS  = 10000.0f;   // steps/s
float DEFAULT_ACCEL_SPS2 = 5000.0f;    // steps/s^2
const float MIN_SPEED_SPS   = 10.0f;
const float MAX_SPEED_SPS   = 20000.0f;
const float MIN_ACCEL_SPS2  = 1.0f;
const uint32_t MIN_PULSE_US = 20;

// ---------- One-shot steps for UDP -1/1 ----------
const long OPEN_STEPS  = 10000;  // 1 => open  (+)
const long CLOSE_STEPS = 10000;  // -1 => close (-)

// ---------- State ----------
volatile long current_steps = 0;
volatile long target_steps  = 0;
volatile bool motion_active = false;
volatile bool softstop_active = false;

float speed_sps       = 0.0f;
float user_target_sps = DEFAULT_SPEED_SPS;
float accel_sps2      = DEFAULT_ACCEL_SPS2;

bool     step_high = false;
uint32_t next_event_us = 0;
uint32_t step_half_period_us = 500;
int      dir_sign = +1;
bool     driver_enabled = false;

// pending command when busy (-1 or 1)
int8_t pending_cmd = 0;

// ---------- Status (Int32 as ASCII) ----------
enum StatusCode : int32_t {
  IDLE_DISABLED = 0,
  RUNNING       = 1,
  SOFTSTOPPING  = 2,
  HARD_STOPPED  = 3,
  TARGET_REACHED= 4,
  ENABLED_IDLE  = 6,
  ERROR_STATUS  = 5
};

int32_t current_status = IDLE_DISABLED;
int32_t last_sent_status = -9999;   // force send on start
uint32_t last_status_hb_ms = 0;     // heartbeat timer

// ---------- Helpers ----------
static inline float clampf(float x, float lo, float hi){
  return x < lo ? lo : (x > hi ? hi : x);
}

void clampConfigs() {
  user_target_sps = clampf(user_target_sps, MIN_SPEED_SPS, MAX_SPEED_SPS);
  if (accel_sps2 < MIN_ACCEL_SPS2) accel_sps2 = MIN_ACCEL_SPS2;
}

void driverEnable(bool en) {
  // ENA-: LOW = Enable, HIGH = Disable
  driver_enabled = en;
  digitalWrite(EN_PIN, en ? LOW : HIGH);
}

// ---------- Status TX ----------
void sendStatus(int32_t st, bool force=false) {
  // ส่งเฉพาะเมื่อเปลี่ยนสถานะ หรือ heartbeat (force=true)
  uint32_t now = millis();
  if (!force && st == last_sent_status && (now - last_status_hb_ms) < 200) return;

  char buf[16];
  ltoa(st, buf, 10);
  UdpStatus.beginPacket(STATUS_DST_IP, STATUS_DST_PORT);
  UdpStatus.write((uint8_t*)buf, strlen(buf));
  UdpStatus.endPacket();

  last_sent_status = st;
  last_status_hb_ms = now;
}

void setStatus(int32_t st, bool force=false) {
  current_status = st;
  sendStatus(st, force);
}

void updateHalfPeriodFromSpeed() {
  float s = clampf(speed_sps, MIN_SPEED_SPS, MAX_SPEED_SPS);
  uint32_t hp = (uint32_t)(500000.0f / s);
  if (hp < MIN_PULSE_US) hp = MIN_PULSE_US;
  step_half_period_us = hp;
}

void setDir(int sign) {
  dir_sign = (sign >= 0) ? +1 : -1;
  digitalWrite(DIR_PIN, (dir_sign >= 0) ? HIGH : LOW);
}

void stopMove() {
  motion_active = false;
  softstop_active = false;
  digitalWrite(STEP_PIN, LOW);
  step_high = false;
  speed_sps = 0.0f;
  driverEnable(false);
  setStatus(IDLE_DISABLED);  // ENA ปลดแล้ว
}

void startMove(long delta_steps) {
  if (delta_steps == 0) return;
  clampConfigs();
  softstop_active = false;
  setDir(delta_steps);
  target_steps   = current_steps + delta_steps;
  motion_active  = true;
  step_high      = false;
  speed_sps      = 0.0f;
  driverEnable(true);
  updateHalfPeriodFromSpeed();
  next_event_us  = micros();
  setStatus(RUNNING, true);  // report start immediately
}

// ---- trapezoid planner (with braking & proper soft-stop) ----
void velocityPlannerUpdate() {
  static uint32_t last_ms = millis();
  uint32_t now_ms = millis();
  float dt = (now_ms - last_ms) / 1000.0f;
  if (dt < 0.005f) return; // ~200 Hz
  last_ms = now_ms;

  if (!motion_active) {
    // heartbeat while idle (ENA อาจ EN/DIS)
    // sendStatus(driver_enabled ? ENABLED_IDLE : IDLE_DISABLED);
    setStatus(driver_enabled ? ENABLED_IDLE : IDLE_DISABLED);
    return;
  }

  long remain_steps = labs(target_steps - current_steps);

  float desired;
  if (softstop_active) {
    desired = 0.0f;
    setStatus(SOFTSTOPPING);
  } else {
    if (remain_steps == 0) {
      // ถึงเป้าแบบปกติ
      driverEnable(false);
      motion_active = false;
      step_high = false;
      speed_sps = 0.0f;
      // แจ้ง TARGET_REACHED แล้วตามด้วย IDLE_DISABLED heartbeat
      setStatus(TARGET_REACHED, true);
      return;
    }
    float vmax_stop = sqrtf(2.0f * accel_sps2 * (float)remain_steps);
    desired = user_target_sps;
    if (vmax_stop < desired) desired = vmax_stop;
    if (remain_steps <= 10) {
      float tail_cap = (float)remain_steps * (float)MIN_SPEED_SPS;
      if (desired > tail_cap) desired = tail_cap;
    }
    setStatus(RUNNING);
  }

  if (speed_sps < desired) {
    speed_sps += accel_sps2 * dt;
    if (speed_sps > desired) speed_sps = desired;
  } else if (speed_sps > desired) {
    speed_sps -= accel_sps2 * dt;
    if (speed_sps < desired) speed_sps = desired;
  }

  if (!softstop_active && speed_sps < MIN_SPEED_SPS && remain_steps <= 2) {
    speed_sps = MIN_SPEED_SPS;
  }
  if (softstop_active && speed_sps < (MIN_SPEED_SPS * 0.8f)) {
    // จบ soft stop
    stopMove(); // จะส่ง IDLE_DISABLED ภายใน
    return;
  }

  updateHalfPeriodFromSpeed();
}

// ---- step edge machine ----
void runMotion() {
  if (!motion_active) return;

  long remaining = target_steps - current_steps;
  int sign;

  if (softstop_active) {
    sign = dir_sign;
  } else {
    if (remaining == 0) {
      // ถึงเป้าแบบปกติ (เผื่อกรณีเหลื่อมเวลา)
      driverEnable(false);
      motion_active = false;
      speed_sps = 0.0f;
      setStatus(TARGET_REACHED, true);
      return;
    }
    sign = (remaining > 0) ? +1 : -1;
  }

  if (sign != dir_sign) setDir(sign);

  uint32_t now = micros();
  if ((int32_t)(now - next_event_us) >= 0) {
    if (!step_high) {
      digitalWrite(STEP_PIN, HIGH);
      step_high = true;
      next_event_us = micros() + step_half_period_us;
    } else {
      digitalWrite(STEP_PIN, LOW);
      step_high = false;
      current_steps += sign;
      next_event_us = micros() + step_half_period_us;
    }
  }
}

// ---- Serial ----
void printStatusSerial() {
  Serial.print("POS: "); Serial.print(current_steps);
  Serial.print("  Target: "); Serial.print(target_steps);
  Serial.print("  v: "); Serial.print(speed_sps, 1);
  Serial.print(" sps  v_user: "); Serial.print(user_target_sps, 1);
  Serial.print(" sps  a: "); Serial.print(accel_sps2, 1);
  Serial.print(" sps^2  mode: ");
  Serial.print(softstop_active ? "SOFTSTOP" : (motion_active ? "RUN" : "IDLE"));
  Serial.print("  ENA: "); Serial.print(driver_enabled ? "EN" : "DIS");
  Serial.print("  STATUS: "); Serial.println(current_status);
}

void handleCommand(String cmd) {
  cmd.trim(); if (cmd.length() == 0) return;
  String u = cmd; u.toUpperCase();

  if (u.startsWith("STEP ")) {
    long st = (long)strtol(u.substring(5).c_str(), nullptr, 10);
    Serial.print("STEP "); Serial.println(st);
    startMove(st);
  }
  else if (u.startsWith("SPEED ")) {
    float v = u.substring(6).toFloat();
    if (!isfinite(v)) v = DEFAULT_SPEED_SPS;
    user_target_sps = v; clampConfigs();
    Serial.print("Target speed = "); Serial.print(user_target_sps, 1); Serial.println(" sps");
  }
  else if (u.startsWith("ACCEL ")) {
    float a = u.substring(6).toFloat();
    if (!isfinite(a)) a = DEFAULT_ACCEL_SPS2;
    accel_sps2 = a; clampConfigs();
    Serial.print("Accel = "); Serial.print(accel_sps2, 1); Serial.println(" sps^2");
  }
  else if (u.startsWith("SOFTSTOP")) {
    if (!motion_active) { Serial.println("Already stopped."); return; }
    softstop_active = true;
    setStatus(SOFTSTOPPING, true);
    Serial.println("Soft-stopping...");
  }
  else if (u.startsWith("STOP")) {
    stopMove();
    setStatus(HARD_STOPPED, true);
    Serial.println("Stopped.");
  }
  else if (u.startsWith("ZERO")) {
    current_steps = 0; target_steps = 0; Serial.println("Zeroed.");
  }
  else if (u.startsWith("STATUS")) {
    printStatusSerial();
  }
  else {
    Serial.println("Commands: STEP <n>, SPEED <val>, ACCEL <val>, SOFTSTOP, STOP, ZERO, STATUS");
  }
}

// ---------- UDP ----------
void setLAN() {
  Ethernet.init(10);        // W5500 CS on D10
  Ethernet.begin(mac, ip);  // static IP
  delay(100);

  Udp.begin(UDP_LOCAL_PORT);              // ฟังคำสั่งจาก ROS 
  UdpStatus.begin(STATUS_DST_PORT);       // ใช้ส่งสถานะ (source port = 30002)
  Serial.print("IP: "); Serial.println(Ethernet.localIP());
  Serial.print("UDP cmd port: "); Serial.println(UDP_LOCAL_PORT);
  Serial.print("Status dst: "); Serial.print(STATUS_DST_IP);
  Serial.print(":"); Serial.println(STATUS_DST_PORT);
}

inline void queueOrStartByCmd(int8_t cmd) {
  long steps = (cmd == 1) ? OPEN_STEPS : -CLOSE_STEPS;
  if (!motion_active) {
    startMove(steps);
  } else {
    // if already moving, request soft stop then queue this command
    softstop_active = true;
    pending_cmd = cmd;
    setStatus(SOFTSTOPPING, true);
  }
}

void handleUdpPacket() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;

  int len = Udp.read(udpBuf, sizeof(udpBuf) - 1);
  if (len <= 0) return;
  udpBuf[len] = '\0';

  int cmd = atoi(udpBuf);  // "-1","0","1","2"
  switch (cmd) {
    case -1: queueOrStartByCmd(-1); break;
    case  0: stopMove(); setStatus(HARD_STOPPED, true); break;
    case  1: queueOrStartByCmd(1);  break;
    case  2: if (motion_active){ softstop_active = true; setStatus(SOFTSTOPPING, true);} break;
    default: /* ignore */ break;
  }
}

// ---------- Setup / Loop ----------
void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(EN_PIN,   OUTPUT);
  pinMode(LED_1,   OUTPUT);
  pinMode(LED_2,   OUTPUT);
  pinMode(LED_3,   OUTPUT);
  pinMode(LED_4,   OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);
  driverEnable(false);

  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);
  digitalWrite(LED_4, HIGH);

  Serial.begin(115200);
  delay(200);
  setLAN();

  Serial.println("\n== NANO -> DE2405C (Trapezoid + SOFTSTOP + ENA + UDP + STATUS) ==");
  Serial.println("UDP cmd: -1(close), 0(stop), 1(open), 2(softstop)");
  Serial.println("ROS status port: 30002 (Int32 as ASCII)");

  clampConfigs();
  speed_sps = 0.0f;
  updateHalfPeriodFromSpeed();
  next_event_us = micros();

  setStatus(IDLE_DISABLED, true); // boot status
}

void loop() {
  velocityPlannerUpdate();
  runMotion();

  handleUdpPacket();

  // run queued command after soft stop done
  if (!motion_active && pending_cmd != 0) {
    int8_t cmd = pending_cmd;
    pending_cmd = 0;
    queueOrStartByCmd(cmd); // now idle -> start
  }

  // Serial CLI
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { handleCommand(buf); buf = ""; }
    else { buf += c; if (buf.length() > 80) buf = ""; }
  }

  // heartbeat while idle/running
  // sendStatus(current_status);
}
