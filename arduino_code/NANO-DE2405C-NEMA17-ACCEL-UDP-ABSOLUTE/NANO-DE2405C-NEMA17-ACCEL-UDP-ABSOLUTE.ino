// ===== Arduino NANO -> DE2405C (Timer1 Pulse Gen) =====
// Non-Blocking Stepper + Timer1 ISR pulse + proper SOFTSTOP + ENA
// + W5500 UDP Control + STATUS burst+sticky + EEPROM Save (AVR-safe)
// Board: Arduino Nano (ATmega328P), Baud: 115200
//
// UDP commands (ASCII):
//  -1 CLOSE (go TARGET_CLOSE_POS), 0 STOP (hard), 1 OPEN (go TARGET_OPEN_POS), 2 SOFTSTOP
// STATUS (ASCII Int32):
//  0 IDLE_DISABLED, 1 RUNNING, 2 SOFTSTOPPING, 3 HARD_STOPPED,
//  4 TARGET_REACHED, 6 ENABLED_IDLE, 5 ERROR (reserved)

// sudo tcpdump -i enx000000000149 udp port 30002

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <EEPROM.h>
#include <math.h>

// ---------- Network ----------
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(10, 1, 100, 222);            // Nano IP
const uint16_t UDP_LOCAL_PORT = 30001;    // command port
IPAddress STATUS_DST_IP(10, 1, 100, 88);  // ROS IP
const uint16_t STATUS_DST_PORT = 30002;   // status port

EthernetUDP Udp, UdpStatus;
char udpBuf[32];

// ---------- Pins ----------
#define STEP_PIN 2  // D2 -> PUL-
#define DIR_PIN 3   // D3 -> DIR-
#define EN_PIN 4    // D4 -> ENA- (LOW=Enable, HIGH=Disable)

// For ultra-fast STEP in ISR (direct port)
#define STEP_PORT PORTD
#define STEP_DDR DDRD
#define STEP_BIT PD2  // D2

// ---------- Motion params ----------
float DEFAULT_SPEED_SPS = 8000.0f;   // steps/s
float DEFAULT_ACCEL_SPS2 = 4000.0f;  // steps/s^2
const float MIN_SPEED_SPS = 10.0f;
const float MAX_SPEED_SPS = 20000.0f;

// Timer1 prescaler = 64 => 16MHz/64 = 250kHz => 4us per tick
// half-period(us) -> OCR1A ticks = half_period_us / 4
const uint16_t OCR1A_MIN_TICKS = 3;  // กันเร็วเกินไป (≈12us)
const uint32_t MIN_PULSE_US = 100;   // guard rail เมื่อคำนวณ half-period

// ---------- EEPROM layout ----------
const int EE_MAGIC_ADDR  = 0;
const int EE_POS_ADDR    = EE_MAGIC_ADDR + 4;
const int EE_OPEN_ADDR   = EE_POS_ADDR   + 4;
const int EE_CLOSE_ADDR  = EE_OPEN_ADDR  + 4;

// เวอร์ชัน magic
const uint32_t EE_MAGIC_OLD = 0x4E414E4FUL; // 'NANO'  (เดิม: ไม่มี SPEED/ACCEL)
const uint32_t EE_MAGIC_NEW = 0x4E414E31UL; // 'NAN1'  (ใหม่: มี SPEED/ACCEL)

// ช่องเพิ่มใหม่
const int EE_SPEED_ADDR  = EE_CLOSE_ADDR + 4; // float user_target_sps
const int EE_ACCEL_ADDR  = EE_SPEED_ADDR + 4; // float accel_sps2

// helpers อ่าน/เขียน MAGIC
inline uint32_t ee_read_magic() {
  uint32_t m; EEPROM.get(EE_MAGIC_ADDR, m); return m;
}
inline void ee_write_magic_new() {
  EEPROM.put(EE_MAGIC_ADDR, EE_MAGIC_NEW);
}
inline bool ee_is_old() { return ee_read_magic() == EE_MAGIC_OLD; }
inline bool ee_is_new() { return ee_read_magic() == EE_MAGIC_NEW; }
inline bool ee_unknown(){ uint32_t m=ee_read_magic(); return (m!=EE_MAGIC_OLD && m!=EE_MAGIC_NEW); }



// ---------- Targets (runtime; load/save EEPROM) ----------
long TARGET_OPEN_POS = 10000;
long TARGET_CLOSE_POS = 0;

// ---------- Autosave ----------
long last_saved_steps = 0;
const int SAVE_DELTA_STEPS = 100;

// ---------- State (shared with ISR) ----------
volatile long current_steps = 0;  // updated in ISR (falling edge)
volatile long target_steps = 0;
volatile bool motion_active = false;
volatile bool softstop_active = false;

float user_target_sps = DEFAULT_SPEED_SPS;
float accel_sps2 = DEFAULT_ACCEL_SPS2;

// Values consumed by ISR
volatile uint16_t isr_ocr1a_ticks = 25000 / 4;  // default half-period ticks
volatile int8_t isr_dir_sign = +1;
volatile bool isr_step_state = false;  // false->rising next
volatile bool isr_reached_target = false;


bool driver_enabled = false;
int8_t pending_cmd = 0;

// ---------- Status ----------
enum StatusCode : int32_t {
  IDLE_DISABLED = 0,
  RUNNING = 1,
  SOFTSTOPPING = 2,
  HARD_STOPPED = 3,
  TARGET_REACHED = 4,
  ENABLED_IDLE = 6,
  ERROR_STATUS = 5
};
int32_t current_status = IDLE_DISABLED;
int32_t last_sent_status = -9999;
uint32_t last_status_hb_ms = 0;

// Burst + sticky
uint8_t status_burst_left = 0;
uint32_t status_burst_due1 = 0, status_burst_due2 = 0;
int32_t burst_status_value = -9999;
int32_t important_status = -9999;
uint32_t important_until_ms = 0;

// ---------- Helpers ----------
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline long clampl(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void driverEnable(bool en) {
  driver_enabled = en;
  digitalWrite(EN_PIN, en ? LOW : HIGH);
}

// Atomic read of 32-bit on AVR
long readCurrentStepsAtomic() {
  noInterrupts();
  long v = current_steps;
  interrupts();
  Serial.println(v);
  return v;
}

void writeISR_Ticks(uint16_t ticks) {
  noInterrupts();
  isr_ocr1a_ticks = ticks;
  interrupts();
}
void writeISR_DirSign(int8_t s) {
  noInterrupts();
  isr_dir_sign = s;
  interrupts();
}

// ---------- STATUS ----------
static inline void rawSendStatusPacket(int32_t st) {
  char buf[12];
  ltoa(st, buf, 10);
  UdpStatus.beginPacket(STATUS_DST_IP, STATUS_DST_PORT);
  UdpStatus.write((uint8_t*)buf, strlen(buf));
  UdpStatus.endPacket();
}
void scheduleStatusBurst(int32_t st) {
  rawSendStatusPacket(st);
  burst_status_value = st;
  status_burst_left = 2;
  uint32_t now = millis();
  status_burst_due1 = now + 120;
  status_burst_due2 = now + 300;
}
void pumpStatusBurst() {
  if (status_burst_left == 0) return;
  uint32_t now = millis();
  if (status_burst_left == 2 && (int32_t)(now - status_burst_due1) >= 0) {
    rawSendStatusPacket(burst_status_value);
    status_burst_left = 1;
  } else if (status_burst_left == 1 && (int32_t)(now - status_burst_due2) >= 0) {
    rawSendStatusPacket(burst_status_value);
    status_burst_left = 0;
  }
}
void sendStatus(int32_t st, bool force = false) {
  uint32_t now = millis();
  if (!force && st == last_sent_status && (now - last_status_hb_ms) < 100) return;
  scheduleStatusBurst(st);
  last_sent_status = st;
  last_status_hb_ms = now;
}
void setStatus(int32_t st, bool force = false) {
  current_status = st;
  sendStatus(st, force);
}

// ---------- EEPROM ----------
void savePositionToEEPROM(long pos, bool force = false) {
  pos = clampl(pos, TARGET_CLOSE_POS, TARGET_OPEN_POS);

  // ถ้ายังไม่ใช่ format ใหม่ ให้เขียน magic ใหม่
  if (!ee_is_new()) ee_write_magic_new();

  if (force || labs(pos - last_saved_steps) >= SAVE_DELTA_STEPS) {
    EEPROM.put(EE_POS_ADDR, pos);
    last_saved_steps = pos;
  }
}

long loadPositionFromEEPROM() {
  if (ee_unknown()) {
    last_saved_steps = 0;
    return 0;
  }
  long v;
  EEPROM.get(EE_POS_ADDR, v);
  v = clampl(v, TARGET_CLOSE_POS, TARGET_OPEN_POS);
  last_saved_steps = v;
  return v;
}

void saveTargetsToEEPROM(long open_pos, long close_pos) {
  if (!ee_is_new()) ee_write_magic_new();

  open_pos  = max(0L, min(open_pos, 500000L));
  close_pos = max(0L, min(close_pos, open_pos));
  EEPROM.put(EE_OPEN_ADDR,  open_pos);
  EEPROM.put(EE_CLOSE_ADDR, close_pos);
}

void loadTargetsFromEEPROM() {
  if (ee_unknown()) return; // ยังไม่เคยเขียนอะไร ปล่อยใช้ค่าดีฟอลต์

  long open_eep = 0, close_eep = 0;
  EEPROM.get(EE_OPEN_ADDR,  open_eep);
  EEPROM.get(EE_CLOSE_ADDR, close_eep);

  if (open_eep  > 0 && open_eep  <= 500000) TARGET_OPEN_POS  = open_eep;
  if (close_eep >= 0 && close_eep <  TARGET_OPEN_POS) TARGET_CLOSE_POS = close_eep;
}


void saveMotionParamsToEEPROM(bool force_magic_upgrade = false) {
  if (!ee_is_new() || force_magic_upgrade) ee_write_magic_new();

  float v_speed = clampf(user_target_sps, MIN_SPEED_SPS, MAX_SPEED_SPS);
  float v_accel = (accel_sps2 < 1.0f) ? 1.0f : accel_sps2;

  EEPROM.put(EE_SPEED_ADDR, v_speed);
  EEPROM.put(EE_ACCEL_ADDR, v_accel);
}

void loadMotionParamsFromEEPROM() {
  if (ee_is_new()) {
    float v_speed=0.0f, v_accel=0.0f;
    EEPROM.get(EE_SPEED_ADDR, v_speed);
    EEPROM.get(EE_ACCEL_ADDR, v_accel);
    if (!isfinite(v_speed) || v_speed <= 0.0f) v_speed = DEFAULT_SPEED_SPS;
    if (!isfinite(v_accel) || v_accel <  1.0f) v_accel = DEFAULT_ACCEL_SPS2;
    user_target_sps = clampf(v_speed, MIN_SPEED_SPS, MAX_SPEED_SPS);
    accel_sps2      = (v_accel < 1.0f) ? 1.0f : v_accel;
  } else if (ee_is_old() || ee_unknown()) {
    // ใช้ค่าดีฟอลต์/ปัจจุบัน แล้วอัปเกรดเป็นฟอร์แมตใหม่
    user_target_sps = clampf(user_target_sps, MIN_SPEED_SPS, MAX_SPEED_SPS);
    accel_sps2      = (accel_sps2 < 1.0f) ? 1.0f : accel_sps2;
    saveMotionParamsToEEPROM(/*force_magic_upgrade=*/true);
  }
}

// ---------- Speed -> Timer ticks ----------
uint16_t halfPeriodToTicks(float speed_sps) {
  float s = clampf(speed_sps, MIN_SPEED_SPS, MAX_SPEED_SPS);
  float half_us = 500000.0f / s;  // 0.5 * 1e6 / sps
  if (half_us < MIN_PULSE_US) half_us = MIN_PULSE_US;
  uint32_t ticks = (uint32_t)(half_us / 4.0f);  // 4us per tick
  if (ticks < OCR1A_MIN_TICKS) ticks = OCR1A_MIN_TICKS;
  if (ticks > 65535) ticks = 65535;
  return (uint16_t)ticks;
}

// ---------- Motion control (non-ISR part) ----------
void stopMove() {
  noInterrupts();
  motion_active = false;
  softstop_active = false;
  isr_step_state = false;
  interrupts();
  STEP_PORT &= ~_BV(STEP_BIT);
  driverEnable(false);
  setStatus(IDLE_DISABLED);
}
void startMoveTo(long target) {
  target = clampl(target, TARGET_CLOSE_POS, TARGET_OPEN_POS);
  long nowpos = readCurrentStepsAtomic();
  long delta = target - nowpos;
  if (labs(delta) < 2) {
    Serial.println("Already at target.");
    setStatus(IDLE_DISABLED);
    return;
  }

  // set dir
  digitalWrite(DIR_PIN, (delta >= 0) ? HIGH : LOW);
  writeISR_DirSign((delta >= 0) ? +1 : -1);

  noInterrupts();
  target_steps = target;
  softstop_active = false;
  motion_active = true;
  isr_step_state = false;
  interrupts();

  driverEnable(true);
  // start from zero speed (ramp up)
  float start_speed = MIN_SPEED_SPS;
  writeISR_Ticks(halfPeriodToTicks(start_speed));
  setStatus(RUNNING, true);
}

void velocityPlannerUpdate() {
  static uint32_t last_ms = millis();
  uint32_t now_ms = millis();
  float dt = (now_ms - last_ms) / 1000.0f;
  if (dt < 0.005f) return;  // ~200Hz
  last_ms = now_ms;

  if (!motion_active) {
    if (isr_reached_target) {
      isr_reached_target = false;
      driverEnable(false);
      setStatus(TARGET_REACHED, true);
      important_status = TARGET_REACHED;
      important_until_ms = millis() + 600;
      savePositionToEEPROM(readCurrentStepsAtomic(), /*force=*/true);
      return;
    }
    // เดิม: idle heartbeat / sticky
    if ((int32_t)(millis() - important_until_ms) < 0 && important_status != -9999)
      setStatus(important_status);
    else
      setStatus(driver_enabled ? ENABLED_IDLE : IDLE_DISABLED);
    return;
  }


  long pos = readCurrentStepsAtomic();
  long remain = labs(target_steps - pos);
  float desired;

  if (softstop_active) {
    desired = 0.0f;
    setStatus(SOFTSTOPPING);
  } else {
    if (remain == 0) {
      // target reached
      noInterrupts();
      motion_active = false;
      isr_step_state = false;
      interrupts();
      STEP_PORT &= ~_BV(STEP_BIT);
      driverEnable(false);
      setStatus(TARGET_REACHED, true);
      important_status = TARGET_REACHED;
      important_until_ms = millis() + 600;
      savePositionToEEPROM(pos, /*force*/ true);
      return;
    }
    float vmax_stop = sqrtf(2.0f * accel_sps2 * (float)remain);
    desired = (vmax_stop < user_target_sps) ? vmax_stop : user_target_sps;
    setStatus(RUNNING);
  }

  // trapezoid ramp
  static float speed_sps = 0.0f;
  if (speed_sps < desired) {
    speed_sps += accel_sps2 * dt;
    if (speed_sps > desired) speed_sps = desired;
  } else if (speed_sps > desired) {
    speed_sps -= accel_sps2 * dt;
    if (speed_sps < desired) speed_sps = desired;
  }

  if (!softstop_active && speed_sps < MIN_SPEED_SPS && remain <= 2) {
    speed_sps = MIN_SPEED_SPS;
  }
  if (softstop_active && speed_sps < (MIN_SPEED_SPS * 0.8f)) {
    stopMove();
    return;
  }

  // push new half-period to ISR
  writeISR_Ticks(halfPeriodToTicks(speed_sps));
}

// ---------- Timer1 ISR: generates clean STEP pulses ----------
ISR(TIMER1_COMPA_vect) {
  if (!motion_active) {
    STEP_PORT &= ~_BV(STEP_BIT);
    isr_step_state = false;
    return;
  }

  if (!isr_step_state) {
    STEP_PORT |= _BV(STEP_BIT);  // rising
    isr_step_state = true;
  } else {
    STEP_PORT &= ~_BV(STEP_BIT);  // falling -> count
    isr_step_state = false;

    // เดิน 1 สเต็ป
    long next = current_steps + isr_dir_sign;

    // เช็คแตะ/ข้ามเป้า (>= เป้าหมายเมื่อเดิน +, <= เมื่อเดิน -)
    if ((isr_dir_sign > 0 && next >= target_steps) || (isr_dir_sign < 0 && next <= target_steps)) {
      current_steps = target_steps;  // ล็อกให้พอดี
      motion_active = false;         // หยุดพัลส์
      isr_reached_target = true;     // บอกฝั่ง planner
      return;                        // STEP ถูกดึงลงแล้วด้านบน
    }

    // ยังไม่ถึงเป้า เดินต่อปกติ
    current_steps = next;
  }

  OCR1A = isr_ocr1a_ticks;  // อัปเดตคาบจาก planner
}


// ---------- UDP ----------
void setLAN() {
  Ethernet.init(10);
  Ethernet.begin(mac, ip);
  delay(100);
  Udp.begin(UDP_LOCAL_PORT);
  UdpStatus.begin(STATUS_DST_PORT);  // fix source port = 30002
  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("UDP cmd port: ");
  Serial.println(UDP_LOCAL_PORT);
  Serial.print("Status dst: ");
  Serial.print(STATUS_DST_IP);
  Serial.print(":");
  Serial.println(STATUS_DST_PORT);
}

inline void queueOrStartByCmd(int8_t cmd) {
  long target = (cmd == 1) ? TARGET_OPEN_POS : TARGET_CLOSE_POS;
  if (!motion_active) {
    startMoveTo(target);
  } else {
    softstop_active = true;
    pending_cmd = cmd;
    setStatus(SOFTSTOPPING, true);
    important_status = SOFTSTOPPING;
    important_until_ms = millis() + 600;
  }
}

void handleUdpPacket() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;
  int len = Udp.read(udpBuf, sizeof(udpBuf) - 1);
  if (len <= 0) return;
  udpBuf[len] = '\0';

  int cmd = atoi(udpBuf);
  switch (cmd) {
    case -1: queueOrStartByCmd(-1); break;
    case 0:
      stopMove();
      setStatus(HARD_STOPPED, true);
      important_status = HARD_STOPPED;
      important_until_ms = millis() + 600;
      break;
    case 1: queueOrStartByCmd(1); break;
    case 2:
      if (motion_active) {
        softstop_active = true;
        setStatus(SOFTSTOPPING, true);
        important_status = SOFTSTOPPING;
        important_until_ms = millis() + 600;
      }
      break;
    default: break;
  }
}

// ---------- Serial CLI ----------
void printHelp() {
  Serial.println("==== Serial Commands ====");
  Serial.println("OPEN | CLOSE");
  Serial.println("GOTO <steps>   (absolute)");
  Serial.println("STEP <n>       (relative)");
  Serial.println("SPEED <val>    (steps/s)");
  Serial.println("ACCEL <val>    (steps/s^2)");
  Serial.println("SOFTSTOP | STOP");
  Serial.println("ZERO");
  Serial.println("SETPOS <steps> (set current pos, no move)");
  Serial.println("SETOPEN <steps> / SETCLOSE <steps> (save to EEPROM)");
  Serial.println("SAVE (pos) | LOAD (pos)");
  Serial.println("STATUS | HELP");
}
void printStatusSerial() {
  long pos = readCurrentStepsAtomic();
  Serial.print("POS: ");
  Serial.print(pos);
  Serial.print("  Target: ");
  Serial.print(target_steps);
  Serial.print("  ENA: ");
  Serial.print(driver_enabled ? "EN" : "DIS");
  Serial.print("  SPEED: ");
  Serial.print(user_target_sps, 1);
  Serial.print("  ACCEL: ");
  Serial.print(accel_sps2, 1);
  Serial.print("  STATUS: ");
  Serial.println(current_status);
}
void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  String u = line;
  u.toUpperCase();

  if (u == "OPEN") startMoveTo(TARGET_OPEN_POS);
  else if (u == "CLOSE") startMoveTo(TARGET_CLOSE_POS);
  else if (u.startsWith("GOTO ")) {
    long sp = strtol(line.c_str() + 5, nullptr, 10);
    startMoveTo(sp);
  } else if (u.startsWith("STEP ")) {
    long rel = strtol(line.c_str() + 5, nullptr, 10);
    startMoveTo(readCurrentStepsAtomic() + rel);
  } else if (u.startsWith("SPEED ")) {
    float v = atof(line.c_str() + 6);
    user_target_sps = clampf(v, MIN_SPEED_SPS, MAX_SPEED_SPS);
    saveMotionParamsToEEPROM();
    Serial.print("Target speed = ");
    Serial.println(user_target_sps, 1);
  } else if (u.startsWith("ACCEL ")) {
    float a = atof(line.c_str() + 6);
    accel_sps2 = (a < 1.0f) ? 1.0f : a;
    saveMotionParamsToEEPROM();
    Serial.print("Accel = ");
    Serial.println(accel_sps2, 1);
  } else if (u == "SOFTSTOP") {
    if (!motion_active) {
      Serial.println("Already stopped.");
      return;
    }
    softstop_active = true;
    setStatus(SOFTSTOPPING, true);
    important_status = SOFTSTOPPING;
    important_until_ms = millis() + 600;
  } else if (u == "STOP") {
    stopMove();
    setStatus(HARD_STOPPED, true);
    important_status = HARD_STOPPED;
    important_until_ms = millis() + 600;
  } else if (u == "ZERO") {
    noInterrupts();
    current_steps = 0;
    interrupts();
    Serial.println("Set current pos=0.");
  } else if (u.startsWith("SETPOS ")) {
    long sp = strtol(line.c_str() + 7, nullptr, 10);
    sp = clampl(sp, TARGET_CLOSE_POS, TARGET_OPEN_POS);
    noInterrupts();
    current_steps = sp;
    interrupts();
    Serial.print("Set pos = ");
    Serial.println(sp);
  } else if (u == "SAVE") {
    savePositionToEEPROM(readCurrentStepsAtomic(), true);
  } else if (u == "LOAD") {
    long v = loadPositionFromEEPROM();
    noInterrupts();
    current_steps = v;
    interrupts();
    Serial.print("Loaded pos = ");
    Serial.println(v);
  } else if (u == "STATUS") {
    printStatusSerial();
  } else if (u == "HELP") {
    printHelp();
  } else if (u.startsWith("SETOPEN ")) {
    long sp = strtol(line.c_str() + 8, nullptr, 10);
    sp = max(0L, min(sp, 500000L));
    TARGET_OPEN_POS = sp;
    if (TARGET_CLOSE_POS > sp) TARGET_CLOSE_POS = sp;
    saveTargetsToEEPROM(TARGET_OPEN_POS, TARGET_CLOSE_POS);
    Serial.print("SETOPEN=");
    Serial.println(TARGET_OPEN_POS);
  } else if (u.startsWith("SETCLOSE ")) {
    long sp = strtol(line.c_str() + 9, nullptr, 10);
    sp = max(0L, min(sp, TARGET_OPEN_POS));
    TARGET_CLOSE_POS = sp;
    saveTargetsToEEPROM(TARGET_OPEN_POS, TARGET_CLOSE_POS);
    Serial.print("SETCLOSE=");
    Serial.println(TARGET_CLOSE_POS);
  } else {
    Serial.println("Unknown. Type HELP");
  }
}

// ---------- Timer1 init ----------
void timer1Init() {
  // STEP pin as output via DDRD, for fast ISR toggling
  STEP_DDR |= _BV(STEP_BIT);

  // Clear Timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // CTC mode (WGM12=1), prescaler /64 (CS11=1, CS10=1)
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);

  // Initial compare
  OCR1A = isr_ocr1a_ticks;

  // Enable compare match A interrupt
  TIMSK1 |= (1 << OCIE1A);
}

// ---------- Setup / Loop ----------
void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  driverEnable(false);

  Serial.begin(115200);
  delay(200);
  setLAN();

  loadTargetsFromEEPROM();
  loadMotionParamsFromEEPROM(); 
  long v = loadPositionFromEEPROM();
  noInterrupts();
  current_steps = v;
  interrupts();
  current_steps = clampl(current_steps, TARGET_CLOSE_POS, TARGET_OPEN_POS);

  timer1Init();  // start clean pulse engine

  Serial.println("\n== NANO -> DE2405C (Timer1 ISR + ABS Target + UDP + EEPROM) ==");
  Serial.println("UDP cmd: -1(close),0(stop),1(open),2(softstop)");
  Serial.println("Type HELP for Serial commands.");

  setStatus(IDLE_DISABLED, true);
}

void loop() {
  velocityPlannerUpdate();
  handleUdpPacket();
  pumpStatusBurst();

  if (motion_active) {
    long pos = readCurrentStepsAtomic();
    savePositionToEEPROM(pos);
  }

  if (!motion_active && pending_cmd != 0) {
    int8_t cmd = pending_cmd;
    pending_cmd = 0;
    queueOrStartByCmd(cmd);
  }

  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleSerialLine(buf);
      buf = "";
    } else {
      buf += c;
      if (buf.length() > 80) buf = "";
    }
  }
}
