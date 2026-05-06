/*
  AlphaBot2-Ar PathRunner v2
  ----------------------------------------------------------
  Baza hardware: sketch-ul existent al utilizatorului
   (TRSensors, OLED SSD1306, NeoPixel, PCF8574 joystick + IR,
    motor driver standard AlphaBot2-Ar).

  CE E NOU FATA DE V1:
   - Detectie checkpoint robusta: PATRAT NEGRU PLIN.
     Un patrat se distinge de un cross junction prin DURATA
     in care toti 5 senzori vad negru continuu (>= CP_FULL_MS).
   - Detectie junction multi-modala: T, Y, cross, curbe stranse,
     cercuri tangente. Mai multe trigger-uri independente,
     fara cerinta de "error stable" (care bloca pe curbe).
   - Eliminat ST_JCT_CP_VERIFY (cauza dubla numarare CP).
   - turns[] (NOUA conventie: 0=u-turn, 1=prima iesire dinspre stanga,
     2=a doua iesire dinspre stanga, ...) injectat de UI-ul web.

  Conventie secventa:
    0 = u-turn (intoarcere prin ramura din spate)
    1 = prima iesire dinspre stanga (cea mai apropiata de spate-stanga)
    2 = a doua iesire dinspre stanga
    ...
    K = ultima iesire (cea mai dinspre dreapta)
  Numerotam DOAR iesirile reale detectate prin scan, ignorand ramura
  prin care am venit. Indexul 0 din tabelul intern jct_exit_actions
  ramane rezervat pentru 'B' (u-turn), iar indecsii >=1 corespund
  iesirilor sortate stanga -> dreapta.
*/

#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TRSensors.h"
#include <Wire.h>

// ============ Pins (din sketch-ul original) ============
#define PWMA   6
#define AIN2   A0
#define AIN1   A1
#define PWMB   5
#define BIN1   A2
#define BIN2   A3
#define PIN 7
#define NUM_SENSORS 5
#define OLED_RESET 9
#define Addr 0x20

// ============ Obstacle (IR) ============
#define OBSTACLE_STOP_MS      350
#define OBSTACLE_DEBOUNCE_MS  120

// ============ Motion / PID (NESCHIMBATE) ============
#define MAX_SPEED             40
#define MIN_SPEED             38
#define CURVE_SLOWDOWN        45
#define PIVOT_THRESHOLD     1450
#define SHARP_PIVOT_SPEED     75
#define INTERSECTION_SPEED    24
#define ERROR_DEADBAND        70
#define KP_DIV                13
#define KD_MULT                1
#define KI_DIV             22000
#define INT_ACTIVE_ZONE      220
#define INT_LIMIT           5000

// ============ Praguri senzori (NESCHIMBATE) ============
#define BLACK_THRESHOLD      700
#define WHITE_THRESHOLD      250
#define CENTER_BLACK         550
#define SIDE_BLACK           600

// ============ CHECKPOINT (PATRAT NEGRU PLIN) - imbunatatit ============
#define CP_BLACK_THRESHOLD   500   // prag mai relaxat decat BLACK_THRESHOLD (patrat are reflexe variabile)
#define CP_MIN_BLACK_COUNT     4   // 4 din 5 senzori negri = pe patrat (era 5 strict)
#define CP_PRELIM_MS          60   // initial pentru a suspecta CP (redus)
#define CP_FULL_MS           180   // durata totala pentru confirmare (redusa - era 220)
#define CP_GLITCH_GRACE_MS    40   // toleranta scurta cand un senzor pierde negrul
#define CP_EXIT_BLACK_MAX      1   // <=1 senzor negru => iesit din patrat
#define CP_LOCKOUT_MS       1500   // dupa CP, ignora alte detectii
#define CP_MAX_DURATION_MS  2500   // siguranta: dupa atat, considera iesit

// ============ JUNCTION (multi-modal, MAI SENSIBIL) ============
// Prag separat pentru senzorii laterali la detectie junction (mai relaxat decat SIDE_BLACK=600).
// Pe linii reale, s0/s4 nu ating mereu 600 cand intra ramura - cu 420 prindem mult mai repede.
#define JCT_SIDE_THRESHOLD   420
#define JCT_INNER_THRESHOLD  500   // pt s1/s3 (inner) la detectie laterala
#define JCT_ARM_SIDE_MS       25   // s0+s2 sau s4+s2 stabil (mult redus)
#define JCT_ARM_DENSE_MS      40   // 5/5 negru pentru cross/Y (redus)
#define JCT_ARM_EDGE_MS       50   // s0/s4 cu error mare (Y, tangente)
#define JCT_ARM_WIDEN_MS      30   // linia se "lateste": centru + lateral oricare
#define JCT_CONFIRM_MS         5   // confirmare suplimentara minima (aproape instant)
#define JCT_MIN_DIST_MM      100   // distanta minima parcursa intre 2 junctions
#define JCT_ENTRY_MIN_MS     220
#define JCT_ENTRY_MAX_MS     320
#define JCT_TURN_SPEED        40
#define JCT_COOLDOWN_MS      700   // redus de la 900
#define JCT_ERR_JUMP         900   // saritura brusca de eroare = bifurcatie Y (era 1100)

// ============ ODOMETRIE (fara encoderi - bazata pe PWM*timp) ============
// Calibrare: la ~PWM 40 robotul merge cu ~85 mm/s.
// Deci 1 unitate PWM * 1 ms = 85/40/1000 = 0.00213 mm. Ajustabil empiric.
#define MM_PER_PWM_MS_X1000   213   // 0.213 mm * (PWM/100) * ms; vezi formula in cod
#define WHEEL_BASE_MM         105   // distanta intre roti pt rotatii
#define ODO_MAX_DT_MS          50   // ignora pasi mari (pauze, delay-uri)

// ============ Scan unghi (din sketch-ul original) ============
#define JCT_STEP45_MS        280
#define JCT_RETURN90_MS      560
#define JCT_SETTLE_MS        140
#define JCT_SCAN_MS          180
#define JCT_LINE_CONFIRM_MS   70

// ============ Turn execution ============
#define TURN_L_MS            430
#define TURN_R_MS            320
#define COMMIT_DRIVE_MS      180

// ============ Recovery ============
#define UTURN_MIN_MS         700
#define UTURN_STOP_MS        180
#define LOST_GRACE_MS        450
#define STUCK_MAX_MS       18000

// ============ Misc ============
#define MOVE_LOG_SIZE         14
#define DEBUG_JUNCTION_STOP    0   // 1 = opreste la fiecare junction (apasa joy)
#define ENABLE_ODOMETRY        0   // 0 = pornire stabila pe UNO; 1 doar dupa ce confirmi ca exista RAM suficient
#define SHOW_ODOMETRY_ON_OLED  0   // UNO are RAM putin; 0 = afisaj sigur ca inainte
#define JOY_DEBOUNCE_MS       30
#define JOY_MIN_HOLD_MS      150
#define MAX_JUNCTION_EXITS     4

// ====================================================
// SECVENTA DE IESIRI (injectata de UI-ul web prin downloadIno)
// 0 = u-turn, 1 = prima iesire dinspre stanga, 2 = a doua, ...
// ====================================================
// <AI:turns_array>
const int turns[] = { 3, 1, 1, 2, 0, 1, 2, 2, 3, 2, 2 };
// </AI:turns_array>
const int TURNS_LEN = sizeof(turns) / sizeof(turns[0]);

// ====================================================
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
TRSensors trs = TRSensors();
Adafruit_NeoPixel RGB = Adafruit_NeoPixel(4, PIN, NEO_GRB + NEO_KHZ800);

unsigned int sensorValues[NUM_SENSORS];
unsigned int position;
int last_error = 0;
long integral = 0;
int recent_side = 1;
uint16_t i_led = 0, j_led = 0;
byte value = 0;
unsigned long last_led_time = 0;
unsigned long last_display = 0;
unsigned long stateEnterTime = 0;
unsigned long last_event_time = 0;
unsigned long checkpoint_lockout_until = 0;
unsigned long obstacle_seen_since = 0;
int checkpoint_count = 0;
int moves_seq_idx = 0;
char move_log[MOVE_LOG_SIZE + 1] = {0};
int move_log_count = 0;
unsigned long live_left_run = 0;
unsigned long live_right_run = 0;

// ===== Odometrie =====
#if ENABLE_ODOMETRY
float odo_x_mm = 0.0f;       // pozitia X relativa la start (mm)
float odo_y_mm = 0.0f;       // pozitia Y relativa la start (mm)
float odo_theta = 0.0f;      // unghiul curent (rad), 0 = directia de start
float odo_dist_mm = 0.0f;    // distanta totala parcursa (mm)
float odo_dist_at_last_jct = -10000.0f; // dist la ultima intersectie (anti dublu-trigger)
unsigned long odo_last_update = 0;
int last_pwm_left = 0;
int last_pwm_right = 0;
#endif

enum RobotState {
  ST_NORMAL,
  ST_LOST,
  ST_CP_CANDIDATE,   // NOU: testeaza durata 5/5 negru
  ST_CP_CONFIRMED,   // NOU: numara, iese din patrat
  ST_CP_STOP,        // optional: stop+joy dupa CP
  ST_UTURN,
  ST_JCT_ENTRY,
  ST_JCT_L45_TURN, ST_JCT_L45_SCAN,
  ST_JCT_L90_TURN, ST_JCT_L90_SCAN,
  ST_JCT_BACK_CENTER_FROM_LEFT,
  ST_JCT_CENTER_SCAN_1,
  ST_JCT_R45_TURN, ST_JCT_R45_SCAN,
  ST_JCT_R90_TURN, ST_JCT_R90_SCAN,
  ST_JCT_BACK_CENTER_FROM_RIGHT,
  ST_JCT_CENTER_SCAN_2,
  ST_JCT_STOP,
  ST_JCT_EXECUTE
};
RobotState robotState = ST_NORMAL;

// ===== timing trigger junction =====
unsigned long left_arm_since = 0;
unsigned long right_arm_since = 0;
unsigned long dense_since = 0;
unsigned long edge_left_since = 0;
unsigned long edge_right_since = 0;

// ===== timing CP =====
unsigned long cp_dense_since = 0;
unsigned long cp_last_seen = 0;
unsigned long cp_enter_time = 0;

// ===== junction snapshots =====
bool jct_saw_entry_corners = false;
unsigned long jct_scan_seen_since = 0;
bool jct_scan_latched = false;
bool ref_center_before = false;
bool ref_center_mid    = false;
bool ref_center_after  = false;
bool ref_left45        = false;
bool ref_left90        = false;
bool ref_right45       = false;
bool ref_right90       = false;
bool jct_arm_L = false;
bool jct_arm_F = false;
bool jct_arm_R = false;
char jct_choice = '?';
unsigned long jct_execute_ms = 0;
char jct_exit_actions[MAX_JUNCTION_EXITS] = {'B', 0, 0, 0};
char jct_exit_labels[20] = "0B";
uint8_t jct_exit_count = 1;
int8_t jct_selected_exit = -1;

// --- Prototypes ---
void PCF8574Write(byte data);
byte PCF8574Read();
uint32_t Wheel(byte WheelPos);
void setMotors(int left, int right);
void logMove(char m);
void showStatus();
bool snapshotHasLineAhead();
bool sweepSampleHasLine();
bool scanLineConfirmed(unsigned long now);
void resetJunctionRefs();
void buildExitTable();
void updateExitLabels();
void finalizeEvidence();
void prepareJunctionChoice(unsigned long now);
char actionFromTurn(int t);
bool joystickPressEdge();
void waitJoystickRelease();
long readVccMilliVolts();
int readBatteryPercent();
bool infraredObstacleAhead();

// ============ Motoare (identic cu original) ============
// Odometrie pe baza ultimei comenzi PWM (fara encoderi)
void updateOdometry(unsigned long now) {
#if ENABLE_ODOMETRY
  if (odo_last_update == 0) { odo_last_update = now; return; }
  unsigned long dt = now - odo_last_update;
  odo_last_update = now;
  if (dt == 0 || dt > ODO_MAX_DT_MS) return;
  // viteza liniara echivalenta in mm/s pentru fiecare roata:
  // v_roata_mm_s = pwm * (MM_PER_PWM_MS_X1000 / 1000) * (1000/ms_per_s simplificat)
  // simplificat: dist_roata_mm = pwm * MM_PER_PWM_MS_X1000 * dt / 100000
  float dl = (float)last_pwm_left  * (float)dt * (float)MM_PER_PWM_MS_X1000 / 100000.0f;
  float dr = (float)last_pwm_right * (float)dt * (float)MM_PER_PWM_MS_X1000 / 100000.0f;
  float d = (dl + dr) * 0.5f;
  float dtheta = (dr - dl) / (float)WHEEL_BASE_MM;
  odo_theta += dtheta;
  // normalizare in (-PI, PI]
  while (odo_theta >  3.14159265f) odo_theta -= 6.28318530f;
  while (odo_theta < -3.14159265f) odo_theta += 6.28318530f;
  odo_x_mm += d * cos(odo_theta);
  odo_y_mm += d * sin(odo_theta);
  odo_dist_mm += fabs(d);
#endif
}
void setMotors(int left, int right) {
  left  = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
#if ENABLE_ODOMETRY
  last_pwm_left = left;
  last_pwm_right = right;
#endif
  if (left >= 0) {
    analogWrite(PWMA, left);
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  } else {
    analogWrite(PWMA, -left);
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  }
  if (right >= 0) {
    analogWrite(PWMB, right);
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  } else {
    analogWrite(PWMB, -right);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }
}

void logMove(char m) {
  if (move_log_count < MOVE_LOG_SIZE) {
    move_log[move_log_count++] = m;
    move_log[move_log_count] = 0;
  } else {
    for (int k = 0; k < MOVE_LOG_SIZE - 1; k++) move_log[k] = move_log[k + 1];
    move_log[MOVE_LOG_SIZE - 1] = m;
    move_log[MOVE_LOG_SIZE] = 0;
  }
}

bool snapshotHasLineAhead() {
  bool c  = sensorValues[2] > CENTER_BLACK;
  bool l1 = sensorValues[1] > BLACK_THRESHOLD;
  bool r1 = sensorValues[3] > BLACK_THRESHOLD;
  return c && (l1 || r1);
}
bool sweepSampleHasLine() {
  bool s0 = sensorValues[0] > SIDE_BLACK;
  bool s1 = sensorValues[1] > BLACK_THRESHOLD;
  bool s2 = sensorValues[2] > CENTER_BLACK;
  bool s3 = sensorValues[3] > BLACK_THRESHOLD;
  bool s4 = sensorValues[4] > SIDE_BLACK;
  int black = (s0?1:0)+(s1?1:0)+(s2?1:0)+(s3?1:0)+(s4?1:0);
  bool left_edge   = s0 && s1;
  bool left_inner  = s1 && s2;
  bool right_inner = s2 && s3;
  bool right_edge  = s3 && s4;
  return left_edge || left_inner || right_inner || right_edge || black >= 3;
}
bool scanLineConfirmed(unsigned long now) {
  if (sweepSampleHasLine()) {
    if (jct_scan_seen_since == 0) jct_scan_seen_since = now;
    if ((now - jct_scan_seen_since) >= JCT_LINE_CONFIRM_MS) jct_scan_latched = true;
  } else {
    jct_scan_seen_since = 0;
  }
  return jct_scan_latched;
}

void resetJunctionRefs() {
  jct_saw_entry_corners = false;
  jct_scan_seen_since = 0;
  jct_scan_latched = false;
  ref_center_before = false; ref_center_mid = false; ref_center_after = false;
  ref_left45 = false; ref_left90 = false;
  ref_right45 = false; ref_right90 = false;
  jct_arm_L = false; jct_arm_F = false; jct_arm_R = false;
  jct_choice = '?';
  jct_selected_exit = -1;
  jct_execute_ms = 0;
  for (int i = 0; i < MAX_JUNCTION_EXITS; i++) jct_exit_actions[i] = 0;
  jct_exit_actions[0] = 'B';
  jct_exit_count = 1;
  strcpy(jct_exit_labels, "0B");
}

void buildExitTable() {
  jct_exit_count = 0;
  jct_exit_actions[jct_exit_count++] = 'B';
  if (jct_arm_L && jct_exit_count < MAX_JUNCTION_EXITS) jct_exit_actions[jct_exit_count++] = 'L';
  if (jct_arm_F && jct_exit_count < MAX_JUNCTION_EXITS) jct_exit_actions[jct_exit_count++] = 'F';
  if (jct_arm_R && jct_exit_count < MAX_JUNCTION_EXITS) jct_exit_actions[jct_exit_count++] = 'R';
}
void updateExitLabels() {
  int w = 0;
  for (uint8_t i = 0; i < jct_exit_count && w < (int)sizeof(jct_exit_labels) - 3; i++) {
    jct_exit_labels[w++] = '0' + i;
    jct_exit_labels[w++] = jct_exit_actions[i];
    jct_exit_labels[w++] = ' ';
  }
  if (w > 0) w--;
  jct_exit_labels[w] = 0;
}
void finalizeEvidence() {
  int center_hits = (ref_center_before?1:0) + (ref_center_mid?1:0) + (ref_center_after?1:0);
  jct_arm_L = ref_left45 || ref_left90;
  jct_arm_R = ref_right45 || ref_right90;
  jct_arm_F = center_hits >= 2;
  buildExitTable();
  updateExitLabels();
}

// ===== mapare turn (din UI) -> actiune =====
// Conventie noua:
//   turns[i] = 0       -> u-turn (intotdeauna disponibil, indexul 0 din tabel)
//   turns[i] = N >= 1  -> a N-a iesire detectata, sortata stanga -> dreapta
// jct_exit_actions[] are: [0]='B', [1..count-1]= iesirile reale L/F/R in ordinea
// stanga -> dreapta (vezi buildExitTable). Deci putem folosi indexul direct.
char actionFromTurnIndex(int t) {
  if (t <= 0) return 'B';
  if (t < (int)jct_exit_count) return jct_exit_actions[t];
  // turn cere o iesire care nu a fost detectata -> fallback
  return jct_exit_actions[jct_exit_count - 1];
}

void prepareJunctionChoice(unsigned long now) {
  // Alege actiunea din secventa daca exista
  int planned_idx = (moves_seq_idx < TURNS_LEN) ? turns[moves_seq_idx] : -1;
  char chosen;
  if (planned_idx < 0) {
    // fara plan: prefera prima iesire reala (1), sau B daca nu exista
    chosen = (jct_exit_count >= 2) ? jct_exit_actions[1] : 'B';
  } else {
    chosen = actionFromTurnIndex(planned_idx);
  }

  jct_choice = chosen;
  // Marker exit selectat (pentru OLED)
  jct_selected_exit = 0;
  for (uint8_t i = 0; i < jct_exit_count; i++) {
    if (jct_exit_actions[i] == chosen) { jct_selected_exit = i; break; }
  }

  if (chosen == 'L')      jct_execute_ms = TURN_L_MS + COMMIT_DRIVE_MS;
  else if (chosen == 'R') jct_execute_ms = TURN_R_MS + COMMIT_DRIVE_MS;
  else if (chosen == 'F') jct_execute_ms = COMMIT_DRIVE_MS;
  else                    jct_execute_ms = UTURN_MIN_MS;

  last_event_time = now;
#if DEBUG_JUNCTION_STOP
  robotState = ST_JCT_STOP;
#else
  logMove(chosen);
  if (moves_seq_idx < TURNS_LEN) moves_seq_idx++;
  robotState = (chosen == 'B') ? ST_UTURN : ST_JCT_EXECUTE;
#endif
  stateEnterTime = now;
}

// ============ Joystick / IR / battery (identic) ============
bool joystickPressEdge() {
  static bool was_pressed = false;
  static unsigned long last_check = 0;
  unsigned long t = millis();
  if (t - last_check < JOY_DEBOUNCE_MS) return false;
  last_check = t;
  PCF8574Write(0x1F | PCF8574Read());
  byte joy = PCF8574Read() | 0xE0;
  bool now_pressed = (joy != 0xFF);
  bool edge = now_pressed && !was_pressed;
  was_pressed = now_pressed;
  return edge;
}
void waitJoystickRelease() {
  unsigned long start = millis();
  while (millis() - start < 50) ;
  while (true) {
    PCF8574Write(0x1F | PCF8574Read());
    byte joy = PCF8574Read() | 0xE0;
    if (joy == 0xFF) break;
    delay(10);
  }
}
long readVccMilliVolts() {
#if defined(__AVR__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) ;
  long result = ADCL; result |= ADCH << 8;
  result = 1125300L / result;
  return result;
#else
  return 5000;
#endif
}
int readBatteryPercent() {
  long mv = readVccMilliVolts();
  int pct = (int)((mv - 4600L) * 100L / 700L);
  return constrain(pct, 0, 100);
}
bool infraredObstacleAhead() {
  PCF8574Write(0xC0 | PCF8574Read());
  byte ir = PCF8574Read() | 0x3F;
  return (ir != 0xFF);
}

// ============ OLED ============
void printStateCode() {
  switch (robotState) {
    case ST_NORMAL:        display.print(F("FOL")); break;
    case ST_LOST:          display.print(F("LST")); break;
    case ST_CP_CANDIDATE:  display.print(F("CP?")); break;
    case ST_CP_CONFIRMED:  display.print(F("CP!")); break;
    case ST_CP_STOP:       display.print(F("CPS")); break;
    case ST_UTURN:         display.print(F("UTN")); break;
    case ST_JCT_ENTRY:     display.print(F("ENT")); break;
    case ST_JCT_L45_TURN:  display.print(F("L45")); break;
    case ST_JCT_L45_SCAN:  display.print(F("S45")); break;
    case ST_JCT_L90_TURN:  display.print(F("L90")); break;
    case ST_JCT_L90_SCAN:  display.print(F("S90")); break;
    case ST_JCT_BACK_CENTER_FROM_LEFT:  display.print(F("C<L")); break;
    case ST_JCT_CENTER_SCAN_1:          display.print(F("SC1")); break;
    case ST_JCT_R45_TURN:  display.print(F("R45")); break;
    case ST_JCT_R45_SCAN:  display.print(F("s45")); break;
    case ST_JCT_R90_TURN:  display.print(F("R90")); break;
    case ST_JCT_R90_SCAN:  display.print(F("s90")); break;
    case ST_JCT_BACK_CENTER_FROM_RIGHT: display.print(F("C>R")); break;
    case ST_JCT_CENTER_SCAN_2:          display.print(F("SC2")); break;
    case ST_JCT_STOP:      display.print(F("JST")); break;
    case ST_JCT_EXECUTE:   display.print(F("JEX")); break;
  }
}

void showStatus() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  printStateCode();
  display.print(F(" CP:"));
  display.print(checkpoint_count);

  if (robotState >= ST_JCT_ENTRY && robotState <= ST_JCT_EXECUTE) {
    display.setCursor(0, 12);
    display.print(F("Ex:")); display.print(jct_exit_count);
    display.print(F(" Sel:"));
    if (jct_selected_exit >= 0) display.print(jct_selected_exit); else display.print("-");
    display.setCursor(0, 24);
    display.print(jct_exit_labels);
    display.setCursor(0, 36);
    display.print(F("L:")); display.print(jct_arm_L?1:0);
    display.print(F(" F:")); display.print(jct_arm_F?1:0);
    display.print(F(" R:")); display.print(jct_arm_R?1:0);
    display.setCursor(0, 48);
    display.print(F("Seq:")); display.print(moves_seq_idx);
    display.print(F("/")); display.print(TURNS_LEN);
    display.print(F(" ")); display.print(jct_choice);
    display.setCursor(0, 56);
    if (robotState == ST_JCT_STOP) display.print(F("Press JOY"));
    else {
      display.print(F("4:"));
      display.print(ref_left45?1:0); display.print(ref_left90?1:0);
      display.print(ref_right45?1:0); display.print(ref_right90?1:0);
    }
  } else {
    display.setTextSize(3);
    display.setCursor(82, 2);
    display.print(checkpoint_count);
    display.setTextSize(1);
    display.setCursor(0, 42);
    display.print(F("M:")); display.print(move_log);
    display.setCursor(0, 56);
#if ENABLE_ODOMETRY && SHOW_ODOMETRY_ON_OLED
    display.print(F("X:")); display.print((int)odo_x_mm);
    display.print(F(" Y:")); display.print((int)odo_y_mm);
    display.print(F(" T:")); display.print((int)(odo_theta * 57.2958f));
#else
    display.print(F("Seq:")); display.print(moves_seq_idx);
    display.print(F("/")); display.print(TURNS_LEN);
#endif
  }
  display.display();
}

// ============ SETUP (identic cu original) ============
void setup() {
  pinMode(8, OUTPUT); digitalWrite(8, LOW);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2); display.setTextColor(WHITE);
  display.setCursor(10, 0);  display.println(F("WaveShare"));
  display.setCursor(22, 25); display.println(F("DC/AC"));
  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print(F("Battery: ")); display.print(readBatteryPercent()); display.println(F("%"));
  display.setCursor(10, 55); display.println(F("Press to calibrate"));
  display.display();

  value = 0;
  while (value != 0xEF) {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
    delay(20);
  }

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  setMotors(0, 0);

  RGB.begin();
  for (int k = 0; k < 4; k++) RGB.setPixelColor(k, 0x00FF00);
  RGB.show();
  delay(500);

  // Calibrare TRSensors (rotire stanga-dreapta)
  analogWrite(PWMA, 65); analogWrite(PWMB, 65);
  for (int i = 0; i < 100; i++) {
    if (i < 25 || i >= 75) {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    } else {
      digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    }
    trs.calibrate();
    delay(5);
  }
  setMotors(0, 0);

  for (int k = 0; k < 4; k++) RGB.setPixelColor(k, 0x0000FF);
  RGB.show();

  value = 0;
  while (value != 0xEF) {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
    position = trs.readLine(sensorValues) / 200;
    display.clearDisplay();
    display.setCursor(0, 25); display.println(F("Calibration Done !!!"));
    display.setCursor(0, 55);
    for (int i = 0; i < 21; i++) display.print(F("_"));
    display.setCursor(position * 6, 55);
    display.print(F("**"));
    display.display();
    delay(20);
  }

  resetJunctionRefs();
  buildExitTable();
  updateExitLabels();
  showStatus();
  delay(500);
  waitJoystickRelease();
  unsigned long t_start = millis();
  stateEnterTime = t_start;
  last_event_time = t_start;
  // === FIX pornire ===
  // 1) Ignora CP in primele 1500ms (robotul poate fi pe patch negru de start)
  checkpoint_lockout_until = t_start + 1500;
  // 2) Reseteaza odometria daca este activata
#if ENABLE_ODOMETRY
  odo_x_mm = 0; odo_y_mm = 0; odo_theta = 0;
  odo_dist_mm = 0;
  odo_dist_at_last_jct = 0;
  odo_last_update = t_start;
#endif
  // 3) Asigura ca robotul intra in ST_NORMAL (nu un state ramas)
  robotState = ST_NORMAL;
  integral = 0; last_error = 0;
  recent_side = 0;
  cp_dense_since = 0; cp_last_seen = 0;
  left_arm_since = 0; right_arm_since = 0;
  dense_since = 0; edge_left_since = 0; edge_right_since = 0;
  // 4) Beep + LED verde scurt ca semnal vizibil de "GO"
  for (int k = 0; k < 4; k++) RGB.setPixelColor(k, 0x00FF00);
  RGB.show();
}

// ============ LOOP ============
void loop() {
  unsigned long now = millis();
  updateOdometry(now);
  position = trs.readLine(sensorValues);
  int error = (int)position - 2000;

  int sensors_black = 0, sensors_white = 0;
  int cp_black_count = 0;  // count cu pragul mai relaxat pentru CP
  for (int k = 0; k < NUM_SENSORS; k++) {
    if (sensorValues[k] > BLACK_THRESHOLD) sensors_black++;
    if (sensorValues[k] < WHITE_THRESHOLD) sensors_white++;
    if (sensorValues[k] > CP_BLACK_THRESHOLD) cp_black_count++;
  }
  bool all_white    = (sensors_white == NUM_SENSORS);
  bool line_visible = !all_white;
  bool dense_black  = (sensors_black == 5);
  // semnatura CP: >= 4 din 5 senzori vad negru cu prag relaxat
  bool cp_pad_seen  = (cp_black_count >= CP_MIN_BLACK_COUNT);

  // Senzori cu praguri RELAXATE pentru detectie junction
  bool s0 = sensorValues[0] > JCT_SIDE_THRESHOLD;
  bool s1 = sensorValues[1] > JCT_INNER_THRESHOLD;
  bool s2 = sensorValues[2] > CENTER_BLACK;
  bool s3 = sensorValues[3] > JCT_INNER_THRESHOLD;
  bool s4 = sensorValues[4] > JCT_SIDE_THRESHOLD;

  // Live runs (debug OLED)
  static unsigned long left_black_start = 0, right_black_start = 0;
  if (s0) { if (left_black_start == 0) left_black_start = now; live_left_run = now - left_black_start; }
  else    { left_black_start = 0; live_left_run = 0; }
  if (s4) { if (right_black_start == 0) right_black_start = now; live_right_run = now - right_black_start; }
  else    { right_black_start = 0; live_right_run = 0; }

  // ===== Detectie saritura brusca de eroare (semn de bifurcatie Y) =====
  static int prev_error = 0;
  static unsigned long err_jump_since = 0;
  int err_delta = abs(error - prev_error);
  if (err_delta >= JCT_ERR_JUMP) { if (err_jump_since == 0) err_jump_since = now; }
  else if (now - err_jump_since > 100) err_jump_since = 0;
  prev_error = error;

  // ===== Trackere trigger junction =====
  // side: lateral + centru (T-uri si cross-uri clasice)
  if ((s0 || s1) && s2) { if (left_arm_since == 0) left_arm_since = now; } else left_arm_since = 0;
  if ((s4 || s3) && s2) { if (right_arm_since == 0) right_arm_since = now; } else right_arm_since = 0;
  // dense: 4-5 negri (cu prag normal)
  if (sensors_black >= 4) { if (dense_since == 0) dense_since = now; } else dense_since = 0;
  // edge: doar lateral cu error mare (Y-uri, tangente)
  if (s0 && abs(error) > 500) { if (edge_left_since == 0) edge_left_since = now; } else edge_left_since = 0;
  if (s4 && abs(error) > 500) { if (edge_right_since == 0) edge_right_since = now; } else edge_right_since = 0;
  // NOU: widen - linia "se lateste". Centru vizibil + oricare lateral activ
  static unsigned long widen_since = 0;
  bool widen_now = s2 && (s0 || s4);
  if (widen_now) { if (widen_since == 0) widen_since = now; } else widen_since = 0;

  bool arm_left_side  = left_arm_since  && (now - left_arm_since)  >= (JCT_ARM_SIDE_MS + JCT_CONFIRM_MS);
  bool arm_right_side = right_arm_since && (now - right_arm_since) >= (JCT_ARM_SIDE_MS + JCT_CONFIRM_MS);
  bool arm_dense      = dense_since     && (now - dense_since)     >= (JCT_ARM_DENSE_MS + JCT_CONFIRM_MS);
  bool arm_edge_l     = edge_left_since && (now - edge_left_since) >= (JCT_ARM_EDGE_MS + JCT_CONFIRM_MS);
  bool arm_edge_r     = edge_right_since && (now - edge_right_since) >= (JCT_ARM_EDGE_MS + JCT_CONFIRM_MS);
  bool arm_err_jump   = err_jump_since != 0;
  bool arm_widen      = widen_since && (now - widen_since) >= JCT_ARM_WIDEN_MS;

  // Gating intre 2 intersectii (anti-rebound)
#if ENABLE_ODOMETRY
  bool jct_gap_ok = (odo_dist_mm - odo_dist_at_last_jct) >= (float)JCT_MIN_DIST_MM;
#else
  bool jct_gap_ok = (now - last_event_time) > JCT_COOLDOWN_MS;
#endif

  // Trigger ferm: ORICE modalitate persistenta declanseaza
  bool jct_armed = jct_gap_ok && (
                     arm_left_side || arm_right_side || arm_dense ||
                     arm_edge_l || arm_edge_r || arm_widen || arm_err_jump
                   );

  // ===== Tracker CP cu grace period =====
  // Folosim cp_pad_seen (4/5 cu prag relaxat) si toleram mici intreruperi
  bool cp_allowed = now >= checkpoint_lockout_until;
  // (cp_last_seen e global)
  if (cp_allowed && (robotState == ST_NORMAL || robotState == ST_LOST)) {
    if (cp_pad_seen) {
      if (cp_dense_since == 0) cp_dense_since = now;
      cp_last_seen = now;
    } else if (cp_dense_since != 0 && (now - cp_last_seen) > CP_GLITCH_GRACE_MS) {
      // intrerupere prea lunga -> resetam
      cp_dense_since = 0;
    }
  } else { cp_dense_since = 0; cp_last_seen = 0; }
  bool cp_prelim = cp_allowed && cp_dense_since && ((now - cp_dense_since) >= CP_PRELIM_MS);

  // recent_side pentru recovery
  if (robotState == ST_NORMAL && line_visible && !dense_black) {
    if (error > 300) recent_side = 1;
    else if (error < -300) recent_side = -1;
  }
  bool centered_line = s2 && abs(error) < 900;

  // Obstacole
  if (infraredObstacleAhead()) { if (obstacle_seen_since == 0) obstacle_seen_since = now; }
  else obstacle_seen_since = 0;
  bool obstacle_blocking = (obstacle_seen_since != 0) && ((now - obstacle_seen_since) >= OBSTACLE_DEBOUNCE_MS);

  // Stuck watchdog
  if (robotState == ST_NORMAL && (now - last_event_time) > STUCK_MAX_MS) {
    logMove('U'); last_event_time = now;
    robotState = ST_UTURN; stateEnterTime = now;
  }

  // ============ STATE MACHINE ============
  switch (robotState) {
    case ST_NORMAL:
      if (obstacle_blocking) {
        setMotors(0, 0); delay(OBSTACLE_STOP_MS);
        logMove('U'); last_event_time = millis();
        robotState = ST_UTURN; stateEnterTime = millis();
        obstacle_seen_since = 0;
      }
      // PRIORITATE: CP candidate inainte de junction (un patrat plin
      // declanseaza si "dense" deci ar putea fi confundat cu junction).
      else if (cp_prelim) {
        cp_enter_time = cp_dense_since;
        last_event_time = now;
        robotState = ST_CP_CANDIDATE;
        stateEnterTime = now;
      }
      else if (jct_armed) {
        integral = 0; last_error = 0;
        resetJunctionRefs();
#if ENABLE_ODOMETRY
        odo_dist_at_last_jct = odo_dist_mm;
#endif
        robotState = ST_JCT_ENTRY;
        stateEnterTime = now;
      }
      else if (all_white) {
        robotState = ST_LOST; stateEnterTime = now;
      }
      break;

    case ST_LOST:
      if (obstacle_blocking) {
        setMotors(0, 0); delay(OBSTACLE_STOP_MS);
        logMove('U'); last_event_time = millis();
        robotState = ST_UTURN; stateEnterTime = millis();
        obstacle_seen_since = 0;
      } else if (cp_prelim) {
        cp_enter_time = cp_dense_since;
        last_event_time = now;
        robotState = ST_CP_CANDIDATE; stateEnterTime = now;
      } else if (line_visible) {
        robotState = ST_NORMAL; stateEnterTime = now;
      } else if ((now - stateEnterTime) > LOST_GRACE_MS) {
        logMove('U'); last_event_time = now;
        robotState = ST_UTURN; stateEnterTime = now;
      }
      break;

    case ST_CP_CANDIDATE: {
      // Avans incet drept; daca cp_pad_seen rezista CP_FULL_MS -> CP confirmat.
      // Tolerez intreruperi scurte (CP_GLITCH_GRACE_MS).
      // Daca scade sub prag mai mult de grace -> a fost cross/junction.
      bool cp_lost_too_long = !cp_pad_seen && cp_last_seen != 0 &&
                              (now - cp_last_seen) > CP_GLITCH_GRACE_MS;
      if (cp_lost_too_long) {
        // A fost doar tranzitie (ex. cross). Trecem la junction logic.
        cp_dense_since = 0;
        resetJunctionRefs();
#if ENABLE_ODOMETRY
        odo_dist_at_last_jct = odo_dist_mm;
#endif
        robotState = ST_JCT_ENTRY;
        stateEnterTime = now;
        last_event_time = now;
      } else if ((now - cp_enter_time) >= (CP_PRELIM_MS + CP_FULL_MS)) {
        // CP confirmat
        checkpoint_count++;
        checkpoint_lockout_until = now + CP_LOCKOUT_MS;
        logMove('C');
        // CP-ul corespunde unui nod din path => consuma o intrare din turns[]
        if (moves_seq_idx < TURNS_LEN) moves_seq_idx++;
#if ENABLE_ODOMETRY
        odo_dist_at_last_jct = odo_dist_mm;
#endif
        last_event_time = now;
        robotState = ST_CP_CONFIRMED;
        stateEnterTime = now;
      }
      break;
    }

    case ST_CP_CONFIRMED: {
      // Mergi drept pana iesi din patrat (cp_black_count <= 2 cu pragul relaxat)
      if (cp_black_count <= 2) {
        // Verifica daca avem inca linie de centru pe care sa continuam
        bool has_line = s2 || (sensorValues[1] > BLACK_THRESHOLD) || (sensorValues[3] > BLACK_THRESHOLD);
        if (has_line) {
          robotState = ST_NORMAL; stateEnterTime = now; last_event_time = now;
        } else {
          // Capat de drum sau iesire stranga -> uturn
          logMove('U'); last_event_time = now;
          robotState = ST_UTURN; stateEnterTime = now;
        }
      } else if ((now - stateEnterTime) > CP_MAX_DURATION_MS) {
        // safety
        robotState = ST_NORMAL; stateEnterTime = now; last_event_time = now;
      }
      break;
    }

    case ST_CP_STOP:
      if ((now - stateEnterTime) > JOY_MIN_HOLD_MS && joystickPressEdge()) {
        robotState = ST_NORMAL; stateEnterTime = now; last_event_time = now;
        waitJoystickRelease();
      }
      break;

    case ST_UTURN:
      if ((now - stateEnterTime) > (UTURN_STOP_MS + UTURN_MIN_MS) && centered_line) {
        robotState = ST_NORMAL; stateEnterTime = now; last_event_time = now;
      }
      break;

    case ST_JCT_ENTRY:
      if (s0 || s4) jct_saw_entry_corners = true;
      if (((now - stateEnterTime) >= JCT_ENTRY_MIN_MS && jct_saw_entry_corners && !s0 && !s4) ||
          ((now - stateEnterTime) >= JCT_ENTRY_MAX_MS)) {
        ref_center_before = snapshotHasLineAhead() || sweepSampleHasLine();
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_L45_TURN; stateEnterTime = now;
      }
      break;
    case ST_JCT_L45_TURN:
      if ((now - stateEnterTime) >= JCT_STEP45_MS) {
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_L45_SCAN; stateEnterTime = now;
      } break;
    case ST_JCT_L45_SCAN:
      if ((now - stateEnterTime) >= JCT_SETTLE_MS) scanLineConfirmed(now);
      if ((now - stateEnterTime) >= (JCT_SETTLE_MS + JCT_SCAN_MS)) {
        ref_left45 = jct_scan_latched;
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_L90_TURN; stateEnterTime = now;
      } break;
    case ST_JCT_L90_TURN:
      if ((now - stateEnterTime) >= JCT_STEP45_MS) {
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_L90_SCAN; stateEnterTime = now;
      } break;
    case ST_JCT_L90_SCAN:
      if ((now - stateEnterTime) >= JCT_SETTLE_MS) scanLineConfirmed(now);
      if ((now - stateEnterTime) >= (JCT_SETTLE_MS + JCT_SCAN_MS)) {
        ref_left90 = jct_scan_latched;
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_BACK_CENTER_FROM_LEFT; stateEnterTime = now;
      } break;
    case ST_JCT_BACK_CENTER_FROM_LEFT:
      if ((now - stateEnterTime) >= JCT_RETURN90_MS) {
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_CENTER_SCAN_1; stateEnterTime = now;
      } break;
    case ST_JCT_CENTER_SCAN_1:
      if ((now - stateEnterTime) >= JCT_SETTLE_MS) scanLineConfirmed(now);
      if ((now - stateEnterTime) >= (JCT_SETTLE_MS + JCT_SCAN_MS)) {
        ref_center_mid = jct_scan_latched;
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_R45_TURN; stateEnterTime = now;
      } break;
    case ST_JCT_R45_TURN:
      if ((now - stateEnterTime) >= JCT_STEP45_MS) {
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_R45_SCAN; stateEnterTime = now;
      } break;
    case ST_JCT_R45_SCAN:
      if ((now - stateEnterTime) >= JCT_SETTLE_MS) scanLineConfirmed(now);
      if ((now - stateEnterTime) >= (JCT_SETTLE_MS + JCT_SCAN_MS)) {
        ref_right45 = jct_scan_latched;
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_R90_TURN; stateEnterTime = now;
      } break;
    case ST_JCT_R90_TURN:
      if ((now - stateEnterTime) >= JCT_STEP45_MS) {
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_R90_SCAN; stateEnterTime = now;
      } break;
    case ST_JCT_R90_SCAN:
      if ((now - stateEnterTime) >= JCT_SETTLE_MS) scanLineConfirmed(now);
      if ((now - stateEnterTime) >= (JCT_SETTLE_MS + JCT_SCAN_MS)) {
        ref_right90 = jct_scan_latched;
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_BACK_CENTER_FROM_RIGHT; stateEnterTime = now;
      } break;
    case ST_JCT_BACK_CENTER_FROM_RIGHT:
      if ((now - stateEnterTime) >= JCT_RETURN90_MS) {
        jct_scan_seen_since = 0; jct_scan_latched = false;
        robotState = ST_JCT_CENTER_SCAN_2; stateEnterTime = now;
      } break;
    case ST_JCT_CENTER_SCAN_2:
      if ((now - stateEnterTime) >= JCT_SETTLE_MS) scanLineConfirmed(now);
      if ((now - stateEnterTime) >= (JCT_SETTLE_MS + JCT_SCAN_MS)) {
        ref_center_after = jct_scan_latched;
        finalizeEvidence();
        prepareJunctionChoice(now);
      } break;

    case ST_JCT_STOP:
      if ((now - stateEnterTime) > JOY_MIN_HOLD_MS && joystickPressEdge()) {
        logMove(jct_choice);
        if (moves_seq_idx < TURNS_LEN) moves_seq_idx++;
        robotState = (jct_choice == 'B') ? ST_UTURN : ST_JCT_EXECUTE;
        stateEnterTime = now; last_event_time = now;
      }
      break;
    case ST_JCT_EXECUTE:
      if ((now - stateEnterTime) > jct_execute_ms) {
        robotState = ST_NORMAL; stateEnterTime = now; last_event_time = now;
      }
      break;
  }

  // ============ PID + ACTUARI ============
  if (robotState == ST_NORMAL && line_visible && abs(error) < INT_ACTIVE_ZONE && !dense_black) {
    integral += error;
    integral = constrain(integral, -INT_LIMIT, INT_LIMIT);
  } else integral = 0;

  int eff_error = error;
  if (abs(eff_error) < ERROR_DEADBAND) eff_error = 0;
  int derivative = eff_error - last_error;
  last_error = eff_error;
  int correction = eff_error / KP_DIV + derivative * KD_MULT + integral / KI_DIV;
  int base_speed = MAX_SPEED - (abs(error) * CURVE_SLOWDOWN) / 2000;
  base_speed = constrain(base_speed, MIN_SPEED, MAX_SPEED);

  int left_speed = 0, right_speed = 0;
  uint32_t led_color = 0x000000;

  switch (robotState) {
    case ST_NORMAL:
      if (dense_black) { left_speed = right_speed = INTERSECTION_SPEED; }
      else if (abs(error) > PIVOT_THRESHOLD) {
        int p = SHARP_PIVOT_SPEED;
        if (error > 0) { left_speed =  p; right_speed = -p; }
        else           { left_speed = -p; right_speed =  p; }
      } else {
        left_speed  = constrain(base_speed + correction, -MAX_SPEED, MAX_SPEED);
        right_speed = constrain(base_speed - correction, -MAX_SPEED, MAX_SPEED);
      }
      break;
    case ST_LOST: {
      int p = SHARP_PIVOT_SPEED;
      if (recent_side >= 0) { left_speed =  p; right_speed = -p; }
      else                  { left_speed = -p; right_speed =  p; }
      led_color = 0xFF0000;
      break;
    }
    case ST_CP_CANDIDATE:
      // continua drept incet; PID pe linie centrata = 0 (toti senzorii negri)
      left_speed = right_speed = INTERSECTION_SPEED;
      led_color = 0xFFFFFF;
      break;
    case ST_CP_CONFIRMED:
      left_speed = right_speed = INTERSECTION_SPEED;
      led_color = 0x00FF00;
      break;
    case ST_CP_STOP:
      left_speed = right_speed = 0;
      led_color = 0xFFFFFF;
      break;
    case ST_UTURN: {
      if ((now - stateEnterTime) < UTURN_STOP_MS) { left_speed = 0; right_speed = 0; }
      else {
        int p = SHARP_PIVOT_SPEED;
        if (recent_side >= 0) { left_speed =  p; right_speed = -p; }
        else                  { left_speed = -p; right_speed =  p; }
      }
      led_color = 0xFF00FF;
      break;
    }
    case ST_JCT_ENTRY: left_speed = right_speed = INTERSECTION_SPEED; led_color = 0x40FF40; break;
    case ST_JCT_L45_TURN:
    case ST_JCT_L90_TURN: left_speed = -JCT_TURN_SPEED; right_speed = JCT_TURN_SPEED; led_color = 0x00FF80; break;
    case ST_JCT_L45_SCAN:
    case ST_JCT_L90_SCAN: left_speed = 0; right_speed = 0; led_color = 0x00AA66; break;
    case ST_JCT_BACK_CENTER_FROM_LEFT: left_speed = JCT_TURN_SPEED; right_speed = -JCT_TURN_SPEED; led_color = 0x00A0FF; break;
    case ST_JCT_CENTER_SCAN_1: left_speed = 0; right_speed = 0; led_color = 0x2080FF; break;
    case ST_JCT_R45_TURN:
    case ST_JCT_R90_TURN: left_speed = JCT_TURN_SPEED; right_speed = -JCT_TURN_SPEED; led_color = 0x0080FF; break;
    case ST_JCT_R45_SCAN:
    case ST_JCT_R90_SCAN: left_speed = 0; right_speed = 0; led_color = 0x4444FF; break;
    case ST_JCT_BACK_CENTER_FROM_RIGHT: left_speed = -JCT_TURN_SPEED; right_speed = JCT_TURN_SPEED; led_color = 0x8040FF; break;
    case ST_JCT_CENTER_SCAN_2: left_speed = 0; right_speed = 0; led_color = 0xAA44FF; break;
    case ST_JCT_STOP: left_speed = 0; right_speed = 0; led_color = 0x0000FF; break;
    case ST_JCT_EXECUTE: {
      unsigned long t = now - stateEnterTime;
      if (jct_choice == 'L') {
        if (t < TURN_L_MS) { left_speed = -JCT_TURN_SPEED; right_speed = JCT_TURN_SPEED; led_color = 0x00FFFF; }
        else { left_speed = right_speed = INTERSECTION_SPEED; led_color = 0xFFFFFF; }
      } else if (jct_choice == 'R') {
        if (t < TURN_R_MS) { left_speed = JCT_TURN_SPEED; right_speed = -JCT_TURN_SPEED; led_color = 0xFFFF00; }
        else { left_speed = right_speed = INTERSECTION_SPEED; led_color = 0xFFFFFF; }
      } else { left_speed = right_speed = INTERSECTION_SPEED; led_color = 0xFFFFFF; }
      break;
    }
  }

  setMotors(left_speed, right_speed);

  if (robotState == ST_NORMAL) {
    if ((now - last_led_time) > 200) {
      last_led_time = now;
      for (i_led = 0; i_led < RGB.numPixels(); i_led++) {
        RGB.setPixelColor(i_led, Wheel(((i_led * 256 / RGB.numPixels()) + j_led) & 255));
      }
      RGB.show();
      if (j_led++ > 256 * 4) j_led = 0;
    }
  } else {
    for (int k = 0; k < 4; k++) RGB.setPixelColor(k, led_color);
    RGB.show();
  }

  if ((now - last_display) > 200) { showStatus(); last_display = now; }
}

uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) return RGB.Color(WheelPos * 50, 255 - WheelPos * 50, 0);
  if (WheelPos < 170) { WheelPos -= 85; return RGB.Color(255 - WheelPos * 50, 0, WheelPos * 50); }
  WheelPos -= 170; return RGB.Color(0, WheelPos * 50, 255 - WheelPos * 50);
}
void PCF8574Write(byte data) {
  Wire.beginTransmission(Addr);
  Wire.write(data);
  Wire.endTransmission();
}
byte PCF8574Read() {
  int data = -1;
  Wire.requestFrom(Addr, 1);
  if (Wire.available()) data = Wire.read();
  return data;
}
