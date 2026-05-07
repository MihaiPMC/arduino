/*
  AlphaBot2-Ar PathRunner v3
  ----------------------------------------------------------
  Baza hardware: sketch-ul existent al utilizatorului
   (TRSensors, OLED SSD1306, NeoPixel, PCF8574 joystick + IR,
    motor driver standard AlphaBot2-Ar).

  NOTE LOGICA:
   - Explorare autonoma DFS: robotul construieste o harta topologica a
     labirintului in memorie si exploreaza fiecare ramura neexplorata.
   - Detectie checkpoint robusta: PATRAT NEGRU PLIN (4/5 senzori >= CP_FULL_MS).
   - Detectie junction multi-modala: T, Y, cross, curbe stranse, cercuri tangente.
   - Heading absolut: 0=N,1=E,2=S,3=W. Actualizat dupa fiecare viraj.
   - nd_exits[]: 2 biti per directie absoluta (N/E/S/W) = UNKNOWN/OPEN/WALL.
   - bt_state: BT_NONE=nod nou, BT_DEADEND=intoarcere din fund de sac,
               BT_POP=intoarcere DFS la parinte.
   - Senzori IR obstacol: detecteaza pereti in fata inainte de a intra pe drum.
   - Virajele stanga/dreapta se opresc cand linia noua este regasita,
     cu timeout de siguranta.

  Directii relative (pt executie viraj):
    DIR_BACK=0, DIR_LEFT=2, DIR_FRONT=4, DIR_RIGHT=6
*/

#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TRSensors.h"
#include <Wire.h>

// ============ Pins (din sketch-ul original) ============
#define PWMA 6
#define AIN2 A0
#define AIN1 A1
#define PWMB 5
#define BIN1 A2
#define BIN2 A3
#define PIN 7
#define NUM_SENSORS 5
#define OLED_RESET 9
#define Addr 0x20

// ============ Obstacle (IR) ============
#define OBSTACLE_STOP_MS 350
#define OBSTACLE_DEBOUNCE_MS 120

// ============ Motion / PID ============
#define MAX_SPEED 32
#define MIN_SPEED 30
#define CURVE_SLOWDOWN 45
#define PIVOT_THRESHOLD 1450
#define SHARP_PIVOT_SPEED 68
#define INTERSECTION_SPEED 20
#define ERROR_DEADBAND 70
#define KP_DIV 13
#define KD_MULT 1
#define KI_DIV 22000
#define INT_ACTIVE_ZONE 220
#define INT_LIMIT 5000

// ============ Praguri senzori (NESCHIMBATE) ============
#define BLACK_THRESHOLD 700
#define WHITE_THRESHOLD 250
#define CENTER_BLACK 550
#define SIDE_BLACK 600

// ============ CHECKPOINT (PATRAT NEGRU PLIN) - imbunatatit ============
#define CP_BLACK_THRESHOLD 500  // prag mai relaxat decat BLACK_THRESHOLD (patrat are reflexe variabile)
#define CP_APPROACH_BLACK_COUNT 3
#define CP_APPROACH_MS 35       // intra rapid in verificare CP; daca e +/T revine prin pending junction
#define CP_MIN_BLACK_COUNT 4    // 4 din 5 senzori negri = pe patrat (era 5 strict)
#define CP_FULL_MS 180          // durata totala pentru confirmare (redusa - era 220)
#define CP_SETTLE_MS 120        // intra putin mai mult in patrat inainte de decizie
#define CP_GLITCH_GRACE_MS 40   // toleranta scurta cand un senzor pierde negrul
#define CP_EXIT_BLACK_MAX 2     // <=2 senzori negri => iesit din patrat
#define CP_CONFIRM_MAX_MS 720   // daca nu devine checkpoint clar, revine la junction/linie
#define CP_APPROACH_SPEED 20
#define CP_LOCKOUT_MS 1500      // dupa CP, ignora alte detectii
#define CP_MAX_DURATION_MS 6000 // patratul de checkpoint poate fi mare
#define CP_REACQUIRE_MS 1200    // dupa patrat, mergi drept pana regasesti linia

// ============ JUNCTION (multi-modal, MAI SENSIBIL) ============
// Prag separat pentru senzorii laterali la detectie junction (mai relaxat decat SIDE_BLACK=600).
// Pe linii reale, s0/s4 nu ating mereu 600 cand intra ramura - cu 420 prindem mult mai repede.
#define JCT_SIDE_THRESHOLD 420
#define JCT_INNER_THRESHOLD 500 // pt s1/s3 (inner) la detectie laterala
#define JCT_ARM_SIDE_MS 35      // ramura pe s0/s4 + linie apropiata stabila
#define JCT_ARM_DENSE_MS 45     // 4-5 negri pentru cross/Y
#define JCT_ARM_EDGE_MS 70      // s0/s4 cu error mare (Y, tangente)
#define JCT_ARM_WIDEN_MS 45     // linia se "lateste": centru + lateral oricare
#define JCT_ARM_FORK_MS 35      // Y: ambele ramuri interioare apar fara patrat plin
#define JCT_ARM_SHALLOW_MS 55   // Y mic: ramura oblica apare ca exterior+interior pe aceeasi parte
#define JCT_CONFIRM_MS 15       // confirmare suplimentara impotriva glitch-urilor
#define JCT_MIN_DIST_MM 100     // distanta minima parcursa intre 2 junctions
#define JCT_ENTRY_MIN_MS 220
#define JCT_ENTRY_MAX_MS 320
#define JCT_TURN_SPEED 34
#define JCT_COOLDOWN_MS 700 // redus de la 900
#define JCT_ERR_JUMP 900    // saritura brusca de eroare = bifurcatie Y (era 1100)
#define JCT_ERR_JUMP_MS 25

// ============ Directii relative (executie viraj) ============
#define DIR_BACK 0
#define DIR_BACK_LEFT 1
#define DIR_LEFT 2
#define DIR_FRONT_LEFT 3
#define DIR_FRONT 4
#define DIR_FRONT_RIGHT 5
#define DIR_RIGHT 6
#define DIR_BACK_RIGHT 7

// ============ DFS explorare autonoma ============
#define MAX_NODES 24
#define DFS_DEPTH 22
#define EX_UNKNOWN 0
#define EX_OPEN    1
#define EX_WALL    2

// ============ ODOMETRIE (fara encoderi - bazata pe PWM*timp) ============
// Calibrare: la ~PWM 40 robotul merge cu ~85 mm/s.
// Deci 1 unitate PWM * 1 ms = 85/40/1000 = 0.00213 mm. Ajustabil empiric.
#define MM_PER_PWM_MS_X1000 213 // 0.213 mm * (PWM/100) * ms; vezi formula in cod
#define WHEEL_BASE_MM 105       // distanta intre roti pt rotatii
#define ODO_MAX_DT_MS 50        // ignora pasi mari (pauze, delay-uri)

// ============ Turn execution ============
#define TURN_L_MS 430
#define TURN_R_MS 320
#define TURN_LEFT_45_MIN_MS 160
#define TURN_RIGHT_45_MIN_MS 140
#define TURN_LEFT_90_MIN_MS 350
#define TURN_RIGHT_90_MIN_MS 240
#define TURN_LEFT_135_MIN_MS 550
#define TURN_RIGHT_135_MIN_MS 440
#define TURN_LEFT_45_MAX_MS 330
#define TURN_RIGHT_45_MAX_MS 270
#define TURN_LEFT_90_MAX_MS 650
#define TURN_RIGHT_90_MAX_MS 520
#define TURN_LEFT_135_MAX_MS 800
#define TURN_RIGHT_135_MAX_MS 600
#define TURN_ALIGN_CONFIRM_MS 70
#define COMMIT_DRIVE_MS 240
#define COMMIT_CORR_LIMIT 8

// ============ Circle / tight curve following ============
#define CIRCLE_INNER_SPEED 12
#define CIRCLE_OUTER_SPEED 32

// ============ Calibration ============
#define CALIBRATION_PWM 42
#define CALIBRATION_SAMPLES 130
#define CALIBRATION_DELAY_MS 8

// ============ Recovery ============
#define UTURN_MIN_MS 700
#define UTURN_STOP_MS 180
#define CP_UTURN_ADVANCE_SPEED 20
#define CP_UTURN_ADVANCE_MS 280
#define CP_UTURN_SPEED 30
#define CP_UTURN_PIVOT_MS 980
#define CP_UTURN_MIN_MS 420
#define CP_UTURN_MAX_MS 1400
#define CP_UTURN_STOP_MS 0
#define LOST_GRACE_MS 450
#define ENABLE_STUCK_WATCHDOG 0
#define STUCK_MAX_MS 30000

// ============ Misc ============
#define MOVE_LOG_SIZE 14
#define DEBUG_JUNCTION_STOP 0   // 1 = opreste la fiecare junction (apasa joy)
#define ENABLE_ODOMETRY 0       // 0 = pornire stabila pe UNO; 1 doar dupa ce confirmi ca exista RAM suficient
#define SHOW_ODOMETRY_ON_OLED 0 // UNO are RAM putin; 0 = afisaj sigur ca inainte
#define JOY_DEBOUNCE_MS 30
#define JOY_MIN_HOLD_MS 150

// ====================================================
// Harta DFS: nd_exits[i] = 2 biti per directie absoluta
//   bits[1:0]=N, [3:2]=E, [5:4]=S, [7:6]=W
//   0=UNKNOWN, 1=OPEN, 2=WALL
// ====================================================
uint8_t nd_exits[MAX_NODES];
uint8_t nd_flags[MAX_NODES]; // bit0 = checkpoint
uint8_t nd_count = 0;
uint8_t dfs_stk[DFS_DEPTH];
int8_t  dfs_top  = -1;
uint8_t cur_nd   = 0;
uint8_t robot_heading = 0; // 0=N,1=E,2=S,3=W
#define BT_NONE    0
#define BT_DEADEND 1
#define BT_POP     2
uint8_t bt_state = BT_NONE;
uint8_t dfs_last_abs_dir = 0;
bool    maze_done = false;

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
char move_log[MOVE_LOG_SIZE + 1] = {0};
int move_log_count = 0;
bool ex_left  = false; // ramura stanga detectata la intrare junction
bool ex_right = false; // ramura dreapta detectata
bool ex_front = false; // cale inainte existenta (blob exit)
bool ex_front_blocked = false; // IR obstacol in fata la blob exit
unsigned long live_left_run = 0;
unsigned long live_right_run = 0;

// ===== Odometrie =====
#if ENABLE_ODOMETRY
float odo_x_mm = 0.0f;                  // pozitia X relativa la start (mm)
float odo_y_mm = 0.0f;                  // pozitia Y relativa la start (mm)
float odo_theta = 0.0f;                 // unghiul curent (rad), 0 = directia de start
float odo_dist_mm = 0.0f;               // distanta totala parcursa (mm)
float odo_dist_at_last_jct = -10000.0f; // dist la ultima intersectie (anti dublu-trigger)
unsigned long odo_last_update = 0;
int last_pwm_left = 0;
int last_pwm_right = 0;
#endif

enum RobotState
{
  ST_NORMAL,
  ST_LOST,
  ST_CP_CANDIDATE,
  ST_CP_CONFIRMED,
  ST_CP_STOP,
  ST_UTURN,
  ST_JCT_ENTRY,
  ST_JCT_STOP,
  ST_JCT_EXECUTE,
  ST_DONE          // labirint complet explorat
};
RobotState robotState = ST_NORMAL;

// ===== timing trigger junction =====
unsigned long left_arm_since = 0;
unsigned long right_arm_since = 0;
unsigned long dense_since = 0;
unsigned long edge_left_since = 0;
unsigned long edge_right_since = 0;
unsigned long fork_since = 0;
unsigned long shallow_left_since = 0;
unsigned long shallow_right_since = 0;
// Mutate din static local in loop() pentru a putea fi resetate din resetArmTimers()
unsigned long widen_since = 0;
unsigned long err_jump_since = 0;
int prev_error = 0;
uint8_t err_jump_streak = 0; // frame-uri consecutive cu salt mare de eroare

// ===== timing CP =====
unsigned long cp_entry_since = 0;
unsigned long cp_entry_last_seen = 0;
unsigned long cp_dense_since = 0;
unsigned long cp_last_seen = 0;
unsigned long cp_exit_time = 0;
bool cp_jct_pending = false;

// ===== junction snapshots =====
bool jct_saw_entry_corners = false;
char jct_choice = '?';
int jct_target_dir = DIR_FRONT;
unsigned long jct_execute_ms = 0;
bool jct_turn_aligned = false;
unsigned long jct_align_seen_since = 0;
unsigned long jct_commit_start = 0;
bool uturn_from_checkpoint = false;

// --- Prototypes ---
void PCF8574Write(byte data);
byte PCF8574Read();
uint32_t Wheel(byte WheelPos);
void setMotors(int left, int right);
void logMove(char m);
void showStatus();
bool turnAlignmentLineSeen();
bool validDirection(int dir);
bool directionTurnsLeft(int dir);
bool directionTurnsRight(int dir);
unsigned int minAlignMsForDirection(int dir);
unsigned int maxAlignMsForDirection(int dir);
char directionLogChar(int dir);
void resetJunctionRefs();
void resetArmTimers();
// DFS
uint8_t getEx(uint8_t nd, uint8_t abs_dir);
void    setEx(uint8_t nd, uint8_t abs_dir, uint8_t v);
uint8_t headingLeft();
uint8_t headingRight();
uint8_t headingBack();
void    updateHeadingAfterTurn(int dir);
void    dfsArriveNewNode(bool is_cp);
void    dfsMarkExitsFromSensors(bool has_front, bool has_left, bool has_right, bool front_blocked);
int     dfsChooseDir();
void prepareJunctionChoice(unsigned long now, bool from_checkpoint);
void prepareCheckpointChoice(unsigned long now);
bool joystickPressEdge();
void waitJoystickRelease();
long readVccMilliVolts();
int readBatteryPercent();
bool infraredObstacleAhead();

// ============ Motoare (identic cu original) ============
// Odometrie pe baza ultimei comenzi PWM (fara encoderi)
void updateOdometry(unsigned long now)
{
#if ENABLE_ODOMETRY
  if (odo_last_update == 0)
  {
    odo_last_update = now;
    return;
  }
  unsigned long dt = now - odo_last_update;
  odo_last_update = now;
  if (dt == 0 || dt > ODO_MAX_DT_MS)
    return;
  // viteza liniara echivalenta in mm/s pentru fiecare roata:
  // v_roata_mm_s = pwm * (MM_PER_PWM_MS_X1000 / 1000) * (1000/ms_per_s simplificat)
  // simplificat: dist_roata_mm = pwm * MM_PER_PWM_MS_X1000 * dt / 100000
  float dl = (float)last_pwm_left * (float)dt * (float)MM_PER_PWM_MS_X1000 / 100000.0f;
  float dr = (float)last_pwm_right * (float)dt * (float)MM_PER_PWM_MS_X1000 / 100000.0f;
  float d = (dl + dr) * 0.5f;
  float dtheta = (dr - dl) / (float)WHEEL_BASE_MM;
  odo_theta += dtheta;
  // normalizare in (-PI, PI]
  while (odo_theta > 3.14159265f)
    odo_theta -= 6.28318530f;
  while (odo_theta < -3.14159265f)
    odo_theta += 6.28318530f;
  odo_x_mm += d * cos(odo_theta);
  odo_y_mm += d * sin(odo_theta);
  odo_dist_mm += fabs(d);
#endif
}
void setMotors(int left, int right)
{
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
#if ENABLE_ODOMETRY
  last_pwm_left = left;
  last_pwm_right = right;
#endif
  if (left >= 0)
  {
    analogWrite(PWMA, left);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  {
    analogWrite(PWMA, -left);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  if (right >= 0)
  {
    analogWrite(PWMB, right);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  {
    analogWrite(PWMB, -right);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
}

void logMove(char m)
{
  if (move_log_count < MOVE_LOG_SIZE)
  {
    move_log[move_log_count++] = m;
    move_log[move_log_count] = 0;
  }
  else
  {
    for (int k = 0; k < MOVE_LOG_SIZE - 1; k++)
      move_log[k] = move_log[k + 1];
    move_log[MOVE_LOG_SIZE - 1] = m;
    move_log[MOVE_LOG_SIZE] = 0;
  }
}

bool turnAlignmentLineSeen()
{
  bool c = sensorValues[2] > CENTER_BLACK;
  bool near_left = sensorValues[1] > JCT_INNER_THRESHOLD;
  bool near_right = sensorValues[3] > JCT_INNER_THRESHOLD;
  int black = 0;
  for (int k = 0; k < NUM_SENSORS; k++)
  {
    if (sensorValues[k] > CP_BLACK_THRESHOLD)
      black++;
  }

  // Nu oprim rotatia pe un patrat/checkpoint sau pe miezul gros al intersectiei.
  return c && (near_left || near_right || sensorValues[2] > BLACK_THRESHOLD) && black <= 3;
}

bool validDirection(int dir)
{
  return dir >= DIR_BACK && dir <= DIR_BACK_RIGHT;
}

bool directionTurnsLeft(int dir)
{
  return dir == DIR_BACK_LEFT || dir == DIR_LEFT || dir == DIR_FRONT_LEFT;
}

bool directionTurnsRight(int dir)
{
  return dir == DIR_FRONT_RIGHT || dir == DIR_RIGHT || dir == DIR_BACK_RIGHT;
}

unsigned int minAlignMsForDirection(int dir)
{
  switch (dir)
  {
  case DIR_BACK_LEFT:
    return TURN_LEFT_135_MIN_MS;
  case DIR_LEFT:
    return TURN_LEFT_90_MIN_MS;
  case DIR_FRONT_LEFT:
    return TURN_LEFT_45_MIN_MS;
  case DIR_FRONT_RIGHT:
    return TURN_RIGHT_45_MIN_MS;
  case DIR_RIGHT:
    return TURN_RIGHT_90_MIN_MS;
  case DIR_BACK_RIGHT:
    return TURN_RIGHT_135_MIN_MS;
  default:
    return 0;
  }
}

unsigned int maxAlignMsForDirection(int dir)
{
  switch (dir)
  {
  case DIR_BACK_LEFT:
    return TURN_LEFT_135_MAX_MS;
  case DIR_LEFT:
    return TURN_LEFT_90_MAX_MS;
  case DIR_FRONT_LEFT:
    return TURN_LEFT_45_MAX_MS;
  case DIR_FRONT_RIGHT:
    return TURN_RIGHT_45_MAX_MS;
  case DIR_RIGHT:
    return TURN_RIGHT_90_MAX_MS;
  case DIR_BACK_RIGHT:
    return TURN_RIGHT_135_MAX_MS;
  default:
    return 0;
  }
}

char directionLogChar(int dir)
{
  return validDirection(dir) ? (char)('0' + dir) : '?';
}

void resetJunctionRefs()
{
  jct_saw_entry_corners = false;
  jct_choice = '?';
  jct_target_dir = DIR_FRONT;
  jct_execute_ms = 0;
  jct_turn_aligned = false;
  jct_align_seen_since = 0;
  jct_commit_start = 0;
  uturn_from_checkpoint = false;
}

// Reseteaza toti acumulatorii de timp pentru detectia de junction.
// Se apeleaza la tranzitia in stari de junction/CP pentru a preveni
// re-triggerarea imediata din timere vechi.
void resetArmTimers()
{
  left_arm_since = 0;
  right_arm_since = 0;
  dense_since = 0;
  edge_left_since = 0;
  edge_right_since = 0;
  fork_since = 0;
  shallow_left_since = 0;
  shallow_right_since = 0;
  widen_since = 0;
  err_jump_since = 0;
  err_jump_streak = 0;
  prev_error = 0;
}

// ============ DFS helpers ============
uint8_t getEx(uint8_t nd, uint8_t abs_dir)
{
  return (nd_exits[nd] >> (abs_dir * 2)) & 0x03;
}
void setEx(uint8_t nd, uint8_t abs_dir, uint8_t v)
{
  nd_exits[nd] = (nd_exits[nd] & ~(0x03u << (abs_dir * 2))) | ((v & 3u) << (abs_dir * 2));
}
uint8_t headingLeft()  { return (robot_heading + 3) & 3; }
uint8_t headingRight() { return (robot_heading + 1) & 3; }
uint8_t headingBack()  { return (robot_heading + 2) & 3; }

void updateHeadingAfterTurn(int dir)
{
  switch (dir)
  {
  case DIR_LEFT:
  case DIR_FRONT_LEFT:
    robot_heading = headingLeft();  break;
  case DIR_RIGHT:
  case DIR_FRONT_RIGHT:
    robot_heading = headingRight(); break;
  case DIR_BACK:
  case DIR_BACK_LEFT:
  case DIR_BACK_RIGHT:
    robot_heading = headingBack();  break;
  default: break; // DIR_FRONT: fara schimbare
  }
}

// Creeaza nod nou si impinge pe stiva DFS.
// Marcheaza iesirea parintelui (dfs_last_abs_dir) ca OPEN si iesirea
// "inapoi" a nodului nou ca OPEN.
void dfsArriveNewNode(bool is_cp)
{
  // Marcheaza iesirea parintelui spre acest nod
  if (dfs_top >= 0)
    setEx(dfs_stk[dfs_top], dfs_last_abs_dir, EX_OPEN);

  uint8_t new_nd = (nd_count < MAX_NODES) ? nd_count++ : (MAX_NODES - 1);
  nd_exits[new_nd] = 0;
  nd_flags[new_nd] = is_cp ? 1 : 0;

  if (dfs_top < DFS_DEPTH - 1)
    dfs_stk[++dfs_top] = new_nd;

  cur_nd = new_nd;
  // Back exit = de unde am venit
  setEx(cur_nd, headingBack(), EX_OPEN);
}

// Marcheaza iesirile nodului curent pe baza senzorilor capturati.
// Numai directiile necunoscute sunt actualizate (nu suprascrie OPEN).
void dfsMarkExitsFromSensors(bool has_front, bool has_left, bool has_right, bool front_blocked)
{
  uint8_t abs_f = robot_heading;
  uint8_t abs_l = headingLeft();
  uint8_t abs_r = headingRight();

  if ((!has_front || front_blocked) && getEx(cur_nd, abs_f) == EX_UNKNOWN)
    setEx(cur_nd, abs_f, EX_WALL);
  if (!has_left && getEx(cur_nd, abs_l) == EX_UNKNOWN)
    setEx(cur_nd, abs_l, EX_WALL);
  if (!has_right && getEx(cur_nd, abs_r) == EX_UNKNOWN)
    setEx(cur_nd, abs_r, EX_WALL);
}

// Alege directia urmatoare (DFS): prioritate front > left > right > backtrack.
// Seteaza dfs_last_abs_dir si bt_state.
int dfsChooseDir()
{
  uint8_t abs_f = robot_heading;
  uint8_t abs_l = headingLeft();
  uint8_t abs_r = headingRight();

  if (getEx(cur_nd, abs_f) == EX_UNKNOWN) { dfs_last_abs_dir = abs_f; return DIR_FRONT; }
  if (getEx(cur_nd, abs_l) == EX_UNKNOWN) { dfs_last_abs_dir = abs_l; return DIR_LEFT; }
  if (getEx(cur_nd, abs_r) == EX_UNKNOWN) { dfs_last_abs_dir = abs_r; return DIR_RIGHT; }

  // Toate iesirile explorate → backtrack la parinte
  if (dfs_top > 0)
  {
    dfs_top--;
    cur_nd = dfs_stk[dfs_top];
    bt_state = BT_POP;
    dfs_last_abs_dir = headingBack();
    return DIR_BACK;
  }
  // La radacina fara iesiri necunoscute → labirint complet
  maze_done = true;
  return DIR_FRONT;
}

void prepareJunctionChoice(unsigned long now, bool from_checkpoint)
{
  if (bt_state == BT_NONE)
  {
    dfsArriveNewNode(from_checkpoint);
    dfsMarkExitsFromSensors(ex_front, ex_left, ex_right, ex_front_blocked);
  }
  else if (bt_state == BT_DEADEND)
  {
    // Intoarcere din fund de sac; directia explorata era deja marcata WALL
    if (nd_count > 0)
      setEx(cur_nd, dfs_last_abs_dir, EX_WALL);
    bt_state = BT_NONE;
  }
  else // BT_POP
  {
    // Revenit la parinte din backtrack DFS
    bt_state = BT_NONE;
  }

  int planned_dir = dfsChooseDir();

  if (maze_done)
  {
    robotState = ST_DONE;
    stateEnterTime = now;
    return;
  }

  jct_target_dir = planned_dir;
  jct_choice = directionLogChar(planned_dir);
  uturn_from_checkpoint = from_checkpoint && (planned_dir == DIR_BACK);

  if (directionTurnsLeft(planned_dir) || directionTurnsRight(planned_dir))
    jct_execute_ms = maxAlignMsForDirection(planned_dir) + COMMIT_DRIVE_MS;
  else if (planned_dir == DIR_FRONT)
    jct_execute_ms = COMMIT_DRIVE_MS;
  else
    jct_execute_ms = UTURN_MIN_MS;

  jct_turn_aligned = false;
  jct_align_seen_since = 0;
  jct_commit_start = 0;
  last_event_time = now;

#if DEBUG_JUNCTION_STOP
  robotState = ST_JCT_STOP;
#else
  logMove(jct_choice);
  robotState = (planned_dir == DIR_BACK) ? ST_UTURN : ST_JCT_EXECUTE;
#endif
  stateEnterTime = now;
}

void prepareCheckpointChoice(unsigned long now)
{
  // Checkpoint: iesiri doar inainte sau inapoi (nu lateral)
  if (bt_state == BT_NONE)
  {
    dfsArriveNewNode(true);
    // Checkpoint nu are ramuri laterale
    dfsMarkExitsFromSensors(true, false, false, false);
  }
  else if (bt_state == BT_DEADEND)
  {
    if (nd_count > 0)
      setEx(cur_nd, dfs_last_abs_dir, EX_WALL);
    bt_state = BT_NONE;
  }
  else // BT_POP
  {
    bt_state = BT_NONE;
  }

  int planned_dir = dfsChooseDir();

  if (maze_done)
  {
    robotState = ST_DONE;
    stateEnterTime = now;
    return;
  }

  // Checkpoint permite doar front sau back
  if (planned_dir != DIR_BACK)
    planned_dir = DIR_FRONT;

  jct_target_dir = planned_dir;
  jct_choice = directionLogChar(planned_dir);
  jct_execute_ms = 0;
  jct_turn_aligned = false;
  jct_align_seen_since = 0;
  jct_commit_start = 0;
  uturn_from_checkpoint = (planned_dir == DIR_BACK);

  logMove(jct_choice);
  cp_exit_time = 0;
  last_event_time = now;
  stateEnterTime = now;
  robotState = (planned_dir == DIR_BACK) ? ST_UTURN : ST_CP_CONFIRMED;
}

// ============ Joystick / IR / battery (identic) ============
bool joystickPressEdge()
{
  static bool was_pressed = false;
  static unsigned long last_check = 0;
  unsigned long t = millis();
  if (t - last_check < JOY_DEBOUNCE_MS)
    return false;
  last_check = t;
  PCF8574Write(0x1F | PCF8574Read());
  byte joy = PCF8574Read() | 0xE0;
  bool now_pressed = (joy != 0xFF);
  bool edge = now_pressed && !was_pressed;
  was_pressed = now_pressed;
  return edge;
}
void waitJoystickRelease()
{
  unsigned long start = millis();
  while (millis() - start < 50)
    ;
  while (true)
  {
    PCF8574Write(0x1F | PCF8574Read());
    byte joy = PCF8574Read() | 0xE0;
    if (joy == 0xFF)
      break;
    delay(10);
  }
}
long readVccMilliVolts()
{
#if defined(__AVR__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  long result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result;
  return result;
#else
  return 5000;
#endif
}
int readBatteryPercent()
{
  long mv = readVccMilliVolts();
  int pct = (int)((mv - 4600L) * 100L / 700L);
  return constrain(pct, 0, 100);
}
bool infraredObstacleAhead()
{
  PCF8574Write(0xC0 | PCF8574Read());
  byte ir = PCF8574Read() | 0x3F;
  return (ir != 0xFF);
}

// ============ OLED ============
void printStateCode()
{
  switch (robotState)
  {
  case ST_NORMAL:
    display.print(F("FOL"));
    break;
  case ST_LOST:
    display.print(F("LST"));
    break;
  case ST_CP_CANDIDATE:
    display.print(F("CP?"));
    break;
  case ST_CP_CONFIRMED:
    display.print(F("CP!"));
    break;
  case ST_CP_STOP:
    display.print(F("CPS"));
    break;
  case ST_UTURN:
    display.print(F("UTN"));
    break;
  case ST_JCT_ENTRY:
    display.print(F("ENT"));
    break;
  case ST_JCT_STOP:
    display.print(F("JST"));
    break;
  case ST_JCT_EXECUTE:
    display.print(F("JEX"));
    break;
  case ST_DONE:
    display.print(F("DON"));
    break;
  }
}

void showStatus()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  printStateCode();
  display.print(F(" CP:"));
  display.print(checkpoint_count);

  if (robotState >= ST_JCT_ENTRY && robotState <= ST_JCT_EXECUTE)
  {
    display.setCursor(0, 12);
    display.print(F("Dir:"));
    display.print(jct_target_dir);
    display.print(F(" H:"));
    static const char* hnames[] = {"N","E","S","W"};
    display.print(hnames[robot_heading & 3]);
    display.setCursor(0, 24);
    display.print(F("Nd:"));
    display.print(cur_nd);
    display.print(F(" Stk:"));
    display.print((int)dfs_top + 1);
    display.print(F("/"));
    display.print(nd_count);
    display.setCursor(0, 36);
    display.print(F("BT:"));
    display.print(bt_state);
    display.print(F(" EX L"));
    display.print(ex_left);
    display.print(F("R"));
    display.print(ex_right);
    display.print(F("F"));
    display.print(ex_front);
    display.setCursor(0, 48);
    display.print(F("M:"));
    display.print(move_log);
    display.setCursor(0, 56);
    if (robotState == ST_JCT_STOP)
      display.print(F("Press JOY"));
    else
      display.print(F("DFS auto"));
  }
  else
  {
    display.setTextSize(3);
    display.setCursor(82, 2);
    display.print(checkpoint_count);
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print(F("Nd:"));
    display.print(cur_nd);
    display.print(F("/"));
    display.print(nd_count);
    display.print(F(" H:"));
    static const char* hn[] = {"N","E","S","W"};
    display.print(hn[robot_heading & 3]);
    display.setCursor(0, 36);
    display.print(F("Stk:"));
    display.print((int)dfs_top + 1);
    display.print(F(" BT:"));
    display.print(bt_state);
    display.setCursor(0, 42);
    display.print(F("M:"));
    display.print(move_log);
    display.setCursor(0, 56);
    display.print(maze_done ? F("MAZE DONE!") : F("Exploring"));
  }
  display.display();
}

// ============ SETUP (identic cu original) ============
void setup()
{
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println(F("WaveShare"));
  display.setCursor(22, 25);
  display.println(F("DC/AC"));
  display.setTextSize(1);
  display.setCursor(0, 45);
  display.print(F("Battery: "));
  display.print(readBatteryPercent());
  display.println(F("%"));
  display.setCursor(10, 55);
  display.println(F("Press to calibrate"));
  display.display();

  value = 0;
  while (value != 0xEF)
  {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
    delay(20);
  }

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  setMotors(0, 0);

  RGB.begin();
  for (int k = 0; k < 4; k++)
    RGB.setPixelColor(k, 0x00FF00);
  RGB.show();
  delay(500);

  // Calibrare TRSensors (rotire stanga-dreapta lenta, ca sa nu sara de pe traseu)
  analogWrite(PWMA, CALIBRATION_PWM);
  analogWrite(PWMB, CALIBRATION_PWM);
  for (int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    if (i < (CALIBRATION_SAMPLES / 4) || i >= (3 * CALIBRATION_SAMPLES / 4))
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    else
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
    trs.calibrate();
    delay(CALIBRATION_DELAY_MS);
  }
  setMotors(0, 0);

  for (int k = 0; k < 4; k++)
    RGB.setPixelColor(k, 0x0000FF);
  RGB.show();

  value = 0;
  while (value != 0xEF)
  {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
    position = trs.readLine(sensorValues) / 200;
    display.clearDisplay();
    display.setCursor(0, 25);
    display.println(F("Calibration Done !!!"));
    display.setCursor(0, 55);
    for (int i = 0; i < 21; i++)
      display.print(F("_"));
    display.setCursor(position * 6, 55);
    display.print(F("**"));
    display.display();
    delay(20);
  }

  // Initializare DFS
  memset(nd_exits, 0, sizeof(nd_exits));
  memset(nd_flags, 0, sizeof(nd_flags));
  nd_count = 0;
  dfs_top = -1;
  cur_nd = 0;
  robot_heading = 0; // start heading Nord
  bt_state = BT_NONE;
  dfs_last_abs_dir = 0;
  maze_done = false;
  ex_left = ex_right = ex_front = ex_front_blocked = false;

  resetJunctionRefs();
  showStatus();
  delay(500);
  waitJoystickRelease();
  unsigned long t_start = millis();
  stateEnterTime = t_start;
  last_event_time = t_start;
  // 1) Ignora CP in primele 1500ms (robotul poate fi pe patch negru de start)
  checkpoint_lockout_until = t_start + 1500;
  // 2) Reseteaza odometria daca este activata
#if ENABLE_ODOMETRY
  odo_x_mm = 0;
  odo_y_mm = 0;
  odo_theta = 0;
  odo_dist_mm = 0;
  odo_dist_at_last_jct = 0;
  odo_last_update = t_start;
#endif
  // 3) Asigura ca robotul intra in ST_NORMAL (nu un state ramas)
  robotState = ST_NORMAL;
  integral = 0;
  last_error = 0;
  recent_side = 0;
  cp_entry_since = 0;
  cp_entry_last_seen = 0;
  cp_dense_since = 0;
  cp_last_seen = 0;
  cp_exit_time = 0;
  cp_jct_pending = false;
  left_arm_since = 0;
  right_arm_since = 0;
  dense_since = 0;
  edge_left_since = 0;
  edge_right_since = 0;
  fork_since = 0;
  shallow_left_since = 0;
  shallow_right_since = 0;
  widen_since = 0;
  err_jump_since = 0;
  err_jump_streak = 0;
  prev_error = 0;
  // 4) Beep + LED verde scurt ca semnal vizibil de "GO"
  for (int k = 0; k < 4; k++)
    RGB.setPixelColor(k, 0x00FF00);
  RGB.show();
}

// ============ LOOP ============
void loop()
{
  unsigned long now = millis();
  updateOdometry(now);
  position = trs.readLine(sensorValues);
  int error = (int)position - 2000;

  int sensors_black = 0, sensors_white = 0;
  int cp_black_count = 0; // count cu pragul mai relaxat pentru CP
  for (int k = 0; k < NUM_SENSORS; k++)
  {
    if (sensorValues[k] > BLACK_THRESHOLD)
      sensors_black++;
    if (sensorValues[k] < WHITE_THRESHOLD)
      sensors_white++;
    if (sensorValues[k] > CP_BLACK_THRESHOLD)
      cp_black_count++;
  }
  bool all_white = (sensors_white == NUM_SENSORS);
  bool line_visible = !all_white;
  bool dense_black = (sensors_black == 5);
  // semnatura CP: >= 4 din 5 senzori vad negru cu prag relaxat
  bool cp_pad_seen = (cp_black_count >= CP_MIN_BLACK_COUNT);
  bool dense_square_like = cp_pad_seen && dense_black;

  // Senzori cu praguri RELAXATE pentru detectie junction
  bool s0 = sensorValues[0] > JCT_SIDE_THRESHOLD;
  bool s1 = sensorValues[1] > JCT_INNER_THRESHOLD;
  bool s2 = sensorValues[2] > CENTER_BLACK;
  bool s3 = sensorValues[3] > JCT_INNER_THRESHOLD;
  bool s4 = sensorValues[4] > JCT_SIDE_THRESHOLD;
  bool cp_entry_seen = cp_pad_seen ||
                       (cp_black_count >= CP_APPROACH_BLACK_COUNT &&
                        s2 && (s1 || s3) && (s0 || s4));

  // Live runs (debug OLED)
  static unsigned long left_black_start = 0, right_black_start = 0;
  if (s0)
  {
    if (left_black_start == 0)
      left_black_start = now;
    live_left_run = now - left_black_start;
  }
  else
  {
    left_black_start = 0;
    live_left_run = 0;
  }
  if (s4)
  {
    if (right_black_start == 0)
      right_black_start = now;
    live_right_run = now - right_black_start;
  }
  else
  {
    right_black_start = 0;
    live_right_run = 0;
  }

  // ===== Detectie saritura brusca de eroare (semn de bifurcatie Y) =====
  // prev_error, err_jump_since, err_jump_streak sunt globale (pot fi resetate din resetArmTimers).
  // Necesitam 2 frame-uri consecutive cu salt >= JCT_ERR_JUMP pentru a evita spike-uri
  // accidentale pe curbe stranse. Resetam imediat cand conditia inceteaza.
  int err_delta = abs(error - prev_error);
  bool err_jump_now = line_visible && !dense_black && (err_delta >= JCT_ERR_JUMP);
  if (err_jump_now)
  {
    err_jump_streak++;
    if (err_jump_streak >= 2 && err_jump_since == 0)
      err_jump_since = now;
  }
  else
  {
    err_jump_streak = 0;
    err_jump_since = 0;
  }
  prev_error = error;

  // ===== Trackere trigger junction =====
  // Curbe pure (numai senzorii ipsilaterali activi, fara centru sau parte opusa):
  bool left_curve_edge = (s0 || s1) && !s2 && !s3 && !s4;
  bool right_curve_edge = (s4 || s3) && !s2 && !s1 && !s0;

  // side: exterior + linie apropiata. Cerem si cel putin un senzor contralateral (s2/s3/s4
  // pentru stanga, s2/s1/s0 pentru dreapta) pentru a exclude curbe stranse unde numai
  // senzorii ipsilaterali (s0+s1 sau s4+s3) sunt activi fara centru.
  bool left_branch_shape  = s0 && (s1 || s2) && (s2 || s3 || s4);
  bool right_branch_shape = s4 && (s3 || s2) && (s2 || s1 || s0);
  bool y_fork_shape = !cp_pad_seen && !dense_black && s1 && s3 &&
                      (s2 || abs(error) < 700);
  bool shallow_left_shape = !dense_square_like && !cp_pad_seen && s0 && s1 &&
                            (s2 || s3 || abs(error) < 1200);
  bool shallow_right_shape = !dense_square_like && !cp_pad_seen && s4 && s3 &&
                             (s2 || s1 || abs(error) < 1200);
  if (left_branch_shape)
  {
    if (left_arm_since == 0)
      left_arm_since = now;
  }
  else
    left_arm_since = 0;
  if (right_branch_shape)
  {
    if (right_arm_since == 0)
      right_arm_since = now;
  }
  else
    right_arm_since = 0;
  // dense: 4-5 negri (cu prag normal)
  if (sensors_black >= 4 && !dense_square_like)
  {
    if (dense_since == 0)
      dense_since = now;
  }
  else
    dense_since = 0;
  // edge: doar lateral cu error mare (Y-uri, tangente)
  if (!left_curve_edge && s0 && abs(error) > 500)
  {
    if (edge_left_since == 0)
      edge_left_since = now;
  }
  else
    edge_left_since = 0;
  if (!right_curve_edge && s4 && abs(error) > 500)
  {
    if (edge_right_since == 0)
      edge_right_since = now;
  }
  else
    edge_right_since = 0;
  if (y_fork_shape)
  {
    if (fork_since == 0)
      fork_since = now;
  }
  else
    fork_since = 0;
  if (shallow_left_shape)
  {
    if (shallow_left_since == 0)
      shallow_left_since = now;
  }
  else
    shallow_left_since = 0;
  if (shallow_right_shape)
  {
    if (shallow_right_since == 0)
      shallow_right_since = now;
  }
  else
    shallow_right_since = 0;
  // widen - linia "se lateste". Centru vizibil + oricare lateral activ (global, nu static local)
  bool curve_widen = s2 && ((s0 && !s3 && !s4) || (s4 && !s0 && !s1));
  bool widen_now = !dense_square_like && s2 && (s0 || s4) && !curve_widen;
  if (widen_now)
  {
    if (widen_since == 0)
      widen_since = now;
  }
  else
    widen_since = 0;

  bool arm_left_side = left_arm_since && (now - left_arm_since) >= (JCT_ARM_SIDE_MS + JCT_CONFIRM_MS);
  bool arm_right_side = right_arm_since && (now - right_arm_since) >= (JCT_ARM_SIDE_MS + JCT_CONFIRM_MS);
  bool arm_dense = dense_since && (now - dense_since) >= (JCT_ARM_DENSE_MS + JCT_CONFIRM_MS);
  bool arm_edge_l = edge_left_since && (now - edge_left_since) >= (JCT_ARM_EDGE_MS + JCT_CONFIRM_MS);
  bool arm_edge_r = edge_right_since && (now - edge_right_since) >= (JCT_ARM_EDGE_MS + JCT_CONFIRM_MS);
  bool arm_err_jump = err_jump_since && (now - err_jump_since) >= JCT_ERR_JUMP_MS;
  bool arm_widen = widen_since && (now - widen_since) >= JCT_ARM_WIDEN_MS;
  bool arm_fork = fork_since && (now - fork_since) >= (JCT_ARM_FORK_MS + JCT_CONFIRM_MS);
  bool arm_shallow_l = shallow_left_since && (now - shallow_left_since) >= JCT_ARM_SHALLOW_MS;
  bool arm_shallow_r = shallow_right_since && (now - shallow_right_since) >= JCT_ARM_SHALLOW_MS;

  // Gating intre 2 intersectii (anti-rebound).
  // Bypass cooldown cand DFS se intoarce la un nod existent.
#if ENABLE_ODOMETRY
  bool jct_gap_ok = (bt_state != BT_NONE) ||
                    ((odo_dist_mm - odo_dist_at_last_jct) >= (float)JCT_MIN_DIST_MM);
#else
  bool jct_gap_ok = (bt_state != BT_NONE) || ((now - last_event_time) > JCT_COOLDOWN_MS);
#endif

  // Trigger ferm: ORICE modalitate persistenta declanseaza
  bool jct_armed = jct_gap_ok && (arm_left_side || arm_right_side || arm_dense ||
                                  arm_edge_l || arm_edge_r || arm_widen ||
                                  arm_err_jump || arm_fork ||
                                  arm_shallow_l || arm_shallow_r);

  // ===== Tracker CP cu grace period =====
  // Pornim pe un semn mai slab (3 senzori lati) ca sa nu ne fure junction-ul,
  // dar confirmam doar dupa 4/5 senzori negri stabil.
  bool cp_allowed = now >= checkpoint_lockout_until;
  bool cp_tracking_state = (robotState == ST_NORMAL || robotState == ST_LOST ||
                            robotState == ST_CP_CANDIDATE);
  if (cp_tracking_state && jct_armed)
    cp_jct_pending = true;
  if (cp_allowed && cp_tracking_state)
  {
    if (cp_entry_seen)
    {
      if (cp_entry_since == 0)
        cp_entry_since = now;
      cp_entry_last_seen = now;
    }
    else if (cp_entry_since != 0 && (now - cp_entry_last_seen) > CP_GLITCH_GRACE_MS)
    {
      cp_entry_since = 0;
    }

    if (cp_pad_seen)
    {
      if (cp_dense_since == 0)
        cp_dense_since = now;
      cp_last_seen = now;
    }
    else if (cp_dense_since != 0 && (now - cp_last_seen) > CP_GLITCH_GRACE_MS)
    {
      // intrerupere prea lunga -> resetam
      cp_dense_since = 0;
    }
  }
  else
  {
    cp_entry_since = 0;
    cp_entry_last_seen = 0;
    cp_dense_since = 0;
    cp_last_seen = 0;
    cp_jct_pending = false;
  }
  bool cp_prelim = cp_allowed && cp_dense_since && ((now - cp_dense_since) >= CP_APPROACH_MS);

  // recent_side pentru recovery
  if (robotState == ST_NORMAL && line_visible && !dense_black)
  {
    if (error > 300)
      recent_side = 1;
    else if (error < -300)
      recent_side = -1;
  }
  bool centered_line = s2 && abs(error) < 900 && !cp_pad_seen && cp_black_count <= CP_EXIT_BLACK_MAX;

  // Obstacole
  if (infraredObstacleAhead())
  {
    if (obstacle_seen_since == 0)
      obstacle_seen_since = now;
  }
  else
    obstacle_seen_since = 0;
  bool obstacle_blocking = (obstacle_seen_since != 0) && ((now - obstacle_seen_since) >= OBSTACLE_DEBOUNCE_MS);

  // Stuck watchdog
  if (ENABLE_STUCK_WATCHDOG && robotState == ST_NORMAL && !line_visible && (now - last_event_time) > STUCK_MAX_MS)
  {
    if (nd_count > 0)
      setEx(cur_nd, dfs_last_abs_dir, EX_WALL);
    bt_state = BT_DEADEND;
    uturn_from_checkpoint = false;
    logMove('U');
    last_event_time = now;
    robotState = ST_UTURN;
    stateEnterTime = now;
  }

  // ============ STATE MACHINE ============
  switch (robotState)
  {
  case ST_NORMAL:
    if (obstacle_blocking)
    {
      // Marcheaza directia curenta ca perete in harta DFS
      if (nd_count > 0)
        setEx(cur_nd, robot_heading, EX_WALL);
      bt_state = BT_DEADEND; // revenim la acelasi nod
      uturn_from_checkpoint = false;
      setMotors(0, 0);
      delay(OBSTACLE_STOP_MS);
      logMove('O');
      last_event_time = millis();
      robotState = ST_UTURN;
      stateEnterTime = millis();
      obstacle_seen_since = 0;
    }
    // PRIORITATE: CP candidate inainte de junction (un patrat plin
    // declanseaza si "dense" deci ar putea fi confundat cu junction).
    else if (cp_prelim)
    {
      last_event_time = now;
      robotState = ST_CP_CANDIDATE;
      stateEnterTime = now;
    }
    else if (jct_armed)
    {
      integral = 0;
      last_error = 0;
      // Captureaza iesirile detectate de timere inainte de reset
      ex_left  = arm_left_side || arm_edge_l || arm_shallow_l || arm_fork;
      ex_right = arm_right_side || arm_edge_r || arm_shallow_r || arm_fork;
      ex_front = false;
      ex_front_blocked = false;
      resetJunctionRefs();
      resetArmTimers();
      if (ex_left || ex_right || arm_dense || arm_widen)
        jct_saw_entry_corners = true;
#if ENABLE_ODOMETRY
      odo_dist_at_last_jct = odo_dist_mm;
#endif
      robotState = ST_JCT_ENTRY;
      stateEnterTime = now;
    }
    else if (all_white)
    {
      robotState = ST_LOST;
      stateEnterTime = now;
    }
    break;

  case ST_LOST:
    if (obstacle_blocking)
    {
      if (nd_count > 0)
        setEx(cur_nd, robot_heading, EX_WALL);
      bt_state = BT_DEADEND;
      uturn_from_checkpoint = false;
      setMotors(0, 0);
      delay(OBSTACLE_STOP_MS);
      logMove('O');
      last_event_time = millis();
      robotState = ST_UTURN;
      stateEnterTime = millis();
      obstacle_seen_since = 0;
    }
    else if (cp_prelim)
    {
      last_event_time = now;
      robotState = ST_CP_CANDIDATE;
      stateEnterTime = now;
    }
    else if (line_visible)
    {
      robotState = ST_NORMAL;
      stateEnterTime = now;
    }
    else if ((now - stateEnterTime) > LOST_GRACE_MS)
    {
      // Fund de sac: marcheaza directia explorata ca perete
      if (nd_count > 0)
        setEx(cur_nd, dfs_last_abs_dir, EX_WALL);
      bt_state = BT_DEADEND;
      uturn_from_checkpoint = false;
      logMove('U');
      last_event_time = now;
      robotState = ST_UTURN;
      stateEnterTime = now;
    }
    break;

  case ST_CP_CANDIDATE:
  {
    // Avans incet drept. Confirmam doar daca 4/5 senzori raman negri suficient,
    // apoi mai mergem foarte putin ca robotul sa fie in patrat, nu pe marginea lui.
    bool cp_lost_too_long = cp_entry_last_seen != 0 &&
                            (now - cp_entry_last_seen) > CP_GLITCH_GRACE_MS;
    bool cp_confirmed = cp_dense_since != 0 &&
                        (now - cp_dense_since) >= (CP_FULL_MS + CP_SETTLE_MS);
    bool cp_confirm_timeout = (now - stateEnterTime) > CP_CONFIRM_MAX_MS;
    if (cp_confirmed)
    {
      // CP confirmat. Nod DFS de tip checkpoint.
      checkpoint_count++;
      checkpoint_lockout_until = now + CP_LOCKOUT_MS;
      cp_exit_time = 0;
      logMove('C');
      cp_entry_since = 0;
      cp_entry_last_seen = 0;
      cp_dense_since = 0;
      cp_last_seen = 0;
      cp_jct_pending = false;
#if ENABLE_ODOMETRY
      odo_dist_at_last_jct = odo_dist_mm;
#endif
      last_event_time = now;
      resetJunctionRefs();
      resetArmTimers();
      prepareCheckpointChoice(now);
    }
    else if (cp_lost_too_long || cp_confirm_timeout)
    {
      // A fost doar o tranzitie lata (ex. cross/junction), nu patrat de checkpoint.
      cp_entry_since = 0;
      cp_entry_last_seen = 0;
      cp_dense_since = 0;
      cp_last_seen = 0;
      resetJunctionRefs();
      resetArmTimers();
#if ENABLE_ODOMETRY
      odo_dist_at_last_jct = odo_dist_mm;
#endif
      if (cp_jct_pending || jct_armed)
      {
        cp_jct_pending = false;
        // Blob dens → presupunem ramuri pe ambele parti (mai sigur decat WALL)
        ex_left  = true;
        ex_right = true;
        ex_front = false;
        ex_front_blocked = false;
        jct_saw_entry_corners = true;
        robotState = ST_JCT_ENTRY;
      }
      else
      {
        cp_jct_pending = false;
        robotState = line_visible ? ST_NORMAL : ST_LOST;
      }
      stateEnterTime = now;
      last_event_time = now;
    }
    break;
  }

  case ST_CP_CONFIRMED:
  {
    // Mergi drept peste patrat, apoi continua scurt pana regasesti linia.
    if (cp_black_count <= CP_EXIT_BLACK_MAX)
    {
      if (cp_exit_time == 0)
        cp_exit_time = now;
      bool has_line = s2 || (sensorValues[1] > JCT_INNER_THRESHOLD) ||
                      (sensorValues[3] > JCT_INNER_THRESHOLD);
      if (has_line)
      {
        cp_exit_time = 0;
        robotState = ST_NORMAL;
        stateEnterTime = now;
        last_event_time = now;
      }
      else if ((now - cp_exit_time) > CP_REACQUIRE_MS)
      {
        cp_exit_time = 0;
        robotState = ST_LOST;
        stateEnterTime = now;
        last_event_time = now;
      }
    }
    else if ((now - stateEnterTime) > CP_MAX_DURATION_MS)
    {
      // Daca patratul e foarte mare, nu intra in U-turn: mergi mai departe cautand linia.
      cp_exit_time = now;
    }
    break;
  }

  case ST_CP_STOP:
    if ((now - stateEnterTime) > JOY_MIN_HOLD_MS && joystickPressEdge())
    {
      robotState = ST_NORMAL;
      stateEnterTime = now;
      last_event_time = now;
      waitJoystickRelease();
    }
    break;

  case ST_UTURN:
  {
    if (uturn_from_checkpoint)
    {
      unsigned long turn_time = now - stateEnterTime;
      if (turn_time >= (CP_UTURN_ADVANCE_MS + CP_UTURN_PIVOT_MS))
      {
        robot_heading = headingBack();
        uturn_from_checkpoint = false;
        cp_exit_time = 0;
        robotState = ST_NORMAL;
        stateEnterTime = now;
        last_event_time = now;
      }
      break;
    }

    unsigned int pre_turn_ms = uturn_from_checkpoint ? CP_UTURN_ADVANCE_MS : UTURN_STOP_MS;
    unsigned int min_ms = uturn_from_checkpoint ? CP_UTURN_MIN_MS : UTURN_MIN_MS;
    unsigned int max_ms = uturn_from_checkpoint ? CP_UTURN_MAX_MS : 0;
    bool checkpoint_line = !cp_pad_seen && cp_black_count <= CP_EXIT_BLACK_MAX &&
                           (s1 || s2 || s3);
    bool line_ready = uturn_from_checkpoint ? checkpoint_line : centered_line;
    unsigned long turn_time = now - stateEnterTime;
    if (turn_time > (pre_turn_ms + min_ms) && line_ready)
    {
      robot_heading = headingBack(); // U-turn = 180 grade
      uturn_from_checkpoint = false;
      robotState = ST_NORMAL;
      stateEnterTime = now;
      last_event_time = now;
    }
    else if (uturn_from_checkpoint && max_ms > 0 &&
             turn_time > (pre_turn_ms + max_ms))
    {
      robot_heading = headingBack();
      uturn_from_checkpoint = false;
      robotState = cp_pad_seen ? ST_CP_CONFIRMED : ST_NORMAL;
      cp_exit_time = 0;
      stateEnterTime = now;
      last_event_time = now;
    }
    break;
  }

  case ST_JCT_ENTRY:
    if (s0 || s4)
      jct_saw_entry_corners = true;
    {
      bool exited_entry_blob = jct_saw_entry_corners && !s0 && !s4 && sensors_black <= 3;
      if (((now - stateEnterTime) >= JCT_ENTRY_MIN_MS && exited_entry_blob) ||
          ((now - stateEnterTime) >= JCT_ENTRY_MAX_MS))
      {
        // Captureaza daca exista cale inainte (linie centru, fara blob dens)
        ex_front = s2 && (sensors_black <= 2) && !dense_black;
        ex_front_blocked = infraredObstacleAhead();
        prepareJunctionChoice(now, false);
      }
    }
    break;

  case ST_JCT_STOP:
    if ((now - stateEnterTime) > JOY_MIN_HOLD_MS && joystickPressEdge())
    {
      logMove(jct_choice);
      robotState = (jct_target_dir == DIR_BACK) ? ST_UTURN : ST_JCT_EXECUTE;
      stateEnterTime = now;
      last_event_time = now;
      waitJoystickRelease();
    }
    break;
  case ST_JCT_EXECUTE:
    if (directionTurnsLeft(jct_target_dir) || directionTurnsRight(jct_target_dir))
    {
      unsigned long t = now - stateEnterTime;
      if (!jct_turn_aligned)
      {
        bool can_accept_line = t >= minAlignMsForDirection(jct_target_dir);
        unsigned int align_timeout = maxAlignMsForDirection(jct_target_dir);
        if (can_accept_line && turnAlignmentLineSeen())
        {
          if (jct_align_seen_since == 0)
            jct_align_seen_since = now;
          if ((now - jct_align_seen_since) >= TURN_ALIGN_CONFIRM_MS)
          {
            jct_turn_aligned = true;
            jct_commit_start = now;
            integral = 0;
            last_error = 0;
          }
        }
        else
        {
          jct_align_seen_since = 0;
        }

        if (!jct_turn_aligned && t >= align_timeout)
        {
          jct_turn_aligned = true;
          jct_commit_start = now;
          integral = 0;
          last_error = 0;
        }
      }
      else if ((now - jct_commit_start) >= COMMIT_DRIVE_MS)
      {
        updateHeadingAfterTurn(jct_target_dir);
        robotState = ST_NORMAL;
        stateEnterTime = now;
        last_event_time = now;
      }
    }
    else if ((now - stateEnterTime) > jct_execute_ms)
    {
      updateHeadingAfterTurn(jct_target_dir);
      robotState = ST_NORMAL;
      stateEnterTime = now;
      last_event_time = now;
    }
    break;

  case ST_DONE:
    setMotors(0, 0);
    break;
  }

  // ============ PID + ACTUARI ============
  if (robotState == ST_NORMAL && line_visible && abs(error) < INT_ACTIVE_ZONE && !dense_black)
  {
    integral += error;
    integral = constrain(integral, -INT_LIMIT, INT_LIMIT);
  }
  else
    integral = 0;

  int eff_error = error;
  if (abs(eff_error) < ERROR_DEADBAND)
    eff_error = 0;
  int derivative = eff_error - last_error;
  last_error = eff_error;
  int correction = eff_error / KP_DIV + derivative * KD_MULT + integral / KI_DIV;
  int base_speed = MAX_SPEED - (abs(error) * CURVE_SLOWDOWN) / 2000;
  base_speed = constrain(base_speed, MIN_SPEED, MAX_SPEED);

  int left_speed = 0, right_speed = 0;
  uint32_t led_color = 0x000000;

  switch (robotState)
  {
  case ST_NORMAL:
    if (cp_entry_seen || dense_black)
    {
      left_speed = right_speed = cp_entry_seen ? CP_APPROACH_SPEED : INTERSECTION_SPEED;
    }
    else if (error < -PIVOT_THRESHOLD && (s0 || s1))
    {
      left_speed = CIRCLE_INNER_SPEED;
      right_speed = CIRCLE_OUTER_SPEED;
      led_color = 0x00AAFF;
    }
    else if (error > PIVOT_THRESHOLD && (s4 || s3))
    {
      left_speed = CIRCLE_OUTER_SPEED;
      right_speed = CIRCLE_INNER_SPEED;
      led_color = 0x00AAFF;
    }
    else if (abs(error) > PIVOT_THRESHOLD)
    {
      int p = SHARP_PIVOT_SPEED;
      if (error > 0)
      {
        left_speed = p;
        right_speed = -p;
      }
      else
      {
        left_speed = -p;
        right_speed = p;
      }
    }
    else
    {
      left_speed = constrain(base_speed + correction, -MAX_SPEED, MAX_SPEED);
      right_speed = constrain(base_speed - correction, -MAX_SPEED, MAX_SPEED);
    }
    break;
  case ST_LOST:
  {
    int p = SHARP_PIVOT_SPEED;
    if (recent_side >= 0)
    {
      left_speed = p;
      right_speed = -p;
    }
    else
    {
      left_speed = -p;
      right_speed = p;
    }
    led_color = 0xFF0000;
    break;
  }
  case ST_CP_CANDIDATE:
    // continua drept incet ca sa ajunga stabil in patrat inainte de decizie
    left_speed = right_speed = CP_APPROACH_SPEED;
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
  case ST_UTURN:
  {
    unsigned long t = now - stateEnterTime;
    if (uturn_from_checkpoint && t < CP_UTURN_ADVANCE_MS)
    {
      left_speed = right_speed = CP_UTURN_ADVANCE_SPEED;
    }
    else
    {
      unsigned int stop_ms = uturn_from_checkpoint ? CP_UTURN_STOP_MS : UTURN_STOP_MS;
      unsigned long pivot_time = uturn_from_checkpoint ? (t - CP_UTURN_ADVANCE_MS) : t;
      if (pivot_time < stop_ms)
      {
        left_speed = 0;
        right_speed = 0;
      }
      else
      {
        int p = uturn_from_checkpoint ? CP_UTURN_SPEED : SHARP_PIVOT_SPEED;
        if (recent_side >= 0)
        {
          left_speed = p;
          right_speed = -p;
        }
        else
        {
          left_speed = -p;
          right_speed = p;
        }
      }
    }
    led_color = 0xFF00FF;
    break;
  }
  case ST_JCT_ENTRY:
    left_speed = right_speed = INTERSECTION_SPEED;
    led_color = 0x40FF40;
    break;
  case ST_JCT_STOP:
    left_speed = 0;
    right_speed = 0;
    led_color = 0x0000FF;
    break;
  case ST_JCT_EXECUTE:
  {
    if (directionTurnsLeft(jct_target_dir))
    {
      if (!jct_turn_aligned)
      {
        left_speed = -JCT_TURN_SPEED;
        right_speed = JCT_TURN_SPEED;
        led_color = 0x00FFFF;
      }
      else
      {
        int commit_correction = constrain(correction / 2, -COMMIT_CORR_LIMIT, COMMIT_CORR_LIMIT);
        left_speed = constrain(INTERSECTION_SPEED + commit_correction, 0, MAX_SPEED);
        right_speed = constrain(INTERSECTION_SPEED - commit_correction, 0, MAX_SPEED);
        led_color = 0xFFFFFF;
      }
    }
    else if (directionTurnsRight(jct_target_dir))
    {
      if (!jct_turn_aligned)
      {
        left_speed = JCT_TURN_SPEED;
        right_speed = -JCT_TURN_SPEED;
        led_color = 0xFFFF00;
      }
      else
      {
        int commit_correction = constrain(correction / 2, -COMMIT_CORR_LIMIT, COMMIT_CORR_LIMIT);
        left_speed = constrain(INTERSECTION_SPEED + commit_correction, 0, MAX_SPEED);
        right_speed = constrain(INTERSECTION_SPEED - commit_correction, 0, MAX_SPEED);
        led_color = 0xFFFFFF;
      }
    }
    else
    {
      left_speed = right_speed = INTERSECTION_SPEED;
      led_color = 0xFFFFFF;
    }
    break;
  }
  case ST_DONE:
    left_speed = 0;
    right_speed = 0;
    led_color = 0x00FF00; // verde = terminat
    break;
  }

  setMotors(left_speed, right_speed);

  if (robotState == ST_NORMAL)
  {
    if ((now - last_led_time) > 200)
    {
      last_led_time = now;
      for (i_led = 0; i_led < RGB.numPixels(); i_led++)
      {
        RGB.setPixelColor(i_led, Wheel(((i_led * 256 / RGB.numPixels()) + j_led) & 255));
      }
      RGB.show();
      if (j_led++ > 256 * 4)
        j_led = 0;
    }
  }
  else
  {
    for (int k = 0; k < 4; k++)
      RGB.setPixelColor(k, led_color);
    RGB.show();
  }

  if ((now - last_display) > 200)
  {
    showStatus();
    last_display = now;
  }
}

uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85)
    return RGB.Color(WheelPos * 50, 255 - WheelPos * 50, 0);
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return RGB.Color(255 - WheelPos * 50, 0, WheelPos * 50);
  }
  WheelPos -= 170;
  return RGB.Color(0, WheelPos * 50, 255 - WheelPos * 50);
}
void PCF8574Write(byte data)
{
  Wire.beginTransmission(Addr);
  Wire.write(data);
  Wire.endTransmission();
}
byte PCF8574Read()
{
  int data = -1;
  Wire.requestFrom(Addr, 1);
  if (Wire.available())
    data = Wire.read();
  return data;
}
