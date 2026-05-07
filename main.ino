/*
  AlphaBot2-Ar PathRunner v3
  ----------------------------------------------------------
  Baza hardware: sketch-ul existent al utilizatorului
   (TRSensors, OLED SSD1306, NeoPixel, PCF8574 joystick + IR,
    motor driver standard AlphaBot2-Ar).

  NAVIGATIE SEGMENTE CU REROUTING:
   - Traseul e impartit in segmente: start→CP1, CP1→CP2, ...
   - Fiecare segment are DOUA rute: A (principala) si B (backup).
   - Robotul urmeaza ruta A. Daca intalneste un obstacol (IR):
       1. Se opreste si face U-turn pe loc.
       2. Se intoarce pe ruta inversa pana la checkpoint-ul safe anterior.
       3. Porneste pe ruta B spre urmatorul checkpoint.
   - Daca obstacol si pe ruta B → ST_ERROR (oprire cu LED rosu).
   - Detectie CP: patrat negru plin (4/5 senzori >= CP_FULL_MS).
   - Detectie junction multi-modala: T, Y, cross, curbe, tangente.

  Conventie directii:
    0=spate  2=stanga  4=fata  6=dreapta
    1=spate-stanga  3=fata-stanga  5=fata-dreapta  7=spate-dreapta
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
// RUTE — editeaza DOAR aceasta sectiune
// Linii perechi: A (principala), B (backup), A, B, A, B...
// Fiecare pereche = un segment (start→CP1, CP1→CP2, etc.)
// 0 = terminator de ruta (nu il folosi ca directie in mijloc!)
// Directii: 2=stanga  4=fata  6=dreapta
//            1=sp-st   3=fa-st  5=fa-dr  7=sp-dr
// Max 15 directii per ruta.
// ====================================================
// <AI:routes>
#define MAX_ROUTE_TURNS 16
const int8_t ROUTES[][MAX_ROUTE_TURNS] = {
  {2, 4, 0},          // seg 1 ruta A: start → CP1
  {4, 2, 2, 6, 0},    // seg 1 ruta B: start → CP1 backup
  {4, 2, 4, 0},       // seg 2 ruta A: CP1 → CP2
  {2, 6, 2, 0},       // seg 2 ruta B: CP1 → CP2 backup
  {2, 0},             // seg 3 ruta A: CP2 → CP3
  {2, 0},             // seg 3 ruta B: CP2 → CP3 backup
};
// </AI:routes>
#define ARR_LEN(a) ((int)(sizeof(a)/sizeof(a[0])))
const int NUM_ROUTES   = ARR_LEN(ROUTES); // trebuie sa fie par
const int NUM_SEGMENTS = NUM_ROUTES / 2;

// ====================================================
// Stare navigatie
// ====================================================
#define NAV_PRIMARY 0
#define NAV_RETURN  1
#define NAV_BACKUP  2
#define NAV_DONE    3
#define NAV_ERROR   4

uint8_t  nav_mode        = NAV_PRIMARY;
uint8_t  nav_segment     = 0;
const int8_t* nav_route  = nullptr;
int      nav_route_len   = 0;
int      nav_route_idx   = 0;

// Viraje completate in leg-ul curent (pt ruta de intoarcere)
int8_t   nav_done_turns[MAX_ROUTE_TURNS];
uint8_t  nav_done_count  = 0;

// Ruta de intoarcere (construita dinamic)
int8_t   nav_return_buf[MAX_ROUTE_TURNS];
uint8_t  nav_return_len  = 0;

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
// moves_seq_idx eliminat — navigatia se face prin nav_route_idx
char move_log[MOVE_LOG_SIZE + 1] = {0};
int move_log_count = 0;
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
  ST_DONE,   // toate segmentele complete
  ST_ERROR   // obstacol si pe ruta B
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
bool jct_consumes_plan = false;
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
int  navGetNextTurn();
bool navHasMoreTurns();
void navRecordTurn(int dir);
void navActivatePrimary(uint8_t seg);
void navActivateBackup(uint8_t seg);
void navActivateReturn();
void navOnObstacle(unsigned long now);
void navOnCheckpointReached(unsigned long now);
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
  jct_consumes_plan = false;
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

// ============ Navigatie segmente ============

static int mirrorTurn(int t)
{
  switch (t)
  {
  case DIR_LEFT:        return DIR_RIGHT;
  case DIR_RIGHT:       return DIR_LEFT;
  case DIR_FRONT_LEFT:  return DIR_FRONT_RIGHT;
  case DIR_FRONT_RIGHT: return DIR_FRONT_LEFT;
  case DIR_BACK_LEFT:   return DIR_BACK_RIGHT;
  case DIR_BACK_RIGHT:  return DIR_BACK_LEFT;
  default:              return t; // FRONT si BACK raman
  }
}

// Calculeaza lungimea rutei (pana la primul 0 terminator).
static int routeLen(const int8_t* r)
{
  int n = 0;
  while (n < MAX_ROUTE_TURNS && r[n] != 0)
    n++;
  return n;
}

void navActivatePrimary(uint8_t seg)
{
  nav_route      = ROUTES[seg * 2];
  nav_route_len  = routeLen(nav_route);
  nav_route_idx  = 0;
  nav_done_count = 0;
  nav_mode = NAV_PRIMARY;
}

void navActivateBackup(uint8_t seg)
{
  nav_route      = ROUTES[seg * 2 + 1];
  nav_route_len  = routeLen(nav_route);
  nav_route_idx  = 0;
  nav_done_count = 0;
  nav_mode = NAV_BACKUP;
}

// Construieste ruta de intoarcere si o activeaza.
void navActivateReturn()
{
  nav_return_len = nav_done_count;
  for (uint8_t i = 0; i < nav_done_count; i++)
    nav_return_buf[i] = (int8_t)mirrorTurn(nav_done_turns[nav_done_count - 1 - i]);
  nav_route     = nullptr; // se va folosi nav_return_buf direct
  nav_route_len = nav_return_len;
  nav_route_idx = 0;
  nav_mode = NAV_RETURN;
}

// Citeste urmatoarea directie din ruta activa.
int navGetNextTurn()
{
  if (nav_route_idx >= nav_route_len)
    return DIR_FRONT;
  if (nav_mode == NAV_RETURN)
    return (int)nav_return_buf[nav_route_idx];
  if (nav_route != nullptr)
    return nav_route[nav_route_idx];
  return DIR_FRONT;
}

bool navHasMoreTurns()
{
  return nav_route_idx < nav_route_len;
}

// Inregistreaza virajul executat (pentru a putea calcula intoarcerea).
void navRecordTurn(int dir)
{
  if ((nav_mode == NAV_PRIMARY || nav_mode == NAV_BACKUP) &&
      nav_done_count < MAX_ROUTE_TURNS)
    nav_done_turns[nav_done_count++] = (int8_t)dir;
}

// Apelat cand senzorul IR detecteaza un obstacol.
void navOnObstacle(unsigned long now)
{
  if (nav_mode == NAV_BACKUP || nav_mode == NAV_ERROR || nav_mode == NAV_DONE)
  {
    // Obstacol si pe ruta B sau stare finala → eroare
    nav_mode = NAV_ERROR;
    setMotors(0, 0);
    robotState = ST_ERROR;
    stateEnterTime = now;
    return;
  }
  // Pe ruta A (primary) → construieste ruta de intoarcere si fa U-turn
  navActivateReturn();
  setMotors(0, 0);
  delay(OBSTACLE_STOP_MS);
  logMove('!');
  uturn_from_checkpoint = false;
  last_event_time = millis();
  robotState = ST_UTURN;
  stateEnterTime = millis();
  obstacle_seen_since = 0;
}

// Apelat la fiecare checkpoint detectat.
void navOnCheckpointReached(unsigned long now)
{
  if (nav_mode == NAV_RETURN)
  {
    // Revenit la checkpoint-ul safe → activeaza ruta B
    navActivateBackup(nav_segment);
    return; // robotul continua drept (ST_CP_CONFIRMED)
  }
  // Segment complet (primar sau backup) → trece la urmatorul
  nav_segment++;
  if (nav_segment >= (uint8_t)NUM_SEGMENTS)
  {
    nav_mode = NAV_DONE;
    robotState = ST_DONE;
    stateEnterTime = now;
  }
  else
  {
    navActivatePrimary(nav_segment);
  }
}

void prepareJunctionChoice(unsigned long now, bool from_checkpoint)
{
  // Daca ruta de intoarcere s-a epuizat la o intersectie (start fara CP)
  if (nav_mode == NAV_RETURN && !navHasMoreTurns())
    navActivateBackup(nav_segment);

  int planned_dir = navGetNextTurn();
  jct_consumes_plan = navHasMoreTurns();
  if (!validDirection(planned_dir))
    planned_dir = DIR_FRONT;

  // Inregistreaza virajul in leg-ul curent (pt ruta de intoarcere)
  if (jct_consumes_plan)
    navRecordTurn(planned_dir);
  nav_route_idx++;

  jct_target_dir = planned_dir;
  jct_choice = directionLogChar(planned_dir);
  uturn_from_checkpoint = from_checkpoint && planned_dir == DIR_BACK;

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
  // Notifica sistemul de navigatie
  navOnCheckpointReached(now);

  // Daca navOnCheckpointReached a schimbat starea in ST_DONE, oprim
  if (robotState == ST_DONE)
    return;

  // La checkpoint mergem intotdeauna inainte (nu luam turns din ruta pentru CP)
  int planned_dir = DIR_FRONT;

  jct_target_dir = planned_dir;
  jct_choice = directionLogChar(planned_dir);
  jct_consumes_plan = false;
  jct_execute_ms = 0;
  jct_turn_aligned = false;
  jct_align_seen_since = 0;
  jct_commit_start = 0;
  uturn_from_checkpoint = false;

  logMove('C');
  cp_exit_time = 0;
  last_event_time = now;
  stateEnterTime = now;
  robotState = ST_CP_CONFIRMED;
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
  case ST_ERROR:
    display.print(F("ERR"));
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
    display.print(F(" Log:"));
    display.print(jct_choice);
    display.setCursor(0, 24);
    display.print(F("0B 1BL 2L 3FL"));
    display.setCursor(0, 36);
    display.print(F("4F 5FR 6R 7BR"));
    display.setCursor(0, 36);
    static const char* mnames[] = {"A","RET","B","DONE","ERR"};
    display.print(F("Ruta:"));
    display.print(mnames[nav_mode > 4 ? 4 : nav_mode]);
    display.print(F(" Seg:"));
    display.print(nav_segment + 1);
    display.print(F("/"));
    display.print(NUM_SEGMENTS);
    display.setCursor(0, 48);
    display.print(F("Idx:"));
    display.print(nav_route_idx);
    display.print(F("/"));
    display.print(nav_route_len);
    display.print(F(" D:"));
    display.print(jct_choice);
    display.setCursor(0, 56);
    if (robotState == ST_JCT_STOP)
      display.print(F("Press JOY"));
    else
      display.print(F("Dir:"));
      display.print(jct_target_dir);
  }
  else
  {
    display.setTextSize(3);
    display.setCursor(82, 2);
    display.print(checkpoint_count);
    display.setTextSize(1);
    display.setCursor(0, 32);
    static const char* mn[] = {"RutaA","Ret","RutaB","DONE","ERR!"};
    display.print(mn[nav_mode > 4 ? 4 : nav_mode]);
    display.print(F(" Seg"));
    display.print(nav_segment + 1);
    display.print(F("/"));
    display.print(NUM_SEGMENTS);
    display.setCursor(0, 42);
    display.print(F("M:"));
    display.print(move_log);
    display.setCursor(0, 56);
    display.print(F("Idx:"));
    display.print(nav_route_idx);
    display.print(F("/"));
    display.print(nav_route_len);
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

  // Initializare navigatie segmente
  nav_segment    = 0;
  nav_route_idx  = 0;
  nav_done_count = 0;
  nav_return_len = 0;
  navActivatePrimary(0); // porneste pe ruta A a primului segment

  resetJunctionRefs();
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

  // Gating intre 2 intersectii (anti-rebound)
#if ENABLE_ODOMETRY
  bool jct_gap_ok = (odo_dist_mm - odo_dist_at_last_jct) >= (float)JCT_MIN_DIST_MM;
#else
  bool jct_gap_ok = (now - last_event_time) > JCT_COOLDOWN_MS;
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
      navOnObstacle(now); // gestioneaza intoarcerea sau eroarea
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
      resetJunctionRefs();
      resetArmTimers();
      // Daca trigger-ul includea senzorii exteriori (s0/s4), colturile au
      // fost deja vazute inainte de ST_JCT_ENTRY; le marcam direct ca sa
      // permitem exitul din blob imediat dupa JCT_ENTRY_MIN_MS.
      if (arm_left_side || arm_edge_l || arm_shallow_l ||
          arm_right_side || arm_edge_r || arm_shallow_r ||
          arm_dense || arm_widen)
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
      navOnObstacle(now);
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
      // CP confirmat. Checkpoint-ul = limita de segment.
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
        // Colturile au fost vazute inainte de CP_CANDIDATE (robotul era pe un blob dens)
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
      uturn_from_checkpoint = false;
      robotState = ST_NORMAL;
      stateEnterTime = now;
      last_event_time = now;
    }
    else if (uturn_from_checkpoint && max_ms > 0 &&
             turn_time > (pre_turn_ms + max_ms))
    {
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
        robotState = ST_NORMAL;
        stateEnterTime = now;
        last_event_time = now;
      }
    }
    else if ((now - stateEnterTime) > jct_execute_ms)
    {
      robotState = ST_NORMAL;
      stateEnterTime = now;
      last_event_time = now;
    }
    break;

  case ST_DONE:
  case ST_ERROR:
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
    left_speed = right_speed = 0;
    led_color = 0x00FF00; // verde = terminat
    break;
  case ST_ERROR:
    left_speed = right_speed = 0;
    led_color = 0xFF0000; // rosu = eroare
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
