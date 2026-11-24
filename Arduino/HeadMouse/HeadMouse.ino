/*
  HeadMouse MG24 - firmware v2
  Placa: Seeed Studio XIAO MG24 Sense (IMU LSM6DS3TR-C, I2C 0x6A)
  Herramientas > Protocol stack: "BLE (Silabs)"
  Libreria: Seeed Arduino LSM6DS3 >= 2.0.4

  Cambios principales respecto a v1:
  - Mapeo de ejes libre (cualquier eje del giroscopio, con signo) para
    horizontal y vertical, detectado con calibracion guiada (CAL=H / CAL=V).
    Asi funciona sin importar en que varilla o en que posicion vaya la placa.
  - Zona muerta "suave": no hay salto al salir de ella.
  - Acumulador sub-pixel: los movimientos lentos y pequenos ya no se pierden.
  - Ganancia en pixeles por grado (independiente de la frecuencia del ciclo).
  - Correccion automatica de deriva cuando la cabeza esta quieta.
  - Calibracion de offset al encender que espera a que la placa este quieta.
  - El emparejamiento BLE se conserva entre reconexiones (v1 lo borraba).
  - Lectura serial no bloqueante.

  v2.1:
  - Aceleracion: los giros rapidos avanzan mas pixeles por grado que los
    lentos. Asi se puede "remar": volver lento hacia un lado (el cursor casi
    no se mueve) y girar rapido hacia el otro (el cursor llega lejos).
  - Reposo automatico: tras N segundos sin mover el cursor, se congela.
  - Gesto para despertar: N giros bruscos (sobre un umbral de grados/s)
    dentro de 1,5 s, por ejemplo un "no" rapido con la cabeza.

  Protocolo serial (115200, lineas terminadas en \n):
    Entrada:  GET | DZ=<dps> | SM=<0..0.95> | GH=<px/grado> | GV=<px/grado>
              PA=<0|1> | RC | CAL=H | CAL=V | INV=H | INV=V
              AX=<ejeH>,<signoH>,<ejeV>,<signoV>   (eje 0=gx 1=gy 2=gz, signo 1|-1)
              RAW=<0|1> | RST
              AC=<0..3>      fuerza de la aceleracion (0 = sin aceleracion)
              AT=<dps>       velocidad desde la que empieza a acelerar
              IT=<s>         segundos sin uso para entrar en reposo (0 = nunca)
              WT=<dps>       umbral de giro brusco para despertar
              WN=<1..4>      cantidad de giros bruscos para despertar
              SL=<0|1>       forzar reposo / despertar desde el panel
    Salida:   VER:<v>
              CFG:dz,sm,gh,gv,ejeH,signoH,ejeV,signoV,pausa,calibrado,
                  ac,at,it,wt,wn,reposo
              R:<h>,<v>,<mult>     velocidad mapeada en grados/s y multiplicador
              ST:SLEEP,<0|1>       entra / sale del reposo
              G:<gx>,<gy>,<gz>     giroscopio crudo sin offset (si RAW=1)
              ST:BLE,<0|1|2>       0 sin conexion, 1 conectado, 2 seguro
              CAL:<H|V>:GO | CAL:<H|V>:OK,<eje>,<signo> | CAL:<H|V>:ERR,<motivo>
              OFF:<ox>,<oy>,<oz>[,inestable]
              ERR:<texto>
*/

#include <LSM6DS3.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

#define DEVICE_NAME "HeadMouse-MG24"
#define FW_VERSION  "2.1"

#ifndef LED_BUILTIN_ACTIVE
#define LED_BUILTIN_ACTIVE   LOW
#define LED_BUILTIN_INACTIVE HIGH
#endif

#ifndef ARDUINO_SILABS_STACK_BLE_SILABS
#error "Selecciona 'BLE (Silabs)' en Herramientas > Protocol stack."
#endif

// ---------------------------------------------------------------- Config
const uint32_t CONFIG_MAGIC = 0x484D3244; // "HM2D": v2.1 (aceleracion y reposo)

struct ConfigData {
  uint32_t magic;
  float deadzone;    // grados/s
  float smoothing;   // 0 .. 0.95
  float gainH;       // pixeles por grado de giro
  float gainV;
  uint8_t axisH;     // 0=gx 1=gy 2=gz
  int8_t  signH;     // +1 => cursor a la derecha
  uint8_t axisV;
  int8_t  signV;     // +1 => cursor hacia abajo
  uint8_t calibrated; // bit0 = H calibrado, bit1 = V calibrado
  float accel;        // 0..3, 0 = sin aceleracion
  float accelThr;     // grados/s desde donde acelera
  uint16_t idleSec;   // 0 = sin reposo automatico
  float wakeThr;      // grados/s de un giro brusco
  uint8_t wakeCount;  // giros bruscos para despertar
};
ConfigData cfg;

void setDefaults() {
  cfg.magic = CONFIG_MAGIC;
  cfg.deadzone = 2.5f;
  cfg.smoothing = 0.6f;
  cfg.gainH = 40.0f;
  cfg.gainV = 40.0f;
  // Medido con la placa en la varilla (USB-C hacia la oreja, cara de chips afuera):
  //   girar a la izquierda -> gy negativo  => horizontal = gy, signo +1
  //   mirar arriba         -> gz positivo  => vertical   = gz, signo -1 (abajo = +)
  //   oreja al hombro der. -> gx negativo  (alabeo, no se usa)
  // Si cambia el montaje, la calibracion guiada (CAL=H / CAL=V) lo corrige.
  cfg.axisH = 1; cfg.signH = 1;
  cfg.axisV = 2; cfg.signV = -1;
  cfg.calibrated = 0;
  cfg.accel = 1.0f;     // a 80 °/s el cursor avanza el doble por grado
  cfg.accelThr = 40.0f;
  cfg.idleSec = 30;     // reposo activado por defecto
  cfg.wakeThr = 120.0f;
  cfg.wakeCount = 2;    // "no" rapido: izquierda-derecha
}

bool configValid() {
  if (cfg.magic != CONFIG_MAGIC) return false;
  if (isnan(cfg.deadzone) || cfg.deadzone < 0 || cfg.deadzone > 20) return false;
  if (isnan(cfg.smoothing) || cfg.smoothing < 0 || cfg.smoothing > 0.95f) return false;
  if (isnan(cfg.gainH) || cfg.gainH < 1 || cfg.gainH > 300) return false;
  if (isnan(cfg.gainV) || cfg.gainV < 1 || cfg.gainV > 300) return false;
  if (cfg.axisH > 2 || cfg.axisV > 2) return false;
  if ((cfg.signH != 1 && cfg.signH != -1) || (cfg.signV != 1 && cfg.signV != -1)) return false;
  if (isnan(cfg.accel) || cfg.accel < 0 || cfg.accel > 3) return false;
  if (isnan(cfg.accelThr) || cfg.accelThr < 5 || cfg.accelThr > 200) return false;
  if (cfg.idleSec > 600) return false;
  if (isnan(cfg.wakeThr) || cfg.wakeThr < 30 || cfg.wakeThr > 500) return false;
  if (cfg.wakeCount < 1 || cfg.wakeCount > 4) return false;
  return true;
}

void loadConfig() {
  EEPROM.begin();
  EEPROM.get(0, cfg);
  if (!configValid()) {
    setDefaults();
    EEPROM.put(0, cfg);
  }
}

bool pendingSave = false;
unsigned long lastChangeTime = 0;
void markDirty() { pendingSave = true; lastChangeTime = millis(); }

// ---------------------------------------------------------------- Estado
LSM6DS3 imu(I2C_MODE, 0x6A);
bool imuOk = false;

const uint32_t TICK_MS = 10;          // 100 Hz
const float    STILL_DPS = 3.0f;      // umbral de "cabeza quieta" para deriva
const uint16_t STILL_TICKS = 100;     // 1 s quieto antes de corregir deriva
const float    DRIFT_ALPHA = 0.01f;   // velocidad de correccion de deriva

float off[3] = { 0, 0, 0 };
float g[3]   = { 0, 0, 0 };
float filtH = 0, filtV = 0;
float accH = 0, accV = 0;             // acumuladores sub-pixel
uint16_t stillCount = 0;
uint8_t isPaused = 0;
bool rawStream = false;
float lastMult = 1.0f;

// Reposo y gesto para despertar
const uint16_t WAKE_WINDOW_MS = 1500;   // los giros bruscos deben caer en esta ventana
const uint16_t WAKE_COOLDOWN_MS = 700;  // tras despertar, el cursor espera quieto
uint8_t sleeping = 0, sleepSent = 255;
unsigned long lastActivity = 0;
unsigned long wakeFirst = 0, cooldownUntil = 0;
uint8_t wakeHits = 0;
bool wakeArmed = true;

unsigned long lastTick = 0;
unsigned long lastTelemetry = 0;

// Calibracion guiada
const uint16_t CAL_N = 300;           // 3 s a 100 Hz
const float    CAL_MIN_PEAK = 20.0f;  // grados/s minimos para aceptar
static float calBuf[CAL_N][3];
uint16_t calIdx = 0;
char calTarget = 0;                   // 0 = inactivo, 'H' o 'V'

// BLE
struct mouse_data { int8_t delta_x; int8_t delta_y; uint8_t buttons; };
static mouse_data report;
static uint8_t report_array[3] = { 0, 0, 0 };
static uint8_t connection_handle = SL_BT_INVALID_CONNECTION_HANDLE;
static uint32_t bonding_handle = SL_BT_INVALID_BONDING_HANDLE;
static uint16_t hid_report;
static bd_addr ble_address = { { 0, 0, 0, 0, 0, 0 } };
static uint8_t bleState = 0, bleStateSent = 255;

static void ble_initialize_gatt_db();
static void ble_start_advertising();

// ---------------------------------------------------------------- Utilidades
void led(bool on) { digitalWrite(LED_BUILTIN, on ? LED_BUILTIN_ACTIVE : LED_BUILTIN_INACTIVE); }

float softDeadzone(float x, float dz) {
  if (x > dz) return x - dz;
  if (x < -dz) return x + dz;
  return 0.0f;
}

void readGyro() {
  g[0] = imu.readFloatGyroX() - off[0];
  g[1] = imu.readFloatGyroY() - off[1];
  g[2] = imu.readFloatGyroZ() - off[2];
}

void resetMotion() {
  filtH = filtV = 0;
  accH = accV = 0;
}

void sendConfig() {
  Serial.print("CFG:");
  Serial.print(cfg.deadzone, 1);  Serial.print(',');
  Serial.print(cfg.smoothing, 2); Serial.print(',');
  Serial.print(cfg.gainH, 0);     Serial.print(',');
  Serial.print(cfg.gainV, 0);     Serial.print(',');
  Serial.print(cfg.axisH);        Serial.print(',');
  Serial.print(cfg.signH);        Serial.print(',');
  Serial.print(cfg.axisV);        Serial.print(',');
  Serial.print(cfg.signV);        Serial.print(',');
  Serial.print(isPaused);         Serial.print(',');
  Serial.print(cfg.calibrated);   Serial.print(',');
  Serial.print(cfg.accel, 2);     Serial.print(',');
  Serial.print(cfg.accelThr, 0);  Serial.print(',');
  Serial.print(cfg.idleSec);      Serial.print(',');
  Serial.print(cfg.wakeThr, 0);   Serial.print(',');
  Serial.print(cfg.wakeCount);    Serial.print(',');
  Serial.println(sleeping);
}

void setSleep(uint8_t s) {
  sleeping = s;
  resetMotion();
  wakeHits = 0; wakeArmed = true;
  lastActivity = millis();
  if (!s) cooldownUntil = millis() + WAKE_COOLDOWN_MS;
}

// Cuenta giros bruscos. Un giro cuenta al superar el umbral y se "rearma"
// cuando la velocidad baja del 40 % del umbral.
bool wakeGesture(float rateH, float rateV) {
  unsigned long now = millis();
  float peak = fmaxf(fabsf(rateH), fabsf(rateV));
  if (wakeHits && now - wakeFirst > WAKE_WINDOW_MS) wakeHits = 0;
  if (wakeArmed && peak > cfg.wakeThr) {
    wakeArmed = false;
    if (wakeHits == 0) wakeFirst = now;
    wakeHits++;
    if (wakeHits >= cfg.wakeCount) return true;
  } else if (!wakeArmed && peak < 0.4f * cfg.wakeThr) {
    wakeArmed = true;
  }
  return false;
}

// Offset del giroscopio: espera a que la placa este quieta (max ~3 s)
void calibrateOffset() {
  const int n = 100;
  float mean[3] = { 0, 0, 0 };
  bool stable = false;

  for (int attempt = 0; attempt < 6 && !stable; attempt++) {
    float sum[3] = { 0, 0, 0 };
    float mn[3] = { 1e9, 1e9, 1e9 }, mx[3] = { -1e9, -1e9, -1e9 };
    for (int i = 0; i < n; i++) {
      float r[3] = { imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ() };
      for (int k = 0; k < 3; k++) {
        sum[k] += r[k];
        if (r[k] < mn[k]) mn[k] = r[k];
        if (r[k] > mx[k]) mx[k] = r[k];
      }
      delay(5);
    }
    stable = true;
    for (int k = 0; k < 3; k++) {
      mean[k] = sum[k] / n;
      if (mx[k] - mn[k] > 4.0f) stable = false;
    }
    led(attempt % 2 == 0);
  }
  for (int k = 0; k < 3; k++) off[k] = mean[k];
  resetMotion();
  stillCount = 0;

  Serial.print("OFF:");
  Serial.print(off[0], 2); Serial.print(',');
  Serial.print(off[1], 2); Serial.print(',');
  Serial.print(off[2], 2);
  Serial.println(stable ? "" : ",inestable");
  led(bleState == 2);
}

// ---------------------------------------------------------------- Calibracion guiada
void startGuidedCal(char target) {
  calTarget = target;
  calIdx = 0;
  resetMotion();
  Serial.print("CAL:"); Serial.print(target); Serial.println(":GO");
}

void calError(const char* why) {
  Serial.print("CAL:"); Serial.print(calTarget); Serial.print(":ERR,"); Serial.println(why);
  calTarget = 0;
}

void finishGuidedCal() {
  float energy[3] = { 0, 0, 0 };
  for (uint16_t i = 0; i < calIdx; i++)
    for (int k = 0; k < 3; k++) energy[k] += fabsf(calBuf[i][k]);

  int best = 0;
  for (int k = 1; k < 3; k++) if (energy[k] > energy[best]) best = k;
  int second = (best == 0) ? 1 : 0;
  for (int k = 0; k < 3; k++) if (k != best && energy[k] > energy[second]) second = k;

  float peak = 0;
  for (uint16_t i = 0; i < calIdx; i++) peak = fmaxf(peak, fabsf(calBuf[i][best]));

  if (peak < CAL_MIN_PEAK) { calError("poco movimiento"); return; }
  if (energy[best] < 1.3f * energy[second]) { calError("movimiento mezclado"); return; }

  // Signo = direccion del primer movimiento claro (ida), no del regreso
  int8_t sign = 1;
  for (uint16_t i = 0; i < calIdx; i++) {
    if (fabsf(calBuf[i][best]) > 0.5f * peak) { sign = calBuf[i][best] > 0 ? 1 : -1; break; }
  }

  // H: la persona gira hacia la DERECHA  -> cursor a la derecha (+x)
  // V: la persona mira hacia ABAJO       -> cursor hacia abajo  (+y)
  if (calTarget == 'H') {
    if ((cfg.calibrated & 2) && best == cfg.axisV) { calError("mismo eje que vertical"); return; }
    cfg.axisH = best; cfg.signH = sign; cfg.calibrated |= 1;
  } else {
    if ((cfg.calibrated & 1) && best == cfg.axisH) { calError("mismo eje que horizontal"); return; }
    cfg.axisV = best; cfg.signV = sign; cfg.calibrated |= 2;
  }

  Serial.print("CAL:"); Serial.print(calTarget); Serial.print(":OK,");
  Serial.print(best); Serial.print(','); Serial.println(sign);
  calTarget = 0;
  markDirty();
  sendConfig();
}

// ---------------------------------------------------------------- Comandos serie
void handleCommand(char* c) {
  if (!strcmp(c, "GET")) {
    Serial.println("VER:" FW_VERSION);
    sendConfig();
    bleStateSent = 255;
  } else if (!strncmp(c, "DZ=", 3)) {
    cfg.deadzone = constrain(atof(c + 3), 0.0f, 20.0f); markDirty();
  } else if (!strncmp(c, "SM=", 3)) {
    cfg.smoothing = constrain(atof(c + 3), 0.0f, 0.95f); markDirty();
  } else if (!strncmp(c, "GH=", 3)) {
    cfg.gainH = constrain(atof(c + 3), 1.0f, 300.0f); markDirty();
  } else if (!strncmp(c, "GV=", 3)) {
    cfg.gainV = constrain(atof(c + 3), 1.0f, 300.0f); markDirty();
  } else if (!strncmp(c, "PA=", 3)) {
    isPaused = atoi(c + 3) ? 1 : 0; resetMotion(); sendConfig();
  } else if (!strcmp(c, "RC")) {
    calibrateOffset();
  } else if (!strcmp(c, "CAL=H")) {
    startGuidedCal('H');
  } else if (!strcmp(c, "CAL=V")) {
    startGuidedCal('V');
  } else if (!strcmp(c, "INV=H")) {
    cfg.signH = -cfg.signH; markDirty(); sendConfig();
  } else if (!strcmp(c, "INV=V")) {
    cfg.signV = -cfg.signV; markDirty(); sendConfig();
  } else if (!strncmp(c, "AX=", 3)) {
    int ah, sh, av, sv;
    if (sscanf(c + 3, "%d,%d,%d,%d", &ah, &sh, &av, &sv) == 4 &&
        ah >= 0 && ah <= 2 && av >= 0 && av <= 2 && ah != av) {
      cfg.axisH = ah; cfg.signH = sh < 0 ? -1 : 1;
      cfg.axisV = av; cfg.signV = sv < 0 ? -1 : 1;
      markDirty(); sendConfig();
    } else {
      Serial.println("ERR:AX invalido");
    }
  } else if (!strncmp(c, "RAW=", 4)) {
    rawStream = atoi(c + 4) != 0;
  } else if (!strncmp(c, "AC=", 3)) {
    cfg.accel = constrain(atof(c + 3), 0.0f, 3.0f); markDirty();
  } else if (!strncmp(c, "AT=", 3)) {
    cfg.accelThr = constrain(atof(c + 3), 5.0f, 200.0f); markDirty();
  } else if (!strncmp(c, "IT=", 3)) {
    cfg.idleSec = constrain(atoi(c + 3), 0, 600); lastActivity = millis(); markDirty();
  } else if (!strncmp(c, "WT=", 3)) {
    cfg.wakeThr = constrain(atof(c + 3), 30.0f, 500.0f); markDirty();
  } else if (!strncmp(c, "WN=", 3)) {
    cfg.wakeCount = constrain(atoi(c + 3), 1, 4); markDirty();
  } else if (!strncmp(c, "SL=", 3)) {
    setSleep(atoi(c + 3) ? 1 : 0);
  } else if (!strcmp(c, "RST")) {
    setDefaults(); markDirty(); sendConfig();
  } else {
    Serial.print("ERR:comando desconocido "); Serial.println(c);
  }
}

void pollSerial() {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { buf[len] = 0; handleCommand(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
}

// ---------------------------------------------------------------- Movimiento
void sendMouse(int8_t dx, int8_t dy) {
  if (connection_handle == SL_BT_INVALID_CONNECTION_HANDLE ||
      bonding_handle == SL_BT_INVALID_BONDING_HANDLE) return;
  report.delta_x = dx;
  report.delta_y = dy;
  report.buttons = 0;
  memcpy(report_array, &report, sizeof(report));
  sl_bt_gatt_server_notify_all(hid_report, sizeof(report_array), report_array);
}

void processTick(float dt) {
  readGyro();

  // Correccion de deriva: si todo esta casi quieto durante 1 s, el offset
  // se acerca lentamente a la lectura actual.
  bool still = fabsf(g[0]) < STILL_DPS && fabsf(g[1]) < STILL_DPS && fabsf(g[2]) < STILL_DPS;
  stillCount = still ? (stillCount < 60000 ? stillCount + 1 : stillCount) : 0;
  if (stillCount > STILL_TICKS && calTarget == 0) {
    for (int k = 0; k < 3; k++) { off[k] += DRIFT_ALPHA * g[k]; g[k] -= DRIFT_ALPHA * g[k]; }
  }

  float rateH = g[cfg.axisH] * cfg.signH;   // grados/s, + = derecha
  float rateV = g[cfg.axisV] * cfg.signV;   // grados/s, + = abajo

  if (millis() - lastTelemetry >= 50) {
    lastTelemetry = millis();
    Serial.print("R:"); Serial.print(rateH, 1); Serial.print(','); Serial.print(rateV, 1);
    Serial.print(','); Serial.println(lastMult, 2);
    if (rawStream) {
      Serial.print("G:"); Serial.print(g[0], 1); Serial.print(',');
      Serial.print(g[1], 1); Serial.print(','); Serial.println(g[2], 1);
    }
  }

  if (calTarget) {
    if (calIdx < CAL_N) {
      for (int k = 0; k < 3; k++) calBuf[calIdx][k] = g[k];
      calIdx++;
    }
    if (calIdx >= CAL_N) finishGuidedCal();
    return;   // cursor quieto mientras se calibra
  }

  unsigned long now = millis();
  if (isPaused) { lastActivity = now; lastMult = 1.0f; return; }

  // Reposo: el cursor queda congelado hasta el gesto de despertar
  if (sleeping) {
    lastMult = 1.0f;
    if (wakeGesture(rateH, rateV)) setSleep(0);
    return;
  }
  if (cfg.idleSec && now - lastActivity > (unsigned long)cfg.idleSec * 1000UL) {
    setSleep(1);
    return;
  }
  // Justo despues de despertar: no mover (el gesto no debe arrastrar el cursor)
  if ((long)(cooldownUntil - now) > 0) { resetMotion(); lastActivity = now; return; }

  float h = softDeadzone(rateH, cfg.deadzone);
  float v = softDeadzone(rateV, cfg.deadzone);
  filtH = cfg.smoothing * filtH + (1.0f - cfg.smoothing) * h;
  filtV = cfg.smoothing * filtV + (1.0f - cfg.smoothing) * v;

  // Aceleracion: bajo el umbral la ganancia es la normal (precision);
  // sobre el umbral crece en linea recta, hasta 1 + 3*accel veces.
  float speed = sqrtf(filtH * filtH + filtV * filtV);
  float mult = 1.0f;
  if (cfg.accel > 0 && speed > cfg.accelThr)
    mult = 1.0f + cfg.accel * fminf((speed - cfg.accelThr) / cfg.accelThr, 3.0f);
  lastMult = mult;

  accH += filtH * dt * cfg.gainH * mult;
  accV += filtV * dt * cfg.gainV * mult;

  int dx = constrain((int)accH, -127, 127);   // trunca hacia 0; el resto queda acumulado
  int dy = constrain((int)accV, -127, 127);
  accH = constrain(accH - dx, -127.0f, 127.0f);
  accV = constrain(accV - dy, -127.0f, 127.0f);

  if (dx != 0 || dy != 0) {
    lastActivity = now;
    sendMouse((int8_t)dx, (int8_t)dy);
  }
}

// ---------------------------------------------------------------- Setup / loop
void setup() {
  memset(&report, 0, sizeof(report));
  pinMode(LED_BUILTIN, OUTPUT);
  led(false);

  loadConfig();

  pinMode(PD5, OUTPUT);       // alimentacion del IMU
  digitalWrite(PD5, HIGH);
  delay(300);

  Serial.begin(115200);
  imuOk = (imu.begin() == 0);
  if (!imuOk) Serial.println("ERR:IMU no responde");
  else calibrateOffset();

  Serial.println("VER:" FW_VERSION);
  sendConfig();
  lastTick = millis();
}

void loop() {
  pollSerial();

  if (pendingSave && millis() - lastChangeTime > 2000) {
    EEPROM.put(0, cfg);
    pendingSave = false;
  }

  if (bleState != bleStateSent) {
    bleStateSent = bleState;
    Serial.print("ST:BLE,"); Serial.println(bleState);
    led(bleState == 2);
  }

  if (sleeping != sleepSent) {
    sleepSent = sleeping;
    Serial.print("ST:SLEEP,"); Serial.println(sleeping);
    led(bleState == 2);
  }
  // En reposo el LED parpadea breve cada 2 s
  if (sleeping) led((millis() % 2000) < 80);

  unsigned long now = millis();
  if (imuOk && now - lastTick >= TICK_MS) {
    float dt = (now - lastTick) / 1000.0f;
    if (dt > 0.05f) dt = 0.05f;
    lastTick = now;
    processTick(dt);
  }
}

// ---------------------------------------------------------------- BLE
void sl_bt_on_event(sl_bt_msg_t* evt) {
  sl_status_t sc;
  uint8_t ble_address_type;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      sc = sl_bt_system_get_identity_address(&ble_address, &ble_address_type);
      app_assert_status(sc);
      ble_initialize_gatt_db();
      sc = sl_bt_sm_configure(0, sl_bt_sm_io_capability_noinputnooutput);
      app_assert_status(sc);
      sc = sl_bt_sm_set_bondable_mode(1);
      app_assert_status(sc);
      ble_start_advertising();
      break;

    case sl_bt_evt_connection_opened_id:
      connection_handle = evt->data.evt_connection_opened.connection;
      bonding_handle = evt->data.evt_connection_opened.bonding;
      bleState = 1;
      sl_bt_sm_increase_security(connection_handle);
      break;

    case sl_bt_evt_sm_bonded_id:
      connection_handle = evt->data.evt_sm_bonded.connection;
      bonding_handle = evt->data.evt_sm_bonded.bonding;
      break;

    case sl_bt_evt_connection_parameters_id:
      if (evt->data.evt_connection_parameters.security_mode != sl_bt_connection_mode1_level1) {
        bleState = 2;
        // Pide un intervalo corto (7.5-15 ms) para menos latencia; el host decide.
        sl_bt_connection_set_parameters(evt->data.evt_connection_parameters.connection,
                                        6, 12, 0, 200, 0, 0xFFFF);
      }
      break;

    case sl_bt_evt_connection_closed_id:
      connection_handle = SL_BT_INVALID_CONNECTION_HANDLE;
      bonding_handle = SL_BT_INVALID_BONDING_HANDLE;
      bleState = 0;
      // v2: NO se borra el emparejamiento; el PC se reconecta solo.
      ble_start_advertising();
      break;

    case sl_bt_evt_sm_bonding_failed_id:
      // Claves desincronizadas (p. ej. el PC recuerda un emparejamiento viejo):
      // se borran y se cierra para reintentar limpio.
      sl_bt_sm_delete_bondings();
      sl_bt_connection_close(evt->data.evt_sm_bonding_failed.connection);
      Serial.println("ERR:emparejamiento fallido; quita el dispositivo en el PC y vuelve a agregarlo");
      break;

    default:
      break;
  }
}

static void ble_start_advertising() {
  static uint8_t advertising_set_handle = 0xff;
  static bool init = true;
  sl_status_t sc;
  if (init) {
    sc = sl_bt_advertiser_create_set(&advertising_set_handle);
    app_assert_status(sc);
    sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, 160, 0, 0);
    app_assert_status(sc);
    init = false;
  }
  sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);
  app_assert_status(sc);
  sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
  app_assert_status(sc);
}

// GATT basado en el ejemplo oficial de Silicon Labs (BLE HID mouse), que
// incluye los servicios Battery y Device Information que pide HOGP.
static void ble_initialize_gatt_db() {
  sl_status_t sc;
  uint16_t session, service, characteristic, descriptor;

  sc = sl_bt_gattdb_new_session(&session);
  app_assert_status(sc);

  // Generic Access
  uint8_t ga_uuid[] = { 0x00, 0x18 };
  sc = sl_bt_gattdb_add_service(session, sl_bt_gattdb_primary_service, SL_BT_GATTDB_ADVERTISED_SERVICE, 2, ga_uuid, &service);
  app_assert_status(sc);
  sl_bt_uuid_16_t name_uuid = { .data = { 0x00, 0x2A } };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service, SL_BT_GATTDB_CHARACTERISTIC_READ, 0, 0, name_uuid,
                                              sl_bt_gattdb_fixed_length_value, strlen(DEVICE_NAME), strlen(DEVICE_NAME),
                                              (uint8_t*)DEVICE_NAME, &characteristic);
  app_assert_status(sc);
  sl_bt_uuid_16_t appearance_uuid = { .data = { 0x01, 0x2A } };
  const uint8_t appearance[] = { 0xC2, 0x03 };   // raton HID
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service, SL_BT_GATTDB_CHARACTERISTIC_READ, 0, 0, appearance_uuid,
                                              sl_bt_gattdb_fixed_length_value, 2, 2, appearance, &characteristic);
  app_assert_status(sc);
  sc = sl_bt_gattdb_start_service(session, service);
  app_assert_status(sc);

  // Battery
  const uint8_t bat_uuid[] = { 0x0F, 0x18 };
  sc = sl_bt_gattdb_add_service(session, sl_bt_gattdb_primary_service, 0, 2, bat_uuid, &service);
  app_assert_status(sc);
  const sl_bt_uuid_16_t bat_level_uuid = { .data = { 0x19, 0x2A } };
  const uint8_t bat_level = 100;
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service, SL_BT_GATTDB_CHARACTERISTIC_READ, 0, 0, bat_level_uuid,
                                              sl_bt_gattdb_fixed_length_value, 1, 1, &bat_level, &characteristic);
  app_assert_status(sc);
  sc = sl_bt_gattdb_start_service(session, service);
  app_assert_status(sc);

  // Device Information (PnP ID)
  const uint8_t dis_uuid[] = { 0x0A, 0x18 };
  sc = sl_bt_gattdb_add_service(session, sl_bt_gattdb_primary_service, 0, 2, dis_uuid, &service);
  app_assert_status(sc);
  const uint8_t manufacturer[] = "Seeed XIAO";
  const sl_bt_uuid_16_t manu_uuid = { .data = { 0x29, 0x2A } };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service, SL_BT_GATTDB_CHARACTERISTIC_READ, 0, 0, manu_uuid,
                                              sl_bt_gattdb_fixed_length_value, sizeof(manufacturer) - 1, sizeof(manufacturer) - 1,
                                              manufacturer, &characteristic);
  app_assert_status(sc);
  const sl_bt_uuid_16_t pnp_uuid = { .data = { 0x50, 0x2A } };
  const uint8_t pnp[] = { 0x02, 0x10, 0xC4, 0x00, 0x01, 0x00, 0x01 };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service, SL_BT_GATTDB_CHARACTERISTIC_READ, 0, 0, pnp_uuid,
                                              sl_bt_gattdb_fixed_length_value, sizeof(pnp), sizeof(pnp), pnp, &characteristic);
  app_assert_status(sc);
  sc = sl_bt_gattdb_start_service(session, service);
  app_assert_status(sc);

  // HID
  uint8_t hid_uuid[] = { 0x12, 0x18 };
  sc = sl_bt_gattdb_add_service(session, sl_bt_gattdb_primary_service, SL_BT_GATTDB_ADVERTISED_SERVICE, 2, hid_uuid, &service);
  app_assert_status(sc);

  sl_bt_uuid_16_t proto_uuid = { .data = { 0x4E, 0x2A } };
  const uint8_t proto_val[] = { 1 };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service,
                                              SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE,
                                              0, 0, proto_uuid, sl_bt_gattdb_fixed_length_value, 1, 1, proto_val, &characteristic);
  app_assert_status(sc);

  const sl_bt_uuid_16_t rep_uuid = { .data = { 0x4D, 0x2A } };
  const uint8_t rep_val[] = { 0, 0, 0 };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service,
                                              SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_WRITE | SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
                                              0, 0, rep_uuid, sl_bt_gattdb_fixed_length_value, 3, 3, rep_val, &characteristic);
  app_assert_status(sc);
  hid_report = characteristic;

  const sl_bt_uuid_16_t ref_uuid = { .data = { 0x08, 0x29 } };
  const uint8_t ref_val[] = { 0x00, 0x01 };   // report ID 0, tipo Input
  sc = sl_bt_gattdb_add_uuid16_descriptor(session, characteristic, SL_BT_GATTDB_DESCRIPTOR_READ, SL_BT_GATTDB_ENCRYPTED_READ,
                                          ref_uuid, sl_bt_gattdb_fixed_length_value, 2, 2, ref_val, &descriptor);
  app_assert_status(sc);

  const sl_bt_uuid_16_t map_uuid = { .data = { 0x4B, 0x2A } };
  const uint8_t map_val[] = {
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
    0x09, 0x30, 0x09, 0x31, 0x15, 0x80, 0x25, 0x7F, 0x95, 0x02,
    0x75, 0x08, 0x81, 0x06, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x05, 0x81, 0x03, 0xC0, 0xC0
  };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service, SL_BT_GATTDB_CHARACTERISTIC_READ, SL_BT_GATTDB_ENCRYPTED_READ, 0,
                                              map_uuid, sl_bt_gattdb_fixed_length_value, sizeof(map_val), sizeof(map_val),
                                              map_val, &characteristic);
  app_assert_status(sc);

  const sl_bt_uuid_16_t ext_ref_uuid = { .data = { 0x07, 0x29 } };
  const uint8_t ext_ref_val[] = { 0x00, 0x00 };
  sc = sl_bt_gattdb_add_uuid16_descriptor(session, characteristic, SL_BT_GATTDB_DESCRIPTOR_READ, 0, ext_ref_uuid,
                                          sl_bt_gattdb_fixed_length_value, 2, 2, ext_ref_val, &descriptor);
  app_assert_status(sc);

  const sl_bt_uuid_16_t info_uuid = { .data = { 0x4A, 0x2A } };
  const uint8_t info_val[] = { 0x01, 0x11, 0x00, 0x02 };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service, SL_BT_GATTDB_CHARACTERISTIC_READ, 0, 0, info_uuid,
                                              sl_bt_gattdb_fixed_length_value, 4, 4, info_val, &characteristic);
  app_assert_status(sc);

  const sl_bt_uuid_16_t cp_uuid = { .data = { 0x4C, 0x2A } };
  const uint8_t cp_val[] = { 0x00 };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service, SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE, 0, 0, cp_uuid,
                                              sl_bt_gattdb_fixed_length_value, 1, 1, cp_val, &characteristic);
  app_assert_status(sc);

  sc = sl_bt_gattdb_start_service(session, service);
  app_assert_status(sc);
  sc = sl_bt_gattdb_commit(session);
  app_assert_status(sc);
}
