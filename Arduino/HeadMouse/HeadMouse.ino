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

  v2.2:
  - Swift Pair: durante 60 s tras encender (o hasta que un PC se conecte) el
    anuncio BLE lleva el bloque de Microsoft que hace aparecer en Windows la
    notificacion "Nuevo dispositivo encontrado -> Conectar".
  - Teclado HID ademas del mouse: la primera vez que se empareja con un PC,
    abre el asistente del panel (Win+R, "msedge gbtox.github.io/airmouse/asistente").
  - Servicio BLE de configuracion: el panel puede configurar por Bluetooth
    (Web Bluetooth) con los mismos comandos que por USB.
  - Hasta 4 PC emparejados; al llenarse se reemplaza el de uso mas antiguo.

  v3.0:
  - Clic por sonido con el microfono de la placa (seccion "Sonido" al final del archivo). Sonido 1 =
    clic izquierdo; dos sonidos 1 dentro de DT ms = clic derecho. Sonido 2 =
    accion configurable (Av Pag, Re Pag, flechas, espacio, clic derecho,
    doble clic). Sin modelo cargado usa un detector de chasquido por reglas;
    con modelo (entrenado en el panel y enviado con MW=) reconoce el sonido
    propio de la persona.
  - Clic quieto: antes del clic se deshace el movimiento de los ultimos
    ~400 ms, para que el clic caiga donde estaba el cursor al empezar el sonido.

  Protocolo (USB serie a 115200, o BLE: caracteristica CMD para escribir y
  OUT para recibir notificaciones; lineas terminadas en \n):
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
              PAIR           reabrir 60 s el aviso Swift Pair
              UNPAIR         olvidar todos los PC emparejados
              OW=<0|1>       abrir el asistente al emparejar un PC nuevo
              MIC=<0|1|2>    microfono: 0 apagado, 1 solo medir, 2 acciones activas
              MG=<dB>        umbral de inicio de sonido sobre el piso de ruido
              DT=<ms>        ventana para el doble sonido = clic derecho (0 = sin)
              A2=<0..6>      sonido 2: 0 nada, 1 AvPag, 2 RePag, 3 flecha abajo x3,
                             4 espacio, 5 clic derecho, 6 doble clic
              CQ=<0|1>       clic quieto (deshacer el movimiento del sonido)
              MP=<0.5..0.99> probabilidad minima del modelo
              MW=<off>,<hex> | MW=END,<largo>,<crc32 hex> | MW=CLR   cargar modelo
    Salida:   VER:<v>
              CFG:dz,sm,gh,gv,ejeH,signoH,ejeV,signoV,pausa,calibrado,
                  ac,at,it,wt,wn,reposo,ow,mic,mg,dt,a2,cq,mp,clasesModelo
              ST:MIC,<nivel dB>,<piso dB>,<us por trama>,<muestras/s>
              EV:<dur ms>,<centroide Hz>,<pico dB>,<tramas>,<clase>,<prob %>,<800 hex>
              ST:SND,<clase>,<izq|der|tecla|doble|ignorado>
              ST:MODEL,<clases>,<bytes>   |  MW:OK | MW:ERR,<motivo>
              ST:PAIR,<0|1>        aviso Swift Pair activo
              ST:BONDS,<n>         PC emparejados guardados
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
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_iadc.h"
#include "nvm3.h"
#include "nvm3_default.h"

// ================================================================ Sonido (declaraciones)
// HeadMouse - modulo de sonido (microfono analogico MSM381ACT001 de la XIAO MG24 Sense)
//
// Cadena: IADC a 16 kHz (timer interno del IADC, interrupcion por FIFO)
//   -> tramas de 512 muestras cada 256 (32 ms, salto 16 ms)
//   -> ventana de Hann + FFT de 512 puntos
//   -> 16 bandas log-mel (300-7000 Hz), en medios dB sobre el piso de ruido de cada banda
//   -> detector de inicio: el nivel total supera el piso en "onsetDb"
//   -> ventana del evento: hasta 25 tramas (400 ms), empieza 2 tramas antes del
//      inicio y se corta antes si vuelve el silencio 3 tramas seguidas.
//
// Clasificador opcional (cargado desde el panel con MW=...):
//   entrada  13 x 16 = 208 valores (pares de tramas promediados) / 16
//   oculta   nHid neuronas ReLU, pesos int8 con escala por neurona
//   salida   nOut clases: 0 = ruido, 1 = sonido 1, 2 = sonido 2 (softmax)

namespace snd {

const int NB = 16;             // bandas mel
const int NF = 25;             // tramas por evento (400 ms)
const int NPOOL = 13;          // tramas tras promediar de a pares
const int NIN = NPOOL * NB;    // 208 entradas del modelo
const int FRAME_MS = 16;
const int MAX_MODEL = 6144;    // bytes

struct Event {
  int8_t feat[NF][NB];         // medios dB sobre el piso (0 = piso)
  uint8_t nFrames;             // tramas con datos reales (el resto queda en 0)
  uint16_t durMs;              // duracion sobre el umbral
  uint16_t centroidHz;         // centroide espectral del sonido
  uint8_t peakDb;              // nivel maximo sobre el piso
  uint32_t onsetMs;            // millis() del inicio
};

bool begin();                  // enciende el microfono y empieza a muestrear
void end();
bool running();

// Procesa las tramas pendientes. Devuelve true cuando hay un evento listo en ev.
bool poll(Event& ev);
bool collecting();             // hay un evento en curso
uint32_t collectingOnset();

extern float onsetDb;          // umbral de inicio (dB sobre el piso)
extern float levelDb, floorDb; // nivel actual y piso de ruido
extern uint32_t frameUs;       // tiempo de proceso de la ultima trama
extern uint32_t overruns;      // tramas perdidas por atraso
extern uint32_t frames;        // tramas procesadas (para medir la frecuencia real)

// Detector por reglas (sin modelo): sonido corto y agudo, p. ej. chasquido.
bool ruleClick(const Event& ev);

// Modelo
bool modelLoaded();
uint8_t modelClasses();
uint16_t modelBytes();
void loadModel();                              // desde flash (NVM3) al iniciar
int classify(const Event& ev, float* probs);   // clase con mayor probabilidad
bool mwChunk(uint16_t off, const char* hex);   // trozo hex recibido del panel
const char* mwEnd(uint16_t len, uint32_t crc); // nullptr = ok; si no, motivo
void mwClear();

uint32_t crc32(const uint8_t* d, size_t n);

}  // namespace snd

#define DEVICE_NAME "HeadMouse-MG24"
#define FW_VERSION  "3.0"
#define WIZARD_CMD  "msedge gbtox.github.io/airmouse/asistente"

#ifndef LED_BUILTIN_ACTIVE
#define LED_BUILTIN_ACTIVE   LOW
#define LED_BUILTIN_INACTIVE HIGH
#endif

#ifndef ARDUINO_SILABS_STACK_BLE_SILABS
#error "Selecciona 'BLE (Silabs)' en Herramientas > Protocol stack."
#endif

// ---------------------------------------------------------------- Config
const uint32_t CONFIG_MAGIC = 0x484D3341; // "HM3A": v3.0 (agrega sonido)

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
  uint8_t openWizard; // 1 = abrir el asistente al emparejar un PC nuevo
  uint8_t micMode;    // 0 apagado, 1 solo medir, 2 acciones activas
  float micGate;      // dB sobre el piso para detectar un sonido
  uint16_t dblMs;     // dos sonidos 1 dentro de este tiempo = clic derecho (0 = sin)
  uint8_t act2;       // accion del sonido 2
  uint8_t stillClick; // deshacer el movimiento durante el sonido
  float minProb;      // probabilidad minima del modelo
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
  cfg.openWizard = 1;
  cfg.micMode = 0;      // el microfono se activa desde el panel
  cfg.micGate = 12.0f;
  cfg.dblMs = 500;
  cfg.act2 = 1;         // Av Pag
  cfg.stillClick = 1;
  cfg.minProb = 0.75f;
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
  if (cfg.openWizard > 1) return false;
  if (cfg.micMode > 2) return false;
  if (isnan(cfg.micGate) || cfg.micGate < 4 || cfg.micGate > 40) return false;
  if (cfg.dblMs > 1000 || cfg.act2 > 6 || cfg.stillClick > 1) return false;
  if (isnan(cfg.minProb) || cfg.minProb < 0.5f || cfg.minProb > 0.99f) return false;
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
static uint16_t hid_report;          // reporte de mouse (ID 1)
static uint16_t hid_kbd;             // reporte de teclado (ID 2)
static uint16_t cfg_cmd, cfg_out;    // servicio de configuracion
static bool cfgOutSub = false;       // alguien (el panel) escucha OUT
static bd_addr ble_address = { { 0, 0, 0, 0, 0, 0 } };
static uint8_t bleState = 0, bleStateSent = 255;
static uint8_t advSet = 0xff;

// Swift Pair: ventana tras encender
const uint32_t PAIR_WINDOW_MS = 60000;
unsigned long pairWindowUntil = 0;
bool swiftPairOn = false;
uint8_t pairSent = 255;
bool bondsDirty = true;              // reenviar ST:BONDS
bool newPeer = false;                // la conexion actual aun no estaba emparejada
volatile bool bondFailMsg = false;

// Apertura del asistente con el teclado
unsigned long welcomeAt = 0;         // 0 = nada pendiente
int16_t welcomeStep = -1;            // -1 = inactivo
unsigned long kbdNext = 0;

static void ble_initialize_gatt_db();
static void ble_start_advertising();
void bleNotifyOut(const uint8_t* data, size_t len);
void openPairWindow();
void applyMic();
void sendModelInfo();

// Sonido
float headPeak = 0;                  // giro mas rapido reciente (decae), para ignorar sonidos con la cabeza en movimiento
struct MoveRec { uint32_t t; int8_t dx, dy; };
static MoveRec moveHist[64];         // ultimos ~640 ms de movimiento enviado
static uint8_t moveIdx = 0;
bool pendingS1 = false;              // sonido 1 esperando por si llega el segundo (clic derecho)
uint32_t pendingOnset = 0;
uint8_t keyQ[8]; uint8_t keyQn = 0, keyQi = 0;   // teclas del sonido 2
bool keyDown = false;
unsigned long keyNext = 0;

// Salida doble: todo lo que se imprime va al USB y, si el panel esta
// suscrito por Bluetooth, tambien a la caracteristica OUT (en trozos de 20
// bytes, que caben con cualquier MTU).
// Las lineas completas pasan a un anillo y bleTxTask() las envia desde loop(),
// reintentando si el stack no tiene bufer libre (las lineas EV: son largas).
static uint8_t bleTx[2048];
static volatile uint16_t txHead = 0, txTail = 0;
class DualOut : public Print {
 public:
  bool bleMute = false;
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (!cfgOutSub || bleMute) return 1;
    if (len < sizeof(buf)) buf[len++] = c; else over = true;
    if (c == '\n') {
      uint16_t used = (txHead - txTail + sizeof(bleTx)) % sizeof(bleTx);
      if (!over && used + len < sizeof(bleTx) - 1) {   // sin espacio: se descarta la linea entera
        uint16_t h = txHead;
        for (size_t i = 0; i < len; i++) { bleTx[h] = buf[i]; h = (h + 1) % sizeof(bleTx); }
        txHead = h;
      }
      len = 0; over = false;
    }
    return 1;
  }
 private:
  uint8_t buf[900];
  size_t len = 0;
  bool over = false;
};
DualOut out;

void bleTxTask() {
  if (!cfgOutSub) { txTail = txHead; return; }
  for (int k = 0; k < 6 && txTail != txHead; k++) {
    uint8_t chunk[20]; size_t n = 0;
    uint16_t t = txTail;
    while (n < 20 && t != txHead) { chunk[n++] = bleTx[t]; t = (t + 1) % sizeof(bleTx); }
    if (sl_bt_gatt_server_notify_all(cfg_out, n, chunk) != SL_STATUS_OK) break;   // reintentar luego
    txTail = t;
  }
}

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
  out.print("CFG:");
  out.print(cfg.deadzone, 1);  out.print(',');
  out.print(cfg.smoothing, 2); out.print(',');
  out.print(cfg.gainH, 0);     out.print(',');
  out.print(cfg.gainV, 0);     out.print(',');
  out.print(cfg.axisH);        out.print(',');
  out.print(cfg.signH);        out.print(',');
  out.print(cfg.axisV);        out.print(',');
  out.print(cfg.signV);        out.print(',');
  out.print(isPaused);         out.print(',');
  out.print(cfg.calibrated);   out.print(',');
  out.print(cfg.accel, 2);     out.print(',');
  out.print(cfg.accelThr, 0);  out.print(',');
  out.print(cfg.idleSec);      out.print(',');
  out.print(cfg.wakeThr, 0);   out.print(',');
  out.print(cfg.wakeCount);    out.print(',');
  out.print(sleeping);         out.print(',');
  out.print(cfg.openWizard);   out.print(',');
  out.print(cfg.micMode);      out.print(',');
  out.print(cfg.micGate, 0);   out.print(',');
  out.print(cfg.dblMs);        out.print(',');
  out.print(cfg.act2);         out.print(',');
  out.print(cfg.stillClick);   out.print(',');
  out.print(cfg.minProb, 2);   out.print(',');
  out.println(snd::modelClasses());
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

  out.print("OFF:");
  out.print(off[0], 2); out.print(',');
  out.print(off[1], 2); out.print(',');
  out.print(off[2], 2);
  out.println(stable ? "" : ",inestable");
  led(bleState == 2);
}

// ---------------------------------------------------------------- Calibracion guiada
void startGuidedCal(char target) {
  calTarget = target;
  calIdx = 0;
  resetMotion();
  out.print("CAL:"); out.print(target); out.println(":GO");
}

void calError(const char* why) {
  out.print("CAL:"); out.print(calTarget); out.print(":ERR,"); out.println(why);
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

  out.print("CAL:"); out.print(calTarget); out.print(":OK,");
  out.print(best); out.print(','); out.println(sign);
  calTarget = 0;
  markDirty();
  sendConfig();
}

// ---------------------------------------------------------------- Comandos serie
void handleCommand(char* c) {
  if (!strcmp(c, "GET")) {
    out.println("VER:" FW_VERSION);
    sendConfig();
    bleStateSent = 255; sleepSent = 255; pairSent = 255; bondsDirty = true;
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
      out.println("ERR:AX invalido");
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
  } else if (!strcmp(c, "PAIR")) {
    openPairWindow();
  } else if (!strcmp(c, "UNPAIR")) {
    sl_bt_sm_delete_bondings();
    bondsDirty = true;
    if (connection_handle != SL_BT_INVALID_CONNECTION_HANDLE) sl_bt_connection_close(connection_handle);
    openPairWindow();
  } else if (!strncmp(c, "OW=", 3)) {
    cfg.openWizard = atoi(c + 3) ? 1 : 0; markDirty(); sendConfig();
  } else if (!strncmp(c, "MIC=", 4)) {
    cfg.micMode = constrain(atoi(c + 4), 0, 2); applyMic(); markDirty(); sendConfig();
  } else if (!strncmp(c, "MG=", 3)) {
    cfg.micGate = constrain(atof(c + 3), 4.0f, 40.0f); snd::onsetDb = cfg.micGate; markDirty();
  } else if (!strncmp(c, "DT=", 3)) {
    cfg.dblMs = constrain(atoi(c + 3), 0, 1000); markDirty();
  } else if (!strncmp(c, "A2=", 3)) {
    cfg.act2 = constrain(atoi(c + 3), 0, 6); markDirty(); sendConfig();
  } else if (!strncmp(c, "CQ=", 3)) {
    cfg.stillClick = atoi(c + 3) ? 1 : 0; markDirty(); sendConfig();
  } else if (!strncmp(c, "MP=", 3)) {
    cfg.minProb = constrain(atof(c + 3), 0.5f, 0.99f); markDirty();
  } else if (!strcmp(c, "MW=CLR")) {
    snd::mwClear(); out.println("MW:OK"); sendModelInfo(); sendConfig();
  } else if (!strncmp(c, "MW=END,", 7)) {
    unsigned int len = 0; unsigned long crc = 0;
    if (sscanf(c + 7, "%u,%lx", &len, &crc) != 2) { out.println("MW:ERR,formato"); return; }
    const char* err = snd::mwEnd(len, crc);
    if (err) { out.print("MW:ERR,"); out.println(err); } else out.println("MW:OK");
    sendModelInfo(); sendConfig();
  } else if (!strncmp(c, "MW=", 3)) {
    char* comma = strchr(c + 3, ',');
    if (!comma || !snd::mwChunk(atoi(c + 3), comma + 1)) out.println("MW:ERR,trozo");
  } else if (!strcmp(c, "RST")) {
    setDefaults(); applyMic(); markDirty(); sendConfig();
  } else {
    out.print("ERR:comando desconocido "); out.println(c);
  }
  if (!strcmp(c, "GET")) sendModelInfo();
}

// Lector de lineas: uno para USB y otro para BLE, asi no se mezclan
struct LineReader {
  char buf[128];
  uint8_t len = 0;
  void feed(char ch) {
    if (ch == '\n' || ch == '\r') {
      if (len) { buf[len] = 0; handleCommand(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
};
LineReader usbReader, bleReader;

// Los comandos por BLE llegan en el contexto del stack Bluetooth; se copian a
// un anillo y se procesan en loop() (calibrateOffset usa delay, EEPROM, etc.).
static char bleRx[256];
static volatile uint16_t bleRxHead = 0, bleRxTail = 0;
void bleRxPush(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    uint16_t next = (bleRxHead + 1) % sizeof(bleRx);
    if (next == bleRxTail) return;   // lleno: se descarta el resto
    bleRx[bleRxHead] = (char)d[i];
    bleRxHead = next;
  }
}

void pollInput() {
  while (Serial.available()) usbReader.feed((char)Serial.read());
  while (bleRxTail != bleRxHead) {
    char ch = bleRx[bleRxTail];
    bleRxTail = (bleRxTail + 1) % sizeof(bleRx);
    bleReader.feed(ch);
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

// ---------------------------------------------------------------- Teclado
// Abre el asistente: Win+R, escribe WIZARD_CMD, Enter. Solo se usan teclas
// que son iguales en las distribuciones latinoamericana, espanola y de EE. UU.:
// letras, espacio, el punto de la fila principal y la "/" del teclado numerico.
void sendKey(uint8_t mod, uint8_t key) {
  uint8_t r[8] = { mod, 0, key, 0, 0, 0, 0, 0 };
  sl_bt_gatt_server_notify_all(hid_kbd, sizeof(r), r);
}

uint8_t usageFor(char ch) {
  if (ch >= 'a' && ch <= 'z') return 0x04 + (ch - 'a');
  if (ch == ' ') return 0x2C;
  if (ch == '.') return 0x37;
  if (ch == '/') return 0x54;   // "/" del teclado numerico
  return 0;
}

// Paso -1 inactivo. 0: Win+R. 1: soltar. 2..: una letra por paso (pulsar y
// soltar en pasos alternos). Al final: Enter.
void keyboardTask() {
  unsigned long now = millis();
  if (welcomeAt && (long)(now - welcomeAt) >= 0) { welcomeAt = 0; welcomeStep = 0; kbdNext = now; }
  if (welcomeStep < 0 || (long)(now - kbdNext) < 0) return;
  if (bleState != 2) { welcomeStep = -1; return; }   // se perdio la conexion

  static const char text[] = WIZARD_CMD;
  const int16_t nText = sizeof(text) - 1;
  int16_t s = welcomeStep;
  if (s == 0)      { sendKey(0x08, 0x15); kbdNext = now + 30; }      // GUI + r
  else if (s == 1) { sendKey(0, 0);       kbdNext = now + 700; }     // esperar la ventana Ejecutar
  else if (s < 2 + 2 * nText) {
    int16_t i = (s - 2) / 2;
    if ((s - 2) % 2 == 0) sendKey(0, usageFor(text[i])); else sendKey(0, 0);
    kbdNext = now + 15;
  }
  else if (s == 2 + 2 * nText) { sendKey(0, 0x28); kbdNext = now + 30; }  // Enter
  else { sendKey(0, 0); welcomeStep = -1; out.println("ST:WIZARD,abierto"); return; }
  welcomeStep++;
}

// ---------------------------------------------------------------- Sonido
void applyMic() {
  snd::onsetDb = cfg.micGate;
  if (cfg.micMode && !snd::running()) snd::begin();
  if (!cfg.micMode && snd::running()) snd::end();
  pendingS1 = false;
}

void sendModelInfo() {
  out.print("ST:MODEL,"); out.print(snd::modelClasses()); out.print(','); out.println(snd::modelBytes());
}

bool linkReady() {
  return connection_handle != SL_BT_INVALID_CONNECTION_HANDLE && bonding_handle != SL_BT_INVALID_BONDING_HANDLE;
}

void sendButtons(uint8_t b) {
  uint8_t r[3] = { 0, 0, b };
  sl_bt_gatt_server_notify_all(hid_report, sizeof(r), r);
}

// Deshace el movimiento enviado desde el instante t (el gesto de la boca
// puede mover los lentes mientras suena)
void undoSince(uint32_t t) {
  int sx = 0, sy = 0;
  for (auto& m : moveHist) {
    if (m.t && (long)(m.t - t) >= 0) { sx += m.dx; sy += m.dy; m.dx = m.dy = 0; }
  }
  while (sx || sy) {
    int dx = constrain(-sx, -127, 127), dy = constrain(-sy, -127, 127);
    uint8_t r[3] = { (uint8_t)(int8_t)dx, (uint8_t)(int8_t)dy, 0 };
    sl_bt_gatt_server_notify_all(hid_report, sizeof(r), r);
    sx += dx; sy += dy;
  }
}

void soundClick(uint8_t btn, uint32_t since, bool dbl) {
  if (!linkReady()) return;
  if (cfg.stillClick) undoSince(since - 32);
  sendButtons(btn); sendButtons(0);
  if (dbl) { sendButtons(btn); sendButtons(0); }
  resetMotion();
  cooldownUntil = millis() + 250;   // el cursor espera un momento quieto tras el clic
  lastActivity = millis();
}

void queueKeys(uint8_t key, uint8_t n) {
  if (keyQn) return;   // aun enviando la anterior
  for (uint8_t i = 0; i < n && i < sizeof(keyQ); i++) keyQ[i] = key;
  keyQn = min(n, (uint8_t)sizeof(keyQ)); keyQi = 0; keyDown = false; keyNext = millis();
}

void keyQueueTask() {
  if (!keyQn || welcomeStep >= 0 || (long)(millis() - keyNext) < 0) return;
  if (!keyDown) { sendKey(0, keyQ[keyQi]); keyDown = true; }
  else { sendKey(0, 0); keyDown = false; if (++keyQi >= keyQn) keyQn = 0; }
  keyNext = millis() + 20;
}

void sndReport(int cls, const char* what) {
  out.print("ST:SND,"); out.print(cls); out.print(','); out.println(what);
}

void action2(uint32_t onset) {
  switch (cfg.act2) {
    case 1: queueKeys(0x4E, 1); sndReport(2, "tecla"); break;   // Av Pag
    case 2: queueKeys(0x4B, 1); sndReport(2, "tecla"); break;   // Re Pag
    case 3: queueKeys(0x51, 3); sndReport(2, "tecla"); break;   // flecha abajo x3
    case 4: queueKeys(0x2C, 1); sndReport(2, "tecla"); break;   // espacio
    case 5: soundClick(2, onset, false); sndReport(2, "der"); break;
    case 6: soundClick(1, onset, true); sndReport(2, "doble"); break;
    default: break;
  }
}

void onSoundEvent(const snd::Event& ev) {
  float pr[3] = { 0, 0, 0 };
  int cls;
  if (snd::modelLoaded()) {
    cls = snd::classify(ev, pr);
    if (cls > 0 && pr[cls] < cfg.minProb) cls = 0;
  } else {
    cls = snd::ruleClick(ev) ? 1 : 0;
    pr[cls] = 1;
  }
  // Para el panel: rasgos del evento (probador y entrenamiento)
  static const char hx[] = "0123456789abcdef";
  out.print("EV:"); out.print(ev.durMs); out.print(','); out.print(ev.centroidHz); out.print(',');
  out.print(ev.peakDb); out.print(','); out.print(ev.nFrames); out.print(','); out.print(cls); out.print(',');
  out.print((int)(pr[cls >= 0 ? cls : 0] * 100)); out.print(',');
  for (int f = 0; f < snd::NF; f++)
    for (int b = 0; b < snd::NB; b++) { uint8_t v = (uint8_t)ev.feat[f][b]; out.write(hx[v >> 4]); out.write(hx[v & 15]); }
  out.println();

  if (cfg.micMode != 2 || cls <= 0) return;
  if (isPaused || sleeping || calTarget || bleState != 2 || headPeak > cfg.wakeThr) { sndReport(cls, "ignorado"); return; }

  if (cls == 2) { action2(ev.onsetMs); return; }
  // Sonido 1
  if (cfg.dblMs == 0) { soundClick(1, ev.onsetMs, false); sndReport(1, "izq"); return; }
  if (pendingS1 && ev.onsetMs - pendingOnset <= cfg.dblMs) {
    pendingS1 = false;
    soundClick(2, pendingOnset, false); sndReport(1, "der");
  } else {
    if (pendingS1) { soundClick(1, pendingOnset, false); sndReport(1, "izq"); }
    pendingS1 = true; pendingOnset = ev.onsetMs;
  }
}

void soundTask() {
  if (!snd::running()) return;
  snd::Event ev;
  if (snd::poll(ev)) onSoundEvent(ev);

  // Sin segundo sonido a tiempo: era un clic izquierdo
  if (pendingS1 && millis() - pendingOnset > cfg.dblMs &&
      !(snd::collecting() && snd::collectingOnset() - pendingOnset <= cfg.dblMs)) {
    pendingS1 = false;
    soundClick(1, pendingOnset, false); sndReport(1, "izq");
  }

  static unsigned long lastMic = 0, lastRate = 0;
  static uint32_t lastFrames = 0, sps = 0;
  static uint8_t micN = 0;
  unsigned long now = millis();
  if (now - lastRate >= 1000) {
    sps = (snd::frames - lastFrames) * 256 * 1000UL / (now - lastRate);
    lastFrames = snd::frames; lastRate = now;
  }
  if (now - lastMic >= 100) {
    lastMic = now;
    out.bleMute = (++micN & 1);   // por BLE a 5 Hz
    out.print("ST:MIC,"); out.print(snd::levelDb, 1); out.print(','); out.print(snd::floorDb, 1);
    out.print(','); out.print(snd::frameUs); out.print(','); out.println(sps);
    out.bleMute = false;
  }
}

// ---------------------------------------------------------------- Emparejamiento
// Si hay un PC conectado, la ventana se aplica cuando se desconecte.
void openPairWindow() {
  pairWindowUntil = millis() + PAIR_WINDOW_MS;
  if (connection_handle == SL_BT_INVALID_CONNECTION_HANDLE) ble_start_advertising();
}

void bleNotifyOut(const uint8_t* data, size_t len) {
  if (cfg_out && cfgOutSub) sl_bt_gatt_server_notify_all(cfg_out, len, data);
}

void pairTask() {
  // Al vencer la ventana, quitar el bloque Swift Pair del anuncio
  if (swiftPairOn && (long)(millis() - pairWindowUntil) >= 0 &&
      connection_handle == SL_BT_INVALID_CONNECTION_HANDLE) {
    ble_start_advertising();
  }
  if (bondFailMsg) {
    bondFailMsg = false;
    out.println("ERR:emparejamiento fallido; quita el dispositivo en el PC y vuelve a agregarlo");
  }
  uint8_t p = swiftPairOn ? 1 : 0;
  if (p != pairSent) { pairSent = p; out.print("ST:PAIR,"); out.println(p); }
  if (bondsDirty) {
    bondsDirty = false;
    uint32_t n = 0; size_t blen = 0; uint8_t bits[4];
    if (sl_bt_sm_get_bonding_handles(0, &n, sizeof(bits), &blen, bits) == SL_STATUS_OK) {
      out.print("ST:BONDS,"); out.println(n);
    }
  }
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
  headPeak = fmaxf(headPeak * 0.97f, fmaxf(fabsf(rateH), fabsf(rateV)));

  if (millis() - lastTelemetry >= 50) {
    static uint8_t telemN = 0;
    lastTelemetry = millis();
    out.bleMute = (++telemN & 1);   // por BLE va a 10 Hz para no cargar el enlace del mouse
    out.print("R:"); out.print(rateH, 1); out.print(','); out.print(rateV, 1);
    out.print(','); out.println(lastMult, 2);
    if (rawStream) {
      out.print("G:"); out.print(g[0], 1); out.print(',');
      out.print(g[1], 1); out.print(','); out.println(g[2], 1);
    }
    out.bleMute = false;
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
    moveHist[moveIdx] = { (uint32_t)now, (int8_t)dx, (int8_t)dy };
    moveIdx = (moveIdx + 1) % 64;
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
  if (!imuOk) out.println("ERR:IMU no responde");
  else calibrateOffset();

  snd::loadModel();
  applyMic();

  out.println("VER:" FW_VERSION);
  sendConfig();
  lastTick = millis();
}

void loop() {
  pollInput();
  keyboardTask();
  keyQueueTask();
  pairTask();
  soundTask();
  bleTxTask();

  if (pendingSave && millis() - lastChangeTime > 2000) {
    EEPROM.put(0, cfg);
    pendingSave = false;
  }

  if (bleState != bleStateSent) {
    bleStateSent = bleState;
    out.print("ST:BLE,"); out.println(bleState);
    led(bleState == 2);
  }

  if (sleeping != sleepSent) {
    sleepSent = sleeping;
    out.print("ST:SLEEP,"); out.println(sleeping);
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
      // Hasta 4 PC; al llenarse se reemplaza el que se uso hace mas tiempo (politica 2)
      sl_bt_sm_store_bonding_configuration(4, 2);
      pairWindowUntil = millis() + PAIR_WINDOW_MS;
      bondsDirty = true;
      ble_start_advertising();
      break;

    case sl_bt_evt_connection_opened_id:
      connection_handle = evt->data.evt_connection_opened.connection;
      bonding_handle = evt->data.evt_connection_opened.bonding;
      newPeer = (bonding_handle == SL_BT_INVALID_BONDING_HANDLE);
      bleState = 1;
      sl_bt_sm_increase_security(connection_handle);
      ble_start_advertising();   // sin Swift Pair y no conectable (visible para Web Bluetooth)
      break;

    case sl_bt_evt_sm_bonded_id:
      connection_handle = evt->data.evt_sm_bonded.connection;
      bonding_handle = evt->data.evt_sm_bonded.bonding;
      bondsDirty = true;
      // PC nuevo: abrir el asistente cuando Windows termine de instalar el driver
      if (newPeer && cfg.openWizard) welcomeAt = millis() + 4000;
      newPeer = false;
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
      if (evt->data.evt_gatt_server_characteristic_status.characteristic == cfg_out &&
          evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config) {
        cfgOutSub = evt->data.evt_gatt_server_characteristic_status.client_config_flags != 0;
      }
      break;

    case sl_bt_evt_gatt_server_attribute_value_id:
      if (evt->data.evt_gatt_server_attribute_value.attribute == cfg_cmd) {
        bleRxPush(evt->data.evt_gatt_server_attribute_value.value.data,
                  evt->data.evt_gatt_server_attribute_value.value.len);
      }
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
      cfgOutSub = false;
      newPeer = false;
      welcomeAt = 0; welcomeStep = -1;
      // v2: NO se borra el emparejamiento; el PC se reconecta solo.
      ble_start_advertising();
      break;

    case sl_bt_evt_sm_bonding_failed_id:
      // Claves desincronizadas (p. ej. el PC recuerda un emparejamiento viejo):
      // se borra ese emparejamiento (o todos, si no se sabe cual) y se cierra
      // para reintentar limpio.
      if (bonding_handle != SL_BT_INVALID_BONDING_HANDLE) sl_bt_sm_delete_bonding(bonding_handle);
      else sl_bt_sm_delete_bondings();
      bondsDirty = true;
      sl_bt_connection_close(evt->data.evt_sm_bonding_failed.connection);
      bondFailMsg = true;   // se informa desde loop()
      break;

    default:
      break;
  }
}

// Anuncio armado a mano:
//   sin conexion  -> conectable; con el bloque Swift Pair si la ventana esta abierta
//   con conexion  -> solo escaneable (para que Chrome/Edge lo vean con Web Bluetooth)
static void ble_start_advertising() {
  sl_status_t sc;
  if (advSet == 0xff) {
    sc = sl_bt_advertiser_create_set(&advSet);
    app_assert_status(sc);
    sc = sl_bt_advertiser_set_timing(advSet, 160, 160, 0, 0);   // 100 ms
    app_assert_status(sc);
  }
  sl_bt_advertiser_stop(advSet);   // puede fallar si no estaba activo; da igual

  bool connected = connection_handle != SL_BT_INVALID_CONNECTION_HANDLE;
  swiftPairOn = !connected && (long)(pairWindowUntil - millis()) > 0;

  uint8_t adv[31];
  uint8_t n = 0;
  const uint8_t base[] = {
    0x02, 0x01, 0x06,               // flags: LE general discoverable, sin BR/EDR
    0x03, 0x19, 0xC2, 0x03,         // apariencia: mouse
    0x03, 0x03, 0x12, 0x18          // servicio HID 0x1812
  };
  memcpy(adv, base, sizeof(base)); n = sizeof(base);
  if (swiftPairOn) {
    // Microsoft Swift Pair: fabricante 0x0006, beacon 0x03, LE 0x00, RSSI reservado 0x80
    const uint8_t sp[] = { 0x06, 0xFF, 0x06, 0x00, 0x03, 0x00, 0x80 };
    memcpy(adv + n, sp, sizeof(sp)); n += sizeof(sp);
  }
  sc = sl_bt_legacy_advertiser_set_data(advSet, sl_bt_advertiser_advertising_data_packet, n, adv);
  app_assert_status(sc);

  uint8_t rsp[31];
  uint8_t nameLen = strlen(DEVICE_NAME);
  rsp[0] = nameLen + 1; rsp[1] = 0x09;   // nombre completo
  memcpy(rsp + 2, DEVICE_NAME, nameLen);
  sc = sl_bt_legacy_advertiser_set_data(advSet, sl_bt_advertiser_scan_response_packet, nameLen + 2, rsp);
  app_assert_status(sc);

  sc = sl_bt_legacy_advertiser_start(advSet, connected ? sl_bt_legacy_advertiser_scannable
                                                       : sl_bt_legacy_advertiser_connectable);
  // Serial directo: esto puede correr en el contexto del stack BLE
  if (sc != SL_STATUS_OK) { Serial.print("ERR:anuncio 0x"); Serial.println(sc, HEX); }
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
  const uint8_t ref_val[] = { 0x01, 0x01 };   // report ID 1 (mouse), tipo Input
  sc = sl_bt_gattdb_add_uuid16_descriptor(session, characteristic, SL_BT_GATTDB_DESCRIPTOR_READ, SL_BT_GATTDB_ENCRYPTED_READ,
                                          ref_uuid, sl_bt_gattdb_fixed_length_value, 2, 2, ref_val, &descriptor);
  app_assert_status(sc);

  // Reporte de teclado (ID 2): modificadores, reservado, 6 teclas
  const uint8_t kbd_val[8] = { 0 };
  sc = sl_bt_gattdb_add_uuid16_characteristic(session, service,
                                              SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
                                              0, 0, rep_uuid, sl_bt_gattdb_fixed_length_value, 8, 8, kbd_val, &characteristic);
  app_assert_status(sc);
  hid_kbd = characteristic;
  const uint8_t kbd_ref[] = { 0x02, 0x01 };   // report ID 2, tipo Input
  sc = sl_bt_gattdb_add_uuid16_descriptor(session, characteristic, SL_BT_GATTDB_DESCRIPTOR_READ, SL_BT_GATTDB_ENCRYPTED_READ,
                                          ref_uuid, sl_bt_gattdb_fixed_length_value, 2, 2, kbd_ref, &descriptor);
  app_assert_status(sc);

  const sl_bt_uuid_16_t map_uuid = { .data = { 0x4B, 0x2A } };
  const uint8_t map_val[] = {
    // Mouse, report ID 1: X, Y (relativos, -128..127) y 3 botones
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x01, 0x09, 0x01, 0xA1, 0x00,
    0x09, 0x30, 0x09, 0x31, 0x15, 0x80, 0x25, 0x7F, 0x95, 0x02,
    0x75, 0x08, 0x81, 0x06, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x05, 0x81, 0x03, 0xC0, 0xC0,
    // Teclado, report ID 2: 8 modificadores, 1 byte reservado, 6 teclas
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x02,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0
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

  // Configuracion por Bluetooth (Web Bluetooth). UUIDs en orden little-endian:
  //   servicio c2a10001-5b3e-4f7a-9d61-8e4f2b6a1d30
  //   CMD      c2a10002-...   (el panel escribe comandos de texto)
  //   OUT      c2a10003-...   (la placa notifica sus lineas de salida)
  uint8_t cfg_svc_uuid[16] = { 0x30, 0x1d, 0x6a, 0x2b, 0x4f, 0x8e, 0x61, 0x9d,
                               0x7a, 0x4f, 0x3e, 0x5b, 0x01, 0x00, 0xa1, 0xc2 };
  sc = sl_bt_gattdb_add_service(session, sl_bt_gattdb_primary_service, 0, 16, cfg_svc_uuid, &service);
  app_assert_status(sc);
  uuid_128 cmd_uuid, out_uuid;
  memcpy(cmd_uuid.data, cfg_svc_uuid, 16); cmd_uuid.data[12] = 0x02;
  memcpy(out_uuid.data, cfg_svc_uuid, 16); out_uuid.data[12] = 0x03;
  const uint8_t empty[1] = { 0 };
  sc = sl_bt_gattdb_add_uuid128_characteristic(session, service,
                                               SL_BT_GATTDB_CHARACTERISTIC_WRITE | SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE,
                                               0, 0, cmd_uuid, sl_bt_gattdb_variable_length_value, 64, 0, empty, &characteristic);
  app_assert_status(sc);
  cfg_cmd = characteristic;
  sc = sl_bt_gattdb_add_uuid128_characteristic(session, service,
                                               SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
                                               0, 0, out_uuid, sl_bt_gattdb_variable_length_value, 20, 0, empty, &characteristic);
  app_assert_status(sc);
  cfg_out = characteristic;
  sc = sl_bt_gattdb_start_service(session, service);
  app_assert_status(sc);

  sc = sl_bt_gattdb_commit(session);
  app_assert_status(sc);
}

// ================================================================ Sonido (implementacion)
namespace snd {

// ---------------------------------------------------------------- Muestreo
const uint32_t FS = 16000;
const int N = 512;              // muestras por trama
const int HOP = 256;            // salto entre tramas
const uint32_t RING = 2048;     // potencia de 2 (128 ms)

static volatile uint16_t ring[RING];
static volatile uint32_t wr = 0;
static uint32_t rd = 0;
static bool on = false;

float onsetDb = 12.0f;
float levelDb = 0, floorDb = 0;
uint32_t frameUs = 0, overruns = 0, frames = 0;

}  // namespace snd

extern "C" void IADC_IRQHandler(void) {
  while (IADC_getSingleFifoCnt(IADC0)) {
    uint32_t v = IADC_pullSingleFifoData(IADC0);
    snd::ring[snd::wr & (snd::RING - 1)] = (uint16_t)v;
    snd::wr = snd::wr + 1;
  }
  IADC_clearInt(IADC0, IADC_IF_SINGLEFIFODVL | IADC_IF_SINGLEFIFOOF);
}

namespace snd {

// ---------------------------------------------------------------- Tablas
static float win[N];
static float cosT[N / 2], sinT[N / 2];
static float re[N], im[N];
static uint8_t binJ[N / 2 + 1];  // 255 = fuera de las bandas
static float binW[N / 2 + 1];    // peso hacia la banda binJ (el resto va a binJ-1)
static float fc[NB];             // frecuencia central de cada banda
static bool tablesReady = false;

static float hz2mel(float f) { return 2595.0f * log10f(1.0f + f / 700.0f); }
static float mel2hz(float m) { return 700.0f * (powf(10.0f, m / 2595.0f) - 1.0f); }

static void buildTables() {
  if (tablesReady) return;
  for (int i = 0; i < N; i++) win[i] = 0.5f - 0.5f * cosf(2.0f * PI * i / (N - 1));
  for (int k = 0; k < N / 2; k++) { cosT[k] = cosf(2.0f * PI * k / N); sinT[k] = sinf(2.0f * PI * k / N); }
  // NB bandas triangulares: NB + 2 puntos equiespaciados en mel entre 300 y 7000 Hz
  float m0 = hz2mel(300.0f), m1 = hz2mel(7000.0f);
  float pts[NB + 2];
  for (int i = 0; i < NB + 2; i++) pts[i] = m0 + (m1 - m0) * i / (NB + 1);
  for (int b = 0; b < NB; b++) fc[b] = mel2hz(pts[b + 1]);
  for (int k = 0; k <= N / 2; k++) {
    float m = hz2mel((float)k * FS / N);
    binJ[k] = 255; binW[k] = 0;
    for (int j = 0; j < NB + 1; j++) {
      if (m >= pts[j] && m < pts[j + 1]) {
        binJ[k] = j;                                  // sube hacia la banda j (centro pts[j+1])
        binW[k] = (m - pts[j]) / (pts[j + 1] - pts[j]);
        break;
      }
    }
  }
  tablesReady = true;
}

static void fft() {
  for (int i = 1, j = 0; i < N; i++) {
    int bit = N >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
  }
  for (int len = 2; len <= N; len <<= 1) {
    int half = len >> 1, step = N / len;
    for (int i = 0; i < N; i += len) {
      for (int k = 0; k < half; k++) {
        float wr_ = cosT[k * step], wi = -sinT[k * step];
        int a = i + k, b = a + half;
        float xr = re[b] * wr_ - im[b] * wi;
        float xi = re[b] * wi + im[b] * wr_;
        re[b] = re[a] - xr; im[b] = im[a] - xi;
        re[a] += xr;        im[a] += xi;
      }
    }
  }
}

// ---------------------------------------------------------------- Encendido
bool begin() {
  if (on) return true;
  buildTables();
  // Alimentacion del microfono (PC8) y entrada analogica (PC9)
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortC, 8, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortC, 9, gpioModeDisabled, 0);

  CMU_ClockEnable(cmuClock_IADC0, true);
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);   // 20 MHz
  IADC_reset(IADC0);

  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t all = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t input = IADC_SINGLEINPUT_DEFAULT;

  init.warmup = iadcWarmupKeepWarm;
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, 20000000, 0);
  init.timerCycles = 20000000 / FS;                         // un disparo cada 62,5 us

  all.configs[0].reference = iadcCfgReferenceInt1V2;       // el microfono va polarizado < 1 V
  all.configs[0].vRef = 1210;
  all.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed8x;     // mas resolucion por sobremuestreo
  all.configs[0].analogGain = iadcCfgAnalogGain1x;
  all.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0, 10000000, 0, iadcCfgModeNormal,
                                                          init.srcClkPrescale);

  initSingle.alignment = iadcAlignRight16;
  initSingle.dataValidLevel = iadcFifoCfgDvl4;
  initSingle.triggerSelect = iadcTriggerSelTimer;
  initSingle.triggerAction = iadcTriggerActionOnce;
  initSingle.start = true;

  input.posInput = iadcPosInputPortCPin9;
  input.negInput = iadcNegInputGnd;

  IADC_init(IADC0, &init, &all);
  IADC_initSingle(IADC0, &initSingle, &input);
  GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0;

  IADC_clearInt(IADC0, _IADC_IF_MASK);
  IADC_enableInt(IADC0, IADC_IEN_SINGLEFIFODVL);
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);

  rd = wr;
  on = true;
  IADC_command(IADC0, iadcCmdEnableTimer);
  return true;
}

void end() {
  if (!on) return;
  IADC_command(IADC0, iadcCmdDisableTimer);
  NVIC_DisableIRQ(IADC_IRQn);
  IADC_reset(IADC0);
  GPIO->CDBUSALLOC &= ~_GPIO_CDBUSALLOC_CDODD0_MASK;
  GPIO_PinModeSet(gpioPortC, 8, gpioModePushPull, 0);
  on = false;
}

bool running() { return on; }

// ---------------------------------------------------------------- Tramas y eventos
static float bandFloor[NB];
static uint16_t warm = 0;          // tramas desde el encendido
static int8_t prevQ[2][NB];
static bool coll = false, armed = true;
static Event cur;
static uint8_t nIn = 0, quiet = 0, above = 0;
static float sumE = 0, sumEF = 0, peak = 0;

bool collecting() { return coll; }
uint32_t collectingOnset() { return cur.onsetMs; }

static int8_t q8(float v) {
  int x = (int)lroundf(v);
  return (int8_t)(x < -128 ? -128 : (x > 127 ? 127 : x));
}

// Devuelve true si termino un evento
static bool processFrame(Event& ev) {
  // DC fuera, ventana, FFT
  float mean = 0;
  for (int i = 0; i < N; i++) mean += re[i];
  mean /= N;
  for (int i = 0; i < N; i++) { re[i] = (re[i] - mean) * win[i]; im[i] = 0; }
  fft();

  float E[NB];
  for (int b = 0; b < NB; b++) E[b] = 0;
  for (int k = 1; k <= N / 2; k++) {
    uint8_t j = binJ[k];
    if (j == 255) continue;
    float p = re[k] * re[k] + im[k] * im[k];
    if (j < NB) E[j] += p * binW[k];
    if (j >= 1) E[j - 1] += p * (1.0f - binW[k]);
  }
  float tot = 0, dB[NB];
  for (int b = 0; b < NB; b++) { dB[b] = 10.0f * log10f(E[b] + 1.0f); tot += E[b]; }
  levelDb = 10.0f * log10f(tot + 1.0f);

  if (warm < 1000) warm++;
  if (warm <= 20) {   // primeras tramas: el piso arranca rapido
    float a = warm == 1 ? 1.0f : 0.3f;
    floorDb += a * (levelDb - floorDb);
    for (int b = 0; b < NB; b++) bandFloor[b] += a * (dB[b] - bandFloor[b]);
    for (int b = 0; b < NB; b++) { prevQ[0][b] = prevQ[1][b]; prevQ[1][b] = 0; }
    return false;
  }

  float rel = levelDb - floorDb;
  int8_t q[NB];
  for (int b = 0; b < NB; b++) q[b] = q8(2.0f * (dB[b] - bandFloor[b]));

  bool done = false;
  if (!coll) {
    if (armed && rel > onsetDb) {
      coll = true;
      memset(&cur, 0, sizeof(cur));
      memcpy(cur.feat[0], prevQ[0], NB);
      memcpy(cur.feat[1], prevQ[1], NB);
      nIn = 2; quiet = 0; above = 0;
      sumE = sumEF = peak = 0;
      cur.onsetMs = millis() - FRAME_MS;
    } else {
      // Piso de ruido: baja rapido, sube lento (~3 s)
      float a = levelDb < floorDb ? 0.1f : 0.005f;
      floorDb += a * (levelDb - floorDb);
      for (int b = 0; b < NB; b++) {
        float ab = dB[b] < bandFloor[b] ? 0.1f : 0.005f;
        bandFloor[b] += ab * (dB[b] - bandFloor[b]);
      }
      if (!armed && rel < 0.5f * onsetDb) armed = true;
    }
  }
  if (coll) {
    memcpy(cur.feat[nIn++], q, NB);
    if (rel > 0.5f * onsetDb) {
      above++; quiet = 0;
      for (int b = 0; b < NB; b++) {
        float e = powf(10.0f, (dB[b] - bandFloor[b]) / 10.0f) - 1.0f;
        if (e > 0) { sumE += e; sumEF += e * fc[b]; }
      }
    } else {
      quiet++;
    }
    if (rel > peak) peak = rel;
    if (quiet >= 3 || nIn >= NF) {
      cur.nFrames = nIn;
      for (int f = nIn; f < NF; f++) memset(cur.feat[f], 0, NB);
      cur.durMs = above * FRAME_MS;
      cur.centroidHz = sumE > 0 ? (uint16_t)(sumEF / sumE) : 0;
      cur.peakDb = (uint8_t)constrain((int)peak, 0, 255);
      armed = quiet >= 3;       // si sigue sonando (soplido largo), esperar a que baje
      coll = false;
      ev = cur;
      done = true;
    }
  }
  memcpy(prevQ[0], prevQ[1], NB);
  memcpy(prevQ[1], q, NB);
  return done;
}

bool poll(Event& ev) {
  if (!on) return false;
  uint32_t w = wr;
  if (w - rd > RING - HOP) { rd = w - N; overruns++; }   // nos atrasamos: saltar
  while (w - rd >= (uint32_t)N) {
    uint32_t t0 = micros();
    for (int i = 0; i < N; i++) re[i] = (float)ring[(rd + i) & (RING - 1)];
    rd += HOP;
    frames++;
    bool done = processFrame(ev);
    frameUs = micros() - t0;
    if (done) return true;
  }
  return false;
}

bool ruleClick(const Event& ev) {
  return ev.durMs > 0 && ev.durMs <= 100 && ev.centroidHz >= 1000 && ev.peakDb >= onsetDb + 3;
}

// ---------------------------------------------------------------- Modelo
// Formato (little-endian), armado por el panel:
//   u32 magic "HSM1" | u16 nIn | u8 nHid | u8 nOut
//   f32 s1[nHid] | f32 b1[nHid] | f32 W2[nOut*nHid] | f32 b2[nOut] | i8 W1[nHid*nIn]
const uint32_t MODEL_MAGIC = 0x314D5348;   // 'H','S','M','1'
const int MAX_HID = 24;
const nvm3_ObjectKey_t KEY_HDR = 0xE000;   // largo + crc; datos en 0xE001..
const size_t OBJ = 240;

static uint8_t blob[MAX_MODEL];
static bool loaded = false;
static uint8_t nHid = 0, nOut = 0;
static float s1[MAX_HID], b1[MAX_HID], W2[3 * MAX_HID], b2[3];
static const int8_t* W1 = nullptr;
static uint16_t blobLen = 0;

bool modelLoaded() { return loaded; }
uint8_t modelClasses() { return loaded ? nOut : 0; }
uint16_t modelBytes() { return loaded ? blobLen : 0; }

uint32_t crc32(const uint8_t* d, size_t n) {
  uint32_t c = 0xFFFFFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320 & (0 - (c & 1)));
  }
  return ~c;
}

static const char* parseBlob(uint16_t len) {
  if (len < 8) return "corto";
  uint32_t magic; uint16_t ni;
  memcpy(&magic, blob, 4); memcpy(&ni, blob + 4, 2);
  uint8_t nh = blob[6], no = blob[7];
  if (magic != MODEL_MAGIC) return "formato";
  if (ni != NIN) return "entradas";
  if (nh < 1 || nh > MAX_HID || no < 2 || no > 3) return "tamano";
  size_t need = 8 + 4 * (2 * nh + no * nh + no) + (size_t)nh * ni;
  if (need != len) return "largo";
  const uint8_t* p = blob + 8;
  memcpy(s1, p, 4 * nh); p += 4 * nh;
  memcpy(b1, p, 4 * nh); p += 4 * nh;
  memcpy(W2, p, 4 * no * nh); p += 4 * no * nh;
  memcpy(b2, p, 4 * no); p += 4 * no;
  W1 = (const int8_t*)p;
  nHid = nh; nOut = no; blobLen = len;
  loaded = true;
  return nullptr;
}

void loadModel() {
  loaded = false;
  uint8_t hdr[6];
  if (nvm3_readData(nvm3_defaultHandle, KEY_HDR, hdr, sizeof(hdr)) != ECODE_NVM3_OK) return;
  uint16_t len; uint32_t crc;
  memcpy(&len, hdr, 2); memcpy(&crc, hdr + 2, 4);
  if (len == 0 || len > MAX_MODEL) return;
  for (size_t off = 0, k = 1; off < len; off += OBJ, k++) {
    size_t n = min(OBJ, (size_t)len - off);
    if (nvm3_readData(nvm3_defaultHandle, KEY_HDR + k, blob + off, n) != ECODE_NVM3_OK) return;
  }
  if (crc32(blob, len) != crc) return;
  parseBlob(len);
}

static int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool mwChunk(uint16_t off, const char* hex) {
  if (off == 0) loaded = false;   // el bufer se reescribe: el modelo viejo deja de valer
  size_t n = strlen(hex) / 2;
  if (off + n > MAX_MODEL) return false;
  for (size_t i = 0; i < n; i++) {
    int a = hexv(hex[2 * i]), b = hexv(hex[2 * i + 1]);
    if (a < 0 || b < 0) return false;
    blob[off + i] = (uint8_t)(a * 16 + b);
  }
  return true;
}

const char* mwEnd(uint16_t len, uint32_t crc) {
  if (len == 0 || len > MAX_MODEL) return "largo";
  if (crc32(blob, len) != crc) return "crc";
  const char* err = parseBlob(len);
  if (err) return err;
  // Guardar en flash (NVM3, el mismo almacenamiento que usa EEPROM)
  for (size_t off = 0, k = 1; off < len; off += OBJ, k++) {
    size_t n = min(OBJ, (size_t)len - off);
    if (nvm3_writeData(nvm3_defaultHandle, KEY_HDR + k, blob + off, n) != ECODE_NVM3_OK) return "flash";
  }
  uint8_t hdr[6];
  memcpy(hdr, &len, 2); memcpy(hdr + 2, &crc, 4);
  if (nvm3_writeData(nvm3_defaultHandle, KEY_HDR, hdr, sizeof(hdr)) != ECODE_NVM3_OK) return "flash";
  if (nvm3_repackNeeded(nvm3_defaultHandle)) nvm3_repack(nvm3_defaultHandle);
  return nullptr;
}

void mwClear() {
  loaded = false;
  nvm3_deleteObject(nvm3_defaultHandle, KEY_HDR);
  for (nvm3_ObjectKey_t k = 1; k <= MAX_MODEL / OBJ + 1; k++) nvm3_deleteObject(nvm3_defaultHandle, KEY_HDR + k);
  if (nvm3_repackNeeded(nvm3_defaultHandle)) nvm3_repack(nvm3_defaultHandle);
}

int classify(const Event& ev, float* probs) {
  if (!loaded) return -1;
  static float x[NIN];
  for (int t = 0; t < NPOOL; t++) {
    int f0 = 2 * t, f1 = (2 * t + 1 < NF) ? 2 * t + 1 : 2 * t;
    for (int b = 0; b < NB; b++) x[t * NB + b] = (ev.feat[f0][b] + ev.feat[f1][b]) / 32.0f;
  }
  float h[MAX_HID];
  for (int j = 0; j < nHid; j++) {
    const int8_t* w = W1 + (size_t)j * NIN;
    float acc = 0;
    for (int i = 0; i < NIN; i++) acc += w[i] * x[i];
    float v = s1[j] * acc + b1[j];
    h[j] = v > 0 ? v : 0;
  }
  float z[3], mx = -1e30f;
  for (int k = 0; k < nOut; k++) {
    float a = b2[k];
    for (int j = 0; j < nHid; j++) a += W2[k * nHid + j] * h[j];
    z[k] = a; if (a > mx) mx = a;
  }
  float s = 0;
  for (int k = 0; k < nOut; k++) { z[k] = expf(z[k] - mx); s += z[k]; }
  int best = 0;
  for (int k = 0; k < nOut; k++) { probs[k] = z[k] / s; if (probs[k] > probs[best]) best = k; }
  return best;
}

}  // namespace snd
