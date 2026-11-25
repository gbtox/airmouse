// HeadMouse - modulo de sonido. Ver Sonido.h.
#include "Sonido.h"
#include <math.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_iadc.h"
#include "nvm3.h"
#include "nvm3_default.h"

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
