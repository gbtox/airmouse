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
#pragma once
#include <Arduino.h>

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
