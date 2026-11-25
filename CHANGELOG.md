# Historial de versiones — HeadMouse MG24

Mouse de cabeza por Bluetooth (BLE HID) para una persona con parálisis
cerebral. Placa **Seeed XIAO MG24 Sense** (IMU LSM6DS3TR-C) montada en la
varilla de los lentes: USB-C hacia la oreja, cara de chips hacia afuera.
Se configura desde el panel web `index.html` por USB (Web Serial) o por
Bluetooth (Web Bluetooth), en Chrome o Edge.

| Versión | Firmware | Panel | Estado |
|---|---|---|---|
| 3.0 | `Arduino/HeadMouse/HeadMouse.ino` (un solo archivo) | `index.html` (tarjetas de sonido) | Compila; panel probado con sonidos simulados; **falta probar en la placa** |
| 2.2 | (reemplazado por 3.0) | publicado en GitHub Pages | Probado en la placa: funciona |
| 2.1 | (reemplazado por 2.2) | (reemplazado por 2.2) | Probado en la placa |
| 2.0 | (reemplazado por 2.1) | (reemplazado por 2.1) | Probado: el cursor se mueve bien |
| 1 | `Arduino/Arduino_v1.txt` (respaldo) | anterior (en el historial de git) | Ejes incorrectos para este montaje |

Para cargar: Arduino IDE, placa *XIAO MG24*, **Herramientas → Protocol stack →
"BLE (Silabs)"**, librería *Seeed Arduino LSM6DS3* ≥ 2.0.4 (se usa 2.0.8).

---

## 3.0 — 2026-11-25

**Nuevo: clic por sonido** con el micrófono analógico de la placa Sense.
- **Sonido 1 = clic izquierdo.** Dos sonidos 1 seguidos, dentro de un tiempo
  ajustable (por defecto 500 ms), dan **clic derecho**. Con 0 ms el clic es
  inmediato y no hay clic derecho.
- **Sonido 2 = acción a elegir**: Av Pág (por defecto), Re Pág, flecha ↓ ×3,
  espacio, clic derecho o doble clic. Las teclas salen por el teclado HID
  de la 2.2, así que **no hay que volver a emparejar**.
- **Sin entrenar** reconoce un **chasquido** (sonido de menos de 100 ms,
  agudo y claramente sobre el ruido).
- **Con entrenamiento** reconoce el sonido propio de la persona. En la
  tarjeta "🧪 Enseñar sonidos":
  - Se graban 10 ejemplos de cada sonido candidato y 60 s de ruido normal.
  - **Comparar** dice cuál se reconoce mejor y cuántos falsos daría.
  - **Entrenar** arma una red pequeña en el navegador (≈2 s, sin internet).
    Muestra el acierto con ejemplos que no vio y la carga en la placa
    (≈3,7 KB, guardada en la flash).
  - "❌ El último fue un error" agrega un falso al ruido para volver a
    entrenar.
  - Las grabaciones se pueden exportar e importar en `.json`.
- **Clic quieto**: antes del clic se deshace el movimiento del cursor de los
  ~400 ms anteriores, porque el gesto de la boca mueve los lentes.
- Los sonidos se ignoran en pausa, en reposo, sin conexión y con la cabeza
  girando más rápido que el umbral de despertar (estornudo, tos).
- Modos del micrófono: apagado (por defecto, no consume), solo medir (para
  probar y enseñar) y activo.

**Cómo funciona en la placa** (sección "Sonido" de `HeadMouse.ino`)
- IADC a 16 kHz.
- FFT de 512 puntos cada 16 ms.
- 16 bandas mel entre 300 y 7000 Hz, medidas sobre un piso de ruido que se
  adapta.
- Un sonido empieza cuando el nivel supera el piso en el umbral
  (por defecto 12 dB). Termina con 48 ms de silencio, o a los 400 ms.

**Protocolo**
- Comandos nuevos:
  - `MIC=0|1|2`
  - `MG=` (umbral, dB)
  - `DT=` (ms)
  - `A2=0..6`
  - `CQ=0|1`
  - `MP=` (probabilidad mínima)
  - `MW=<off>,<hex>` / `MW=END,<largo>,<crc32>` / `MW=CLR` (modelo)
- `CFG:` agrega `mic,mg,dt,a2,cq,mp,clasesModelo`.
- Líneas nuevas:
  - `ST:MIC,<nivel>,<piso>,<us>,<muestras/s>` (10 Hz)
  - `EV:<dur>,<centroide>,<pico>,<tramas>,<clase>,<prob>,<800 hex>`
  - `ST:SND,<clase>,<acción>`
  - `ST:MODEL,<clases>,<bytes>`
  - `MW:OK` / `MW:ERR,<motivo>`
- Por Bluetooth, la salida pasa por una cola y se reintenta si el enlace va
  lleno (las líneas `EV:` miden ~830 bytes).

**Compatibilidad**: cambió el formato guardado (`HM3A`): la configuración
vuelve a los valores de fábrica al cargar la 3.0. Los emparejamientos se
conservan.

**Por confirmar en la placa**
- `ST:MIC` debe marcar unas **16000 muestras/s**. Si marca otra cifra, el
  disparo del IADC necesita ajuste.
- El nivel debe subir con la voz y con un chasquido (+20 dB o más), y estar
  quieto en silencio.
- Cada trama debe tardar menos de 3 ms, sin afectar el movimiento del
  cursor.

---

## 2.2 — 2026-11-24

**Nuevo**
- **Aviso de conexión en Windows (Swift Pair)**: durante 60 s después de
  encender, o hasta que un PC se conecta, el anuncio Bluetooth lleva el bloque
  de Microsoft que hace aparecer *"Nuevo dispositivo encontrado → Conectar"* en
  los PC con Windows cercanos (~1 m). Un PC ya emparejado se reconecta solo y
  el aviso se apaga. Para emparejar un PC nuevo: desenchufar y volver a enchufar.
- **Abre el asistente sola**: la placa es también **teclado**. La primera vez
  que se empareja con un PC, a los ~4 s pulsa Win+R y escribe
  `msedge gbtox.github.io/airmouse/asistente` + Enter. Usa solo teclas iguales
  en las distribuciones latinoamericana, española y de EE. UU. (la `/` del
  teclado numérico). Se desactiva en el panel.
- **Asistente paso a paso** (`#asistente`, botón 🧭): conectar, punto cero,
  dirección horizontal y vertical (con corrección o calibración si va mal),
  medición de 15 s de movimientos involuntarios con umbral de despertar
  sugerido, práctica con 6 blancos en los bordes con ajuste de velocidad y
  aceleración, prueba del gesto para despertar y resumen.
- **Configuración por Bluetooth**: servicio BLE propio con los mismos comandos
  que el USB. El panel tiene "Conectar por Bluetooth" (Web Bluetooth) y
  "Conectar por USB".
- **Varios PC**: hasta 4 emparejados; al llenarse se reemplaza el que se usó
  hace más tiempo. Tarjeta "Bluetooth y emparejamiento" en el panel.
- **Sin internet**: el panel se puede instalar como app (PWA) y abre sin
  conexión; también se puede descargar como un solo archivo (`HeadMouse-panel.html`)
  para llevar en un pendrive y abrir con Chrome o Edge (desde el archivo,
  solo por USB).

**Protocolo**
- Comandos nuevos: `PAIR` (reabre 60 s el aviso; si hay un PC conectado,
  rige al desconectarse), `UNPAIR` (olvida todos los PC), `OW=0|1` (abrir el
  asistente al emparejar).
- `CFG:` agrega un campo al final: `ow`.
- Líneas nuevas: `ST:PAIR,0|1`, `ST:BONDS,<n>`, `ST:WIZARD,abierto`.
- Servicio BLE de configuración `c2a10001-5b3e-4f7a-9d61-8e4f2b6a1d30`:
  `c2a10002-…` (CMD, escribir comandos) y `c2a10003-…` (OUT, notificaciones en
  trozos de 20 bytes; `R:` va a 10 Hz por BLE).
- Reporte HID compuesto: mouse con report ID 1 y teclado con report ID 2.

**Compatibilidad**
- Cambió el formato guardado (`HM2E`): la configuración vuelve a los valores
  de fábrica al cargar la 2.2.
- Cambió la descripción HID: **hay que quitar "HeadMouse-MG24" en cada PC**
  (Configuración → Bluetooth y dispositivos) y volver a emparejarlo.
- Si falla un emparejamiento, ahora se borra solo ese PC y no todos.

**Por confirmar en la placa**
- Que Chrome o Edge en Windows puedan abrir el servicio de configuración por
  Bluetooth con la placa ya emparejada como mouse (mientras está conectada,
  la placa sigue anunciándose como "solo escaneable" para aparecer en la lista).
- Que el texto del teclado salga bien con el teclado latinoamericano.
  Con Bloq Mayús activo la ruta llegaría en mayúsculas y GitHub Pages no la
  encontraría.

---

## 2.1 — 2026-11-24

**Nuevo**
- **Aceleración**: los giros rápidos avanzan más píxeles por grado que los
  lentos. Bajo el umbral la ganancia es la normal (precisión); sobre él crece
  en línea recta hasta `1 + 3 × fuerza` veces. Permite "remar" hacia un borde:
  volver lento hacia el lado contrario y girar rápido hacia el borde.
- **Reposo automático**: si el cursor no se mueve durante N segundos se
  congela. El LED parpadea breve cada 2 s.
- **Gesto para despertar**: N giros bruscos (sobre un umbral en °/s) dentro de
  1,5 s, p. ej. un "no" rápido. Tras despertar, el cursor espera 0,7 s quieto
  para que el gesto no lo arrastre.
- Panel: tarjeta **Aceleración** (curva en vivo con el multiplicador actual) y
  tarjeta **Reposo automático** (estado, medidor del giro más rápido de los
  últimos 3 s frente al umbral, botón para dormir/despertar a mano).

**Valores de fábrica nuevos**: fuerza 1.0 · umbral de aceleración 40 °/s ·
reposo a los 30 s · despertar con 2 giros sobre 120 °/s.

**Protocolo serial**
- Comandos nuevos: `AC=` (fuerza 0–3), `AT=` (umbral °/s), `IT=` (segundos de
  reposo, 0 = nunca), `WT=` (umbral del giro brusco), `WN=` (giros 1–4),
  `SL=0|1` (forzar reposo/despertar).
- `CFG:` agrega 6 campos al final: `ac,at,it,wt,wn,reposo`.
- `R:` agrega un tercer valor: el multiplicador de aceleración.
- Línea nueva `ST:SLEEP,0|1`.

**Compatibilidad**: cambió el formato guardado (`HM2D`). Al cargar 2.1, la
configuración guardada vuelve a los valores de fábrica (los ejes de fábrica ya
corresponden al montaje en la varilla). El panel sigue funcionando con
firmware 2.0, sin las tarjetas nuevas.

---

## 2.0

**Causa del problema de v1**: v1 suponía la placa plana. En la varilla, v1
tomaba el asentir (gz) como horizontal y la oreja al hombro (gx) como
vertical; girar la cabeza (gy) no hacía nada. Medido con
`Arduino/test_ejes/test_ejes.ino`:

| Movimiento | Eje | Signo |
|---|---|---|
| Girar a la izquierda | gy | negativo |
| Mirar arriba | gz | positivo |
| Oreja derecha al hombro | gx | negativo (no se usa) |

**Cambios**
- Mapeo de ejes libre con signo; por defecto horizontal = **gy (+)**,
  vertical = **gz (−)**.
- Calibración guiada desde el panel (`CAL=H`, `CAL=V`): detecta eje y signo
  para cualquier otro montaje. Botones para invertir cada eje.
- Zona muerta suave (sin salto al salir de ella), suavizado exponencial,
  acumulador sub-píxel, ganancia en píxeles por grado.
- Corrección automática de deriva cuando la cabeza está quieta 1 s.
- Punto cero al encender que espera a que la placa esté quieta; botón
  "Resetear punto cero" (`RC`).
- Bluetooth: se conserva el emparejamiento (v1 lo borraba en cada
  desconexión); intervalo de conexión corto (7,5–15 ms); servicios Battery y
  Device Information según el ejemplo oficial de Silicon Labs.
- Lectura serial no bloqueante; configuración guardada en EEPROM 2 s después
  del último cambio.
- Panel nuevo: radar de velocidad con zona muerta, pausa, calibración guiada
  con cuenta regresiva, ajustes, diagnóstico con giroscopio crudo y registro.

---

## 1

Versión original (`Arduino/Arduino_v1.txt`). Cursor con ejes fijos pensados
para la placa plana, emparejamiento borrado al desconectar. Síntomas: el
cursor no seguía los movimientos esperados, temblor, deriva y movimiento a
saltos.
