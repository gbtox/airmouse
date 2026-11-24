# Historial de versiones — HeadMouse MG24

Mouse de cabeza por Bluetooth (BLE HID) para una persona con parálisis
cerebral. Placa **Seeed XIAO MG24 Sense** (IMU LSM6DS3TR-C) montada en la
varilla de los lentes: USB-C hacia la oreja, cara de chips hacia afuera.
Se configura desde el panel web `index.html` por USB (Web Serial, Chrome o Edge).

| Versión | Firmware | Panel | Estado |
|---|---|---|---|
| 2.1 | `Arduino/HeadMouse/HeadMouse.ino` | `index.html` (local) | Compila; en prueba en la placa |
| 2.0 | (reemplazado por 2.1) | — | Probado: el cursor se mueve bien |
| 1 | `Arduino/Arduino_v1.txt` (respaldo) | versión publicada en GitHub Pages | Ejes incorrectos para este montaje |

Para cargar: Arduino IDE, placa *XIAO MG24*, **Herramientas → Protocol stack →
"BLE (Silabs)"**, librería *Seeed Arduino LSM6DS3* ≥ 2.0.4 (se usa 2.0.8).

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
