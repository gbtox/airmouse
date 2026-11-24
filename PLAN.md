# HeadMouse MG24 — plan de funciones

Usuario: persona con parálisis cerebral; placa en la varilla de los lentes
(USB-C hacia la oreja, cara de chips hacia afuera). Movimientos: girar
(izquierda/derecha) y asentir (arriba/abajo).

## Hecho

Detalle completo en [CHANGELOG.md](CHANGELOG.md).

- **v2.0** — mapeo de ejes medido (horizontal = gy +, vertical = gz −),
  calibración guiada, zona muerta suave, suavizado, corrección de deriva,
  Bluetooth que se reconecta solo, panel por USB.
- **v2.1** — aceleración configurable (fuerza y umbral), reposo automático
  tras N s sin uso, gesto para despertar (N giros bruscos en 1,5 s),
  controles y medidor de picos en el panel.
- **v2.2** — aviso Swift Pair en Windows (60 s desde cada encendido o hasta
  conectar), teclado que abre el asistente al emparejar un PC nuevo,
  asistente paso a paso, configuración por Bluetooth (Web Bluetooth) o USB,
  hasta 4 PC, panel instalable (PWA) y descargable para uso sin internet.

## Próximo

1. **Validar la 2.2 en la placa**: el aviso de Windows, la apertura del
   asistente con el teclado latinoamericano y la conexión del panel por
   Bluetooth. Si Web Bluetooth no logra abrir el servicio con la placa
   emparejada como mouse, el panel sigue solo por USB.
2. **Autoajuste en la placa (2.3)**: en los primeros 3 minutos de uso tras
   valores de fábrica, o con el comando `AUTO`, medir el ruido en reposo
   (zona muerta entre 1,5 y 6 °/s) y los picos de los movimientos normales
   (umbral de despertar = máx(120, pico × 1,3), tope 250 °/s). Avisa con
   `ST:AUTO,<dz>,<wt>` y se desactiva si se cambian esos valores a mano.

Otros sistemas: Android tiene Fast Pair, pero requiere registrar el modelo
en Google, así que queda para después. macOS e iOS no tienen equivalente
para mouses de otras marcas: se empareja desde Ajustes como hoy.

## Después (sin micrófono)

1. **Clic por permanencia (dwell)**: si el cursor queda quieto X ms dentro
   de un radio pequeño, hace clic izquierdo. Configurable: tiempo, radio,
   activar/desactivar. Aviso visual previo vía LED o sonido del PC.
2. **Gesto para dormir**: el mismo gesto brusco (u otro distinto) para
   congelar el cursor a voluntad, no solo por tiempo.
3. **Despertar por acelerómetro**: usar el pico de aceleración lineal
   (golpe/"cabezazo") como alternativa al giro rápido, por si los
   movimientos involuntarios tienen velocidad angular alta.
4. **Batería real** leyendo el divisor de tensión (PD3/PD4) en lugar del
   100 % fijo.
5. **Perfiles**: guardar 2–3 configuraciones (p. ej. "silla", "cama").

## Futuro con micrófono (la placa Sense trae un micrófono analógico)

- **Detección de sonido corto** (chasquido, "tsk", soplido): umbral de
  energía + duración máxima → clic o despertar. Es lo más simple y robusto.
- **Palabra breve**: reconocimiento de 1–3 palabras ("clic", "doble",
  "para") con un modelo pequeño de palabras clave en el propio chip
  (el MG24 tiene acelerador de IA). Requiere grabar muestras de la voz
  del usuario para entrenar.
- Configurar desde el panel: sensibilidad, qué acción hace cada sonido,
  y un medidor de nivel en vivo, igual que el de giros bruscos.

## Riesgos a vigilar

- Movimientos involuntarios rápidos pueden despertar el cursor o activar
  la aceleración: subir el umbral de despertar y/o pedir 3 giros.
- El reposo cuenta "sin uso" como "el cursor no se movió"; si la persona
  mira la pantalla sin moverse más de N s, entra en reposo.
- El teclado escribe en el PC por su cuenta (solo la primera vez con cada
  PC). Si el foco cambia en ese segundo, o si Bloq Mayús está activo, el
  texto puede no llegar bien. Se puede desactivar en el panel.
- Con varios PC cerca y emparejados, se conecta el primero que la encuentre.
