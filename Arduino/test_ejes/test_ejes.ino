// Prueba de ejes del giroscopio - XIAO MG24 Sense (LSM6DS3TR-C, 0x6A)
// Sin Bluetooth: en Herramientas > Protocol stack elige "None".
// Abre el Serial Plotter (o Monitor Serie) a 115200.
//
// Procedimiento, con la placa ya montada en la varilla de los lentes:
//   1. Deja la cabeza quieta 3 s (ves el "ruido" de base y el offset).
//   2. Gira la cabeza a la IZQUIERDA y vuelve (3 veces).
//   3. Asiente: mira ARRIBA y vuelve (3 veces).
//   4. Inclina la oreja derecha hacia el HOMBRO derecho y vuelve (3 veces).
// Anota que curva (gx, gy o gz) se dispara en cada paso y si sube (+) o baja (-).

#include <LSM6DS3.h>
#include <Wire.h>

LSM6DS3 imu(I2C_MODE, 0x6A);
float ox = 0, oy = 0, oz = 0;

void setup() {
  pinMode(PD5, OUTPUT);       // alimentacion del IMU
  digitalWrite(PD5, HIGH);
  delay(300);

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  if (imu.begin() != 0) {
    Serial.println("ERROR: IMU no responde");
    while (true) {}
  }

  // Offset con la placa quieta
  delay(1000);
  const int n = 200;
  for (int i = 0; i < n; i++) {
    ox += imu.readFloatGyroX();
    oy += imu.readFloatGyroY();
    oz += imu.readFloatGyroZ();
    delay(5);
  }
  ox /= n; oy /= n; oz /= n;

  Serial.print("# offset dps: ");
  Serial.print(ox, 2); Serial.print(" ");
  Serial.print(oy, 2); Serial.print(" ");
  Serial.println(oz, 2);
  Serial.print("# gravedad g: ax=");
  Serial.print(imu.readFloatAccelX(), 2); Serial.print(" ay=");
  Serial.print(imu.readFloatAccelY(), 2); Serial.print(" az=");
  Serial.println(imu.readFloatAccelZ(), 2);
}

void loop() {
  static unsigned long t = 0;
  if (millis() - t < 20) return;   // 50 Hz
  t = millis();

  Serial.print("gx:"); Serial.print(imu.readFloatGyroX() - ox, 1);
  Serial.print(" gy:"); Serial.print(imu.readFloatGyroY() - oy, 1);
  Serial.print(" gz:"); Serial.println(imu.readFloatGyroZ() - oz, 1);
}
