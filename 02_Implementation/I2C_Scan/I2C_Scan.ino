#include <Wire.h>

void setup() {
  Serial.begin(115200);

  // 🔌 Inicializar I2C
  Wire.begin();           
  Wire.setClock(50000);   // 🔥 más lento = más estable

  Serial.println("\nScanner I2C listo...");
}

void loop() {
  byte error, address;
  int devices = 0;

  Serial.println("\n--- Escaneando bus I2C ---");

  for (address = 1; address < 127; address++) {

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("✅ Dispositivo en: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devices++;
    }
    else if (error == 4) {
      Serial.print("⚠️ Error en: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (devices == 0) {
    Serial.println("❌ No se encontraron dispositivos");
  } else {
    Serial.print("🔎 Total encontrados: ");
    Serial.println(devices);
  }

  Serial.println("--------------------------");

  delay(2000);
}