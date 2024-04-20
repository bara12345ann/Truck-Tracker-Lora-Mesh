#include <Preferences.h>
#include <EEPROM.h>

// change this to be the ID of your node in the mesh network
uint8_t nodeId = 4;

void setup() {
  Serial.begin(115200);
  while (!Serial) ; // Wait for serial port to be available

  Serial.println("setting nodeId...");
  Preferences preferences;
  preferences.begin("myApp", false);  // Membuka Preferences dengan namespace "myApp"

  //int data = 42;  // Data yang akan ditulis
  preferences.putInt("data", nodeId);  // Menyimpan data integer ke Preferences

  uint8_t readVal = preferences.getInt("data", 0);
  
  preferences.end();  // Menutup Preferences
  
  EEPROM.begin(512);

  EEPROM.write(0, nodeId);
  EEPROM.commit();  // Simpan perubahan
  EEPROM.end();    // Akhiri penggunaan EEPROM
  
  Serial.print(F("set nodeId = "));
  Serial.println(nodeId);

  Serial.print(F("read nodeId: "));
  Serial.println(readVal);

  if (nodeId != readVal) {
    Serial.println(F("*** FAIL ***"));
  } else {
    Serial.println(F("SUCCESS"));
  }
}

void loop() {

}
