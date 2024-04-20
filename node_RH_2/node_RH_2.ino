// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent weird crashes
#define RH_MESH_MAX_MESSAGE_LEN 50
#include <Preferences.h>
#include <RHMesh.h>
#include <RHRouter.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <math.h>
#include <mbedtls/aes.h>
mbedtls_aes_context aes;

// In this small artifical network of 4 nodes,
#define BRIDGE_ADDRESS 1  // address of the bridge ( we send our data to, hopefully the bridge knows what to do with our data )
uint8_t NODE_ADDRESS;    // address of this node

// security = 0 OFF
// security = 1 ON
#define security 0

// The TinyGPS++ object
void hitungGPS();
static void smartDelay();
int32_t jam, hari, menit, detik, bulan, tahun, sat1;
float lat1, long1, alt1, speed1;
String node_bit, payload;
char char_payload[200];

//2 = otvjmujcuh 3 = usv7jnyz7o 4 = bbyhnmeqnp
TinyGPSPlus gps;

// lilygo T3 v2.1.6
// lora SX1276/8
#define LLG_SCK 5
#define LLG_MISO 19
#define LLG_MOSI 27
#define LLG_CS  18
#define LLG_RST 14
#define LLG_DI0 26

#define LLG_LED_GRN 25

#define TXINTERVAL 10000  // delay between successive transmissions
unsigned long nextTxTime;

// Singleton instance of the radio driver
RH_RF95 rf95(LLG_CS, LLG_DI0); // slave select pin and interrupt pin, [heltec|ttgo] ESP32 Lora OLED with sx1276/8

// Class to manage message delivery and receipt, using the driver declared above
RHMesh *manager;



void packet() {
  if (NODE_ADDRESS == 2) {
    node_bit = "otvjmujcuh";
  }
  else if (NODE_ADDRESS == 3) {
    node_bit = "usv7jnyz7o";
  }
  else if (NODE_ADDRESS == 4) {
    node_bit = "bbyhnmeqnp";
  }
  payload = ("*" + node_bit + "," + String(tahun) + "," + String(bulan) + "," + String(hari) + "," + String(jam) + "," + String(menit) + "," + String(detik) + "," + String(lat1, 6) + "," + String(long1, 6) + "," + String(alt1) + "," + String(speed1) + "," + String(sat1) + "#");
  Serial.println(payload);
  if (security == 0) {
    payload.toCharArray(char_payload, sizeof(char_payload));
  }
  else {
    String encryptedData = encryptData(payload);
    encryptedData.toCharArray(char_payload, sizeof(char_payload));
  }
}

String encryptData(String dataRaw) {
  // Aes encryption
  String longString = payload;
  float dataRawLen = (dataRaw.length());
  int dataLen = ceil(dataRawLen / 12);
  Serial.print(dataLen);
  String substrings[dataLen];
  for (int i = 0; i < dataLen; i++) {
    substrings[i] = longString.substring(i * 12, (i * 12) + 12);
  }
  String encryted_substrings[8];
  unsigned char plaintext[16];
  unsigned char ciphertext[16];
  for (int i = 0; i < dataLen; i++) {
    Serial.println(substrings[i]);
    strcpy((char*)plaintext, substrings[i].c_str());
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plaintext, ciphertext);
    char encryptedString[33];
    for (int i = 0; i < 16; i++) {
      snprintf(&encryptedString[i * 2], 3, "%02X", ciphertext[i]);
    }
    Serial.println("Encrypted String: ");
    encryted_substrings[i] = encryptedString;
    Serial.println(encryted_substrings[i]);
  }
  String combine_encryted_substrings;
  for (int i = 0; i < dataLen; i++) {
    combine_encryted_substrings.concat(encryted_substrings[i]);
  }

  Serial.print("Combined String: ");
  Serial.println(combine_encryted_substrings);
  return (combine_encryted_substrings);
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 34, 12); // Serial GPS
  Preferences preferences;
  preferences.begin("myApp", false);  // Membuka Preferences dengan namespace "myApp"

  NODE_ADDRESS = preferences.getInt("data", 0);

  preferences.end();  // Menutup Preferences
  Serial.print(F("initializing node "));
  Serial.print(NODE_ADDRESS);
  SPI.begin(LLG_SCK, LLG_MISO, LLG_MOSI, LLG_CS);

  pinMode(LLG_RST, OUTPUT);
  digitalWrite(LLG_RST, HIGH);

  manager = new RHMesh(rf95, NODE_ADDRESS);

  if (!manager->init())
  {
    Serial.println(" init failed");
  }
  else
  {
    Serial.println(" done"); // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
  }

  if (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  rf95.setTxPower(10, false); // with false output is on PA_BOOST, power from 2 to 20 dBm, use this setting for high power demos/real usage
  //rf95.setTxPower(1, true); // true output is on RFO, power from 0 to 15 dBm, use this setting for low power demos ( does not work on lilygo lora32 )
  rf95.setFrequency(868.0);
  rf95.setCADTimeout(500);

  // long range configuration requires for on-air time
  boolean longRange = false;
  if (longRange)
  {
    // custom configuration
    RH_RF95::ModemConfig modem_config = {
      0x78, // Reg 0x1D: BW=125kHz, Coding=4/8, Header=explicit
      0xC4, // Reg 0x1E: Spread=4096chips/symbol, CRC=enable
      0x08  // Reg 0x26: LowDataRate=On, Agc=Off.  0x0C is LowDataRate=ON, ACG=ON
    };
    rf95.setModemRegisters(&modem_config);
  }
  else
  {
    // Predefined configurations( bandwidth, coding rate, spread factor ):
    // Bw125Cr45Sf128     Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
    // Bw500Cr45Sf128     Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
    // Bw31_25Cr48Sf512   Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
    // Bw125Cr48Sf4096    Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
    // Bw125Cr45Sf2048    Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range
    if (!rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128))
    {
      Serial.println(F("set config failed"));
    }
  }
  Serial.println("RF95 ready");
  nextTxTime = millis();
  // enkripsi
  mbedtls_aes_init(&aes);

  // Set AES key
  unsigned char key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
  // unsigned char key[16] = {0x0A, 0x05, 0x02, 0x0B, 0x04, 0x05, 0x06, 0x0C, 0x08, 0x0D, 0x01, 0x03, 0x0C, 0x07, 0x0F, 0x0E};
  mbedtls_aes_setkey_enc(&aes, key, 128);
}


// Dont put this on the stack:
char buf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t res;

void loop()
{
  // send message every TXINTERVAL millisecs
  hitungGPS();
  if (millis() > nextTxTime)
  {
    nextTxTime += TXINTERVAL;
    packet();
    Serial.print("Sending to bridge n.");
    Serial.print(BRIDGE_ADDRESS);
    Serial.print(" res=");

    // Send a message to a rf95_mesh_server
    // A route to the destination will be automatically discovered.
    res = manager->sendtoWait((uint8_t *)char_payload, strlen(char_payload), BRIDGE_ADDRESS);
    Serial.println(res);
    if (res == RH_ROUTER_ERROR_NONE)
    {
      // Data has been reliably delivered to the next node.
      // now we do...
    }
    else
    {
      // Data not delivered to the next node.
      Serial.println("sendtoWait failed. Are the bridge/intermediate mesh nodes running?");
    }
  }

  // radio needs to stay always in receive mode ( to process/forward messages )
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager->recvfromAck((uint8_t *)buf, &len, &from))
  {
    Serial.print("message from node n.");
    Serial.print(from);
    Serial.print(": ");
    Serial.print((char*)buf);
    Serial.print(" rssi: ");
    Serial.println(rf95.lastRssi());
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

void hitungGPS() {
  sat1 = gps.satellites.value();
  speed1 = gps.speed.kmph();
  lat1 = gps.location.lat();
  long1 = gps.location.lng();
  alt1 = gps.altitude.meters();
  jam = gps.time.hour() + 0;
  hari = gps.date.day();
  menit = gps.time.minute();
  detik = gps.time.second();
  bulan = gps.date.month();
  tahun = gps.date.year();
  if (jam >= 24) { // ini akan aktif jika offset jam di tambah, semisal di indo +7 (pada code +0)
    jam = gps.time.hour() + 0 - 24; // 0 adalah UTC_OFFSET
    hari = gps.date.day() + 1;
  }
  //dataGPS = ("*" + node_bit + "," + String(hari) + "," + String(bulan) + "," + String(tahun) + "," + String(jam) + "," + String(menit) + "," + String(detik) + "," + String(lat1 * 1000000, 0) + "," + String(long1 * 1000000, 0) + "," + String(alt1 * 100, 0) + "," + String(speed1 * 100, 0) + "," + String(sat1) + "#");
  smartDelay(0);
}
