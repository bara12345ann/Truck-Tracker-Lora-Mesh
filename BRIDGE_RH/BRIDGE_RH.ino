// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent wierd crashes
#define RH_MESH_MAX_MESSAGE_LEN 50
#include <Preferences.h>
#include <RHMesh.h>
#include <RHRouter.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <vector>
#include <Arduino.h>
#include <mbedtls/aes.h>

// MQTT lib and var
#include <WiFi.h>
#include <PubSubClient.h>

// OLED
#include <Wire.h>
#include <Adafruit_SSD1306.h>

mbedtls_aes_context aes;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char* ssid = "Synapsis.id Branch Jogja";
const char* password = "synaps1sjogja23@";
const char* mqttServer = "mqtt.synapsis.id";
const int mqttPort = 32100;
char* mqttUser;
const char* mqttPassword = "moriwqxxsr";
const char* pubTopic;
const char* subTopic;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

String mqtt_message;

// mode 0 = sebagai MQTT publisher
// mode 1 = sebagai bridge >> ke arduino LoRa untuk di foward
#define bridge_mode 0

// security = 0 OFF
// security = 1 ON
#define security 1

// In this small artifical network of 4 nodes,
// address of the bridge
//( we send our data to, hopefully the bridge knows what to do with our data )
#define BRIDGE_ADDRESS 1

// lilygo T3 v2.1.6
// lora SX1276/8
#define LLG_SCK 5
#define LLG_MISO 19
#define LLG_MOSI 27
#define LLG_CS  18
#define LLG_RST 14
#define LLG_DI0 26

// Singleton instance of the radio driver
// slave select pin and interrupt pin, [heltec|ttgo] ESP32 Lora OLED with sx1276/8
RH_RF95 rf95(LLG_CS, LLG_DI0);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(rf95, BRIDGE_ADDRESS);

String dataIn = "", node2, dt[50], data_all;
String waktuTest;
// Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t res;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 34, 12);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  Serial.print(F("initializing node "));
  Serial.print(BRIDGE_ADDRESS);
  SPI.begin(LLG_SCK, LLG_MISO, LLG_MOSI, LLG_CS);
  if (!manager.init())
  {
    Serial.println(" init failed"); //error apabila mesh tidak dapat digunakan
  }
  else
  {
    Serial.println(" done");
  }

  rf95.setTxPower(2, false); // with false output is on PA_BOOST, power from 2 to 20 dBm, use this setting for high power demos/real usage
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

  if (bridge_mode == 0) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
      display.print(".");
      display.display();
    }
    Serial.println("Connected to WiFi");
    display.println(" ");
    display.println("Wifi Connected ");
    display.println(WiFi.localIP());
    display.display();

    mqttClient.setServer(mqttServer, mqttPort);

    while (!mqttClient.connected()) {
      String mqtt_clientId = "mesh_gateway-";
      mqtt_clientId += String(random(0xffff), HEX);

      if (mqttClient.connect("ESP8266Client", "otvjmujcuh", mqttPassword)) {
        Serial.println("Connected to MQTT broker");

        display.println(" ");
        display.print("Connected to MQTT broker");
      } else {
        Serial.print("Failed to connect to MQTT broker, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" Retrying in 5 seconds...");

        display.println(" ");
        display.print("Failed to connect to MQTT broker, rc=");
        display.print(mqttClient.state());
        display.println(" Retrying in 5 seconds...");
        delay(5000);
      }
    }

    // mqttClient.subscribe("topic/subscriber");
  } else {
    // Serial setup / i2c setup untuk komunikasi ke mikrokontroller lain
  }
  delay(2000);
  display.clearDisplay();

  // AES enkripsi
  mbedtls_aes_init(&aes);

  // Set AES key
  unsigned char key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
  // unsigned char key[16] = {0x0A, 0x05, 0x02, 0x0B, 0x04, 0x05, 0x06, 0x0C, 0x08, 0x0D, 0x01, 0x03, 0x0C, 0x07, 0x0F, 0x0E};
  // mbedtls_aes_setkey_enc(&aes, key, 128);
  mbedtls_aes_setkey_dec(&aes, key, 128);
}


void loop()
{
  if (bridge_mode == 0) {
    mqttClient.loop();
  } else {
    // none
  }

  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from))
  {
    //Serial.print("request from node n.");
    //Serial.print(from);
    //Serial.print(": ");
    //Serial.print((char*)buf);
    //Serial.print(" rssi: ");
    //Serial.println(rf95.lastRssi());
    parsingData((char*)buf);
    if (bridge_mode == 0) {
      // dilakukan konversi data string ke char sesuai format data MQTT publish
      for (int i = 0; i <= RH_MESH_MAX_MESSAGE_LEN; i++) {
        buf[i] = 0;
      }
      const char * bufferGateway = data_all.c_str();
      
      publishMessage(bufferGateway);
    } else {
      //Serial.print(data_all);
    }
  }

}

void publishMessage(const char* message) {
  const char* convertedStr = node2.c_str();
  String mqtt_clientId = "mesh_gateway-";
  mqtt_clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(mqtt_clientId.c_str(), convertedStr, mqttPassword)) {
      Serial.println(convertedStr);
      mqttClient.publish(pubTopic, message);
      Serial.println("Message published");
      mqttClient.disconnect();
    }

}

/*
void callback(char* topic, byte* payload, unsigned int length) { // digunakan apabila membutuhkan subscriber
  Serial.print("Received message on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    mqtt_message += ((char)payload[i]);
  }
  Serial.println();
  if (String(topic) == "isi/topic") {
    String mess = mqtt_message;
  }
}*/

void parsingData(const String& data) {
  bool parsing = false;
  //Serial.println(data);

  if (security == 0) {
    dataIn = data;
  } else {
    dataIn = decryptData(data);
  }

  if (dataIn.indexOf('#') != -1) {
    Serial.println("Karakter '#' ditemukan");
    parsing = true;
  }

  if (parsing) {
    parsingString();
    parsing = false;
    dataIn = "";
  }
}

String decryptData(const String& dataUncrypted) {
  // buat dekripsi
  String combine_encryted_substrings = dataUncrypted;
  int dataRawLen = (dataUncrypted.length() / 32);
  //int dataLen = ceil(dataRawLen/12);
  Serial.println(dataUncrypted);
  //Serial.println(dataRawLen);

  String raw_decryption_substrings[dataRawLen];
  String decrypted_substrings[dataRawLen];
  for (int i = 0; i < dataRawLen; i++) {
    raw_decryption_substrings[i] = combine_encryted_substrings.substring(i * 32, (i * 32) + 32);
    //Serial.println(raw_decryption_substrings[i]);
  }
  unsigned char decryptedText[16];
  unsigned char output[16];
  for (int i = 0; i < dataRawLen; i++) {
    //Serial.println(raw_decryption_substrings[i]);
    hexStringToUnsignedChar(raw_decryption_substrings[i].c_str(), output);

    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, output, decryptedText);
    //Serial.println("Decrypted String: ");
    //Serial.println((char*)decryptedText);
    decrypted_substrings[i] = String((char*)decryptedText);
    //Serial.println(decrypted_substrings[i]);
  }

  String combine_decrypted_substrings;
  for (int i = 0; i < dataRawLen; i++) {
    combine_decrypted_substrings.concat(decrypted_substrings[i]);
  }

  //Serial.print("Combined String final: ");
  Serial.println(combine_decrypted_substrings);
  return (combine_decrypted_substrings);
}

void parsingString() {
  int j = 0;
  dt[j] = "";
  String data_buf = dataIn;
  String hariF, bulanF, tahunF, jamF, menitF, detikF, satF;
  String latF, longF, altF, spdF;

  if (data_buf.length() > 0) {
    for (int i = 1; i < 100; i++) {

      //pengecekan tiap karakter dengan karakter (#) dan (,)
      if ((dataIn[i] == '*') || (dataIn[i] == ',') || (dataIn[i] == '#'))
      {
        //increment variabel j, digunakan untuk merubah index array penampung
        j++;
        dt[j] = "";     //inisialisasi variabel array dt[j]

      }
      else
      {
        //proses tampung data saat pengecekan karakter selesai.
        dt[j] = dt[j] + dataIn[i];
      }
    }
    node2 = dt[0];
    tahunF = dt[1];
    bulanF = dt[2];
    hariF = dt[3];
    jamF = dt[4];
    menitF = dt[5];
    detikF = dt[6];
    latF = dt[7];
    longF = dt[8];
    altF = dt[9];
    spdF = dt[10];
    satF = dt[11];
  }
  int16_t RssI = rf95.lastRssi();
  String string_RSSI = String(RssI);
  String waktu;
  String waktu_tgl;

  if (bulanF.length() < 2) {
    waktu_tgl += "0";
  }
  waktu_tgl += bulanF + "-";

  if (hariF.length() < 2) {
    waktu_tgl += "0";
  }
  waktu_tgl += hariF;

  if (jamF.length() < 2) {
    waktu += "0";
  }
  waktu += jamF + ":";

  if (menitF.length() < 2) {
    waktu += "0";
  }
  waktu += menitF + ":";

  if (detikF.length() < 2) {
    waktu += "0";
  }
  waktu += detikF;

  String mission_time = tahunF + "-" + waktu_tgl + " " + waktu;

  waktuTest = waktu + ", " + node2;

  data_all = "{\"node_id\": \"" + node2 + "\", \"timestamp\": \"" + mission_time + "\", \"data\": {\"latitude\": \"" + latF + "\", \"longitude\": \"" + longF + "\", \"altitude\": \"" + altF + "\", \"speed\": \"" + spdF + "\", \"sattelite\": \"" + satF + "\", \"RSSI\": \"" + string_RSSI + "\"}}";

  char prefix[] = "node/";
  char postfix[] = "/data";
  char nodeID[11];
  node2.toCharArray(nodeID, sizeof(nodeID));
  int totalLength = strlen(prefix) + strlen(nodeID) + strlen(postfix) + 1;
  pubTopic = (const char*)malloc(totalLength);
  strcpy((char*)pubTopic, prefix);
  strcat((char*)pubTopic, nodeID);
  strcat((char*)pubTopic, postfix);
  Serial.println(data_all);
  Serial.println(pubTopic);
}

void hexStringToUnsignedChar(const char* hexString, unsigned char* output) {
  size_t length = strlen(hexString);
  for (size_t i = 0, j = 0; i < length; i += 2, ++j) {
    sscanf(&hexString[i], "%2hhx", &output[j]);
  }
}
