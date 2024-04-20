
#define IOTEXPERIMENTER

#include <WiFi.h>              // Built-in
#include <SPI.h>
#include <Wire.h>
#ifdef IOTEXPERIMENTER
#include <Adafruit_GFX.h>
//#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#endif
#include <PubSubClient.h>

#ifdef IOTEXPERIMENTER
#define OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define LED_PIN 15
//Adafruit_NeoPixel led = Adafruit_NeoPixel(1, LED_PIN, NEO_RGB + NEO_KHZ800);
#endif

const char* ssid = "Synapsis CCTV";
const char* password = "synaps1s21";
const char* mqtt_server = "test.mosquitto.org";
int mqtt_port = 1883;
const char* mqtt_username = "";
const char* mqtt_password = "";
const char* dataTopic = "node/otvjmujcuh/data";

WiFiClient espClient;
PubSubClient mqtt_client(espClient);
char data[128];

// Connect to MQTT broker
void mqtt_connect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    log("Connecting to MQTT...");
    // Attempt to connect
    String mqtt_clientId = "mesh_gateway-";
    mqtt_clientId += String(random(0xffff), HEX);
    if (mqtt_client.connect(mqtt_clientId.c_str(), mqtt_username, mqtt_password)) {
      log("connected");
    } else {
      log("failed, rc=", false);
      log(mqtt_client.state());
      delay(2000);
    }
  }
}

void setup()   {
  pinMode(2, OUTPUT); // ESP8266 LED
  digitalWrite(2, LOW);
  delay(200);
  digitalWrite(2, HIGH);

  Serial.begin(115200);
  while (!Serial);

#ifdef IOTEXPERIMENTER
  //led.begin();
  //led.show(); // Initialize all pixels to 'off'

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 36);
#endif

  log("Connecting to ");
  log(ssid, false);
  log("...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    log(".", false);
  }
  log("");
  log("connected");

  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_connect();
}

void log(const char *s) {
#ifdef OLED
  if (display.getCursorY() > 58) {
    display.clearDisplay();
    display.setCursor(0, 0);
  }
  display.println(s);
  display.display(); delay(1);
#endif
}

void log(const char *s, boolean newline) {
  if (newline) {
    return log(s);
  }
#ifdef OLED
  if (display.getCursorY() > 58) {
    display.clearDisplay();
    display.setCursor(0, 0);
  }
  display.print(s);
  display.display(); delay(1);
#endif
}


void loop() {
  if (!mqtt_client.connected()) {
    mqtt_connect();
  }
  mqtt_client.loop();

  delay(50); // Give the ESP time to handle network.

  if (Serial.available()) {
    Serial.setTimeout(100);
    String s = Serial.readStringUntil('\n');
    log(s.c_str());
    if (s.startsWith("node: ")) {
      String data;
      int end = s.indexOf('\r');
      if (end > 0) {
        data = s.substring(6, end);
      } else {
        data = s.substring(6);
      }
      mqtt_client.publish(dataTopic, data.c_str());
    }
  }
}
