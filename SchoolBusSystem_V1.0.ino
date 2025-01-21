/*
 * Σκοπός: Ανάπτυξη συστήματος παρακολούθησης σχολικών λεωφορείων.
 * Υλικά: ESP32-CAM, MPU6050, GPS NEO 6M, Mini A6 GSM, OLED 1.3'', 3 LEDs, Buzzer, Push Button.
 * Λειτουργίες: Παρακολούθηση σε πραγματικό χρόνο, ανίχνευση ατυχημάτων, παραβάσεις και ειδοποιήσεις.
 MPU6050 (I2C πρωτόκολλο)

    SCL: GPIO 14
    SDA: GPIO 15

OLED Display (I2C πρωτόκολλο)

    SCL: GPIO 14
    SDA: GPIO 15

LEDs

    Κόκκινο LED: GPIO 33 LED_BUILTIN

Buzzer

    GPIO GPIO 33

Push Button

    GPIO 16

ESP32-CAM Camera Pins

    D0 έως D7: GPIOs 5, 18, 19, 21, 36, 39, 34, 35
    XCLK: GPIO 0
    PCLK: GPIO 22
    VSYNC: GPIO 25
    HREF: GPIO 23
    SIOD (SDA): GPIO 26
    SIOC (SCL): GPIO 27

GPS Module (Software Serial)

    RX: GPIO 12
    TX: GPIO 13

GSM Module (Software Serial)

    RX: GPIO 2
    TX: GPIO 4
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <esp_camera.h>

// WiFi και MQTT στοιχεία
const char* ssid = "YourSSID"; // Όνομα WiFi δικτύου
const char* password = "YourPassword"; // Κωδικός WiFi δικτύου
const char* mqtt_server = "192.168.1.100"; // Διεύθυνση MQTT broker
const int mqtt_port = 1883; // Πόρτα MQTT broker
const char* mqtt_topic = "school_bus/data"; // Θέμα MQTT για αποστολή δεδομένων

WiFiClient espClient;
PubSubClient client(espClient);

// GPS Module
TinyGPSPlus gps; // Αντικείμενο GPS για επεξεργασία δεδομένων
SoftwareSerial gpsSerial(12, 13); // RX, TX του GPS NEO 6M

// GSM Module
SoftwareSerial gsmSerial(2, 4); // RX, TX του GSM A6

String server = "http://yourserver.com/upload"; // Διεύθυνση διακομιστή για αποστολή δεδομένων

// MPU6050
Adafruit_MPU6050 mpu; // Αντικείμενο για την επικοινωνία με το MPU6050

// OLED Display
#define SCREEN_WIDTH 128 // Πλάτος OLED οθόνης
#define SCREEN_HEIGHT 32 // Ύψος OLED οθόνης
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// LEDs και Buzzer

#define LED_RED LED_BUILTIN // Κόκκινο LED για κρίσιμες ειδοποιήσεις
#define BUZZER 33 // Buzzer για ήχο ειδοποιήσεων

// Push Button
#define BUTTON_PIN 16 // Κουμπί για χειροκίνητη ενεργοποίηση

// Εικόνα από ESP32-CAM
#define PWDN_GPIO_NUM     -1 // GPIO για ενεργοποίηση/απενεργοποίηση κάμερας
#define RESET_GPIO_NUM    -1 // GPIO για reset κάμερας
#define XCLK_GPIO_NUM      0 // GPIO για εξωτερικό clock
#define SIOD_GPIO_NUM     26 // GPIO για SDA του SCCB
#define SIOC_GPIO_NUM     27 // GPIO για SCL του SCCB
#define Y9_GPIO_NUM       35 // GPIO για δεδομένα εικόνας
#define Y8_GPIO_NUM       34 // GPIO για δεδομένα εικόνας
#define Y7_GPIO_NUM       39 // GPIO για δεδομένα εικόνας
#define Y6_GPIO_NUM       36 // GPIO για δεδομένα εικόνας
#define Y5_GPIO_NUM       21 // GPIO για δεδομένα εικόνας
#define Y4_GPIO_NUM       19 // GPIO για δεδομένα εικόνας
#define Y3_GPIO_NUM       18 // GPIO για δεδομένα εικόνας
#define Y2_GPIO_NUM        5 // GPIO για δεδομένα εικόνας
#define VSYNC_GPIO_NUM    25 // GPIO για VSYNC σήμα
#define HREF_GPIO_NUM     23 // GPIO για HREF σήμα
#define PCLK_GPIO_NUM     22 // GPIO για PCLK σήμα

void setupCamera() {
  // Διαμόρφωση ρυθμίσεων της κάμερας
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; // Κανάλι LEDC για PWM
  config.ledc_timer = LEDC_TIMER_0; // Χρονόμετρο LEDC
  config.pin_d0 = Y2_GPIO_NUM; // Ρύθμιση GPIO για δεδομένα
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; // Συχνότητα XCLK
  config.pixel_format = PIXFORMAT_JPEG; // Μορφή εικόνας JPEG

  // Εάν υπάρχει διαθέσιμη μνήμη PSRAM
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // Μέγεθος καρέ UXGA (1600x1200)
    config.jpeg_quality = 10; // Ποιότητα JPEG
    config.fb_count = 2; // Αριθμός frame buffers
  } else {
    config.frame_size = FRAMESIZE_SVGA; // Μέγεθος καρέ SVGA (800x600)
    config.jpeg_quality = 12; // Ποιότητα JPEG
    config.fb_count = 1; // Ένας frame buffer
  }

  // Ενεργοποίηση της κάμερας
  esp_camera_init(&config);
}

void setupWiFi() {
  // Σύνδεση στο WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); // Αναμονή μέχρι να συνδεθεί
  }
}

void setupMQTT() {
	client.setServer(mqtt_server, mqtt_port);
		int attempts = 0;
		while (!client.connected() && attempts < 5) {
			if (client.connect("ESP32-CAM")) {
				break;
			}
			delay(1000);
			attempts++;
		}
}

void sendGSMData(String data) {
  // Αποστολή δεδομένων μέσω GSM GPRS
  gsmSerial.println("AT+HTTPINIT"); // Αρχικοποίηση HTTP υπηρεσίας
  delay(1000);
  gsmSerial.println("AT+HTTPPARA=\"URL\",\"" + server + "\""); // Ορισμός URL
  delay(1000);
  gsmSerial.println("AT+HTTPDATA=" + String(data.length()) + ",10000"); // Προετοιμασία για αποστολή δεδομένων
  delay(1000);
  gsmSerial.print(data); // Αποστολή δεδομένων
  delay(1000);
  gsmSerial.println("AT+HTTPACTION=1"); // Εκκίνηση HTTP POST
  delay(1000);
  gsmSerial.println("AT+HTTPTERM"); // Τερματισμός HTTP υπηρεσίας
  delay(1000);
}

void sendGSMImage(camera_fb_t *fb) {
  // Αποστολή εικόνας μέσω GSM GPRS
  gsmSerial.println("AT+HTTPINIT");
  delay(1000);
  gsmSerial.println("AT+HTTPPARA=\"URL\",\"" + server + "\"");
  delay(1000);
  gsmSerial.println("AT+HTTPDATA=" + String(fb->len) + ",10000");
  delay(1000);
  gsmSerial.write(fb->buf, fb->len); // Αποστολή δεδομένων εικόνας
  delay(1000);
  gsmSerial.println("AT+HTTPACTION=1");
  delay(1000);
  gsmSerial.println("AT+HTTPTERM");
  delay(1000);
}

void setup() {
  // Ρύθμιση σειριακής επικοινωνίας για debug
  Serial.begin(115200);

  // Ρύθμιση GSM module
  gsmSerial.begin(9600);

  // Ενεργοποίηση GPS
  gpsSerial.begin(9600);

  // Ρύθμιση OLED οθόνης
  if (!display.begin(SSD1306_I2C_ADDRESS, 0x3C)) {
    while (1); // Αν η OLED δεν λειτουργεί, σταματάμε το πρόγραμμα
  }
  display.clearDisplay();

  // Ρύθμιση MPU6050
  if (!mpu.begin()) {
    while (1); // Αν η MPU6050 δεν λειτουργεί, σταματάμε το πρόγραμμα
  }

  // Ρύθμιση LEDs και Buzzer
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Ρύθμιση Push Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Ρύθμιση κάμερας
  setupCamera();

  // Σύνδεση σε WiFi και MQTT
  setupWiFi();
  setupMQTT();
}

void loop() {
  // Έλεγχος σύνδεσης MQTT
  if (!client.connected()) {
    setupMQTT(); // Επανασύνδεση εάν αποσυνδεθεί
  }
  client.loop(); // Επεξεργασία εισερχόμενων μηνυμάτων
  
  // Έλεγχος κατάστασης κουμπιού
  static unsigned long buttonPressTime = 0;
  static bool isButtonPressed = false;

  if (digitalRead(BUTTON_PIN) == LOW) { // Το κουμπί πατήθηκε
    if (!isButtonPressed) {
      isButtonPressed = true;
      buttonPressTime = millis(); // Καταγραφή χρόνου που πατήθηκε το κουμπί
    } else if (millis() - buttonPressTime > 3000) { // Πατημένο για 3 δευτερόλεπτα
      // Επαναφορά ειδοποιήσεων
      digitalWrite(LED_RED, LOW);
      digitalWrite(BUZZER, LOW);
      client.publish(mqtt_topic, "System reset to normal operation.");
      isButtonPressed = false;
    }
  } else { // Το κουμπί απελευθερώθηκε
    if (isButtonPressed) {
      isButtonPressed = false;
      if (millis() - buttonPressTime < 3000) { // Σύντομο πάτημα
        // Αποστολή συναγερμού
        digitalWrite(LED_RED, HIGH);
        digitalWrite(BUZZER, HIGH);
        client.publish(mqtt_topic, "Manual alert triggered!");
        sendGSMData("Manual alert triggered!");
        delay(1000); // Διατήρηση συναγερμού για 1 δευτερόλεπτο
        digitalWrite(LED_RED, LOW);
        digitalWrite(BUZZER, LOW);
      }
    }
  }

  // Ενημέρωση δεδομένων GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
	if (gps.location.isValid()) {	// Έλεγχος εγκυρότητας
		String location = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
		client.publish(mqtt_topic, location.c_str()); // Αποστολή μέσω MQTT
		sendGSMData(location); // Αποστολή μέσω GSM GPRS
	}
    if (gps.location.isUpdated()) {
      // Δημιουργία μηνύματος με τις συντεταγμένες
      String location = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);

      client.publish(mqtt_topic, location.c_str()); // Αποστολή μέσω MQTT
      digitalWrite(LED_YELLOW, LOW);
    }
  }

  // Ανίχνευση επιτάχυνσης με MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if (a.acceleration.x > 3 || a.acceleration.y > 3 || a.acceleration.z > 3) {
    // Ενεργοποίηση συναγερμού αν εντοπιστεί ατύχημα
    digitalWrite(LED_RED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(1000);
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER, LOW);
  }

  // Λήψη και αποστολή εικόνας από την κάμερα
  camera_fb_t *fb = esp_camera_fb_get();
	if (fb != nullptr) {
		client.publish("school_bus/image", (const char *)fb->buf, fb->len);
		sendGSMImage(fb);
		esp_camera_fb_return(fb);
	} else {
		Serial.println("Camera capture failed!");
	}

  delay(5000); // Αναμονή 5 δευτερολέπτων πριν τον επόμενο κύκλο
}
