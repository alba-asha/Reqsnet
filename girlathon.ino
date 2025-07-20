#include <DHT.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // RX2 (GPIO16), TX2 (GPIO17)

#define SOUND_SENSOR A0 // GPIO36
#define VIBRATION_SENSOR 33
#define BUTTON_PIN 26
#define LED_PIN 2
void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  dht.begin();

  pinMode(SOUND_SENSOR, INPUT);
  pinMode(VIBRATION_SENSOR, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("ResQNet Initializing...");
}

void loop() {
  // put your main code here, to run repeatedly:
float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print(" C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  }

  // GPS Data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      Serial.print("Location: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", ");
      Serial.println(gps.location.lng(), 6);
    }
  }

  // Sound Sensor Reading
  int soundLevel = analogRead(SOUND_SENSOR);
  Serial.print("Sound Level: ");
  Serial.println(soundLevel);

  // Vibration Sensor
  int vibration = digitalRead(VIBRATION_SENSOR);
  Serial.print("Vibration: ");
  Serial.println(vibration == HIGH ? "Detected" : "None");

  // Button to trigger SOS (LED ON)
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("SOS Triggered!");
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
  }

  delay(1000);
}
