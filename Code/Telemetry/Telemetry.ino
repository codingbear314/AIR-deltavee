#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial Telemetry(10, 9);
SoftwareSerial LoRa(2, 3);
SoftwareSerial GPSserial(8, 7);
Adafruit_GPS GPS(&GPSserial);

void setup() {
  Telemetry.begin(9600);
  LoRa.begin(9600);
  GPSserial.begin(9600);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  GPSserial.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();
uint32_t launchTime = millis();
bool launched = false;
bool armed = false;

void loop() {
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (millis() - timer > 2000) {
    timer = millis();
    LoRa.print("T");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    LoRa.print("F");
    LoRa.print((int)GPS.fix);

    LoRa.print("P");
    LoRa.print(GPS.latitude, 4); LoRa.print(GPS.lat);
    LoRa.print(",");
    LoRa.print(GPS.longitude, 4); LoRa.println(GPS.lon);
  }

  if (Telemetry.available()) {
    LoRa.print(Telemetry.read());
  }

  if (LoRa.available()) {
    char command = LoRa.read();
    switch(command) {
      case 'A' : // Arm
        LoRa.write('C'); // Asking for comfirm
        armed = true;
        break;
      case 'L' : {
        if (!armed) break;
        digitalWrite(6, HIGH);
        launchTime = millis();
        launched = true;
        break;
      }
      case 'P':
        LoRa.write('P'); // Ping
        break;
    }
  }

  if (millis() - launchTime > 10000 && launched)
  {
    digitalWrite(6, LOW);
    launched = false;
  }
}