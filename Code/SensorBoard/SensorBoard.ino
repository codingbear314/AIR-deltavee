#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <PMS.h>
#include "DFRobot_OxygenSensor.h"

class SensorData {
public:
    float pressure;
    float temperature;
    float altitude;
    float roll, pitch, yaw;
    int co2;
    int o2x100;
    int pm1, pm2_5, pm10;
};

class GlobalData {
public:
    unsigned long time = 0;
    int state = 0;
    SensorData sensorData;
    SoftwareSerial telemetry;
    SoftwareSerial dust;
    SoftwareSerial co2Serial;
    char buffer[128] = {0};

    GlobalData() : telemetry(9, 10), dust(7, 8), co2Serial(2, 3) {}

    void updateBuffer() {
        char pressure[10], temperature[10], altitude[10], roll[10], pitch[10], yaw[10];
        dtostrf(sensorData.pressure, 6, 2, pressure);
        dtostrf(sensorData.temperature, 5, 2, temperature);
        dtostrf(sensorData.altitude, 6, 2, altitude);
        dtostrf(sensorData.roll, 5, 2, roll);
        dtostrf(sensorData.pitch, 5, 2, pitch);
        dtostrf(sensorData.yaw, 5, 2, yaw);

        sprintf(buffer, "%s:%s:%s,%s:%s:%s,%d,%d,%d:%d:%d",
                pressure, temperature, altitude, roll, pitch, yaw,
                sensorData.co2, sensorData.o2x100,
                sensorData.pm1, sensorData.pm2_5, sensorData.pm10);
    }
};

GlobalData global;

class SDManager {
private:
    File flightLog;

public:
    /**
     * Flight log format:
     * DeltaVee 2024 - 1 Flight Log
     * 
     * Pressure:Temperature:Altitude,Roll:Pitch:Yaw,CO2,O2,PM1:PM2.5:PM10
     * 
     * Communication protocol:
     * SPI, 4 pin
     * Chip select: 4
     */
    void init() {
        if (!SD.begin(4)) {
            Serial.println("SD Card initialization failed!");
            return;
        }
        Serial.println("SD Card initialization done.");
        flightLog = SD.open("FlightLog.txt", FILE_WRITE);
        if (flightLog) {
            Serial.println("FlightLog.txt opened.");
            flightLog.println("DeltaVee 2024 - 1 Flight Log");
            flightLog.close();
        } else {
            Serial.println("FlightLog.txt open failed.");
        }
    }

    void write() {
        global.time = millis();
        flightLog = SD.open("FlightLog.txt", FILE_WRITE);
        if (flightLog) {
            flightLog.println(global.buffer);
            flightLog.close();
        }
    }
};

class MPU6050Sensor {
    /**
     * MPU6050 (gyro) sensor
     * 
     * Communication protocol:
     * I2C, 400 kHz
     * Address 0x68
     */
private:
    MPU6050 mpu;
    bool dmpReady = false;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

public:
    bool init() {
        mpu.initialize();
        if (!mpu.testConnection()) {
            return false;
        }
        devStatus = mpu.dmpInitialize();
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);
        if (devStatus == 0) {
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();
            mpu.setDMPEnabled(true);
            mpuIntStatus = mpu.getIntStatus();
            dmpReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize();
            return true;
        }
        return false;
    }

    void readYPR() {
        if (!dmpReady) {
            global.sensorData.roll = global.sensorData.pitch = global.sensorData.yaw = 0;
            return;
        }
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            global.sensorData.roll = ypr[2] * 180 / M_PI;
            global.sensorData.pitch = ypr[1] * 180 / M_PI;
            global.sensorData.yaw = ypr[0] * 180 / M_PI;
        }
    }
};

class BMP280Sensor {
    /**
     * BMP280 (temperature, pressure, altitude) sensor
     * 
     * Communication protocol:
     * I2C, 400 kHz
     * Address 0x76
     * 
     * Sensor data:
     * Pressure:Temperature:Altitude
     */
private:
    Adafruit_BMP280 bmp;
    Adafruit_Sensor *bmp_temp;
    Adafruit_Sensor *bmp_pressure;

public:
    void init() {
        Serial.println("Initializing BMP280");
        if (!bmp.begin(0x76)) {
            Serial.println("BMP280 initialization failed!");
            return;
        }
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_500);
        bmp_temp = bmp.getTemperatureSensor();
        bmp_pressure = bmp.getPressureSensor();
        bmp_temp->printSensorDetails();
        Serial.println("BMP280 initialization done.");
    }

    void read() {
        sensors_event_t temp_event, pressure_event;
        bmp_temp->getEvent(&temp_event);
        bmp_pressure->getEvent(&pressure_event);
        global.sensorData.temperature = temp_event.temperature;
        global.sensorData.pressure = pressure_event.pressure;
        global.sensorData.altitude = bmp.readAltitude(1006.8);
    }
};

class CO2Sensor {
    /**
     * CO2 sensor
     * 
     * Communication protocol:
     * UART, 9600 baud
     * 
     * Sensor data:
     * CO2 (ppm)
     */
private:
    const unsigned char cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    unsigned int lastRequest = 0;
    bool requested = false;

public:
    void init() {
        global.co2Serial.begin(9600);
        Serial.println("CO2 Sensor initialization done.");
    }

    void read() {
        if (!requested && millis() - lastRequest > 1000) {
            global.co2Serial.write(cmd, 9);
            lastRequest = millis();
            requested = true;
        } else if (requested && global.co2Serial.available() > 0 && millis() - lastRequest > 100) {
            unsigned char buf[9];
            int i = 0;
            while (global.co2Serial.available() > 0 && i < 9) {
                buf[i++] = global.co2Serial.read();
            }
            if (i == 9 && buf[0] == 0xFF && buf[1] == 0x86) {
                global.sensorData.co2 = buf[2] * 256 + buf[3];
            }
            requested = false;
        }
    }
};

class DustSensor {
    /**
     * Dust sensor
     * 
     * Communication protocol:
     * UART, 9600 baud
     * 
     * Sensor data:
     * PM1, PM2.5, PM10 (ug/m^3)
     */
private:
    PMS pms;
    PMS::DATA data;

public:
    DustSensor() : pms(global.dust) {}

    void init() {
        global.dust.begin(9600);
        Serial.println("Dust Sensor initialization done.");
    }

    void read() {
        if (pms.read(data)) {
            global.sensorData.pm1 = data.PM_AE_UG_1_0;
            global.sensorData.pm2_5 = data.PM_AE_UG_2_5;
            global.sensorData.pm10 = data.PM_AE_UG_10_0;
        }
    }
};

class O2Sensor {
    /**
     * Oxygen sensor
     * 
     * Communication protocol:
     * I2C, 400 kHz
     * Address 0x73
     * 
     * Sensor data:
     * O2 (%)
     */
private:
    DFRobot_OxygenSensor o2;

public:
    void init() {
        if (o2.begin(ADDRESS_3)) {
            Serial.println("O2 Sensor initialization done.");
        } else {
            Serial.println("O2 Sensor initialization failed.");
        }
    }

    void read() {
        global.sensorData.o2x100 = int(o2.getOxygenData(10) * 100.0);
    }
};

class Communication {
public:
    /**
     * Telemetry -> System
     * L - Launch
     * D - Deploy
     * R - Reset
     * T - Test
     * default - Unknown
     * 
     * System -> Telemetry
     * P - Ping
     * U - Unknown
     * Whole sensor buffer - Sensor data
     * 
     * Protocol system:
     * UART, 9600 baud, software serial (pin 9, 10)
     * 
     * Data format:
     * Pressure:Temperature:Altitude,Roll:Pitch:Yaw,CO2,O2,PM1:PM2.5:PM10
     */

    /**
     * Parachute deployment:
     * If the rocket is descending and the altitude is 7 meters below the maximum altitude,
     * deploy the parachute.
     * 
     * Each pin:
     * pin 5 - if high, tell the parachute processer to deploy
     * pin 6 - if the parachute processer is dead, override and deploy
     *  - Note, this pin will only work if and only if the parachute processer is dead
     */

    void init() {
        global.telemetry.begin(9600);
        pinMode(6, OUTPUT);
        digitalWrite(6, LOW);
        pinMode(5, OUTPUT);
        digitalWrite(5, LOW);
        Serial.println("Telemetry initialization done.");
    }

    void send() {
        global.telemetry.println(global.buffer);
    }

    void send(const char* data) {
        global.telemetry.print(data);
    }

    void send(const float& f) {
        global.telemetry.print(f);
    }

    void sendln(const char* data) {
        global.telemetry.println(data);
    }

    void control() {
        if (global.telemetry.available() > 0) {
            char c = global.telemetry.read();
            Serial.print("Received: ");
            Serial.println(c);
            switch (c) {
                case 'L':
                    global.state = 1;
                    break;
                case 'D':
                    global.state = 2;
                    digitalWrite(6, HIGH);
                    digitalWrite(5, HIGH);
                    break;
                case 'R':
                    global.state = 0;
                    digitalWrite(6, LOW);
                    digitalWrite(5, LOW);
                    break;
                case 'T':
                    global.telemetry.write('P');
                    break;
                default:
                    global.telemetry.write('U');
            }
        }
    }
};

SDManager sdManager;
MPU6050Sensor mpu6050;
BMP280Sensor bmp280;
CO2Sensor co2Sensor;
DustSensor dustSensor;
O2Sensor o2Sensor;
Communication communication;

void setup() {
    Serial.begin(9600);
    Serial.println("DeltaVee 2024 - 1");
    Serial.println("Main - 2");
    
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock to 400 kHz
    delay(3000);

    Serial.println("Initializing sensors...");
    sdManager.init();
    bmp280.init();
    if (mpu6050.init()) {
        Serial.println("MPU6050 initialization successful");
    } else {
        Serial.println("MPU6050 initialization failed");
    }
    co2Sensor.init();
    dustSensor.init();
    o2Sensor.init();
    communication.init();

    global.state = 0;
    global.time = millis();
    global.updateBuffer();

    Serial.println("Initialization complete. Waiting for launch...");
}

int maxHeight = 0;
int suspect = 0;

void loop() {
    Serial.println(global.state);
    switch (global.state) {
        case 0:
            Serial.println("Waiting for launch...");
            communication.control();
            break;
        case 1:
            bmp280.read();
            mpu6050.readYPR();
            co2Sensor.read();
            dustSensor.read();
            o2Sensor.read();
            global.updateBuffer();
            communication.control();
            communication.send();
            sdManager.write();
            Serial.print("Status : Assending");
            Serial.println(global.buffer);

            if (global.sensorData.altitude > maxHeight) {
                maxHeight = global.sensorData.altitude;
            }

            if (global.sensorData.altitude < maxHeight - 7) {
                ++suspect;
            }

            if (suspect > 10) {
                global.state = 1;
                digitalWrite(6, HIGH);
                digitalWrite(5, HIGH);
                communication.send("Max height was ");
                communication.send(maxHeight);
                communication.send(", now ");
                communication.send(global.sensorData.altitude);
                communication.sendln(" deploying.");
            }
            break;
        case 2:
            bmp280.read();
            mpu6050.readYPR();
            co2Sensor.read();
            dustSensor.read();
            o2Sensor.read();
            global.updateBuffer();
            digitalWrite(6, HIGH);
            digitalWrite(5, HIGH);
            communication.send();
            sdManager.write();
            Serial.print("Status : Descending");
            Serial.println(global.buffer);
            break;
    }
}