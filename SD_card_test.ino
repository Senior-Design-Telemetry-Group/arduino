#include <SPI.h>
#include <SD.h>
#include "Wire.h"
#include <Adafruit_GPS.h>

#define DispSerial Serial1
#define LoRaSerial Serial2
#define GPSSerial Serial3

#define LOG_PACKETS

Adafruit_GPS GPS(&GPSSerial);

struct telemPacket {
  int RPM;
  float speed;
  float slope;
  float BV;
  float lon;
  float lat;
  bool gps_valid;
};

struct imuData {
  int16_t accelerometer_x, accelerometer_y, accelerometer_z;
  float gyro_x, gyro_y, gyro_z;
  int16_t temperature;
};

struct GPSData {
  bool valid;
  float lon;
  float lat;
};

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. 
const char* fn = "test.csv";

const int LORA_ADDR = 101;
const int FIELD_ADDR = 100;

const int LORA_SF  = 7;
const int LORA_BW  = 9;
const int LORA_CR  = 4;
const int LORA_PRE = 12;

void init_imu() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

imuData read_imu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  imuData data;
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  data.accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  data.accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  data.accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  data.temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  data.gyro_x = (Wire.read()<<8 | Wire.read()) / 131.0; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  data.gyro_y = (Wire.read()<<8 | Wire.read()) / 131.0; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  data.gyro_z = (Wire.read()<<8 | Wire.read()) / 131.0; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  // Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  // Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x / 131.0));
  // Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y / 131.0));
  // Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z / 131.0));

  return data;
}

void init_gps() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
}

GPSData read_gps() {
  GPSData data;

  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  // if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum
  if (GPS.newNMEAreceived()) {
    // Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;
  }
  data.valid = false;
  if (GPS.fix) {
    data.valid = true;
    data.lat = GPS.latitude;
    data.lon = GPS.longitude;
    // Serial.print("Location: ");
    // Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    // Serial.print(", ");
    // Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    // Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    // Serial.print("Angle: "); Serial.println(GPS.angle);
    //Serial.print("Altitude: "); Serial.println(GPS.altitude);
    //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    //Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
  }
  return data;
}

void init_sd() {
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
}

void init_file() {
  SD.remove(fn);
  File f = SD.open(fn, FILE_WRITE);
  if (f) {
    f.println("RPM,Slope,Speed,BV,Lat,Lon");
    // close the file:
    f.close();
  } else {
    Serial.println("Error opening file!");
  }
}

void dump_file(telemPacket packet) {
  File f = SD.open(fn, FILE_WRITE);
  if (f) {
    f.seek(EOF);
    f.print(packet.RPM);
    f.print(",");
    f.print(packet.slope);
    f.print(",");
    f.print(packet.speed);
    f.print(",");
    f.print(packet.BV);
    if (packet.gps_valid) {
      f.print(",");
      f.print(packet.lat);
      f.print(",");
      f.print(packet.lon);
    } else {
      f.print(",");
      f.print(",");
    }
    f.print("\n");
    // close the file:
    f.close();
  } else {
    Serial.println("Error opening file!");
  }
}

char packet_buffer[241];
int packet_length = 0;
void send_packet_part(char* s) {
  strcpy(packet_buffer + packet_length, s);
  packet_length += DispSerial.print(s);
  // Serial.print(s);
}
void send_packet_part(char c) {
  packet_buffer[packet_length] = c;
  packet_length += DispSerial.print(c);
  // Serial.print(c);
}

void send_packet_part(int i) {
  sprintf(packet_buffer + packet_length, "%d", i);
  packet_length += DispSerial.print(i);
  // Serial.print(i);
}
void send_packet_part(float f, int prec) {
  dtostrf(f, 2, prec, packet_buffer+packet_length);
  // sprintf(packet_buffer + packet_length, "%f", f);
  packet_length += DispSerial.print(f);
  // Serial.println(f);
}
void send_packet_part(float f) {
  send_packet_part(f, 3);
}

void start_packet() {
  packet_length = 0;
}
void finish_packet() {
  packet_buffer[packet_length] = 0;
  LoRaSerial.print("AT+SEND=");
  LoRaSerial.print(FIELD_ADDR);
  LoRaSerial.print(",");
  LoRaSerial.print(packet_length);
  LoRaSerial.print(",");
  LoRaSerial.print(packet_buffer);
  LoRaSerial.print("\r\n");
#ifdef LOG_PACKETS
  Serial.println("New Packet:");
  Serial.println(packet_buffer);
#endif
  // Read back +OK
  char buffer[8];
  LoRaSerial.readBytes(buffer, 5); // +OK\r\n
  buffer[5] = 0;
  Serial.println(buffer);
}

void write_field(char* s, int v) {
  send_packet_part(';');
  send_packet_part(s);
  send_packet_part('=');
  send_packet_part(v); 
}
void write_field(char* s, float v, int prec) {
  send_packet_part(';');
  send_packet_part(s);
  send_packet_part('=');
  send_packet_part(v, prec);
}
void write_field(char* s, float v) {
  write_field(s, v, 3);
}

int packet_idx = 0;
void send_packet(telemPacket packet) {
  start_packet();
  send_packet_part("TELEM");
  send_packet_part(packet_idx++);
  write_field("RPM", packet.RPM);
  write_field("Slope", packet.slope);
  write_field("BV", packet.BV);
  write_field("Speed", packet.speed);
  if (packet.gps_valid) {
    write_field("LAT", packet.lat, 8);
    write_field("LON", packet.lon, 8);
  }

  finish_packet();
}

void read_dummy_packets() {
  File f = SD.open("dummy.txt", FILE_READ);
  if (f) {
    char ch;
    start_packet();
    while ((ch = f.read()) > 0) {
      if (ch == '\n') {
        // inject real data
        imuData d = read_imu();
        write_field("Slope", d.accelerometer_y);
      }
      if (ch == '\n') {
        finish_packet();
        delay(200);
        start_packet();
      } else {
        send_packet_part(ch);
      }
    }
    f.close();
  } else {
    Serial.println("Error opening dummy file!");
  }
}

telemPacket poll_data() {
  telemPacket p;
  // IMU data
  imuData d = read_imu();
  p.slope = d.accelerometer_y;
  // GPS data
  GPSData g = read_gps();
  p.gps_valid = g.valid;
  p.lat = g.lat;
  p.lon = g.lon;
  // Placeholders
  p.BV = 12;
  p.RPM = 3000;
  p.speed = 10;
  return p;
}

void init_lora() {
  Serial.println("Resetting Lora");
  LoRaSerial.print("AT+RESET\r\n");
  char buffer[100];
  LoRaSerial.readBytes(buffer, 18); // +RESET\r\n+READY\r\n
  buffer[16] = 0;
  Serial.println(buffer);
  Serial.println("Setting Lora Address");
  LoRaSerial.print("AT+ADDRESS=");
  LoRaSerial.print(LORA_ADDR);
  LoRaSerial.print("\r\n");
  LoRaSerial.readBytes(buffer, 5); // +OK\r\n
  buffer[5] = 0;
  Serial.println(buffer);
  LoRaSerial.print("AT+CRFOP=");
  LoRaSerial.print(10);
  LoRaSerial.print("\r\n");
  LoRaSerial.readBytes(buffer, 5); // +OK\r\n
  buffer[5] = 0;
  Serial.println(buffer);

  LoRaSerial.print("AT+PARAMETER=");
  LoRaSerial.print(LORA_SF);
  LoRaSerial.print(",");
  LoRaSerial.print(LORA_BW);
  LoRaSerial.print(",");
  LoRaSerial.print(LORA_CR);
  LoRaSerial.print(",");
  LoRaSerial.print(LORA_PRE);
  LoRaSerial.print("\r\n");
  LoRaSerial.readBytes(buffer, 5); // +OK\r\n
  buffer[5] = 0;
  Serial.println(buffer);
}

void setup() {
  Serial.begin(57600);
  DispSerial.begin(57600);
  LoRaSerial.begin(115200);
  init_sd();
  init_imu();
  init_lora();
  // init_gps();

  init_file();
  // telemPacket p = {3000, 28.2, 0.8, 11.99};
  // dump_file(p);
  // while (1) {
  //   send_packet(p);
  // }
  // test_dump_file(p);
}

void loop() {
  telemPacket p = poll_data();
  send_packet(p);
  dump_file(p);
  // read_dummy_packets();
  // Serial.println("New section!");
}
