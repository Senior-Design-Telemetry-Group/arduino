#include <SPI.h>
#include <SD.h>
#include "Wire.h"

struct telemPacket {
  int RPM;
  float speed;
  float slope;
  float BV;
};

struct imuData {
  int16_t accelerometer_x, accelerometer_y, accelerometer_z;
  int16_t gyro_x, gyro_y, gyro_z;
  int16_t temperature;
};

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. 
const char* fn = "test.csv";

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
  data.gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  data.gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  data.gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  // Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  // Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x / 131.0));
  // Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y / 131.0));
  // Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z / 131.0));

  return data;
}
void init_sd() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
}

void init_file() {
  SD.remove(fn);
  File f = SD.open(fn, FILE_WRITE);
  if (f) {
    f.println("RPM,Slope,Speed,BV");
    // close the file:
    f.close();
    Serial.println("Initialized file.");
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
    f.print("\n");
    // close the file:
    f.close();
    Serial.println("Wrote data.");
  } else {
    Serial.println("Error opening file!");
  }
}

void send_packet_part(char* s) {
  Serial.print(s);
}
void send_packet_part(char c) {
  Serial.print(c);
}
void send_packet_part(int i) {
  Serial.print(i);
}
void send_packet_part(float f) {
  Serial.print(f);
}

void write_field(char* s, int v) {
  send_packet_part(s);
  send_packet_part('=');
  send_packet_part(v); 
  send_packet_part(';');
}
void write_field(char* s, float v) {
  send_packet_part(';');
  send_packet_part(s);
  send_packet_part('=');
  send_packet_part(v);
}

int packet_idx = 0;
void send_packet(telemPacket packet) {
  send_packet_part("TELEM");
  send_packet_part(packet_idx++);
  write_field("RPM", packet.RPM);
  write_field("Slope", packet.slope);
  write_field("BV", packet.BV);
  write_field("Speed", packet.speed);

  send_packet_part("\n");
}

void read_dummy_packets() {
  File f = SD.open("dummy.txt", FILE_READ);
  if (f) {
    char ch;
    while ((ch = f.read()) > 0) {
      if (ch == '\n') {
        // inject real data
        imuData d = read_imu();
        write_field("Slope", d.accelerometer_x);
      }
      send_packet_part(ch);
      if (ch == '\n') {
        delay(200);
      }
    }
    f.close();
  } else {
    Serial.println("Error opening dummy file!");
  }
}


void setup() {
  Serial.begin(9600);
  init_sd();
  init_imu();

  // init_file();
  // telemPacket p = {3000, 28.2, 0.8, 11.99};
  // dump_file(p);
  // while (1) {
  //   send_packet(p);
  // }
  // test_dump_file(p);
}

void loop() {
  read_dummy_packets();
}
