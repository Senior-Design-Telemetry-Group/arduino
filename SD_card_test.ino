#include <SPI.h>
#include <SD.h>

struct telemPacket {
  int RPM;
  float speed;
  float slope;
  float BV;
};

const char* fn = "test.csv";

void init_sd() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
}

void init_file() {
  File f = SD.open(fn, FILE_WRITE);
  if (f) {
    f.println("RPM,Slope,Speed,BV");
    // close the file:
    f.close();
  } else {
    Serial.println("Error opening file!");
  }

}

void dump_file(telemPacket packet) {
  File f = SD.open(fn, O_APPEND);
  if (f) {
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
    Serial.println("done.");
  } else {
    Serial.println("Error opening file!");
  }
}

void setup() {
  Serial.begin(9600);
  init_sd();

  init_file();
  telemPacket p = {3000, 28.2, 0.8, 11.99};
  dump_file(p);
}

void loop() {
}
