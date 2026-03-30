#include <Arduino.h>
#include <EEPROM.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

struct imuData {
  int16_t acc_x, acc_y, acc_z;
  int16_t mag_x, mag_y, mag_z;
  int16_t gyr_x, gyr_y, gyr_z;
  int16_t eul_heading, eul_roll, eul_pitch;
  int16_t qua_w, qua_x, qua_y, qua_z;
  int16_t lia_x, lia_y, lia_z;
  int16_t grv_x, grv_y, grv_z;
  int8_t temp;
  uint8_t calib_stat;
};

template <uint8_t N, class input_t = uint16_t, class sum_t = uint32_t>
class SMA {
 public:
  input_t operator()(input_t input) {
    sum -= previousInputs[index];
    sum += input;
    previousInputs[index] = input;
    if (++index == N) {
      index = 0;
    }
    if (input == 0.0) {
      return 0.0;
    }
    return (sum + (N / 2)) / N;
  }

 private:
  uint8_t index = 0;
  input_t previousInputs[N] = {};
  sum_t sum = 0;
};

int SLAVE_ADDRESS = 0x72;

uint8_t BNO_ADDR = 0x28;
uint8_t ACC_DATA_X_LSB = 0x08;
uint8_t CALIB_STAT = 0x35;
uint8_t OPR_MODE = 0x3D;
uint8_t NDOF = 0x0C;

imuData imu;

int FLW = 0;
int FRW = 0;
int BLW = 0;
int BRW = 0;

int FLD = 0;
int FRD = 0;
int BLD = 0;
int BRD = 0;

int TRR = 0;
int TRL = 0;
int TLR = 0;
int TLL = 0;

int pwmPin_R = 14;
int dirPin_R = 20;
int pwmPin_L = 23;
int dirPin_L = 22;

unsigned int addressFLW = 0;
unsigned int addressFRW = 2;
unsigned int addressBLW = 4;
unsigned int addressBRW = 6;

unsigned int addressFLD = 8;
unsigned int addressFRD = 10;
unsigned int addressBLD = 12;
unsigned int addressBRD = 14;

unsigned int addressTRR = 16;
unsigned int addressTRL = 18;
unsigned int addressTLR = 20;
unsigned int addressTLL = 22;

int encoderPin_1_L = 7;
int encoderPin_2_L = 8;

int encoderPin_1_R = 5;
int encoderPin_2_R = 4;

volatile int lastEncoded_L = 0;
volatile long encoderValue_L = 0;
long lastencoderValue_L = 0;
int lastMSB_L = 0;
int lastLSB_L = 0;

volatile int lastEncoded_R = 0;
volatile long encoderValue_R = 0;
long lastencoderValue_R = 0;
int lastMSB_R = 0;
int lastLSB_R = 0;

int delayBrake = 50;
int pin_Emergency = 15;

bool printAlter = false;
bool emergencyAlter = true;
bool systemCounter = false;

char data = '0';
int rpmAlter_T = 0;
int rpmAlter = 0;
int _dirData = 0;

float timeConstant = 100;
float startTime, elaspedTime = 0, currentTime;

float timeConstantControlCounter = 1000;
float startTimeControlCounter, elaspedTimeControlCounter = 0, currentTimeControlCounter;

const float ENCODER_COUNTS_PER_REV = 4000.0f;
const float RPM_CALIBRATION_LEFT = 0.573f;
const float RPM_CALIBRATION_RIGHT = 0.685f;

void printSetting();
void updatedEEPROM(String _data);
int handleEEPROMwrite(String _data, unsigned int address);
void readEEPROM();
void motion(char _data);
uint32_t getTeensySerial();
void receiveEvent(int bytesReceived);
void updateEncoder_L();
void updateEncoder_R();
void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t dataByte);
void bno_read_multiple(uint8_t i2c_addr, uint8_t reg, uint8_t *buf, uint8_t length);
void thread_func();

void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t dataByte) {
  kire1.beginTransmission(i2c_addr);
  kire1.write(reg);
  kire1.write(dataByte);
  kire1.endTransmission(true);
}

void bno_read_multiple(uint8_t i2c_addr, uint8_t reg, uint8_t *buf, uint8_t length) {
  for (uint32_t n = 0; n < length; n++) {
    if ((n & 31) == 0) {
      kire1.beginTransmission(i2c_addr);
      kire1.write(reg + n);
      kire1.endTransmission(false);
      kire1.requestFrom(i2c_addr, min(length - n, 32), true);
    }

    while (kire1.available()) {
      *buf++ = kire1.read();
    }
  }
}

void thread_func() {
  while (1) {
    bno_read_multiple(BNO_ADDR, ACC_DATA_X_LSB, (uint8_t *)&imu, sizeof imu);
  }
}

void printSetting() {
  Serial.println("Chaging to Setting Mode");
  Serial.print(" T -> Trun \n F -> Forward \n R -> Right \n L -> Left \n B -> Backward \n W -> Work \n D -> Drive \n");
  Serial.println("Enter With as shown with the range below.");
  Serial.println();
  Serial.print(" TRR : Values form 0 - 63    \n TRL : Values form 192 - 255\n");
  Serial.print(" TLR : Values form 64 - 127  \n TLL : Values form 128 - 191\n");
  Serial.println();
  Serial.print(" FRW : Values form 0 - 40    \n FLW : Values form 128 - 168\n");
  Serial.print(" BRW : Values form 64 - 104  \n BLW : Values form 192 - 232\n");
  Serial.println();
  Serial.print(" FRD : Values form 40 - 63    \n FLD : Values form 168 - 191\n");
  Serial.print(" BRD : Values form 104 - 127  \n BLD : Values form 232 - 255\n");
}

void updatedEEPROM(String _data) {
  if (_data.substring(0, _data.indexOf(':')) == "TRR") {
    TRR = handleEEPROMwrite(_data, addressTRR);
  } else if (_data.substring(0, _data.indexOf(':')) == "TRL") {
    TRL = handleEEPROMwrite(_data, addressTRL);
  } else if (_data.substring(0, _data.indexOf(':')) == "TLR") {
    TLR = handleEEPROMwrite(_data, addressTLR);
  } else if (_data.substring(0, _data.indexOf(':')) == "TLL") {
    TLL = handleEEPROMwrite(_data, addressTLL);
  } else if (_data.substring(0, _data.indexOf(':')) == "FRW") {
    FRW = handleEEPROMwrite(_data, addressFRW);
  } else if (_data.substring(0, _data.indexOf(':')) == "FLW") {
    FLW = handleEEPROMwrite(_data, addressFLW);
  } else if (_data.substring(0, _data.indexOf(':')) == "BRW") {
    BRW = handleEEPROMwrite(_data, addressBRW);
  } else if (_data.substring(0, _data.indexOf(':')) == "BLW") {
    BLW = handleEEPROMwrite(_data, addressBLW);
  } else if (_data.substring(0, _data.indexOf(':')) == "FRD") {
    FRD = handleEEPROMwrite(_data, addressFRD);
  } else if (_data.substring(0, _data.indexOf(':')) == "FLD") {
    FLD = handleEEPROMwrite(_data, addressFLD);
  } else if (_data.substring(0, _data.indexOf(':')) == "BRD") {
    BRD = handleEEPROMwrite(_data, addressBRD);
  } else if (_data.substring(0, _data.indexOf(':')) == "BLD") {
    BLD = handleEEPROMwrite(_data, addressBLD);
  } else {
    Serial.println("Enter Correct Variable");
  }
}

int handleEEPROMwrite(String _data, unsigned int address) {
  EEPROM.update(address, _data.substring(_data.indexOf(':')).remove(0, 1).toInt());
  Serial.print("Data Updated | ");
  Serial.print(_data.substring(0, _data.indexOf(':')));
  Serial.print(" : ");
  Serial.println(EEPROM.read(address));
  return EEPROM.read(address);
}

void readEEPROM() {
  FLW = EEPROM.read(addressFLW);
  FRW = EEPROM.read(addressFRW);
  BLW = EEPROM.read(addressBLW);
  BRW = EEPROM.read(addressBRW);

  FLD = EEPROM.read(addressFLD);
  FRD = EEPROM.read(addressFRD);
  BLD = EEPROM.read(addressBLD);
  BRD = EEPROM.read(addressBRD);

  TRR = EEPROM.read(addressTRR);
  TRL = EEPROM.read(addressTRL);
  TLR = EEPROM.read(addressTLR);
  TLL = EEPROM.read(addressTLL);
}

void updateEncoder_L() {
  int MSB = digitalRead(encoderPin_1_L);
  int LSB = digitalRead(encoderPin_2_L);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded_L << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_L++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_L--;

  lastEncoded_L = encoded;
  lastMSB_L = MSB;
  lastLSB_L = LSB;
}

void updateEncoder_R() {
  int MSB = digitalRead(encoderPin_1_R);
  int LSB = digitalRead(encoderPin_2_R);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded_R << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_R++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_R--;

  lastEncoded_R = encoded;
  lastMSB_R = MSB;
  lastLSB_R = LSB;
}

void motion(char _data) {
  if (_data == '0') {
    rpmAlter = false;
    rpmAlter_T = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
  } else if (_data == '1') {
    rpmAlter_T = false;
    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 215 : 250);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 205 : 240);
  } else if (_data == '2') {
    rpmAlter_T = false;
    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 202 : 240);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 211 : 249);
  } else if (_data == '3') {
    rpmAlter = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 82 : 148);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80 : 146);
  } else if (_data == '4') {
    rpmAlter = false;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 83 : 150);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80 : 147);
  } else if (_data == '5') {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 204 : 245);
    analogWrite(pwmPin_R, 150);
  } else if (_data == '6') {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 215 : 250);
  } else if (_data == '7') {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 209 : 251);
    analogWrite(pwmPin_R, 150);
  } else if (_data == '8') {
    rpmAlter_T = false;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 202 : 245);
  }
}

uint32_t getTeensySerial() {
  uint32_t num;
  num = HW_OCOTP_MAC0 & 0xFFFFFF;
  if (num < 10000000) {
    num = num * 10;
    return num;
  }
  return num;
}

void receiveEvent(int bytesReceived) {
  (void)bytesReceived;
  int _data = kire.read();
  Serial.print(_data);

  if (_data == 'a') {
    if (data != '3' && data != '4' && data != '0') {
      rpmAlter = !rpmAlter;
    }
  } else if (_data == 'b') {
    if (data != '1' && data != '2' && data != '0') {
      rpmAlter_T = !rpmAlter_T;
    }
  } else if (_data == 'R') {
  } else if (_data != 10) {
    if (_data == data && elaspedTimeControlCounter < timeConstantControlCounter) {
      startTimeControlCounter = currentTimeControlCounter;
    } else {
      data = _data;
      startTimeControlCounter = currentTimeControlCounter;
    }
  }
}

void setup() {
  Serial.begin(115200);
  kire.begin(SLAVE_ADDRESS);

  pinMode(dirPin_L, OUTPUT);
  pinMode(pwmPin_L, OUTPUT);
  pinMode(dirPin_R, OUTPUT);
  pinMode(pwmPin_R, OUTPUT);

  pinMode(pin_Emergency, INPUT_PULLDOWN);

  startTime = millis();
  startTimeControlCounter = millis();

  digitalWrite(dirPin_L, LOW);
  digitalWrite(dirPin_R, LOW);
  digitalWrite(pwmPin_L, LOW);
  digitalWrite(pwmPin_R, LOW);

  pinMode(encoderPin_1_L, INPUT);
  pinMode(encoderPin_2_L, INPUT);
  digitalWrite(encoderPin_1_L, HIGH);
  digitalWrite(encoderPin_2_L, HIGH);
  attachInterrupt(encoderPin_1_L, updateEncoder_L, CHANGE);
  attachInterrupt(encoderPin_2_L, updateEncoder_L, CHANGE);

  pinMode(encoderPin_1_R, INPUT);
  pinMode(encoderPin_2_R, INPUT);
  digitalWrite(encoderPin_1_R, HIGH);
  digitalWrite(encoderPin_2_R, HIGH);
  attachInterrupt(encoderPin_1_R, updateEncoder_R, CHANGE);
  attachInterrupt(encoderPin_2_R, updateEncoder_R, CHANGE);

  readEEPROM();

  Serial5.write(0);
  Serial5.write(192);
}

void loop() {
  kire.onReceive(receiveEvent);

  static SMA<20> filter_L;
  double rpm_L = 0;
  uint16_t avgRPM_L = 0;

  static SMA<20> filter_R;
  double rpm_R = 0;
  uint16_t avgRPM_R = 0;

  int emergency = analogRead(pin_Emergency);

  currentTime = millis();
  elaspedTime = currentTime - startTime;

  currentTimeControlCounter = millis();
  elaspedTimeControlCounter = currentTimeControlCounter - startTimeControlCounter;

  if (Serial.available()) {
    if (systemCounter == false) {
      int _data = Serial.read();
      if (_data == char('m')) {
        Serial.print(getTeensySerial());
        Serial.print(" | ");
        Serial.println("Motion Module");
        data = char('0');
      } else if (_data == char('s')) {
        systemCounter = true;
        printAlter = false;
        data = '0';
        printSetting();
      } else if (_data == char('p')) {
        printAlter = !printAlter;
      } else if (_data == char('a')) {
        if (data != '3' && data != '4' && data != '0') {
          rpmAlter = !rpmAlter;
        }
      } else if (_data == char('b')) {
        if (data != '1' && data != '2' && data != '0') {
          rpmAlter_T = !rpmAlter_T;
        }
      } else if (_data != 10) {
        data = _data;
        if (_data == data && elaspedTimeControlCounter < timeConstantControlCounter) {
          startTimeControlCounter = currentTimeControlCounter;
        } else {
          data = _data;
          startTimeControlCounter = currentTimeControlCounter;
        }
      }
    } else {
      String _data = Serial.readString();
      _data = _data.remove(_data.length() - 1, 1);
      if (_data.length() == 1 && _data == "s") {
        Serial.println("Chaging to Control Mode");
        systemCounter = false;
      } else if (_data.length() >= 1) {
        updatedEEPROM(_data);
      }
    }
  }

  if (elaspedTimeControlCounter > timeConstantControlCounter) {
    data = '0';
    startTimeControlCounter = currentTimeControlCounter;
  }

  if (emergency > 900) {
    rpmAlter = false;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
    data = '0';
  } else if (emergency < 900) {
    motion(data);
  }

  if (elaspedTime > timeConstant) {
    const float deltaTimeMs = elaspedTime;
    startTime = currentTime;

    long encoderCountL = 0;
    long encoderCountR = 0;
    noInterrupts();
    encoderCountL = encoderValue_L;
    encoderCountR = encoderValue_R;
    encoderValue_L = 0;
    encoderValue_R = 0;
    interrupts();

    rpm_L = ((abs(encoderCountL) * 60000.0f) / (ENCODER_COUNTS_PER_REV * deltaTimeMs)) * RPM_CALIBRATION_LEFT;
    rpm_R = ((abs(encoderCountR) * 60000.0f) / (ENCODER_COUNTS_PER_REV * deltaTimeMs)) * RPM_CALIBRATION_RIGHT;

    avgRPM_L = filter_L(rpm_L);
    avgRPM_R = filter_R(rpm_R);

    if (printAlter == true) {
      Serial.print(data);
      Serial.print(" | ");
      Serial.print(emergency);
      Serial.print(" | ");
      Serial.print(avgRPM_L);
      Serial.print(" | ");
      Serial.print(avgRPM_R);
      Serial.print(" | ");
      Serial.print(1.0 / 16 * imu.eul_heading);
      Serial.print(" | ");
      Serial.print(1.0 / 16 * imu.eul_roll);
      Serial.print(" | ");
      Serial.print(1.0 / 16 * imu.eul_pitch);
      Serial.print(" | ");
      Serial.println(imu.temp);
    }
  }
}
