#include <L3G.h>
#include <Wire.h>
#include <math.h>

#include "global_objects.h"

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define L3G4200D_ADDRESS_SA0_LOW  (0xD0 >> 1)
#define L3G4200D_ADDRESS_SA0_HIGH (0xD2 >> 1)
#define L3GD20_ADDRESS_SA0_LOW    (0xD4 >> 1)
#define L3GD20_ADDRESS_SA0_HIGH   (0xD6 >> 1)

// Public Methods //////////////////////////////////////////////////////////////

bool L3G::init(byte device, byte sa0) {
  _device = device;
  base.x = 0;
  base.y = 0;
  base.z = 0;
  switch (_device) {
  case L3G4200D_DEVICE:
    if (sa0 == L3G_SA0_LOW) {
      address = L3G4200D_ADDRESS_SA0_LOW;
      return true;
    } else if (sa0 == L3G_SA0_HIGH) {
      address = L3G4200D_ADDRESS_SA0_HIGH;
      return true;
    } else
      return autoDetectAddress();
    break;

  case L3GD20_DEVICE:
    if (sa0 == L3G_SA0_LOW) {
      address = L3GD20_ADDRESS_SA0_LOW;
      return true;
    } else if (sa0 == L3G_SA0_HIGH) {
      address = L3GD20_ADDRESS_SA0_HIGH;
      return true;
    } else
      return autoDetectAddress();
    break;

  default:
    return autoDetectAddress();
  }
}

// Turns on the L3G's gyro and places it in normal mode.
void L3G::enableDefault(void) {
  // 0x0F = 0b00001111
  // Normal power mode, all axes enabled
  writeReg(L3G_CTRL_REG1, 0x0F);
}

// Writes a gyro register
void L3G::writeReg(byte reg, byte value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reads a gyro register
byte L3G::readReg(byte reg) {
  byte value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte) 1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Reads the 3 gyro channels and stores them in vector g
void L3G::read() {
  Wire.beginTransmission(address);
  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  Wire.write(L3G_OUT_X_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte) 6);

  while (Wire.available() < 6)
    ;

  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  // combine high and low bytes
  r.x = (int16_t) (xhg << 8 | xlg);
  r.y = (int16_t) (yhg << 8 | ylg);
  r.z = (int16_t) (zhg << 8 | zlg);

  buffer[bufferIndex].x = r.x - base.x;
  buffer[bufferIndex].y = r.y - base.y;
  buffer[bufferIndex].z = r.z - base.z;

  bufferIndex++;
  if (bufferIndex >= bufferSize) {
    bufferIndex = 0;
  }

  g.x = 0;
  g.y = 0;
  g.z = 0;
  for (int i = 0; i < bufferSize; i++) {
    g.x += buffer[i].x;
    g.y += buffer[i].y;
    g.z += buffer[i].z;
  }
  g.x /= bufferSize;
  g.y /= bufferSize;
  g.z /= bufferSize;
}

void L3G::vector_cross(const vector *a, const vector *b, vector *out) {
  out->x = a->y * b->z - a->z * b->y;
  out->y = a->z * b->x - a->x * b->z;
  out->z = a->x * b->y - a->y * b->x;
}

float L3G::vector_dot(const vector *a, const vector *b) {
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

void L3G::vector_normalize(vector *a) {
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

bool L3G::autoDetectAddress(void) {
  // try each possible address and stop if reading WHO_AM_I returns the expected response
  address = L3G4200D_ADDRESS_SA0_LOW;
  if (readReg(L3G_WHO_AM_I) == 0xD3)
    return true;
  address = L3G4200D_ADDRESS_SA0_HIGH;
  if (readReg(L3G_WHO_AM_I) == 0xD3)
    return true;
  address = L3GD20_ADDRESS_SA0_LOW;
  if (readReg(L3G_WHO_AM_I) == 0xD4)
    return true;
  address = L3GD20_ADDRESS_SA0_HIGH;
  if (readReg(L3G_WHO_AM_I) == 0xD4)
    return true;

  return false;
}

void L3G::calibrate() {

  groundStation.textMessage("Calibrating gyro");

  delay(100);
  read();
  base.x = r.x;
  base.y = r.y;
  base.z = r.z;
//  printAll();

  int16_t diff;
  for (int i = 0; i < 10; i++) {
    delay(20);
    read();

    diff = g.x;
    if (diff != 0) {
      base.x += diff / 2;
    }
    diff = g.y;
    if (diff != 0) {
      base.y += diff / 2;
    }
    diff = g.z;
    if (diff != 0) {
      base.z += diff / 2;
    }
  }

  for (int i = 0; i < bufferSize; i++) {
    buffer[i].x = 0;
    buffer[i].y = 0;
    buffer[i].z = 0;
  }

  printAll();
}

void L3G::printAll() {

  groundStation.beginMessage(CMD_GYRO); //G
  groundStation.writeVInt16Field(1, g.x);
  groundStation.writeVInt16Field(2, g.y);
  groundStation.writeVInt16Field(3, g.z);
  groundStation.writeVInt16Field(4, base.x);
  groundStation.writeVInt16Field(5, base.y);
  groundStation.writeVInt16Field(6, base.z);
  groundStation.writeVInt16Field(7, r.x);
  groundStation.writeVInt16Field(8, r.y);
  groundStation.writeVInt16Field(9, r.z);
  groundStation.finishMessage();
}
