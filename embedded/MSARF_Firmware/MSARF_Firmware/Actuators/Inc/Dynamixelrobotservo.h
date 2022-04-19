/*
 * SmartServo.h
 *
 *  Created on: Mar 20, 2022
 *  Adapted from tdubuke actuator code
 *      Author: eroberts
 */

#ifndef DYNAMIXELROBOTSERVO_H_
#define DYNAMIXEL_H_

#include "Actuator.h"
#include "main.h"

#define RADS_PER_TICK 0.00563

/// The possible communication errors that can happen when reading the
/// acknowledgment packet from a servo.
enum class DynamixelrobotServoError
{
  /// No error.
  None = 0,
  /// There was a timeout waiting to receive the 7-byte acknowledgment header.
  HeaderTimeout = 1,
  /// The first byte of received header was not 0xFF.
  HeaderByte1Wrong = 2,
  /// The second byte of the received header was not 0xFF.
  HeaderByte2Wrong = 3,
  /// The ID byte in the received header was wrong.
  IdWrong = 4,
  /// The CMD bytes in the received header was wrong.
  CmdWrong = 5,
  /// The size byte in the received header was wrong.
  SizeWrong = 6,
  /// There was a timeout reading the first expected block of data in the
  /// acknowledgment.
  Data1Timeout = 7,
  /// There was a timeout reading the second expected block of data in the
  /// acknowledgment.
  Data2Timeout = 8,
  /// The first byte of the checksum was wrong.
  Checksum1Wrong = 9,
  /// The second byte of the checksum was wrong.
  Checksum2Wrong = 10,
  /// The offset byte returned by an EEPROM Read or RAM Read command was wrong.
  ReadOffsetWrong = 16,
  /// The length byte returned by an EEPROM Read or RAM Read command was wrong.
  ReadLengthWrong = 17,
};

enum class DynamixelrobotServoBaudRate
{
  B1000000 = 1,
  B500000 = 3,
  B400000 = 4,
  B250000 = 7,
  B200000 = 9,
  B115200 = 16,
  B57600 = 34,
  B19200 = 103,
  B9600 = 207,
};

static inline uint32_t DynamixelrobotServoBaudRateToInt(DynamixelrobotServoBaudRate baud)
{
  switch (baud)
  {
  case DynamixelrobotServoBaudRate::B1000000: return 1000000;
  case DynamixelrobotServoBaudRate::B500000: return 500000;
  case DynamixelrobotServoBaudRate::B400000: return 400000;
  case DynamixelrobotServoBaudRate::B250000: return 250000;
  case DynamixelrobotServoBaudRate::B200000: return 200000;
  case DynamixelrobotServoBaudRate::B115200: return 115200;
  case DynamixelrobotServoBaudRate::B57600: return 57600;
  case DynamixelrobotServoBaudRate::B19200: return 19200;
  case DynamixelrobotServoBaudRate::B9600: return 9600;
  default: return 0;
  }
}

/// The possible values for the ACK_Policy parameter stored in the servo's
/// EEPROM and RAM.  This parameter determins which commands the servo will send
/// an acknowledgment response for.
enum class DynamixelrobotServoAckPolicy
{
  // The servo only responds to STAT commands.
  OnlyStat = 0,
  // The servo only responds to STAT, EEPROM Read, and RAM Read commands.
  OnlyReadAndStat = 1,
  // The servo responds to all commands.
  All = 2,
};

enum class DynamixelrobotServoSpdctrlPolicy
{
  OpenLoop = 0,
  CloseLoop = 1,
};

/// This struct represents the data returned by a STAT command.
struct DynamixelrobotServoStatus
{
  uint8_t statusError;
  uint8_t statusDetail;
  uint16_t pwm;
  uint16_t posRef;
  uint16_t position;
  uint16_t iBus;
} __attribute__((packed));

class DynamixelrobotServo : public Actuator, public Sensor{
public:
  DynamixelrobotServo(USART_HandleTypeDef *_husart, uint8_t _id, uint16_t nucleoAddress);
  void eepromWrite(uint8_t startAddress, const uint8_t *, uint8_t dataSize);
  void eepromRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize);
  void ramWrite(uint8_t startAddress, const uint8_t *, uint8_t dataSize);
  void ramRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize);
  void writeBaudRateEeprom(DynamixelrobotServoBaudRate baudRate);
  DynamixelrobotServoBaudRate readBaudRateEeprom();
  void writeIdEeprom(uint8_t id);
  uint8_t readIdEeprom();
  void writeIdRam(uint8_t id);
  void writeAckPolicyEeprom(DynamixelrobotServoAckPolicy);
  DynamixelrobotServoAckPolicy readAckPolicyEeprom();
  void writeAckPolicyRam(DynamixelrobotServoAckPolicy);
  void writeAlarmLedPolicyRam(uint8_t);
  void writeSpdctrlPolicyRam(DynamixelrobotServoSpdctrlPolicy policy);
  void writeMaxPwmRam(uint16_t value);
  void writeLedControl(uint8_t);
  DynamixelrobotServoAckPolicy readAckPolicyRam();
  DynamixelrobotServoStatus readStatus();

  /// Uses a STAT command to read the current PWM duty cycle.
  ///
  /// See readStatus().
  uint16_t readPwm() { return readStatus().pwm; }

  /// Uses a STAT command to read the servo position.
  ///
  /// See readStatus().
  uint16_t readPosition() { return readStatus().position; }

  /// Uses a STAT command to read the servo position goal.
  ///
  /// If the servo has no position goal, this is just its current measured
  /// position.
  ///
  /// See readStatus().
  uint16_t readPosRef() { return readStatus().posRef; }

  /// Uses a STAT command to read the bus current.
  ///
  /// See readStatus().
  uint16_t readIBus() { return readStatus().iBus; }

  void setPosition(uint16_t position, uint8_t playtime = 0);
  void setSpeed(int16_t speed, uint8_t playtime = 0);
  void torqueOff();
  void rollback();
  void reboot();

  void move(float rad, float time);
  void torque(bool isOn);
  float getValue();
  /// Returns the communication error from the last command.  The return value
  /// will be 0 if there was no error and non-zero if there was an error.  The
  /// return value will be one of the values of the XYZrobotServoError enum.
  uint8_t getLastError() const { return (uint8_t)lastError; }

  uint8_t detectServo();

  /// Get the servo ID assigned to this object.
  uint8_t getId() const { return id; }

private:
  void flushRead();
  void sendRequest(uint8_t cmd,
    const uint8_t * data1, uint8_t data1Size,
    const uint8_t * data2 = NULL, uint8_t data2Size = 0);
  void readAck(uint8_t cmd,
    uint8_t * data1, uint8_t data1Size,
    uint8_t * data2 = NULL, uint8_t data2Size = 0);
  void memoryWrite(uint8_t cmd, uint8_t startAddress, const uint8_t * data, uint8_t dataSize);
  void memoryRead(uint8_t cmd, uint8_t startAddress, uint8_t * data, uint8_t dataSize);
  void sendIJog(uint16_t goal, uint8_t type, uint8_t playtime);

  DynamixelrobotServoError lastError;

  uint8_t id;
  USART_HandleTypeDef *husart;

  uint8_t garbageBuffer[50];

  //DynamixelrobotServoStatus *_currStatus = 0;
  //bool _isBrake = true;
};

#endif /* DYNAMIXELROBOTSERVO_H_ */
