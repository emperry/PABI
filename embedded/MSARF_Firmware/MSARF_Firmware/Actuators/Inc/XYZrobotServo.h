/*
 * SmartServo.h
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#ifndef XYZROBOTSERVO_H_
#define XYZROBOTSERVO_H_

#include "Actuator.h"
#include "main.h"

#define RADS_PER_TICK 0.00563

/// The possible communication errors that can happen when reading the
/// acknowledgment packet from a servo.
enum class XYZrobotServoError
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

enum class XYZrobotServoBaudRate
{
  B9600 = 1,
  B19200 = 2,
  B57600 = 6,
  B115200 = 12,
};

static inline uint32_t XYZrobotServoBaudRateToInt(XYZrobotServoBaudRate baud)
{
  switch (baud)
  {
  case XYZrobotServoBaudRate::B9600:   return 9600;
  case XYZrobotServoBaudRate::B19200:  return 19200;
  case XYZrobotServoBaudRate::B57600:  return 57600;
  case XYZrobotServoBaudRate::B115200: return 115200;
  default: return 0;
  }
}

/// The possible values for the ACK_Policy parameter stored in the servo's
/// EEPROM and RAM.  This parameter determins which commands the servo will send
/// an acknowledgment response for.
enum class XYZrobotServoAckPolicy
{
  // The servo only responds to STAT commands.
  OnlyStat = 0,
  // The servo only responds to STAT, EEPROM Read, and RAM Read commands.
  OnlyReadAndStat = 1,
  // The servo responds to all commands.
  All = 2,
};

enum class XYZrobotServoSpdctrlPolicy
{
  OpenLoop = 0,
  CloseLoop = 1,
};

/// This struct represents the data returned by a STAT command.
struct XYZrobotServoStatus
{
  uint8_t statusError;
  uint8_t statusDetail;
  uint16_t pwm;
  uint16_t posRef;
  uint16_t position;
  uint16_t iBus;
} __attribute__((packed));

class XYZrobotServo : public Actuator, public Sensor{
public:
  XYZrobotServo(UART_HandleTypeDef *_huart, uint8_t _id, uint16_t nucleoAddress);
  void eepromWrite(uint8_t startAddress, const uint8_t *, uint8_t dataSize);
  void eepromRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize);
  void ramWrite(uint8_t startAddress, const uint8_t *, uint8_t dataSize);
  void ramRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize);
  void writeBaudRateEeprom(XYZrobotServoBaudRate baudRate);
  XYZrobotServoBaudRate readBaudRateEeprom();
  void writeIdEeprom(uint8_t id);
  uint8_t readIdEeprom();
  void writeIdRam(uint8_t id);
  void writeAckPolicyEeprom(XYZrobotServoAckPolicy);
  XYZrobotServoAckPolicy readAckPolicyEeprom();
  void writeAckPolicyRam(XYZrobotServoAckPolicy);
  void writeAlarmLedPolicyRam(uint8_t);
  void writeSpdctrlPolicyRam(XYZrobotServoSpdctrlPolicy policy);
  void writeMaxPwmRam(uint16_t value);
  void writeLedControl(uint8_t);
  XYZrobotServoAckPolicy readAckPolicyRam();
  XYZrobotServoStatus readStatus();

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

  void move(float position, float speed);
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

  XYZrobotServoError lastError;

  uint8_t id;
  UART_HandleTypeDef *huart;

  uint8_t garbageBuffer[50];

  //XYZrobotServoStatus *_currStatus = 0;
  //bool _isBrake = true;
};

#endif /* SMARTSERVO_H_ */
