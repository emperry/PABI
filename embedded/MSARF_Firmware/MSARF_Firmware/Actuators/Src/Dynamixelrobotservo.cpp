/*
 * Dynamixel.cpp
 *
 *  Created on: Mar 20, 2022
 *  Adapted from tdubuke
 *      Author: eroberts
 */

#include <DynamixelrobotServo.h>

#define CMD_EEPROM_WRITE 0x01
#define CMD_EEPROM_READ  0x02
#define CMD_RAM_WRITE    0x03
#define CMD_RAM_READ     0x04
#define CMD_I_JOG        0x05
#define CMD_S_JOG        0x06
#define CMD_STAT         0x07
#define CMD_ROLLBACK     0x08
#define CMD_REBOOT       0x09

#define SET_POSITION_CONTROL 0
#define SET_SPEED_CONTROL 1
#define SET_TORQUE_OFF 2
#define SET_POSITION_CONTROL_SERVO_ON 3

DynamixelrobotServo::DynamixelrobotServo(USART_HandleTypeDef *husart, uint8_t _id, uint16_t nucleoAddress):
	Actuator(nucleoAddress), Sensor(nucleoAddress), id(_id), husart(husart){}

/// Writes data from the specified buffer to the servo's EEPROM.
///
/// After running this command, we recommend delaying for 10 ms per data byte
/// before sending the next command to this servo, since writing to EEPROM
/// takes some time and the servo cannot receive more commands until it is
/// done.
void DynamixelrobotServo::eepromWrite(uint8_t startAddress, const uint8_t * data, uint8_t dataSize)
{
  memoryWrite(CMD_EEPROM_WRITE, startAddress, data, dataSize);
}

/// Reads data from the servo's EEPROM and stores it in the specified buffer.
///
/// The data size should be 35 or less
void DynamixelrobotServo::eepromRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize)
{
  memoryRead(CMD_EEPROM_READ, startAddress, data, dataSize);
}

/// Writes data from the specified buffer to the servo's RAM.
void DynamixelrobotServo::ramWrite(uint8_t startAddress, const uint8_t * data, uint8_t dataSize)
{
  memoryWrite(CMD_RAM_WRITE, startAddress, data, dataSize);
}

/// Reads data from the servo's RAM and stores it in the specified buffer.
///
/// The data size should be 35 or less
void DynamixelrobotServo::ramRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize)
{
  memoryRead(CMD_RAM_READ, startAddress, data, dataSize);
}

/// Write the Baud_Rate parameter byte in EEPROM, which determines which baud
/// rate the servo uses on its serial interface.
///
/// After running this command, we recommend delaying for 10 ms before sending
/// the next command to this servo, since writing to EEPROM takes some time
/// and the servo cannot receive more commands until it is done.
void DynamixelrobotServo::writeBaudRateEeprom(DynamixelrobotServoBaudRate baud)
{
  uint8_t b = (uint8_t)baud;
  memoryWrite(CMD_EEPROM_WRITE, 5, &b, 1);
}

/// Reads the Baud_Rate parameter byte in EEPROM, which determines which baud
 /// rate the servo uses on its serial interface.
DynamixelrobotServoBaudRate DynamixelrobotServo::readBaudRateEeprom()
{
  uint8_t b = 0;
  memoryRead(CMD_EEPROM_READ, 5, &b, 1);
  return (DynamixelrobotServoBaudRate)b;
}

/// Write the sID parameter byte in EEPROM, which determines which ID the
/// servo uses on its serial interface.
///
/// After running this command, we recommend delaying for 10 ms before sending
/// the next command to this servo, since writing to EEPROM takes some time
/// and the servo cannot receive more commands until it is done.
void DynamixelrobotServo::writeIdEeprom(uint8_t id)
{
  memoryWrite(CMD_EEPROM_WRITE, 6, &id, 1);
}

/// Reads the sID parameter byte in EEPROM, which determines which ID the
/// servo uses on its serial interface.
uint8_t DynamixelrobotServo::readIdEeprom()
{
  uint8_t id = 0;
  memoryRead(CMD_EEPROM_READ, 6, &id, 1);
  return id;
}

/// Write the sID parameter byte in RAM, which determines which ID the
/// servo uses on its serial interface.
///
/// Write the ACK_Policy parameter byte in RAM.
void DynamixelrobotServo::writeIdRam(uint8_t id)
{
  memoryWrite(CMD_RAM_WRITE, 0, &id, 1);
}

/// Write the ACK_Policy parameter byte in EEPROM.
///
/// After running this command, we recommend delaying for 10 ms before sending
/// the next command to this servo, since writing to EEPROM takes some time
/// and the servo cannot receive more commands until it is done.
void DynamixelrobotServo::writeAckPolicyEeprom(DynamixelrobotServoAckPolicy policy)
{
  uint8_t p = (uint8_t)policy;
  eepromWrite(7, &p, 1);
}

/// Read the ACK_Policy parameter byte in EEPROM.
DynamixelrobotServoAckPolicy DynamixelrobotServo::readAckPolicyEeprom()
{
  uint8_t result = 0;
  eepromRead(7, &result, 1);
  return (DynamixelrobotServoAckPolicy)result;
}

/// Write the ACK_Policy parameter byte in RAM.
void DynamixelrobotServo::writeAckPolicyRam(DynamixelrobotServoAckPolicy policy)
{
  uint8_t p = (uint8_t)policy;
  ramWrite(1, &p, 1);
}

/// Write the Alarm_LED_Policy byte in RAM.  This controls which LEDs on the
/// servo are controlled by the user and which are controlled by the system.
///
/// A 0 bit means the LED is controlled by the system, and a 1 bit means the
/// LED is controlled by the user.
///
/// - Bit 0: White LED
/// - Bit 1: Blue LED
/// - Bit 2: Green LED
/// - Bit 3: Red LED
///
/// To control user LEDs, see writeLedControl().
void DynamixelrobotServo::writeAlarmLedPolicyRam(uint8_t policy)
{
  ramWrite(2, &policy, 1);
}

/// Sets the SPDctrl_Policy variable in RAM.
void DynamixelrobotServo::writeSpdctrlPolicyRam(DynamixelrobotServoSpdctrlPolicy policy)
{
  uint8_t p = (uint8_t)policy;
  ramWrite(4, &p, 1);
}

/// Sets the maximum PWM value in RAM.
///
/// This should be a number between 0 and 1023 that indicates how strong the
/// servo should be allowed to drive its motor, with 1023 corresponding to
/// 100%.
void DynamixelrobotServo::writeMaxPwmRam(uint16_t value)
{
  ramWrite(16, (uint8_t *)&value, 1023);
}

/// After calling writeAlarmLedPolicyRam(), you can use this to control any
/// LEDs that are configured as user LED.
///
/// - Bit 0: White LED
/// - Bit 1: Blue LED
/// - Bit 2: Green LED
/// - Bit 3: Red LED
void DynamixelrobotServo::writeLedControl(uint8_t control)
{
  ramWrite(53, &control, 1);
}

/// Read the ACK_Policy parameter byte in RAM.
DynamixelrobotServoAckPolicy DynamixelrobotServo::readAckPolicyRam()
{
  uint8_t result = 0;
  ramRead(1, &result, 1);
  return (DynamixelrobotServoAckPolicy)result;
}

/// Sends a STAT command to the servo and returns the results.
DynamixelrobotServoStatus DynamixelrobotServo::readStatus()
{
  flushRead();

  DynamixelrobotServoStatus status;
  sendRequest(CMD_STAT, NULL, 0);
  readAck(CMD_STAT, (uint8_t *)&status, 10);
  return status;
}

/// Sends an I-JOG command to set the target/goal position for this servo.
///
/// The position should be a number between 0 and 1023.
///
/// The playtime should the desired time for the movement to take, in units of
/// 10 ms.  For example, a playtime of 50 would correspond to 500 ms or 0.5
/// seconds.
void DynamixelrobotServo::setPosition(uint16_t position, uint8_t playtime)
{
  sendIJog(position, SET_POSITION_CONTROL, playtime);
}

/// Sends an I-JOG command to set the speed for this servo.
///
/// The speed should be a number between -1023 and 1023.
///
/// A value of 0 causes an abrupt stop.  Non-zero values generally cause the
/// servo to smoothly ramp its speed up or down to the specified value.
///
/// The playtime specifies how long this command will last, in units of 10 ms.
/// A value of 0 makes it last indefinitely.
///
/// See writeSpeedControlPolicyRam().
void DynamixelrobotServo::setSpeed(int16_t speed, uint8_t playtime)
{
  sendIJog(speed, SET_SPEED_CONTROL, playtime);
}

/// Sends an I-JOG command to turn off the servo's motor.
///

void DynamixelrobotServo::torqueOff()
{
  sendIJog(0, SET_TORQUE_OFF, 0);
}

// Resets all parameters in EEPROM to their default values.
//
// After running this command, we recommend delaying for 2500 ms before
// sending the next command to this servo, since it takes the servo a while to
// change its parameters.
void DynamixelrobotServo::rollback()
{
  sendRequest(CMD_ROLLBACK, NULL, 0);
}

// Resets the servo.
//
// After running this command, we recommend delaying for 2500 ms before
// sending the next command to this servo, since it takes the servo a while to
// restart.
void DynamixelrobotServo::reboot()
{
  sendRequest(CMD_REBOOT, NULL, 0);
}

void DynamixelrobotServo::flushRead()
{
	HAL_USART_Receive(husart, garbageBuffer, 50, 0);
}

void DynamixelrobotServo::sendRequest(uint8_t cmd,
  const uint8_t * data1, uint8_t data1Size,
  const uint8_t * data2, uint8_t data2Size)
{
  uint8_t header[7];

  uint8_t size = data1Size + data2Size + sizeof(header);

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < data1Size; i++) { checksum ^= data1[i]; }
  for (uint8_t i = 0; i < data2Size; i++) { checksum ^= data2[i]; }

  header[0] = 0xFF;
  header[1] = 0xFF;
  header[2] = size;
  header[3] = id;
  header[4] = cmd;
  header[5] = checksum & 0xFE;
  header[6] = ~checksum & 0xFE;

  HAL_USART_Transmit(husart, header, sizeof(header), HAL_MAX_DELAY);

  if (data1Size) { HAL_USART_Transmit(husart, (uint8_t *)data1, data1Size, HAL_MAX_DELAY); }
  if (data2Size) { HAL_USART_Transmit(husart, (uint8_t *)data2, data2Size, HAL_MAX_DELAY); }

  lastError = DynamixelrobotServoError::None;
}

void DynamixelrobotServo::readAck(uint8_t cmd,
  uint8_t * data1, uint8_t data1Size,
  uint8_t * data2, uint8_t data2Size)
{
  cmd |= 0x40;
  uint8_t rec_header[7];
  uint8_t size = sizeof(rec_header) + data1Size + data2Size;

  HAL_StatusTypeDef status = HAL_USART_Receive(husart, rec_header, sizeof(rec_header), 100);
  if(status == HAL_TIMEOUT){
	  lastError = DynamixelrobotServoError::HeaderTimeout;
	  return;
  }

  if (rec_header[0] != 0xFF)
  {
    lastError = DynamixelrobotServoError::HeaderByte1Wrong;
    return;
  }

  if (rec_header[1] != 0xFF)
  {
    lastError = DynamixelrobotServoError::HeaderByte2Wrong;
    return;
  }

  if (rec_header[3] != id)
  {
    lastError = DynamixelrobotServoError::IdWrong;
    return;
  }

  if (rec_header[4] != cmd)
  {
    lastError = DynamixelrobotServoError::CmdWrong;
    return;
  }

  if (rec_header[2] != size)
  {
    lastError = DynamixelrobotServoError::SizeWrong;
    return;
  }

  if (data1Size)
  {
	  status = HAL_USART_Receive(husart, data1, data1Size, 50);
	  if(status == HAL_TIMEOUT){
		  return;
	  }
  }

  if (data2Size)
  {
	  status = HAL_USART_Receive(husart, data2, data2Size, 50);
	  if(status == HAL_TIMEOUT){
		  return;
	  }
  }

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < data1Size; i++) { checksum ^= data1[i]; }
  for (uint8_t i = 0; i < data2Size; i++) { checksum ^= data2[i]; }

  if (rec_header[5] != (checksum & 0xFE))
  {
    lastError = DynamixelrobotServoError::Checksum1Wrong;
    return;
  }

  if (rec_header[6] != (~checksum & 0xFE))
  {
    lastError = DynamixelrobotServoError::Checksum2Wrong;
    return;
  }

  lastError = DynamixelrobotServoError::None;
}

void DynamixelrobotServo::memoryWrite(uint8_t cmd, uint8_t startAddress,
  const uint8_t * data, uint8_t dataSize)
{
  uint8_t request[2];
  request[0] = startAddress;
  request[1] = dataSize;

  sendRequest(cmd, request, sizeof(request), data, dataSize);
}

void DynamixelrobotServo::memoryRead(uint8_t cmd, uint8_t startAddress,
  uint8_t * data, uint8_t dataSize)
{
  flushRead();

  uint8_t request[2];
  request[0] = startAddress;
  request[1] = dataSize;
  sendRequest(cmd, request, sizeof(request));

  uint8_t response[4];
  readAck(cmd, response, 4, data, dataSize);
  if (getLastError()) { return; }

  if (response[2] != request[0])
  {
    lastError = DynamixelrobotServoError::ReadOffsetWrong;
    return;
  }

  if (response[3] != request[1])
  {
    lastError = DynamixelrobotServoError::ReadLengthWrong;
    return;
  }
}

void DynamixelrobotServo::sendIJog(uint16_t goal, uint8_t type, uint8_t playtime)
{
  uint8_t data[5];
  data[0] = goal & 0xFF;
  data[1] = goal >> 8 & 0xFF;
  data[2] = type;
  data[3] = id;
  data[4] = playtime;
  sendRequest(CMD_I_JOG, data, sizeof(data));
}

uint8_t DynamixelrobotServo::detectServo(){

	uint8_t header[7];
	uint8_t size = sizeof(header);

	for(uint8_t i = 1; i < 255; i++){
		uint8_t checksum = size ^ i ^ CMD_STAT;

		header[0] = 0xFF;
		header[1] = 0xFF;
		header[2] = size;
		header[3] = i;
		header[4] = CMD_STAT;
		header[5] = checksum & 0xFE;
		header[6] = ~checksum & 0xFE;

		HAL_USART_Transmit(husart, header, sizeof(header), HAL_MAX_DELAY);

		DynamixelrobotServoStatus status;
		readAck(CMD_STAT, (uint8_t *)&status, 10);

		if(lastError != DynamixelrobotServoError::HeaderTimeout){
			return i;
		}
	}

	return -1;
}

void DynamixelrobotServo::move(float position, float speed){
	float pos = position / RADS_PER_TICK;
	float playtime = speed * 100;
	if(playtime > 255){
		playtime = 255;
	}
	sendIJog((uint16_t)pos, SET_POSITION_CONTROL, (uint8_t)playtime);
}

void DynamixelrobotServo::torque(bool isOn){
	if(isOn){
		writeMaxPwmRam(1023);
	}else{
		writeMaxPwmRam(0);
	}
}

float DynamixelrobotServo::getValue(){
	uint16_t position = readPosition();
	if(lastError != DynamixelrobotServoError::None){
		return 0;
	}
	return (float)position * RADS_PER_TICK;
}
