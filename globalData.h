#pragma once
#define di_encoderPinA 2
#define di_encoderPinB 3
#define do_motorDirection 4
#define di_powerOn 5
#define do_pwm 9
#define do_csMcp2515 8
#define do_csArduino 10
#define di_mcp2515_int_rec 7

union controlData
{
	short data;
	byte bytes[2];
};

controlData soll_motorAngle, ist_motorAngle, ref_pos;
int motorVel;
short pwmValueTemp = 0;
bool motorIsActive = false;
bool firstStart;
const int MAX_ENCODER_OFFSET = 60;

long timeout_start = 0;
int value = 0;
int encoderValueTemp = 0;
long MAX_TIMEOUT = 2000;


// TODO: Expand content of the send byte array
// CONTENT
// 0. byte: Number of the task 
// 1. byte: ID of the motor to move
// 2-3. byte: Velocity to move the motor to the specified position
// 4-5. byte: Endposition for the specific motor 
// 6. byte: Direction of the motor
// 7. byte: 

// CONTENT
// 0. byte: Direction of the motor
// 1-2. byte: Current angle of motor
byte sendBuffer[8];

// GLOBAL DATA
const long MAX_WAIT_TIME = 10000;
const int MAX_ERROR_COUNTER_MODESWITCH = 3;
bool stopAllOperations = false;
bool debugMode;
const int ADXL = 1;
const int MCP2515 = 2;
const int NO_DEVICE = 3;
long errorTimerValue;
const long MAX_PULSE_TYPE_DURATION = (5 * 3.14) * 1000;

byte ReadBuffer[7];
int pulseCounter = 1;
int counter = 1;

// PDI controller data
const int MULTIPLICATION_FACTOR = 10;

bool pid_controller_enabled = true;
bool writeToEeprom_inUse = false;
byte REFERENCE_POSITION[2];

