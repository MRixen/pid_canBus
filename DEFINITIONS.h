#define di_powerOn 5
#define di_mcp2515_int_rec 7

#define do_motorDirection1 3
//#define do_motorDirection2 4

#define do_enableController 6
#define do_csMcp2515 8
#define do_pwm 9
#define do_csArduino 10

#define I2C_ID_CAN_BUS 7
#define I2C_ID_PID_CONTROLLER 8
#define I2C_ID_MONITOR 9

unsigned char canId;

const byte MESSAGE_SIZE_SPI = 4;
MCP_CAN CAN(do_csMcp2515);
byte len = 0;

int current_motor_angle = 0;
int motorAngle = 0;
double soll_motor_angle_temp = 0; // deg
double const SAMPLE_TIME = 0.05; // s
long encoderValue = 0;
short pwmValueTemp = 0;
bool motorIsActive = false;
bool firstStart;
const int MAX_ENCODER_OFFSET = 60;

long timeout_start = 0;
int value = 0;
int encoderValueTemp = 0;
long MAX_TIMEOUT = 2000;
bool eeprom_states[2];
bool reset_eeprom;

// ------------------
// PACKAGE CONSTRUCT INCOMING
// ------------------
//
// 0. byte: action
// 1. byte: motor id
// 2. byte: velocity
// 3-4. byte: angle
// 5. byte: Direction of the motor 
// 6. byte: 
// 7. byte: 
// ------------------
// ------------------

// ------------------
// PACKAGE CONSTRUCT OUTGOING
// ------------------
//
// 0. byte: action
// 1. byte: motor id
// 2. byte: action state
// 3. byte: 
// 4. byte: 
// 5. byte: 
// 6. byte: 
// 7. byte:                                   
// ------------------
// ------------------

union controlData
{
	short data;
	byte bytes[2];
};

controlData soll_motorAngle, ist_motorAngle, ref_pos;
byte i2c_data_out[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
short soll_motorSpeed;


enum EepromAddresses
{
	eeprom_addr_ref_pos_1,
	eeprom_addr_ref_pos_2,
	eeprom_addr_act_pos_1,
	eeprom_addr_act_pos_2,
	eeprom_addr_error
};

enum ErrorCodes
{
	error_ok,
	error_eeprom_reset,
	error_eeprom_ref_pos,
	error_eeprom_act_pos
};

enum RobotActions
{
	action_nothingToDo,
	action_newPosition,
	action_saveRefPosToEeprom,
	action_disablePidController,
	action_enablePidController,
	action_saveActPosToEeprom
};

enum Outgoing_Package_Content
{
	out_action,
	out_motorId,
	out_actionState
};

//enum Incoming_Package_Content
//{
//	in_action,
//	in_motorId,
//	in_velocity,
//	in_angle_1,
//	in_angle_2,
//	in_motorDir
//};

// UPDATE
enum Incoming_Package_Content
{
	in_action,
	in_angle,
	in_velocity,
	in_motorDir
};

enum ActionStates
{
	state_init,
	state_complete,
	state_pending
};

RobotActions robotActions;
Outgoing_Package_Content outgoing_Package_Content;
Incoming_Package_Content incoming_Package_Content;
ActionStates actionStates;

byte incoming_data[MESSAGE_SIZE_SPI];
byte outgoing_data[MESSAGE_SIZE_SPI];


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
bool lockAction = false;
const int MAX_MOTOR_ANGLE = 90; 
const int MIN_MOTOR_ANGLE = -90;
int di_powerOn_state_old;
bool posOutReached = false;

union motor_id
{
	short lowByte;
	short highByte;
	byte bytes[2];
};
