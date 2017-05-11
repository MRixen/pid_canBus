#pragma once
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
int motorVel;


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

enum Incoming_Package_Content
{
	in_action,
	in_motorId,
	in_velocity,
	in_angle_1,
	in_angle_2,
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

byte incoming_data[BYTES_TO_READ];
byte outgoing_data[BYTES_TO_SEND];
