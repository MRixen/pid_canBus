#include <Wire.h>
#include <SPI.h>
#include "mcp_can.h"
#include <EEPROM.h>
#include "DEFINITIONS.h"
#include "MCP2515.h"

// ------------------------------------
// Controlling motors with id 
const int MOTOR_ID = 6; // Motor id (its NOT possible to use the same idetifier for two devices in the bus
// ------------------------------------

// !!!! NOTE !!!!
// Motor ID: 1-4 reserved for big motor
// Motor ID: 5-6 reserved for small motor -> There are two pins for direction! AND standbyPin
// 
//
// TODO: Configure identifier for can bus to receive data for motor 0 only
void setup()
{
	// Configure serial interface
	Serial.begin(115200);

	// Configure program data
	firstStart = true;

	// USER CONFIGURATION
	debugMode = true;

	// Define I/Os
	pinMode(do_csMcp2515, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9
	//pinMode(do_csArduino, OUTPUT); // Set as input to enable pull up resistor. It's neccessary because the ss line is defined at pin 10 + 9
	//pinMode(di_mcp2515_int_rec, INPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(di_powerOn, INPUT);
	pinMode(do_motorDirection1, OUTPUT);
	//pinMode(do_motorDirection2, OUTPUT);
	pinMode(do_enableController, OUTPUT);

	// Init I/Os
	digitalWrite(do_enableController, HIGH);

	// Read input signal for storing actual encoder position
	di_powerOn_state_old = digitalRead(di_powerOn);

	// Configure SPI
	//SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
	//SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
	//SPI.begin();

	// Configure MCP2515
	//initMcp2515();
	while (CAN_OK != CAN.begin(CAN_250KBPS, MODE_NORMAL))            // init can bus : baudrate = 500k
	{
		Serial.println("CAN BUS Shield init fail");
		Serial.println(" Init CAN BUS Shield again");
		delay(100);
	}
	Serial.println("CAN BUS Shield init ok!");


	// Set identifier, message length, etc.
	//mcp2515_init_tx_buffer0(REGISTER_TXB0SIDL_VALUE, REGISTER_TXB0SIDH_VALUE, BYTES_TO_SEND);
	//mcp2515_init_tx_buffer1(REGISTER_TXB1SIDL_VALUE, REGISTER_TXB1SIDH_VALUE, BYTES_TO_SEND);
	//mcp2515_init_tx_buffer2(REGISTER_TXB2SIDL_VALUE, REGISTER_TXB2SIDH_VALUE, BYTES_TO_SEND);

	// Init
	soll_motorAngle.data = 0;

	// Give time to set up
	delay(100);

	// Start timer to measure the program execution
	errorTimerValue = millis();

	// TODO:
	// Turn green led on to show that everything was inititalized and maybe ok

	Wire.begin(I2C_ID_CAN_BUS); // join i2c bus (address optional for master)
}


void loop()
{
	// TODO:
	// Cancel pending operations

	rxStateIst = 0x00;
	pwmValueTemp = 0;
	motorIsActive = true;
	canId = 0;

	//Serial.println(digitalRead(di_mcp2515_int_rec));

	// Check if message is received in buffer 0 or 1
	//if ((digitalRead(di_mcp2515_int_rec) == 0))
	//{
	//	// Get current rx buffer
	//	rxStateIst = mcp2515_execute_read_state_command(do_csMcp2515);

	//	// Read incoming data package
	//	receiveData(rxStateIst, rxStateSoll);

	//	// Wait after receive command
	//	delay((SAMPLE_TIME / 2) * 1000);
	//}
	if (CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
	{
		CAN.readMsgBuf(&len, incoming_data);    // read data,  len: data length, buf: data buf

		canId = CAN.getCanId();

		//Serial.println("-----------------------------");
		//Serial.print("get data from ID: ");
		

		//for (int i = 0; i < len; i++)    // print the data
		//{
		//	Serial.print(incoming_data[i]);
		//	Serial.print("\t");
		//}
		//Serial.println();
	}


	// Do something when the motor id is correct
	if (MOTOR_ID == canId) {

		if ((incoming_data[in_action] == action_nothingToDo)) {
			lockAction = false;
			for (size_t i = 0; i < BYTES_TO_SEND; i++) outgoing_data[i] = 0;
			// Send motor id every time as alive flag
			outgoing_data[out_motorId] = MOTOR_ID;

			// Send data (1. byte: soll_angle, 2. byte: save-action)
			// Possible actions: 0: Nothing to do, 1: Save ref pos, 2: Save act pos
			i2c_data_out[0] = 0;
			i2c_data_out[1] = 0;
			i2c_data_out[2] = 0;

			Wire.beginTransmission(I2C_ID_PID_CONTROLLER);
			Wire.write(i2c_data_out, 3);
			Wire.endTransmission();
		}

		// Check incoming action and set outgoing package
		//if ((!lockAction) & ((incoming_data[in_action] == action_saveRefPosToEeprom) | (incoming_data[in_action] == action_disablePidController) | (incoming_data[in_action] == action_enablePidController) | (incoming_data[in_action] == action_newPosition) | (incoming_data[in_action] == action_saveActPosToEeprom)))
		//{
		//	outgoing_data[out_action] = incoming_data[in_action];
		//	outgoing_data[out_actionState] = state_pending;
		//	outgoing_data[out_motorId] = incoming_data[in_motorId];
		//}

		// Send package back to client
		//sendData(BYTES_TO_SEND, outgoing_data);

		// Wait after send command
		//delay((SAMPLE_TIME / 2) * 1000);

		// Work on action <saveRefPosToEeprom>
		if ((!lockAction) & (incoming_data[in_action] == action_saveRefPosToEeprom))
		{
			Serial.print("action_saveRefPosToEeprom");

			// Send data (1. byte: soll_angle, 2. byte: save-action)
			// Possible actions: 0: Nothing to do, 1: Save ref pos, 2: Save act pos
			i2c_data_out[0] = 0;
			i2c_data_out[1] = 1;
			i2c_data_out[2] = 0;

			Wire.beginTransmission(I2C_ID_PID_CONTROLLER);
			Wire.write(i2c_data_out, 3);
			Wire.endTransmission();
		}

		// Work on action <safe actual position to eeprom>
		if ((!lockAction) & (incoming_data[in_action] == action_saveActPosToEeprom))
		{
			// Send data (1. byte: soll_angle, 2. byte: save-action)
			// Possible actions: 0: Nothing to do, 1: Save ref pos, 2: Save act pos
			i2c_data_out[0] = 0;
			i2c_data_out[1] = 2;
			i2c_data_out[2] = 0;

			Wire.beginTransmission(I2C_ID_PID_CONTROLLER);
			Wire.write(i2c_data_out, 3);
			Wire.endTransmission();
		}

		// Work on action <disablePidController>
		if ((incoming_data[in_action] == action_disablePidController))
		{
			digitalWrite(do_enableController, LOW);
			outgoing_data[out_actionState] = state_complete;
			Serial.println("disablePidController");
		}

		// Work on action <enablePidController>
		if ((incoming_data[in_action] == action_enablePidController))
		{
			digitalWrite(do_enableController, HIGH);
			outgoing_data[out_actionState] = state_complete;
			Serial.println("enablePidController");
		}

		// Work on action <newPosition>
		if ((incoming_data[in_action] == action_newPosition))
		{
			// Get soll angle 
			/*soll_motorAngle.bytes[0] = incoming_data[in_angle_2];
			soll_motorAngle.bytes[1] = incoming_data[in_angle_1];*/

			// UPDATE
			soll_motor_angle_temp = incoming_data[in_angle] + ref_pos.data;

			// Get velocity
			soll_motorSpeed = incoming_data[in_velocity];

			// Set sign for the soll angle value (0 negative / 1 positive)
			if (incoming_data[in_motorDir] == 0) digitalWrite(do_motorDirection1, LOW);
			else digitalWrite(do_motorDirection1, HIGH);

			// Send data (1. byte: soll_angle, 2. byte: save-action, 3. byte: speed)
			// Possible actions: 0: Nothing to do, 1: Save ref pos, 2: Save act pos
			i2c_data_out[0] = (byte)soll_motor_angle_temp;
			i2c_data_out[1] = 0;
			i2c_data_out[2] = soll_motorSpeed;

			//Serial.print("soll_motor_angle_temp: ");
			//if(incoming_data[in_motorDir] == 0) Serial.println(-soll_motor_angle_temp);
			//else Serial.println(soll_motor_angle_temp);

			Wire.beginTransmission(I2C_ID_PID_CONTROLLER);
			Wire.write(i2c_data_out, 3);
			Wire.endTransmission();
		}
	}
}

bool receiveData(byte rxStateIst, byte rxStateSoll)
{
	if ((rxStateIst & rxStateSoll) == 1) readControlValue(SPI_INSTRUCTION_READ_RX_BUFFER0, do_csMcp2515);
	else if ((rxStateIst & rxStateSoll) == 2) readControlValue(SPI_INSTRUCTION_READ_RX_BUFFER1, do_csMcp2515);
	return true;
}

void sendData(byte maxByte, byte buffer[]) {
	for (size_t i = 0; i < maxByte; i++) {
		mcp2515_load_tx_buffer0(buffer[i], i, maxByte);
	}
	mcp2515_execute_rts_command(0);
}

void blinkErrorCode(int error, int waitBefore, int waitAfter) {
	// Reset led state and wait some time
	if (waitBefore) setLedState(1000, 0, 0, LED_BUILTIN, LOW);

	// Set specific error code
	if (error = error_eeprom_act_pos)
	{
		setLedState(500, 0, 500, LED_BUILTIN, HIGH);
		for (size_t i = 0; i < 2; i++) setLedState(200, 0, 200, LED_BUILTIN, HIGH);
	}

	// Set specific error code
	if (error = error_eeprom_ref_pos)
	{
		setLedState(500, 0, 500, LED_BUILTIN, HIGH);
		for (size_t i = 0; i < 3; i++) setLedState(200, 0, 200, LED_BUILTIN, HIGH);
	}

	// Set specific error code
	if (error = error_eeprom_reset)
	{
		setLedState(500, 0, 500, LED_BUILTIN, HIGH);
		for (size_t i = 0; i < 4; i++) setLedState(200, 0, 200, LED_BUILTIN, HIGH);
	}

	// Reset led state and wait some time
	if (waitAfter) setLedState(1000, 0, 0, LED_BUILTIN, LOW);
}

void setLedState(int blinkLength, int pauseBefore, int pauseAfter, int led, int state) {
	delay(pauseBefore);
	digitalWrite(led, state);
	delay(blinkLength);
	digitalWrite(led, !state);
	delay(pauseAfter);
}
