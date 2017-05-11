#pragma once
// -----------------------------------------
// PID CONTROLLER DATA
// -----------------------------------------

// -----------------------------------------
// Premium gear motor (638810 - 12rpm)
// -----------------------------------------
// VERY SLOW
//double P = -1.46749932499104;
//double I = -0.115013862304265;
//double D = 0.548945887967433;
//double N = 0.862662739489541;

// FAST AS POSSIBLE (BEST RESULTS)
//double P = -60.0661737867048;
//double I = -4.94953312365104;
//double D = -0.454950956464534;
//double N = 3.15338115327649;

//const double ENCODER_TO_DEGREE = ((double)360 / 34608);

// TEST FOR FASTER RESPONSE AT 30deg SOLLVALUE
//double P = -6.44036045007237;
//double I = -2.12725533466775;
//double D = 0.0227955275430827;
//double N = 3.51293845336784;
// -----------------------------------------
// -----------------------------------------


// -----------------------------------------
// Econ gear motor (638340 - 19rpm)
// -----------------------------------------
// FAST AS POSSIBLE
//double P = -12.8539033109958;
//double I = -0.712646331260047;
//double D = 0.343845666242764;
//double N = 5.46914538556796;

// PID MOTOR BIG
//double P = -10.6155510552541;
//double I = -0.487958012814397;
//double D = 0.312079469943773;
//double N = 4.6424770619278;

// PID MOTOR SMALL
double P = -1.5769363355766;
double I = -1.80574105020461;
double D = 0.816663126382965;
double N = 4.351022616953;

// -----------------------------------------
// -----------------------------------------
const double ENCODER_TO_DEGREE = 1;
double pid_error;
double current_motor_angle = 0;
double p_term = 0;
double i_term = 0;
double d_term = 0;
double preSat = 0;
double d_filter = 0;

double const SAMPLE_TIME = 0.05; // s
double const UPPER_SATURATION_LIMIT = 255;
double const LOWER_SATURATION_LIMIT = -255;
double const MIN_PID_ERROR = 0.15; // Big motor
//double const MIN_PID_ERROR = 3; // Small motor



int pwm_motorDirection, taskNumber;