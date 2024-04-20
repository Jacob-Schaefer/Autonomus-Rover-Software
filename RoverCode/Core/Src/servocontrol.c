/*
 * servocontrol.c
 *
 *  Created on: Apr 8, 2024
 *      Author: bjbiv
 */

#include <math.h>

#include "servocontrol.h"
#include "main.h"

/* Defines given in MM */
#define WHEELBASE 400
#define WHEELWIDTH 250
#define RADIUSMAXLIMIT 2000 // At what radius do i set the wheels to straight (0 degrees)

/* Defines for Servo PWM */
#define SERVOPWMMAX 1250
#define SERVOPWMMIN 250

/* defines for Motor PWM */
#define MOTOR_MAX_PWM 3359
#define MOTOR_MIN_PWM 400

//For turning, set a max centripetal acceleration
#define A_C 900 // given in mm/s^s
#define STRAIGHT_V 1400 // given in mm/s
#define MAX_V 2300 // given in mm/s

#define PI 3.141592654

float servoVert = (WHEELBASE / 2);
float innerTheta, innerAngle, innerHor;
float outerTheta, outerAngle, outerHor;
float Rref, Vref;
float motor1Radius, motor2Radius, motor3Radius, motor4Radius;

/*
 * outerHor = radius + ( WHEELWIDTH / 2 );
 * outerTheta = atan( outerHor / servoVert );
 * servo1Angle = (( outerTheta * 180 ) / PI);
 */
void driveServos( int radius )
{
	if( ( radius < -RADIUSMAXLIMIT) || ( radius > RADIUSMAXLIMIT ))
	{
		setServo( 1, 90);
		setServo( 2, 90);
		setServo( 3, 90);
		setServo( 4, 90);
		// set motors
		setMotor( 1, STRAIGHT_V, 1 );
		setMotor( 2, STRAIGHT_V, 1 );
		setMotor( 3, STRAIGHT_V, 1 );
		setMotor( 4, STRAIGHT_V, 1 );
	}
	else
	{
		if( radius < ( 0 - (WHEELWIDTH / 2)))
		{
			//Servp 1 and 2 (Inner Wheels)
			int absRadius = abs( radius );
			innerHor = absRadius - ( WHEELWIDTH / 2 );
			innerTheta = atan( innerHor / servoVert );
			innerAngle = (( innerTheta * 180 ) / PI);
			setServo( 1, innerAngle); //Set servo 1
			setServo( 2, ( 180 - innerAngle ));

			//Servo 3 and 4 (Outer Wheel)
			outerHor = absRadius + ( WHEELWIDTH / 2 );
			outerTheta = atan( outerHor / servoVert );
			outerAngle = (( outerTheta * 180 ) / PI);
			setServo( 3, ( 180 - outerAngle ));
			setServo( 4, outerAngle);

			// set motor driver 2
			Rref = sqrt( (pow( outerHor, 2) + pow( (WHEELBASE / 2), 2) ) );
			Vref = sqrt( (A_C * Rref ) );

			int motor2Velo = Vref;

			// motor driver 4
			motor4Radius = outerHor;
			int motor4Velo = (Vref * motor4Radius) / Rref;

			// set motor driver 1
			motor1Radius = sqrt( (pow( innerHor, 2) + pow( (WHEELBASE / 2), 2) ) );
			int motor1Velo = (Vref * motor1Radius) / Rref;

			// motor driver 3
			motor3Radius = innerHor;
			int motor3Velo = (Vref * motor3Radius) / Rref;

			setMotor( 1, motor1Velo, 1 );
			setMotor( 2, motor2Velo, 1 );
			setMotor( 3, motor3Velo, 1 );
			setMotor( 4, motor4Velo, 1 );
		}
		else if( (radius > ( 0 - (WHEELWIDTH /2))) && (radius < 0) )
		{
			//Servp 1 and 2 (Inner Wheels)
			int absRadius = abs( radius );
			innerHor = ( WHEELWIDTH / 2 ) - absRadius;
			innerTheta = atan( innerHor / servoVert );
			innerAngle = (( innerTheta * 180 ) / PI);
			setServo( 1, ( 180 - innerAngle ));
			setServo( 2, innerAngle);

			//Servo 3 and 4 (Outer Wheel)
			outerHor = ( WHEELWIDTH / 2 ) + absRadius;
			outerTheta = atan( outerHor / servoVert );
			outerAngle = (( outerTheta * 180 ) / PI);
			setServo( 3, ( 180 - outerAngle ));
			setServo( 4, outerAngle);

			// find outside wheel radius as Rref and Vref
			Rref = sqrt( (pow( outerHor, 2) + pow( (WHEELBASE / 2), 2) ) );
			Vref = sqrt( (A_C * Rref ) );

			int motor2Velo = Vref;

			// motor driver 4
			motor4Radius = outerHor;
			int motor4Velo = (Vref * motor4Radius) / Rref;

			// set motor driver 1
			motor1Radius = sqrt( (pow( innerHor, 2) + pow( (WHEELBASE / 2), 2) ) );
			int motor1Velo = (Vref * motor1Radius) / Rref;

			// motor driver 3
			motor3Radius = innerHor;
			int motor3Velo = (Vref * motor3Radius) / Rref;

			setMotor( 1, motor1Velo, 0 );
			setMotor( 2, motor2Velo, 1 );
			setMotor( 3, motor3Velo, 0 );
			setMotor( 4, motor4Velo, 1 );
		}
		/*
		else if( radius == 0 )
		{
			//Servp 1 and 2
			setServo( 1, 32); //Set servo 1
			setServo( 2, 148);

			//Servo 3 and 4 (Outer Wheel)
			setServo( 3, 32);
			setServo( 4, 148);

			// set motor drivers 3 and 4 first
			// motor driver 3
			setMotor( 3, TURNING_V, direction );

			// motor driver 4
			setMotor( 4, TURNING_V, direction );

			// set motor driver 1
			setMotor( 1, TURNING_V, direction );

			// set motor driver 2
			setMotor( 2, TURNING_V, direction );
		}
		*/
		else if( (radius < (WHEELWIDTH /2)) && (radius > 0) )
		{
			//Servp 3 and 4 (Inner Wheels)
			innerHor = ( WHEELWIDTH / 2 ) - radius;
			innerTheta = atan( innerHor / servoVert );
			innerAngle = (( innerTheta * 180 ) / PI);
			setServo( 3, ( 180 - innerAngle ));
			setServo( 4, innerAngle);

			//Servo 1 and 2 (Outer Wheel)
			outerHor = ( WHEELWIDTH / 2 ) + radius;
			outerTheta = atan( outerHor / servoVert );
			outerAngle = (( outerTheta * 180 ) / PI);
			setServo( 1, ( 180 - outerAngle ));
			setServo( 2, outerAngle);

			// find outside wheel radius as Rref and Vref
			Rref = sqrt( (pow( outerHor, 2) + pow( (WHEELBASE / 2), 2) ) );
			Vref = sqrt( (A_C * Rref ) );

			int motor1Velo = Vref;

			// motor driver 4
			motor3Radius = outerHor;
			int motor3Velo = (Vref * motor3Radius) / Rref;

			// set motor driver 1
			motor2Radius = sqrt( (pow( innerHor, 2) + pow( (WHEELBASE / 2), 2) ) );
			int motor2Velo = (Vref * motor2Radius) / Rref;

			// motor driver 3
			motor4Radius = innerHor;
			int motor4Velo = (Vref * motor4Radius) / Rref;

			setMotor( 1, motor1Velo, 1 );
			setMotor( 2, motor2Velo, 0 );
			setMotor( 3, motor3Velo, 1 );
			setMotor( 4, motor4Velo, 0 );
		}
		else if ( radius > (WHEELWIDTH /2) )
		{
			//Servo 3 and 4 (Inner Wheel)
			innerHor =  radius - ( WHEELWIDTH / 2 );
			innerTheta = atan( innerHor / servoVert );
			innerAngle = (( innerTheta * 180 ) / PI);
			setServo( 3, innerAngle);
			setServo( 4, ( 180 - innerAngle ));

			//Servp 1 and 2 (Outer Wheels)
			outerHor = ( WHEELWIDTH / 2 ) + radius;
			outerTheta = atan( outerHor / servoVert );
			outerAngle = (( outerTheta * 180 ) / PI);
			setServo( 1, ( 180 - outerAngle ));
			setServo( 2, outerAngle);

			// find outside wheel radius as Rref and Vref
			Rref = sqrt( (pow( outerHor, 2) + pow( (WHEELBASE / 2), 2) ) );
			Vref = sqrt( (A_C * Rref ) );

			int motor1Velo = Vref;

			// motor driver 4
			motor3Radius = outerHor;
			int motor3Velo = (Vref * motor3Radius) / Rref;

			// set motor driver 1
			motor2Radius = sqrt( (pow( innerHor, 2) + pow( (WHEELBASE / 2), 2) ) );
			int motor2Velo = (Vref * motor2Radius) / Rref;

			// motor driver 3
			motor4Radius = innerHor;
			int motor4Velo = (Vref * motor4Radius) / Rref;

			setMotor( 1, motor1Velo, 1 );
			setMotor( 2, motor2Velo, 1 );
			setMotor( 3, motor3Velo, 1 );
			setMotor( 4, motor4Velo, 1 );
		}
	}
}



void setServo( int servoNum, float angle)
{
	switch (servoNum)
	{
	case 1:
		//code for servo 1
		int servo1PWM = map( angle, 0, 180, SERVOPWMMIN, SERVOPWMMAX );
		TIM3->CCR1 = servo1PWM;
		break;
	case 2:
		//code for servo 2
		int servo2PWM = map( angle, 0, 180, SERVOPWMMIN, SERVOPWMMAX);
		TIM3->CCR2 = servo2PWM;
		break;
	case 3:
		//code for servo 3
		int servo3PWM = map( angle, 0, 180, SERVOPWMMIN, SERVOPWMMAX);
		TIM3->CCR3 = servo3PWM;
		break;
	case 4:
		//code for servo 4
		int servo4PWM = map( angle, 0, 180, SERVOPWMMIN, SERVOPWMMAX);
		TIM3->CCR4 = servo4PWM;
		break;
	default:
		break;
	}

}

void setMotor( int motorNum, float power, int direction )
{
	switch (motorNum)
	{
	case 1:
		//code for motor driver 1
		int motor1PWM = map( power, 0, MAX_V, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
		if( direction == 1 ) //forward
		{
			// set other to 0
			TIM2->CCR1 = 0;
			TIM1->CCR1 = motor1PWM;
		}
		else // backward
		{
			// set other to 0
			TIM1->CCR1 = 0;
			TIM2->CCR1 = motor1PWM;
		}

		break;
	case 2:
		//code for motor driver 2
		int motor2PWM = map( power, 0, MAX_V, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
		if( direction == 1 ) //forward
		{
			//set other to 0
			TIM1->CCR2 = 0;
			TIM2->CCR2 = motor2PWM;
		}
		else // backward
		{
			// set other to 0
			TIM2->CCR2 = 0;
			TIM1->CCR2 = motor2PWM;

		}
		break;
	case 3:
		//code for motor driver 3
		int motor3PWM = map( power, 0, MAX_V, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
		if( direction == 1 ) //forward
		{
			// set other to 0
			TIM2->CCR3 = 0;
			TIM1->CCR3 = motor3PWM;
		}
		else // backward
		{
			//set other to 0
			TIM1->CCR3 = 0;
			TIM2->CCR3 = motor3PWM;

		}
		break;
	case 4:
		//code for motor driver 4
		int motor4PWM = map( power, 0, MAX_V, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
		if( direction == 1 ) //forward
		{
			//set other to 0
			TIM1->CCR4 = 0;
			TIM2->CCR4 = motor4PWM;

		}
		else // backward
		{
			// set other to 0
			TIM2->CCR4 = 0;
			TIM1->CCR4 = motor4PWM;

		}
		break;
	default:
		break;
	}
}
