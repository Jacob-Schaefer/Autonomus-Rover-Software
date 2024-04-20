/*
 * servocontrol.h
 *
 *  Created on: Apr 8, 2024
 *      Author: bjbiv
 */

#ifndef INC_SERVOCONTROL_H_
#define INC_SERVOCONTROL_H_

void driveServos( int radius );
void setServo( int servoNum, float angle );
void setMotor( int motorNum, float power, int direction );


#endif /* INC_SERVOCONTROL_H_ */
