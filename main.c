/*
 NUST SEECS BEE-9C FOURTH SEMESTER 
 MPS Project
 SELF BALANCING ROBOT WITH ATMEGA16A
 
 NOTE: In I2C_Master_H_file.h line 11, F_CPU should be 1Mhz
*/ 

// ** CONSTANTS ** //

#define F_CPU 1000000UL // Set clock frequency to 1MHz
#define GYRO_SETPOINT 180 // Must determine
#define UART_EN 1 // For debugging: allows ATmega to send MPU6050 values to PC via UART
#define TIMER1_PRESCALAR 8

// ** LIBRARIES ** //

#include <avr/io.h> // Include AVR std. library file
#include <util/delay.h> // Include delay header file
#include <inttypes.h>  /* Include integer type header file */
#include <stdlib.h>	 /* Include standard library file */
#include <stdio.h>  /* Include standard library file */
#include <math.h> // For math functions
#include "MPU6050_res_define.h"  /* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"  /* Include I2C Master header file */
#include "pid.h" // Include PID library
//#include "USART_RS232_H_file.h"  /* Include USART header file */

// ** GLOBAL VARIABLES ** //

float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;
float rad_to_deg = 180/3.141592654;
char buffer[20];

float Kp = 1;
float Ki = 0;
float Kd = 1;

// *** UART *** //

// Function to initialize UART communication
void UART_initializer() {
	UBRRL = 0x0C; // set baud rate to 4800
	UCSRB |= (1<<TXEN) | (1<<RXEN); // enable transmitter and receiver
	UCSRC |= (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1); // set data size
}

void UART_transmit_string() {
	unsigned char i;
	for (i = 0; i < 20; i=i+1) {
		while( !( UCSRA & (1<<UDRE) ) );
		UDR = buffer[i];
	}
}

// *** MPU6050 VIA I2C *** //

void MPU6050_Init()										/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */ 
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void Read_RawValue()
{
	MPU_Start_Loc(); // Read Gyro values
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
}

// ** MAIN FUNCTION ** //

int main() {
	I2C_Init();	// Initialize I2C
	MPU6050_Init(); // Initialize MPU6050
	pid_setpid(Kp, Ki, Kd); // Initialize PID algorithm
	pid_setlimitsIerr(-255, 255);
	pid_setoutputlimits(-255, 255);
	
	if (UART_EN) UART_initializer();
	
	char float_[10];
	float Xa, Ya, Za, t, pid_input, pid_output = 0;
	float Xg = 0, Yg = 0, Zg = 0;
	float acc_angle_1, acc_angle_2, gyro_angle_1, gyro_angle_2;
	float total_angle_1, total_angle_2; // After applying filters
	float elapsed_time;
	
	// Prescalar: 8
	TCNT1 = 0;
	TCCR1A = 0x00;
	TCCR1B = 0x02;
	
	while(1) {
		elapsed_time = TCNT1 * TIMER1_PRESCALAR / 1000000;
		TCNT1 = 0; // Restart timer
		
		// Read raw values from MPU6050
		Read_RawValue();
		
		// Divide raw values by sensitivity scale factor to get real values
		Xa = Acc_x/16384.0; // Get acceleration in g units
		Ya = Acc_y/16384.0;
		Za = Acc_z/16384.0;
		
		Xg = Gyro_x/131.0; // Initially 16.4
		Yg = Gyro_y/131.0;
		Zg = Gyro_z/131.0;
		
		//acc_angle_1 = atan((Ya)/sqrt(pow((Xa),2) + pow((Za),2)))*rad_to_deg;
		//acc_angle_2 = atan(-1*(Xa)/sqrt(pow((Ya),2) + pow((Za),2)))*rad_to_deg;
		//gyro_angle_1 = Xg;
		//gyro_angle_2 = Yg;
		
		// Apply filters and get final angle
		
		/*---X axis angle---*/
		//total_angle_1 = 0.98 *(total_angle_1 + gyro_angle_1*elapsed_time) + 0.02*acc_angle_1;
		/*---Y axis angle---*/
		//total_angle_2 = 0.98 *(total_angle_2 + gyro_angle_2*elapsed_time) + 0.02*acc_angle_2;
		
		// PID
		
		pid_input = Xa * 180/M_PI + GYRO_SETPOINT;
		pid_output = pid_update(GYRO_SETPOINT, pid_input, 1); // Compute PID and assign to output
		
		// Debug
		
		t = (Temperature/340.00)+36.53; // Convert to degrees Celsius

		if (UART_EN) {
			dtostrf( Xa, 3, 2, float_ );					/* Take values in buffer to send all parameters over USART */
			sprintf(buffer," Ax = %s g\t",float_);
			//USART_SendString(buffer);
			UART_transmit_string();

			dtostrf( Ya, 3, 2, float_ );
			sprintf(buffer," Ay = %s g\t",float_);
			//USART_SendString(buffer);
			UART_transmit_string();
			
			dtostrf( Za, 3, 2, float_ );
			sprintf(buffer," Az = %s g\t",float_);
			//USART_SendString(buffer);
			UART_transmit_string();

			dtostrf( t, 3, 2, float_ );
			sprintf(buffer," T = %s%cC\t",float_,0xF8);           /* 0xF8 Ascii value of degree '°' on serial */
			//USART_SendString(buffer);
			UART_transmit_string();
			
			dtostrf( pid_input, 3, 2, float_ );
			sprintf(buffer," PID input = %s\t",float_,0xF8);
			//USART_SendString(buffer);
			UART_transmit_string();
			
			dtostrf( pid_output, 3, 2, float_ );
			sprintf(buffer," PID output = %s\t",float_,0xF8);
			//USART_SendString(buffer);
			UART_transmit_string();

			dtostrf( Xg, 3, 2, float_ );
			sprintf(buffer," Gx = %s%c/s\t",float_,0xF8);
			//USART_SendString(buffer);
			UART_transmit_string();

			dtostrf( Yg, 3, 2, float_ );
			sprintf(buffer," Gy = %s%c/s\t",float_,0xF8);
			//USART_SendString(buffer);
			UART_transmit_string();
			
			dtostrf( Zg, 3, 2, float_ );
			sprintf(buffer," Gz = %s%c/s\r\n",float_,0xF8);
			//USART_SendString(buffer);
			UART_transmit_string();

			
			sprintf(buffer," Time elapsed = %d /s\r\n",TCNT1);
			//USART_SendString(buffer);
			UART_transmit_string();
			
			//dtostrf( total_angle_1, 3, 2, float_ );
			//sprintf(buffer," Angle 1 (x-axis) = %s%c/s\r\n",float_,0xF8);
			//USART_SendString(buffer);
			//UART_transmit_string();
			//
			//dtostrf( total_angle_2, 3, 2, float_ );
			//sprintf(buffer," Angle 2 (y-axis) = %s%c/s\r\n",float_,0xF8);
			//USART_SendString(buffer);
			//UART_transmit_string();
		}
		
		//_delay_ms(500); // Must change
	}
}
