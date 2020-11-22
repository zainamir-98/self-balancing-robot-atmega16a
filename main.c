{\rtf1\ansi\ansicpg1252\cocoartf2576
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 ArialMT;\f1\froman\fcharset0 Times-Roman;}
{\colortbl;\red255\green255\blue255;\red0\green0\blue0;}
{\*\expandedcolortbl;;\cssrgb\c0\c0\c0;}
\paperw11900\paperh16840\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\deftab720
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 \expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 /*
\f1\fs24 \

\f0\fs29\fsmilli14667 \'a0NUST SEECS BEE-9C FOURTH SEMESTER\'a0
\f1\fs24 \

\f0\fs29\fsmilli14667 \'a0MPS Project
\f1\fs24 \

\f0\fs29\fsmilli14667 \'a0SELF BALANCING ROBOT WITH ATMEGA16A
\f1\fs24 \

\f0\fs29\fsmilli14667 \'a0
\f1\fs24 \

\f0\fs29\fsmilli14667 \'a0NOTE: In I2C_Master_H_file.h line 11, F_CPU should be 1Mhz
\f1\fs24 \

\f0\fs29\fsmilli14667 */\'a0
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 // ** CONSTANTS ** //
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 #define F_CPU 1000000UL // Set clock frequency to 1MHz
\f1\fs24 \

\f0\fs29\fsmilli14667 #define GYRO_SETPOINT 180 // Must determine
\f1\fs24 \

\f0\fs29\fsmilli14667 #define UART_EN 1 // For debugging: allows ATmega to send MPU6050 values to PC via UART
\f1\fs24 \

\f0\fs29\fsmilli14667 #define TIMER1_PRESCALAR 8
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 // ** LIBRARIES ** //
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 #include <avr/io.h> // Include AVR std. library file
\f1\fs24 \

\f0\fs29\fsmilli14667 #include <util/delay.h> // Include delay header file
\f1\fs24 \

\f0\fs29\fsmilli14667 #include <inttypes.h>\'a0 /* Include integer type header file */
\f1\fs24 \

\f0\fs29\fsmilli14667 #include <stdlib.h>	 /* Include standard library file */
\f1\fs24 \

\f0\fs29\fsmilli14667 #include <stdio.h>\'a0 /* Include standard library file */
\f1\fs24 \

\f0\fs29\fsmilli14667 #include <math.h> // For math functions
\f1\fs24 \

\f0\fs29\fsmilli14667 #include "MPU6050_res_define.h"\'a0 /* Include MPU6050 register define file */
\f1\fs24 \

\f0\fs29\fsmilli14667 #include "I2C_Master_H_file.h"\'a0 /* Include I2C Master header file */
\f1\fs24 \

\f0\fs29\fsmilli14667 #include "pid.h" // Include PID library
\f1\fs24 \

\f0\fs29\fsmilli14667 //#include "USART_RS232_H_file.h"\'a0 /* Include USART header file */
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 // ** GLOBAL VARIABLES ** //
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;
\f1\fs24 \

\f0\fs29\fsmilli14667 float rad_to_deg = 180/3.141592654;
\f1\fs24 \

\f0\fs29\fsmilli14667 char buffer[20];
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 float Kp = 1;
\f1\fs24 \

\f0\fs29\fsmilli14667 float Ki = 0;
\f1\fs24 \

\f0\fs29\fsmilli14667 float Kd = 1;
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 // *** UART *** //
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 // Function to initialize UART communication
\f1\fs24 \

\f0\fs29\fsmilli14667 void UART_initializer() \{
\f1\fs24 \

\f0\fs29\fsmilli14667 	UBRRL = 0x0C; // set baud rate to 4800
\f1\fs24 \

\f0\fs29\fsmilli14667 	UCSRB |= (1<<TXEN) | (1<<RXEN); // enable transmitter and receiver
\f1\fs24 \

\f0\fs29\fsmilli14667 	UCSRC |= (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1); // set data size
\f1\fs24 \

\f0\fs29\fsmilli14667 \}
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 void UART_transmit_string() \{
\f1\fs24 \

\f0\fs29\fsmilli14667 	unsigned char i;
\f1\fs24 \

\f0\fs29\fsmilli14667 	for (i = 0; i < 20; i=i+1) \{
\f1\fs24 \

\f0\fs29\fsmilli14667 		while( !( UCSRA & (1<<UDRE) ) );
\f1\fs24 \

\f0\fs29\fsmilli14667 		UDR = buffer[i];
\f1\fs24 \

\f0\fs29\fsmilli14667 	\}
\f1\fs24 \

\f0\fs29\fsmilli14667 \}
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 // *** MPU6050 VIA I2C *** //
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 void MPU6050_Init()										/* Gyro initialization function */
\f1\fs24 \

\f0\fs29\fsmilli14667 \{
\f1\fs24 \

\f0\fs29\fsmilli14667 	_delay_ms(150);										/* Power up time >100ms */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Start_Wait(0xD0);								/* Start with device write address */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(0x07);									/* 1KHz sample rate */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Stop();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 	I2C_Start_Wait(0xD0);
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Stop();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 	I2C_Start_Wait(0xD0);
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(CONFIG);									/* Write to Configuration register */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(0x00);									/* Fs = 8KHz */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Stop();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 	I2C_Start_Wait(0xD0);
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Stop();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 	I2C_Start_Wait(0xD0);
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(0x01);
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Stop();
\f1\fs24 \

\f0\fs29\fsmilli14667 \}
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 void MPU_Start_Loc()
\f1\fs24 \

\f0\fs29\fsmilli14667 \{
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */\'a0
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
\f1\fs24 \

\f0\fs29\fsmilli14667 \}
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 void Read_RawValue()
\f1\fs24 \

\f0\fs29\fsmilli14667 \{
\f1\fs24 \

\f0\fs29\fsmilli14667 	MPU_Start_Loc(); // Read Gyro values
\f1\fs24 \

\f0\fs29\fsmilli14667 	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
\f1\fs24 \

\f0\fs29\fsmilli14667 	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
\f1\fs24 \

\f0\fs29\fsmilli14667 	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
\f1\fs24 \

\f0\fs29\fsmilli14667 	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
\f1\fs24 \

\f0\fs29\fsmilli14667 	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
\f1\fs24 \

\f0\fs29\fsmilli14667 	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
\f1\fs24 \

\f0\fs29\fsmilli14667 	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Stop();
\f1\fs24 \

\f0\fs29\fsmilli14667 \}
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 // ** MAIN FUNCTION ** //
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 int main() \{
\f1\fs24 \

\f0\fs29\fsmilli14667 	I2C_Init();	// Initialize I2C
\f1\fs24 \

\f0\fs29\fsmilli14667 	MPU6050_Init(); // Initialize MPU6050
\f1\fs24 \

\f0\fs29\fsmilli14667 	pid_setpid(Kp, Ki, Kd); // Initialize PID algorithm
\f1\fs24 \

\f0\fs29\fsmilli14667 	pid_setlimitsIerr(-255, 255);
\f1\fs24 \

\f0\fs29\fsmilli14667 	pid_setoutputlimits(-255, 255);
\f1\fs24 \

\f0\fs29\fsmilli14667 	
\f1\fs24 \

\f0\fs29\fsmilli14667 	if (UART_EN) UART_initializer();
\f1\fs24 \

\f0\fs29\fsmilli14667 	
\f1\fs24 \

\f0\fs29\fsmilli14667 	char float_[10];
\f1\fs24 \

\f0\fs29\fsmilli14667 	float Xa, Ya, Za, t, pid_input, pid_output = 0;
\f1\fs24 \

\f0\fs29\fsmilli14667 	float Xg = 0, Yg = 0, Zg = 0;
\f1\fs24 \

\f0\fs29\fsmilli14667 	float acc_angle_1, acc_angle_2, gyro_angle_1, gyro_angle_2;
\f1\fs24 \

\f0\fs29\fsmilli14667 	float total_angle_1, total_angle_2; // After applying filters
\f1\fs24 \

\f0\fs29\fsmilli14667 	float elapsed_time;
\f1\fs24 \

\f0\fs29\fsmilli14667 	
\f1\fs24 \

\f0\fs29\fsmilli14667 	// Prescalar: 8
\f1\fs24 \

\f0\fs29\fsmilli14667 	TCNT1 = 0;
\f1\fs24 \

\f0\fs29\fsmilli14667 	TCCR1A = 0x00;
\f1\fs24 \

\f0\fs29\fsmilli14667 	TCCR1B = 0x02;
\f1\fs24 \

\f0\fs29\fsmilli14667 	
\f1\fs24 \

\f0\fs29\fsmilli14667 	while(1) \{
\f1\fs24 \

\f0\fs29\fsmilli14667 		elapsed_time = TCNT1 * TIMER1_PRESCALAR / 1000000;
\f1\fs24 \

\f0\fs29\fsmilli14667 		TCNT1 = 0; // Restart timer
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		// Read raw values from MPU6050
\f1\fs24 \

\f0\fs29\fsmilli14667 		Read_RawValue();
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		// Divide raw values by sensitivity scale factor to get real values
\f1\fs24 \

\f0\fs29\fsmilli14667 		Xa = Acc_x/16384.0; // Get acceleration in g units
\f1\fs24 \

\f0\fs29\fsmilli14667 		Ya = Acc_y/16384.0;
\f1\fs24 \

\f0\fs29\fsmilli14667 		Za = Acc_z/16384.0;
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		Xg = Gyro_x/131.0; // Initially 16.4
\f1\fs24 \

\f0\fs29\fsmilli14667 		Yg = Gyro_y/131.0;
\f1\fs24 \

\f0\fs29\fsmilli14667 		Zg = Gyro_z/131.0;
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		//acc_angle_1 = atan((Ya)/sqrt(pow((Xa),2) + pow((Za),2)))*rad_to_deg;
\f1\fs24 \

\f0\fs29\fsmilli14667 		//acc_angle_2 = atan(-1*(Xa)/sqrt(pow((Ya),2) + pow((Za),2)))*rad_to_deg;
\f1\fs24 \

\f0\fs29\fsmilli14667 		//gyro_angle_1 = Xg;
\f1\fs24 \

\f0\fs29\fsmilli14667 		//gyro_angle_2 = Yg;
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		// Apply filters and get final angle
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		/*---X axis angle---*/
\f1\fs24 \

\f0\fs29\fsmilli14667 		//total_angle_1 = 0.98 *(total_angle_1 + gyro_angle_1*elapsed_time) + 0.02*acc_angle_1;
\f1\fs24 \

\f0\fs29\fsmilli14667 		/*---Y axis angle---*/
\f1\fs24 \

\f0\fs29\fsmilli14667 		//total_angle_2 = 0.98 *(total_angle_2 + gyro_angle_2*elapsed_time) + 0.02*acc_angle_2;
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		// PID
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		pid_input = Xa * 180/M_PI + GYRO_SETPOINT;
\f1\fs24 \

\f0\fs29\fsmilli14667 		pid_output = pid_update(GYRO_SETPOINT, pid_input, 1); // Compute PID and assign to output
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		// Debug
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		t = (Temperature/340.00)+36.53; // Convert to degrees Celsius
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 		if (UART_EN) \{
\f1\fs24 \

\f0\fs29\fsmilli14667 			dtostrf( Xa, 3, 2, float_ );					/* Take values in buffer to send all parameters over USART */
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," Ax = %s g\\t",float_);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 			dtostrf( Ya, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," Ay = %s g\\t",float_);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \

\f0\fs29\fsmilli14667 			
\f1\fs24 \

\f0\fs29\fsmilli14667 			dtostrf( Za, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," Az = %s g\\t",float_);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 			dtostrf( t, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," T = %s%cC\\t",float_,0xF8); \'a0 \'a0 \'a0 \'a0 \'a0 /* 0xF8 Ascii value of degree '\'b0' on serial */
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \

\f0\fs29\fsmilli14667 			
\f1\fs24 \

\f0\fs29\fsmilli14667 			dtostrf( pid_input, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," PID input = %s\\t",float_,0xF8);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \

\f0\fs29\fsmilli14667 			
\f1\fs24 \

\f0\fs29\fsmilli14667 			dtostrf( pid_output, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," PID output = %s\\t",float_,0xF8);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 			dtostrf( Xg, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," Gx = %s%c/s\\t",float_,0xF8);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 			dtostrf( Yg, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," Gy = %s%c/s\\t",float_,0xF8);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \

\f0\fs29\fsmilli14667 			
\f1\fs24 \

\f0\fs29\fsmilli14667 			dtostrf( Zg, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," Gz = %s%c/s\\r\\n",float_,0xF8);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \
\pard\pardeftab720\partightenfactor0
\cf2 \
\pard\pardeftab720\sl400\partightenfactor0

\f0\fs29\fsmilli14667 \cf2 			
\f1\fs24 \

\f0\fs29\fsmilli14667 			sprintf(buffer," Time elapsed = %d /s\\r\\n",TCNT1);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			UART_transmit_string();
\f1\fs24 \

\f0\fs29\fsmilli14667 			
\f1\fs24 \

\f0\fs29\fsmilli14667 			//dtostrf( total_angle_1, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			//sprintf(buffer," Angle 1 (x-axis) = %s%c/s\\r\\n",float_,0xF8);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//UART_transmit_string();
\f1\fs24 \

\f0\fs29\fsmilli14667 			//
\f1\fs24 \

\f0\fs29\fsmilli14667 			//dtostrf( total_angle_2, 3, 2, float_ );
\f1\fs24 \

\f0\fs29\fsmilli14667 			//sprintf(buffer," Angle 2 (y-axis) = %s%c/s\\r\\n",float_,0xF8);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//USART_SendString(buffer);
\f1\fs24 \

\f0\fs29\fsmilli14667 			//UART_transmit_string();
\f1\fs24 \

\f0\fs29\fsmilli14667 		\}
\f1\fs24 \

\f0\fs29\fsmilli14667 		
\f1\fs24 \

\f0\fs29\fsmilli14667 		//_delay_ms(500); // Must change
\f1\fs24 \

\f0\fs29\fsmilli14667 	\}
\f1\fs24 \

\f0\fs29\fsmilli14667 \}
\f1\fs24 \
}