/*
 * Balancing_arm.ino
 *
 * Created: 4/19/2019 3:18:21 PM
 * Author: piete
 */ 

#include <Arduino_FreeRTOS.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <L3G4200D.h>
#include <Wire.h>
#include <Servo.h>

#define RAD_TO_DEG (180/PI)
#define DEADZONE 128

float KP = 3.55;
float KI = 0.005;
float KD = 2.05;

float desired_angle = 0;
float total_angle_x = 0;
//float total_angle_y = 0;

//float angle_a_x = 0;
//float angle_a_y = 0;

bool armed = false;

float PID_p;
float PID_i;
float PID_d;
float previous_error;

void setup() {
	Serial.begin(9600);

	xTaskCreate(
	TaskIMU,
	(const portCHAR *)"IMU",
	300,  // Stack size
	NULL,
	2,  // priority
	NULL
	);

	xTaskCreate(
	TaskSerial,
	(const portCHAR *)"Serial",
	128,
	NULL,
	1,
	NULL
	);
}

void loop() {

}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSerial(void *pvParameters) {
	(void) pvParameters;

	vTaskDelay( 1000 / portTICK_PERIOD_MS );

	for(;;) {
		if (Serial.available() > 0) {
			char character = Serial.read();
			
			float value;
			switch (character)
			{
				case 'O':
					if (Serial.read() != '/') break;
					armed = !armed;
					if (armed) {
						Serial.println("The balancing arm is now armed!");
					} else {
						Serial.println("The balancing arm is now disarmed!");
					}
					vTaskDelay( 1000 / portTICK_PERIOD_MS );
					break;
				case 'A':
					value = Serial.parseFloat();
					if (Serial.read() != '/') break;
					Serial.print("Changing desired angle to: "); Serial.println(value);
					if (value < -5) value = -5;
					if (value > 70) value = 70;
					desired_angle = value;
					vTaskDelay( 1000 / portTICK_PERIOD_MS );
					break;
				case 'P':
					value = Serial.parseFloat();
					if (Serial.read() != '/') break;
					Serial.print("Changing P gain to: "); Serial.println(value);
					KP = value;	
					vTaskDelay( 1000 / portTICK_PERIOD_MS );
					break;
				case 'I':
					value = Serial.parseFloat();
					if (Serial.read() != '/') break;
					Serial.print("Changing I gain to: "); Serial.println(value);
					KI = value;
					vTaskDelay( 1000 / portTICK_PERIOD_MS );
					break;
				case 'D':
					value = Serial.parseFloat();
					if (Serial.read() != '/') break;
					Serial.print("Changing D gain to: "); Serial.println(value);
					KD = value;
					vTaskDelay( 1000 / portTICK_PERIOD_MS );
					break;
					
			}
			Serial.flush();
		}
		
		Serial.print("Desired angle: "); Serial.print(desired_angle);
		Serial.print(" current angle: "); Serial.println(total_angle_x);
		vTaskDelay(1);
	}
}

void TaskIMU(void *pvParameters) {
    (void) pvParameters;
	
	Servo ESC;

	ESC.attach(9);
	ESC.writeMicroseconds(1000);

    L3G4200D gyroscope;
    Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

    unsigned long last_time = 0;

    while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_100HZ_12_5)) {
	    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
	    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }

    while (!accel.begin()) {
	    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
	    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }

    gyroscope.calibrate(100);

    for (;;) {
	    sensors_event_t event;

	    Vector norm = gyroscope.readNormalize();
	    accel.getEvent(&event);
	    
	    unsigned long timer = millis();
	    float timeStep = (float)(timer-last_time)/1000;
	    
		//Calculate angles from accelerometer data using Euler formula
		float angle_a_x = event.orientation.y*9; 
		//value is too small by a factor of 9 for some reason, should normally return 
		//orientation of y axis in degrees

		//Complementary filter to combine gyroscope and accelerometer angles
		total_angle_x = 0.98 *(total_angle_x + (norm.XAxis * timeStep)) + 0.02*angle_a_x;
		//total_angle_y = 0.98 *(total_angle_y + (norm.YAxis * timeStep)) + 0.02*angle_a_y;
		
		//float error = total_angle_x - desired_angle;
		float error = total_angle_x - desired_angle;
		
		PID_p = KP*error;
		
		if (-3 < error < 3) {
			PID_i = PID_i+(KI*error); 
		}
		
		PID_d = KD*((error - previous_error)/timeStep);
		
		float PID = PID_p + PID_i + PID_d;
		PID = -PID;
		
		if (PID < 0) PID = 0;
		if (PID > 1000) PID = 1000;
		
		if (armed) {
			ESC.writeMicroseconds(1000 + DEADZONE + PID);
		} else {
			ESC.writeMicroseconds(1000);
		}
		
		previous_error = error;
		last_time = millis();
	    vTaskDelay(1);
    }
}
