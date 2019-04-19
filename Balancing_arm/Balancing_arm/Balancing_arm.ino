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

const float KP = 3.55;
const float KI = 0.005;
const float KD = 2.05;
const int DEADZONE = 128;

void TaskGyroscope( void *pvParameters );
void TaskESC( void *pvParameters );

volatile float desired_angle = 0;
float rad_to_deg = 180/3.141592654;
float total_angle_x = 0;
float total_angle_y = 0;
float previous_error;
float PID_p;
float PID_i;
float PID_d;

void setup() {
	Serial.begin(115200);

	xTaskCreate(
	TaskIMU,
	(const portCHAR *)"IMU",
	384,  // Stack size
	NULL,
	1,  // priority
	NULL
	);

	xTaskCreate(
	TaskSerial,
	(const portCHAR *)"Serial",
	128,
	NULL,
	2,
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

	int val = 0;
	
	vTaskDelay( 1000 / portTICK_PERIOD_MS );

	for(;;) {
		if (Serial.available() > 0) {
			char inputBuffer[16];
			Serial.readBytes(inputBuffer, sizeof(inputBuffer));
			val = atoi(inputBuffer);
			if (val < -5) val = -5;
			if (val > 70) val = 70;
			desired_angle = val;
			Serial.print("Desired angle: "); Serial.println(desired_angle);
			Serial.flush();
		}

		vTaskDelay( 500 / portTICK_PERIOD_MS );
	}
}

void TaskIMU(void *pvParameters) {
    (void) pvParameters;
	
	Servo ESC;

	ESC.attach(9);
	ESC.writeMicroseconds(1000);

    L3G4200D gyroscope;
    Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

    Serial.println("HALLOOOOO");

    unsigned long last_time = millis();

    while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50)) {
	    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
	    vTaskDelay( 500 / portTICK_PERIOD_MS ); // wait for one second
    }

    while (!accel.begin()) {
	    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
	    vTaskDelay( 500 / portTICK_PERIOD_MS );
    }

    gyroscope.calibrate(100);
    accel.setRange(ADXL345_RANGE_16_G);
    

    for (;;) {
	    sensors_event_t event;

	    Vector norm = gyroscope.readNormalize();
	    accel.getEvent(&event);
	    
	    unsigned long timer = millis();
	    float timeStep = (float)(timer-last_time)/1000;
	    last_time = timer;
	    
		//Calculate angles from accelerometer data using Euler formula
		float angle_a_x = atan((event.acceleration.y)/sqrt(pow((event.acceleration.x),2) + pow((event.acceleration.z),2)))*rad_to_deg;
		float angle_a_y = atan(-1*(event.acceleration.x)/sqrt(pow((event.acceleration.y),2) + pow((event.acceleration.z),2)))*rad_to_deg;

		//Complementary filter to combine gyroscope and accelerometer angles
		float total_angle_x = 0.98 *(total_angle_x + (norm.XAxis * timeStep)) + 0.02*angle_a_x;
		float total_angle_y = 0.98 *(total_angle_y + (norm.YAxis * timeStep)) + 0.02*angle_a_y;
		
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
		
		ESC.writeMicroseconds(1000 + DEADZONE + PID);
		
		/**Serial.print("Desired angle: "); Serial.print(desired_angle);
	    Serial.print(" Accel: x: "); Serial.print(angle_a_x);
	    Serial.print("  y: ");Serial.print(angle_a_y);
		Serial.print(" Total angle: X: "); Serial.print(total_angle_x);
		Serial.print(" Y: "); Serial.print(total_angle_y);
		Serial.print(" Throttle value (PID): "); Serial.println(PID);*/
		
		previous_error = error;
	    vTaskDelay( 20 / portTICK_PERIOD_MS );
    }
}
