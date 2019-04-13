#include <Arduino_FreeRTOS.h>
#include <L3G4200D.h>
#include <Wire.h>
#include <Servo.h>

void TaskGyroscope( void *pvParameters );
void TaskESC( void *pvParameters );

volatile float pitch = 0;
volatile float roll = 0;
volatile float yaw = 0;

void setup() {
    Serial.begin(115200);

    xTaskCreate(
        TaskGyroscope,
        (const portCHAR *)"Gyroscope",
        256,  // Stack size
        NULL,
        1,  // priority
        NULL 
    );

    xTaskCreate(
        TaskESC,
        (const portCHAR *)"ESC",
        256,
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

void TaskESC(void *pvParameters) {
    (void) pvParameters;

    int val = 0;

    Servo myServo;

    myServo.attach(9, 1000, 2000);
    myServo.write(val);
    
    vTaskDelay( 1000 / portTICK_PERIOD_MS );

    for(;;) {
        if (Serial.available() > 0) {
            val = Serial.parseInt();
            Serial.parseInt();
            if (val < 0) val = 0;
            if (val > 180) val = 180;
            //Serial.flush();
            myServo.write(val);
        }

        vTaskDelay( 30 / portTICK_PERIOD_MS );
    }
}

void TaskGyroscope(void *pvParameters) {
    (void) pvParameters;

    L3G4200D gyroscope;


    Serial.println("HALLOOOOO");

    unsigned long last_time = millis();

    while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50)) {
        Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
        vTaskDelay( 500 / portTICK_PERIOD_MS ); // wait for one second
    }

    gyroscope.calibrate(100);

    for (;;) {
        Vector norm = gyroscope.readNormalize();
        
        unsigned long timer = millis();
        float timeStep = (float)(timer-last_time)/1000;
        last_time = timer;
        
        pitch = pitch + norm.YAxis * timeStep;
        roll = roll + norm.XAxis * timeStep;
        yaw = yaw + norm.ZAxis * timeStep;

        Serial.print(" Pitch = ");
        Serial.print(pitch);
        Serial.print(" Roll = ");
        Serial.print(roll);  
        Serial.print(" Yaw = ");
        Serial.println(yaw);

        vTaskDelay( 2 / portTICK_PERIOD_MS );
    }
}
