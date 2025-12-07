/* 
 * An MPU6050 program to send YAW/BEARING/HEADING data
 * through UART communication.
 * 
 * Microcontroller: Arduino Nano
 * Sensor         : MPU6050
 * Library link   : https://github.com/ElectronicCats/mpu6050
 * 
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (DEFAULT)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN       2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN_RED         8 
#define LED_PIN_GREEN       5
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define MINIMUM_ERROR      0.001
#define TIME_PERIOD_MS     1000 
#define MAX_CALIBRATE_CNT  7
long int t_now;
long int t_prev;
float yaw_now;
float yaw_prev;
float dYaw;
char calibrate_cnt;
float yaw_data;
float pitch_data;
float roll_data;
char send_ypr_data[15] = "ABC";
char calibrate_status = 0;
float offset_degree;

unsigned long int ledNow = 0;
unsigned long int ledPrev = 0;
int led_state = 0;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial); 
    delay(50);
    
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    delay(50);
    
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(LED_PIN_RED, OUTPUT);
    pinMode(LED_PIN_GREEN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    digitalWrite(LED_PIN_RED, HIGH);
    digitalWrite(LED_PIN_GREEN, LOW);

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) 
    {    
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();

        delay(250);
    } 
 
        
    while(!calibrate_status)
    {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
        {   // Get the Latest packet 
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
            calibrationProcess(); 
        }
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

const byte numChars = 32;  // Maximum length of data (adjust as needed)
char receivedChars[numChars];  // Array to store received data
boolean newData = false;

float value1 = 0, value2 = 0;

void loop() {
    
    if (!dmpReady) 
    {
        return;
    }

    ledNow = millis();

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
    {  
        if(ledNow - ledPrev >= 1000)
        {
            ledPrev = millis();
            led_state = !(led_state);
            digitalWrite(LED_BUILTIN, led_state);
            digitalWrite(LED_PIN_RED, LOW);
            digitalWrite(LED_PIN_GREEN, led_state);
        }
        // Get the Latest packet 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
        mainProcess();        
    }
   
}

void calibrationProcess()
{
    blinkState = !(blinkState);
    digitalWrite(LED_PIN_RED, blinkState);
    
    yaw_data = ypr[0] * 180/M_PI;
    
    t_now = millis();
    
    if(t_now - t_prev < TIME_PERIOD_MS) 
    {
        yaw_now = ypr[0];
    }
    else
    {           
        dYaw = abs(yaw_now - yaw_prev);
        yaw_prev = yaw_now;
        
        if(dYaw < MINIMUM_ERROR)    calibrate_cnt += 1; 
        else                        calibrate_cnt  = 0;
        
        if (calibrate_cnt > MAX_CALIBRATE_CNT)
        {
            calibrate_status = 1;
            offset_degree = yaw_now;
        }
        
        t_prev = t_now;
     }
}

void mainProcess()
{   
    yaw_data = -(ypr[0] - offset_degree) * 180/M_PI;
    
    if(yaw_data > 180)       yaw_data -= 360;
    else if(yaw_data < -180) yaw_data += 360;

    pitch_data = ypr[1];

    roll_data = ypr[2];

    memcpy(send_ypr_data + 3, &yaw_data, 4);
    memcpy(send_ypr_data + 7, &pitch_data, 4);
    memcpy(send_ypr_data + 11, &roll_data, 4); 

    for(int i = 0; i < sizeof(send_ypr_data); i++)
        Serial.write(send_ypr_data[i]);

}
