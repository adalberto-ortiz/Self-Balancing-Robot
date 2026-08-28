#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Functions

void readIMU();
void calculateAngle();
void readEncoder();
void setMotorSpeed(int speed);
void stopMotors();

// Variables for left motor

const int PWMA = 25;
const int AIN1 = 26;
const int AIN2 = 27;

//Variables for right motor

const int PWMB = 13;
const int BIN1 = 12;
const int BIN2 = 15;

// Standby

const int STBY = 14;

//For PWM

const int freq = 1000;
const int resolution = 8;

//Variables for Accelerometer

float ax;
float ay;
float az;

//Variables for Gyroscope

float gyroX;
float gyroY;
float gyroZ;

//Angles variables

float roll;
float pitch;

//Pins for the Encoders

//Left encoder
const int LEFT_ENCODER_A = 34;
const int LEFT_ENCODER_B = 35;

// Right encoder
const int RIGHT_ENCODER_A = 32;
const int RIGHT_ENCODER_B = 33;

//Variables for Encoder

volatile int32_t leftPulseCount = 0;
volatile int32_t rightPulseCount = 0;

float leftRPM;
float rightRPM;

// MPU6050

Adafruit_MPU6050 mpu;

//Left Encoder Interrupt

void IRAM_ATTR leftEncoderISR()
{
  if (digitalRead(LEFT_ENCODER_B))
  {
    leftPulseCount++;
  }
  else
  {
    leftPulseCount--;
  }
}

// Right Encoder Interrupt

void IRAM_ATTR rightEncoderISR()
{
  if (digitalRead(RIGHT_ENCODER_B))
  {
    rightPulseCount++;
  }
  else
  {
    rightPulseCount--;
  }
}

//Balancing variables
float targetAngle = -0.5;
float error;
float Kp = 40;
float Kd = 9;
float gyroCorrection;
int motorOutput;

float filteredPitch = 0.0;
float previousTime = 0.0;

const float alpha = 0.98; //98 percent of angle estimate comes from gyro

void setup()
{
  Serial.begin(115200);

  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER_A),
    leftEncoderISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER_A),
    rightEncoderISR,
    RISING
  );

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  ledcAttach(PWMA, freq, resolution);
  ledcAttach(PWMB, freq, resolution);

  digitalWrite(STBY, HIGH);

  //For MPU6050

  while (!Serial)
  {
    delay(10);
  }

  if (!mpu.begin())
  {
    Serial.println("MPU6050 not found!");

    while (1)
    {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");


  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  Serial.println("System Ready!");
  previousTime = millis() / 1000.0;
}

void loop()
{
  //Read from the IMU and print

  readIMU();

  calculateAngle();

  if (abs(pitch) > 30) {
    stopMotors();
    return;
  }

  Serial.println("---------------------------");

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.println(" degrees");

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.println(" degrees");


  //Print the Gyroscope values

  Serial.print("Gyro X: ");
  Serial.print(gyroX);
  Serial.println(" rad/s");

  Serial.print("Gyro Y: ");
  Serial.print(gyroY);
  Serial.println(" rad/s");

  Serial.print("Gyro Z: ");
  Serial.print(gyroZ);
  Serial.println(" rad/s");

  //For Encoder readings

  readEncoder();

  //Find pitch error
  error = targetAngle - pitch;
  gyroCorrection = gyroY * Kd;
  motorOutput = (error * Kp) + gyroCorrection;
  motorOutput = constrain(motorOutput, -255, 255);
  setMotorSpeed(-motorOutput);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Motor Output: ");
  Serial.println(motorOutput);
  Serial.print("Gyro Correction: ");
  Serial.println(gyroCorrection);
}

void readIMU()
{
  sensors_event_t accel, gyro, temp;

  mpu.getEvent(&accel, &gyro, &temp);

  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;

  ax = accel.acceleration.x;
  ay = accel.acceleration.y;
  az = accel.acceleration.z;
}

void calculateAngle()
{
    // Accelerometer pitch
    float accelPitch = atan2(
        -ax,
        sqrt(ay * ay + az * az)
    ) * 180.0 / PI;

    // Time since last calculation
    float currentTime = millis() / 1000.0;
    float dt = currentTime - previousTime;
    previousTime = currentTime;

    //Gyroscope integration
    float gyroPitch = filteredPitch + gyroY * 180.0 / PI * dt;

    //Complementary filter
    filteredPitch = alpha * gyroPitch + (1.0 - alpha) * accelPitch;

    pitch = filteredPitch;
}

void readEncoder()
{
  Serial.print("Left pulses: ");
  Serial.println(leftPulseCount);

  Serial.print("Right pulses: ");
  Serial.println(rightPulseCount);
}

void setMotorSpeed(int speed)
{
  speed = constrain(speed, -255, 255);


  if (speed > 0)
  {
    //For Forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    ledcWrite(PWMA, speed);
    ledcWrite(PWMB, speed);
  }


  else if (speed < 0)
  {
    //For Reverse
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    ledcWrite(PWMA, -speed);
    ledcWrite(PWMB, -speed);
  }


  else
  {
    stopMotors();
  }
}

void stopMotors()
{
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}