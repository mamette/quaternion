//#include <AHRS.h>
#include <Wire.h> //The I2C library
#include <Servo.h>
#include <math.h>
#include <HMC5883L.h>  //Compass Library

#define BMA180              0x40
#define ee_w_MASK           0x10
#define mode_config_MASK    0x03
#define bw_MASK             0xF0
#define range_MASK          0x0E
#define lat_int_MASK        0x01
#define lat_int             0x01
#define betaDef  	    0.1f
#define sampleFreq	    512.0f	

//////////////////////////////////////////////////////Variabel Declaration////////////////////////////////////////////////////////////////
// Store our compass as a variable.
HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;
int gyroResult[3], accelResult[3];
int magX, magY, magZ;
int AccelX;

int x,y,z,temp;

float timeStep = 0.02;          //20ms. Need a time step value for integration of gyro angle from angle/sec
float biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ;
float pitchGyro;// = 0;
float pitchAccel;// = 0;
float pitchPrediction = 0; //Output of Kalman filter
float rollGyro;// = 0;
float rollAccel;// = 0;
float rollPrediction = 0;  //Output of Kalman filter
float yawGyro;// = 0;
float yawAccel;
float giroVar = 0.1;
float deltaGiroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;
float headingDegrees;

unsigned long timer;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = betaDef;
float pitch,roll,yaw;
float pitchRad;
float rollRad;
float Xmag;
float Ymag;
float Zmag;
float pitchGyrorad;
float rollGyrorad;
float yawGyrorad;

float invSqrt(float x);

  
  
//////////////////////////////////////////////////////////////////Function Declaration on Setup Section//////////////////////////////////////////////////////////////
//writTo Function
void writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

//readFrom Function
void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

//Gyro Init
void getGyroscopeReadings(int gyroResult[]) {
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1]; //Combine two bytes into one int
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

//Accelerometer Init
byte initializeBMA180()
{
  /*Set EEPROM image to write mode so we can change configuration*/
  delay(20);
  Wire.beginTransmission(BMA180);
  Wire.write(0x0D);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte ee_w = Wire.read();
  ee_w |= ee_w_MASK;
  Wire.beginTransmission(BMA180);
  Wire.write(0x0D);
  Wire.write(ee_w);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set mode configuration register to Mode 00*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x30);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte mode_config = Wire.read();
  mode_config &= ~(mode_config_MASK);
  Wire.beginTransmission(BMA180);
  Wire.write(0x30);
  Wire.write(mode_config);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set bandwidth to 10Hz*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x20);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte bw = Wire.read();
  bw &= ~(bw_MASK);
  bw |= 0x00 << 4;
  Wire.beginTransmission(BMA180);
  Wire.write(0x20);
  Wire.write(bw);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set acceleration range to 2g*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x35);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte range = Wire.read();
  range &= ~(range_MASK);
  range |= 0x00 << 1 ;
  /*    case B000: // 1g
        case B001: // 1.5g
        case B010/0x02: // 2g
        case B011: // 3g
        case B100: // 4g
        case B101: // 8g
        case B110: // 16g
        */
  
  Wire.beginTransmission(BMA180);
  Wire.write(0x35);
  Wire.write(range);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set interrupt latch state to non latching*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte latch_int = Wire.read();
  latch_int &= ~(0x01);
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  Wire.write(latch_int);
  if(Wire.endTransmission()){return(1);}
  delay(20); 
  /*Set interrupt type to new data*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte int_type = Wire.read();
  int_type |= 0x02;
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  Wire.write(int_type);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  return(0);
}

void MagnetometerInit() {
   //*************************COMPASS Init****************************
  Serial.println("Starting the I2C interface.");
  Wire.begin(); // Start the I2C interface.

  Serial.println("Constructing new HMC5883L");
  compass = HMC5883L(); // Construct a new HMC5883 compass.
    
  Serial.println("Setting scale to +/- 1.3 Ga");
  //error = compass.SetScale(1.3); // Set the scale of the compass.
  //if(error != 0) // If there is an error, print it out.
    //Serial.println(compass.GetErrorText(error));
  
  Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
}

void GyroSetting() {
   //Accelerometer and Gyro Setting
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
}

void AccelerometerGyroBias() {
  int i;
  int totalGyroXValues = 0;
  int totalGyroYValues = 0;
  int totalGyroZValues = 0;
  int totalAccelXValues = 0;
  int totalAccelYValues = 0;
  int totalAccelZValues = 0;
  
  // Determine zero bias for all axes of both sensors by averaging 50 measurements
  delay(100); //wait for gyro to "spin" up
  for (i = 0; i < 50; i += 1) {
    getGyroscopeReadings(gyroResult);

    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    totalAccelXValues += x;
    totalAccelYValues += y;
    totalAccelZValues += z;
    delay(50);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  biasAccelX = totalAccelXValues / 50;
  biasAccelY = totalAccelYValues / 50;
  biasAccelZ = (totalAccelZValues / 50); //Don't compensate gravity away! We would all (float)!
}




//########################################################################--Main Program--#############################################################
void setup() {
 
  Wire.begin();            //Open I2C communications as master
  Serial.begin(115200);    //Open serial communications to the PC to see what's happening

  MagnetometerInit();
  GyroSetting();
  initializeBMA180();
  AccelerometerGyroBias();
  
}

void loop() {
  
  timer = millis(); //get a start value to determine the time the loop takes
  
  readCompass();
  getGyroscopeReadings(gyroResult);
  readAccel();
  
  calculateSection();
  
  AHRSupdate();
  GetEuler();
   
 
 
  //Print data Ouput
  Serial.print(pitch);
  Serial.print("pitch\t");
  Serial.print(roll);
  Serial.print("roll \t");
  Serial.print(yaw);
  Serial.print("yaw \t"); 
     
  Serial.print("\n");
   
  timer = millis() - timer;          //how long did the loop take?
  timer = (timeStep * 1000) - timer; //how much time to add to the loop to make it last time step msec
  delay(timer);    //make one loop last time step msec                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
}

//###################################################--Main Program End--##################################################################






//**********************************************Function Declaration On Loop Section******************************************//

void readCompass() {
//*********************************Read Compass Begin*****************************************
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  magX=scaled.XAxis;
  magY=scaled.YAxis;
  magZ=scaled.ZAxis;
  
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(magY, magX);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2ï¿½ 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.55;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/M_PI; 
 
  // Output the data via the serial port.
  //Output(raw, scaled, heading, headingDegrees);

  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  // delay(66);
  
  //*************************************************Read Compas End*********************************************
}

byte readAccel()
{
  Wire.beginTransmission(BMA180);
  Wire.write(0x02);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,7) != 7){return(2);}
  x = Wire.read();
  x |= Wire.read() << 8;
  x >>= 2;
  //x = map(x, -4096, 4096, 0, 179);
  //pitch.write(x);
  
  y = Wire.read();
  y |= Wire.read() << 8;
  y >>= 2;
  //y = map(y, -4096, 4096, 0, 179);
  //roll.write(y);
  
  z = Wire.read();
  z |= Wire.read() << 8;
  z >>= 2;
  temp = Wire.read();
}

void calculateSection() {
  pitchAccel = atan2((y - biasAccelY) / 1024, (z - biasAccelZ) / 1024) * 360.0 / (2*PI); //calculate Accelerometer Pitch Degree
  pitchRad = atan2((y - biasAccelY) / 1024, (z - biasAccelZ) / 1024); //calculate Accelerometer Pitch Degree
  //AccelX = (x - biasAccelX) / 8192;
  
  pitchGyro = pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep; //calculate Gyroscope Pitch Degree
  pitchGyrorad = ((gyroResult[0] - biasGyroX) / 14.375);
 
  rollAccel = atan2((x - biasAccelX) / 1024, (z - biasAccelZ) / 1024) * 360.0 / (2*PI); //calculate Accelerometer Roll Degree
  rollRad = atan2((x - biasAccelX) / 1024, (z - biasAccelZ) / 1024);
  //AccelY = (accelResult[1] - biasAccelY) / 1024;
  
  rollGyro = rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;  //calculate Ggyroscope Roll Degree
  rollGyrorad =((gyroResult[1] - biasGyroY) / 14.375);

  yawGyro = yawGyro - ((gyroResult[2] - biasGyroZ) / 14.375) * timeStep; //calculate Gyroscope Yaw Degree
  yawGyrorad = ((gyroResult[2] - biasGyroZ) / 14.375); //calculate Gyroscope Yaw Degree
  //AccelZ = (accelResult[2] - biasAccelZ) / 1024;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void GetEuler(void){
  
  roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 360/(2*PI);
  pitch = asin(2 * (q0 * q2 - q3 * q1)) * 360/(2*PI);
  yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2* (q2 * q2 + q3 * q3)) * 360/(2*PI);
  if (yaw < 0){
    yaw +=360;
  }

}

void AHRSupdate() {

  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;
  static float mx;
  static float my;
  static float mz;

  gx = pitchGyrorad;
  gy = rollGyrorad;
  gz = yawGyrorad;

  ax = x;
  ay = y;
  az = z;

  mx = magX;
  my = magY;
  mz = magZ;
  
        float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  
  // Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

