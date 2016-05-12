/*
 * Arduino code for communicating with the LSM9DS1 9DOF sensor.
 * It continuously outputs the roll, pitch, and yaw. These euler angles are calculated from gyroscope + accel + magn, and a complemantary filter is used.
 * 
 * Author: Nathan Lim
 */

#include <SparkFunLSM9DS1.h>
#include <Wire.h>

#define MAG_ADDR 0x1E
#define AG_ADDR 0x6B
#define PRINT_SPEED 10

#define DECLINATION -13.58 // Declination (degrees) in Ottawa, ON.


LSM9DS1 imu;

float gpitch,groll,gyaw;
float goffX,goffY,goffZ;
float tx,ty,tz;

/*
 * gyroscope data is very unstable, we find the average of a number of samples to smooth the data
 */
void averageGyro(float *gxT, float *gyT, float *gzT, int num){
  double a=0,b=0,c=0;
  for(int i=0; i<num;i++){
    imu.readGyro();
    a += imu.calcGyro(imu.gx);
    b += imu.calcGyro(imu.gy);
    c += imu.calcGyro(imu.gz);
  }

    *gxT = a/num;
    *gyT = b/num;
    *gzT = c/num;

}

void setup() {
  
  Serial.begin(115200);
  
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = MAG_ADDR;
  imu.settings.device.agAddress = AG_ADDR;

  if(!imu.begin()){
    Serial.println("Could not connect");
  }

  gpitch = 0;
  gyaw = 0;
  groll = 0;

  //the gyroscope offsets must be first taken into account. 1000 is a lot, but the z axis for the gyropscope may drift if this number goes down.
  averageGyro(&goffX,&goffY,&goffZ,1000);

}


void loop() {

  imu.readAccel();
  imu.readMag();
  averageGyro(&tx,&ty,&tz,10);

  //integrate gyropscope data to find approx angle. This will have errors over time.
  //If the gyroscope data is very small then it is ignored.
  groll += abs(tx-goffX)<0.05? 0:(tx-goffX)/2000;
  gpitch += abs(ty-goffY)<0.05? 0:(ty-goffY)/2000;
  gyaw += abs(tz-goffZ)<0.025? 0:(tz-goffZ)/2000;

  //Complementary filter. Combined the gyropscope data with the accelerometer and magnetometer data.
  gpitch = gpitch*0.98 - 0.02*atan2(-imu.calcAccel(imu.ax), sqrt(imu.calcAccel(imu.ay) * imu.calcAccel(imu.ay) + imu.calcAccel(imu.az) * imu.calcAccel(imu.az)));
  groll = groll*0.98 - 0.02*atan2(imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
  //the mx and my must be offset by a certain amount. 0.105 and 0.155 are found by looking at the max and min data and finding where 0,0 should be.
  gyaw = gyaw*0.98 + 0.02*atan2(imu.calcMag(imu.mx)-0.105,imu.calcMag(imu.my)-0.155);

//  Serial.print(groll*180/PI);
//  Serial.print(" " + String(gpitch*180/PI));
//  Serial.println(" " + String((gyaw)*180/PI-DECLINATION));

  Serial.print(groll*180/PI);
  Serial.print(" ");
  Serial.print(gpitch*180/PI);
  Serial.print(" ");
  Serial.println(gyaw*180/PI);
//  Serial.println(" " + String(imu.calcMag(imu.mz)-goffZ));

  delay(PRINT_SPEED);

}
