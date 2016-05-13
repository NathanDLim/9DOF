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
int16_t mxmin,mxmax,mymin,mymax,mzmin;
bool calibrated;

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
  mxmin = 20000;
  mymin = 20000;
  mxmax = -1;
  mymax = -1;
  mzmin= 0;
  calibrated = false;

  //the gyroscope offsets must be first taken into account. 1000 is a lot, but the z axis for the gyropscope may drift if this number goes down.
  //if the heading is accurate, this number can go lower
  averageGyro(&goffX,&goffY,&goffZ,1000);

}


void loop() {

  if(Serial.available() > 0){
    String s = Serial.readString();
    if(s.length() > 3){
      if(s.substring(0,3).equals("cal")){
        s=s.substring(3);
        imu.magOffset(X_AXIS,s.toInt());
        imu.magOffset(Y_AXIS,s.toInt());
        Serial.println("Calibrated " + s);
        mxmin = 20000;
        mymin = 20000;
        mxmax = -1;
        mymax = -1;
        calibrated=true;
      }
    }
    else if (s.equals("cal")){
      imu.magOffset(X_AXIS,(mxmax+mxmin)/2);
      imu.magOffset(Y_AXIS,(mymax+mymin)/2);
      Serial.println("Calibrated");
      mxmin = 20000;
      mymin = 20000;
      mxmax = -1;
      mymax = -1;
    }
  }

  imu.readAccel();
  
  
  averageGyro(&tx,&ty,&tz,10);

  float m[3];
  if(!calibrated){
    fixMagOffsets();
    m[0] = (imu.calcMag(imu.my)-imu.calcMag((mymax+mymin)/2));
    m[1] = (imu.calcMag(imu.mx)-imu.calcMag((mxmax+mxmin)/2));
    m[2] = imu.calcMag(imu.mz);
  }
  else
  {
    fixMagOffsets();
    m[0] = imu.calcMag(imu.my);
    m[1] = imu.calcMag(imu.mx);
    m[2] = imu.calcMag(imu.mz);
  }

  //integrate gyropscope data to find approx angle. This will have errors over time.
  groll += abs(tx-goffX)<0.05? 0:(tx-goffX)/2000;
  gpitch += abs(ty-goffY)<0.05? 0:(ty-goffY)/2000;
  gyaw += abs(tz-goffZ)<0.025? 0:(tz-goffZ)/2000;

  //Complementary filter. Combined the gyropscope data with the accelerometer and magnetometer data.
//  float apitch = atan2(-imu.calcAccel(imu.ax), sqrt(imu.calcAccel(imu.ay) * imu.calcAccel(imu.ay) + imu.calcAccel(imu.az) * imu.calcAccel(imu.az)));
//  float aroll = atan2(imu.calcAccel(imu.ay), imu.calcAccel(imu.az))

  float apitch = atan2(-imu.calcAccel(imu.ax), sqrt(imu.calcAccel(imu.ay) * imu.calcAccel(imu.ay) + imu.calcAccel(imu.az) * imu.calcAccel(imu.az)));
  float aroll = atan2(imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
  gpitch = gpitch*0.98 - 0.02*apitch;
  groll = groll*0.98 - 0.02*aroll;
  gyaw = gyaw*0.98 + 0.02*atan2(-m[1],m[0]);

  
//  float xh = m[0]*cos(gpitch) + m[1]*sin(gpitch)*sin(groll) + m[2]*sin(gpitch)*cos(groll);
//  float yh = m[2]*sin(groll)-m[1]*cos(groll);

  //xh is calibrated to counteract roll
  float xh = m[0]*cos(gpitch) +m[2]*sin(groll) - m[1]*sin(groll);
  float yh = m[1];
  
  gyaw = atan2(yh,xh);


  /*
   * counteracting roll for xh TODO: need to calibrate xh and yh against pitch, and yh against roll.
   */
//  Serial.print(m[0],5);
//  Serial.print(" ");
//  Serial.print(m[0]*cos(gpitch) +m[2]*sin(groll) - m[1]*sin(groll),5);
//  Serial.print(" ");
//  Serial.print(m[1]*sin(groll),5);
//  Serial.print(" ");
//  Serial.println((m[2])*sin(groll),5);
  
//  
//  Serial.print(" ");
//  Serial.print(mxmin); 
//  Serial.print(" ");
//  Serial.print(mxmax);
//  Serial.print(" ");
//  Serial.println((mxmax+mxmin)/2);
  
  Serial.print(aroll*180/PI);
  Serial.print(" ");
  Serial.print(apitch*180/PI);
  Serial.print(" ");
  Serial.println(gyaw*180/PI);
//  Serial.print(" ");
//  Serial.println(atan2(-m[1],m[0])*180/PI);

  delay(PRINT_SPEED);

}

int sign(float x){
  return (x>0)? 1:-1;
}

/*
 * This function reads the magnetometer and finds the highest and lowest values of the x and y axis. It is used to offset the magnetometers.
 * In order to have valid highs and lows, the IMU must be rotated slowly in a full circle.
 */
void fixMagOffsets(){
  imu.readMag();

  int16_t tempmx = imu.mx;
  int16_t tempmy = imu.my;
  int16_t tempmz = imu.mz;

  //some sort of error
  if(abs(tempmx) > 100000 || abs(tempmy) > 10000 || abs(tempmz) > 5000)
    return;
  
  if(tempmx > mxmax)
    mxmax = tempmx;
  else if(tempmx < mxmin)
    mxmin = tempmx;

  if(tempmy > mymax)
    mymax = tempmy;
  else if(tempmy < mymin)
    mymin = tempmy;

  if(tempmz < mzmin)
    mzmin = tempmz;
  
}


