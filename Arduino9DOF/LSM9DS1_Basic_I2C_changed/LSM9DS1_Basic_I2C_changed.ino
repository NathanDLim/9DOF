
/*****************************************************************
LSM9DS1_Basic_I2C.ino
SFE_LSM9DS1 Library Simple Example Code - I2C Interface
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 30, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

The LSM9DS1 is a versatile 9DOF sensor. It has a built-in
accelerometer, gyroscope, and magnetometer. Very cool! Plus it
functions over either SPI or I2C.

This Arduino sketch is a demo of the simple side of the
SFE_LSM9DS1 library. It'll demo the following:
* How to create a LSM9DS1 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS1 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and 
  the gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, 
  magnetic field strength using the calcAccel(), calcGyro() 
  and calcMag() functions.
* How to use the data from the LSM9DS1 to calculate 
  orientation and heading.

Hardware setup: This library supports communicating with the
LSM9DS1 over either I2C or SPI. This example demonstrates how
to use I2C. The pin-out is as follows:
	LSM9DS1 --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VDD ------------- 3.3V
	 GND ------------- GND
(CSG, CSXM, SDOG, and SDOXM should all be pulled high. 
Jumpers on the breakout board will do this for you.)

The LSM9DS1 has a maximum voltage of 3.6V. Make sure you power it
off the 3.3V rail! I2C pins are open-drain, so you'll be 
(mostly) safe connecting the LSM9DS1's SCL and SDA pins 
directly to the Arduino.

Development environment specifics:
	IDE: Arduino 1.6.3
	Hardware Platform: SparkFun Redboard
	LSM9DS1 Breakout Version: 1.0

This code is beerware. If you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, 
please buy us a round!

Distributed as-is; no warranty is given.
*****************************************************************/


/*
 * This class is to filter the incoming data from the heart rate monitor and remove unneccesary noise
 */
class DynamicFilter
{
  private:
    float *oldx;
    float *oldy;
    float *a;
    float *b;
    int arrLen;

    
  public:  
  /*
   * Constructor for the Filter
   * Assigns the a and b values, a and b must have 3 values each. a[0] must be 1.
   */
  DynamicFilter(float aValues[],float bValues[],int baSize)
  {    
    arrLen = baSize-1;
    oldx = new float[arrLen];
    oldy = new float[arrLen];
    a = new float[baSize];
    b = new float[baSize];
    
    for(int i = 0;i<arrLen;i++)
    {
      a[i] = aValues[i];
      b[i] = bValues[i];
      oldx[i] = 0;
      oldy[i] = 0;
    }


    a[arrLen] = aValues[arrLen];
    b[arrLen] = bValues[arrLen];
  }

  ~DynamicFilter()
  {
    delete[] a;
    delete[] b;
    delete[] oldx;
    delete[] oldy;
  }

  /*
   * When a new value is added, it performs the IIR filter
   */
 void addValue(float in)
  {    
    
    float newY = in*b[0];
    
    for(int i = arrLen;i>=1;i--)
    {
      newY += oldx[i-1]*b[i] - oldy[i-1]*a[i];
    }



    for(int i = arrLen-1;i>=1;i--)
    {
      oldy[i] = oldy[i-1];
      oldx[i] = oldx[i-1];
    }

    oldy[0] = newY;
    oldx[0] = in;


//    float newy = in*b[0] + oldx[0]*b[1] + oldx[1]*b[2] - oldy[0]*a[1] - oldy[1]*a[2];
//    oldy[1] = oldy[0];
//    oldy[0] = newy;
//    oldx[1] = oldx[0];
//    oldx[0] = in;
  }

  /*
   * @return y_1 and y_2 as a string
   */
  String getLastTwoValues()
  {
    return String(oldy[0]) + " " + String(oldy[1]); 
  }

  String getLastValue()
  {
    return String(oldy[0]);
  }
  
};




// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 40 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -13.58 // Declination (degrees) in Ottawa, ON.



float a[] = {1.0, -1.3073, 0.4918};
float b[] = {0.0461, 0.0923, 0.0461};

DynamicFilter *df;

void setup() 
{
  
  Serial.begin(115200);


  df = new DynamicFilter(a,b,3);
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
      
  }
}

void loop()
{
//  printGyro();  // Print "G: gx, gy, gz"
//  printAccel(); // Print "A: ax, ay, az"
//  printMag();   // Print "M: mx, my, mz"
  
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  imu.readAccel();
  imu.readMag();
  printAttitude(imu.ax, imu.ay, imu.az, imu.calcMag(imu.my), imu.calcMag(imu.mx), imu.calcMag(imu.mz));
  //Serial.println();
  
  delay(PRINT_SPEED);
}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();
  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
//  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
//  Serial.print("M: ");
  Serial.print(imu.calcMag(imu.mx), 5);
  Serial.print(" ");
  Serial.print(imu.calcMag(imu.my), 5);
  Serial.print(" ");
  Serial.println(atan2(imu.calcMag(imu.mx)-0.125, imu.calcMag(imu.my)-0.125), 5);
//  Serial.println(df->getLastValue());
//  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
//  Serial.println(imu.mz);
  Serial.println(atan(imu.my/imu.mx));
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
/*
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
//  if (heading > PI) heading -= (2 * PI);
//  else if (heading < -PI) heading += (2 * PI);
//  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180/PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

// df->addValue((atan2(imu.calcMag(imu.my),imu.calcMag(imu.mx))));
//  df->addValue(360/2.2*(0.2+atan2(imu.calcMag(imu.my),imu.calcMag(imu.mx))));
  
  
  
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  
  float heading;
  if (abs(my) < 0.001)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx,my);

//    heading= atan2( (-my*cos(roll) + mz*sin(roll) ) , (mx*cos(pitch) + my*sin(pitch)*sin(roll)+ mz*sin(pitch)*cos(roll)) ) ;

//  heading -= DECLINATION * PI / 180;
  
//  if (heading > PI) heading -= (2*PI);
//  else if (heading < -PI) heading += (2*PI);
//  else if (heading < 0) heading += PI;
  
  // Convert everything from radians to degrees:
  
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print(pitch, 2);
  Serial.print(" ");
  Serial.print(roll, 2);
  Serial.print(" "); Serial.println(heading);

  */

  
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
//  if (my == 0)
//    heading = (mx < 0) ? 180.0 : 0;
//  else
//    heading = atan2(imu.calcMag(imu.mx)-0.125, imu.calcMag(imu.my)-0.125);

   // Tilt compensated magnetic field X
  float mag_x = (imu.calcMag(imu.mx)-0.125) * cos(pitch) +(imu.calcMag(imu.my)-0.125) * sin(roll) * sin(pitch) + (imu.calcMag(imu.mz)-0.125) * cos(roll) * sin(pitch);
  // Tilt compensated magnetic field Y
  float mag_y = (imu.calcMag(imu.my)-0.125) * cos(roll) - (imu.calcMag(imu.mz)-0.125) * sin(roll);
  // Magnetic Heading
  heading = atan2(-mag_y, mag_x);

//  heading= atan2( (-(imu.calcMag(imu.my)-0.125)*cos(roll) + (imu.calcMag(imu.mz)-0.125)*sin(roll) ) , ((imu.calcMag(imu.mx)-0.125)*cos(pitch) + (imu.calcMag(imu.my)-0.125)*sin(pitch)*sin(roll)+ (imu.calcMag(imu.mz)-0.125)*sin(pitch)*cos(roll)) ) ;
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  df->addValue(heading);
  
//  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(" ");
  Serial.print(roll, 2);
  Serial.print(" "); Serial.println(df->getLastValue());
}
