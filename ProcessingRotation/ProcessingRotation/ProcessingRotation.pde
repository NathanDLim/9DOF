
import processing.serial.*;

Serial myPort;  // Create object from Serial class
String incoming;      // Data received from the serial port

float x,y,z,a;
int rectW,rectH,rectL;
float zerox,zeroy,zeroz;
PrintWriter output;
boolean calibrate;

void setup() {
  size(1000, 800, P3D);
  rectW = 210;
  rectH = 60;
  rectL = 300;
  zerox =0;
  zeroy=0;
  zeroz=0;
  calibrate = false;
  
  String[] ports= Serial.list();
  text(ports[2],30,20);
  if(ports.length >0)
  {
   String portName = Serial.list()[2];
   myPort = new Serial(this, portName, 9600);
  }
  
  //output = createWriter("angles.txt");
  
  background(0);
  ortho();
}


void draw() {
  background(0);
  translate(width/2, height/2, 0);
  
  
 if ( myPort.available() > 0) {  // If data is available,
   incoming = myPort.readString();         // read it and store it in val
   String[] vals = split(incoming, " ");
   if(vals.length == 4){
     x = float(vals[1]);
     y = float(vals[2]);
     z = float(vals[3]);
     println(str(x) + " " + str(y)+ " " + str(z));
   }
 }
 
 if(mousePressed){
  zerox = x;
  zeroy=y;
  zeroz=z;
 }
 
 
  if(calibrate)
    text("Calibrating Magnetometer",100,100);
  //output.println(str(x) + " " + str(y));
  //println(str(x) + " " + str(y));
rotateZ((zeroz-z)/180*PI);
 rotateX((x-zerox)/180*PI);
 
 rotateY(((y-zeroy))/180*PI);
  

  //stroke(255);
  
  //pushMatrix();
  //fill(25);
  
  //stroke(255,0,0);
  //fill(255,0,0);
  //drawCylinder(18,10,700,30);
  
  //stroke(0);
  //fill(100);
  //drawCylinder(20,20,-150,30);
  
  //beginShape(QUADS);
  
  box(140,100,10);
  
  //endShape();
  
  //popMatrix();
}

void keyPressed() {
  if(calibrate)
    myPort.write("cal");
  else
    myPort.write("cal0");
  calibrate = !calibrate;
  //output.flush(); // Writes the remaining data to the file
  //output.close(); // Finishes the file
  //exit(); // Stops the program
}