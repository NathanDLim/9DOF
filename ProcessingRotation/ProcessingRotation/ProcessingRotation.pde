
import processing.serial.*;

Serial myPort;  // Create object from Serial class
String incoming;      // Data received from the serial port

float x,y,z,a;
int rectW,rectH,rectL;
float zerox,zeroy,zeroz;
PrintWriter output;

void setup() {
  size(1000, 800, P3D);
  rectW = 210;
  rectH = 60;
  rectL = 300;
  zerox =0;
  zeroy=0;
  zeroz=0;
  
  String[] ports= Serial.list();
  text(ports[2],30,20);
  if(ports.length >0)
  {
   String portName = Serial.list()[2];
   myPort = new Serial(this, portName, 115200);
  }
  
  output = createWriter("angles.txt");
  
  background(0);
  ortho();
}


void draw() {
  background(0);
  translate(width/2, height/2, 0);
  
  
 if ( myPort.available() > 0) {  // If data is available,
   incoming = myPort.readString();         // read it and store it in val
   String[] vals = split(incoming, " ");
   if(vals.length == 3){
     x = float(vals[0]);
     y = float(vals[1]);
     z = float(vals[2]);
   }
 }
 
 if(mousePressed){
   zerox = x;
   zeroy=y;
   zeroz=z;
 }
 
  output.println(str(x) + " " + str(y));
  println(str(x) + " " + str(y));

  rotateX((x-zerox)/180*PI);
  rotateZ((zeroz-z)/180*PI);
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
  
  beginShape(QUADS);
  
  box(140,100,10);
  
  endShape();
  
  //popMatrix();
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
  float angle = 0;
  float angleIncrement = TWO_PI / sides;
  beginShape(QUAD_STRIP);
  for (int i = 0; i < sides + 1; ++i) {
    vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
    vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
    angle += angleIncrement;
  }
  endShape();
  
  // If it is not a cone, draw the circular top cap
  if (topRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);
    
    // Center point
    vertex(0, 0, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }

  // If it is not a cone, draw the circular bottom cap
  if (bottomRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, tall, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }
}

void keyPressed() {
  output.flush(); // Writes the remaining data to the file
  output.close(); // Finishes the file
  exit(); // Stops the program
}