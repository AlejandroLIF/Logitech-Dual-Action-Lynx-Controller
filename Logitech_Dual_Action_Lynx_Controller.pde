import processing.opengl.*;
import processing.serial.*;

import procontroll.*;
import net.java.games.input.*;

import cc.arduino.*;

Arduino arduino;

ControllIO controllIO;
ControllDevice joypad;
ControllStick stick1;
ControllStick stick2;
ControllButton button1;
ControllButton button2;
ControllButton button3;
ControllButton button4;
ControllButton button5;
ControllButton button6;
ControllButton button7;
ControllButton button8;
ControllButton button9;
ControllButton button10;
ControllButton button11;
ControllButton button12;
ControllCoolieHat coolieHat;

float x, y, z, r, ang_WRIST_OFFSET, ang_T1, ang_T2, ang_T3, ang_T4, ang_WRIST_ROT, ang_HAND;
boolean maintainWristOrientation;

final float XY_MULTIPLIER = 0.2;
final float R_MULTIPLIER = 0.2;
final float Z_MULTIPLIER = 0.1;
final float ANG_BASE_MULTIPLIER = 1.5;
final float ANG_SHOULDER_MULTIPLIER = 1.5;
final float ANG_ELBOW_MULTIPLIER = 1.5;
final float ANG_WRIST_MULTIPLIER = 2;
final float ANG_WRIST_ROT_MULTIPLIER = -1.5;
final float ANG_HAND_MULTIPLIER = 1.5;
final float MINIMUM_RADIUS = 2;
final float MAXIMUM_RADIUS = 20;

ArrayList<Angles> memory;

class Angles{
 float ang_wrist_offset, ang_t1, ang_t2, ang_t3, ang_t4, ang_wrist_rot, ang_hand;
 boolean maintainwristorientation;
 Angles(){
   ang_wrist_offset = ang_WRIST_OFFSET;
   ang_t1 = ang_T1;
   ang_t2 = ang_T2;
   ang_t3 = ang_T3;
   ang_t4 = ang_T4;
   ang_wrist_rot = ang_WRIST_ROT;
   ang_hand = ang_HAND;
   maintainwristorientation = maintainWristOrientation;
 }
 
 float getAng_WRIST_OFFSET(){
  return ang_wrist_offset;
 }
 
 float getAng_T1(){
   return ang_t1;
 }
 
 float getAng_T2(){
   return ang_t2;
 }
 
 float getAng_T3(){
   return ang_t3;
 }
 
 float getAng_T4(){
   return ang_t4;
 }
 
 float getAng_WRIST_ROT(){
   return ang_wrist_rot;
 }
 
 float getAng_HAND(){
   return ang_hand;
 }
 
 boolean getMaintainWristOrientation(){
   return maintainwristorientation;
 }
}

void setup(){
  /******  SETUP THE ARDUINO  ******/
  println(Arduino.list());
  arduino = new Arduino(this, Arduino.list()[0], 57600);
  
  arduino.pinMode(3, Arduino.SERVO);  //Base
  arduino.pinMode(5, Arduino.SERVO);  //Shoulder
  arduino.pinMode(6, Arduino.SERVO);  //Elbow
  arduino.pinMode(9, Arduino.SERVO);  //Wrist
  arduino.pinMode(10, Arduino.SERVO); //Wrist_ROT
  arduino.pinMode(11, Arduino.SERVO); //Hand
  arduino.pinMode(14, Arduino.OUTPUT);//Stabilization indicator LED.
  
  /******  SETUP THE LOGITECH DUAL ACTION JOYPAD  ******/
  controllIO = ControllIO.getInstance(this);
  
  joypad = controllIO.getDevice("Logitech Dual Action");
  
  joypad.printButtons();
//  joypad.plug(this, "handleButton1Press", ControllIO.ON_PRESS, 1);
//  joypad.plug(this, "handleButton1Release", ControllIO.ON_RELEASE, 1);
 
  stick1 = joypad.getStick(1);
  stick1.setMultiplier(1);
  stick1.setTolerance(0.3);  //20% dead-zone

  stick2 = joypad.getStick(0);
  stick2.setMultiplier(1);
  stick2.setTolerance(0.3);  //20% dead-zone
  
  button1 = joypad.getButton(1);
  button2 = joypad.getButton(2);
  button3 = joypad.getButton(3);
  button4 = joypad.getButton(4);
  button5 = joypad.getButton(5);
  button6 = joypad.getButton(6);
  button7 = joypad.getButton(7);
  button8 = joypad.getButton(8);
  button9 = joypad.getButton(9);
  button10 = joypad.getButton(10);
  button11 = joypad.getButton(11);
  button12 = joypad.getButton(12);
  
  coolieHat = joypad.getCoolieHat(0);
  coolieHat.setMultiplier(1);
  goToHome();
}

void goToHome(){
  ang_T1 = 0;
  ang_T2 = 0;
  ang_T3 = 90;
  ang_T4 = 0;
  ang_WRIST_ROT = 90;
  ang_HAND = 90;
  ang_WRIST_OFFSET = 0;
  
  maintainWristOrientation = false;
  arduino.digitalWrite(13, Arduino.LOW);
  
  calcInverseKinematics();
  memory = new ArrayList<Angles>();
  memory.add(new Angles());
}

void handleBaseRotation(){
  ang_T1 += stick1.getX() * ANG_BASE_MULTIPLIER;
  constrainAng_T1();
  calcInverseKinematics();
}

void handleRadius(){
  r -= stick1.getY() * R_MULTIPLIER; //The Y axis is inverted.
  constrainR();
  x = r*cos(degToRad(ang_T1));
  y = r*sin(degToRad(ang_T1));
  calcForwardKinematics();
}

void handleXPosition(){
  x += coolieHat.getX() * XY_MULTIPLIER;
  r = sqrt(x*x + y*y);
  constrainR();
  constrainX();
  calcForwardKinematics();
}

void handleYPosition(){
  y += coolieHat.getY() * XY_MULTIPLIER; //The Y axis is inverted. The robot's axes are inverted as well. Therefore, end sign is positive.
  r = sqrt(x*x + y*y);
  constrainR();
  constrainY();
  calcForwardKinematics();
}

void handleZPosition(boolean orientation){
  z += orientation ? Z_MULTIPLIER : -Z_MULTIPLIER;
  constrainZ();
  calcForwardKinematics();
}

void handleWristRotation(){
  ang_WRIST_ROT += stick2.getX() * ANG_WRIST_ROT_MULTIPLIER;
  constrainAng_WRIST_ROT();
}

void handleWristOrientation(){
  ang_WRIST_OFFSET -= stick2.getY() * ANG_WRIST_MULTIPLIER; //The Y axis is inverted.
  constrainAng_WRIST_OFFSET();
}

void handleHandAperture(boolean open){
  ang_HAND += open ? ANG_HAND_MULTIPLIER : -ANG_HAND_MULTIPLIER;
  constrainAng_HAND();
}

void handleShoulderRotation(boolean orientation){
  ang_T2 += orientation ? ANG_SHOULDER_MULTIPLIER : -ANG_SHOULDER_MULTIPLIER;
  constrainAng_T2();
  calcInverseKinematics();
}

void handleElbowRotation(boolean orientation){
  ang_T3 += orientation ? ANG_ELBOW_MULTIPLIER : -ANG_ELBOW_MULTIPLIER;
  constrainAng_T3();
  calcInverseKinematics();
}

float radToDeg(float rad){
  return rad*180/PI;
}

float degToRad(float deg){
  return deg*PI/180;
}

void constrainX(){// 'x' always takes positive values
//  x = constrain(x, MINIMUM_RADIUS*cos(degToRad(ang_T1)), r*cos(degToRad(ang_T1)));
    x = constrain(x, MINIMUM_RADIUS*cos(degToRad(ang_T1)), MAXIMUM_RADIUS);
}

void constrainY(){  // 'y' may take negative values.
//  y = (y > 0) ? constrain(y, MINIMUM_RADIUS*sin(degToRad(ang_T1)), r*sin(degToRad(ang_T1))) : constrain(y, -r*sin(degToRad(ang_T1)), -MINIMUM_RADIUS*sin(degToRad(ang_T1)));
    y = constrain(y, -MAXIMUM_RADIUS, MAXIMUM_RADIUS);
}

void constrainZ(){
  z = constrain(z, -2, 15);
}

void constrainR(){
  r = constrain(r, MINIMUM_RADIUS, MAXIMUM_RADIUS);
}

void constrainAng_T1(){
  ang_T1 = constrain(ang_T1, -90, 90);
}

void constrainAng_T2(){
  ang_T2 = constrain(ang_T2, -50, 70);
}

void constrainAng_T3(){
  ang_T3 = constrain(ang_T3, 55, 155);
}

void constrainAng_WRIST_OFFSET(){
  ang_WRIST_OFFSET = constrain(ang_WRIST_OFFSET, -90, 90); //Verify validity.
}

void constrainAng_WRIST_ROT(){
  ang_WRIST_ROT = constrain(ang_WRIST_ROT, 10, 170);
}

void constrainAng_HAND(){
  ang_HAND = constrain(ang_HAND, 50, 110);
}

void calcInverseKinematics(){
  float t1 = degToRad(ang_T1);
  float t2 = degToRad(ang_T2);
  float t3 = degToRad(ang_T3);
  float R = 12;
  z = R*cos(t2) - R*sin(t3 - PI/2 + t2);
  y = sin(t1)*(R*sin(t2)+R*cos(t3 - PI/2 + t2));
  x = cos(t1)*(R*sin(t2)+R*cos(t3 - PI/2 + t2));
  r = sqrt(x*x + y*y);
}


void calcForwardKinematics(){
  final float x2y2z2 = x*x + y*y + z*z;
  final byte R = 12;
  ang_T1 = radToDeg(atan2(y, x));
  constrainAng_T1();
  ang_T2 = radToDeg(PI/2 - atan2(z, sqrt(x*x + y*y)) - acos(sqrt(x*x + y*y + z*z)/(2*R)));
  constrainAng_T2();
  ang_T3 = radToDeg(PI - acos(1 - (x*x + y*y + z*z)/(2*R*R)));
  constrainAng_T3();
  ang_T4 = -(ang_T2 + ang_T3);
  calcInverseKinematics();
}

/*
*  The Lynx Robot with which we're working is not an ideal robot.
*  These adjustments allow us to adjust the real behavior to better match the ideal values.
*/
void writeAnglesToLynx(){
  float t1 = 90 + ang_T1;
  float t2 = 90 - ang_T2;
  float t3 = 14 + ang_T3; //It's offset from the 90 degree mark.
  float t4 = maintainWristOrientation ? 90 + ang_WRIST_OFFSET - ang_T4 : 90 + ang_WRIST_OFFSET;
  t4 = constrain(t4, 10, 170);
  println("x \t y \t z \t r \t ang_T1 \t ang_T2 \t ang_T3 \t ang_T4 \t ang_WRIST_OFFSET \t ang_T4_ROT \t ang_HAND");
  print(nfs(x, 2, 2));
  print('\t');
  print(nfs(y, 2, 2));
  print('\t');
  print(nfs(z, 2, 2));
  print('\t');
  print(nfs(r, 2, 2));
  print('\t');
  print(nfs(t1, 2, 2));
  print('\t');
  print(nfs(t2, 2, 2));
  print('\t');
  print(nfs(t3, 2, 2));
  print('\t');
  print(nfs(t4, 2, 2));
  print('\t');
  print(nfs(ang_WRIST_OFFSET, 2, 2));
  print('\t');
  print('\t');
  print(nfs(ang_WRIST_ROT, 2, 2));
  print('\t');
  print('\t');
  println(nfs(ang_HAND, 2, 2));
  arduino.servoWrite(3, (int)t1);
  arduino.servoWrite(5, (int)t2);
  arduino.servoWrite(6, (int)t3);
  arduino.servoWrite(9, (int)t4);
  arduino.servoWrite(10, (int)ang_WRIST_ROT);
  arduino.servoWrite(11, (int)ang_HAND);
}

void writeAnglesToLynx(Angles angles){
  float t1 = 90 + angles.getAng_T1();
  float t2 = 90 - angles.getAng_T2();
  float t3 = 14 + angles.getAng_T3(); //It's offset from the 90 degree mark.
  float t4 = angles.getMaintainWristOrientation() ? 90 - angles.getAng_T4() : 90 + angles.getAng_WRIST_OFFSET();
  t4 = constrain(t4, 10, 170);
  println("x \t y \t z \t r \t ang_T1 \t ang_T2 \t ang_T3 \t ang_T4 \t ang_WRIST_OFFSET \t ang_T4_ROT \t ang_HAND");
  print(nfs(x, 2, 2));
  print('\t');
  print(nfs(y, 2, 2));
  print('\t');
  print(nfs(z, 2, 2));
  print('\t');
  print(nfs(r, 2, 2));
  print('\t');
  print(nfs(t1, 2, 2));
  print('\t');
  print(nfs(t2, 2, 2));
  print('\t');
  print(nfs(t3, 2, 2));
  print('\t');
  print(nfs(t4, 2, 2));
  print('\t');
  print(nfs(angles.getAng_WRIST_OFFSET(), 2, 2));
  print('\t');
  print('\t');
  print(nfs(angles.getAng_WRIST_ROT(), 2, 2));
  print('\t');
  print('\t');
  println(nfs(angles.getAng_HAND(), 2, 2));
  arduino.servoWrite(3, (int)t1);
  arduino.servoWrite(5, (int)t2);
  arduino.servoWrite(6, (int)t3);
  arduino.servoWrite(9, (int)t4);
  arduino.servoWrite(10, (int)angles.getAng_WRIST_ROT());
  arduino.servoWrite(11, (int)angles.getAng_HAND());  
}

void draw(){
  if(button1.pressed()){
//    println("button1");
    handleShoulderRotation(false);
  }
  if(button2.pressed()){
//    println("button2");
    handleShoulderRotation(true);
  }
  if(button3.pressed()){
//    println("button3");
    handleElbowRotation(true);
  }
  if(button4.pressed()){
//    println("button4");
    handleElbowRotation(false);
  }
  if(button5.pressed()){
//    println("button5");
    handleHandAperture(false);
  }
  if(button6.pressed()){
//    println("button6");
    handleHandAperture(true);
    
  }
  if(button7.pressed()){
//    println("button7");
    handleZPosition(false);
  }
  if(button8.pressed()){
//    println("button8");
    handleZPosition(true);
  }
  if(button9.pressed()){
//    println("button9");
    maintainWristOrientation = !maintainWristOrientation;
    while(button9.pressed()){
      println();
    }
    if(maintainWristOrientation){
      ang_WRIST_OFFSET += ang_T4;
    }
    else{
      ang_WRIST_OFFSET -= ang_T4;
    }
    arduino.digitalWrite(13, maintainWristOrientation ? Arduino.HIGH : Arduino.LOW);
  }
  if(button10.pressed()){
//    println("button10");
    while(button10.pressed()){
      println();
    }
    goToHome();
  }
  if(button11.pressed()){    // UNUSED
//    println("button11");
    while(button11.pressed()){
      println();
    }
    memory.add(new Angles());
  }
  if(button12.pressed()){    // UNUSED
//    println("button12");
    while(button12.pressed()){
      println();
    }
    for(Angles angles : memory){
      writeAnglesToLynx(angles);
      delay(1000);
    }
  }
  if(coolieHat.pressed()){
//    println(coolieHat.getX() + "\t" + coolieHat.getY());
    handleXPosition();
    handleYPosition();
  }
  if(stick1.getX() != 0){
//    println("Stick 1 X: " + stick1.getX());
    handleBaseRotation();
  }
  if(stick1.getY() != 0){
//    println("Stick 1 Y: " + stick1.getY());
    handleRadius();
  }
  if(stick2.getX() != 0){
//    println("Stick 2 X: " + stick2.getX());
    handleWristRotation();
  }
  if(stick2.getY() != 0){
//    println("Stick 2 Y: " + stick2.getY());
    handleWristOrientation();
  }
  writeAnglesToLynx();
}
