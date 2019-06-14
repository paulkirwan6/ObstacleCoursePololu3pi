#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

Pololu3pi robot;
unsigned int sensors[5];
OrangutanLCD lcd;
OrangutanMotors motors;
OrangutanBuzzer buzzer;

const int buttonA = 9;
const int buttonB = 12;
const int buttonC = 13;
const int LDRright = 7;
const int LDRleft = 6;

#define buffersize 50
int LeftVal, RightVal, i = 0;
int ldrmin=10000;
int total, prevtotal;
int tiltsensor = 5, tiltval, tiltspeed, maxtilt=360;
int filtered, unfiltered, buffer[buffersize], count = 0,value,t=0;

void setup() {
  pinMode(buttonA, INPUT);
  pinMode(buttonB, INPUT);
  pinMode(buttonC, INPUT);
  pinMode(LDRleft,INPUT);
  pinMode(LDRright,INPUT);
  pinMode(tiltsensor, INPUT);
}
void loop() {

  while (digitalRead(buttonA)==HIGH && digitalRead(buttonB)==HIGH && digitalRead(buttonC)==HIGH){
  lcd.clear();
    lcd.gotoXY(0,0);
    lcd.print(" A-Disp");
    lcd.gotoXY(0,1);
    lcd.print(" B-Folw");
    delay(100);  
  }
  
  //Button A is used to balance the see-saw only
  if(digitalRead(buttonA)==LOW) {
    balance();
  }
  
  //Button B is used to follow the line and balance the see-saw
  else if(digitalRead(buttonB)==LOW) {
       find_line(); 
       follow_line();
       balance();
  }
  
  //Button C is used to complete each task consecutively
  else if(digitalRead(buttonC)==LOW) {
       find_light();
       go_to_light();
       find_line();
       follow_line();
       balance();
  }
  else {
  }
}


//function used to go the light
void go_to_light() {
  total=0;
  prevtotal=1000;
  while (true) {
    LeftVal=analogRead(LDRleft);
    RightVal=analogRead(LDRright);
    LeftVal=map(LeftVal, 50, 1000, 0, 100); 
    RightVal=map(RightVal, 50, 1000, 0, 100); 
    total = RightVal + LeftVal;
    
    //When a there is more light coming from the left with a tolerance, turn left
    if (RightVal < (LeftVal - 0.1*LeftVal)) {
      OrangutanMotors::setSpeeds(65,0);
      delay(10*(LeftVal-RightVal));
      OrangutanMotors::setSpeeds(0,0);
    }
     //When a there is more light coming from the left with a tolerance, turn left
    else if (LeftVal < (RightVal - 0.1*RightVal)) {
      OrangutanMotors::setSpeeds(0,65);
      delay(10*(RightVal-LeftVal));
      OrangutanMotors::setSpeeds(0,0);
    }
    //When the light drops by a large amount; the torch being turned off, break and move onto next task
    else if (prevtotal < (total-10)) {
      OrangutanMotors::setSpeeds(0,0);
      lcd.clear();
      lcd.gotoXY(0,0);
      lcd.print("break");
      break;
    }
    //at all other times move in a straight line
    else {
      OrangutanMotors::setSpeeds(50,50);
      delay(500);
      OrangutanMotors::setSpeeds(0,0);
    }
    //allows it to compare consecutive totals in order to detect the torch being turned off
    prevtotal=total;
  }
}

//function used to detect the line 
void find_line () {
  pololu_3pi_init_disable_emitter_pin(2000);
  OrangutanMotors::setSpeeds(-40, 40);
  delay(400);
  OrangutanMotors::setSpeeds(0, 0);
  for (i=0; i<80; i++)  {
    if (i < 20 || i >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(25);
  }
  OrangutanMotors::setSpeeds(0, 0);
}

//function used to follow the line once it has been detected
void follow_line() {
  filtered = 0;
    // Get the position of the line.  Note that we *must* provide
    // the "sensors" argument to read_line() here, even though we
    // are not interested in the individual sensor readings.
   while (t<50 || filtered<346) { //loop until buffer breaks
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
    unfiltered = analogRead(tiltsensor);
    filtered = median_filter(unfiltered);
    lcd.clear();
    lcd.gotoXY(0,1);
    lcd.print(unfiltered);
    lcd.gotoXY(0,0);
    lcd.print(filtered);
    delay(20);
    if (position < 1000)
    {
      // We are far to the right of the line: turn left.
      OrangutanMotors::setSpeeds(5,50);
    }
    else if (position < 3000)
    {
      // We are somewhat close to being centered on the line:
      // drive straight.
      OrangutanMotors::setSpeeds(50,50);
    }
    else
    {
      // We are far to the left of the line: turn right.
      OrangutanMotors::setSpeeds(50, 5);
    }
    t++;
  }
  
  //makes noise to show that it has completed the task
  OrangutanMotors::setSpeeds(0, 0);
  buzzer.playNote(NOTE_A(6), 200, 15);
  delay(200);
  buzzer.stopPlaying();
  delay(100);
  buzzer.playNote(NOTE_A(6), 200, 15);
  delay(200);
  buzzer.stopPlaying();
}

//function used to initially find position of light
void find_light() {
  //spins and takes light readings
  OrangutanMotors::setSpeeds(40,-40);
  for (i=0;i<15;i++) {
    LeftVal = analogRead(LDRleft);
    RightVal = analogRead(LDRright);
    total = LeftVal + RightVal;
    delay(100);
    //finds the minumum of the light readings
    if (total<ldrmin) {
      ldrmin = total;
    }
  }
  OrangutanMotors::setSpeeds(0,0);
  LeftVal = analogRead(LDRleft);
  RightVal = analogRead(LDRright);
  total = LeftVal + RightVal;
  
  //spins again and this time faces the light source
  while(true) {
    OrangutanMotors::setSpeeds(40,-40);
    if (total>ldrmin + 75) {
      LeftVal = analogRead(LDRleft);
      RightVal = analogRead(LDRright);
      total = LeftVal + RightVal;
    }
    else {
    //makes noise to indicate it has found the position of the light
    OrangutanMotors::setSpeeds(0, 0);  
    buzzer.playNote(NOTE_A(6), 200, 15);
    delay(500);
    buzzer.stopPlaying();
    break;
    }
  }
}

void balance() {
  while (true){
    OrangutanMotors::setSpeeds(0,0);
    tiltval = analogRead(tiltsensor);
    lcd.clear();
    lcd.print(tiltval);
    delay(80);
    
    //moves backwards when the see-saw is tilted forwards
    if (tiltval<330) {
      OrangutanMotors::setSpeeds(-28,-28);
      delay(200);
      OrangutanMotors::setSpeeds(0,0);
      delay(500);
    }
    
    //moves backwards when the see-saw is tilted backwards
    else if (tiltval>345){
      OrangutanMotors::setSpeeds(28,28);
      delay(200);
      OrangutanMotors::setSpeeds(0,0);
      delay(500);
    }
    
    //does not move when it is reasonably level such that both ends of see-saw are off the table
    else {
      OrangutanMotors::setSpeeds(0,0);
      lcd.clear();
      lcd.print("level");
      //makes noise after 3 seconds to indicate it has balanced th see saw for the 3 seconds required
      delay(3000);
      buzzer.playNote(NOTE_A(6), 200, 15);
      delay(200);
      buzzer.stopPlaying();
      delay(100);
      buzzer.playNote(NOTE_A(6), 200, 15);
      delay(200);
      buzzer.stopPlaying();
      delay(100);
      buzzer.playNote(NOTE_A(6), 200, 15);
      delay(200);
      buzzer.stopPlaying();
    }
  }
} 

//uses a buffer
int median_filter(int value){
  boolean swapped = true;
  int temp, j =0;
  int median_buffer[buffersize] = {};
  for (i = 0; i<=buffersize-2; i++) {
   buffer[i]=buffer[i+1]; 
  }
  
  buffer[buffersize-1] = value;
  
  if (count < buffersize) {
    count++;
    return value; 
  }
  else{
    for (i=0;i<buffersize; i++){
      median_buffer[i]=buffer[i];
    }   
  }
  while (swapped)
  {
    swapped = false;
    j++;
    for (i=0; i < (buffersize-1)-j; i++){
      if (median_buffer[i]>median_buffer[i+1]){
        temp=median_buffer[i];
        median_buffer[i] = median_buffer[i+1];
        median_buffer[i+1] = temp;
        swapped = true;
      }
    }
  }
  return median_buffer[buffersize/2];
}

   

