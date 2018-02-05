/* 

BAaT: Battery-powered Arduino Alarm Technology
In this sketch, students learn the basics of Arduino-based
circuit design and coding as the build a robotic bat that squeaks
and flaps its wings as objects or people get closer to it.
The Arduino microcontroller is coupled to a standard "Arduino
Ping" circuit (which costs about $3) and the BaAT uses the 
Ping in its main loop to measure distance to objects in front of it.

After each distance has been measured, the loop then checks to
see if "TargetDistance" is "TooFarAwayToCare" (You can set that distance
in the header to this file, just below.) If the TargetDistance is less than
TooFarAwayToCare the loop next checks to see if the TargetDistance is
"WayTooClose". If it is WayTooClose, the BaAT flaps its wings frantically
for a few seconds and then the loop repeats, checking distance again.
If the TargetDistance is between ToFarToCare and WayTooClose the BaAT
makes a whistling sound that grows higher and higher pitched as the
TargetDistance approaches WayTooClose.

Overview of Ping Sensor
   This sketch reads a PING))) ultrasonic rangefinder and returns the
   distance to the closest object in range. To do this, it sends a pulse
   to the sensor's "Trig" (trigger) line to initiate a reading, and then 
   listens to a pulse that is genrated by the PING on its "Echo" line 
   until it hears the returning echo at which time the Echo pulse terminates.
   The length of the Echo pulse is proportional to the distance of the object 
   from the sensor. By taking into account the speed of sound, we can use the
   delay time to compute TargetDistance.

   The circuit:
	* +V connection of the PING))) attached to **+5V
	* GND connection of the PING))) attached to ground
	* Trig connection of the PING))) attached to the digital pin called "Trig"
  * Echo connection of the Ping))) attached to digital pin called "Echo"

  Note that some Pings have only three wires, placing the Trig and Echo functions
  together. For more on the ping see:
   http://www.arduino.cc/en/Tutorial/Ping


Overview of the ServoMotor that does the flapping
  This design uses the SG90 servo motor to flap the wings.    
  The SG90, like most small servo motors has three inputs.
  One for GND, one for Power and one to Control the position
  of the motor. The servo can me moved to one of three positions:
  Left, Right, or Middle. You select a position on the "Control"
  line by controlling the voltage pattern on the Control line.
  If you set the control line High (~5 Volts) for 1msec followed
  by 19msec at Low (0 Volts) and repeat, the servo swings towards
  the left. If you set the control line High (~5 Volts) for 2msec 
  followed by 19msec at Low (0 Volts) and repeat, the servo swings 
  towards the left. 

 The circuit:
  * +V connection of the motor (Red Wire) to +5
  * GND connection of the motor (Brown Wire) ground
  * PWM wire of the motor (Orange Wire) to the digital pin called "Flap"

  Note that the SG90 can also be used with one of the analogue voltage outputs 
  of the arduino which use pulse wave modulation to generate a voltage that
  controls position, if you change the code appropriately. For more on the 
  motor see it's spec sheet at:
  http://akizukidenshi.com/download/ds/towerpro/SG90.pdf

Overview of the Buzzer for Alarm Sound
  We use a simple and inexpensive 3.3Volt Buzzer to make the BaAt's alarm cry.
  One wire of the Buzzer is connected to one of the Ground pins of the Arduino Board
  and the other is connected to the digital pin called "Buzz". Buzz is an
  analogue output (one of the pins marked with a "-") and we regulate the 
  sound by regulating voltage at that pin.

  The circuit:
  * Long pin on the Buzzer to analogue Pin called "Buzz"
  * Short Pin on the Buzzer to any of the GND pins
  * 
This example code is in the public domain.
Written For FS Maker Day
March 2018
 */

//List of Pins

const int Trig = 7;
const int Echo = 8;
const int Flap = 5;
const int Buzz = 6;

//List of Behavior Control Variables, in Centimeters

const int WayTooClose = 30; //10cm is about 4 inches. You can change this to whatever you want
const int TooFarAwayToCare = 150; //100cm, or one meter, is about 39 inches. You can change 
                                  //this to whatever you want, but keep it below 151, the Ping tends to
                                  //false alarm below that, unless your room is really empty
const int LowestFreq = 200; //This is the lowest tone our buzzer will make, indicating a far away target
const int HighestFreq = 2000; //This is the highest tone our buzzer will make, just before it starts flapping
const int NumOfFlaps = 3; //This sets how many flaps the BaAT makes per target detection
const int FlapAmplitude = 3; //This sets how big the flaps are

//List of additional variables

int TargetDistance;
int freq;
int BuzzerRange;
int FreqRange;
int FreqIncrementPerCm;
int DistFromWayTooClose;

void setup() {
  // initialize serial communication:
  Serial.begin(9600);  //This will allow us to monitor the actual distance as long
  //as the BaAT is plugged into your computer. To see the distance in a window click on
  //Tools> Serial Monitor
  
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(Flap, OUTPUT);
  pinMode(Buzz, OUTPUT);
}

void loop() {
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration; 
  long inches; 
  long cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:


  digitalWrite(Trig,LOW);
  delay(1);  //this makes the sketch wait for 1msec
  digitalWrite(Trig,HIGH);
  delay(1);
  digitalWrite(Trig,LOW);
  
  // The Echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(Echo,HIGH);

  // convert the time into a distance and set TargetDistance

  
  inches = (duration / 73.746) / 2;
  // there are 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide that by 2 to get the distance of the obstacle.
  
  cm = (duration / 2) / 29; 
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.

  TargetDistance = cm;
  // If you want to use inches instead of cm's, change that here
 
  //Here we print the output to the computer so you can check the actual distance
  //as long as your BaAT is plugged in to your computer. 
  Serial.print(cm);
  Serial.print("cm");
  Serial.print(inches);
  Serial.print("in, ");
  Serial.println();

//Next we have to have to decide if the object is WayToFarToCare

if (TargetDistance < TooFarAwayToCare)
{
//Next we have to decide whether to Buzz or Flap

if (TargetDistance > WayTooClose) //If this is true we Buzz
{
  //Figure Out What Frequency to Use
  BuzzerRange = TooFarAwayToCare - WayTooClose;
  FreqRange = HighestFreq - LowestFreq;
  FreqIncrementPerCm = FreqRange / BuzzerRange;
  DistFromWayTooClose = TargetDistance - WayTooClose;
  freq = HighestFreq - (DistFromWayTooClose * FreqIncrementPerCm);
  
  tone(Buzz,freq);  // Activate the Buzzer;
  delay(500);      // ...for 0.5 sec
  noTone(Buzz);     // Stop sound...
 
} else { //else we Flap

for (int Flaps=0; Flaps < NumOfFlaps; Flaps++){

  for (int i=0; i <= FlapAmplitude; i++){ //Move motor leftwards

    digitalWrite(Flap,HIGH);
    delay(1);
    digitalWrite(Flap,LOW);
    delay(19);
  }

  for (int i=0; i <= FlapAmplitude; i++){ //Move motor rightwards

    digitalWrite(Flap,HIGH);
    delay(2);
    digitalWrite(Flap,LOW);
    delay(19);
  }
}
  
}

//Now we are done and can restart the loop to make another distance measurement
 
}



  

}


