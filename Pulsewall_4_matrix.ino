/*
>> Pulse Sensor Amped 1.1 <<
This code is for Pulse Sensor Amped by Joel Murphy and Yury Gitman
    www.pulsesensor.com 
    >>> Pulse Sensor purple wire goes to Analog Pin 0 <<<
Pulse Sensor sample aquisition and processing happens in the background via Timer 2 interrupt. 2mS sample rate.
PWM on pins 3 and 11 will not work when using this code, because we are using Timer 2!
The following variables are automatically updated:
Signal :    int that holds the analog signal data straight from the sensor. updated every 2mS.
IBI  :      int that holds the time interval between beats. 2mS resolution.
BPM  :      int that holds the heart rate value, derived every beat, from averaging previous 10 IBI values.
QS  :       boolean that is made true whenever Pulse is found and BPM is updated. User must reset.
Pulse :     boolean that is true when a heartbeat is sensed then false in time with pin13 LED going out.

This code is designed with output serial data to Processing sketch "PulseSensorAmped_Processing-xx"
The Processing sketch is a simple data visualizer. 
All the work to find the heartbeat and determine the heartrate happens in the code below.
Pin 13 LED will blink with heartbeat.
If you want to use pin 13 for something else, adjust the interrupt handler
It will also fade an LED on pin fadePin with every beat. Put an LED and series resistor from fadePin to GND.
Check here for detailed code walkthrough:
http://pulsesensor.myshopify.com/pages/pulse-sensor-amped-arduino-v1dot1

Code Version 02 by Joel Murphy & Yury Gitman  Fall 2012
This update changes the HRV variable name to IBI, which stands for Inter-Beat Interval, for clarity.
Switched the interrupt to Timer2.  500Hz sample rate, 2mS resolution IBI value.
Fade LED pin moved to pin 5 (use of Timer2 disables PWM on pins 3 & 11).
Tidied up inefficiencies since the last version. 
*/


//  VARIABLES
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin
const int buttonPin = 4;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status


// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.


/// END OF PULSE SENSOR CODE ///

// Heat sensor
#include <Wire.h>
#include "Adafruit_MCP9808.h"

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();


// Heat sensor end


//  FASTLED STARTS HERE
#include "FastLED.h"

#define DATA_PIN    2
#define CLK_PIN   3
#define LED_TYPE    WS2801
#define COLOR_ORDER RGB
#define NUMPIXELS 50
#define NUMNEOPIXELS 7

// Define the array of leds
CRGB leds[NUMPIXELS];
CRGB neos[7];

#define BRIGHTNESS          255

//DEFINE_GRADIENT_PALETTE( heatmap_gp ) {
//  0,     0,  0,  0, 
//  50,     0,  0,  255,   
//160,   255, 265,  0,   
//240,   255,0,0 }; //full white

DEFINE_GRADIENT_PALETTE( heatmap_gp ) {

  0,   0,    0,  0,
  1,   0,    0,  255,
// 83,   255,  0,  0,
120,   0,  255,  0,
240,   255,  20, 147 ,
255,   255,  0,  0 };

const byte fadeArray[] =  {0,8,16,24,32,40,48,56,64,72,80,88,96,104,112,120,128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,248,240,232,224,216,208,200,192,184,176,168,160,152,144,136,128,120,112,104,96,88,80,72,64,56,48,40,32,24,16,8,0,0,8,16,24,32,40,48,56,64,72,80,88,96,104,112,120,128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,253,250,248,245,242,240,237,234,232,229,226,224,221,218,216,213,210,208,205,202,200,197,194,192,189,186,184,181,178,176,173,170,168,165,162,160,157,154,152,149,146,144,141,138,136,133,130,128,125,122,120,117,114,112,109,106,104,101,98,96,93,90,88,85,82,80,77,74,72,69,66,64,61,58,56,53,50,48,45,42,40,37,34,32,29,26,24,21,18,16,13,10,8,5,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

const int Matrix[][40] ={       {9,10,29,30,49,50,69,70,89,90,109,110,129,130,149,150,169,170,189,190,209,210,229,230,249,250,269,270,289,290,309,310,329,330,349,350,369,370,389,390,},
                                {8,11,28,31,48,51,68,71,88,91,108,111,128,131,148,151,168,171,188,191,208,211,228,231,248,251,268,271,288,291,308,311,328,331,348,351,368,371,388,391,},
                                {7,12,27,32,47,52,67,72,87,92,107,112,127,132,147,152,167,172,187,192,207,212,227,232,247,252,267,272,287,292,307,312,327,332,347,352,367,372,387,392,},
                                {6,13,26,33,46,53,66,73,86,93,106,113,126,133,146,153,166,173,186,193,206,213,226,233,246,253,266,273,286,293,306,313,326,333,346,353,366,373,386,393,},
                                {5,14,25,34,45,54,65,74,85,94,105,114,125,134,145,154,165,174,185,194,205,214,225,234,245,254,265,274,285,294,305,314,325,334,345,354,365,374,385,394,},
                                {4,15,24,35,44,55,64,75,84,95,104,115,124,135,144,155,164,175,184,195,204,215,224,235,244,255,264,275,284,295,304,315,324,335,344,355,364,375,384,395,},
                                {3,16,23,36,43,56,63,76,83,96,103,116,123,136,143,156,163,176,183,196,203,216,223,236,243,256,263,276,283,296,303,316,323,336,343,356,363,376,383,396,},
                                {2,17,22,37,42,57,62,77,82,97,102,117,122,137,142,157,162,177,182,197,202,217,222,237,242,257,262,277,282,297,302,317,322,337,342,357,362,377,382,397,},
                                {1,18,21,38,41,58,61,78,81,98,101,118,121,138,141,158,161,178,181,198,201,218,221,238,241,258,261,278,281,298,301,318,321,338,341,358,361,378,381,398,},
                                {0,19,20,39,40,59,60,79,80,99,100,119,120,139,140,159,160,179,180,199,200,219,220,239,240,259,260,279,280,299,300,319,320,339,340,359,360,379,380,399}};


const int MatrixX = sizeof(Matrix[0])/sizeof(Matrix[0][0]);


// FASTLED ENDS HERE


unsigned long time;
unsigned long sleepTime, pulseTime , pixelTime;
bool pixelPushed = false;

int pixelPeriod[NUMPIXELS];
byte pixelTemp[NUMPIXELS];

float newPeriod;
float newTemp;

#define PULSETIME 8000
#define SLEEPTIME 15000
#define PIXELTIME 5000

byte scaled;


void setup(){
  // Pulse sensor
  pinMode(buttonPin, INPUT);
  pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!
  Serial.begin(9600);             // we agree to talk fast!
//  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
   // UN-COMMENT THE NEXT LINE IF YOU ARE POWERING The Pulse Sensor AT LOW VOLTAGE, 
   // AND APPLY THAT VOLTAGE TO THE A-REF PIN
   //analogReference(EXTERNAL);   
   // end of pulse sensor

// heat sensor
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example
if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
//    while (1); // this makes the code stop running if the sensor is not found
  }

// end of heat sensor

// FASTLED
  FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUMPIXELS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<NEOPIXEL, 6>(neos, NUMNEOPIXELS);

  // set master brightness control
//  FastLED.setBrightness(BRIGHTNESS);


// FASTLED END


//Serial.begin(9600);


for(int i=0;i<NUMPIXELS;i++) {
  pixelPeriod[i]=60000/random(50,70);
  pixelTemp[i]=random(0,255);
}


  
  
 

}






void loop(){
  time=millis();


  



//Serial.println(sizeof(Matrix)/sizeof(Matrix[0]));
//Serial.println(sizeof(Matrix[0])/sizeof(Matrix[0][0]));

  // first dealing with readings

  buttonState = digitalRead(buttonPin);

  if(buttonState) {

  // this code reads pulse and color
  if(time < pixelTime) {

  if(QS) { // this only runs if the heart beat sensor is picking up a heart beat
    newPeriod == 60000/BPM;
  }

  newPeriod=60000/random(50,70); // this should be removed when the heart beat sensor is connected

  newTemp = readTemp();
  }
  

  if(time < sleepTime && time > pulseTime && pixelPushed == false && newPeriod && newTemp) {
    movePixels();
    pixelPeriod[0]=newPeriod;
    pixelTemp[0]=newTemp;
    pixelPushed=true;
  }

  if (time > sleepTime) {
    pixelTime=millis()+PIXELTIME;
    pulseTime=millis()+PULSETIME;
    sleepTime=millis()+SLEEPTIME;
    pixelPushed=false;
    }
  
  }
else {
    pixelTime=0;
    pulseTime=0;
    sleepTime=0;
  
}

// blinky lighs below


CRGBPalette16 myPal = heatmap_gp;
  // this code updates all the pixels
  for(int i=0;i<NUMPIXELS;i++) {
//    Serial.println(i);
//    getMatrix(i) = i;

    leds[i] = ColorFromPalette( myPal, pixelTemp[i]);
    leds[i].fadeLightBy(255-pixelAnim(pixelPeriod[i]));
  }


  for(int i=0; i<NUMNEOPIXELS;i++) {
    neos[i]=0;
  }
 
  
  if(time<pixelTime) {
    neoAnim(CRGB(255,255,255));
  }

  if(time>pixelTime && time<pulseTime) {
    for(int i=0;i<NUMNEOPIXELS;i++) {
      neos[i] = ColorFromPalette( myPal, newTemp);
      neos[i].fadeToBlackBy(255-pixelAnim(newPeriod));
      
    }
  }

    
  FastLED.show();



}

int readTemp() {
  float floatTemp;
  int newTemp;
  floatTemp=tempsensor.readTempC();


 
  int min=25;
  int max=32;
  newTemp=constrain(240*(floatTemp-min)/(max-min),0,240);

//  Serial.print(floatTemp);
//  Serial.print(" ");
//  Serial.println(newTemp);

  return newTemp;
}


byte pixelAnim(int period) {

  // denna körs för varje pixel vid varje uppdatering. returnerar den nuvarande färgen  
  //byte scaled = map(millis()%period,0,period,0,255);
  /*
int section=period/8;
  if(now < section)
    scaled = map(now,0,section,0,255);
  else if(now < section*2)
    scaled = map(now,section,section*2,255,0);
  else if(now < section*3)
    scaled = map(now,section*2,section*3,0,255);
  else if(now < section*6)
    scaled = map(now,section*3,section*6,255,0);
  else
    scaled = 0;

  return scaled;
*/
  return fadeArray[map(millis()%period,0,period,0,255)];
}

void neoAnim(CRGB color) {
  int now=PIXELTIME-(pixelTime-millis());
  Serial.print("now:");
  Serial.println(now);
  int maxpixel = (long)now*NUMNEOPIXELS/PIXELTIME+1;
  Serial.print("maxpixel:");
  Serial.println(maxpixel);
  for(int i =0 ;i<maxpixel;i++) {
    neos[i]=color;
  }
}


void movePixels() {
  // this code moves all pixels one step forward
  for(int i=NUMPIXELS-1;i>0;i--) {
    pixelPeriod[i]=pixelPeriod[i-1];
    pixelTemp[i]=pixelTemp[i-1];
  }
};

int getMatrix(int n){
  
     int x;
     int y;
     int MagicNumber;
     
     y = n / MatrixX;
     x = n - (MatrixX*y);
    // Serial.print("x: ");
    // Serial.println(x);
    // Serial.print("y: ");
    // Serial.println(y);
     MagicNumber = Matrix[y][x];
    
      
      Serial.print("Magic Number: ");
     Serial.println(MagicNumber);
     
      return MagicNumber;

 } ;


//CRGB colorFade(uint32_t color, int fade) {
// uint8_t r,g,b;
//   r=(color >> 16);
//   g=(color >>  8);
//   b=(color);
//   r=(uint8_t)r*(long)fade/255L;
//   g=(uint8_t)g*(long)fade/255L;
//   b=(uint8_t)b*(long)fade/255L;
// return CRGB(r,g,b);
//}

