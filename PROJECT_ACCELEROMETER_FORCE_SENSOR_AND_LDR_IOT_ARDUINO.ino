

/* This code reads and filters data from Accelerometer on the Arduino IoT,
 a force sensitive sensor and LDR,  to prints assigned values on the serial. Those assigned values are later read by processing to
    trigger changes on the animations. This also triggers animations on the NeoPixels connected to the Arduino.


  /*
   FORCE SENSOR WIRING:
   leg to 5V
   leg connected to ground throu 10k resistor and to pin analog 0 pin on the arduino.

LDR SENSOR / BUTTON
Analog Pin 2 to read.
LDR connected directly to ground and a to 5V throug a 3K Resistor.

   NEOPIXELS WIRING
   Data pin connected to Analog Pin 1, throu 330ohms resistor
   
   Code References:
   > NEO Pixels animations based on https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#LEDStripEffectTwinkle
   > For filtering the data, I adapted the Smoothing Example from the Arduino IDE.
   
   Written By: Kelvin Feliz, 2020.
   
*/





////LIBRARIES////

#include <SimpleKalmanFilter.h> // filter library
#include <Arduino_LSM6DS3.h> //Accelerometer library




//NEOPIXELS//
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define NUMPIXELS 7 // Number of NEOPIXELS controlled with the sensor
#define PIN       A1 // Where the NEOPIXEL data input is conected. Must be connected to an analog pin.
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);




//ACCELEROMETER VARIABLES//
const int numReadings = 5;
int readIndex = 0;              // the index of the current reading
float readingsX[numReadings];      // the readings from the X axis on accelerometer
float readingsY[numReadings];      // the readings from the Y axis in the accelerometer
float totalX = 0;                  // the running total of X
float totalY = 0;                  // the running total of Y
float averageX = 0;                // the average of X
float averageY = 0;                // the average of Y
float accelerometerX;               //Readings of X
float accelerometerY;         //Readings of Y

//FORCE SENSOR VARIABLES
int fsrAnalogPin = 0;               //Analog Pin to which the sensor is connected. (A0)
int fsrReading = 0;
const int numReadingsFsr = 20;
int readIndexFsr = 0;              // the index of the current reading on the FSR sensor
int readingsFsr[numReadingsFsr];      // the readings from the force sensor
int totalFsr = 0;                  // the running total of Fsr Sensor
int averageFsr = 0;                // the average of FSR sensor

//LDR VARIABLES
int ldrAnalogPin = 2; // A2
int ldrReading = 0;



void setup() {

  Serial.begin(9600);
  while (!Serial);
  //Serial.println("Started");

  ///LDR PIN SETUP
  pinMode (ldrAnalogPin, INPUT);

  //// ACCELEROMETER CONFIGURATION/////
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }



  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsX[thisReading] = 0;
    readingsY[thisReading] = 0;
    readingsFsr[thisReading] = 0;
  }




  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();          // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void loop(void) {

   // buttonState = digitalRead(buttonPin);

 

  AcceAverage();
  forceAverage();

if ((  averageFsr > 0) && (  averageFsr <= 20)) {
    strip.clear();
    strip.show();
    delay (10);
  } 

  
  if ((  averageFsr > 20) && (  averageFsr <= 100)) {
    strip.setPixelColor (0, 255, 0, 0);
    strip.show();
    delay (10);

  } 

    if ((  averageFsr > 100) && (  averageFsr <= 400)) {
      strip.setPixelColor (0, 255, 0, 0);
      strip.setPixelColor (1, 255, 0, 0);
      strip.show();
      delay (10);

    }

  if ((  averageFsr > 400) && (  averageFsr <= 700)) {
       strip.setPixelColor (0, 255, 0, 0);
      strip.setPixelColor (1, 255, 0, 0);
      strip.setPixelColor (2, 255, 0, 0);
      strip.show();
      delay (10);

    }

    
  if ((  averageFsr > 700) && (  averageFsr <= 800)) {
            strip.setPixelColor (0, 255, 0, 0);
      strip.setPixelColor (1, 255, 0, 0);
      strip.setPixelColor (2, 255, 0, 0);
      strip.setPixelColor (3, 255, 0, 0);
      strip.show();
      delay (10);

    } 

      if ((  averageFsr > 800) && (  averageFsr<= 850)) {
               strip.setPixelColor (0, 255, 0, 0);
      strip.setPixelColor (1, 255, 0, 0);
      strip.setPixelColor (2, 255, 0, 0);
      strip.setPixelColor (3, 255, 0, 0);
      strip.setPixelColor (4, 255, 0, 0);
      strip.show();
      delay (10); }
      
            if ((  averageFsr > 850) && (  averageFsr <= 900)) {
               strip.setPixelColor (0, 255, 0, 0);
      strip.setPixelColor (1, 255, 0, 0);
      strip.setPixelColor (2, 255, 0, 0);
      strip.setPixelColor (3, 255, 0, 0);
      strip.setPixelColor (4, 255, 0, 0);
      strip.setPixelColor (5, 255, 0, 0);
      strip.show();
      delay (10); }
      
               if ((  averageFsr > 900) && (  averageFsr <= 1022)) {
               strip.setPixelColor (0, 255, 0, 0);
      strip.setPixelColor (1, 255, 0, 0);
      strip.setPixelColor (2, 255, 0, 0);
      strip.setPixelColor (3, 255, 0, 0);
      strip.setPixelColor (4, 255, 0, 0);
      strip.setPixelColor (5, 255, 0, 0);
       strip.setPixelColor (6, 255, 0, 0);
      strip.show();
      delay (10); } 

            if ((ldrReading == 1023)) {
      Fire(55,120,15);
      
      delay (10); } 
     
                  if ((averageFsr == 1023)) {
   theaterChase(strip.Color(127, 127, 127), 30); // White, half brightness
       } else { strip.clear(); }

       

  ////PRINTS ALL THE SENSORS DATA////
  printSensors();

}


void Fire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[NUMPIXELS];
  int cooldown;
 
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUMPIXELS; i++) {
    cooldown = random(0, ((Cooling * 10) / NUMPIXELS) + 2);
   
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
 
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUMPIXELS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
   
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUMPIXELS; j++) {
    setPixelHeatColor(j, heat[j] );
  }

  showStrip();
  delay(SpeedDelay);
}

void setPixelHeatColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixel(Pixel, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixel(Pixel, 255, heatramp, 0);
  } else {                               // coolest
    setPixel(Pixel, heatramp, 0, 0);
  }
}

void showStrip() {
  strip.show();
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELS; i++ ) {
    setPixel(i, red, green, blue);
  }
  showStrip();
}

void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

void forceAverage () {
  // subtract the last force reading:
  totalFsr = totalFsr - readingsFsr[readIndexFsr];

  // read from the force sensor
  fsrReading = analogRead(fsrAnalogPin);
  readingsFsr[readIndexFsr] = fsrReading;

  // add the reading to the total:
  totalFsr = totalFsr + readingsFsr[readIndexFsr];

  // advance to the next position in the array:
  readIndexFsr = readIndexFsr + 1;

  // if we're at the end of the array...
  if (readIndexFsr >= numReadingsFsr) {
    // ...wrap around to the beginning:
    readIndexFsr = 0;
  }
  // calculate the average:
  averageFsr = totalFsr / numReadingsFsr;
}

//delay(1000);        // delay in between reads for stability



void AcceAverage () {

  // subtract the last accelerometer reading:
  totalX = totalX - readingsX[readIndex];
  totalY = totalY - readingsY[readIndex];

  // read from the accelerometer:
  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    accelerometerX = x;
    accelerometerY = y;
    readingsX [readIndex] = x;
    readingsY[readIndex] = y;
  }

  // add the reading to the total:
  totalX = totalX + readingsX[readIndex];
  totalY = totalY + readingsY[readIndex];

  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  averageX = totalX / numReadings;
  averageY = totalY / numReadings;

  //delay(1000);        // delay in between reads for stability
}



void printSensors() {
  //   Accelerometer
  Serial.print(averageX);
  Serial.print (',');
  Serial.print(averageY);
  Serial.print (',');

//LDR resistor-buttom
  ldrReading = analogRead (ldrAnalogPin);
  Serial.print (ldrReading);
  Serial.print (',');
 
  //Force Sensor
  Serial.print(averageFsr);
 Serial.print(',');


   // Depending on how hard the FSR is pressed, different status will be printed on the serial
  if (averageFsr == 1023) {
    // hardpressed
   Serial.println('2');
  } else if  ((  averageFsr >= 400) && (  averageFsr <= 1022)) {
    // mediumpressed // using this value for counter on Proccessing
      Serial.println('1');

      //Nopressed or barely pressed
  } else if ((  averageFsr >= 0) && (  averageFsr <= 400)) {
    Serial.println('0');
  }




}
