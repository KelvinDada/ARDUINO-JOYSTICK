



/* This code reads and filters data from PDM Microphone and Accelerometer on the Arduino BLE Sense,
    and a force sensitive sensor,
    to prints assigned values on the serial. Those assigned values are later read by processing to
    trigger changes on the animations.

  > Eliminated the if (sampleread) conditionals from the Mic block and that fixed the accelerometer slowin
  problem on Processing.

  /*
   FORCE SENSOR WIRING:
   leg to 5V
   leg connected to ground throu 10k resistor and to pin analog 0 pin on the arduino.



   NEOPIXELS WIRING
   Data pin connected to Analog Pin 1, throu 330ohms resistor
*/





////LIBRARIES////

#include <SimpleKalmanFilter.h> // filter library
#include <Arduino_LSM6DS3.h> //Accelerometer library


//WIFI
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(80);

WiFiClient client;


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




void setup() {

  //WIFI CONFIGURATION
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();
  WiFiClient client = server.available();
  // you're connected now, so print out the status:
  printWifiStatus();






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

WiFiClient client = server.available();

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

  if ((  averageFsr > 800) && (  averageFsr <= 850)) {
    strip.setPixelColor (0, 255, 0, 0);
    strip.setPixelColor (1, 255, 0, 0);
    strip.setPixelColor (2, 255, 0, 0);
    strip.setPixelColor (3, 255, 0, 0);
    strip.setPixelColor (4, 255, 0, 0);
    strip.show();
    delay (10);
  }

  if ((  averageFsr > 850) && (  averageFsr <= 900)) {
    strip.setPixelColor (0, 255, 0, 0);
    strip.setPixelColor (1, 255, 0, 0);
    strip.setPixelColor (2, 255, 0, 0);
    strip.setPixelColor (3, 255, 0, 0);
    strip.setPixelColor (4, 255, 0, 0);
    strip.setPixelColor (5, 255, 0, 0);
    strip.show();
    delay (10);
  }

  if ((  averageFsr > 900) && (  averageFsr <= 1023)) {
    strip.setPixelColor (0, 255, 0, 0);
    strip.setPixelColor (1, 255, 0, 0);
    strip.setPixelColor (2, 255, 0, 0);
    strip.setPixelColor (3, 255, 0, 0);
    strip.setPixelColor (4, 255, 0, 0);
    strip.setPixelColor (5, 255, 0, 0);
    strip.setPixelColor (6, 255, 0, 0);
    strip.show();
    delay (10);
  } else {
    strip.clear();
  }


  ////PRINTS ALL THE SENSORS DATA////
  printSensors();

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
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: open");  // the connection will be closed after completion of the response
          client.println("Refresh: 0");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");

          //   Accelerometer
          client.print(averageX);
          client.print (',');
          client.print(averageY);
          client.print (',');


          //Force Sensor
          client.print(averageFsr);
          client.print(',');


          // Depending on how hard the FSR is pressed, different status will be printed on the serial
          if (averageFsr == 1023) {
            // hardpressed
            client.println('2');
          } else if  ((  averageFsr >= 400) && (  averageFsr <= 1022)) {
            // mediumpressed // using this value for counter on Proccessing
            client.println('1');

            //Nopressed or barely pressed
          } else if ((  averageFsr >= 0) && (  averageFsr <= 400)) {
            client.println('0');
          }
          client.println(client.remotePort());

          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    //client.stop();
    Serial.println("client disconnected");

  }

}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
