// Test code for Adafruit Flora GPS modules
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Flora GPS module
//    ------> http://adafruit.com/products/1059
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada



#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_NeoPixel.h>


//Adafruit_GPS GPS(&Serial1);
//HardwareSerial mySerial = Serial1;

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):



// Pin D7 has an LED connected on FLORA.
// give it a name:
int led = 7;

#define PIN 6

int nLEDs=55;

// Chose 2 pins for output; can be any valid output pins:

Adafruit_NeoPixel strip = Adafruit_NeoPixel(nLEDs, PIN, NEO_GRB + NEO_KHZ800);


LSM303 compass;
int compasLEDs=8;
//--------------------------------------------------|
//                    WAYPOINTS                     |
//--------------------------------------------------|
//Please enter the latitude and longitude of your   |
//desired destination:                              |
  #define GEO_LAT                35.260695
  #define GEO_LON               -120.693534
//--------------------------------------------------|

//--------------------------------------------------|
//                    DISTANCE                      |
//--------------------------------------------------|
//Please enter the distance (in meters) from your   |
//destination that you want your LEDs to light up:  |
  #define DESTINATION_DISTANCE_0 20   // WHITE
  #define DESTINATION_DISTANCE_1 50   // GREEN
  #define DESTINATION_DISTANCE_2 200  // BLUE
  #define DESTINATION_DISTANCE_3 500  // YELLOW
  #define DESTINATION_DISTANCE_4 1000 // RED
//--------------------------------------------------|

// Navigation location
float targetLat = GEO_LAT;
float targetLon = GEO_LON;

// Trip distance
float tripDistance;

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

//Adafruit_NeoPixel strip = Adafruit_NeoPixel(45, 6, NEO_GRB + NEO_KHZ800);
//int FarRight = 9;
//int CenterRight = 10;
//int CenterLeft = 34;
//int FarLeft = 35;
//int HeadsUp[] = {35, 34, 10, 9};
//int RightStrip[] = {8, 7, 6, 5, 4, 3, 2, 1, 0};
//int RightCenterStrip[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
//int LeftCenterStrip[] = {33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23};
//int LeftStrip[] = {36, 37, 38, 39, 40, 41, 42, 43, 44};
//int counter = 0;

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

void setup()  
{
    pinMode(led, OUTPUT);   
    // Start up the LED strip
  strip.begin();

  // Update the strip, to start they are all 'off'
  strip.show();
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(5000);
  
  Serial.println("initialize");
  //test to find the closest location from the list, just plug in a lat lon from the above Array
  //Serial.print("Closest Test Loc: ");
  //Serial.println(find_closest_location(40.726378, -74.005437));

  Wire.begin();
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  compass.init();
  compass.enableDefault();
  // Calibration values. Use the Calibrate example program to get the values for
  // your compass.
//  M min X: -543 Y: -354 Z: -466 M max X: 523 Y: 693 Z: 481

  compass.m_min.x = -543; compass.m_min.y = -354; compass.m_min.z = -466;
  compass.m_max.x = +523; compass.m_max.y = 693; compass.m_max.z = 481;
  delay(1000);
  // Ask for firmware version
  Serial1.println(PMTK_Q_RELEASE);
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  for(int i=0; i<nLEDs; i++) {
      strip.setPixelColor(i,0,0,127); // (led#,r,g,b) r/g/b = 0-127
    }
  strip.show();
  delay(1000);
  for(int i=0; i<nLEDs; i++) {
      strip.setPixelColor(i,0,0,0); // (led#,r,g,b) r/g/b = 0-127
    }
  strip.show();
}



uint32_t timer = millis();
void loop()                     // run over and over again
{
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
      if (c) Serial.print(c);
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // every 300 milliseconds, update the heading/distance indicators
  if (millis() - timer > 300) { 
    timer = millis(); // reset the timer
    if (GPS.fix) {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      Serial.print("GPS FIX");      
      float fLat = decimalDegrees(GPS.latitude, GPS.lat);
      float fLon = decimalDegrees(GPS.longitude, GPS.lon);
      clearPixels();
      float currentHeading = getHeading(); //your curent heading
      float currentDistance = (double)calc_dist(fLat, fLon, targetLat, targetLon); // how far you are from your waypoint
      float currentBearing = calc_bearing(fLat, fLon, targetLat, targetLon); //calc the deg bearing to your desitination
      float adjustBearing = currentBearing - currentHeading; //adjusted bearing is the amount of degress you need to turn 
      if (adjustBearing > 0) {
        headingDirection(adjustBearing, currentDistance);
       }
      else {
        headingDirection(adjustBearing+360, currentDistance);
      }     
      //Serial.print("Distance Remaining:"); Serial.println((double)calc_dist(fLat, fLon, targetLat, targetLon));
    }
//    else {
//      strip.setPixelColor(nLEDs / 2,0,0,127); // (led#,r,g,b) r/g/b = 0-127
//      strip.setPixelColor(nLEDs / 2 + 1,0,0,127); // (led#,r,g,b) r/g/b = 0-127
//      strip.setPixelColor(nLEDs / 2 - 1,0,0,127); // (led#,r,g,b) r/g/b = 0-127
//      strip.show();
//      delay(1000);
//      strip.setPixelColor(nLEDs / 2,0,0,0); // (led#,r,g,b) r/g/b = 0-127
//      strip.setPixelColor(nLEDs / 2 + 1,0,0,0); // (led#,r,g,b) r/g/b = 0-127
//      strip.setPixelColor(nLEDs / 2 - 1,0,0,0); // (led#,r,g,b) r/g/b = 0-127
//      strip.show();
//      delay(1000);
//    }
  }
}

int getHeading () {
    compass.read();
    int heading = compass.heading((LSM303::vector){0,-1,0});
    heading = heading+135;
    Serial.print("Heading: ");
    Serial.println(heading);
    return heading;
}

int pixelArray0[5]={23,25,27,28,30};
int pixelArray1[4]={0,19,27,35};
int pixelArray2[4]={5,16,27,38};
int pixelArray3[4]={41,27,14,10};
int pixelArray4[5]={24,26,27,29,31};
int pixelArray5[4]={19,27,35,54};
int pixelArray6[4]={16,27,38,49};
int pixelArray7[4]={44,41,27,14};

void headingDirection(float heading, float distance) 
{
  int multiplier = 360/compasLEDs;
  int pixel = heading/multiplier;
  if (pixel==0) {
    pixel=32;
    for(int i = 0; i < sizeof(pixelArray0)/2; i++) {
      strip.setPixelColor(pixelArray0[i],127,80,0);
    }
  }
  if (pixel==1) {
    pixel=54;
    for(int i = 0; i < sizeof(pixelArray1)/2; i++) {
      strip.setPixelColor(pixelArray1[i],127,80,0);
    }
  }
  if(pixel==2) {
    for(int i = 0; i < sizeof(pixelArray1)/2; i++) {
      strip.setPixelColor(pixelArray2[i],127,80,0);
    }
   pixel=49; 
  }
   if(pixel==3) {
     for(int i = 0; i < sizeof(pixelArray1)/2; i++) {
      strip.setPixelColor(pixelArray3[i],127,80,0);
    }
   pixel=44; 
  }
   if(pixel==4) {
     for(int i = 0; i < sizeof(pixelArray4)/2; i++) {
      strip.setPixelColor(pixelArray4[i],127,80,0);
    }
   pixel=22; 
  }
   if(pixel==5) {
     for(int i = 0; i < sizeof(pixelArray1)/2; i++) {
      strip.setPixelColor(pixelArray5[i],127,80,0);
    }
   pixel=0; 
  }
   if(pixel==6) {
     for(int i = 0; i < sizeof(pixelArray1)/2; i++) {
      strip.setPixelColor(pixelArray6[i],127,80,0);
    }
   pixel=5; 
  }
   if(pixel==7) {
     for(int i = 0; i < sizeof(pixelArray1)/2; i++) {
      strip.setPixelColor(pixelArray7[i],127,80,0);
    }
   pixel=10; 
  }
   if(pixel==8) {
      for(int i = 0; i < sizeof(pixelArray0)/2; i++) {
      strip.setPixelColor(pixelArray0[i],127,80,0);
    }
   pixel=32; 
  }
  if (distance <= DESTINATION_DISTANCE_0) { //Distance:20-0
   strip.setPixelColor(pixel, 127, 127, 127); //WHITE
  }
  if (distance > DESTINATION_DISTANCE_0 && distance <= DESTINATION_DISTANCE_1) { //Distance:21-50M
   strip.setPixelColor(pixel, 0, 127, 0); //GREEN
  }
  if (distance > DESTINATION_DISTANCE_1 && distance <= DESTINATION_DISTANCE_2) {//Distance:51-200M
   strip.setPixelColor(pixel, 0, 0, 127); //BLUE
  }
  if (distance > DESTINATION_DISTANCE_2 && distance <= DESTINATION_DISTANCE_3) {//Distance:201-500M
   strip.setPixelColor(pixel, 127, 127, 0); //YELLOW
  }
  if (distance > DESTINATION_DISTANCE_3 && distance <= DESTINATION_DISTANCE_4) {//Distance:501M-1000M
    strip.setPixelColor(pixel, 127, 0, 127); //VIOLET
  }
  if (distance > DESTINATION_DISTANCE_4) {//Distance: 2000M+
    strip.setPixelColor(pixel, 127, 0, 0); //RED
  }
  strip.show();
}


int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);

  calc=atan2(y,x);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc; 
  }
  return bear_calc;
}

unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
}

//returns the location in the lat_lon array of the closest lat lon to the current location
//int find_closest_location(float current_lat, float current_lon)
//{  
//  int closest = 0;
//  unsigned long minDistance = -1;
//  unsigned long tempDistance;
//  for (int i=0; i < LAT_LON_SIZE; i++) {
//    float target_lat = pgm_read_float(&lat_lon[i][0]);
//    float target_lon = pgm_read_float(&lat_lon[i][1]);
//
//    tempDistance = calc_dist(current_lat, current_lon, target_lat, target_lon);
//    
//    /*
//    Serial.print("current_lat: ");
//    Serial.println(current_lat, 6);
//    Serial.print("current_lon: ");
//    Serial.println(current_lon, 6); 
//    Serial.print("target_lat: ");
//    Serial.println(target_lat, 6);
//    Serial.print("target_lon: ");
//    Serial.println(target_lon, 6);  
//    
//    Serial.print("tempDistance: ");
//    Serial.println(tempDistance);
//    Serial.print("Array Loc: ");
//    Serial.println(i); 
//    */
//    
//    if ((minDistance > tempDistance) || (minDistance == -1)) {
//      minDistance = tempDistance;
//      closest = i;
//
//    }
// 
//  }
//  return closest;
//}

// Convert NMEA coordinate to decimal degrees
float decimalDegrees(float nmeaCoord, char dir) {
  uint16_t wholeDegrees = 0.01*nmeaCoord;
  int modifier = 1;

  if (dir == 'W' || dir == 'S') {
    modifier = -1;
  }
  
  return (wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0) * modifier;
}

void clearPixels () {
  int i;
  for (i=0; i<nLEDs; i++) {
      strip.setPixelColor(i, 0, 0, 0); 
  }
}

