/*
  Reading Position, Velocity and Time (PVT) via UBX binary commands

  Hardware Connections:
  Hook up the TX, RX and GND pins, plus 3V3 or 5V depending on your needs
  Connect: GNSS TX to microcontroller RX; GNSS RX to microcontroller TX
  Open the serial monitor at 115200 baud to see the output
*/
char foo;  // will let prog to comment out #define lines
// #define DEBUG

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "FS.h"
#include "SD.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

SPIClass hSPI(HSPI);


// miso vit 22
// mosi grå 21
//   cs gul 15
//  sck blå  4

// #define H_MISO 22
// #define H_MOSI 21
// #define H_CS   15
// #define H_SCK   4

#define H_MISO 02
#define H_MOSI 15
#define H_CS   13
#define H_SCK  14

#define RX 34
#define TX 04
#define gpsBaud 115200
#define led 12
// wroom dev kit v1 onBoardLed 02
#define onBoardLed 25

HardwareSerial uart(1);
TinyGPSPlus gps;
File dataFile;

//variables
unsigned long sdWait  = 120000; //millisecs
unsigned long arrayWait = 1000; //millisecs
unsigned long oledWait  = 5000; //millisecs
double lastDistLAT = 0, lastDistLON = 0; 
unsigned long totalDist = 0;
static const double START_LAT = 51.508131, START_LON = -0.128002;  // London
unsigned long arrayWriteTimer = millis();
unsigned long sdWriteTimer = millis();
unsigned long oledWriteTimer = millis();
int hourOffset = 2; //summer time
//
// variable to hold out nmea data and labels.
String nmea[7];
char *header[] = {"Nr","Time","Lat","Lon","Sats","Distance","FromHome"};

const int rows = 120;
String list[rows];
int i = 0;
unsigned int counter = 0;
bool cardMounted = false;
bool oledMounted = true;


void setup() {
  pinMode(led, OUTPUT);
  pinMode(onBoardLed,OUTPUT);
  digitalWrite(led, LOW);
  Serial.begin(115200);
  uart.begin(gpsBaud,SERIAL_8N1,RX,TX);
  hSPI.begin(H_SCK, H_MISO, H_MOSI, H_CS);
  
  //OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED SSD1306 allocation failed"));
    oledMounted = false;
    // for(;;); // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();
  display.display();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();

  // Serial.println(SPI.pinSS());
  // Serial.println("");
  Serial.print("MISO VIT ");
  Serial.println(H_MISO);
  Serial.print("MOSI GRÅ ");
  Serial.println(H_MOSI);
  Serial.print("  CS GUL ");
  Serial.println(H_CS);
  Serial.print(" SCK BLÅ ");
  Serial.println(H_SCK);
  Serial.println("");

  cardMounted = checkForSD_card();
  // if (!SD.begin(H_CS,hSPI)) {
  //     Serial.println("Card Mount Failed");
  //     return;
  // }
  // else{
  //   digitalWrite(onBoardLed, HIGH);
  // }

  printSdType();
  writeHeader();

  readFile(SD, "/gps.txt");
  Serial.println(F(""));    
}





void loop() {

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

  while (uart.available() > 0){
    gps.encode(uart.read());
    // if (gps.encode(uart.read())){
    //   displayInfo();
    //   // Serial.println("first loop for printing");
    //   // Serial.println("");
    // }  
    // if (gps.location.isValid()){
    //   displayInfo();
    //   Serial.print("distance to home: ");
    //   Serial.println(displayHome());
    // }

    // smartDelay(1000);
    if (lastDistLAT == 0){
        lastDistLAT = gps.location.lat();
        lastDistLON = gps.location.lng();
    }

    if (gps.location.isValid()){
      digitalWrite(led, HIGH);}
      else{digitalWrite(led, LOW);
    }

    if (millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
    
    // collect data in array before writing to sd.
    // write to array every arrayWait
    if (millis() - arrayWriteTimer > arrayWait){
      arrayWriteTimer = millis();
      if (gps.satellites.value() >= 3){
        list[i] = buildLocationString();
        counter++;

        #ifdef DEBUG
          Serial.print("parameter i: ");
          Serial.print(i);
          Serial.print(" - SD-card mounted = ");
          Serial.println(cardMounted);
          Serial.println(list[i]);
        #endif

        if (i < rows){
          i++;
        }
        else{
          cardMounted = checkForSD_card();
          i = 0;
        }
      }   
    }
    // print to screen Oled
    #ifdef OLED
      printToOled();
    #endif

    // write to SD every sdWait
    if ((millis() - sdWriteTimer > sdWait) and cardMounted){
      sdWriteTimer = millis();
      appendArray(SD, "/gps.txt", list, i); 
      i = 0;

      blinkLED();

      #ifdef DEBUG
        readFile(SD, "/gps.txt");
        Serial.println(F(""));      
      #endif

      // if (gps.satellites.value() >= 3){
      //   appendFile(SD, "/gps.txt", buildLocationString().c_str()); //c_str() to convert from string to const char as expected from function
      //   Serial.println(buildLocationString());
      // }
    }

  }
}


void printToOled(){
      if ((millis() - oledWriteTimer > oledWait) and oledMounted){
      oledWriteTimer = millis();
      #ifdef DEBUG
        Serial.println("--- OLED ---");
      #endif

      if (!cardMounted){
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(2, 5);
        display.print("No SD-card found");
      }
      else{
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(2, 5);
        display.print("SD-card mounted");
      }

        display.setCursor(2, 15);
        display.print("From home: ");
        display.setCursor(65, 15);
        display.print(displayHome(), 1);

        display.setCursor(2, 25);
        display.print("Dst: ");
        display.setCursor(31, 25); // horizontal , vertical
        display.print(totalDist);

        display.setCursor(2, 35);
        display.print("Lat: ");
        display.setCursor(31, 35);
        display.print(gps.location.lat(), 6);

        display.setCursor(2, 45);
        display.print("Lng: ");
        display.setCursor(31, 45); // horizontal , vertical
        display.print(gps.location.lng(), 6);

        display.setCursor(2, 55);
        display.print("Spd: ");
        display.setCursor(31, 55); // horizontal , vertical
        // display.print(gps.speed.kmph()/60, 0);
        display.print(minPerKM(gps.speed.kmph() ));


        display.display();
    }
}

String minPerKM(double kmph){
  // kmph = 18.65123;
  String result = "0, 00";
  int seconds = 3600 / kmph;
  int m = seconds / 60;
  int s = seconds - m * 60;

  // Serial.print(kmph);
  // Serial.print(" , ");
  // Serial.print(seconds);
  // Serial.print(" , ");
  // Serial.print(m);
  // Serial.print(" , ");
  // Serial.print(s);
  // Serial.println("");
  if (m <10){
    result = m;
    result += ":";
    result += s;
    result += " min/km";
  }
  else{
    result = "-:-- min/km";
  }
  return result;
}

bool checkForSD_card(){
  bool success = false;
  if (!SD.begin(H_CS,hSPI)) {
      Serial.println("Card Mount Failed");
      success = false;
  }
  else{
    digitalWrite(onBoardLed, HIGH);
    success = true;
  }
  return success;
}

void writeHeader(){
  
  // char *updates[] = {"Nr","Time","Lat","Lon","Sats","Distance","FromHome"};

  // writeFile(SD, "/hello.txt", "Hello ");  
  // writeFile(SD, "/hello.txt", * header);  

    dataFile = SD.open("/gps.txt", FILE_WRITE);
    if (dataFile) {
      Serial.println("writing");
      // nr
      dataFile.print(header[0]);
      dataFile.print(",");
      // time
      dataFile.print(header[1]);
      dataFile.print(",");
      // latitude
      dataFile.print(header[2]);
      dataFile.print(",");
      // longitude
      dataFile.print(header[3]);
      dataFile.print(",");
      // sats
      dataFile.print(header[4]);
      dataFile.print(",");
      // distance
      dataFile.print(header[5]);
      dataFile.print(",");
      // distance_from_home
      dataFile.print(header[6]);
      dataFile.println("");
      Serial.println("done");  
    }
    else {
      Serial.println("error writing header to SD ");  
    }  
    // close file on SD card
    dataFile.close();

}

void blinkLED(){
    digitalWrite(onBoardLed, HIGH);
    smartDelay(200);
    digitalWrite(onBoardLed, LOW);
    smartDelay(200);
    digitalWrite(onBoardLed, HIGH);
    smartDelay(200);
    digitalWrite(onBoardLed, LOW);
    smartDelay(200);
    digitalWrite(onBoardLed, HIGH);
    smartDelay(200);
    digitalWrite(onBoardLed, LOW);
    smartDelay(200);
    digitalWrite(onBoardLed, HIGH);
    smartDelay(200);
    digitalWrite(onBoardLed, LOW);
    smartDelay(200);
}

unsigned long displayHome(){
  unsigned long distanceKmToStart = 0;
  if (gps.location.isValid()){
      distanceKmToStart =
      (unsigned long)TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        START_LAT, 
        // START_LON) / 1000; //km
        START_LON); // meters
  }
  return distanceKmToStart;
}

// 1
unsigned long writeToSD() {
  // dataFile = SD.open("gps.txt", FILE_WRITE);
  // if (dataFile) {
  //   Serial.println("writing");
  //   // update number
  //   dataFile.print(updates);
  //   dataFile.print(",");  
  //   // time
  //   dataFile.print(nmea[0]);
  //   dataFile.print(",");
  //   // latitude
  //   dataFile.print(nmea[1]);
  //   dataFile.print(",");
  //   // longitude
  //   dataFile.print(nmea[3]);
  //   dataFile.println("");
  //   Serial.println("done");  

  // }
  // else {
  //   Serial.println("error");  
  // }  

}

String buildLocationString(){
  //{"Nr","Time","Lat","Lon","Sats","Distance","FromHome"};
  String dataString = "";
  // dataString += 0;   //Nr
  dataString += counter;   //Nr
  dataString += ","; //Time
  dataString += String(gps.date.year());
  dataString += "-";
  if (gps.date.month() < 10) dataString += "0";
  dataString += String(gps.date.month());
  dataString += "-";
  if (gps.date.day() < 10) dataString += "0";
  dataString += String(gps.date.day());
  dataString += " ";
  if (gps.time.hour() < 10) dataString += "0";
  dataString += String(gps.time.hour() + hourOffset);
  dataString += ":";
  if (gps.time.minute() < 10) dataString += "0";
  dataString += String(gps.time.minute());
  dataString += ":";
  if (gps.time.second() < 10) dataString += "0";
  dataString += String(gps.time.second());
  dataString += ".";
  dataString += String(gps.time.centisecond());
  dataString += ","; //Lat
  // dataString += String(gps.location.lat(), 6); 
  double lat = gps.location.lat();
  dataString += String(lat, 6); 
  dataString += ","; //Lon
  // dataString += String(gps.location.lng(), 6);
  double lon =  gps.location.lng();
  dataString += String(lon, 6);
  dataString += ","; //Sats
  dataString += String(gps.satellites.value());
  dataString += ","; //Distance
  // #ifdef DEBUG
  //       Serial.println(lat);      
  //       Serial.println(", ");      
  //       Serial.println(lon);      
  //       Serial.println(", ");      
  //       Serial.println(lastDistLAT);      
  //       Serial.println(", ");      
  //       Serial.println(lastDistLON);      
  // #endif
  float dist = TinyGPSPlus::distanceBetween(lat,lon,lastDistLAT,lastDistLON);
  // Serial.println(TinyGPSPlus::distanceBetween(lat,lon,lastDistLAT,lastDistLON));
  // Serial.println(dist);
  totalDist = totalDist + dist; // totalDist is defined as type long, so it only adds if dist is greater than one. Acts as a filter against small jitter movement.
  // Serial.println(totalDist);
  dataString += String(totalDist);
  lastDistLAT = lat;
  lastDistLON = lon;

  // dataString += 0;
  dataString += ","; //FromHome
  dataString += displayHome();
  dataString += "\n";
  // Serial.println(dataString);
return dataString;
}

void printSdType(){
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.println("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}


void appendArray(fs::FS &fs, const char * path, String * theList, int sizeOfIt){
    Serial.printf("Appending the list to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        checkForSD_card();
        return;
    }
    for (int k = 0; k < sizeOfIt; k++){
      if(!file.print(theList[k])){
        Serial.println("Append of the list failed");
      }
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

// This custom version of delay() ensures that the gps object is being "fed".
static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do {
    while (uart.available() > 0)
      gps.encode(uart.read());
  } while (millis() - start < ms);
}

void displayInfo(){
  if (gps.location.isValid())
  {
    // digitalWrite(led, HIGH); //moved to main loop
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    if (gps.satellites.value() < 3){
      digitalWrite(led, LOW);
    }

    Serial.print(F("Location: ")); 
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    digitalWrite(led, LOW);
    Serial.print(F("INVALID "));
  }

  if (gps.date.isValid())
  {
    Serial.print(F("  Date/Time: "));
    Serial.print(gps.date.year());
    Serial.print(F("-"));
    if (gps.date.month() < 10) Serial.print(F("0"));
    Serial.print(gps.date.month());
    Serial.print(F("-"));
    if (gps.date.day() < 10) Serial.print(F("0"));
    Serial.print(gps.date.day());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}