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

#include <Bme280.h> //BME

#define OLED             // OLED premounted to the ESP-board from Lilygo via I2C
#define BMP280           // BMP280 via I2C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

SPIClass hSPI(HSPI);
Bme280TwoWire sensor; //BME

// SD-card SPI-bus
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
static const double START_LAT = 51.508131, START_LON = -0.128002;  // London
double lastDistLAT = 0, lastDistLON = 0; 
unsigned long totalDist = 0;
unsigned long arrayWriteTimer = millis();
unsigned long sdWriteTimer = millis();
unsigned long oledWriteTimer = millis();
unsigned long fileNameTimer = millis();

unsigned long dimDisplayTimer;
bool dimDisplay =false;
int display_screen = 0;

int hourOffset = 2; //summer time
int altitude = -1;
//
// variable to hold out nmea data and labels.
// String nmea[7];
char* header[] = {"Nr","Time","Lat","Lon","Sats","Distance","FromHome","Alt"};
char* fileName = "/gps.txt";  // format is "/gps.txt"
char* myfileName = "/gps_IIa.txt";  // format is "/gps.txt"
char str_array[27];

const int rows = 120;
String gpsData[rows][8];
String list[rows];
int i = 0;
unsigned int counter = 0;
bool cardMounted = false;
bool fileInit = false;
// bool oledMounted = true;

int altDiff = 0;
bool setAltDiffOnce = true;

// button press
void IRAM_ATTR isr();
struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

// Button button1 = {18, 0, false}; // use pin 18, first number
Button button1 = {0, 0, false}; // use pin 0, first number

//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;  
unsigned long last_button_time = 0; 


// struct experiment
// String apa = "höna";
String apa("höna");
struct{
  char* Position;
  String values[6];
}data[7] = {
  {"8",{"2024-04-23 11:43:35.0","59.280623","18.075560","11","7",apa }},
  {"9",{"2024-04-23 11:43:36.0","59.280624","18.075556","11","7","11"}},
  {"10",{"2024-04-23 11:43:37.0","59.280626","18.075550","11","7","11"}},
  {"11",{"2024-04-23 11:43:38.0","59.280627","18.075547","11","7","11"}},
  {"12",{"2024-04-23 11:43:39.0","59.280639","18.075549","11","8","12"}},
  {"13",{"2024-04-23 11:43:40.0","59.280645","18.075549","11","8","13"}},
  {"14",{"2024-04-23 11:43:41.0","59.280645","18.075549","11","8","14"}},
};


void setup() {
  // WiFi.mode(WIFI_OFF);
  pinMode(led, OUTPUT);
  pinMode(onBoardLed,OUTPUT);
  digitalWrite(led, LOW);
  Serial.begin(115200);
  uart.begin(gpsBaud,SERIAL_8N1,RX,TX);
  hSPI.begin(H_SCK, H_MISO, H_MOSI, H_CS);

  Wire.begin(); //BME
  sensor.begin(Bme280TwoWireAddress::Primary); //BME
  sensor.setSettings(Bme280Settings::indoor()); //BME

// struct experiment
  for(int t = 0;t<7;++t){
    Serial.print(data[t].Position);
    Serial.print(" ");
      for(int q = 0;q<6;++q){
        Serial.print(data[t].values[q]);
        Serial.print(",");
      }
    Serial.println("");
  }
  //OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED SSD1306 allocation failed"));
    // oledMounted = false;
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

  // button press
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr, FALLING);

  if(checkForSD_card()){
    cardMounted = true;
    // initFileOnCard();
    // fileInit = true;
    Serial.println(F(""));

  }    
}





void loop() {

  // if (counter <= 30){
  //   dimDisplayTimer = 0;
  // }
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

    if (millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }

    // Set first location to calc distans. Maybe to simple
    if ((lastDistLAT == 0) or lastDistLON == 0){
        lastDistLAT = gps.location.lat();
        lastDistLON = gps.location.lng();
    }
    


    if (gps.location.isValid()){
      digitalWrite(led, HIGH);      }
      else{ digitalWrite(led, LOW); }

    // calc diff pressure altitude from gps. Wait for stabilized gps.altitude
    if (setAltDiffOnce){
      if (millis() > 30000 && gps.location.isValid()){
        if ( gps.altitude.meters() != 0){
          altDiff = gps.altitude.meters() - bmp280();
          setAltDiffOnce = false;
          #ifdef DEBUG
            Serial.print(F("gps.alt: "));
            Serial.print(gps.altitude.meters());
            Serial.print(F(" - bmp280.alt: "));
            int tmpalt = bmp280();
            Serial.print(tmpalt);
            Serial.println(F(""));
            Serial.println(F("---------- alt diff set -------------------"));
          #endif
          }
      }
    }
    
    // collect data in array before writing to sd.
    // write to array every arrayWait
    if (millis() - arrayWriteTimer > arrayWait){
      arrayWriteTimer = millis();
      if (gps.satellites.value() >= 3){
        list[i] = buildLocationString();
        buildLocationData(i);
        
        #ifdef DEBUG
          for ( int p = 0;p <= 7; p++){
          Serial.print(gpsData[i][p]);      
          Serial.print("[");
          Serial.print(p);
          Serial.print("]");
          Serial.print(" ** ");
          }
          Serial.println(F(""));
        #endif

        counter++;

        #ifdef DEBUG
          Serial.print("parameter i: ");
          Serial.print(i);
          Serial.print(" - SD-card mounted = ");
          Serial.print(cardMounted);
          Serial.print(" - file_initilized = ");
          Serial.print(fileInit);
          Serial.print(" - display_screen = ");
          Serial.print(display_screen);
          Serial.print(" - button_pressed = ");
          Serial.println(button1.numberKeyPresses);

          Serial.println(list[i]);
        #endif

        if (i < (rows-1)){
          i++;
        }
        else{
          cardMounted = checkForSD_card(); // check for sd-card every rows ie 120 sec if list grows to big.
          i = 0;
        }
      }   
    }

    if (button1.pressed) {
        // Serial.printf("Button has been pressed %u times\n", button1.numberKeyPresses);
        // if display was dark, return to same display as befor dark
        if (!dimDisplay){
          if (display_screen < 4){
            display_screen++;
          }
          else{
            display_screen=1;
          }
        }
        button1.pressed = false;
        oledWriteTimer = 0; //no wait at screen print

        dimDisplayTimer = millis();
        display.ssd1306_command(SSD1306_DISPLAYON);
        dimDisplay = false;
    }
    
    if (millis() - dimDisplayTimer >120000){
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      dimDisplay = true;
    }

    // same timer as oled to update altitude data
    if ((millis() - oledWriteTimer > oledWait)){
    }
    // print to screen Oled
    #ifdef OLED
      // if ((millis() - oledWriteTimer > oledWait) and oledMounted){
      if ((millis() - oledWriteTimer > oledWait)){
        switch (display_screen){
          case 1:
              printToOled();
              break;
          case 2:
              printPaceToOled();
              break;
          case 3:
              printInfoToOled();
              break;
          case 4:
              printTimeToOled();
              break;
          default:
              printToOled();
              break;
        }

      oledWriteTimer = millis();
      }
    #endif


    // write to SD every sdWait
    if (cardMounted){
      if (!fileInit){
        initFileOnCard();
      }

      if (millis() - sdWriteTimer > sdWait){
        if (checkForSD_card()){
          sdWriteTimer = millis();
          appendArray(SD, fileName, list, i); 
          i = 0;

          blinkLED();

          #ifdef DEBUG
            readFile(SD, fileName);
            Serial.println(F(""));      
          #endif

      }
     }
    }

  }
}

//interupt button press
void IRAM_ATTR isr(){
  button_time = millis();
  if (button_time - last_button_time > 350){
    last_button_time = button_time;
    button1.numberKeyPresses++;
    button1.pressed = true;
  }
}

void initFileOnCard(){
    if (fileName == "/gps.txt"){
      if (millis() - fileNameTimer > 3000){
        Serial.println(F("init file now"));
        fileNameTimer = millis();
        if (gps.date.isValid()){
          setNewFileName();
          printSdType();
          writeHeader();
          readFile(SD, fileName);
          fileInit = true;
        }

        #ifdef DEBUG
          Serial.print(F("Filename is now: "));
          Serial.print(String(fileName));  // filename
          Serial.print(" ");
          Serial.print(sizeof(fileName));
          Serial.println(" *** ");
        #endif
     }
    }

}

void setNewFileName(){
  String dataString = "";
  if (gps.date.isValid()){
    dataString += "/gps_";
    dataString += String(gps.date.year());
    dataString += "-";
    if (gps.date.month() < 10) dataString += "0";
    dataString += String(gps.date.month());
    dataString += "-";
    if (gps.date.day() < 10) dataString += "0";
    dataString += String(gps.date.day());
    dataString += "_";
    if ((gps.time.hour() + hourOffset) < 10) dataString += "0";
    dataString += String(gps.time.hour() + hourOffset);
    // dataString += ":";
    if (gps.time.minute() < 10) dataString += "0";
    dataString += String(gps.time.minute());
    // dataString += ":";
    if (gps.time.second() < 10) dataString += "0";
    dataString += String(gps.time.second());
    
    dataString += ".txt";

  }

  // some trix to update the global filename
  dataString.toCharArray(str_array, dataString.length()+1);
  myfileName = strtok(str_array, " ");
  fileName = myfileName;  
}

int bmp280(){
  // auto temperature = String(sensor.getTemperature()) + " °C";
  // auto pressure = String(sensor.getPressure() / 100.0) + " hPa";
  // auto humidity = String(sensor.getHumidity()) + " %";

  // String measurements = temperature + ", " + pressure + ", " + humidity;
  // Serial.println(measurements);

  float pressure_a = sensor.getPressure();
  pressure_a /= 100;
  altitude = 44330 * (1.0 - pow(pressure_a / 1013, 0.1903));

  return altitude;
}

void printToOled(){
      // if ((millis() - oledWriteTimer > oledWait) and oledMounted){
      //   oledWriteTimer = millis();
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

          display.setCursor(65, 25);
          display.print("Alt: ");
          display.setCursor(95, 25); // horizontal , vertical
          display.print(altitude);

          display.setCursor(95, 35);
          display.print(int(gps.altitude.meters()));

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
          display.print(gps.speed.kmph(), 0);
          display.print(" km/t");
          // display.print(minPerKM(gps.speed.kmph() ));
          // display.print(" min/km");


          display.display();
    // }
}

void printPaceToOled(){
      // if ((millis() - oledWriteTimer > oledWait) and oledMounted){
      // if ((millis() - oledWriteTimer > oledWait)){
      //   oledWriteTimer = millis();
        #ifdef DEBUG
          Serial.println("--- Pace OLED ---");
        #endif

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(4);
        display.setCursor(15, 18); // horizontal , vertical
        display.print(minPerKM(gps.speed.kmph() ));
        display.display();
}

void printInfoToOled(){
        #ifdef DEBUG
          Serial.println("--- Info OLED ---");
        #endif

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(3);
        display.setCursor(25, 18); // horizontal , vertical
        display.print("i=");
        display.print(i);
        display.display();
}

void printTimeToOled(){
        #ifdef DEBUG
          Serial.println("--- Time OLED ---");
        #endif

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(3);
        display.setCursor(13, 10); // horizontal , vertical
        String dataString = "";
        if ((gps.time.hour() + hourOffset) < 10) dataString += "0";
        dataString += String(gps.time.hour() + hourOffset);
        dataString += ":";
        if (gps.time.minute() < 10) dataString += "0";
        dataString += String(gps.time.minute());
        display.print(dataString);

        dataString = "";
        display.setTextSize(2);
        display.setCursor(52, 40); // horizontal , vertical
        dataString += ":";
        if (gps.time.second() < 10) dataString += "0";
        dataString += String(gps.time.second());

        display.print(dataString);
        display.display();
}

String minPerKM(double kmph){
  String result = "-:--";
  // kmph +=6.66;
  if(kmph > 0){
    int seconds = 3600 / kmph;
    int m = seconds / 60;
    int s = seconds - m * 60;

    // 0 - 10 = 6.66/8.20 ->
    if (m <10){
      result = m;
      result += ":";
      if (s < 10) result += "0";
      result += s;
    }
  }
  return result;
}

bool checkForSD_card(){
  bool success = false;
  if (!SD.begin(H_CS,hSPI)) {
      Serial.println("Card Mount Failed");
      success = false;
      cardMounted = false;
  }
  else{
    digitalWrite(onBoardLed, HIGH);
    success = true;
  }
  return success;
}

void writeHeader(){
  
  // char *header[] = {"Nr","Time","Lat","Lon","Sats","Distance","FromHome","Alt"};
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      Serial.print("writing to SD");
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
      dataFile.print(",");
      // altitude
      dataFile.print(header[7]);
      dataFile.println("");
      Serial.println(" -- done");  
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
  // dataFile = SD.open(fileName, FILE_WRITE);
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
  //{"Nr","Time","Lat","Lon","Sats","Distance","FromHome","Altitude"};
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
  if ((gps.time.hour() + hourOffset) < 10) dataString += "0";
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

  totalDist = totalDist + dist; // totalDist is defined as type long, so it only adds if dist is greater than one. Acts as a filter against small jitter movement.
  // Serial.println(totalDist);
  dataString += String(totalDist);
  lastDistLAT = lat;
  lastDistLON = lon;

  // dataString += 0;
  dataString += ","; //FromHome
  dataString += displayHome();

  dataString += ","; //Altitude
  int tmp = bmp280();
  dataString += tmp;
  // dataString +=String(bmp280(), 2);
  dataString += "\n";
  // Serial.println(dataString);
  return dataString;
}

void buildLocationData(int s){
  //{"Nr","Time","Lat","Lon","Sats","Distance","FromHome","Alt"};
  String dataString = "";
  int k = 0;
  // Counter - 0
  gpsData[s][k]=counter;
  k++; 
  // Date Time - 1 YYYY-MM-DD HH:MM:SS.ms
  dataString += String(gps.date.year());
  dataString += "-";
  if (gps.date.month() < 10) dataString += "0";
  dataString += String(gps.date.month());
  dataString += "-";
  if (gps.date.day() < 10) dataString += "0";
  dataString += String(gps.date.day());
  dataString += " ";
  if ((gps.time.hour() + hourOffset) < 10) dataString += "0";
  dataString += String(gps.time.hour() + hourOffset);
  dataString += ":";
  if (gps.time.minute() < 10) dataString += "0";
  dataString += String(gps.time.minute());
  dataString += ":";
  if (gps.time.second() < 10) dataString += "0";
  dataString += String(gps.time.second());
  dataString += ".";
  dataString += String(gps.time.centisecond());
  gpsData[s][k]=dataString;
  k++; 
  // Latitude - 2
  gpsData[s][k]=String(gps.location.lat(), 8);
  k++; 
  // Longitude - 3
  gpsData[s][k]=String(gps.location.lng(), 8);
  k++; 
  // Sats - 4
  gpsData[s][k]=gps.satellites.value();
  k++; 
  //Distance - 5
  double lat = gps.location.lat();
  dataString += String(lat, 6); 
  dataString += ","; //Lon
  // dataString += String(gps.location.lng(), 6);
  double lon =  gps.location.lng();
  dataString += String(lon, 6);
  dataString += ","; //Sats
  dataString += String(gps.satellites.value());
  dataString += ","; //Distance
  float dist = TinyGPSPlus::distanceBetween(lat,lon,lastDistLAT,lastDistLON);
  // Serial.println(TinyGPSPlus::distanceBetween(lat,lon,lastDistLAT,lastDistLON));
  // Serial.println(dist);
  totalDist = totalDist + dist; // totalDist is defined as type long, so it only adds if dist is greater than one. Acts as a filter against small jitter movement.
  // Serial.println(totalDist);
  // dataString += String(totalDist);
  lastDistLAT = lat;
  lastDistLON = lon;
  gpsData[s][k]=String(totalDist);
  k++; 
  // FromHome - 6
  gpsData[s][k]=displayHome();
  k++; 
  // Altitude - 7
  // set altitude to -1 until the gps-altitude i settled and a difference can be calculated
  if (!setAltDiffOnce){
    gpsData[s][k]=(bmp280() + altDiff);
  }
  else{
    gpsData[s][k]=-1;
  }
  
}

void printSdType(){
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println(F("No SD card attached"));
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println(F("MMC"));
  } else if(cardType == CARD_SD){
      Serial.println(F("SDSC"));
  } else if(cardType == CARD_SDHC){
      Serial.println(F("SDHC"));
  } else {
      Serial.println(F("UNKNOWN"));
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println(F("Failed to open file for reading"));
        return;
    }

    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    digitalWrite(onBoardLed, HIGH);
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println(F("File written"));
    } else {
        Serial.println(F("Write failed"));
    }
    file.close();
    digitalWrite(onBoardLed, LOW);
}


void appendArray(fs::FS &fs, const char * path, String * theList, int sizeOfIt){
    Serial.printf("Appending the list to file: %s\n", path);
    digitalWrite(onBoardLed, HIGH);
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println(F("Failed to open file for appending"));
        checkForSD_card();
        return;
    }
    for (int k = 0; k < sizeOfIt; k++){
      if(!file.print(theList[k])){
        Serial.print(F("Append of the list failed - "));
        Serial.println(theList[k]);
      }
    }
    file.close();
    digitalWrite(onBoardLed, LOW);
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);
    digitalWrite(onBoardLed, HIGH);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println(F("Failed to open file for appending"));
        return;
    }
    if(file.print(message)){
        Serial.println(F("Message appended"));
    } else {
        Serial.println(F("Append failed"));
    }
    file.close();
    digitalWrite(onBoardLed, LOW);
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

  Serial.println(F(""));
}

