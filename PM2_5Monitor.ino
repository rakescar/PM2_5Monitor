#include <AltSoftSerial.h>
//#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
//#include <TimeLib.h>
//#include <uartWIFI.h>



//软串口，Uno开发板：Tx-D9、Rx-D8。Rx接攀藤G5S传感器的Tx。
AltSoftSerial altSerial;

//I2C 1602液晶屏
LiquidCrystal_I2C lcd(0x20, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//攀藤G5S的数据格式
struct _panteng {
  unsigned char len[2];
  unsigned char pm1_0_cf1[2];
  unsigned char pm2_5_cf1[2];
  unsigned char pm10_0_cf1[2];
  unsigned char pm1_0[2];
  unsigned char pm2_5[2];
  unsigned char pm10_0[2];
  unsigned char pm0_3_count[2];
  unsigned char pm0_5_count[2];
  unsigned char pm1_0_count[2];
  unsigned char pm2_5_count[2];
  unsigned char pm5_count[2];
  unsigned char pm10_count[2];
  unsigned char hcho[2];
  unsigned char checksum[2];
} panteng;


// API Key for yeelink
#define APIKEY         "72f18469a82f9396526051ab7ab3e08c" // PLEASE replace your yeelink api key here!!!

//replace the device ID and sensor ID for temperature sensor.
#define DEVICEID0       354276 // replace your device ID
#define SENSORID0       400070 // replace your sensor ID

//replace the device ID and sensor ID for humidity sensor.
#define DEVICEID1       354276 // replace your device ID
#define SENSORID1       400071 // replace your sensor ID

#define FILTER_SIZE 6  // size of filter buffer for previous reading values

int pm2_5_values[FILTER_SIZE] = {0};
int hcho_values[FILTER_SIZE] = {0};
int filter_index =0;
int coe[FILTER_SIZE];
int coe_total;

#define RESET_PIN 13    //ESP8266 reset pin is connected to Arduino pin 13 using a 1K resistor

const char server[] = "api.yeelink.net";   // name address for yeelink API

unsigned long lastConnectionTime = 0;          // last time you connected to the server, in milliseconds
unsigned long lastUploadTime =0;               // last time of successful upload


int frame_count = 0;   // number of data frames received from G5S Sensor
int upload_count = 0;  // number of uploads to the server
int upload_success_count = 0; // number of successful uploads
//float pm2_5_avg = 0;
//float hcho_avg = 0;
//int data_count = 0;
int pm1_0, pm2_5, pm10_0, hcho, frame_length, frame_checksum;        //PM1.0、PM2.5、PM10
int checksum, error_count = 0;
int reset_count =0;

//timestamp for last upload time of PM2.5 & HCHO data
unsigned long pm2_5_time=0, hcho_time=0;

//these are filtered values
float pm2_5_value=0, hcho_value=0;

void setup()
{

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH); //seems the digital pins are default to LOW, need to initialize to HIGH to avoid constant reset.

  //initialize coefficient array of weight ratio.
  for (int i=0; i<FILTER_SIZE; i++) {
    coe[i]=FILTER_SIZE-i;
    coe_total += coe[i];
  }

  lcd.init();                      // initialize the lcd
  //lcd.backlight();
  lcd.noBacklight();

  Serial.begin(115200);        //Hardware Serial to ESP8266
  altSerial.begin(9600);        //Soft Serial on Pin 8 to G5S Sensor

  lcd.setCursor(0, 0);
  lcd.print("ready...");

  randomSeed(analogRead(0));
}


void loop()
{
  unsigned char c;


  static int state = 0;
  static int count = 0;
  static int time = 0;

  if (altSerial.available()) {
    c = altSerial.read();
    switch (state) {
      case 0:
      if (0x42 == c)
        state = 1;
      count = 0;
      break;
      case 1:
      if (0x4d == c) {
        state = 2;
        count = 0;
      }
      break;
      case 2:
      ((unsigned char *) &panteng)[count++] = c;
      if (count > 30) {
        state = 0;
        pm1_0 = panteng.pm1_0[0] * 256 + panteng.pm1_0[1];
        pm2_5 = panteng.pm2_5[0] * 256 + panteng.pm2_5[1];
        pm10_0 = panteng.pm10_0[0] * 256 + panteng.pm10_0[1];
        hcho = panteng.hcho[0] * 256 + panteng.hcho[1];
        frame_length = panteng.len[0] * 256 + panteng.len[1];
        frame_checksum = panteng.checksum[0] * 256 + panteng.checksum[1];

        char pm2_5_str[20];
        snprintf(pm2_5_str, 16, "PM2.5/10=%d/%d   ", pm2_5, pm10_0);
        char pm1_0_str[20];
        snprintf(pm1_0_str, 16, "PM1.0/10=%d/%d   ", pm1_0, pm10_0);
        char hcho_f[10];
        char hcho_str[20];
        sprintf(hcho_str, "HCHO:%s  ", dtostrf(hcho * 0.001, 1, 3, hcho_f));

        frame_count++;

        checksum = calculateChecksum();

        debug();

        if (checksum == frame_checksum ) {

          filter_index= (filter_index+1) % FILTER_SIZE;

          pm2_5_value = dataFilter(pm2_5_values, filter_index, pm2_5);
          hcho_value = dataFilter(hcho_values, filter_index, hcho);

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("PM2.5: " + String(pm2_5) + "/" + String(pm2_5_value));
  //        lcd.setCursor(0, 1);
  //        lcd.print("HCHO: " + String(hcho) + "/" + String(hcho_value));
          lcd.setCursor(0, 1);
          lcd.print(String(reset_count)+" Wifi reset");
        }

      }
      break;

      default:
      break;
    }
  }

  if ( pm2_5_time <= hcho_time ) {
    if (timerCheck(&pm2_5_time, 5000)) {
      sendData(DEVICEID0, SENSORID0, pm2_5_value);
    }
  }
  else {
    if (timerCheck(&pm2_5_time, 5000)) {
      sendData(DEVICEID0, SENSORID1, hcho_value * 0.001);
      hcho_time = pm2_5_time;
    }
  }
}

int calculateChecksum() {
  int checksum = 143; //0x42+0x4d = 143
  for (int i = 0; i < 28; i++) {
  checksum += ((unsigned char *) &panteng)[i];
  }
  return checksum;
}

// This method will filter the data to:
//      1. avoid random spike of high value.
//      2. avoid oscillation between adjacent value points
//      3. allow tuning between smooth output and fast response
//      4. minimize memory usage
float dataFilter( int dataArray[], int headIndex, int reading) {

  //    TO CONSIDER:
  //    cap the increase/decrease change to 10 points per second, which is 10 ug/m3 for PM2.5 and 0.01 mg/m3 for HCHO
  //    need to be careful with this time based calculation as the time between two readings could be less than a second
  //    create a branch and use Serial to debug first

  float result = 0;

  dataArray[headIndex]= reading;

  int index;
  for (int i=0; i<FILTER_SIZE; i++) {
    index = headIndex - i;

    //wraps around when reaching the first element of the array
    if (index <0) index += FILTER_SIZE;

    result += (dataArray[index]*coe[i]) *1.00/ coe_total; // TODO: remove 1.00 and test
  }

  return result;
}

boolean timerCheck(unsigned long *lastMillis, unsigned int delay )
{
 unsigned long currentMillis = millis();
 if(currentMillis - *lastMillis >= delay)
 {
   *lastMillis = currentMillis;
   return true;
 }
 else
   return false;
}

void debug () {

  if (frame_count % 100 == 0) {
  Serial.println("\n>>>>>>>>>>>>>> Frame Count: " + String(frame_count) + "Error Count: " + String(error_count) + "<<<<<<<<<<<<<<<<");
  }

  if (checksum != frame_checksum ) {
  error_count++;
  Serial.println("\n>>>>>> OH NO !!!  " + String(error_count) + "<<<<<<<<<<<");

  Serial.print("PM2.5: ");
  Serial.println(pm2_5);
  Serial.print("HCHO: ");
  Serial.println(hcho * 0.001);

  Serial.println("Frame Length: " + String(frame_length));
  Serial.println("Frame Checksum: " + String(frame_checksum) + " Calculated Checksum: " + String(checksum));

  }
}

// this method makes a HTTP connection to the server:
void sendData(long device_id, long sensor_id, float thisData) {

  upload_count++;

//  Serial.begin(115200);

  while (!Serial) {
  ; // wait for serial port to connect. Needed for Leonardo only. TODO: remove this block and test
  }

      char data_str[10];

      dtostrf(thisData, 1, 3, data_str);

  String json = "{";
  json += "\"value\":" + String(data_str);
  json += "}";

  String cmd; // The HTTP request body to be sent out
  cmd = "POST /v1.0/device/";
  cmd += String(device_id);
  cmd += "/sensor/";
  cmd += String(sensor_id);
  cmd += "/datapoints";
  cmd += " HTTP/1.1\r\n";
  cmd += "Host: api.yeelink.net\r\n"; //this is mandatory for HTTP POST request
  //        cmd += "Accept: */*\r\n"; //this seems to be optional
  cmd += "U-ApiKey: ";
  cmd += APIKEY;
  cmd += "\r\n";
  cmd += "Connection: close\r\n";
  //        cmd += "Content-Type: application/x-www-form-urlencoded\r\n"; //this seems to be optional
  cmd += "Content-Length: " + String(json.length());
  cmd += "\r\n\r\n";
  cmd += json;
  cmd += "\r\n";
  cmd += "\r\n";

  Serial.print(cmd); //send out the request thru Serial port

  Serial.flush(); //not sure whether this is needed or not. TODO: remove this line and test

  delay(2000); //delay 2 seconds and wait for response from server
  boolean result = readResponse(); //read the response from server and see whether upload is successful

//  Serial.end(); //not sure whether this is needed or not. TODO: remove this line and test

  // record the time that the connection was made
  lastConnectionTime = millis();
  if ( result ) {
  //if result is successful record the time of last successful upload time
  lastUploadTime = lastConnectionTime;
  }
  else {
  //if the result is not successful, check whether need to reset wifi
  //reset wifi if there has been no successful upload in a certain period
  if ((lastConnectionTime-lastUploadTime) > 60000 ) {
    resetWifi();
    lastUploadTime += 60000; //avoid resetting wifi again in the next 60 seconds
  }
  }
}

void resetWifi() {
  reset_count++;
  lcd.setCursor(0, 1);
  lcd.print("Resetting Wifi...");
  digitalWrite(RESET_PIN, LOW);    // Reset ESP8266
  delay(50);                       // wait for a short period
  digitalWrite(RESET_PIN, HIGH);
  delay(10000);                    // wait for wifi reconnect
  lcd.setCursor(0, 1);
  lcd.print(String(reset_count)+" Wifi reset");
}

boolean readResponse() {
  boolean result = false;

  String s; //response string, which is the first line of response that contains the HTTP response code.
  while (Serial.available()) {

  // read one char each time
  // add it to the response string unless end of line is reached.
  char inChar = (char)Serial.read();
  if (inChar == '\n' || inChar == '\r') {
    break;
  }

  s += inChar;

  }

  //if the response code is 200, upload was successful;
  //assume everything else means failure
  if (s == "HTTP/1.1 200 OK") {
  result = true;

  upload_success_count++;
//  lcd.setCursor(0, 0);
//  lcd.print("Upload: " + String(upload_success_count) + "/" + String(upload_count));

  } else {

    result = false;

//    lcd.setCursor(0, 0);
//    lcd.print("Upload: " + String(upload_success_count) + "/" + String(upload_count));
//    lcd.setCursor(0, 1);
//    lcd.print(s);


  }

  Serial.flush();

  //clear the remaining response content
  while (Serial.available()) {
  Serial.read();
  }

  return result;
}
