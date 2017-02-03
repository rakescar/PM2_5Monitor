#include <AltSoftSerial.h>
//#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
//#include <TimeLib.h>
//#include <uartWIFI.h>



//软串口，Uno开发板：Tx-D9、Rx-D8。Rx接传感器的Tx。
AltSoftSerial altSerial;
//SoftwareSerial altSerial;

//I2C 1602液晶屏
LiquidCrystal_I2C lcd(0x20, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//攀藤G5的数据格式
struct _panteng {
  unsigned char len[2];
  unsigned char pm1_cf1[2];
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


void setup()
{

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH); //seems the digitial pins are default to LOW, need to initialize to HIGH to avoid constant reset.

  lcd.init();                      // initialize the lcd
  //lcd.backlight();
  lcd.noBacklight();

  Serial.begin(115200);        //USB串口向PC发送数据
  altSerial.begin(9600);        //软串口连接传感器

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

          
          //TODO: Implement a low pass filter to smooth out the data a bit
//          if (data_count == 0) {
//            pm2_5_avg = pm2_5;
//          }
//          else {
//            pm2_5_avg = pm2_5_avg * (data_count * 1.0 / (data_count + 1)) + pm2_5 * 1.0 / (data_count + 1);
//
//          }
//          
//          if (data_count < 100) {
//            data_count++;
//          }

          checksum = calculateChecksum();

          debug();

          if (checksum == frame_checksum ) {

            sendData(DEVICEID0, SENSORID0, pm2_5);

            delay(5000);

            sendData(DEVICEID0, SENSORID1, hcho * 0.001);

            delay(5000);
          }

        }
        break;
        
      default:
        break;
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

  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  String json = "{";
  json += "\"value\":" + String(thisData);
  json += "}";

  String cmd;
  cmd = "POST /v1.0/device/";
  cmd += String(device_id);
  cmd += "/sensor/";
  cmd += String(sensor_id);
  cmd += "/datapoints";
  cmd += " HTTP/1.1\r\n";
  cmd += "Host: api.yeelink.net\r\n";
  //        cmd += "Accept: */*\r\n";
  cmd += "U-ApiKey: ";
  cmd += APIKEY;
  cmd += "\r\n";
  cmd += "Connection: close\r\n";
  //        cmd += "Content-Type: application/x-www-form-urlencoded\r\n";
  //        cmd += "\r\n";
  cmd += "Content-Length: " + String(json.length());
  cmd += "\r\n\r\n";
  cmd += json;
  cmd += "\r\n";
  cmd += "\r\n";

  Serial.print(cmd);
  Serial.flush();


  delay(2000);
  boolean result = readResponse();

  Serial.end();

  // note the time that the connection was made, regardless of whether the upload was successful
  lastConnectionTime = millis();
  if (result ) {
    lastUploadTime = lastConnectionTime; //record last successful upload time
  }
  else {
    //reset wifi if there has been no sucessfull upload in a certain period
    if ((lastConnectionTime-lastUploadTime) > 60000 ) {
      resetWifi();
      lastUploadTime += 60000; //avoid resetting wifi again in the next 60 seconds
    }
  }
}

void resetWifi() {
  lcd.setCursor(0, 1);
  lcd.print("Resetting Wifi...");
  digitalWrite(RESET_PIN, LOW);    // Reset ESP8266
  delay(50);                       // wait for a short period
  digitalWrite(RESET_PIN, HIGH);
  delay(10000);                    // wait for wifi reconnect
  lcd.setCursor(0, 1);
  lcd.print("Wifi reconnected...");
}

boolean readResponse() {
  boolean result = false;
  String s;
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    // add it to the inputString unless end of line is reached.

    if (inChar == '\n' || inChar == '\r') {
      break;
    }

    s += inChar;

  }

  if (s == "HTTP/1.1 200 OK") {
    result = true;
    
    upload_success_count++;
    lcd.setCursor(0, 0);
    lcd.print("Upload: " + String(upload_success_count) + "/" + String(upload_count));

  } else {
    result = false;
    
    lcd.setCursor(0, 0);
    lcd.print("Upload: " + String(upload_success_count) + "/" + String(upload_count));
    lcd.setCursor(0, 1);
    lcd.print(s);

    delay(5000);
  }

  Serial.flush();

  //clear the remaining response content
  while (Serial.available()) {
    Serial.read();
  }

  return result;
}
