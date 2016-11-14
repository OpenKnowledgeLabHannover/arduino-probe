
#include "Adafruit_FONA.h"

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_FONA.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

unsigned int Pm25 = 0;
unsigned int Pm10 = 0;
uint16_t vbat = 0;

// this is a large buffer for replies
char replybuffer[255];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
// HardwareSerial *PMSensorSerial = &Serial1;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

//Adafruit_MQTT_FONA mqtt(&fona, "35.156.19.41", 1883);
Adafruit_MQTT_FONA mqtt(&fona, "io.adafruit.com", 1883, "npohle", "------<HIDDEN>---------");
Adafruit_MQTT_Publish pm10feed = Adafruit_MQTT_Publish(&mqtt, "npohle/feeds/npohle-pm10");
Adafruit_MQTT_Publish pm25feed = Adafruit_MQTT_Publish(&mqtt, "npohle/feeds/npohle-pm25");
Adafruit_MQTT_Publish batfeed = Adafruit_MQTT_Publish(&mqtt, "npohle/feeds/npohle-battery");

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;

Adafruit_SSD1306 display = Adafruit_SSD1306();

void setup() {
  //while (!Serial);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  display.println("FONA basic test");
  display.println("Initializing....(May take 3 secs)");
  display.display();
  
  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));
  
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    display.println("Couldn't find FONA");
    display.display();
    while (1);
  }
  
  // Optionally configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.
  //fona.setGPRSNetworkSettings(F("your APN"), F("your username"), F("your password"));
  fona.setGPRSNetworkSettings(F("em"), F(""), F(""));

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

  //set local time AT+CLTS=1
  fona.enableNetworkTimeSync(true);
  
  Serial1.begin(9600,SERIAL_8N1);
  Pm25=0;
  Pm10=0;
  vbat=0;
}

void loop() {
  
  sample();
  //delay(600000); //10min
  delay(20000); //20sec
    
}

void sample() {

  Pm25=0;
  Pm10=0;
  vbat=0;
  
  uint16_t statuscode;
  int16_t length;
  char url[80];
  
  uint8_t mData = 0;
  uint8_t i = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;

  int8_t ret;

  String SendStatus = "Sent: ";
  char time[23];
  String StrTime = "";
  
   Serial.println(F("Read Battery"));

   if (fona.getBattVoltage(&vbat)) {
      if (fona.getBattPercent(&vbat)) {
        Serial.println(vbat);
      } else {
        Serial.println(F("Error reading Battery Percentage!"));
      }
    } 
    else {
      Serial.println(F("Error reading Battery Voltage!"));
    }

    Serial.println(F("Read PM Sensor"));
    if (Serial1.available() > 0) 
    {  
      Serial.println(F("Serial available"));
      // from www.inovafitness.com
      // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB
       mData = Serial1.read();     delay(2);//wait until packet is received
      if(mData == 0xAA)//head1 ok
       {
          Serial.println(F("Serial PM Sensor: Start reading header"));
          mPkt[0] =  mData;
          mData = Serial1.read();
          if(mData == 0xc0)//head2 ok
          {
            Serial.println(F("Serial PM Sensor: Start reading data"));
            mPkt[1] =  mData;
            mCheck = 0;
            for(i=0;i < 6;i++)//data recv and crc calc
            {
               mPkt[i+2] = Serial1.read();
               delay(2);
               mCheck += mPkt[i+2];
            }
            mPkt[8] = Serial1.read();
            delay(1);
            mPkt[9] = Serial1.read();
            Serial.println(F("Serial PM Sensor: Finished reading data"));
            if(mCheck == mPkt[8])//crc ok
            {
              Serial.println(F("Serial PM Sensor: CRC OK"));
              Serial1.flush();
              Serial.write(mPkt,10);
  
              Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3]<<8);
              Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5]<<8);
              if(Pm25 > 9999)
               Pm25 = 9999;
              if(Pm10 > 9999)
               Pm10 = 9999;            
              //get one good packet
              Serial.println(Pm25);
              Serial.println(Pm10);
              Serial.println(F("-------"));

              //sprintf(url, "http://dweet.io/dweet/quietly/for/feinstaub-alpha?pm10=%d&pm25=%d", Pm10, Pm25);

            } else {
              Serial.println(F("Serial PM Sensor: CRC NOT OK"));
            }
          } else {
            Serial.println(F("Serial PM Sensor: Header2 NOT OK"));
          }      
       } else {
        Serial.println(F("Serial PM Sensor: Header1 NOT OK"));
       }
    } else {
      Serial.println("Serial1 not available!");
    }

    // Sending
    
    if (!fona.enableGPRS(true)) {
      Serial.println(F("Failed to turn on GPRS"));
    }
    
    fona.getTime(time, 23);  // make sure replybuffer is at least 23 bytes!
    StrTime = String(time);
    StrTime.replace("\"", "");
    
    Serial.print(F("Network Time: "));
    Serial.println(StrTime);
    
    //Serial.print("Connecting to MQTT... ");
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      Serial.println(mqtt.connectErrorString(ret));
      Serial.println("Retrying MQTT connection in 5 seconds...");
      mqtt.disconnect();
      
      if (!fona.enableGPRS(false)) {
        Serial.println(F("Failed to turn off GPRS"));
      }                
      
      delay(5000);  // wait 5 seconds
      
      if (!fona.enableGPRS(true)) {
        Serial.println(F("Failed to turn on GPRS"));
      }
      
    }
     Serial.println("MQTT Connected!");

    if (! pm10feed.publish((int32_t)Pm10)) {
      Serial.println(F("Failed to publish PM10"));
    } else {
      Serial.println(F("Published PM10!"));
      SendStatus += "PM10 ";
    }
    
    if (! pm25feed.publish((int32_t)Pm25)) {
      Serial.println(F("Failed to publish PM25"));
    } else {
      Serial.println(F("Published PM25!"));
      SendStatus += "PM25 ";
    }
    
    if (! batfeed.publish((int32_t)vbat)) {
      Serial.println(F("Failed to publish Battery"));
    } else {
      Serial.println(F("Published Battery!"));
      SendStatus += "BAT ";
    }
                 
    mqtt.disconnect();
    if (!fona.enableGPRS(false)) {
      Serial.println(F("Failed to turn off GPRS"));
    }
   
   display.clearDisplay();
   display.setCursor(0,0);

   display.println(StrTime);
   display.print("PM10="); display.print(Pm10); display.print(" PM25="); display.println(Pm25);
   display.println(SendStatus);
   display.print("Battery: "); display.print(vbat); display.println("%"); 
   
   display.display();
}


