
#include <Wire.h>

#include <DallasTemperature.h>
#include <OneWire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);


// Data wire is plugged into pin 10 on the Adafruit
#define ONE_WIRE_BUS 10


// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

unsigned int Pm25 = 0;
unsigned int Pm10 = 0;
uint16_t vbat = 0;
unsigned int gas = 0;
unsigned int spl = 0;
int particleerror;

// this is a large buffer for replies
char replybuffer[255];

#include <SoftwareSerial.h>
// Hardware serial is also possible!
HardwareSerial *PMSensorSerial = &Serial1;

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;

// Adafruit_SSD1306 display = Adafruit_SSD1306();

void setup() {

 // //while (!Serial);
 // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
 // display.display();
 // //delay(2000);
 // display.clearDisplay();
 // display.setTextSize(1);
 // display.setTextColor(WHITE);
 // display.setCursor(0, 0);


 // display.print("Battery: "); display.print(vbat); display.println("%");
 // display.display();

  Serial1.begin(9600, SERIAL_8N1);

  Pm25 = 0;
  Pm10 = 0;
  vbat = 0;
  gas = 0;
  spl = 0;
  particleerror = 0;

  pinMode(A0, INPUT); // gas
  pinMode(A1, INPUT); // spl



  //-------------------------------------------------
   Serial.println("Light Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  /* Display some basic information on this sensor */
  // displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();

  /* We're ready to go! */
  Serial.println("");
}

void loop() {
Serial.println("Starting Sensor Read...");
  sample(5000);

   /* Get a new sensor event */
  sensors_event_t event;
  tsl.getEvent(&event);

  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    Serial.print(event.light); Serial.println(" lux");
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }
  delay(250);
  //sample(600000); //10min
  //sample(300000); //5min
  //sample(20000); //20sec

}

void sample(int32_t window) {

  Pm25 = 0;
  Pm10 = 0;
  vbat = 0;
  gas = 0;
  spl = 0;

  Serial.print("Reading Gas...");
  gas = analogRead(A0);
  spl = getSPL(window);
  particleerror = getParticles(&Pm10, &Pm25);
  Serial.print("Done Reading Gas...");

  Serial.print("{\"gas\":");
  Serial.print(gas);
  Serial.print(",\"sp1\":");
  Serial.print(spl);
  Serial.print(",\"pm10\":");
  Serial.print(Pm10);
  Serial.print(",\"pm25\":"),
  Serial.print(Pm25);
  Serial.println("}");

  uint16_t statuscode;
  int16_t length;
  char url[80];

  Serial.println(F("Read PM Sensor"));


  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print(" Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  Serial.print("Temperature for Device 1 is: ");
  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
    // You can have more than one IC on the same bus.
    // 0 refers to the first IC on the wire

//  display.clearDisplay();
//  display.setCursor(0, 0);
//
//  //display.println(StrTime);
//  display.print("P1="); display.print(Pm10); display.print(" P2="); display.print(Pm25); display.print(" G="); display.print(gas); display.print(" S="); display.println(spl);
//  display.println(SendStatus);
//  display.print("Battery: "); display.print(vbat); display.println("%");
//
//  display.display();
}

uint16_t getSPL(int32_t sampleWindow) {

   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;
   unsigned int sample = 0;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(A1);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   return peakToPeak;

}

uint8_t getParticles (unsigned int *Pm25, unsigned int *Pm10) {

  const uint16_t sampleWindow = 10000;
  unsigned long startMillis= millis();

  uint8_t ret = 1; //default: timeout

  while (millis() - startMillis < sampleWindow) {

    uint8_t mData = 0;
    uint8_t i = 0;
    uint8_t mPkt[10] = {0};
    uint8_t mCheck = 0;

    //Serial.print("t");
    if (Serial1.available() > 0)
    {
      Serial.println(F("Serial available"));
      // from www.inovafitness.com
      // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB
      mData = Serial1.read();     delay(2);//wait until packet is received
      if (mData == 0xAA) //head1 ok
      {
        //Serial.println(F("Serial PM Sensor: Start reading header"));
        mPkt[0] =  mData;
        mData = Serial1.read(); delay(2);
        if (mData == 0xc0) //head2 ok
        {
          //Serial.println(F("Serial PM Sensor: Start reading data"));
          mPkt[1] =  mData;
          mCheck = 0;
          for (i = 0; i < 6; i++) //data recv and crc calc
          {
            mPkt[i + 2] = Serial1.read();
            delay(2);
            mCheck += mPkt[i + 2];
          }
          mPkt[8] = Serial1.read();delay(2);
          mPkt[9] = Serial1.read();delay(2);
          mPkt[10] = Serial1.read();delay(2);
          Serial.print("MCheck: ");
          Serial.print(mCheck, HEX);
          Serial.print(" Package: ");
          for (int k = 0; k < 10; k++) {
            Serial.print(mPkt[k], HEX);
            Serial.print(" ");
          }
          Serial.println("");
          if (mCheck == mPkt[8]) //crc ok
          {
            //Serial.println(F("Serial PM Sensor: CRC OK"));
            //Serial1.flush();
            //Serial.write(mPkt, 10);

            *Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3] << 8);
            *Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5] << 8);
            if (*Pm25 > 9999)
              *Pm25 = 9999;
            if (*Pm10 > 9999)
              *Pm10 = 9999;

            return 0; // no error

          } else {
            Serial.println(F("Serial PM Sensor: CRC NOT OK"));
            ret = 3;
          }
        }
      }
    }
  }

  return ret;
}



// /**************************************************************************/
// /*
//     Displays some basic information on this sensor from the unified
//     sensor API sensor_t type (see Adafruit_Sensor for more information)
// */
// /**************************************************************************/
// void displaySensorDetails(void)
// {
//   sensor_t sensor;
//   tsl.getSensor(&sensor);
//   Serial.println("------------------------------------");
//   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
//   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
//   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");
//   Serial.println("------------------------------------");
//   Serial.println("");
//   delay(500);
// }

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
