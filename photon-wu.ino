// This #include statement was automatically added by the Particle IDE.
#include <SparkFun_Photon_Weather_Shield_Library.h>

/******************************************************************************
  SparkFun_Photon_Weather_Wunderground.ino
  SparkFun Photon Weather Shield basic example
  Joel Bartlett @ SparkFun Electronics
  Original Creation Date: May 18, 2015
  Updated August 21, 2015
  This sketch prints the temperature, humidity, and barometric pressure OR
  altitude to the Serial port.
  The library used in this example can be found here:
  https://github.com/sparkfun/SparkFun_Photon_Weather_Shield_Particle_Library
  Hardware Connections:
	This sketch was written specifically for the Photon Weather Shield,
	which connects the HTU21D and MPL3115A2 to the I2C bus by default.
  If you have an HTU21D and/or an MPL3115A2 breakout,	use the following
  hardware setup:
      HTU21D ------------- Photon
      (-) ------------------- GND
      (+) ------------------- 3.3V (VCC)
       CL ------------------- D1/SCL
       DA ------------------- D0/SDA
    MPL3115A2 ------------- Photon
      GND ------------------- GND
      VCC ------------------- 3.3V (VCC)
      SCL ------------------ D1/SCL
      SDA ------------------ D0/SDA
  Development environment specifics:
  	IDE: Particle Dev
  	Hardware Platform: Particle Photon
                       Particle Core
  This code is beerware; if you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!
  Distributed as-is; no warranty is given.
//---------------------------------------------------------------
  Weather Underground Upload sections: Dan Fein @ Weather Underground
  Weather Underground Upload Protocol:
  http://wiki.wunderground.com/index.php/PWS_-_Upload_Protocol
  Sign up at http://www.wunderground.com/personal-weather-station/signup.asp
*******************************************************************************/
#include "SparkFun_Photon_Weather_Shield_Library.h"
#include "math.h"   //For Dew Point Calculation
int WDIR = A0;
int RAIN = D2;
int WSPEED = D3;
//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120]; //120 bytes to keep track of 2 minute average
int winddiravg[120]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
long lastWindCheck = 0;
volatile float dailyrainin = 0; // [rain inches so far today in local time]

float humidity = 0;
float humTempF = 0;  //humidity sensor temp reading, fahrenheit
float humTempC = 0;  //humidity sensor temp reading, celsius
float baroTempF = 0; //barometer sensor temp reading, fahrenheit
float baroTempC = 0; //barometer sensor temp reading, celsius
float tempF = 0;     //Average of the sensors temperature readings, fahrenheit
float tempC = 0;     //Average of the sensors temperature readings, celsius
float dewptF = 0;
float dewptC = 0;
float pascals = 0;
float inches = 0;
int count = 0;

// volatiles are subject to modification by IRQs
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile unsigned long raintime, rainlast, raininterval, rain;

//Wunderground Vars

char SERVER[] = "rtupdate.wunderground.com";      	    //Rapidfire update server - for multiple sends per minute
//char SERVER [] = "weatherstation.wunderground.com";   //Standard server - for sends once per minute or less
char WEBPAGE [] = "GET /weatherstation/updateweatherstation.php?";

//Station Identification
char ID [] = "KNMOHKAY2"; //Your station ID here
char PASSWORD [] = "Chickens1"; //your Weather Underground password here

TCPClient client;

const unsigned long IDLE_TIMEOUT_MS = 750; // Time to listen for the WunderGround response

//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barometric sensor
Weather sensor;
//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}


//---------------------------------------------------------------
void setup()
{
    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor
    
    Serial.begin(9600);   // open serial over USB at 9600 baud

    //Initialize the I2C sensors and ping them
    sensor.begin();

    /*You can only receive acurate barrometric readings or acurate altitiude
    readings at a given time, not both at the same time. The following two lines
    tell the sensor what mode to use. You could easily write a function that
    takes a reading in one made and then switches to the other mode to grab that
    reading, resulting in data that contains both acurate altitude and barrometric
    readings. For this example, we will only be using the barometer mode. Be sure
    to only uncomment one line at a time. */
    sensor.setModeBarometer();//Set to Barometer Mode
    //baro.setModeAltimeter();//Set to altimeter Mode

    //These are additional MPL3115A2 functions the MUST be called for the sensor to work.
    sensor.setOversampleRate(7); // Set Oversample rate
    //Call with a rate from 0 to 7. See page 33 for table of ratios.
    //Sets the over sample rate. Datasheet calls for 128 but you can set it
    //from 1 to 128 samples. The higher the oversample rate the greater
    //the time between data samples.


    sensor.enableEventFlags(); //Necessary register calls to enble temp, baro ansd alt
    seconds = 0;
    lastSecond = millis();

    // attach external interrupt pins to IRQ functions
    attachInterrupt(RAIN, rainIRQ, FALLING);
    attachInterrupt(WSPEED, wspeedIRQ, FALLING);

    // turn on interrupts
interrupts();
}
//---------------------------------------------------------------
void loop()
{
     //Keep track of which minute it is
  if(millis() - lastSecond >= 1000)
  {

    lastSecond += 1000;

    //Take a speed and direction reading every second for 2 minute average
    if(++seconds_2m > 119) seconds_2m = 0;

    //Calc the wind speed and direction every second for 120 second to get 2 minute average
    float currentSpeed = get_wind_speed();
    Serial.print("currentSpeed = ");
    Serial.print(currentSpeed);
    delay(100);
    //float currentSpeed = random(5); //For testing
    int currentDirection = get_wind_direction();
     Serial.print("currentDirection = ");
    Serial.print(currentDirection);
    delay(1000);
    windspdavg[seconds_2m] = (int)currentSpeed;
    winddiravg[seconds_2m] = currentDirection;
    //if(seconds_2m % 10 == 0) displayArrays(); //For testing

    //Check to see if this is a gust for the minute
    if(currentSpeed > windgust_10m[minutes_10m])
    {
      windgust_10m[minutes_10m] = currentSpeed;
      windgustdirection_10m[minutes_10m] = currentDirection;
    }

    //Check to see if this is a gust for the day
    if(currentSpeed > windgustmph)
    {
      windgustmph = currentSpeed;
      windgustdir = currentDirection;
    }

    if(++seconds > 59)
    {
      seconds = 0;

      if(++minutes > 59) minutes = 0;
      if(++minutes_10m > 9) minutes_10m = 0;

      rainHour[minutes] = 0; //Zero out this minute's rainfall amount
      windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
    }

    //Get readings from all sensors
    getWeather();
    //Rather than use a delay, keeping track of a counter allows the photon to
    // still take readings and do work in between printing out data.
    count++;
    //alter this number to change the amount of time between each reading
    if(count == 5)
    {
       printInfo();
       count = 0;
    }
  }


      //Send data to Weather Underground
      sendToWU();

      //get response to confirm
      listen();

      //Power down between sends to save power, measured in seconds.
      //System.sleep(SLEEP_MODE_DEEP,300);  //for Particle Photon
      //Spark.sleep(SLEEP_MODE_DEEP,300);   //for Spark Core
      //delay(100);
}

//---------------------------------------------------------------
void printInfo()
{
//This function prints the weather data out to the default Serial Port
Serial.print("Wind_Dir:");
Serial.print(winddir);
      
        default:
          Serial.print("No Wind");
          // if nothing else matches, do the
          // default (which is optional)
      }

      Serial.print(" Wind_Speed:");
      Serial.print(windspeedmph, 1);
      Serial.print("mph, ");

      Serial.print("Rain:");
      Serial.print(rainin, 2);
Serial.print("in., ");

  Serial.print("Temp:");
  Serial.print(tempF);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.print("%, ");

  Serial.print("Baro_Temp:");
  Serial.print(baroTempF);
  Serial.print("F, ");

  Serial.print("Humid_Temp:");
  Serial.print(humTempF);
  Serial.print("F, ");

  Serial.print("Pressure:");
  Serial.print(pascals/100);
  Serial.print("hPa, ");
  Serial.print(inches);
  Serial.println("in.Hg");
  //The MPL3115A2 outputs the pressure in Pascals. However, most weather stations
  //report pressure in hectopascals or millibars. Divide by 100 to get a reading
  //more closely resembling what online weather reports may say in hPa or mb.
  //Another common unit for pressure is Inches of Mercury (in.Hg). To convert
  //from mb to in.Hg, use the following formula. P(inHg) = 0.0295300 * P(mb)
  //More info on conversion can be found here:
  //www.srh.noaa.gov/images/epz/wxcalc/pressureConversion.pdf

  //If in altitude mode, print with these lines
  //Serial.print("Altitude:");
  //Serial.print(altf);
  //Serial.println("ft.");

}
//---------------------------------------------------------------
void sendToWU()
{
  Serial.println("connecting...");

  if (client.connect(SERVER, 80)) { 
  Serial.println("Connected");
  client.print(WEBPAGE);
  client.print("ID=");
  client.print(ID);
  client.print("&PASSWORD=");
  client.print(PASSWORD);
  client.print("&dateutc=now");      //can use 'now' instead of time if sending in real time
  client.print("&tempf=");
  client.print(tempF);
  client.print("&dewptf=");
  client.print(dewptF);
  client.print("&humidity=");
  client.print(humidity);
  client.print("&baromin=");
  client.print(inches);
  client.print("&winddir=");
  client.print(winddir);
  client.print("&windspeedmph=");
  client.print(windspeedmph);
  client.print("&windgustmph=");
  client.print(windgustmph);
  client.print("&windgustdir=");
  client.print(windgustdir);
  client.print("&windspdmph_avg2m=");
  client.print(windspdmph_avg2m);
  client.print("&windgustmph_10m=");
  client.print(windgustmph_10m);
  client.print("&rainin=");
  client.print(rainin);
  
  //client.print("&action=updateraw");    //Standard update rate - for sending once a minute or less
  client.print("&softwaretype=Particle-Photon&action=updateraw&realtime=1&rtfreq=30");  //Rapid Fire update rate - for sending multiple times per minute, specify frequency in seconds
  client.print(" HTTP/1.0\r\n");
  client.print("Accept: text/html\r\n");
  client.print("Host: ");
  client.print(SERVER);
  client.print("\r\n\r\n");
  Serial.println("Upload complete");
  delay(300);                         //-if using sleep- it goes to sleep  too fast and the send is unreliable
  }else{
    Serial.println(F("Connection failed"));
  return;
  }
}
//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor
//Serial.print("Adc reading = ");
//Serial.print(adc);

//delay(1000);
  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  //Wind Vanes may vary in the values they return. To get exact wind direction,
  //it is recomended that you AnalogRead the Wind Vain to make sure the values
  //your wind vain output fall within the values listed below.
  if (adc < 1495) return (113);
    if (adc < 1570) return (68);
    if (adc < 1605) return (90);
    if (adc < 1750) return (158);
    if (adc < 1950) return (135);
    if (adc < 2150) return (203);
    if (adc < 2290) return (180);
    if (adc < 2660) return (23);
    if (adc < 2830) return (45);
    if (adc < 3190) return (248);
    if (adc < 3270) return (225);
    if (adc < 3450) return (338);
    if (adc < 3625) return (0);
    if (adc < 3720) return (293);
    if (adc < 3850) return (315);
    if (adc < 3960) return (270);

  return (-1); // error, disconnected?
   
}
//---------------------------------------------------------------
//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  Serial.println();

  return(windSpeed);
}
//---------------------------------------------------------------
void getWeather()
{
  // Measure Relative Humidity from the HTU21D or Si7021
  humidity = sensor.getRH();

  // Measure Temperature from the HTU21D or Si7021
  humTempC = sensor.getTemp();
  humTempF = (humTempC * 9)/5 + 32;
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()

  //Measure the Barometer temperature in F from the MPL3115A2
  baroTempC = sensor.readBaroTemp();
  baroTempF = (baroTempC * 9)/5 + 32; //convert the temperature to F

  //Measure Pressure from the MPL3115A2
  pascals = sensor.readPressure();
  inches = pascals * 0.0002953; // Calc for converting Pa to inHg (Wunderground expects inHg)

  //If in altitude mode, you can get a reading in feet with this line:
  //float altf = sensor.readAltitudeFt();

 
  //Average the temperature reading from both sensors
  tempC=((humTempC+baroTempC)/2);
  tempF=((humTempF+baroTempF)/2);

  //Calculate Dew Point
  dewptC = dewPoint(tempC, humidity);
  dewptF = (dewptC * 9.0)/ 5.0 + 32.0;
}
//---------------------------------------------------------------
// dewPoint function from NOAA
// reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
// reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
//---------------------------------------------------------------
double dewPoint(double celsius, double humidity)
{
	// (1) Saturation Vapor Pressure = ESGG(T)
	double RATIO = 373.15 / (273.15 + celsius);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
	RHS += log10(1013.246);

  // factor -3 is to adjust units - Vapor Pressure SVP * humidity
	double VP = pow(10, RHS - 3) * humidity;

  // (2) DEWPOINT = F(Vapor Pressure)
	double T = log(VP/0.61078);   // temp var
	return (241.88 * T) / (17.558 - T);
}
//---------------------------------------------------------------
void listen()
{
  Serial.println("Server Response...");
  unsigned long lastRead = millis();
  while (client.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS))
    {
    if (client.available()) {
      char c = client.read();
      Serial.print(c);
      }
    }
}
//---------------------------END---------------------------------
