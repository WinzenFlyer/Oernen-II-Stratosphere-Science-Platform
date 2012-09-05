// Radio and Sensor Code by Janez Stefulj & Kevin Glinka, End of 2011

#include <string.h>
#include <Wire.h>
#include <OneWire.h>
#include <util/crc16.h>

// Variables

int32_t lat = 0, lon = 0, alt = 0;
uint8_t hour = 0, minute = 0, second = 0, lock = 0, sats = 0;
unsigned long startGPS = 0;
int GPSerrorM = 0, GPSerrorL = 0, GPSerrorP = 0, GPSerrorT = 0, count = 0, n, gpsstatus, lockcount = 0;
byte navmode = 99;
byte error = 0;
char latitude[11];
char longitude[11];
char GPSAltitude[20];


uint8_t buf[70]; // GPS receive buffer 
int DS18B20_Pin = 2; // DS18B20 Signal Pin on digital 2
int HumPin = A7;
int BattPin = A1;

int RH;
float BitToVolt = 0.0025;  // 2.56/1024

char TemperatureString[25];
char PressureString[25];
char PressAltitudeString[25];
char DSTemperString[25];
char HumidityString[25];
char DewPointString[25];
char VoltageString[25];
char GPSAltitudeString[25];
char NavString[25];
char ErrorString[25];

#define BMP085_ADDRESS 0x77 // I2C address of BMP085

//Temperature Chip I/0
OneWire ds(DS18B20_Pin); // on digital pin 2

void fmtDouble(double val, byte precision, char *buf, unsigned bufLen = 0xffff);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen = 0xffff, byte width = 0);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width)
{
  if(!buf || !bufLen)
  return(0);
  
  // produce the digit string (backwards in the digit buffer)
  char dbuf[10];
  unsigned idx = 0;
  while (idx < sizeof(dbuf))
  {
    dbuf[idx++] = (val % 10) + '0';
    if((val /= 10) == 0)
    break;
  }
  
  // copy the optional leading zeroes and digits to the target buffer
  unsigned len = 0;
  byte padding = (width > idx) ? width - idx : 0;
  char c = '0';
  while ((--bufLen > 0) && (idx || padding))
  {
    if(padding)
    padding--;
    else
    c = dbuf[--idx];
    *buf++ = c;
    len++;
  }
  
  // add the null termination
  *buf = '\0';
  return(len);
}
void fmtDouble(double val, byte precision, char *buf, unsigned bufLen)
{
  if(!buf || !bufLen)
  return;
  
  // limit the precision to the maximum allowed value
  const byte maxPrecision = 6;
  if (precision > maxPrecision)
  precision = maxPrecision;
  
  if(--bufLen > 0)
  {
    // check for a negative value
    if(val < 0.0)
    {
      val = -val;
      *buf = '-';
      bufLen--;
    }
    
    // compute the rounding factor and fractional multiplier
    double roundingFactor = 0.5;
    unsigned long mult = 1;
    for(byte i = 0; i < precision; i++)
    {
      roundingFactor /= 10.0;
      mult *= 10;
    }
    
    if(bufLen > 0)
    {
      // apply the rounding factor
      val += roundingFactor;
      
      // add the integral portion to the buffer
      unsigned len = fmtUnsigned((unsigned long)val, buf, bufLen);
      buf += len;
      bufLen -= len;
    }
    
    // handle the fractional portion
    
    if((precision > 0) && (bufLen > 0))
    {
      *buf++ = '.';
      if(--bufLen > 0)
      buf += fmtUnsigned((unsigned long)((val - (unsigned long)val) * mult), buf, bufLen, precision);
    }
  }
  // null-terminate the string
  *buf = '\0';
}


int RADIO_SPACE_PIN = 4;
int RADIO_MARK_PIN = 5;
int LED_PIN = 6; // onboard LED
char NAVISTRING[200];
char DATASTRING[400];


uint16_t crccat(char *msg)
{
	uint16_t x;
	for(x = 0xFFFF; *msg; msg++)
		x = _crc_xmodem_update(x, *msg);
	snprintf(msg, 8, "*%04X\n", x);
	return(x);
}


 
// Setup and Loop.

void setup(){
  analogReference(INTERNAL2V56);
  pinMode(RADIO_SPACE_PIN, OUTPUT);
  pinMode(RADIO_MARK_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Wire.begin();
  setupGPS();
  delay(100);
  bmp085Calibration();
  delay(100);
}

void loop()
{
  char checksum[10];
  int n;
  
  gps_check_nav();
  gps_check_lock();
  gps_get_position();
  gps_get_time();
  error = GPSerrorM + GPSerrorL + GPSerrorP + GPSerrorT;
  float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pressure = bmp085GetPressure(bmp085ReadUP());
  float atm = pressure / 101325; // "standard atmosphere"
  float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 
  
  float temperature_DS = getTemp();
  float hum = Humidity();
  float delta = DewPoint();
  float volt = BattV();

  
  fmtDouble(temperature, 2, TemperatureString);
  fmtDouble(pressure, 2, PressureString);
  fmtDouble(altitude, 2, PressAltitudeString);
  fmtDouble(alt, 1, GPSAltitudeString);
  fmtDouble(temperature_DS, 2, DSTemperString);
  fmtDouble(error, 1, ErrorString);
  fmtDouble(navmode, 1, NavString);
  fmtDouble(hum, 2, HumidityString);
  fmtDouble(delta, 2, DewPointString);
  fmtDouble(volt, 2, VoltageString);
  
  sprintf(NAVISTRING, "$$OERNEN-II,%d,%02d:%02d:%02d,%s,%s,%s,%s,%s,%s", count, hour, minute, second, latitude, longitude, GPSAltitudeString, NavString, ErrorString, VoltageString);
  sprintf(DATASTRING, "%d,%02d:%02d:%02d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", count, hour, minute, second, latitude, longitude, GPSAltitudeString, NavString, ErrorString, TemperatureString, PressureString, PressAltitudeString, DSTemperString, HumidityString, DewPointString, VoltageString);
  /* Append the checksum, skipping the $$ prefix */
  crccat(NAVISTRING + 2);
  noInterrupts();
  rtty_txstring(NAVISTRING);
  interrupts();
  Serial2.print(DATASTRING);
  count++;
}
