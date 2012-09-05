uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
  
  crc = 0xFFFF;
  
  // Calculate Checksum ignoring the first two $s
  for ( i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
  return crc;
}

void setupGPS(){
  // Turning off all GPS NMEA  strings apart on the uBlox module
  Serial1.println("$PUBX,40,GLL,0,0,0,0*5C");
  delay(1000);
  Serial1.println("$PUBX,40,GGA,0,0,0,0*5A");
  delay(1000);
  Serial1.println("$PUBX,40,GSA,0,0,0,0*4E");
  delay(1000);
  Serial1.println("$PUBX,40,RMC,0,0,0,0*47");
  delay(1000);
  Serial1.println("$PUBX,40,GSV,0,0,0,0*59");
  delay(1000);
  Serial1.println("$PUBX,40,VTG,0,0,0,0*5E");
  delay(3000); // Wait for the GPS to process all the previous commands
  
  // Set the GPS to airborne mode
  uint8_t setNav[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
      gps_get_data;
}

/*
* Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
*/
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb)
{
  *cka = 0;
  *ckb = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}

/*
* Verify the checksum for the given data and length
*/
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
  return false;
  else
  return true;
}

/*
* Get data from GPS, times out after 1 second
*/
void gps_get_data()
{
  int i = 0;
  unsigned long startTime = millis();
  while(1) {
    // Make sure data is available to read
    if (Serial1.available()) {
      buf[i] = Serial1.read();
      i++;
    }
    // Timeout if no valid response in 1 second
    if (millis() - startTime > 1000) {
      break;
    }
  }
}

/*
* Check the navigation status to determine the quality of the fix currently held by the receiver with a NAV-STATUS message
*/
void gps_check_lock()
{
  GPSerrorL = 0;
  Serial1.flush();
  // Construct the request to the GPS
  uint8_t request[8] = {0xB5, 0x62, 0x01, 0x06, 0x00, 0x00, 0x07, 0x016};
  sendUBX(request, 8);
  
  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerrorL = 100;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerrorL = 200;
  }
  
  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !_gps_verify_checksum(&buf[2], 56) ) {
    GPSerrorL = 300;
  }
  
  if(GPSerrorL == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
    lock = buf[16];
    else
    lock = 0;
    
    sats = buf[53];
  }
  else {
    lock = 0;
  }
}

/*
* Poll the GPS for a positition message then extract the useful information from it - POLLSH
*/
void gps_get_position()
{
  GPSerrorP = 0;
  Serial1.flush();
  // Request a NAV-POLLSH message from the GPS
  uint8_t request[8] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A};
  sendUBX(request, 8);
  
  // Get the message back from the GPS
  gps_get_data();
  
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62){
    GPSerrorP = 10;
  }
  if( buf[2] != 0x01 || buf[3] != 0x02 ){
    GPSerrorP = 20;
  }
  if( !_gps_verify_checksum(&buf[2], 32)){
    GPSerrorP = 30;
  }
  
  if(GPSerrorP == 0) {
    // 4 bytes of longitude (1e-7)
    lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
    //lon /= 1000;
    // Convert longitude e.g. from 48252779 to 4.8252779
    decimal_2(lon, longitude);
    
    // 4 bytes of latitude (1e-7)
    lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
    // lat /= 10000000.0000000;
    
    // Convert latititude e.g. from 518926628 to 51.8926628
    decimal_2(lat, latitude);
    
    // 4 bytes of altitude above MSL (mm)
    alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    alt /= 1000;
  }
}

/*
* Get the hour, minute and second from the GPS using the NAV-TIMEUTC message
*/
void gps_get_time()
{
  GPSerrorT = 0;
  Serial1.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {0xB5, 0x62, 0x01, 0x21, 0x00, 0x00, 0x22, 0x67};
  sendUBX(request, 8);
  
  // Get the message back from the GPS
  gps_get_data();
  
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerrorT = 1;
  }
  if( buf[2] != 0x01 || buf[3] != 0x21 ) {
    GPSerrorT = 2;
  }
  if( !_gps_verify_checksum(&buf[2], 24)){
    GPSerrorT = 3;
  }
  if(GPSerrorT == 0) {
    if(hour > 23 || minute > 59 || second > 59)
    {
      GPSerrorT = 4;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}

/*
* Verify that the uBlox 6 GPS Receiver is set to the 1G airborne mode
*/
void gps_check_nav()
{
  GPSerrorM = 0;
  Serial1.flush();
  // Send request about NAVMODE to GPS
  uint8_t request[8] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};
  sendUBX(request, 8);
  
  // Get the message back from the GPS
  gps_get_data();
  
  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerrorM = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerrorM = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&buf[2], 40) ) {
    GPSerrorM = 43;
  }
  
  // Return the navigation mode and let the caller analyze it
  navmode = buf[8];
}
  
  
// Send a byte array of UBX protocol to GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++){
    Serial1.write(MSG[i]);
  }
}

// Convert position data
void decimal_2(int32_t in, char uit[11])
{
  int integer;
  long rest;
  String result;
  
  integer = in / 10000000; 
  rest = in % (integer * 10000000);
  
  result = String(integer) + "." + String(rest);
  result.toCharArray(uit,11);
}
