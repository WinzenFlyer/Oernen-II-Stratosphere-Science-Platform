
void rtty_txstring(char * string)
{
  /* Simple function to send a char at a time to 
  rtty_txbyte function.
  NB each char is one byte (8 bits) */
  
  char c;
  
  c= *string++;
  
  while( c != '\0')
  {
    rtty_txbyte(c);
    c = *string++;
  }
}

void rtty_txbyte(char c)
{
  /* Simple function to send a each bit of a char to rtty_txbit function.
  
  NB The bits are sent Least Significant Bit first
  
  All chars should be preceded with a 0 and succeeded by a 1. 0 = start bit, 1 = stop bit */
  
  int i;
  
  rtty_txbit(0); // Start Bit
  
  // Send bits for char LSB first
  
  for(i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if(c & 1) rtty_txbit(1);
    
    else rtty_txbit(0);
    
    c = c >> 1;
  }
  rtty_txbit(1); // Stop Bit
}

void rtty_txbit(int bit)
{
  if(bit)
  {
    // high
    digitalWrite(RADIO_MARK_PIN, HIGH);
    digitalWrite(RADIO_SPACE_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
  // low
  digitalWrite(RADIO_MARK_PIN, LOW);
  digitalWrite(RADIO_SPACE_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);
  }
  
//  delayMicroseconds(1680); // 600 Baud, unlikely to work
  delayMicroseconds(3370); // 300 Baud
//  delayMicroseconds(10000); // For 50 Baud uncommend this and the line below.
//  delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.
}
