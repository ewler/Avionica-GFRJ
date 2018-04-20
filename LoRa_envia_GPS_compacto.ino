#include <SPI.h>
#include <RH_RF95.h>

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

SoftwareSerial mySerial(8, 7);

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean);

int i;
int lat;
int lon;
bool estado=true;
String str;
char lati[9];
char longe[5];

void setup()
{
 //LoRa transmission
 pinMode(RFM95_RST, OUTPUT);
 digitalWrite(RFM95_RST, HIGH);

 GPS.begin(9600);
 
 while (!Serial);
 Serial.begin(115200);
 
 Serial.println("GFRJ - GPS LOCATION");
 
 delay(100);
 Serial.println("Arduino LoRa TX Test!");
 // manual reset
 digitalWrite(RFM95_RST, LOW);
 delay(10);
 digitalWrite(RFM95_RST, HIGH);
 delay(10);
 while (!rf95.init()) {
 Serial.println("LoRa radio init failed");
 while (1);
 }
 Serial.println("LoRa radio init OK!");
 // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250,
//+13dbM
 if (!rf95.setFrequency(RF95_FREQ)) {
 Serial.println("setFrequency failed");
 while (1);
 }
 Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

 /* Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
  Sf = 128chips/symbol, CRC on
  The default transmitter power is 13dBm, using PA_BOOST.
  If you are using RFM95/96/97/98 modules which uses the PA_BOOST
  transmitter pin, then
  you can set transmitter powers from 5 to 23 dBm:*/
 rf95.setTxPower(23, false);

 
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
 GPS.sendCommand(PGCMD_ANTENNA);

 useInterrupt(true);

 delay(1000);
 // Ask for firmware version
 mySerial.println(PMTK_Q_RELEASE);

}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
//conversao de float para char
char* dtostrf ( double __val, signed char __width, unsigned char __prec, char* __s);

char conversao(char buf[55], int value){
sprintf(buf, "%d", value);
*lati = buf;
}

char conversaob(char buf[55], int value){
sprintf(buf, "%d", value);
*longe = buf;
}


uint32_t timer = millis();

int16_t packetnum = 0; // packet counter, we increment per xmission

void loop()
{
 Serial.println("Sending to rf95_server");
 Serial.println( );

if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 500) { 
    timer = millis(); // reset the timer
  
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" Qualidade: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      if(estado){
        Serial.print("Local atual: ");
        Serial.print(GPS.latitudeDegrees, 4);
        i = int(GPS.latitudeDegrees);
        lat = (GPS.latitudeDegrees-i)*10000;
        Serial.print(", "); 
        Serial.println(GPS.longitudeDegrees, 4);
        i = int(GPS.longitudeDegrees);
        lon = (GPS.longitudeDegrees-i)*10000;
        estado=false;
      }
      else{
        lat= int(GPS.altitude);
        Serial.println(lat);
        lon= int (GPS.angle);
        Serial.println(lon);
        estado=true;
      }
    }
  }
   
 if(lat<0)
  lat = lat*(-1);
 if(lon<0)
  lon = lon*(-1);
 
 //Serial.println(i);
 Serial.println(lon);
 
 str=String(lat);
 str.toCharArray(lati,5);

 str=String(lon);
 str.toCharArray(longe,5);


 Serial.println("Aqui");
 Serial.println(lati);
 

 lati[8] = '\0';
 
 for(i=4; i<8; i++){
  lati[i] = longe[i-4];
 }
 
 Serial.println(lati);
 Serial.print("Sending "); //Serial.println(radiopacket);
 

 Serial.println("Sending value..."); delay(10);
 rf95.send((uint8_t *)lati, 9);
 //Serial.println("Waiting for packet to complete..."); delay(10);
 //rf95.waitPacketSent();
 // Now wait for a reply
 //uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
 //uint8_t len = sizeof(buf);
 //Serial.println("Waiting for reply..."); delay(10);
/* if (rf95.waitAvailableTimeout(1000))
 {
 // Should be a reply message for us now
 if (rf95.recv(buf, &len))
 {
 Serial.print("Got reply: ");
 Serial.println((char*)buf);
 Serial.print("RSSI: ");
 Serial.println(rf95.lastRssi(), DEC);
 }
 else
 {
 Serial.println("Receive failed");
 }
 }
 else
 {
 Serial.println("No reply, is there a listener around?");
 }*/
 
 
 delay(1000);
}
