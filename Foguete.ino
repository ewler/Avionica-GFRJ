/* Author: GFRJ
 * Date: 
 * Description: Code for .
 *
 * This code originated from Adafruit's website and has been
 * adjusted for use in the.
 * 
 */
// LoRa RFM9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client
//(transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for
//addressing or
// reliability, so you should only use RH_RF95 if you do not need the
//higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 433.0 // Change to 434.0 or other frequency, must match RX's freq!

#define GPSECHO  false

int ADXAddress = 0x53;  // the default 7-bit slave address
int reading = 0;
int val=0;
int X0,X1,X_out;
int Y0,Y1,Y_out;
int Z1,Z0,Z_out;
double Xg,Yg,Zg;

boolean usingInterrupt = false;
void useInterrupt(boolean);

int i;
float lat;
float lon;
char data[10];
char datab[10];
char lati[20];
char longe[10];

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

SoftwareSerial mySerial(8, 7);

Adafruit_GPS GPS(&mySerial);

void setup()
{
 //LoRa transmission
 pinMode(RFM95_RST, OUTPUT);
 digitalWrite(RFM95_RST, HIGH);

 GPS.begin(9600);
 Wire.begin(); //Gy-80                

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


 Wire.beginTransmission(ADXAddress);
 Wire.write(Register_2D);
 Wire.write(8);                //measuring enable
 Wire.endTransmission();     // stop transmitting 

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

uint32_t timer = millis();

int16_t packetnum = 0; // packet counter, we increment per xmission


void loop()
{

//--------------X
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_X0);
  Wire.write(Register_X1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    X0 = Wire.read();
    X1 = Wire.read(); 
    X1=X1<<8;
    X_out=X0+X1;   
  }

  //------------------Y
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_Y0);
  Wire.write(Register_Y1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    Y0 = Wire.read();
    Y1 = Wire.read(); 
    Y1=Y1<<8;
    Y_out=Y0+Y1;
  }
  //------------------Z
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_Z0);
  Wire.write(Register_Z1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    Z0 = Wire.read();
    Z1 = Wire.read(); 
    Z1=Z1<<8;
    Z_out=Z0+Z1;
  }
  //
  Xg=X_out/256.0;
  Yg=Y_out/256.0;
  Zg=Z_out/256.0;

  //falta armazenar essas paradas

  
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
      Serial.print("Local atual: ");
      Serial.print(GPS.latitudeDegrees, 4);
      lat = GPS.latitudeDegrees;
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      lon = GPS.longitudeDegrees;
    }
  }
   
 // Change the char to from ASCII to HEX(16) or DECI(10) value
 // itoa(packetnum++, radiopacket+13, 16);

 char *lati = dtostrf(lat,8,4,data);
 char *longe = dtostrf(lon,8,4,datab);

 lati[8] = '/';
 lati[9] = '/';
 lati[18] = '\0';
 
 for(i=10; i<18; i++){
  lati[i] = longe[i-10];
 }
 
 Serial.println(lati);
 Serial.print("Sending "); //Serial.println(radiopacket);
 

 Serial.println("Sending value..."); delay(10);
 rf95.send((uint8_t *)lati, 20);
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
