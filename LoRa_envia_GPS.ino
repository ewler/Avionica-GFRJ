/* Author: Marissa Kwon
 * Date: 2/28/2017
 * Description: Code for "Transmitter" LoRa radio breakout; initilizes
 * LoRa radio transmitter and sends data to LoRa receiver on the same
 * frequency.
 *
 * This code originated from Adafruit's website and has been
 * adjusted for use in the Internet of Agriculture project.
 * URSA Engage Student Research at OPEnS Lab at Oregon State
University.
 */
// LoRa 9x_TX
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

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
//led, sensorPin, sensorValue
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

SoftwareSerial mySerial(8, 7);

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean);

float lat;
char data[10];
char lati[20];
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
      Serial.print("Local atual: ");
      Serial.print(GPS.latitudeDegrees, 4);
      lat = GPS.latitudeDegrees;
      char *lati = dtostrf(lat,8,4,data);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
    }
  }
   
 // Change the char to from ASCII to HEX(16) or DECI(10) value
 // itoa(packetnum++, radiopacket+13, 16);
 
 Serial.print("Sending "); //Serial.println(radiopacket);
 //lati[9] = 0;

 Serial.println("Sending value..."); delay(10);
 rf95.send((uint8_t *)lati, 10);
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
