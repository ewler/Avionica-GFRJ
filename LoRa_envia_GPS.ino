/* Author: GFRJ
 * Date: 06/03/2018
 * Description: Code for "Transmitter" GPS location with LoRa.
 *
 * This code originated from Adafruit's website and has been
 * adjusted for use in the Internet of rocket project.
 * GFRJ - Grupo de Foguetes do Rio de Janeiro
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

#define GPSECHO  true

//led, sensorPin, sensorValue
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int sensorPin = A0;
int ledPin = 7;
int sensorValue = -1;

int teste = 10;

SoftwareSerial mySerial(0, 1);

Adafruit_GPS GPS(&mySerial);

//bool ledON = false;

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()
{
 //LED setup
 pinMode(ledPin, OUTPUT);
 digitalWrite(ledPin, LOW);

 //LoRa transmission
 pinMode(RFM95_RST, OUTPUT);
 digitalWrite(RFM95_RST, HIGH);
 
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
 GPS.sendCommand(PGCMD_ANTENNA);

 while (!Serial);
 Serial.begin(115200);
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

 // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
//Sf = 128chips/symbol, CRC on
 // The default transmitter power is 13dBm, using PA_BOOST.
 // If you are using RFM95/96/97/98 modules which uses the PA_BOOST
//transmitter pin, then
 // you can set transmitter powers from 5 to 23 dBm:
 rf95.setTxPower(23, false);
 
 useInterrupt(true);
 delay(1000);
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

uint32_t timer = millis();

int16_t packetnum = 0; // packet counter, we increment per xmission

void loop()
{

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
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
  
 Serial.println("Sending to rf95_server");
 //MB1220 sensing ...
 Serial.println("Sensing....");
 sensorValue = analogRead(sensorPin);
 Serial.print("Value found: ");
 Serial.println(sensorValue);
 // Send a message to rf95_server
 char radiopacket[5] = "liga";
  
 // Change the char to from ASCII to HEX(16) or DECI(10) value
 // itoa(packetnum++, radiopacket+13, 16);
 Serial.print("Sending "); Serial.println(radiopacket);
 radiopacket[4] = 0;

 Serial.println("Sending value..."); delay(10);
 rf95.send((int)teste, 3);
 Serial.println("Waiting for packet to complete..."); delay(10);
 rf95.waitPacketSent();
 // Now wait for a reply
 uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
 uint8_t len = sizeof(buf);
 Serial.println("Waiting for reply..."); delay(10);
 if (rf95.waitAvailableTimeout(1000))
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
 }
  
 
 
 delay(1000);
}
