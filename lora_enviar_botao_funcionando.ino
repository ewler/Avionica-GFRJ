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
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
//led, sensorPin, sensorValue
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int sensorPin = A0;
int ledPin = 7;
int sensorValue = -1;
//bool ledON = false;
void setup()
{
 //LED setup
 pinMode(ledPin, OUTPUT);
 digitalWrite(ledPin, LOW);

 //LoRa transmission
 pinMode(RFM95_RST, OUTPUT);
 digitalWrite(RFM95_RST, HIGH);

 pinMode(4, INPUT);

 while (!Serial);
 Serial.begin(9600);
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
}
int16_t packetnum = 0; // packet counter, we increment per xmission
void loop()
{
 Serial.println("Sending to rf95_server");
  int valor = 0;
 //MB1220 sensing ...
 Serial.println("Sensing....");
 sensorValue = analogRead(sensorPin);
 Serial.print("Value found: ");
 Serial.println(sensorValue);
 valor = digitalRead(4);
 Serial.println(valor);
 // Send a message to rf95_server
 //char palavra = "nada";
 char radiopacket[5] = "liga";
 char radiopacket2[10] = "desliga";

 if(valor == 1){
   
 // Change the char to from ASCII to HEX(16) or DECI(10) value
 // itoa(packetnum++, radiopacket+13, 16);
 Serial.print("Sending "); Serial.println(radiopacket);
 radiopacket[4] = 0;

 Serial.println("Sending value..."); delay(10);
 rf95.send((uint8_t *)radiopacket, 5);
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
 } else{
   
 // Change the char to from ASCII to HEX(16) or DECI(10) value
 // itoa(packetnum++, radiopacket+13, 16);
 Serial.print("Sending "); Serial.println(radiopacket2);
 radiopacket2[9] = 0;

 Serial.println("Sending value..."); delay(10);
 rf95.send((uint8_t *)radiopacket2, 10);
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
 }
 
 delay(1000);
}
