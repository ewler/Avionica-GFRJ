#include <SPI.h>
#include <RH_RF95.h>



#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

//========BMP180=========
//=========CONSTANTES/VARIAVEIS E BIBLIOTECAS SBPM 180=========
//Define o endereco I2C do BMP085 - 0x77 ou 119 em decimal
#define BMP085_ADDRESS 0x77  
const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
long b5; 
#include <Wire.h>

#define MEDIA 10
#define TAMANHOVETOR 5

float dados[MEDIA];
int cont=0;
float vetAlt[TAMANHOVETOR];
int contApogeu=0;
bool estado=false;


//===================================CONFIGURATIONS====================================
// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}
//
// Armazena todos os valores de calibracao do BMP085 em 
// variaveis globais. Valores de calibracao sao exigidos para
// calcular temperatura e pressao
// Esta funcao deve ser chamada/acionada no inicio do programa
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}


int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calcula B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calcula B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

float calcAltitude(float pressure)
{
  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}

// Calcula a temperatura em graus C
float bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

//=================================FIM CONFIGURATIONS==============================

//=================================FILTRO DOS DADOS================================
float mediamovel(float valor) {
  float soma=0;
  for (int i = MEDIA - 1; i > 0; i--) {
    dados[i] = dados[i - 1];
  }
  dados[0]=valor;  
  for (int i=0; i<MEDIA; i++){
    soma+= dados[i];  
  }
  
  return soma/MEDIA;
}

//====================================DETECÇÃO DE APOGEU===================================
void detecapogeu(float altitude[]){
  int cont=0;
  float apogeu= 0.0; 
  for(int i=1; i<TAMANHOVETOR; i++){
    if(altitude[i-1]>altitude[i]){
      cont++;
      Serial.println(cont);
    }
    if(apogeu<altitude[i-1]){
      apogeu=altitude[i-1];
    }
  }
  if(cont>(TAMANHOVETOR*0.65)){
    estado=true;
    Serial.print("Apogeu");
  }
}


//=========================FIM BMP180==========================



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
String str;
char lati[9];
char longe[5];

void setup(){  
   //LoRa transmission
   pinMode(RFM95_RST, OUTPUT);
   digitalWrite(RFM95_RST, HIGH);
  
   GPS.begin(9600);
   
   while (!Serial);
   Serial.begin(115200);

   Wire.begin();
   // Inicializa o BMP085
   bmp085Calibration();

   
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
 //Serial.println("Sending to rf95_server");
 //Serial.println( );
  
  if(estado){
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
          i = int(GPS.latitudeDegrees);
          lat = (GPS.latitudeDegrees-i)*10000;
          Serial.print(", "); 
          Serial.println(GPS.longitudeDegrees, 4);
          i = int(GPS.longitudeDegrees);
          lon = (GPS.longitudeDegrees-i)*10000;
          }
      }
       
     if(lat<0)
      lat = lat*(-1);
     if(lon<0)
      lon = lon*(-1);
      
     str=String(lat);
     str.toCharArray(lati,5);
    
     str=String(lon);
     str.toCharArray(longe,5);
    
    // Serial.println("Aqui");
    // Serial.println(lati); 
     lati[8] = '\0';
     for(i=4; i<8; i++){
      lati[i] = longe[i-4];
     }
  }
  else{
      float temperature = bmp085GetTemperature(bmp085ReadUT());

      float pressure = bmp085GetPressure(bmp085ReadUP()); 
      // Chama a rotina que calcula a altitude
      float altitude = calcAltitude(pressure); 
      float realaltitude=mediamovel(altitude);
      //Serial.println(altitude);
      Serial.println(realaltitude);
      if((contApogeu+1)==TAMANHOVETOR){
        detecapogeu(vetAlt);
        contApogeu=0;
      }
      else{
        vetAlt[contApogeu++]= realaltitude;
        
      }

     str=String(int(realaltitude));
     str.toCharArray(lati,6);
     Serial.println(lati);
      
  }
  
 //Serial.println(lati); 
 //Serial.print("Sending "); //Serial.println(radiopacket);
 //Serial.println("Sending value..."); delay(10);
 rf95.send((uint8_t *)lati, 9);
      
 delay(1000);
}
