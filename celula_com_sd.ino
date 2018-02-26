
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "HX711.h"
#define DOUT  A1
#define CLK  A0
#define ESCALA 0 //Número colhido no AlgCalibracao


HX711 celula(DOUT, CLK);

const int chipSelect=4;
File dataFile;

/*  vcc 5v
    miso 12
    mosi 11
    sck 13
    cs 4
    */
void setup() {
  Serial.begin(250000);

  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect)){
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.print("Leitura do Valor: ");
  Serial.println(celula.read());
  Serial.println("Não coloque nenhum objeto");
  Serial.println("Aguarde...");
  Serial.println("...");
  celula.set_scale(5737.704); // Colocaremos a ESCALA GERADA PELO O AlgCalibração
  celula.tare();  //El peso actual es considerado Tara.
  delay(5000);
  Serial.println("Coloque o peso");  


   
}

void loop() {
  dataFile=SD.open("dadosbmp.csv", FILE_WRITE);
  float forca= celula.get_units();
  
   if(dataFile){
      dataFile.print(millis());
      dataFile.print(";");
      dataFile.print(forca,3);
      dataFile.print(";");
      dataFile.print('\n');
      dataFile.close();
   }
  
  
  Serial.print(millis());
  Serial.print('\t');
  Serial.print("Peso: ");
  Serial.print(forca,3);
  Serial.println("kg");
  
  
 
}
  
