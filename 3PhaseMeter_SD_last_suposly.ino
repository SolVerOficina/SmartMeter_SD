/*
Copyright (c) 2021 Jakub Mandula
Example of using multiple PZEM modules together on one ModBUS.
================================================================
First of all, use the PZEMChangeAddress example in order to assign 
each individual PZEM module a unique custom address. This example 
requires 2 PZEM modules with addresses 0x10 and 0x11.
Then for each PZEM module create a PZEM004Tv30 instance passing a custom address
to the address field.
The instances can either be stored as individual objects:
```c
PZEM004Tv30 pzem0(&Serial2, 0x10);
PZEM004Tv30 pzem1(&Serial2, 0x11);
PZEM004Tv30 pzem2(&Serial2, 0x12);
pzem0.voltage();
pzem1.pf();
```
Or in an array and addressed using the array index:
```c
PZEM004Tv30 pzems[] = {
    PZEM004Tv30(&Serial2, 0x10),
    PZEM004Tv30(&Serial2, 0x11),
    PZEM004Tv30(&Serial2, 0x12)};
pzems[0].voltage();
pzems[1].pf();*/


#include <PZEM004Tv30.h>
 //#include <esp_now.h>
//#include <esp_wifi.h>
//#include <Arduino_JSON.h>
#include <WiFi.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <ESP32Time.h>
#include <Preferences.h>
#include <BLEDevice.h> 
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//#include <TimeLib.h>
#include <Separador.h>

#define PIN_RED    25 // 
#define PIN_GREEN  26 // 
#define PIN_BLUE   27 //

#define SERVICE_UUID           "ffe0" // UART service UUID a5f81d42-f76e-11ea-adc1-0242ac120002
#define CHARACTERISTIC_UUID_RX "ffe1"
#define CHARACTERISTIC_UUID_TX "ffe2"
    
#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#endif

#if !defined(PZEM_SERIAL)
#define PZEM_SERIAL Serial2
#endif

#define NUM_PZEMS 3
#define led 2
//PZEM004Tv30 pzems[NUM_PZEMS];

PZEM004Tv30 pzems[NUM_PZEMS];

/* ***************************************************************
 * Uncomment USE_SOFTWARE_SERIAL in order to enable Softare serial
 *
 * Does not work for ESP32
 *****************************************************************/
//#define USE_SOFTWARE_SERIAL

Preferences preferences;

#if defined(USE_SOFTWARE_SERIAL) && defined(ESP32)
    #error "Can not use SoftwareSerial with ESP32"
#elif defined(USE_SOFTWARE_SERIAL)

#include <SoftwareSerial.h>

SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
#endif

#define contador_num 1
//tiempo entre reseteos de la tarjeta
int contador_callback = 0; 
int rest;
int lectura;
int lapso;
int estadoActualV1 = 0;
int estadoActualI1 = 0;
int estadoAnteriorV1 = 0;
int estadoAnteriorI1 = 0;
int estadoActualV2 = 0;
int estadoActualI2 = 0;
int estadoAnteriorV2 = 0;
int estadoAnteriorI2 = 0;
int estadoActualV3 = 0;
int estadoActualI3 = 0;
int estadoAnteriorV3 = 0;
int estadoAnteriorI3 = 0;
String hoy;
bool bandera1 = false;
int reset_time = 900;
String intervalo;
unsigned long inter;
String nombre;
String timestamp;
String words;
unsigned long epoch = 0;
//tiempo entre envio de la 
int cont = 0;;
unsigned long lastTime = 0;
unsigned long timerDelay = 20000;
int contador = 0;
String datosbt;
int contador_envio = 0;
unsigned long tiempo_lecturas = 5;
bool start;
bool flagset;
bool banderasd = true;
int ehora;
int eminuto;
int esegundo;          
int edia;
int emes;
int eano;
Separador s;

const int readInterval = 30000; // Intervalo de tiempo entre las lecturas en milisegundos (30 segundos)
const int writeInterval = 60000 * 60 * 24; // Intervalo de tiempo entre las escrituras en milisegundos (1 día)
unsigned long lastReadTime = 0; // Variable para almacenar el tiempo de la última lectura
unsigned long lastWriteTime = 0; // Variable para almacenar el tiempo de la última escritura

const int MAX_DATA_POINTS = 80; // Número máximo de puntos de datos que se almacenarán
String dataPoints[MAX_DATA_POINTS]; // Array para almacenar los puntos de datos

int numDataPoints = 0; // Contador de puntos de datos

//fase 1
float voltaje1;
float corriente1;
float current;
float podersct;
float energiasct;
float frecuencia1;
float factor1;
int readingId;

//fase 2
float voltaje2;
float corriente2;
float current2;
float poder2;
float energia2;
float frecuencia2;
float factor2;

//fase 3
float voltaje3;
float corriente3;
float current3;
float poder3;
float energia3;
float frecuencia3;
float factor3;

// Variables micro SD
float v1;
float i1;
float fp1;
float fr1;
float pw1;
float en1;
float v2;
float i2;
float fp2;
float fr2;
float pw2;
float en2;
float v3;
float i3;
float fp3;
float fr3;
float pw3;
float en3;
String dataMessage;
String fecha;
String visualdp;
String savedataa;
ESP32Time rtc;

//Offset para el diferente uso de sensores---
//Sensor SCT-003
//calculo real(multimetro)/calculo aproximado (medidor)
float sct_offset1 = 1.78;
float sct_offset2 = 1.75;
float sct_offset3 = 1.74;

void setColor(int R, int G, int B) {
  analogWrite(PIN_RED,   R);
  analogWrite(PIN_GREEN, G);
  analogWrite(PIN_BLUE,  B);
}

void failure(){
  Serial.println("failure");
  setColor(255, 0, 0);
  delay(1000);
  setColor(0, 0, 0);
  delay(500);
  setColor(255, 0, 0);
  delay(1000);
  setColor(0, 0, 0);
  delay(500);
  setColor(255, 0, 0);
  delay(500);
  setColor(0, 0, 0);
}
void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
     file = root.openNextFile();
  }
}

void save_sd_inicio(){
  setColor(0, 0, 255);
   // Get sensor readings
    v1 = 0;
    i1 = 0;
    fp1 = 0;
    fr1 = 0;
    pw1 = 0;
    en1 = 0;
    v2 = 0;
    i2 = 0;
    fp2 = 0;
    fr2 = 0;
    pw2 = 0;
    en2 = 0;
    v3 = 0;
    i3 = 0;
    fp3 = 0;
    fr3 = 0;
    pw3 = 0;
    en3 = 0;
    hoy = "00:00:00";
    cont++;   
    Serial.print("Dato N°: ");
    Serial.println(cont);
    Serial.println();
    //Concatenate all info separated by commas
    dataMessage =String(fecha) + ", " + String(hoy) +  ", " + String(v1) + ", " + String(i1) + ", " + String(pw1)+ ", " + String(en1)+ ", " + String(fr1) + ", " + String(fp1) + ", " + String(v2) + ", " + String(i2) + ", " + String(pw2)+ ", " + String(en2)+ ", " + String(fr2) + ", " + String(fp2)+ ", " + String(v3) + ", " + String(i3) + ", " + String(pw3)+ ", " + String(en3)+ ", " + String(fr3) + ", " + String(fp3)+ "\r\n";
    // String(cont) + ", " + 
    Serial.print("Saving data: ");
    Serial.println(dataMessage);

    
    //Append the data to file
    appendFile(SD, "/Medicion.txt", dataMessage.c_str());
   delay(500);  
   setColor(0, 0, 0);
   
}

void initSDCard(){
   if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    setColor(255, 0, 0);
    delay(500);
    setColor(0, 0, 0);
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    Serial.println("----------------------------------------------------------------------------------------------------------------");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
    Serial.println("----------------------------------------------------------------------------------------------------------------");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device Connected True");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
       Serial.println("Device Connected False");
       delay(1000);
       ESP.restart();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  // esta tiene que ser global 
   //sabe que tiene que llegar hasta 4
  //antes llegaba un string: 3,hogar,1231231243
  //ahora va a llega por separad: 3........ hogar ....... 1231231241
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {           
        Serial.println("***");
        
    if (rxValue.find("delete") != -1) {
      Serial.println("reiniciar sd - borrar datos");         
      deleteFile(SD, "/Medicion.txt");      
      nombre = " "; 
 
      ESP.restart();     
    }
   if (rxValue.find("restart") != -1) {
       Serial.println("reiniciar contador de energia");
       pzems[0].resetEnergy();
       pzems[1].resetEnergy();
       pzems[2].resetEnergy();  
       flagset = false;
       ESP.restart(); 
    }          
        for (int i = 0; i < rxValue.length(); i++){
         // Serial.print(rxValue[i]);
         datosbt += rxValue[i];                                   
        } 

         if(contador_callback == 0 ){
           setColor(255, 0, 255);
           nombre = datosbt;
           preferences.begin("my-app", false);
           preferences.putString("nombre",nombre);
           preferences.end();
           Serial.println("Nombre:" + nombre + "-");
           datosbt = " ";                    
         }      
         if(contador_callback == 1 ){
           setColor(0, 0, 0);
           intervalo = datosbt;  
            String words = intervalo; //reassign same string at the start of loop.
            Serial.println(words);
            char c;
            char no = ' '; //character I want removed.
            for (int i=0; i<words.length()-1;++i){
                c = words.charAt(i);
                if(c==no){
                    words.remove(i, 1);
                }
            }
            Serial.println(words);
           preferences.begin("my-app", false);
           preferences.putString("intervalo",words);
           preferences.end(); 
           Serial.println("Intervalo:" + words + "-");  
           datosbt = " ";       
         }
         if(contador_callback == 2 ){
           setColor(255, 0, 255);
           timestamp = datosbt; 
           String words = timestamp; //reassign same string at the start of loop.


            char c;
            char no = ' '; //character I want removed.
            for (int i=0; i<words.length()-1;++i){
                c = words.charAt(i);
                if(c==no){
                    words.remove(i, 1);
                }
            }
           preferences.begin("my-app", false);
           preferences.putString("timestamp",words);
           preferences.end();
           Serial.println("TimeStamp:" + words + "-"); 
                   

           Serial.println(ehora);           
                             
           datosbt = " ";   
           contador_callback = 0;
           flagset = false; 
           setColor(0, 0, 0);   
           delay(1000);              
         }             
           contador_callback++; 

 

      //  //flagset = false;
  } 
 }
};

void ble_start(){
   // Create the BLE Device
  BLEDevice::init("SMARTMETERSD-SV"); // Give it a name SMARTMETERSD-SV

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void read_values(){
        Serial.print("PZEM ");
        Serial.print("1");
        Serial.print(" - Address:");
        Serial.println(pzems[0].getAddress(), HEX);
        Serial.println("===================");
        // Read the data from the sensor
        voltaje1 = pzems[0].voltage();
        corriente1 = pzems[0].current();
        if(corriente1 < 1 ){
          current = pzems[0].current();
        }else{
          current = pzems[0].current()*sct_offset1;
        }
        factor1 = pzems[0].pf();
        //poder activo 1
        podersct = pzems[0].power()*sct_offset1;
        //poder aparente 1
       // int aparente = poder1/factor1;
        energiasct = pzems[0].energy()*sct_offset1;
        frecuencia1 = pzems[0].frequency();
        // Check if the data is valid
        if(isnan(voltaje1)){
            Serial.println("Error reading voltage1");
        } else if (isnan(current)) {
            Serial.println("Error reading current1");
        } else if (isnan(podersct)) {
            Serial.println("Error reading power1");
        } else if (isnan(energiasct)) {
            Serial.println("Error reading energy1");
        } else if (isnan(frecuencia1)) {
            Serial.println("Error reading frequency1");
        } else if (isnan(factor1)) {
            Serial.println("Error reading power factor1");
        } else {
            // Print the values to the Serial console
            Serial.print("Voltage 1: ");      Serial.print(voltaje1);      Serial.println("V");
            Serial.print("Current 1: ");      Serial.print(current);      Serial.println("A");           
            Serial.print("Power 1: ");        Serial.print(podersct);        Serial.println("W");
            Serial.print("Energy 1: ");       Serial.print(energiasct,3);     Serial.println("kWh");      
            Serial.print("Frequency 1: ");    Serial.print(frecuencia1, 1); Serial.println("Hz");
            Serial.print("PF 1: ");           Serial.println(factor1);          
        }

        Serial.println("-------------------");
        Serial.println();

 //Fase 2 --------------------------------------

        // Print the Address of the PZEM
        Serial.print("PZEM ");
        Serial.print("2");
        Serial.print(" - Address:");
        Serial.println(pzems[1].getAddress(), HEX);
        Serial.println("===================");

        // Read the data from the sensor
        voltaje2 = pzems[1].voltage();
        corriente2 = pzems[1].current();
        if(corriente2 < 1 ){
          current2 = pzems[1].current();
        }else{
          current2 = pzems[1].current()*sct_offset2;
        }
        factor2 = pzems[1].pf();
        poder2 = pzems[1].power()*sct_offset2;
        energia2 = pzems[1].energy()*sct_offset2;
        frecuencia2 = pzems[1].frequency();
    
        // Check if the data is valid
        if(isnan(voltaje2)){
            Serial.println("Error reading voltage 2");
        } else if (isnan(corriente2)) {
            Serial.println("Error reading current 2");
        } else if (isnan(poder2)) {
            Serial.println("Error reading power 2");
        } else if (isnan(energia2)) {
            Serial.println("Error reading energy 2");
        } else if (isnan(frecuencia2)) {
            Serial.println("Error reading frequency 2");
        } else if (isnan(factor2)) {
            Serial.println("Error reading power factor 2");
        } else {
            // Print the values to the Serial console
            Serial.print("Voltage 2: ");      Serial.print(voltaje2);      Serial.println("V");
            Serial.print("Current 2: ");      Serial.print(current2);      Serial.println("A");
            Serial.print("Power 2: ");        Serial.print(poder2);        Serial.println("W");
            Serial.print("Energy 2: ");       Serial.print(energia2,3);     Serial.println("kWh");
            Serial.print("Frequency 2: ");    Serial.print(frecuencia2, 1); Serial.println("Hz");
            Serial.print("PF 2: ");           Serial.println(factor2);

        }

        Serial.println("-------------------");
        Serial.println();

 //Fase 3 -------------------------------------------

        // Print the Address of the PZEM
        Serial.print("PZEM ");
        Serial.print("3");
        Serial.print(" - Address:");
        Serial.println(pzems[2].getAddress(), HEX);
        Serial.println("===================");

        // Read the data from the sensor
        voltaje3 = pzems[2].voltage();
        corriente3 = pzems[2].current();
        if(corriente3 < 1 ){
          current3 = pzems[2].current();
        }else{
          current3 = pzems[2].current()*sct_offset3;
        }
        factor3 = pzems[2].pf();
        poder3 = pzems[2].power()*sct_offset3;
        energia3 = pzems[2].energy()*sct_offset3;
        frecuencia3 = pzems[2].frequency();
        
        // Check if the data is valid
        if(isnan(voltaje3)){
            Serial.println("Error reading voltage3");
        } else if (isnan(corriente3)) {
            Serial.println("Error reading current3");
        } else if (isnan(poder3)) {
            Serial.println("Error reading power3");
        } else if (isnan(energia3)) {
            Serial.println("Error reading energy3");
        } else if (isnan(frecuencia3)) {
            Serial.println("Error reading frequency3");
        } else if (isnan(factor3)) {
            Serial.println("Error reading power factor3");
        } else {
            // Print the values to the Serial console
            Serial.print("Voltage 3: ");      Serial.print(voltaje3);      Serial.println("V");
            Serial.print("Current 3: ");      Serial.print(current3);      Serial.println("A");
            Serial.print("Power 3: ");        Serial.print(poder3);        Serial.println("W");
            Serial.print("Energy 3: ");       Serial.print(energia3,3);     Serial.println("kWh");
            Serial.print("Frequency 3: ");    Serial.print(frecuencia3, 1); Serial.println("Hz");
            Serial.print("PF 3: ");           Serial.println(factor3);
        }
        Serial.println("Next -------------------");
        Serial.println();
    Serial.println();
    delay(1000);
}

void setup() {
    Serial.begin(115200);
    pinMode(led, OUTPUT);
      pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);
    
    //Serial.print("Tiempo empezando el loop  -----> ");
    //Serial.println(hoy);
    Serial.println(WiFi.macAddress());
    //rtc.setTime(30, 34, 8, 11, 10, 2022);
    preferences.begin("my-app", false);
    //unsigned int counter = preferences.getUInt("counter", 0);
    nombre = preferences.getString("nombre", "");
    intervalo = preferences.getString("intervalo", "");
    timestamp = preferences.getString("timestamp", "");
    preferences.putUInt("counter", contador);
    preferences.end();
    Serial.println("------------ DATOS GUARDADOS ------------");
    Serial.println(nombre);
    Serial.println(intervalo);
    Serial.println(timestamp); 
     String temp = String(timestamp);
           String hora = s.separa(temp, ',',0);
           String minuto = s.separa(temp, ',',1);                       
           String segundo = s.separa(temp, ',',2);  
           String dia = s.separa(temp, ',',3);
           String mes = s.separa(temp, ',',4);                      
           String ano = s.separa(temp, ',',5);   

            ehora = hora.toInt();
            eminuto = minuto.toInt(); //minuto.toInt();
            esegundo = segundo.toInt();          
            edia = dia.toInt();
            emes = mes.toInt();
            eano = ano.toInt();  
            int sano = eano + 2000;  
           Serial.println(ehora);
           Serial.println(eminuto);
           Serial.println(esegundo);
           Serial.println(edia);
           Serial.println(emes);
           Serial.println(sano);
 //rtc.setTime(second(esegundo), minute(eminuto), hour(ehora), day(edia), month(emes), year(eano));   
  rtc.setTime(esegundo, eminuto, ehora, edia, emes, sano);  
   inter = intervalo.toInt();
   Serial.println(inter);
    //fecha = dia+"-"+mes+"-"+ano;
  hoy = rtc.getTime("%H:%M:%S");  
  Serial.println(fecha);
  Serial.println(hoy);
  
              
  // Eliminar archivo de SD
  //    Serial.print("Entrando en modo Operacion ");
      initSDCard();    
      createDir(SD, "/Medicion");    
      listDir(SD, "/", 0);
  // std::string str = "std::string to const char*";
  //     const char *c = str.c_str();
 
  String txt = "/"+nombre+".txt";
  Serial.println(txt);
  File file = SD.open(txt);
  if(!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "Medicion.txt", "");
  }
  else {Serial.println("File already exists");  
  }
  file.close();


 #if defined(USE_SOFTWARE_SERIAL)
        // Initialize the PZEMs with Software Serial
        pzems[i] = PZEM004Tv30(pzemSWSerial, 0x10 + i);
        Serial.println("Software Serial");
 #elif defined(ESP32)
        // Initialize the PZEMs with Hardware Serial2 on RX/TX pins 16 and 17
      //  pzems[i] = PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x10 + i);

      pzems[0] = PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x86);
      pzems[1] = PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x78);
      pzems[2] = PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x76);

      
  //        pzems[3] = {
  //    PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x53),
  //    PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x35),
  //    PZEM004Tv30(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, 0x67)
  //    };
 #else
        // Initialize the PZEMs with Hardware Serial2 on the default pins

        /* Hardware Serial2 is only available on certain boards.
        *  For example the Arduino MEGA 2560
        */
        pzems[i] = PZEM004Tv30(PZEM_SERIAL, 0x10 + i);
        Serial.println("Hardware Serial");
 #endif

 save_sd_inicio();
    //} 
    // Serial.println("1 - Iniciar contador de energia");
    // Serial.println("2 - Reiniciar SD - Borrar datos");  
    // Serial.println("3 - Reiniciar PZEM");
    // Serial.println("4 - Sacar de forma segura tarjeta SD");
    
    if (touchRead(4) < 50){    
      Serial.println(touchRead(4));
      preferences.begin("my-app", false);
      //preferences.remove("nombre");
      preferences.clear();
      preferences.end();
      ESP.restart();
    }

    if(nombre == "" ){
      Serial.print("Entrando en modo configuracion ");
 
   // preferences.remove();
    flagset = true; 
    ble_start();  
                    
   }else{
      flagset = false;
   }
}
  
void armar_string_datos(){
    v1 = voltaje1;
    i1 = current;
    fp1 = factor1;
    fr1 = frecuencia1;
    pw1 = podersct;
    en1 = energiasct;
    v2 = voltaje2;
    i2 = current2;
    fp2 = factor2;
    fr2 = frecuencia2;
    pw2 = poder2;
    en2 = energia2;
    v3 = voltaje3;
    i3 = current3;
    fp3 = factor3;
    fr3 = frecuencia3;
    pw3 = poder3;
    en3 = energia3;
    // cont++;   
    // Serial.print("Dato N°: ");
    // Serial.println(cont);
    // Serial.println();
    //Concatenate all info separated by commas
    //ArduinoJSON
    dataMessage =String(fecha) + ", " + String(hoy) +  ", " + String(v1) + ", " + String(i1) + ", " + String(pw1)+ ", " + String(en1)+ ", " + String(fr1) + ", " + String(fp1) + ", " + String(v2) + ", " + String(i2) + ", " + String(pw2)+ ", " + String(en2)+ ", " + String(fr2) + ", " + String(fp2)+ ", " + String(v3) + ", " + String(i3) + ", " + String(pw3)+ ", " + String(en3)+ ", " + String(fr3) + ", " + String(fp3)+ "\r\n";
    // String(cont) + ", " + 
    // Serial.print("Saving data: ");
   // Serial.println(dataMessage);
    //Append the data to file  
   delay(500); 
 /*    char buf[1000];
    JSONVar myObject;
  myObject["ID"] = contador; // el id es el idetificador desde que sereseteo preferences, debe guardar el ultimo en preferences y seguir cone se valor 
  myObject["Tiempo"] = hoy; // relojs de tiempo real no importa la hroa que de lo que importa es ver la diferencia entre las horas para saber cuanto tiempo paso entre lecturas
 // myObject["Voltaje 1"] = estadoActualV1;
  myObject["Voltaje 1"] = voltage1;
  //myObject["Corriente 1"] = estadoActualI1;
  myObject["Corriente 1"] = current1;
  myObject["Potencia 1"] = power1,2;
  myObject["Energia 1"] = energy1,2;
  myObject["Frequencia 1"] = frequency1,2;
  myObject["Factor de poterncia 1"] = pf1,2;
  //myObject["Voltaje 2"] = estadoActualV2;
  myObject["Voltaje 2"] = voltage2;
  //myObject["Corriente 2"] = estadoActualI2;
  myObject["Corriente 2"] = current2;
  myObject["Potencia 2"] = power2,2;
  myObject["Energia 2"] = energy2,2;
  myObject["Frequencia 2"] = frequency2,2;
  myObject["Factor de poterncia 2"] = pf2,2;
  //myObject["Voltaje 3"] = estadoActualV3;
  myObject["Voltaje 3"] = voltage3;
  //myObject["Corriente 3"] = estadoActualI3;
  myObject["Corriente 3"] = current3;
  myObject["Potencia 3"] = power3,2;
  myObject["Energia 3"] = energy3,2;
  myObject["Frequencia 3"] = frequency3,2;
  myObject["Factor de poterncia 3"] = pf3,2;
  //myObject["TimeStamp"] = epochTime;  
  Serial.print("myObject.keys() = ");
  Serial.println(myObject.keys());
   String jsonString = JSON.stringify(myObject);
   Serial.println(jsonString);
   String jString = jsonString + ";";
   Serial.print("JSON.stringify(myObject) = ");
   Serial.println(jString);
   //Append the data to file
   appendFile(SD, "/Medicion.txt", jString.c_str());
   lastTime = millis();
  */

}

void save_sda(){
  setColor(0, 0, 255);
    v1 = voltaje1;
    i1 = current;
    fp1 = factor1;
    fr1 = frecuencia1;
    pw1 = podersct;
    en1 = energiasct;
    v2 = voltaje2;
    i2 = current2;
    fp2 = factor2;
    fr2 = frecuencia2;
    pw2 = poder2;
    en2 = energia2;
    v3 = voltaje3;
    i3 = current3;
    fp3 = factor3;
    fr3 = frecuencia3;
    pw3 = poder3;
    en3 = energia3;
    // cont++;   
    // Serial.print("Dato N°: ");
    // Serial.println(cont);
    // Serial.println();
  
    dataMessage =String(fecha) + ", " + String(hoy) +  ", " + String(v1) + ", " + String(i1) + ", " + String(pw1)+ ", " + String(en1)+ ", " + String(fr1) + ", " + String(fp1) + ", " + String(v2) + ", " + String(i2) + ", " + String(pw2)+ ", " + String(en2)+ ", " + String(fr2) + ", " + String(fp2)+ ", " + String(v3) + ", " + String(i3) + ", " + String(pw3)+ ", " + String(en3)+ ", " + String(fr3) + ", " + String(fp3)+ "\r\n";
    String(cont) + ", " + 
   Serial.println(dataMessage);
    //Append the data to file  
         Serial.println("Guardando datos");
    appendFile(SD, "/Medicion.txt", savedataa.c_str());
   delay(500);
   setColor(0, 0, 0);  
}

void loop() {  
 //timestamp = 16714700005424;
  bool x = false;
  int longitud;
     hoy = rtc.getTime("%H:%M:%S");
     fecha = rtc.getDate("%B/%d/%Y");
    // Serial.print("Tiempo empezanso el loop  -----> ");
    // Serial.println(hoy);
 if(flagset == true){
    setColor(255, 0, 180);
    delay(500);
    setColor(0, 0, 0);    
    delay(500);
    Serial.println("Modo configuracion");
   

 }else{
    Serial.println("-----------LOOP MODO OPERACION--------------- ");           
   //Fecha y hora
    setColor(0, 255, 0);   
    Serial.println(contador_envio);
    if(contador_envio >= inter){
      Serial.println("COMENZANDO A LLENAR EL ARRAY");
      //read_values();
      armar_string_datos();
      banderasd = false;
      contador_envio = 0;
    }else{
      contador_envio++;
      read_values();  
     // delay(1000); 
     }

  if (banderasd == false) {
    setColor(0, 255, 0);
    delay(100);
    setColor(0, 0, 0);
    //int arraysd = dataMessage.toInt();
    // Almacenar el valor en el array de datos

    dataPoints[numDataPoints] = dataMessage;
    //dataPoints[numDataPoints] = fecha [0] + ", " + hoy [1] +  "- " + v1 [2] + ", " + i1 [3] + ", " + pw1 [4] + ", " + en1 [5]+ ", " + fr1 [6] + ", " + fp1 [7] + "/ " + v2 [8] + ", " + i2 [9] + ", " + pw2 [10] + ", " + en2 [11] + ", " + fr2 [12] + ", " + fp2 [13] + "/ " + v3 [14] + ", " + i3 [15] + ", " + pw3 [16] + ", " + en3 [17] + ", " + fr3 [18] + ", " + fp3 [19]);
    
    // dataPoints[3][8]= {
    //   {fecha + "," +hoy + "," + v1 + "," + i1 + "," + pw1 + "," +en1+ ","+ fr1 + "," +fp1},
    //   {fecha + "," +hoy + "," + v1 + "," + i1 + "," + pw1 + "," +en1+ ","+ fr1 + "," +fp1},
    //   {fecha + "," +hoy + "," + v1 + "," + i1 + "," + pw1 + "," +en1+ ","+ fr1 + "," +fp1}
    // }
    // Incrementar el contador de puntos de datos
    //  visualdp = dataPoints[19];
    numDataPoints++;
    longitud = sizeof(dataPoints) / sizeof(dataPoints[0]);
    //Serial.println("Longitud del array: " + String(longitud));
    for(int i=0;i<longitud;i++){
      if(dataPoints[i] != ""){
        Serial.println(dataPoints[i]);
      }
    }
    banderasd = true;
    // Verificar si se ha alcanzado el número máximo de puntos de datos
    if (numDataPoints >= MAX_DATA_POINTS) {
      // Escribir los datos en la tarjeta SD
       Serial.println("EL ARRAY dataPoints se lleno por completo a esta listo para ser guardado y blanqueado");
       // read_values();
        //appendFile(SD, "/Medicion.txt", arraysd.c_str());
        //appendFile(SD, "/Medicion.txt", dataPoints);
        // Reiniciar el contador de puntos de datos
        numDataPoints = 0;
        banderasd = true;
        x = true;
     }
  }
    // Verificar si ha pasado suficiente tiempo desde la última escritura
  if (x == true) {
    // Escribir los datos en la tarjeta SD
    for(int i=0;i<longitud;i++){
       savedataa = dataPoints[i];
     save_sda();
      if(dataPoints[i] != ""){
        dataPoints[i] = "";
      }
    }
     Serial.println("Guardando datos");
    //appendFile(SD, "/Medicion.txt", dataMessage.c_str());
    numDataPoints = 0;
    x = false;
  }
  int contres = 0;
  contres++;
  if (contres == 500){
  ESP.restart();
  }    
 }
}    

