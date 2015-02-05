#include <SoftwareSerial.h>
#include <XBee.h>
#include <OneWire.h>
#include <I2C.h>
#include <avr/sleep.h>
#include "RTClib_DV.h"
#include <Fat16.h>
#include <Fat16util.h>
#include <MemoryFree.h>
#include <avr/pgmspace.h>

// Terminal code for Fat16 formatting
// Check disk by fomatting straight from Disk Utility and seeing greyed out name
// diskutil partitionDisk /dev/disk2 1 MBRFormat "MS-DOS FAT16" "M1" 1000M

// Clock variables
RTC_DS3231 RTC;
DateTime dStart, dAlarm, dNow;

// Sensor variables
SoftwareSerial mySerial(4, 5);
OneWire ds(3);

// XBee variables
XBee xbee = XBee(); 

uint8_t mPayload[30];
uint8_t dPayload[70];
uint8_t cPayload[5];
uint8_t flag[1];
XBeeAddress64 addr64C = XBeeAddress64(0x0013A200, 0x40866510); // Comp Address
XBeeAddress64 addr64M = XBeeAddress64(0x0013A200, 0x4086647A); // Master Address
ZBTxRequest zbTxM = ZBTxRequest(addr64C, mPayload, sizeof(mPayload));
ZBTxRequest zbTxD = ZBTxRequest(addr64C, dPayload, sizeof(dPayload));
ZBTxRequest zbTxC = ZBTxRequest(addr64M, cPayload, sizeof(cPayload));
ZBTxRequest zbTxF = ZBTxRequest(addr64C, flag, sizeof(flag));
ZBRxResponse zbRx = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

// Operating variables
float dT;
long runTime, seqCount=0;
int PRBSmultiple, seqLength, seqPeriod, seqPos=0, seqMultiple=0, state=0, CO2max=0, CO2seqMin=2001, CO2seqMax=0;
int nCO2readings, CO2readingSum, pCO2reading, CO2reading, CO2iReadingSum, pCO2iReading, CO2iReading;
int interCount;
int nodeRef, decayFlag=0;

// SD Card variables
SdCard card;
char dataFileName[13];
Fat16 dataFile;
String dataTime;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setup()  
{   
  // Local variables
  DateTime dSync; // Synchronisation time for all nodes
  long sleepPoke; // Timer to keep XBee awake
  boolean initialised = false; // Node intialisation variable
  char inChar;
  String dataN;
  
  // Set up Input/Output pins
  pinMode(9, OUTPUT);        // Status LED
  digitalWrite(9, LOW);
  pinMode(A0,OUTPUT);        // XBee Sleep Control (Active Low)
  digitalWrite(A0, LOW);  
  pinMode(2, INPUT);         // RTC Interrupt to wake from sleep
  pinMode(10, OUTPUT);       // Card Select for SD card
  pinMode(A1, INPUT);        // Battery Voltage with 1M/1M PD
  
  // Begin libraries
  I2c.begin();
  RTC.begin();
  mySerial.begin(9600);
  card.init();
  Fat16::init(&card);
   
  // Set up COZIR CO2 Sensor
  showStringSS(PSTR("K 2\r\n"));  // Set sensor to Polling
  showStringSS(PSTR("M 6\r\n"));  // Return both filtered & unfiltered CO2 Readings
  
  // Start XBee
  Serial.begin(9600);
  xbee.begin(Serial);
  flag[0]=1;
  
  // Wait for Initialisation from coordinator XBee
  while (initialised == false) {
    if (millis()-sleepPoke>3000) {
      xbee.send(zbTxF); // Transmit ready response
      digitalWrite(9, !digitalRead(9)); // Toggle status LED
      sleepPoke=millis();
    }
    
    xbee.readPacket(); // Check for instructions
    if (xbee.getResponse().isAvailable()) {
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        xbee.getResponse().getZBRxResponse(zbRx);
        // Initialisation command
        if (zbRx.getData(0) == 99) { // Check first byte for 99 identifier
          initialised = true;
          dSync =  (zbRx.getData(1) * 16777216L) + (zbRx.getData(2) * 65536L) + (zbRx.getData(3) * 256L) + zbRx.getData(4);
          RTC.adjust(dSync); // Sychronise RTC       
          dStart =  (zbRx.getData(5) * 16777216L) + (zbRx.getData(6) * 65536L) + (zbRx.getData(7) * 256L) + zbRx.getData(8);
          seqLength = (zbRx.getData(9) * 256L) + zbRx.getData(10);
          seqPeriod = ((zbRx.getData(11) * 256L) + zbRx.getData(12))*60;
          PRBSmultiple = zbRx.getData(13);
          dT = seqPeriod / (seqLength*PRBSmultiple);
        }
        // Readout command
        if (zbRx.getData(0) == 103) { // Check first byte for identifier
          uint16_t index = 0;
          dir_t dir;
          long lastFileID = 0;
          
          // read next directory entry
          while (Fat16::readDir(&dir, &index)) {
            if(dir.name[0]==82) {
              char strValue[7];
              for (uint8_t k=1; k<8; k++) {
                strValue[k-1]=dir.name[k];
              }
              if (atol(strValue)>lastFileID) lastFileID=atol(strValue);
            }   
            index++;
          }
          
          dataN = "R";
          dataN += lastFileID;
          dataN.concat(".CSV");
          dataN.toCharArray(dataFileName,sizeof(dataFileName));
          dataFile.open(dataFileName, O_READ);
          
          // Send file name in first packet
          dPayload[0]=5; // Data packet identifier
          for (uint8_t k=1; k<9; k++) dPayload[k]=dataFileName[k-1];
          dataSend();
          
          int i=1;
          int16_t c;
          while ((c = dataFile.read()) > 0) {
            dPayload[i]=c;
            i++;
            if (i>69) {
              i=1;
              dataSend();
            }
          }
          while (i<70) { // Wipe rest of packet
            dPayload[i]=0;
            i++;
          }            
          dataSend();         
          dataFile.close();
        } 
      }
    }
  }
  
  // Get node reference from SD Card file
  dataFile.open("NodeRef.dat", O_READ);
  if (dataFile.isOpen()) {
    nodeRef=dataFile.read();
    dataFile.close();
  }
    
  //Set up SD card file
  dataN = "R";
  dataN += dStart.unixtime()/60 & 0xffffff;
  dataN.concat(".CSV");
  dataN.toCharArray(dataFileName,sizeof(dataFileName));
  
  dataFile.open(dataFileName, O_CREAT | O_EXCL | O_WRITE);
  showString(PSTR("Node V3_4\r\nCO2 Sensor Details\r\n"));
  while (mySerial.available()) mySerial.read();  // Clear buffer  
  showStringSS(PSTR("*\r\n")); // Request sensor details to append to file
  delay(500);
  while (mySerial.available()) {
    inChar=mySerial.read();
    dataFile.print(inChar);
  }
  dataFile.println();
  showStringSS(PSTR("s\r\n")); // Request sensor details to append to file
  delay(500);
  while (mySerial.available()) {
    inChar=mySerial.read();
    dataFile.print(inChar);
  }
  showString(PSTR("\r\n\r\n"));
  showString(PSTR("PRBSLength,PRBSPeriod,PRBSMultiple\r\n"));
  dataFile.print(seqLength);
  showString(PSTR(","));
  dataFile.print(seqPeriod/3600);
  showString(PSTR(","));
  dataFile.println(PRBSmultiple);
  dataFile.println();
  showString(PSTR("Interrupt,Timestamp,pCO2,CO2,piCO2,iCO2,Temp,SeqCount,SeqPosition,State,RunTime,BatVoltage,VccVoltage,FreeRAM\r\n"));
  dataFile.close();
 
  digitalWrite(A0,HIGH); // Put Zigbee to sleep
  digitalWrite(9, LOW); // Turn off status LED
  RTC.initAlarm(dStart); // Initialise RTC alarm
  flag[0]=2; // Set flag to awake
  delay(1000); // Delay to let clock settle 
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void(* resetFunc) (void) = 0;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop()
{
  int tempInput, memory, voltRaw, vccRaw;
  float dataTemp, voltReading, vccReading;
  long tCheck;
  
  digitalWrite(9, HIGH);
  delay(50);
  digitalWrite(9, LOW);  
  
  // Go to Sleep & wake up with RTC alarm
  sleepNow();
  
  //Send 'Awake' flag
  digitalWrite(A0,LOW);
  digitalWrite(9, HIGH);
  delay(250);
  digitalWrite(9, LOW);
  xbee.send(zbTxF);
  delay(250);
  decaySend();
  
  // Check for instructions
  xbee.readPacket(500);
  while (xbee.getResponse().isAvailable() == true) {
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      xbee.getResponse().getZBRxResponse(zbRx);
      if (zbRx.getData(0) == 102) {        
        mySerial.print("F " + String((zbRx.getData(1) * 256L) + zbRx.getData(2)) + " " + String((zbRx.getData(3) * 256L) + zbRx.getData(4)) + "\r\n");
        delay(2000);
      }
      else if (zbRx.getData(0) == 103) {        
        mySerial.print("S " + String((zbRx.getData(1) * 256L) + zbRx.getData(2)) + "\r\n");
        delay(2000);
      }  
      else if (zbRx.getData(0) == 109) {
        resetFunc();  //call reset
        while (true);
      }
    }
    delay(50);
    xbee.readPacket(450); // Check for instructions
  }
  delay(250);
  digitalWrite(A0,HIGH);  // XBee off

  // Request temperature from 1Wire Temp Sensor
  tempReq();
   
  // Take CO2 readings    
  CO2readingSum=0;
  CO2iReadingSum=0;
  nCO2readings=0;
  
  // 5 readings before changing outputs
  tCheck=millis();
  while (mySerial.available() && millis()-tCheck<3000) mySerial.read(); // Clear buffer
  while (nCO2readings<5 && millis()-tCheck<3000) CO2Read();
  
  // Get time and set outputs
  getTimeS();
  
  // 5 readings after changing outputs
  tCheck=millis();
  while (mySerial.available() && millis()-tCheck<3000) mySerial.read(); // Clear buffer
  while (nCO2readings<10 && millis()-tCheck<3000) CO2Read();
    
  if (nCO2readings<10) {
    CO2reading = 0;
    CO2iReading = 0;
  }
  else {
    CO2reading = CO2readingSum/10; // Calculate average reading for both filtered & direct readings
    CO2iReading = CO2iReadingSum/10;
  }
  
  tempInput = tempRead(); // Receive temperature from 1Wire Temp Sensor
  if (CO2reading>CO2max) CO2max = CO2reading;
  if (CO2reading>CO2seqMax) CO2seqMax = CO2reading;
  if (CO2reading<CO2seqMin) CO2seqMin = CO2reading;
  
  // Tracer gas flags for decay test
  if (CO2reading>2000) decayFlag=1;
  if (CO2reading<1000) decayFlag=0;

  // Construct decay flag package to Master Node
  cPayload[0]=20; // Node decay flay Identifier
  cPayload[1]=nodeRef; // Node reference number
  cPayload[2]=decayFlag & 0xff ; // Decay flag
    
  memory = freeMemory();

  voltRaw = analogRead(A1);
  voltReading = (float) voltRaw/1023*3.3*2;
  vccRaw = readVcc();
  vccReading = (float) vccRaw/1000;
  
  // Add CO2 & temperature readings into XBee mPayload  
  mPayload[0] = 4; // Measurement Identifier
  mPayload[1] = seqCount >> 8 & 0xff;
  mPayload[2] = seqCount & 0xff;
  mPayload[3] = seqPeriod/60 >> 8 & 0xff;
  mPayload[4] = seqPeriod/60 & 0xff;
  mPayload[5] = seqPos >> 8 & 0xff;
  mPayload[6] = seqPos & 0xff;
  mPayload[7] = seqLength >> 8 & 0xff;
  mPayload[8] = seqLength & 0xff;
  mPayload[9] = CO2reading >> 8 & 0xff;
  mPayload[10] = CO2reading & 0xff;
  mPayload[11] = CO2iReading >> 8 & 0xff;
  mPayload[12] = CO2iReading & 0xff;
  mPayload[13] = CO2max >> 8 & 0xff;
  mPayload[14] = CO2max & 0xff;
  mPayload[15] = CO2seqMin >> 8 & 0xff;
  mPayload[16] = CO2seqMin & 0xff;
  mPayload[17] = CO2seqMax >> 8 & 0xff;
  mPayload[18] = CO2seqMax & 0xff;
  mPayload[19] = tempInput >> 8 & 0xff;
  mPayload[20] = tempInput & 0xff;
  mPayload[21] = voltRaw >> 8 & 0xff;
  mPayload[22] = voltRaw & 0xff;
  mPayload[23] = vccRaw >> 8 & 0xff;
  mPayload[24] = vccRaw & 0xff;
  
  // Transmit ZBee mPayload
  digitalWrite(A0,LOW);
  delay(250);
  xbee.send(zbTxM);
  delay(250);
  digitalWrite(A0,HIGH); // Turn XBee off
  
  // Assemble data for writing to SD card
  dataTemp = tempInput/16.0;
  
  // Write data to SD card
  dataFile.open(dataFileName, O_WRITE | O_APPEND);  
  if (dataFile.isOpen()) {
     dataFile.print(interCount);
     showString(PSTR(","));
     dataFile.print(dataTime);
     showString(PSTR(","));
     dataFile.print(pCO2reading);
     showString(PSTR(","));
     dataFile.print(CO2reading);
     showString(PSTR(","));
     dataFile.print(pCO2iReading);
     showString(PSTR(","));
     dataFile.print(CO2iReading);
     showString(PSTR(","));
     dataFile.print(dataTemp);
     showString(PSTR(","));
     dataFile.print(seqCount);
     showString(PSTR(","));
     dataFile.print(seqPos);
     showString(PSTR(","));
     dataFile.print(state);
     showString(PSTR(","));
     dataFile.print(runTime);
     showString(PSTR(","));
     dataFile.print(voltReading);
     showString(PSTR(","));
     dataFile.print(vccReading);
     showString(PSTR(","));
     dataFile.println(memory);
     dataFile.close();
  }
  
  // Set the RTC alarm based on the specified interval
  setAlarm();
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void getTimeS(void)
{
  dNow = RTC.now();
  
  // Assemble dataTime string for writing to SD Card
  dataTime = String(dNow.year(),DEC);
  dataTime += "/";
  if (dNow.month() < 10) {dataTime += "0";}
  dataTime = String(dataTime + dNow.month() + "/");
  if (dNow.day() < 10) {dataTime += "0";}
  dataTime = String(dataTime + dNow.day() + " ");
  if (dNow.hour() < 10) {dataTime += "0";}
  dataTime = String(dataTime + dNow.hour() + ":");
  if (dNow.minute() < 10) {dataTime += "0";}
  dataTime = String(dataTime + dNow.minute() + ":");
  if (dNow.second() < 10) {dataTime += "0";}
  dataTime += dNow.second();
}

void setAlarm(void)
{
  if (seqMultiple+1 >= PRBSmultiple) {
    seqMultiple=0;
    if (seqPos+1 >= seqLength) {
      seqCount=seqCount+1;
      seqPos=0;
      CO2seqMin=2001;
      CO2seqMax=0;
    }
    else seqPos=seqPos+1;
  }
  else seqMultiple=seqMultiple+1;    
  
  runTime = (seqCount * seqPeriod) + (((seqPos * PRBSmultiple) + seqMultiple) * dT) + 0.5;
  dAlarm = dStart.unixtime() + runTime;
  
  RTC.setAlarm(dAlarm);
}

void CO2Read(void)
{
  char CO2input[6];
  long tcheck=millis();
  showStringSS(PSTR("Q\r\n"));
  delay(500);
  while (mySerial.available()<15 && millis()-tcheck<1000) {}    
  if (mySerial.available()>=15) {
    while (mySerial.available()>0) {
      if (mySerial.read() == 0x5A) {
        mySerial.read();
        mySerial.read();
        for (int i=0; i < 4; i++) {
          CO2input[i] = mySerial.read(); // Read the characters into an array
        }
        if (nCO2readings==5) pCO2reading=atoi(CO2input);
        CO2readingSum += atoi(CO2input); // Covert char to integer
        mySerial.read();
        mySerial.read();
        mySerial.read();
        mySerial.read();
        for (int i=0; i < 4; i++) {
          CO2input[i] = mySerial.read(); // Read the characters into an array
        }
        if (nCO2readings==5) pCO2iReading=atoi(CO2input);
        CO2iReadingSum += atoi(CO2input); // Covert char to integer
        nCO2readings+=1;
      }
    }
  }
}

void tempReq(void)
{
  ds.reset();
  ds.skip();
  ds.write(0x44);
}

int tempRead(void) 
{
  byte data[12];
  ds.reset();
  ds.skip();
  ds.write(0xBE);       
  for (int i = 0; i < 9; i++) {data[i] = ds.read();}
  int raw = (data[1] << 8) | data[0];
  return raw;
}

void wakeUpNow()
{
  detachInterrupt(0);
  sleep_disable();
  interCount+=1;
}

void sleepNow()
{
  sleep_enable();
  attachInterrupt(0,wakeUpNow, LOW);  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_cpu();
  RTC.clearAlarm();
}

void dataSend(void)
{
  boolean success=0;
  do {
    xbee.send(zbTxD);
    if (xbee.readPacket(1000)) {
      if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
        xbee.getResponse().getZBTxStatusResponse(txStatus);
        if (txStatus.getDeliveryStatus() == SUCCESS) {
          success=1;
        } 
      }      
    }
  } while (success==0);
}

void decaySend(void)
{
  boolean success=0;
  do {
    xbee.send(zbTxC);
    if (xbee.readPacket(1000)) {
      if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
        xbee.getResponse().getZBTxStatusResponse(txStatus);
        if (txStatus.getDeliveryStatus() == SUCCESS) {
          success=1;
        } 
      }      
    }
  } while (success==0);
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void flashLed(int pin, int times, int wait) 
{   
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);
    
    if (i + 1 < times) {
      delay(wait);
    }
  }
}

void showString (PGM_P s) {
  char c;
  while ((c = pgm_read_byte(s++)) != 0)
      dataFile.print(c);
}

void showStringSS (PGM_P s) {
  char c;
  while ((c = pgm_read_byte(s++)) != 0)
      mySerial.print(c);
}
