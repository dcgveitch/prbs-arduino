#include <XBee.h>
#include <I2C.h>
#include <AccelStepper_DV.h>
#include "RTClib_DV.h"
#include <Fat16.h>
#include <Fat16util.h>
#include <avr/pgmspace.h>

#define NP 2 // Fan Tach Poles
#define NFans 8 // Number of Fans

// Terminal code for Fat16 formatting
// Check disk by fomatting straight from Disk Utility and seeing greyed out name
// diskutil partitionDisk /dev/disk2 1 MBRFormat "MS-DOS FAT16" "M1" 1000M

// Clock variables
RTC_DS3231 RTC;
DateTime dStart, dAlarm, dNow;

// XBee variables
XBee xbee = XBee(); 

uint8_t mPayload[32];
uint8_t dPayload[70];
uint8_t fPayload[14];
XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x40866510); // Comp Address
ZBTxRequest zbTxM = ZBTxRequest(addr64, mPayload, sizeof(mPayload));
ZBTxRequest zbTxD = ZBTxRequest(addr64, dPayload, sizeof(dPayload));
ZBTxRequest zbTxF = ZBTxRequest(addr64, fPayload, sizeof(fPayload));
ZBRxResponse zbRx = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

// Operating variables
float dT;
long runTime, seqCount=0;
int PRBS, PRBSmultiple, seqLength, seqPeriod, seqPos=0, seqMultiple=0, nZones;
int interCount;
boolean processFlag;
int voltRaw, vccRaw;
float voltReading, vccReading;

// SD Card variables
SdCard card;
char dataFileName[13];
char prbsFileName[13];
Fat16 dataFile;
String dataTime, prbsData;

// Fan variables
int targetRPM[NFans], actualRPM[NFans], prevRPM[NFans], nextRPM[NFans], SR[NFans];
long prevFanTime, nextFanTime, interTime, diffTime, fanFilePos;

// Motor variables
AccelStepper z1(AccelStepper::DRIVER, 8, 5);
AccelStepper z2(AccelStepper::DRIVER, 7, 5); 
int z1state=0, z2state=0;
int z1speed, z2speed;
long z1pos, z2pos;
boolean z1run, z2run, warmup;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setup()  
{ 
  //----- INITIALISE VARIABLES & HARDWRRE
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
  
  // Begin libraries
  I2c.begin();
  RTC.begin();
  card.init();
  Fat16::init(&card);
  
  // Set up fan speed controllers and warm up whilst waiting to be initialised  
  I2c.write(0x20,0x01,0xbb); // Set Max Freq PWM
  I2c.write(0x23,0x01,0xbb); // Set Max Freq PWM
  for (int i = 0; i < NFans; i++) {
    int address = 0x20+i/6*3;
    I2c.write(address,0x02+(i%6),0xe8); // Configure all fans for PWM, 2s Spin Up, with Tach
    I2c.write(address,0x08+(i%6),0x76); // Configure FAN Dynamics
    I2c.write(address,0x60+(i%6),0x40); // Configure FAN Window
    targetRPM[i]=2500;   
    tachWrite(i,targetRPM[i]);       
  }  
  
  // Start XBee
  Serial.begin(9600);
  xbee.begin(Serial);
  fPayload[0]=0;
  
  //-----WAIT FOR INITIALISATION PACKET
  // Wait for Initialisation from coordinator XBee
  while (initialised == false) {
    if (millis()-sleepPoke>3000) {
      fPayload[1]=NFans;
      for (int i = 0; i < NFans; i++) {
        fPayload[i+2] = tachRead(i)/10;
      }
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
          nZones= zbRx.getData(14);
          dT = seqPeriod / (seqLength*PRBSmultiple);
          z1speed = zbRx.getData(15)*10;
          z2speed = zbRx.getData(16)*10;
          warmup = zbRx.getData(17);
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
  digitalWrite(9, LOW); // Turn off status LED 
  
  //---- SET UP FILES & SD CARD  
  //Assemble PRBS filename
  prbsData = "zpbs"; 
  prbsData += seqLength;
  prbsData += "z";
  prbsData += nZones;
  prbsData += ".dat";
  prbsData.toCharArray(prbsFileName,sizeof(prbsFileName));
  
  //Set up SD card file
  dataN = "R";
  dataN += dStart.unixtime()/60 & 0xffffff;
  dataN.concat(".CSV");
  dataN.toCharArray(dataFileName,sizeof(dataFileName));
  
  dataFile.open(dataFileName, O_CREAT | O_EXCL | O_WRITE);
  showString(PSTR("NodeMaster V???\r\n"));
  showString(PSTR("PRBSLength,PRBSPeriod,PRBSMultiple,nZones\r\n"));
  dataFile.print(seqLength);
  showString(PSTR(","));
  dataFile.print(seqPeriod/3600);
  showString(PSTR(","));
  dataFile.print(PRBSmultiple);
  showString(PSTR(","));
  dataFile.println(nZones);
  dataFile.println();
  showString(PSTR("Interrupt,Timestamp,SeqCount,SeqPosition,RunTime,z1State,z2State,z1Speed,z2Speed,z1Pos,z2Pos,PrevFanTime,NextFanTime,Tach,Target\r\n"));
  dataFile.close();
  
  //-----START FANS & GAS RELEASE
  // Set initial fan speed
  dataFile.open("zfnspeed.dat", O_READ);
  if (dataFile.isOpen()) {
    prevFanTime = 0;
    dataFile.seekSet(4);
    for (int i = 0; i < NFans; i++) {
      prevRPM[i]=dataFile.read()*256L;
      prevRPM[i]=prevRPM[i]+dataFile.read();
      if (warmup) targetRPM[i] = 2000;
      else targetRPM[i] = prevRPM[i];
      tachWrite(i,targetRPM[i]);
    }
    
    fanFilePos = (NFans*2)+4;
    nextFanTime=dataFile.read();
    nextFanTime=nextFanTime+(dataFile.read()*256L);
    nextFanTime=nextFanTime+(dataFile.read()*65536L);
    nextFanTime=nextFanTime+(dataFile.read()*16777216L);
    for (int i = 0; i < NFans; i++) {
      nextRPM[i]=dataFile.read()*256L;
      nextRPM[i]=nextRPM[i]+dataFile.read();
    }
    dataFile.close();
  }
  else {
    for (int i = 0; i < 3; i++) { 
      digitalWrite(9, HIGH);
      delay(100);
      digitalWrite(9, LOW);
      delay(100);
    }
  }
  
  // Setup motor parameters
  z1.setMaxSpeed(z1speed);
  z2.setMaxSpeed(z2speed);
  z1.setAcceleration(z1speed);
  z2.setAcceleration(z2speed);
  z1.setSpeed(0);
  z2.setSpeed(0);
   
  RTC.initAlarm(dStart); // Initialise RTC alarm
  processFlag=true;
  fPayload[0]=2; // Set flag to awake
  attachInterrupt(0, setProcessFlag, LOW);
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void(* resetFunc) (void) = 0;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop()
{
  while (processFlag) {
    z1.runSpeed();
    z2.runSpeed();
  }
  
  do 
  {
    z1run=z1.runDeAcc();
    z2run=z2.runDeAcc();
  } while (z1run || z2run);
  
  z1pos=z1.currentPosition();
  z2pos=z2.currentPosition();
  z1.setCurrentPosition(0);
  z2.setCurrentPosition(0);  
  
  RTC.clearAlarm();
  digitalWrite(9, HIGH);
  
  xbee.send(zbTxF);
  
  //-----CHECK FOR INSTRUCTIONS
  xbee.readPacket(5);
  while (xbee.getResponse().isAvailable() == true) {
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      xbee.getResponse().getZBRxResponse(zbRx);
      if (zbRx.getData(0) == 110) { // Change motor speeds
        z1speed=zbRx.getData(1)*10;
        z2speed=zbRx.getData(2)*10;
        z1.setMaxSpeed(z1speed);
        z2.setMaxSpeed(z2speed);
        z1.setAcceleration(z1speed);
        z2.setAcceleration(z2speed);
      }
      else if (zbRx.getData(0) == 109) {
        resetFunc();  //call reset
        while (true);
      }
    }
    xbee.readPacket(5); // Check for instructions
  }
  
  //----- SET PUMPS
  // OPTION 1 - Read prbs states from SD Card
  dataFile.open(prbsFileName, O_READ);
  if (dataFile.isOpen()) {
    if (dataFile.seekSet(seqPos*nZones)) {
      z1state = dataFile.read();
      if (nZones>1) z2state = dataFile.read();
    }
    dataFile.close();
  }
  else {
    for (int i = 0; i < 5; i++) { 
      digitalWrite(9, HIGH);
      delay(100);
      digitalWrite(9, LOW);
      delay(100);
    }
  }
  
  if (z1state | warmup) z1.moveOn();
  else z1.moveOff();
  if (z2state | warmup) z2.moveOn();
  else z2.moveOff();

  //----- CALCULATE FAN SPEEDS
  getTimeS(); 
  // Get correct time and speed points for interpolation
  
  while ((dNow.unixtime()-dStart.unixtime())>nextFanTime) {
    fanFilePos = fanFilePos+(NFans*2)+4;
    dataFile.open("zfnspeed.dat", O_READ);
    if (dataFile.isOpen()) {
      dataFile.seekSet(fanFilePos);
      prevFanTime=nextFanTime;
      nextFanTime=dataFile.read();
      nextFanTime=nextFanTime+(dataFile.read()*256L);
      nextFanTime=nextFanTime+(dataFile.read()*65536L);
      nextFanTime=nextFanTime+(dataFile.read()*16777216L);
      for (int i = 0; i < NFans; i++) {
        prevRPM[i]=nextRPM[i];
        nextRPM[i]=dataFile.read()*256L;
        nextRPM[i]=nextRPM[i]+dataFile.read();
      }
    }
    dataFile.close();
  }
  interTime=(dNow.unixtime()-dStart.unixtime())-prevFanTime;
  diffTime=nextFanTime-prevFanTime;
  for (int i = 0; i < NFans; i++) {    
    float target = prevRPM[i]+(nextRPM[i]-prevRPM[i])*(float(interTime)/diffTime);
    if (warmup) targetRPM[i] = 2000;
    else targetRPM[i] = (int) target;
    actualRPM[i] = tachRead(i);
  }
   
  voltRaw = analogRead(A1);
  voltReading = (float) voltRaw/1023*3.3*2;
  vccRaw = readVcc();
  vccReading = (float) vccRaw/1000;
  
  //-----MAKE & SEND ZIGBEE PAYLOAD
  mPayload[0] = 3; // Master Identifier
  mPayload[1] = seqCount >> 8 & 0xff;
  mPayload[2] = seqCount & 0xff;
  mPayload[3] = seqPeriod/60 >> 8 & 0xff;
  mPayload[4] = seqPeriod/60 & 0xff;
  mPayload[5] = seqPos >> 8 & 0xff;
  mPayload[6] = seqPos & 0xff;
  mPayload[7] = seqLength >> 8 & 0xff;
  mPayload[8] = seqLength & 0xff;
  mPayload[9] = z1state;
  mPayload[10] = z2state;
  mPayload[12] = z1speed/10;
  mPayload[13] = z2speed/10;
  mPayload[15] = NFans;
  for (int i = 0; i < NFans; i++) {
    mPayload[16+i] = actualRPM[i]/10;
  }
  mPayload[28] = voltRaw >> 8 & 0xff;
  mPayload[29] = voltRaw & 0xff;
  mPayload[30] = vccRaw >> 8 & 0xff;
  mPayload[31] = vccRaw & 0xff;
  
  
  // Transmit ZBee mPayload
  xbee.send(zbTxM);
  
  //-----SAVE DATA TO SD CARD
  dataFile.open(dataFileName, O_WRITE | O_APPEND);  
  if (dataFile.isOpen()) {
     dataFile.print(interCount);
     showString(PSTR(","));
     dataFile.print(dataTime);
     showString(PSTR(","));
     dataFile.print(seqCount);
     showString(PSTR(","));
     dataFile.print(seqPos);
     showString(PSTR(","));
     dataFile.print(runTime);
     showString(PSTR(","));
     dataFile.print(z1state);
     showString(PSTR(","));
     dataFile.print(z2state);
     showString(PSTR(","));
     dataFile.print(z1speed);
     showString(PSTR(","));
     dataFile.print(z2speed);
     showString(PSTR(","));
     dataFile.print(z1pos);
     showString(PSTR(","));
     dataFile.print(z2pos);
     showString(PSTR(","));
     dataFile.print(prevFanTime);
     showString(PSTR(","));
     dataFile.print(nextFanTime);
     for (int i = 0; i < NFans; i++) {
       showString(PSTR(","));
       dataFile.print(tachRead(i));
       showString(PSTR(","));
       dataFile.print(targetRPM[i]);
     }
     dataFile.println();
     dataFile.close();
  }
  
  //-----WRITE FAN SPEEDS
  for (int i = 0; i < NFans; i++) {
    tachWrite(i,targetRPM[i]);
  }

  setAlarm();
  
  do 
  {
    z1run=z1.runAcc();
    z2run=z2.runAcc();
  } while (z1run || z2run);
  
  digitalWrite(9, LOW);
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
    }
    else seqPos=seqPos+1;
  }
  else seqMultiple=seqMultiple+1;    
  
  runTime = (seqCount * seqPeriod) + (((seqPos * PRBSmultiple) + seqMultiple) * dT) + 3.5;
  dAlarm = dStart.unixtime() + runTime;
  
  RTC.setAlarm(dAlarm);
  processFlag=true;
  attachInterrupt(0, setProcessFlag, LOW);
}

void setProcessFlag(void) {
  detachInterrupt(0);
  interCount+=1;
  processFlag=false;
}


void tachWrite(int fan, long rpm)
{
  byte valByte[2];
  long output;
  int address = 0x20+fan/6*3;
  if (rpm<400) {  
    SR[fan]=2;
    I2c.write(address,0x08+(fan%6),0x36); // SR to 1
  }
  else if (rpm<750) {  
    SR[fan]=2;
    I2c.write(address,0x08+(fan%6),0x36); // SR to 2
  }
  else if (rpm<1500) {
    SR[fan]=4;
    I2c.write(address,0x08+(fan%6),0x56); // SR to 4
  }
  else {
    SR[fan]=8;
    I2c.write(address,0x08+(fan%6),0x76); // SR to 8
  }
  output = (long) 60*SR[fan]*8192/rpm/NP; 
  valByte[0]=output>>3 & 0xff;
  valByte[1]=output<<5 & 0xff;
  I2c.write(address, 0x50+2*(fan%6), valByte, 2);
}

void fanStop(int fan)
{
  byte valByte[2];
  long output;
  int address = 0x20+fan/6*3;
  output = 2047;
  valByte[0]=output>>3 & 0xff;
  valByte[1]=output<<5 & 0xff;
  I2c.write(address, 0x50+2*(fan%6), valByte, 2);
}

void fanRun(int fan)
{
  byte valByte[2];
  long output;
  int address = 0x20+fan/6*3;
  output = 0;
  valByte[0]=output>>3 & 0xff;
  valByte[1]=output<<5 & 0xff;
  I2c.write(address, 0x50+2*(fan%6), valByte, 2);
}
    
int tachRead(int fan)
{
  int msb, lsb;
  long result, count;
  int address = 0x20+fan/6*3;
  I2c.read(address, 0x18+2*(fan%6), 2);
  if(I2c.available()==2) {
    msb = I2c.receive();
    lsb = I2c.receive();
    count = msb<<3 | lsb>>5;
  }
  result = (long) 60*SR[fan]*8192/count/NP;
  return result;
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
