#include <XBee.h>
#include <I2C.h>
#include <AccelStepper.h>
#include "RTClib_DV.h"
#include <Fat16.h>
#include <Fat16util.h>
#include <avr/pgmspace.h>

#define NP 2 // Fan Tach Poles
#define NFans 12 // Number of Fans

// Terminal code for Fat16 formatting
// Check disk by fomatting straight from Disk Utility and seeing greyed out name
// diskutil partitionDisk /dev/disk2 1 MBRFormat "MS-DOS FAT16" "M1" 1000M

// Clock variables
RTC_DS3231 RTC;
DateTime dStart, dAlarm, dNow;

// XBee variables
XBee xbee = XBee(); 

uint8_t mPayload[30];
uint8_t dPayload[70];
uint8_t flag[1];
XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x00000000); // Coordinator Address
ZBTxRequest zbTxM = ZBTxRequest(addr64, mPayload, sizeof(mPayload));
ZBTxRequest zbTxD = ZBTxRequest(addr64, dPayload, sizeof(dPayload));
ZBTxRequest zbTxF = ZBTxRequest(addr64, flag, sizeof(flag));
ZBRxResponse zbRx = ZBRxResponse();

// Operating variables
float dT;
long runTime, seqCount=0;
int PRBS, PRBSmultiple, seqLength, seqPeriod, seqPos=0, seqMultiple=0, state=0, nZones;
int interCount;
boolean processFlag;

// SD Card variables
SdCard card;
char dataFileName[13];
char prbsFileName[13];
Fat16 dataFile;
String dataTime, prbsData;

// Fan variables
int targetRPM[NFans], prevRPM[NFans], nextRPM[NFans];
long prevFanTime, nextFanTime, interTime, diffTime;
int SR[NFans], fanFilePos;

// Motor variables
AccelStepper z1(AccelStepper::DRIVER, 8, 6);
AccelStepper z2(AccelStepper::DRIVER, 7, 5); 
AccelStepper z3(AccelStepper::DRIVER, 8, 6);
int z1speed=700, z2speed=420, z3speed=0;

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
    int address = 0x20+(i-1)/6*3;
    I2c.write(address,0x02+i%6,0xe8); // Configure all fans for PWM, 2s Spin Up, with Tach
    I2c.write(address,0x08+i%6,0x76); // Configure FAN Dynamics
    I2c.write(address,0x60+i%6,0x80); // Configure FAN Window
    targetRPM[i]=1200;   
    tachWrite(address,i%6,targetRPM[i]);      
  }    
  
  // Start XBee
  Serial.begin(9600);
  xbee.begin(Serial);
  flag[0]=1;
  
  //-----WAIT FOR INITIALISATION PACKET
  // Wait for Initialisation from coordinator XBee
  while (initialised == false) {
    if (millis()-sleepPoke>1000) {
      xbee.send(zbTxF); // Transmit ready response
      digitalWrite(9, !digitalRead(9)); // Toggle status LED
      sleepPoke=millis();
    }
    
    xbee.readPacket(); // Check for instructions
    if (xbee.getResponse().isAvailable()) {
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        xbee.getResponse().getZBRxResponse(zbRx);
        // Initialisation command
        if (zbRx.getData(0) == 99 && zbRx.getData(1) == 99 && zbRx.getData(2) == 99) { // Check first 3 bytes for 99,99,99 identifier
          initialised = true;
          dSync =  (zbRx.getData(3) * 16777216L) + (zbRx.getData(4) * 65536L) + (zbRx.getData(5) * 256L) + zbRx.getData(6);
          RTC.adjust(dSync); // Sychronise RTC       
          dStart =  (zbRx.getData(7) * 16777216L) + (zbRx.getData(8) * 65536L) + (zbRx.getData(9) * 256L) + zbRx.getData(10);
          seqLength = (zbRx.getData(11) * 256L) + zbRx.getData(12);
          seqPeriod = ((zbRx.getData(13) * 256L) + zbRx.getData(14))*60;
          PRBSmultiple = zbRx.getData(15);
          nZones= zbRx.getData(16);
          dT = seqPeriod / (seqLength*PRBSmultiple);
        }
        // Readout command
        if (zbRx.getData(0) == 103) { // Check first byte for identifier
          uint16_t index = 0;
          int16_t c;
          dir_t dir;
          int i=1;
         
          while (Fat16::readDir(&dir, &index)) index++;
          dataFile.open(index-1, O_READ);
          dPayload[0]=4; // Data packet identifier
          while ((c = dataFile.read()) > 0) {
            dPayload[i]=c;
            i++;
            if (i>69) {
              i=1;
              xbee.send(zbTxD);
            }
          }
          while (i<70) { // Wipe rest of packet
            dPayload[i]=0;
            i++;
          }            
          xbee.send(zbTxD);         
          dataFile.close();
        } 
      }
    }
  }
  digitalWrite(9, LOW); // Turn off status LED 
  
  //---- SET UP FILES & SD CARD  
  //Assemble PRBS filename
  prbsData = "prbs_"; 
  prbsData += seqLength;
  prbsData += "_";
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
  showString(PSTR("Interrupt,Timestamp,SeqCount,SeqPosition,State,RunTime,BatVoltage,VccVoltage,FreeRAM,FanVar\r\n"));
  dataFile.close();
  
  //-----START FANS & GAS RELEASE
  // Set initial fan speed
  dataFile.open("fanspeed.dat", O_READ);
  if (dataFile.isOpen()) {
    prevFanTime = 0;
    fanFilePos = NFans+4;
    dataFile.seekSet(4);
    for (int i = 0; i < NFans; i++) {
      int address = 0x20+(i-1)/6*3;
      prevRPM[i]=dataFile.read()*10;
      targetRPM[i]=prevRPM[i];
      tachWrite(address,i%6,targetRPM[i]); 
    }
    nextFanTime=2^24*dataFile.read();
    nextFanTime=nextFanTime+2^16*dataFile.read();
    nextFanTime=nextFanTime+2^8*dataFile.read();
    nextFanTime=nextFanTime+dataFile.read();
    for (int i = 0; i < NFans; i++) {
      nextRPM[i]=dataFile.read()*10;
    }
    dataFile.close();
  }
  
  // Setup motor parameters
  z1.setMaxSpeed(z1speed);
  z2.setMaxSpeed(z2speed);
  z3.setMaxSpeed(z3speed);
  z1.setAcceleration(1400);
  z2.setAcceleration(1400);
  z3.setAcceleration(1400);
   
  processFlag=false;
  RTC.initAlarm(dStart); // Initialise RTC alarm
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void(* resetFunc) (void) = 0;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop()
{
  if (processFlag) {
    int z1state, z2state, z3state;
    
    //-----CHECK FOR INSTRUCTIONS
    xbee.readPacket(500);
    while (xbee.getResponse().isAvailable() == true) {
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        xbee.getResponse().getZBRxResponse(zbRx);
        if (zbRx.getData(0) == 101) {
          if (PRBS==0) state=!state; // Manual Remote Tracer Control
        }
        else if (zbRx.getData(0) == 110) { // Change motor speeds
          z1.setMaxSpeed(zbRx.getData(1)*10);
          z2.setMaxSpeed(zbRx.getData(2)*10);
          z3.setMaxSpeed(zbRx.getData(3)*10);
        }
        else if (zbRx.getData(0) == 109) {
          resetFunc();  //call reset
          while (true);
        }
      }
      delay(50);
      xbee.readPacket(450); // Check for instructions
    }
    
    //----- SET PUMPS
    // Read prbs states from SD Card
    dataFile.open(prbsFileName, O_READ);
    if (dataFile.isOpen()) {
      if (dataFile.seekSet(seqPos*3)) {
        z1state = dataFile.read();
        z2state = dataFile.read();
        z3state = dataFile.read();
      }
      dataFile.close();
    }
    // Run or stop pumps
    if (z1state==1) z1.move(1000000);
    else z1.stop();
    if (z2state==1) z2.move(1000000);
    else z2.stop();
    if (z3state==1) z3.move(1000000);
    else z3.stop();
    
    //----- SET FAN SPEEDS
    getTimeS(); 
    // Get correct time and speed points for interpolation
    while ((dNow.unixtime()-dStart.unixtime())>nextFanTime) {
      fanFilePos = fanFilePos+NFans+4;
      dataFile.open("fanspeed.dat", O_READ);
      if (dataFile.isOpen()) {
        dataFile.seekSet(fanFilePos);
        prevFanTime=nextFanTime;
        nextFanTime=2^24*dataFile.read();
        nextFanTime=nextFanTime+2^16*dataFile.read();
        nextFanTime=nextFanTime+2^8*dataFile.read();
        nextFanTime=nextFanTime+dataFile.read();
        for (int i = 0; i < NFans; i++) {
          prevRPM[i]=nextRPM[i];
          nextRPM[i]=dataFile.read()*10;
        }
      }
      dataFile.close();
    }
    interTime=(dNow.unixtime()-dStart.unixtime())-prevFanTime;
    diffTime=nextFanTime-prevFanTime;
    for (int i = 0; i < NFans; i++) {
      int address = 0x20+(i-1)/6*3;
      float target = prevRPM[i]+(nextRPM[i]-prevRPM[i])*(interTime/diffTime);
      targetRPM[i] = (int) target;
      tachWrite(address,i%6,targetRPM[i]);
    }
    
    //-----MAKE & SEND ZIGBEE PAYLOAD
    mPayload[0] = 3; // Measurement Identifier
    mPayload[7] = seqCount >> 8 & 0xff;
    mPayload[8] = seqCount & 0xff;
    mPayload[9] = seqPeriod/60 >> 8 & 0xff;
    mPayload[10] = seqPeriod/60 & 0xff;
    mPayload[11] = seqPos >> 8 & 0xff;
    mPayload[12] = seqPos & 0xff;
    mPayload[13] = seqLength >> 8 & 0xff;
    mPayload[14] = seqLength & 0xff;
    
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
       dataFile.print(state);
       showString(PSTR(","));
       dataFile.print(runTime);
       showString(PSTR(","));
       dataFile.print(prevFanTime);
       showString(PSTR(","));
       dataFile.print(nextFanTime);
       showString(PSTR(","));
       dataFile.print(interTime);
       showString(PSTR(","));
       dataFile.print(diffTime);
       for (int i = 6; i < 8; i++) {
         int address = 0x20+(i-1)/6*3;
         showString(PSTR(","));
         dataFile.print(tachRead(address,i%6));
         showString(PSTR(","));
         dataFile.print(targetRPM[i]);
         showString(PSTR(","));
         dataFile.print(prevRPM[i]);
         showString(PSTR(","));
         dataFile.print(nextRPM[i]);
       }
       dataFile.println();
       dataFile.close();
    }
    
    processFlag=false;
    setAlarm();
  }  
  
  z1.run();
  z2.run();
  z3.run();
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
  
  runTime = (seqCount * seqPeriod) + (((seqPos * PRBSmultiple) + seqMultiple) * dT) + 0.5;
  dAlarm = dStart.unixtime() + runTime;
  
  RTC.setAlarm(dAlarm);
}

void tachWrite(int address, int fan, long rpm)
{
  byte valByte[2];
  long output;
  if (rpm<700) {  
    SR[fan]=2;
    I2c.write(address,0x08+fan%6,0x36); // SR to 2
  }
  else if (rpm>1200) {
    SR[fan]=8;
    I2c.write(address,0x08+fan%6,0x76); // SR to 8
  }
  else {
    SR[fan]=4;
    I2c.write(address,0x08+fan%6,0x56); // SR to 4
  }
  output = (long) 60*SR[fan]*8192/rpm/NP; 
  valByte[0]=output>>3 & 0xff;
  valByte[1]=output<<5 & 0xff;
  I2c.write(address, 0x50+2*fan, valByte, 2);
}
    
int tachRead(int address, int fan)
{
  int msb, lsb;
  long result, count;
  I2c.read(address, 0x18+2*fan, 2);
  if(I2c.available()==2) {
    msb = I2c.receive();
    lsb = I2c.receive();
    count = msb<<3 | lsb>>5;
  }
  result = (long) 60*SR[fan]*8192/count/NP;
  return result;
}

void showString (PGM_P s) {
  char c;
  while ((c = pgm_read_byte(s++)) != 0)
      dataFile.print(c);
}
