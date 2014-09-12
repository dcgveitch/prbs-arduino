#include <I2C.h>
#include <LiquidCrystal.h>

#define NP 2 // Fan Tach Poles
#define NFans 6 // Number of Fans

int targetRPM[6]={1200, 1200, 1200, 1200, 1200, 1200};
int SR[6]={4, 4, 4, 4, 4, 4};
long interval=300000, changeTime;
int LCDPos=0;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() 
{
  Serial.begin(9600);  
  
  I2c.begin();
  I2c.timeOut(500);
  I2c.pullup(1);
 
  lcd.begin(16,2);
  lcd.print("Initialising...");
 
  pinMode(6, INPUT); // Change Channel
  digitalWrite(6, HIGH); // Pullup
  pinMode(7, INPUT); // Change RPM
  digitalWrite(7, HIGH); // Pullup 
  pinMode(8, OUTPUT); // GROUND for Buttons
  digitalWrite(8, LOW);

  I2c.write(0x20,0x01,0xbb); // Set Max Freq PWM

  for (int i = 0; i < NFans; i++) PWMSet(0x20,i,100);
  
  delay(10000);
  
  for (int i = 0; i < NFans; i++) {
    I2c.write(0x20,0x02+i,0xe8); // Configure all fans for PWM, 2s Spin Up, with Tach
    I2c.write(0x20,0x08+i,0x76); // Configure FAN Dynamics
    I2c.write(0x20,0x60+i,0x80); // Configure FAN Window
  }
 
  for (int i = 0; i < NFans; i++) {
    targetRPM[i]=1200;
    SR[LCDPos]=4;    
    RPMSet(0x20,i,targetRPM[i]);
    I2c.write(0x20,0x08+i,0x56); // SR to 4
  }
  
  changeTime=millis();
  
}

void loop() 
{
  if (digitalRead(6)==0) LCDPos++;
  if (LCDPos>=NFans) LCDPos=0;
  
  for (int i = 0; i < NFans; i++) {
    Serial.print(tachRead(0x20,i));
    Serial.print(", ");
    Serial.print(targetRPM[i]);
    if (i!=5) Serial.print(", ");
    
    lcd.setCursor(i*5-(i/3*15),i/3);
    lcd.print("      ");
    if (i==LCDPos) {
      lcd.setCursor(i*5-(i/3*15),i/3);
      lcd.print(">");
    }
    lcd.setCursor(i*5-(i/3*15)+1,i/3);
    lcd.print(tachRead(0x20,i));
  }
  Serial.println();
  
  if (digitalRead(7)==0) {
    targetRPM[LCDPos]+=100;
    if (targetRPM[LCDPos]>1500) targetRPM[LCDPos]=400;
    
    if (targetRPM[LCDPos]<700) {  
      SR[LCDPos]=2;
      I2c.write(0x20,0x08+LCDPos,0x36); // SR to 2
    }
    else if (targetRPM[LCDPos]>1200) {
      SR[LCDPos]=8;
      I2c.write(0x20,0x08+LCDPos,0x76); // SR to 8
    }
    else {
      SR[LCDPos]=4;
      I2c.write(0x20,0x08+LCDPos,0x56); // SR to 4
    }
    RPMSet(0x20,LCDPos,targetRPM[LCDPos]);
    lcd.setCursor(LCDPos*5-(LCDPos/3*15)+1,LCDPos/3);
    lcd.print("    ");
    lcd.setCursor(LCDPos*5-(LCDPos/3*15)+1,LCDPos/3);
    lcd.print(targetRPM[LCDPos]);
  }
  
  delay(1000);
}

int readByte(int reg)
{
  int msb;
  I2c.read(0x20, reg, 1);
  if(I2c.available()==1) {
    msb = I2c.receive();
  }
  int result = msb;
  return result;
}

int tachRead(int address, int fan)
{
  int reg, msb, lsb;
  long result, count;
  reg=0x18 +2*fan;
  I2c.read(address, reg, 2);
  if(I2c.available()==2) {
    msb = I2c.receive();
    lsb = I2c.receive();
    count = msb<<3 | lsb>>5;
  }
  result = (long) 60*SR[fan]*8192/count/NP;
  return result;
}

int PWMRead(int address, int fan)
{
  int msb, lsb, count, reg;
  reg=0x30 +2*fan;
  I2c.read(address, reg, 2);
  if(I2c.available()==2) {
    msb = I2c.receive();
    lsb = I2c.receive();
    count = msb<<1 | lsb>>7;
  }
  int result = count;
  return result;
}

int PWMTarRead(int address, int fan)
{
  int msb, lsb, count, reg;
  reg=0x40 +2*fan;
  I2c.read(address, reg, 2);
  if(I2c.available()==2) {
    msb = I2c.receive();
    lsb = I2c.receive();
    count = msb<<1 | lsb>>7;
  }
  int result = count;
  return result;
}
  

void PWMSet(int address, int fan, long output)
{
  int  reg;
  long value;
  byte valByte[2];
  reg=0x40 +2*fan;
  value=output*511/100;
  valByte[0]=value>>1 & 0xff;
  valByte[1]=value<<7 & 0xff;
  I2c.write(address, reg, valByte, 2);
}

int RPMTarRead(int address, int fan)
{
  int msb, lsb, count, reg;
  reg=0x50 +2*fan;
  I2c.read(address, reg, 2);
  if(I2c.available()==2) {
    msb = I2c.receive();
    lsb = I2c.receive();
    count = msb<<3 | lsb>>5;
  }
  int result = count;
  return result;
}

void RPMSet(int address, int fan, long rpm)
{
  int  reg;
  byte valByte[2];
  long output;
  reg=0x50 +2*fan;
  output = (long) 60*SR[fan]*8192/rpm/NP; 
  valByte[0]=output>>3 & 0xff;
  valByte[1]=output<<5 & 0xff;
  I2c.write(address, reg, valByte, 2);
}

