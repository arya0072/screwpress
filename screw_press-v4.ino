#include <Wire.h>
#include <ADS1X15.h>
#include <ModbusRTU.h>
#include <EEPROM.h>

#define RXD2 16
#define TXD2 17
#define DIR 5   // pin untuk arah modul ttl-rs485  non automatic
#define REGN 2
#define SLAVE_ID 1

ModbusRTU mb;

#define pbload 33
#define pbunload 32
#define switch_load 34
#define switch_unload 35
#define manauto 25
#define dacpin 26

#define relay_load 18
#define relay_unload 19
#define relay3 4

#define en485 5


ADS1115 ADS(0x48);
const float factor=100;
const float multifier=0.0575;


  unsigned long lastMillis = 0;


int pb1, pb2, pb3, pb4, pb5;
uint16_t au16data[20];
int16_t val_01;
float volts_01;
float power;
float voltage_rms;
float current_rms;

float total_volt = 0;
const int freq = 1000;
const int resolution = 16;
unsigned int nilai_pwm;
unsigned int input_pwm;
unsigned int mapped_pwm;

  float volt_diferencial;
  float current;

//============================================
unsigned int pressure;
unsigned int ampere;
unsigned int voltage;

unsigned int mode_kerja;

struct strConfig 
{ 
  unsigned int setpoint_pressure;
  unsigned int max_ampere;
  unsigned int ampere_threshold;
  unsigned int max_pressure;
}   config;

unsigned int temp_setpoint_pressure;
  unsigned int temp_max_ampere;
  unsigned int temp_ampere_threshold;
  unsigned int temp_max_pressure;

//===============================================

float f_setpoint_pressure;
float f_max_ampere;
float f_ampere_threshold;
float f_max_pressure;

float f1,f2,f3,f4,f5;
float m,c;

//======================================================================
void EEPROMWriteint(unsigned int address, unsigned int value)
      {
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      
      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
     }
unsigned int EEPROMReadint(unsigned int address)
      {
      //Read the 4 bytes from the eeprom memory.
      unsigned int four = EEPROM.read(address);
      unsigned int three = EEPROM.read(address + 1);
     
      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF);
}
//=======================================================


void WriteConfig()
{
  //Serial.println("Writing Config");
  EEPROM.write(0,'C');
  EEPROM.write(1,'F');
  EEPROM.write(2,'G');
  EEPROMWriteint(100, config.setpoint_pressure);
  EEPROMWriteint(102, config.max_ampere);
  EEPROMWriteint(104, config.ampere_threshold);
  EEPROMWriteint(106, config.max_pressure);
   EEPROM.commit();

}

boolean ReadConfig()
{
  if (EEPROM.read(0) == 'C' && EEPROM.read(1) == 'F'  && EEPROM.read(2) == 'G' )
  {    
    config.setpoint_pressure=EEPROMReadint(100); 
    config.max_ampere=EEPROMReadint(102); 
    config.ampere_threshold=EEPROMReadint(104); 
    config.max_pressure=EEPROMReadint(106); 
    return true;
  }
  else
  {
    return false;
  }
}


void DefaultConfig()
{
     config.setpoint_pressure=20; 
    config.max_ampere=100; 
    config.ampere_threshold=25; 
    config.max_pressure=30;  
    WriteConfig();
}






void init_devices()
{
  Serial.begin(9600);

  pinMode(pbload, INPUT_PULLUP);
  pinMode(pbunload, INPUT_PULLUP);
  pinMode(switch_load, INPUT_PULLUP);
  pinMode(switch_unload, INPUT_PULLUP);
  pinMode(manauto, INPUT_PULLUP);
  
  pinMode(relay_load, OUTPUT);
  pinMode(relay_unload, OUTPUT);
  pinMode(relay3, OUTPUT);
  
  pinMode(en485, OUTPUT);

  digitalWrite(relay_load, LOW);
  digitalWrite(relay_unload, LOW);
  digitalWrite(relay3, LOW);
}
//==============================================================================
int getMaxValue()
{
  int sensorValue;
  int sensorMax = 0;
  uint32_t start_time = millis();
  while((millis() - start_time) < 200)
  {
    sensorValue = ADS.readADC(0);
    float f = ADS.toVoltage(1);
    if (sensorValue > sensorMax) 
    {
      sensorMax = sensorValue;
    }
  }
  return sensorMax;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void baca_arus()
{
  int sensor_max = getMaxValue();
  current = mapFloat(sensor_max, 13265 , 13772, 0, 6.8) ;
  if (current < 0.0)
  {
    current = 0.0;
  }
  ampere = current*100;
  
  Serial.print("current: ");
  Serial.println(current);
}

void hitung_pressure()
{
  f_setpoint_pressure=config.setpoint_pressure;
  f_max_ampere=config.max_ampere;
  f_ampere_threshold=config.ampere_threshold;
  m=(0-f_setpoint_pressure)/(f_max_ampere-f_ampere_threshold);
  c=-(m*f_ampere_threshold)+ f_setpoint_pressure; 
  f4=(m*current)+c;
  pressure=f4;  
  
  if(pressure<0)pressure=0;
  //Serial.print("pressure=");
  //Serial.println(f4);
  
  //Serial.print("max pressure=");
  //Serial.println(max_pressure);
  
}

void hitung_voltage()
{
  if(pressure>config.max_pressure)pressure=config.max_pressure;
  voltage=map(pressure,0,config.max_pressure,0,1000);
  if(voltage> 1000) voltage = 1000;
  mapped_pwm = map(voltage,0,1000,0,65500);
  ledcWrite(0, mapped_pwm);
}

void relay_enable()
{
  digitalWrite(relay3,LOW);
}


void relay_disble()
{
  digitalWrite(relay3,HIGH);
}
//=====================================================
void main_program()
{
   baca_arus();
   mb.Hreg(0x1,pressure);  //register 40001
   mb.Hreg(0x2,ampere);  //register 40002
   mb.Hreg(0x3,voltage);  //register 40003
       
   config.setpoint_pressure=mb.Hreg(0x4);  //register 40004
   config.max_ampere=mb.Hreg(0x5);  //register 40004
   config.ampere_threshold=mb.Hreg(0x6);  //register 40004
   config.max_pressure=mb.Hreg(0x7);  //register 40004
  mb.Hreg(0x8,mode_kerja);  //register 40003

  if(temp_setpoint_pressure!=config.setpoint_pressure)
  {
    temp_setpoint_pressure=config.setpoint_pressure;
    WriteConfig();
  }
  if(temp_max_ampere!=config.max_ampere)
  {
    temp_max_ampere=config.max_ampere;
    WriteConfig();
  }
  if(temp_ampere_threshold!=config.ampere_threshold)
  {
    temp_ampere_threshold=config.ampere_threshold;
    WriteConfig();
  }
  if(temp_max_pressure!=config.max_pressure)
  {
    temp_max_pressure=config.max_pressure;
    WriteConfig();
  }
  
  
/*
  Serial.print("p:");
  Serial.println(pressure);
  Serial.print("a:");
  Serial.println(ampere);
  Serial.print("v:");
  Serial.println(voltage);
  Serial.print("sp:");
  Serial.println(setpoint_pressure);
  Serial.print("ma:");
  Serial.println(max_ampere);
  Serial.print("at:");
  Serial.println(ampere_threshold);
  Serial.print("mp:");
  Serial.println(max_pressure);
  */
  
  
    
    
  
  if(config.setpoint_pressure ==0 && config.max_ampere ==0 &&  config.ampere_threshold && config.max_pressure==0)
  {
    pressure=0;
    voltage=0;
    ampere=0;
  }
  else
  {
    if(digitalRead(manauto)== HIGH)
    {
      mode_kerja=0;
      digitalWrite(relay_load,HIGH);
      digitalWrite(relay_unload,LOW);
      relay_enable();
      if (current> config.ampere_threshold)
      {
        //Serial.println("tes1");
        hitung_pressure();
        hitung_voltage();
      }
      else
      {
        //Serial.println("tes2");
         pressure=config.setpoint_pressure;
         hitung_voltage();
      } 
    }
    else if(digitalRead(manauto)== HIGH)     // no automode
    {  
      
      mode_kerja=1;
       if (current > config.ampere_threshold)
      {
        hitung_pressure();
        hitung_voltage();
      }
      else
      {
        pressure=config.setpoint_pressure;
         hitung_voltage();
      } 
      if((digitalRead(pbload) == LOW)||(digitalRead(switch_load) == LOW))
      {
        relay_enable();
        digitalWrite(relay_load, HIGH);
      }
      else if((digitalRead(pbload) == HIGH)&&(digitalRead(switch_load) == HIGH))
      {
        relay_enable();
        digitalWrite(relay_load,LOW);
      }
            
      if((digitalRead(pbunload) == LOW)||(digitalRead(switch_unload) == LOW))
      {
        relay_enable();
        digitalWrite(relay_unload, HIGH);
      }
      else if((digitalRead(pbunload) ==   HIGH)&&(digitalRead(switch_unload) ==HIGH))
      {
        relay_enable();
        digitalWrite(relay_unload,LOW);
      }
    }
  }
}



void setup() {
  init_devices();
  EEPROM.begin(200);
  if (!ReadConfig())
  {
    DefaultConfig();
  }

   temp_setpoint_pressure=config.setpoint_pressure;
   temp_max_ampere=config.max_ampere;
   temp_ampere_threshold=config.ampere_threshold;
   temp_max_pressure=config.max_pressure;
  ADS.begin();
  ADS.setGain(3);

  ledcSetup(0, freq, resolution);
  ledcAttachPin(dacpin, 0);

  input_pwm=0;
  
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  mb.begin(&Serial2, DIR);
  mb.slave(SLAVE_ID);//Initializng modbus slave device with ID 1
  mb.addHreg(1); // add the register  40001
  mb.addHreg(2); // add the register  40002
  mb.addHreg(3); // add the register  40003
  mb.addHreg(4); // add the register  40004
  mb.addHreg(5); // add the register  40004
  mb.addHreg(6); // add the register  40004
  mb.addHreg(7); // add the register  40004  
  mb.addHreg(8); // add the register  40004  


}

void loop() 
{  
  mb.task();
  yield();

  long currentMillis = millis();
  if (currentMillis - lastMillis > 200)
  {
    main_program();  
    lastMillis = currentMillis;    
  } 
}
