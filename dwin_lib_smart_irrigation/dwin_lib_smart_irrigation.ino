#include <Wire.h>
//#include "Time.h"
#include "RTClib.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <DWIN_Arduino.h>

#define ADDRESS_A     "1010"
#define ADDRESS_B     "1020"

#define DGUS_BAUD     9600

// If Using ESP 32
 // DWIN hmi(2, 3, DGUS_BAUD);

#if defined(AVR_ATmega1280) || defined(AVR_ATmega2560) || defined(FORCEHWSERIAL)
#define DGUS_SERIAL Serial1

#endif
DWIN hmi( Serial1, DGUS_BAUD);
void onHMIEvent(String address, int lastByte, String message, String response);
unsigned long getWordReply(String response, byte bytesBack);
uint16_t readVP(uint16_t vpAddress);

 #include "DWIN_Arduino_Helpers.hpp"
//Timestamp in milliseconds: 946684800000

//pins assignment
const int level_sensor =2;
const int motor = 12;
const int led= 13;

//aux variable
String crop_type;
String soil_type;
int soil_index;
int crop_index;
int day_index=0;
bool crop_flag=0;
bool soil_flag=0;
bool backup=0;
int  mean_moist=0;
int max_th;
int min_th;
int days_count;
int new_day;
int backup_irrigate=0;
int water_level_th=0;
int volume;
int pre_day =0 ;
volatile long pulse;
bool daily_motor=0;
DateTime now;
unsigned long start_time;
unsigned long curr_time;
unsigned char Buffer[9];
unsigned long sent_1;
unsigned long sent_0;
unsigned long sent_no;

struct configs{
  uint32_t config_done;
  bool crop_flag;
  bool soil_flag;
  int crop_index;
  int soil_index;
  unsigned long pulse;
  unsigned long irrigate_day;
  unsigned long time_spane;
  int days_count;
  unsigned long time_curr;
};

 void(* resetFunc) (void) = 0;//declare reset function at address 0

configs param_config;

//DATA arrays
int moist_sensor[2] = {A0,A1};
String crops[4]= {"wheat","rice","cotton","sugar"};
String daysOfTheWeek[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String soil[4]= {"SLS1","STLS2","SCLS3","LS004"};
//int button_array[7] = {2,3,4,5,6,7,8};//buttons
int max_th_array [4]={22,33,32,31};//wheat,cotton,sugar
// soil minimum threshold array for wrt crop and soil type 
int min_th_array [4][4] ={{15,22,25,22},
                    {14,20,24,20},
                    {14,20,24,21},
                   {14,20,24,20},};

//3d array index [crop][soil][day_of_irrigation] 
int end_event[4]={240,120,204,365};
int event_day[4][4][15]={
                      {{15,37,51,73,97,129,185,199,212,229},
                       {27,111,150,187,199,220},
                       {13,21,62,107,133,151,168,183,195,208,221},
                       {19,59,117,148,179,190,224},},

                        {{5,11,18,31,60,96,115},
                       {9,24,63,106},
                       {4,9,15,24,44,79,99,116},
                       {6,14,29,62,100},},
                       
                       {{95,126,148,168,97,191},
                       {112,148,168,179},
                       {91,119,140,159,178},
                       {103,137,161,188},},

                       {{98, 131, 153, 171, 188, 202, 215,228, 253, 337},
                       {116, 153,180, 202, 223, 224,363},
                       {95, 126, 147, 164, 179, 183, 205, 218, 229, 243, 333},
                       {107, 143 , 165, 186, 203,  220, 238, 348},}
                       };
//3d array index [crop][soil][water_level_in mm] 
int event_level[4][4][15]={ 
                        {{42,60,111,139,158,173,175,175,175,175},
                        {79,186,236,263,268,263},
                        {13,21,62,107,133,151,168,183,195,208,221},
                        {19,59,117,148,179,190,224},},
                        
                       {{17,28,39,64,94,94,94},
                       {36,74,142,142},
                       {14,22,30, 44, 73,84,84,84},
                       {23,40,70,115,115},},
                       
                       {{172,172,172,172,172,172},
                       {256,256,256,256},
                       {152,152,152,152,152},
                       {208,208,208,208},},
                       
                       {{194,194, 194, 194, 194, 194, 194,194, 194, 194},
                       {293, 293,293, 293, 293, 293,293},
                       {174, 174, 174, 174, 174, 174, 174, 174, 174, 174,174},
                       {237, 237 , 237, 237, 237,  237, 237, 237},}
                       };



unsigned char tft_days[8]= {0x5a, 0xa5, 0x05, 0x82,0x90 , 0x00, 0x00, 0x00};
unsigned char tft_pump[8]= {0x5a, 0xa5, 0x05, 0x82,0x91 , 0x00, 0x00, 0x00};
unsigned char tft_backup[8]= {0x5a, 0xa5, 0x05, 0x82,0x92 , 0x00, 0x00, 0x00};
unsigned char tft_moisture[8]= {0x5a, 0xa5, 0x05, 0x82,0x93 , 0x00, 0x00, 0x00};
unsigned char tft_done[8]= {0x5a, 0xa5, 0x05, 0x82,0x95 , 0x00, 0x00, 0x00};
unsigned char tft_reset[8]= {0x5a, 0xa5, 0x05, 0x82,0x94 , 0x00, 0x00, 0x00};

//lib function initialization
//RTC_DS1307 rtc;
RTC_DS3231 rtc;

bool motor_value;

unsigned long irrigate_day;
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(1000);
  Serial.println("DWIN HMI ~ Read Vars from vp's");
  Serial.print("DWIN Hardware Version ");
  Serial.println(hmi.getHWVersion());
  hmi.echoEnabled(false);
  hmi.hmiCallBack(onHMIEvent);
  hmi.setPage(0);
 hmi.setVPWord(0x9500,0);
  
  Wire.begin();
  // wait for Arduino Serial Monitor
 // while (!Serial) ;
 //rtc.adjust(DateTime(F(DATE), F(TIME)));

 // param_config.crop_flag=1;
  //   param_config.soil_flag=1;
  //    param_config.crop_index=1;
  //    param_config.soil_index=1;
  //    param_config.days_count=0;
  //   param_config.config_done=0xABCD;

    EEPROM.get(0,param_config);
  if(param_config.config_done==0xABCD){
    Serial.print("CONFIG");
    Serial.println(String(param_config.config_done));
    Serial.print("c_flag");
    Serial.println(String(param_config.crop_flag));
    Serial.print("S_flag");
    Serial.println(String(param_config.soil_flag));
    Serial.print("C_index");
    Serial.println(String(param_config.crop_index));
    Serial.print("soil_index");
    Serial.println(String(param_config.soil_index));
    
    crop_flag = param_config.crop_flag;
    soil_flag = param_config.soil_flag;
    crop_index= param_config.crop_index;
    soil_index= param_config.soil_index;
    days_count= param_config.days_count;
    hmi.setVPWord(0x9100,0);
    hmi.setVPWord(0x9200,0);
     hmi.setVPWord(0x9500,0);

    hmi.setPage(3);
    Serial.print("CONFIG");
    Serial.println(String(param_config.config_done));
    Serial.print("c_flag");
    Serial.println(String(crop_flag));
    Serial.print("S_flag");
    Serial.println(String(soil_flag));
    Serial.print("C_index");
    Serial.println(String(crop_index));
    Serial.print("soil_index");
    Serial.println(String(soil_index));
    
   
   // sprintf(buff,"config_done %b /n crop_flag %b /n soil_flag %b /n crop_index %d /n soil_index %d  /n Days_elapsed",
     //             param_config.config_done,param_config.crop_flag,param_config.soil_flag,param_config.crop_index,param_config.soil_index,param_config.days_count);

  }

  else if (param_config.config_done !=0xABCD)
  {Serial.println("NOT CONFIGURED");
   //Serial.print("CONFIG");
    // Serial.println(String(param_config.config_done));
    // Serial.print("c_flag");
    // Serial.println(String(param_config.crop_flag));
    // Serial.print("S_flag");
    // Serial.println(String(param_config.soil_flag));
    // Serial.print("C_index");
    // Serial.println(String(param_config.crop_index));
    // Serial.print("soil_index");
    // Serial.println(String(param_config.soil_index));
    
    //sprintf(buff,"config_done %b /n crop_flag %b /n soil_flag %b /n crop_index %d /n soil_index %d  /n Days_elapsed",
      //            param_config.config_done,param_config.crop_flag,param_config.soil_flag,param_config.crop_index,param_config.soil_index,param_config.days_count);


    //  param_config.crop_flag=1;
    // param_config.soil_flag=1;
    //  param_config.crop_index=1;
    //  param_config.soil_index=1;
    //  param_config.days_count=0;
    // param_config.config_done=0xABCD;
    // crop_flag = param_config.crop_flag;
    // soil_flag = param_config.soil_flag;
    // crop_index= param_config.crop_index;
    // soil_index= param_config.soil_index;
    // days_count= param_config.days_count;

  }

  delay(1000);

  //initialization and assignment

  pinMode(11, OUTPUT);
  pinMode(level_sensor,INPUT);
  pinMode(moist_sensor[0],INPUT);
  pinMode(moist_sensor[1],INPUT);
  pinMode(motor,OUTPUT);
  digitalWrite(motor,1);
  
  //interupt for water level_sensor
  attachInterrupt(digitalPinToInterrupt(level_sensor), level_sensor_call, RISING);  

  //now=rtc.now();
  //start_time=now.unixtime();
  analogWrite(11,8);
  delay(1000);
}

unsigned long time;
unsigned long daily_irrigation;
bool stop;
int prex;
int air_val = 550;
int water_val =440;
int moist_1 ; 
int moist_2 ; 
int nox;
int tft_crop;
int iterr;
int tft_soil;
void loop() {
  hmi.listen();
  
 sent_1=millis();
//  //read from event
//  //Serial.println(readVP(0x5500));
//  //Serial.println(readVP(0x6100));
////hmi.readVPWord(0x5500, 1);
//hmi.readVPWord(0x6100, 1);
  // fill up some data in the vp's
  iterr+=10;
  //hmi.setVPWord(0x9000,iterr);
  
  delay(10);
 
//  
////  if (Serial1.available()>0)
////  { for (int i = 0; i <= 8; i++) //this loop will store whole frame in buffer array.
////    {
////      Buffer[i] = Serial1.read();
////      //Serial.print(Buffer[i],HEX);
////      
////    }
////  Serial.println("THe recvied byte are ...: ");
////  for (int i = 0; i <= 8; i++) //this loop will store whole frame in buffer array.
////    {
////      Serial.print(i);
////      Serial.print(" = ");
////      Serial.println(Buffer[i],HEX);
////    }
////  Serial.println(" ");
////
////    
////    if (Buffer[0] == 0x5A)
////    {
////      switch (Buffer[4])
////      {
////        case 0x55:  // soil
////          Serial.println("crop type is select");
////          Serial.println(Buffer[8]);
////
////         // analogWrite(servo, Buffer[8]);
////          break;
//// 
////        case 0x61:
////         Serial.println("Soil type is select");   //for red
////          Serial.println(Buffer[8]);
////         
////          
////         // analogWrite(red, Buffer[8]);
////          break;
////        case 0x94: 
////         Serial.println("Reset is pressed");
////            //for red
////          Serial.println(Buffer[8]);
////         // analogWrite(red, Buffer[8]);
////          break;
////
////        default:
////          Serial.println("Nothing");
////          break;
////      }
////    }
////  delay(10);
////   if (Buffer[0] == 0xA5)
////    {
////      switch (Buffer[3])
////      {
////        case 0x55:  // soil
////          Serial.println("crop type is select");
////          Serial.println(Buffer[7]);
////
////         // analogWrite(servo, Buffer[8]);
////          break;
//// 
////        case 0x61:
////         Serial.println("Soil type is select");   //for red
////          Serial.println(Buffer[7]);
////         
////          
////         // analogWrite(red, Buffer[8]);
////          break;
////        case 0x94: 
////         Serial.println("Reset is pressed");
////            //for red
////          Serial.println(Buffer[7]);
////         // analogWrite(red, Buffer[8]);
////          break;
////
////        default:
////          Serial.println("Nothing");
////          break;
////      }
////    }
////    delay(10);
//
//
////  nox++;
////  if(Serial1.available()){
////    Serial.println("DAY SERIAL AVAILABEL ");
////  tft_days[6]=highByte(nox);
////  tft_days[7]=lowByte(nox);
////   Serial1.write(tft_days,8);
////  }
////   delay(50);
////   motor_value++;
////  if(Serial1.available()){
////    Serial.println("PUMP SERIAL AVAILABEL ");
////  tft_pump[7] = motor_value%2;
////  Serial1.write(tft_pump,8);
////    delay(50);}
////}
//
//
//
//  
//
//
//
//
//
//
//
//// if(Serial.available())
////   {
////     for(int i=0;i<=8;i++)   //TO store whole frame in buffer array. 0X5A A5 06 83 55 00 01 00 01 For ON
////     {
////     Buffer[i]= Serial.read();
////     }
////     delay(100);
////     if(Buffer[0]==0x5A)
////       { Serial.println("RECIVED BUFFER IS ");
//
////         for(int i ==0 ; i<8; i++){
////             Serial.print(Buffer[i]);
////         }
////         Serial.println();
//
////         if(Buffer[4]==0x94){
////           Serial.println("RECIVED COMMAND IS RESET ");
////         //tft_crop =Buffer[8];
//         
////         //Serial.print("ERASE CONFIG");
////         if(Buffer[]==1){
////           Serial.print("RECIVED COMMAND IS : ");
////           Serial.println(Buffer[]);
//          
////          //for(int i =0 ; i<EEPROM.length();i++)
////       }}}}
//
//
////   if(Serial.available())
////   {
////     for(int i=0;i<=8;i++)   //TO store whole frame in buffer array. 0X5A A5 06 83 55 00 01 00 01 For ON
////     {
////     Buffer[i]= Serial.read();
////     }
//    
////     if(Buffer[0]==0x5A)
////       {
////         if(Buffer[4]==0x94){
////         //tft_crop =Buffer[8];
//        
////         Serial.print("ERASE CONFIG");
////         if(Buffer[8]==1){
////          for(int i =0 ; i<EEPROM.length();i++)
////       {
////       EEPROM.write(i,0);
////       }
////       resetFunc();
////       }
////         }
////         }
////       delay(50);
////       }
//
//// //rtc days counting
 if(stop==0){
  // hmi.listen();
//delay(1000);
   
// DateTime now= rtc.now();
//curr_time=now.unixtime();
////Serial.print("RTC: ");
//
////Serial.println(curr_time);
//
if(((millis()-time)>2000)&&(soil_flag==1 && crop_flag==1)){
//
////if(((curr_time-time)>1000)&&(soil_flag==1 && crop_flag==1)){
time=millis();
////time =now.unixtime();
////now= rtc.now();
////curr_time=now.unixtime();
////Serial.print("RTC: ");
////Serial.println(time);
//
days_count++;
 param_config.days_count=days_count;
  EEPROM.put(0, param_config);
   hmi.setVPWord(0x9000,days_count);
//  
////  tft_days[6]=highByte(days_count);
////  tft_days[7]=lowByte(days_count);
////  Serial.write(tft_days,8);
////
////  
//  
if(days_count> end_event[crop_index]){
  stop=1;
   hmi.setVPWord(0x9500,1);
  for(int i =0 ; i<EEPROM.length();i++)
      {
      EEPROM.write(i,0);}
      resetFunc();

//  
//  

}
//
}
//
//
////days_count = (rtc.now()-start_time)/3; // millis()/2000;//
//// // if((curr_time-start_time)>3000){
//// //   ++days_count;
//// //   now=rtc.now();
//// //   start_time=now.unixtime();
//// // }
//// //now=rtc.now();
//// //curr_time  = now.unixtime();
//// //days_count = (curr_time - start_time)/3 ; 
//// // 
// 
//// //
//// //if(days_count-pre_day>1){
//// //pre_day = days_count;
//// //}
//
//
//

if(crop_flag==0){

//
//    
//     if(Buffer[0]==0x5A)
//       {
//         if(Buffer[4]==0x55)
//         tft_crop =Buffer[8];
//         Serial.print("TFT CROP");
//         Serial.println(tft_crop);
//        
//         }
//
//         if(Buffer[0]==0xA5)
//       {
//         if(Buffer[3]==0x55)
//         tft_crop =Buffer[7];
//         Serial.print("TFT CROP");
//         Serial.println(tft_crop);
//         delay(50);
//         }
//     
//     
//     
//       
//       
//
////   //crop selection
 if((tft_crop==1)){
crop_flag=1;
crop_type=crops[0]; 
Serial.println("crop is WHEAT");
crop_index=0;
Serial.println("SELECT SOIL");
     }
//
    if((tft_crop==2)){
     crop_flag=1;
     crop_type=crops[1]; 
     Serial.println("RICE CROP");
     crop_index=1;
     Serial.println("SELECT SOIL");
    }
//    
  if((tft_crop==3)){
  crop_flag=1;
  crop_type=crops[2]; 
  Serial.println("crop is cotton"); 
  crop_index=2;
    Serial.println("SELECT SOIL");
  }
//    
   if((tft_crop==4)){
   crop_flag=1;
   crop_type=crops[2]; 
   Serial.println("crop is sugar");
  crop_index=3;
  Serial.println("SELECT SOIL");
  }
//
 }
//   
// //selection of soilt type only after crops crop selection  
if (crop_flag==1 && soil_flag ==0){

   
    
//   if(Buffer[0]==0x5A)
//     {
//       if(Buffer[4]==0x61)
//       tft_soil =Buffer[8];
//       Serial.print("TFT soil");
//       Serial.println(tft_soil);
//       }
//       
//  if(Buffer[0]==0xA5)
//     {
//       if(Buffer[3]==0x61)
//       tft_soil =Buffer[7];
//       Serial.print("TFT soil");
//       Serial.println(tft_soil);
//       }
//       
//     delay(50);
//     
//
//  
 if((tft_soil==1)){
  soil_flag=1;
  soil_type=soil[0]; 
  Serial.println("SOIL is sandy loam soil");
  soil_index=0;
//lcd.print(String(crops[crop_index]));
   
   }
 if((tft_soil==2)){
  soil_flag=1;
  soil_type=soil[1]; 
  Serial.println("SOIL is SILT loam soil");
  soil_index=1;
//lcd.print(String(crops[crop_index]));
   }
    
   if((tft_soil==3)){
  soil_flag=1;
  soil_type=soil[2]; 
  Serial.println("SOIL is sandy clay loam soil");
  soil_index=2;
//lcd.print(String(crops[crop_index]));
  }

  if( (tft_soil==4)){
  soil_flag=1;
  soil_type=soil[3]; 
  Serial.println("SOIL is  loam soil");
  soil_index=3;
//lcd.print(String(crops[crop_index]));
  }

if(crop_flag==1 & soil_flag==1){
   param_config.config_done=0xABCD;
   param_config.crop_flag=1;
  param_config.soil_flag=1;
  param_config.crop_index=crop_index;
  param_config.soil_index=soil_index;
  EEPROM.put(0, param_config);
// EEPROM.commit();
  }
   
 }
//
//// //selection threshold from soil moisture based irrigation
 max_th = max_th_array[soil_index];
 min_th = min_th_array[crop_index][soil_index];
//
// // irrigation main system based on soil_mositure reading
if(((days_count - daily_irrigation>1)|daily_motor==0)&&(soil_flag==1 && crop_flag==1)){
//
moist_1 = map(analogRead(A0),air_val,water_val,0,100); 
moist_2 = map(analogRead(A1),air_val,water_val,0,100); 
//
//
 mean_moist = (moist_1 + moist_2)/2;
//// //delay(100);
//
 if((mean_moist>70 | mean_moist<5 ))
 {
 backup=1;
 //tft_backup[7]=1;
 //Serial.write(tft_backup,8);

// // lcd.setCursor(0,0);
// // lcd.print("ALERT BACKUP    ");
// // lcd.setCursor(0,1);
// // lcd.print("     SYSTEM     ");
  
 }
//
else 
 {backup=0;
// //tft_backup[7]=0;
//// Serial.write(tft_backup,8);
//
 }
//
if((mean_moist < min_th) && backup==0){
   digitalWrite(motor,0);//motor on
 daily_motor=1;
 //tft_pump[7]=1;
 //Serial.write(tft_pump,8);
// tft_moisture[7]=0;
// Serial.write(tft_moisture,8);
  
 } 
//
 else if((mean_moist > max_th)&& backup==0 && daily_motor==1){
   digitalWrite(motor,1);//motor off
   daily_irrigation = days_count;
//  // tft_pump[7]=0;
//  // Serial.write(tft_pump,8);
////   tft_moisture[7]=1;
////   Serial.write(tft_moisture,8);
//  
//  // delay(50);
//   // lcd.setCursor(0,0);
//   // lcd.print("CROP IRRIGATION");
//   // lcd.setCursor(0,1);
//   // lcd.print("DAY : ");
//   // lcd.print(days_count);
   daily_motor=0;
 }
 }
//
//// //backup system schedule baseed irrigation
if((soil_flag==1 && crop_flag==1)&&(days_count == event_day[crop_index][soil_index][day_index])){
 day_index++;
 backup_irrigate= 1 ;
pulse=0;
 } 
// 
////flow meter water threshold selection based on soil and crop type 
water_level_th = event_level[crop_index][soil_index][day_index] ; 
//
//// //for debuggin only 
 if((soil_flag==1 && crop_flag==1) &&(sent_1-sent_0>500 )){
sent_0=sent_1;
 if((volume<water_level_th && backup==1 &&  backup_irrigate==1)||((mean_moist < min_th) && backup==0))
 {
 //tft_pump[7]=1;
 //Serial.write(tft_pump,8);
   hmi.setVPWord(0x9100,1);
 //delay(50);
 }
//
 else if((volume>water_level_th && backup==1)||((mean_moist > max_th)&& backup==0 && daily_motor==1))
 {
   
 //tft_pump[7]=0;
 //Serial.write(tft_pump,8);
 hmi.setVPWord(0x9100,0);
 //delay(50);
 } 
//
// if((mean_moist < min_th) && backup==0){
// //tft_pump[7]=1;
// //Serial.write(tft_pump,8);
// hmi.setVPWord(0x9100,1);
// //delay(50);  
// } 
////
// else if((mean_moist > max_th)&& backup==0 && daily_motor==1){
//  // tft_pump[7]=0;
//  // Serial.write(tft_pump,8);
//  hmi.setVPWord(0x9100,0);
//  // delay(50);
// }
//
// //tft_backup[7]=backup;
//   //Serial.write(tft_backup,8);
   hmi.setVPWord(0x9200,backup);
   // hmi.setVPWord(0x9100,1);
//   delay(50);


//delay(1000);
 }
//
//// //flow meter volume
//
volume = (2.66*pulse);/////2.66 calibration factor and div by 1000 for mmm
//// //backup system motor on the scedule days and irrigating for water level threshold
//
 if(volume<water_level_th && backup==1 &&  backup_irrigate==1)
 { digitalWrite(motor,0);//motor on
//// tft_pump[7]=1;
//// Serial.write(tft_pump,8);
//// tft_moisture[7]=0;
//// Serial.write(tft_moisture,8);
}
//
 else if(volume>water_level_th && backup==1)
 {
 digitalWrite(motor,1);//motor off
//   //Serial.println("PUMP is off .........");
//// tft_pump[7]=0;
//// Serial.write(tft_pump,8);
//// tft_moisture[7]=1;
//// Serial.write(tft_moisture,8);
   backup_irrigate =1;
 } 
// 
 }
else if(stop==1){
Serial.println("IRRIGATION COMPLETE");
 
  }
}

//function for flow meter pulse counting
void level_sensor_call(){
  pulse++;
  }



// Event Occurs when response comes from HMI
void onHMIEvent(String address, int lastByte, String message, String response){  
  Serial.println("OnEvent : [ A : " + address + " | D : "+ String(lastByte, HEX)+ " | M : "+message+" | R : "+response+ " ]"); 
  if (address == "5500"){
     Serial.println("");
     Serial.print("0x");
     Serial.println(getWordReply(response,0),HEX);
     tft_crop= getWordReply(response,0);
  // Take your custom action call
  }

  if (address == "6100"){
     Serial.println("");
     Serial.print("0x");
     Serial.println(getWordReply(response,0),HEX);
     tft_soil = getWordReply(response,0);
  // Take your custom action call
  }

  if (address == "9400"){
     Serial.println("");
     Serial.print("0x");
     Serial.println(getWordReply(response,0),HEX);
     if(getWordReply(response,0)==1){
      hmi.setVPWord(0x9100,0);
      hmi.setPage(0);
     
        for(int i =0 ; i<EEPROM.length();i++)
      {
      EEPROM.write(i,0);}
      resetFunc();
     
      
      }
  // Take your custom action call
  }
}


//// Event Occurs when response comes from HMI
//void onHMIEvent(String address, int lastByte, String message, String response)
//{
//  Serial.println("OnEvent : [ A : " + address + " | D : " + String(lastByte, HEX) + " | M : " + message + " | R : " + response + " ]");
//  if (address == "1000")
//  { 
//     Serial.println("");
//     Serial.print("0x");
//     Serial.println(getWordReply(response,0),HEX);
//    //or print the word at 0x1001 if hmi.readVPWord(0x1000, 2); in the loop
//    //Serial.print("0x");
//    //Serial.println(getWordReply(response,2),HEX);
//  }
//}
