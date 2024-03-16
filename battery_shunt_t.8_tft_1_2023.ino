/*

Hardware required
ESP8266 - I used wemos d1 mini (clones) i bought a 20 pack for about $3 each.
INA219 Module - Used for measuring the current and voltage accuratly - I use the modules from ali express with a built in 1 ohm resistor. You will need to desolder this then run leads from the pcb to the new shunt.
Shunt - I have purchased quite a few shunts from a store called "CG Instrument Store" search "CG FL-2C 20a" to find the shunt I have used for this sketch. I have used 20amp and 50amp and find both accurate altho as you go up in amps you loose resolution e.g. each step becomes in the 10s of milliamps and accuracy over days becomes off.
Screen - I am pretty sure I used these - https://www.aliexpress.com/item/32834972099.html the 1.3 inch version that says IPS. I found these screens to be quite nice however I found a higher resolution 1.6 inch screen from another seller I would like to swap over to, its a really nice screen i suspect used for watches. (https://www.aliexpress.com/item/1005005039702168.html).
The screen required modifications to user_setup.h for the type of screen. I recall spending hours trying to get the display to work correctly just trying all the variations one at a time.

*/




/* TO DO
 *  
 *  set default display
 *  reset totals via button
 *  control scrreen led
 *  
   display max current draw
   display min voltage
   soft reset
   running hours
   calculate amp hours left
   calculate avg amp usage and calc a hour left/days left
   setup DEEP SLEEP - reduce power usage


   for the 1.3 inch use Setup 18 ST7789 without any changes.
 * */


#include <Wire.h>
#include <INA219.h>// arduinoina219 - https://github.com/flav1972/ArduinoINA219
#include <math.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <TFT_eWidget.h>               // Widget library


// lcd
#include <TFT_eSPI.h> //https://github.com/Bodmer/TFT_eSPI
#include <SPI.h>
#include "NotoSansBold15.h"
#include "NotoSansBold36.h"
#define AA_FONT_SMALL NotoSansBold15
#define AA_FONT_LARGE NotoSansBold36



// check user_setup.h altho using default config for current version 3/1/23
TFT_eSPI tft = TFT_eSPI();

ESP8266WiFiMulti WiFiMulti;

INA219 monitor;

GraphWidget gr = GraphWidget(&tft);    // Graph widget gr instance with pointer to tft
TraceWidget tr = TraceWidget(&gr);     // Graph trace tr with pointer to gr




// ina219 config
#define SHUNT_MAX_V 0.075
#define BUS_MAX_V   16.0
#define MAX_CURRENT 20
#define SHUNT_R   0.00375
//#define SHUNT_R   0.0015 // 50a - .075/50

/////////////////////////////////
///////////////// Config
////////////////////////////////
const word bat_capacity = 100;
const word dev_mode_splash = 0; // set to 1 to disable splash screen
const byte dev_random_ma = 0; // if this is set then random numbers are used to amps instead of actual readings.
const byte dev_speed = 0; // this is how often to refresh, speeding it up just helped with developing, 0 for normal mode (1000ms) , 100 for 100ms 
const long dev_random_ma_min = 10000;
const long dev_random_ma_max = 50000;
const byte dev_start_screen = 40; // 40 - display 5
////////////////////////////////

const char* shunt_version = "1.0";
const float v_offset = 0.1;
const char* shunt_size = "50amp";

unsigned long total_ma_out = 0;
unsigned long total_ma_in  = 0;
unsigned long total_watts  = 0;
byte display_state = 0;
unsigned long display_tmp = 0; // temp var used for timings/counting for display screens
float min_voltage = 0;
int32_t last_ma_reading = 0;
float last_shunt_reading = 0;
float last_v_reading = 0;
byte initial_v_meter_tft = 1;
uint32_t max_ma = 0;
uint32_t ma_readings = 0;
uint32_t seconds = 1 ;
unsigned long lastMillis = 3000;
unsigned long lastSendMillis = 3000;
float remaining_ma = bat_capacity * 1000;
uint16_t button_debounce = 0;
byte button_released = 1;

float update_lcd_prev_v = 0.0;
unsigned long update_lcd_prev_a = 0;

long historic_seconds_arr[60];
long historic_hours_arr[256];
long historic_minutes_arr[256];

byte historic_seconds = 0;
byte historic_hours = 0;
byte historic_minutes = 0;
byte historic_minute_counter = 0;

// for eeprom saving - https://forum.arduino.cc/t/arduino-easy-way-to-save-your-variables-on-eeprom-flash-in-esp8266-no-need-of-preferences-h-library/993975
struct s_eeprom_var {
  unsigned long total_ma_out;
  unsigned long total_ma_in;
  uint16_t flash_write_count;
  uint16_t boot_count;
  unsigned long total_seconds;
  unsigned long total_watt_seconds;
} eeprom_var;
byte last_flash_counter = 0;
unsigned long last_ma_out_reading = 0;
unsigned long last_ma_in_reading = 0;

void setup() {

  // zero out memory storage
  for (uint16_t i = 0; i<256; i++){
    if (i<60){
      historic_seconds_arr[i] = 0;
    }
    historic_minutes_arr[i] = 0;
    historic_hours_arr[i]   = 0;
  }

  EEPROM.begin(1000);


  // reset eeprom
  //eeprom_var.total_ma_out = 0;
  //eeprom_var.total_ma_in = 0;
  //eeprom_var.flash_write_count = 1;
  //eeprom_var.boot_count = 1;
  //eeprom_var.total_seconds = 0;
  //eeprom_var.total_watt_seconds = 0;
  //EEPROM.put(0, eeprom_var);  EEPROM.commit();

  //pinMode(D7, OUTPUT);
  //digitalWrite(D7, HIGH);

  pinMode(D4, INPUT);

  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
 
  tft.init();
  tft.setRotation(0);

  randomSeed(micros());

  Serial.begin(115200);
  monitor.begin();
  monitor.configure(INA219::RANGE_32V, INA219::GAIN_2_80MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  monitor.calibrate(SHUNT_R, SHUNT_MAX_V, BUS_MAX_V, MAX_CURRENT);

  EEPROM.get(0, eeprom_var);
  Serial.println("");
  Serial.println("----- eeprom load and printout");
  Serial.println(eeprom_var.total_ma_out);
  Serial.println(eeprom_var.total_ma_in);
  Serial.println(eeprom_var.flash_write_count);
  Serial.println(eeprom_var.boot_count);
  Serial.println(eeprom_var.total_seconds);
  Serial.println(eeprom_var.total_watt_seconds);

  // partial eeprom blanking
  //eeprom_var.total_seconds = 0;
  //eeprom_var.total_watt_seconds = 0;
  //EEPROM.put(0, eeprom_var);  EEPROM.commit();

  eeprom_var.boot_count++;


  // display splash screen
  // this was just playing around and something i made when i was a kid on an amstrad pc using basic in dos something I replicated here.
  tft.fillScreen(TFT_BLACK);
  if (!dev_mode_splash){
    for (long i=0; i<1500; i++){
      tft.drawLine(tft.width()/2, tft.height()/2, random(0,tft.width()), random(0,tft.height()), random(0,65536));
      delay(1);
    }
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Battery Shunt v1.0",0,0,2);
    tft.setTextFont(2);
    tft.setCursor(0, 20);
    tft.print(__DATE__);
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.setCursor(0, 50);
    tft.print("AH Out: ");
    tft.print((float)eeprom_var.total_ma_out/1000);
    tft.setCursor(0, 80);
    tft.print("AH In: ");
    tft.print((float)eeprom_var.total_ma_in/1000);
    delay(3000);
    tft.fillScreen(TFT_BLACK);
  }
  

  display_state = dev_start_screen;

}

void prepare_display(){
  update_lcd_prev_a = 0;
  update_lcd_prev_v = 0;
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(0);
  initial_v_meter_tft = 1; // force a initial volt meter incase voltage = 0;
}

void loop() {
  if ((millis() - lastMillis) >= (dev_speed ? dev_speed : 1000)) {
    lastMillis = millis();
    ampCounter();
    updateLCD();
    seconds++;
    eeprom_var.total_seconds++;    
  }
  if ((millis() - lastSendMillis) >= 60000) { // this code originally sent amp readings to be logged over http. kept the bit that recaibrates the ina219 incase it has reset
    lastSendMillis = millis();
    monitor.recalibrate();
    monitor.reconfig();

    last_flash_counter++;
    if (last_flash_counter == 10) {
      update_eeprom();
      last_flash_counter = 0;
    }
  }

  
  button_debounce <<= 1;
  button_debounce = button_debounce | digitalRead(D4);
  if (button_debounce == 0 && button_released == 1) {
    button_released = 0;
    if (display_state < 10) display_state = 10;
    else if (display_state < 20) display_state = 20;
    else if (display_state < 30) display_state = 30;
    else if (display_state < 40) display_state = 40;
    else display_state = 1;
    prepare_display();
    updateLCD();
  }
  if (button_debounce == 0xFFFF){
    button_released = 1;
  }
}

void updateLCD_V_AMPS_TOP(){
   if ( (update_lcd_prev_a != last_ma_reading || update_lcd_prev_v != last_v_reading) || initial_v_meter_tft){
    initial_v_meter_tft = 0;
    
    tft.loadFont(AA_FONT_SMALL);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK, true);
    tft.setCursor(5, 10);
    tft.print("V");
    tft.setCursor(5, 40);
    tft.print("A");
    tft.unloadFont();    
      
    update_lcd_prev_a = last_ma_reading;
    update_lcd_prev_v = last_v_reading;
    tft.loadFont(AA_FONT_LARGE);
    tft.setTextColor(TFT_BLUE, TFT_BLACK, true);
    tft.setCursor(20, 1);
    tft.print(last_v_reading,2);
    tft.print("    ");
    tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
    tft.setCursor(20, 30);
    tft.print((float)last_ma_reading / 1000);
    tft.print("    ");
    tft.drawRect(0, 0, 128, 65, TFT_OLIVE);
    }  
}

float remaining_amp_hours(){
  /*float rem_hrs = 0;
  float ah_used = 0;
  float ah_charged = 0;

  ah_used = (float)total_ma_out / 3600 / 1000;
  ah_charged = (float)total_ma_in / 3600 / 1000 * 0.99;
  rem_hrs = bat_capacity - ah_used + ah_charged;
  
  return (rem_hrs > bat_capacity ? bat_capacity : rem_hrs); 
  */
  return remaining_ma/1000;
}

void updateLCD() {
// voltage and amps at the top


  ////////////////////////////////////////////////////////////////////////////////////
  ///////// mode 1
  ////////////////////////////////////////////////////////////////////////////////////

  if (display_state < 10) {

    updateLCD_V_AMPS_TOP();
  
    // cumulative stats
    tft.unloadFont();
    tft.loadFont(AA_FONT_SMALL);
  
    tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
    tft.setCursor(5, 70);
    tft.print("AH Out:");
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.setCursor(70, 70);
    tft.print((float)total_ma_out / 3600 / 1000);
    tft.print("    ");
  
    tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
    tft.setCursor(5, 88);
    tft.print("Avg:");
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.setCursor(70, 88);
    tft.print((float)total_ma_out / seconds / 1000);
    tft.print("    ");
  
  
    tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
    tft.setCursor(5, 106);
    tft.print("AH In:");
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.setCursor(70, 106);
    tft.print((float)total_ma_in / 3600 / 1000);
    tft.print("    ");
  
    tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
    tft.setCursor(5, 124);
    tft.print("Est AH:");
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.setCursor(70, 124);
    tft.print(remaining_amp_hours());
    tft.print("    ");
  
    tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
    tft.setCursor(5, 142);
    tft.print("% Rem:");
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.setCursor(70, 142);
    float battery_percent = remaining_amp_hours() / bat_capacity * 100;
    tft.print((int)floor(battery_percent));
    tft.print("%    ");
  }

  ////////////////////////////////////////////////////////////////////////////////////
  ///////// mode 2
  ////////////////////////////////////////////////////////////////////////////////////
  if (display_state >= 10 && display_state < 20){
    if (display_state == 10){
      updateLCD_V_AMPS_TOP();

      tft.unloadFont();
      tft.setTextFont(7);
      tft.setTextSize(1);
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.setCursor(20, 80);
      float battery_percent = remaining_amp_hours() / bat_capacity * 100;
      tft.print((int)floor(battery_percent));
      tft.setTextFont(4);
      tft.print("%");

      tft.setTextFont(2);
      tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
      tft.setCursor(15, 145);
      tft.print("Rem AH: ");
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      //tft.setCursor(70, 124);
      tft.print(remaining_amp_hours());
      tft.print("    ");
      
    }
  }

  
  ////////////////////////////////////////////////////////////////////////////////////
  ///////// mode 3
  ////////////////////////////////////////////////////////////////////////////////////

  if (display_state >= 20 && display_state < 30){
    if (display_state == 20) {
      tft.loadFont(AA_FONT_SMALL);
      tft.setTextColor(TFT_GOLD, TFT_BLACK, true);
      tft.setCursor(25, 5);
      tft.print("Flash Stats");
      tft.drawLine(25,20,105,20,TFT_GOLD);
  
      tft.setCursor(0, 30);
      tft.setTextColor(TFT_BROWN, TFT_BLACK, true);
      tft.print("AH Out: ");
      tft.setCursor(60, 30);
      tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      tft.print((float)eeprom_var.total_ma_out / 1000);
  
      tft.setCursor(0, 45);
      tft.setTextColor(TFT_BROWN, TFT_BLACK, true);
      tft.print("AH In: ");
      tft.setCursor(60, 45);
      tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      tft.print((float)eeprom_var.total_ma_in / 1000);
  
      tft.setCursor(0, 60);
      tft.setTextColor(TFT_BROWN, TFT_BLACK, true);
      tft.print("Writes: ");
      tft.setCursor(60, 60);
      tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      tft.print(eeprom_var.flash_write_count);    
  
      tft.setCursor(0, 75);
      tft.setTextColor(TFT_BROWN, TFT_BLACK, true);
      tft.print("Boots: ");
      tft.setCursor(60, 75);
      tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      tft.print(eeprom_var.boot_count);    
      display_state++;

      tft.setCursor(0, 90);
      tft.setTextColor(TFT_BROWN, TFT_BLACK, true);
      tft.print("W/Hrs: ");
      tft.setCursor(60, 90);
      tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      tft.print(eeprom_var.total_watt_seconds/3600);

      tft.setCursor(0, 105);
      tft.setTextColor(TFT_BROWN, TFT_BLACK, true);
      tft.print("Hrs: ");
      tft.setCursor(60, 105);
      tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
      tft.print((float)eeprom_var.total_seconds/3600);
      display_state++;
    }
  }

  
  ////////////////////////////////////////////////////////////////////////////////////
  ///////// mode 4
  ////////////////////////////////////////////////////////////////////////////////////
if (display_state >= 30 && display_state < 40){
  uint32_t bg_colr = TFT_DARKGREEN;
  uint32_t hd_colr = TFT_YELLOW;
  uint32_t fg_colr = TFT_GREEN;
    
    if (display_state == 30){

      tft.fillRect(0,87,128,40,bg_colr);
      tft.unloadFont();
      tft.loadFont(AA_FONT_SMALL);
    
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.setCursor(8, 70);
      tft.print("Load Averages");

      display_state = 31;

    }

    if (display_state == 31){
      
      updateLCD_V_AMPS_TOP();

      tft.unloadFont();
      tft.loadFont(AA_FONT_SMALL);

      tft.setTextColor(hd_colr, bg_colr, true);
      tft.setCursor(5, 90);
      tft.print("5:");
      tft.setTextColor(fg_colr, bg_colr, true);
      tft.setCursor(28, 90);
      tft.print((float)calc_avg_historic(historic_minutes_arr, 5, historic_minutes)/1000,1);
      tft.print("    ");

      tft.setTextColor(hd_colr, bg_colr, true);
      tft.setCursor(67, 90);
      tft.print("20:");
      tft.setTextColor(fg_colr, bg_colr, true);
      tft.setCursor(93, 90);
      tft.print((float)calc_avg_historic(historic_minutes_arr, 20, historic_minutes)/1000,1);
      tft.print("    ");

      tft.setTextColor(hd_colr, bg_colr, true);
      tft.setCursor(5, 110);
      tft.print("60:");
      tft.setTextColor(fg_colr, bg_colr, true);
      tft.setCursor(28, 110);
      tft.print((float)calc_avg_historic(historic_minutes_arr, 60, historic_minutes)/1000,1);
      tft.print("    ");

      tft.setTextColor(hd_colr, bg_colr, true);
      tft.setCursor(67, 110);
      tft.print("6h:");
      tft.setTextColor(fg_colr, bg_colr, true);
      tft.setCursor(93, 110);
      tft.print((float)calc_avg_historic(historic_hours_arr, 6, historic_hours)/1000,1);
      tft.print("    ");

      tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
      tft.setCursor(5, 130);
      tft.print("Overall:");
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.setCursor(70, 130);
      tft.print((float)(long(total_ma_out)-long(total_ma_in)) / ma_readings / 1000);
      tft.print("    ");

      tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
      tft.setCursor(5, 145);
      tft.print("Hr:");
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.setCursor(30, 145);
      tft.print((float)seconds / 3600,1);
      tft.print("    ");

      tft.setTextColor(TFT_SILVER, TFT_BLACK, true);
      tft.setCursor(60, 145);
      tft.print("Rm:");
      tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
      tft.setCursor(90, 145);
      tft.print(remaining_amp_hours(),0);
      tft.print("    ");

      

      
    } 
  }

    ////////////////////////////////////////////////////////////////////////////////////
  ///////// mode 5 - charts 1
  ////////////////////////////////////////////////////////////////////////////////////
if (display_state >= 40 && display_state < 50){

  byte ipos = 0;
      // find min max to chart configuration
      long chart_min = 0;
      long chart_max = 0;
      for (byte i=0; i<=20; i++){
        ipos = historic_minutes-i;
        if (historic_minutes_arr[ipos] < chart_min) chart_min = historic_minutes_arr[ipos];
        if (historic_minutes_arr[ipos] > chart_max) chart_max = historic_minutes_arr[ipos];
      }

      chart_max = chart_max / 1000;
      chart_max = (floor(chart_max/10)+1)*10;

      // normalise the numbers from 0-100;
      float num_arr[21];
      for (byte i=0; i<=20; i++){
        ipos = historic_minutes-i;
        num_arr[i] = (float)historic_minutes_arr[ipos] / 1000 / chart_max * 100;
      }
  
      static uint32_t plotTime = millis();
      static float gx = 0.0, gy = 0.0;
      static float delta = 10.0;

      
    if (display_state == 40){
      tft.setRotation(1);
       // Graph area is 200 pixels wide, 150 pixels high, dark grey background
  gr.createGraph(159, 80, tft.color565(5, 5, 5));

  if (chart_min == 0){
  // x scale units is from 0 to 100, y scale units is -512 to 512  
    gr.setGraphScale(0, 100, 0, 100.0);
  } else {
    gr.setGraphScale(0, 100, -100.0, 100.0);
  }

  // X grid starts at 0 with lines every 20 x-scale units
  // Y grid starts at -512 with lines every 64 y-scale units
  // blue grid
  gr.setGraphGrid(0, 20.0, -100, 20.0, TFT_BLUE);

  // Draw empty graph, top left corner at pixel coordinate 40,10 on TFT
  gr.drawGraph(0, 0);

    // Start a trace with using red, trace points are in x and y scale units
  // In this example a horizontal line is drawn
  if (chart_min < 0){
    tr.startTrace(TFT_RED);
    // Add a trace point at 0.0,0.0 on graph
    tr.addPoint(0.0, 0.0);
    // Add another point at 100.0, 0.0 this will be joined via line to the last point added
    tr.addPoint(100.0, 0.0);
  }

  // Start a new trace with using white
  tr.startTrace(TFT_WHITE);

    display_state = 41; 
    }

    if (display_state == 41){
      for (byte i=0; i<=100; i+=5){
        tr.addPoint(i, num_arr[i/5]);
      }
      display_state++;
      display_tmp = historic_minute_counter;
    } else {
      if (display_tmp != historic_minute_counter) {
        display_state = 40;
        display_tmp = historic_minute_counter;
        prepare_display();
      }
    }
  }
}

void ampCounter() {
  last_v_reading = monitor.busVoltage();
  last_shunt_reading = monitor.shuntVoltage();
  last_ma_reading = monitor.shuntCurrent() * 1000;

  if (dev_random_ma) last_ma_reading = (int32_t)random(dev_random_ma_min, dev_random_ma_max);
  
  if (last_ma_reading > 0){
    total_watts += last_v_reading * last_ma_reading / 1000;
    eeprom_var.total_watt_seconds += last_v_reading * last_ma_reading / 1000;
  }

  remaining_ma -= (last_ma_reading/3600);
  if (remaining_ma < 0) remaining_ma = 0;
  if (remaining_ma > (bat_capacity * 1000)) remaining_ma = bat_capacity*1000;



  if (last_v_reading < 3.00){
    last_v_reading = 0;
    last_shunt_reading = 0;
    last_ma_reading = 0;
  }
  
  //last_ma_reading = 50000;
  //last_v_reading = 13.85;
  //last_ma_reading = 2300;

  ma_readings++;
  if (last_ma_reading > 0) {
    total_ma_out += last_ma_reading;
  } else {
    total_ma_in += abs(last_ma_reading);
  }

  if (last_v_reading < min_voltage) {
    min_voltage = last_v_reading;
  }
  if (last_ma_reading > max_ma) {
    max_ma = last_ma_reading;
  }

  // capture readings every second for historic averages.
  if(seconds > 1) historic_seconds++;
  if (historic_seconds == 60) historic_seconds = 0;
  historic_seconds_arr[historic_seconds] = last_ma_reading;

  // every minute average this reading and then push that into the minute and hourly historic arrays
  if (historic_seconds == 59){
    
    // updating minute averages
    
    long min_total = 0;
    for (byte i=0; i<60; i++){
      min_total += historic_seconds_arr[i];
    }
    if (seconds > 70) historic_minutes++;
    historic_minutes_arr[historic_minutes] = min_total/60;
    

    // update hourly
    historic_minute_counter++;
    if (historic_minute_counter == 60){ // 1 hour has elapsed
      if(seconds > 3610) historic_hours++;
      historic_minute_counter = 0;
      historic_hours_arr[historic_hours] = calc_avg_historic(historic_minutes_arr, 60, historic_minutes);
    }
  }
}

long calc_avg_historic(long d[], word size_of_avg, byte current_position){
  long sum_of_d = 0;
  if ( (current_position+1) >= size_of_avg){ //simple just iterate over the elements. +1 on current position to account for item 0, e.g. current position = 4 but were asking for an avg from 5. there are 5 items in the array 0,1,2,3,4
    for (word i=(current_position-size_of_avg)+1; i<=current_position; i++){
      sum_of_d += d[i];
    }
  } else { // more complex as we need to go back to the end of the array for some of the figures
    for (byte i=0; i<=current_position; i++){ // collect from the start of the array first
      sum_of_d += d[i];
    }
    // now go backwards from the end of the array and collect the rest of the data
    for (word i=256-(size_of_avg-(current_position+1)); i<256; i++){
      sum_of_d += d[i];
    }
  }
  return sum_of_d/size_of_avg;
}

void update_eeprom() {
  eeprom_var.flash_write_count++;
  Serial.println("updating eeprom");
  Serial.println(millis());
  Serial.print("adding mah: ");
  Serial.println(((total_ma_out - last_ma_out_reading) / 3600));

  eeprom_var.total_ma_out += ((total_ma_out - last_ma_out_reading) / 3600);
  last_ma_out_reading = total_ma_out;
  eeprom_var.total_ma_in += ((total_ma_in - last_ma_in_reading) / 3600);
  last_ma_in_reading = total_ma_in;

  EEPROM.put(0, eeprom_var);  EEPROM.commit();

}
