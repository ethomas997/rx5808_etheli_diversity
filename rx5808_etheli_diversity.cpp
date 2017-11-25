/*
   SPI driver based on fs_skyrf_58g-main.c Written by Simon Chambers
   TVOUT by Myles Metzel
   Scanner by Johan Hermen
   Initial 2 Button version by Peter (pete1990)
   Refactored and GUI reworked by Marko Hoepken
   Universal version my Marko Hoepken
   Diversity Receiver Mode and GUI improvements by Shea Ivey
   OLED Version by Shea Ivey
   Separating display concerns by Shea Ivey
   10 Favorite channels and functionality by George Chatzisavvidis
   change in displays by George Chatzisavvidis
   Adding Fatshark button to work with the module by George Chatzisavvidis
   OSD SUPPORT George Chatzisavvidis

   RX5808_etheli v1.0 version (by ET http://www.etheli.com):
     Implemented frequency set BY-MHZ MODE
     Renamed 'C' and 'D' band names to 'R' and 'L'
     Various code and functionality improvements

  The MIT License (MIT)

  Copyright (c) 2015 Marko Hoepken

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

#include "settings.h"
#include "Rx5808Fns.h"


// uncomment depending on the display you are using.
// this is an issue with the arduino preprocessor

#ifdef OLED_128x64_ADAFRUIT_SCREENS

#ifdef SH1106
// #include "Adafruit_SH1106.h"
#else
#include "Adafruit_SSD1306.h"
#endif
#include "Adafruit_GFX.h"
#include <Wire.h>
#include <SPI.h>
#endif

#include "screens.h"
screens drawScreen;


// addresses for persistent storage of settings in EEPROM:

#define EEPROM_ADR_STATE 0
#define EEPROM_ADR_CHANIDX 1
#define EEPROM_ADRW_RSSI_MIN_A 2
#define EEPROM_ADRW_RSSI_MAX_A 4
#ifdef USE_DIVERSITY
#define EEPROM_ADRW_RSSI_MIN_B 6
#define EEPROM_ADRW_RSSI_MAX_B 8
#define EEPROM_ADR_DIVERSITY 10
#endif
#define EEPROM_ADR_BEEP 11
#define EEPROM_ADR_ORDERBY 12
#define EEPROM_ADR_OSD 14
#define EEPROM_ADRW_FREQMHZ 16
#define EEPROM_ADRW_CHECKWORD 18       // integrity-check value EEPROM
#define EEPROM_ADR_CALLSIGN 20

#define EEPROM_ADR_TUNE_FAV_LAST 108

#define EEPROM_CHECK_VALUE 0x2759      // EEPROM integrity-check value

#define EEPROM_WRITE_WORD(addr,val) EEPROM.write((addr),lowByte(val));\
                                    EEPROM.write((addr+1),highByte(val));
#define EEPROM_READ_WORD(addr) ((((uint16_t)EEPROM.read(addr+1)) << 8) +\
                               EEPROM.read(addr))

#ifdef USE_GC9N_OSD
#define MAX_MENU_COUNT 8
#else    // if OSD disabled then no OSD:ON/OFF menu item
#define MAX_MENU_COUNT 7
#endif

void setup();
void loop();
void setTunerToCurrentChannel();
uint16_t channelIndexToName(uint8_t idx);
uint16_t getCurrentChannelInMhz();
void saveChannelToEEPROM();
void beep(uint16_t time);
void FillTemp_Tune_fav(int FavChannelx);
#ifdef USE_GC9N_OSD
void SendToOSD();
#endif
int8_t FSButtonDirection ();

int EEPROM_ADR_TUNE_FAV[10]  = {100, 101, 102, 103, 104, 105, 106, 107, 110, 111};
int temp_EEPROM_ADR_TUNE_FAV[10] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
//setting the 10 favorite channels memory blocks George Chatzisavvidis GC9N

#ifdef USE_GC9N_OSD
int OSDParams[4] = {0, 3, 0, 0};
char  OSDCommand[18] = " DRAWMENU";
#endif

// do coding as simple hex value to save memory.
const uint8_t channelNameCodes[] PROGMEM = {
  0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, // Band A
  0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, // Band B
  0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, // Band E
  0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, // Band F / Airwave
#ifdef USE_LBAND
  0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, // Band R / Immersion Raceband
  0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18  // BAND L / 5.3
#else
  0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78  // Band R / Immersion Raceband
#endif
};


#ifdef USE_DIVERSITY
uint8_t diversity_mode = useReceiverAuto;
char diversity_check_count = 0; // used to decide when to change antennas.
#endif

uint8_t system_state = START_STATE;
uint8_t state_last_used = START_STATE;
char last_state_menu_id = 2;
uint8_t last_state = START_STATE + 1; // force screen draw
uint8_t active_receiver = useReceiverA;

uint16_t rssi_best = 0; // used for band scanner
uint16_t rssi_min_a = RSSI_MIN_VAL;
uint16_t rssi_max_a = RSSI_MAX_VAL;
uint16_t rssi_setup_min_a = RSSI_MIN_VAL;
uint16_t rssi_setup_max_a = RSSI_MAX_VAL;
#ifdef USE_DIVERSITY
uint16_t rssi_min_b = RSSI_MIN_VAL;
uint16_t rssi_max_b = RSSI_MAX_VAL;
uint16_t rssi_setup_min_b = RSSI_MIN_VAL;
uint16_t rssi_setup_max_b = RSSI_MAX_VAL;
#endif

static uint8_t rssi_seek_threshold = RSSI_SEEK_TRESHOLD;
static uint8_t rssi_setup_run = 0;
static bool force_menu_redraw = 0;

static uint8_t current_channel_index = 0;
static uint8_t tracking_channel_index = 0;
static uint16_t current_channel_mhz = 0;
static char channel_sort_idx = 0;
static uint8_t switch_count = 0;
static uint8_t last_channel_index = 0xFF;
static uint16_t last_channel_mhz = 0;
static uint8_t force_seek = 0;
static uint8_t seek_direction = 1;
static unsigned long time_screen_saver = 0;
static uint8_t seek_found = 0;
static uint8_t scan_start = 0;
static bool stateManualInitFlag = false;
static uint8_t play_startup_beeps = 1;

static char call_sign[10];
static bool settings_beeps = true;
#ifdef USE_GC9N_OSD
static bool settings_OSD = false;
#endif
static bool settings_orderby_channel = true;
static bool HaveFav = false;
static bool RefreshFav = false;
static uint8_t FirstFav = 0;
//static bool FATSHARK_BUTTON_PUSHED=false;
//static int  LAST_FATSHARK_BUTTON_STATE=0;
static int lfavs = 0;
static byte FS_BUTTON_DIR = 0;

static bool chanChangedSaveFlag = false;
static bool previousUpDnButtonFlag = false;


// SETUP ----------------------------------------------------------------------------
void setup()
{

  Serial.begin(57600);
  // initialize digital pin 13 LED as an output.
  pinMode(led, OUTPUT); // status pin for TV mode errors
  digitalWrite(led, HIGH);
  // buzzer
  pinMode(buzzer, OUTPUT); // Feedback buzzer (active buzzer, not passive piezo)
  digitalWrite(buzzer, HIGH);
  // minimum control pins
  pinMode(buttonUp, INPUT);
  digitalWrite(buttonUp, INPUT_PULLUP);
  pinMode(buttonMode, INPUT);
  digitalWrite(buttonMode, INPUT_PULLUP);
  // optional control
  pinMode(buttonDown, INPUT);
  digitalWrite(buttonDown, INPUT_PULLUP);
  pinMode(buttonSave, INPUT);
  digitalWrite(buttonSave, INPUT_PULLUP);
  //Receiver Setup
  pinMode(receiverA_led, OUTPUT);
#ifdef USE_DIVERSITY
  pinMode(receiverB_led, OUTPUT);
#endif
  setReceiver(useReceiverA);
  // SPI pins for RX control
  pinMode (slaveSelectPin, OUTPUT);
  pinMode (spiDataPin, OUTPUT);
  pinMode (spiClockPin, OUTPUT);

  // use values only of EEprom is not 255 = unsaved
  uint8_t eeprom_check = EEPROM.read(EEPROM_ADR_STATE);

  // check EEPROM-integrity value; write defaults if mismatch
  if (EEPROM_READ_WORD(EEPROM_ADRW_CHECKWORD) != EEPROM_CHECK_VALUE)
  {
    EEPROM.write(EEPROM_ADR_STATE, START_STATE);
    EEPROM.write(EEPROM_ADR_CHANIDX, CHANNEL_MIN_INDEX);
    EEPROM_WRITE_WORD(EEPROM_ADRW_FREQMHZ, 0);
    EEPROM.write(EEPROM_ADR_BEEP, settings_beeps);
#ifdef USE_GC9N_OSD
    EEPROM.write(EEPROM_ADR_OSD, settings_OSD);
#else
    EEPROM.write(EEPROM_ADR_OSD, false);
#endif
    EEPROM.write(EEPROM_ADR_ORDERBY, settings_orderby_channel);
    // save 16 bit
    EEPROM_WRITE_WORD(EEPROM_ADRW_RSSI_MIN_A, RSSI_MIN_VAL);
    // save 16 bit
    EEPROM_WRITE_WORD(EEPROM_ADRW_RSSI_MAX_A, RSSI_MAX_VAL);

    // save default call sign
    strcpy(call_sign, CALL_SIGN); // load callsign
    for (uint8_t i = 0; i < sizeof(call_sign); i++)
    {
      EEPROM.write(EEPROM_ADR_CALLSIGN + i, call_sign[i]);
    }

#ifdef USE_DIVERSITY
    // diversity
    EEPROM.write(EEPROM_ADR_DIVERSITY, diversity_mode);
    // save 16 bit
    EEPROM_WRITE_WORD(EEPROM_ADRW_RSSI_MIN_B, RSSI_MIN_VAL);
    // save 16 bit
    EEPROM_WRITE_WORD(EEPROM_ADRW_RSSI_MAX_B, RSSI_MAX_VAL);
#endif

    // write EEPROM-integrity check value
    EEPROM_WRITE_WORD(EEPROM_ADRW_CHECKWORD, EEPROM_CHECK_VALUE);
  }

  // read saved settings from EEPROM
  system_state = EEPROM.read(EEPROM_ADR_STATE);
  state_last_used = system_state;
  current_channel_index = EEPROM.read(EEPROM_ADR_CHANIDX);
  tracking_channel_index = current_channel_index;
  current_channel_mhz = EEPROM_READ_WORD(EEPROM_ADRW_FREQMHZ);

  // set the channel as soon as we can for faster boot up times
  setTunerToCurrentChannel();

  settings_beeps = EEPROM.read(EEPROM_ADR_BEEP);
#ifdef USE_GC9N_OSD
  settings_OSD = EEPROM.read(EEPROM_ADR_OSD);
#endif

  settings_orderby_channel = EEPROM.read(EEPROM_ADR_ORDERBY);

  // load saved call sign
  for (uint8_t i = 0; i < sizeof(call_sign); i++)
  {
    call_sign[i] = EEPROM.read(EEPROM_ADR_CALLSIGN + i);
  }

  rssi_min_a = EEPROM_READ_WORD(EEPROM_ADRW_RSSI_MIN_A);
  rssi_max_a = EEPROM_READ_WORD(EEPROM_ADRW_RSSI_MAX_A);
#ifdef USE_DIVERSITY
  diversity_mode = EEPROM.read(EEPROM_ADR_DIVERSITY);
  rssi_min_b = EEPROM_READ_WORD(EEPROM_ADRW_RSSI_MIN_B);
  rssi_max_b = EEPROM_READ_WORD(EEPROM_ADRW_RSSI_MAX_B);
#endif
  force_menu_redraw = 1;

  // Init Display
  if (drawScreen.begin(call_sign) > 0)
  {
    // on Error flicker LED
    while (true)
    { // stay in ERROR for ever
      digitalWrite(led, !digitalRead(led));
      delay(100);
    }
  }

#ifdef USE_DIVERSITY
  // make sure we use receiver Auto when diveristy is unplugged.
  if (!isDiversity())
  {
    diversity_mode = useReceiverAuto;
  }
#endif
  // Setup Done - Turn Status LED off.
  digitalWrite(led, LOW);

  // initially select menu item for last-used mode
  if (system_state == STATE_FREQ_BYMHZ)
    last_state_menu_id = 3;
  else if (system_state == STATE_FAVORITE)
    last_state_menu_id = 4;

}


// LOOP ----------------------------------------------------------------------------

void loop()
{
  /*******************/
  /*   Mode Select   */
  /*******************/
  uint8_t in_menu;
  uint8_t in_menu_time_out;
  if (digitalRead(buttonMode) == LOW) // key pressed ?
  {
    //Serial.println("kei");

    time_screen_saver = 0;
    beep(50); // beep & debounce
    delay(KEY_DEBOUNCE / 2); // debounce
    beep(50); // beep & debounce
    delay(KEY_DEBOUNCE / 2); // debounce

    uint8_t press_time = 0;
    // on entry wait for release
    while (digitalRead(buttonMode) == LOW && press_time < 10)
    {
      delay(100);
      press_time++;
    }

#ifdef USE_GC9N_OSD
    OSDParams[0] = 0; //this is main menu
    OSDParams[1] = 3; //By default goes over manual
    SendToOSD(); //UPDATE OSD
#endif

    char menu_id = last_state_menu_id;
    // Show Mode Screen
    if (system_state == STATE_SEEK_FOUND)
    {
      system_state = STATE_SEEK;
    }
    in_menu = 1;
    in_menu_time_out = 50; // 20x 100ms = 5 seconds
    /*
      Enter Mode menu
      Show current mode
      Change mode by MODE key
      Any Mode will refresh screen
      If not MODE changes in 2 seconds, it uses last used mode
    */
    do
    {

      if (press_time >= 10) // if menu held for 1 second invoke quick save.
      {
        // user held the mode button and wants to quick save.
        in_menu = 0; // EXIT
        system_state = STATE_SAVE;
#ifdef USE_GC9N_OSD
        OSDParams[0] = -2; //this is save
        SendToOSD(); //UPDATE OSD
#endif
        break;
      }

      if (chanChangedSaveFlag)
      {  //need to save new channel
        chanChangedSaveFlag = false;
        saveChannelToEEPROM();
      }
 
      switch (menu_id)
      {
        case 0: // AUTO MODE
          system_state = STATE_SEEK;
          last_state_menu_id = menu_id;
          force_seek = 1;
          seek_found = 0;
          break;
        case 1: // Band Scanner
          system_state = STATE_SCAN;
          scan_start = 1;
          break;
        case 2: // manual mode
          system_state = STATE_MANUAL;
          last_state_menu_id = menu_id;
          break;
        case 3: // Set freq by MHz
          system_state = STATE_FREQ_BYMHZ;
          last_state_menu_id = menu_id;
          break;
        case 4: // Favorites Menu       //gc9n
          system_state = STATE_FAVORITE;       //gc9n
          last_state_menu_id = menu_id;
          break;                        //gc9n
        case 5: // Setup Menu           //gc9n
          system_state = STATE_SETUP_MENU;     //gc9n
          break;                        //gc9n
#ifdef USE_DIVERSITY
        case 6: // Diversity
          if (isDiversity()) {
            system_state = STATE_DIVERSITY;
          }
          else {
            menu_id++;
          }
          break;
#else
        case 6: // Skip
          menu_id++;
#endif
        case 7://Vres modelo            //gc9n
           system_state = STATE_SCREEN_SAVER_LITE;       //gc9n
           //drawScreen.updateScreenSaver(rssi);
          break;
        case 8:// OSD enable/disable  //gc9n
          break;                        //gc9n
      } // end switch

      // draw mode select screen
      ////Serial.println (systemState);
      if (menu_id > 4)
      {
#ifdef USE_GC9N_OSD
        drawScreen.mainMenuSecondPage(menu_id - 5, settings_OSD);
#else
        drawScreen.mainMenuSecondPage(menu_id - 5, false);
#endif
      }
      else
      {
        drawScreen.mainMenu(menu_id);

      }


      while (digitalRead(buttonMode) == LOW || digitalRead(buttonUp) == LOW || digitalRead(buttonDown) == LOW  || FSButtonDirection() == 1 || FSButtonDirection() == 2)
      {
        // wait for MODE release
        in_menu_time_out = 50;
      }
      while (--in_menu_time_out && ((digitalRead(buttonMode) == HIGH) && (digitalRead(buttonUp) == HIGH) && (digitalRead(buttonDown) == HIGH) &&  FSButtonDirection() == 0  )) // wait for next key press or time out
      {
        delay(100); // timeout delay
      }
      if (in_menu_time_out == 0 || digitalRead(buttonMode) == LOW)
      {
        if (menu_id == 8)
        {
#ifdef USE_GC9N_OSD
          settings_OSD = !settings_OSD;
          EEPROM.write(EEPROM_ADR_OSD, settings_OSD);
          if (settings_OSD == false)
          {
            OSDParams[0] = -99; // CLEAR OSD
            SendToOSD(); //UPDATE OSD
          }
          else
          {
            OSDParams[0] = 0; // ENABLE OSD AND GO OVER MAIN MENU OSD SELECTION
            OSDParams[1] = 7; // OSD
            SendToOSD(); //UPDATE OSD
          }
#endif
        }
        else
        {

          // Serial.read();
          if (digitalRead(buttonMode) != LOW) {
            system_state = state_last_used; // exit to last state on timeout.
          }
          in_menu = 0; // EXIT
          beep(KEY_DEBOUNCE / 2); // beep & debounce
          delay(50); // debounce
          beep(KEY_DEBOUNCE / 2); // beep & debounce
          delay(50); // debounce
        }
      }
      else // no timeout, must be keypressed
      {
        /*********************/
        /*   Menu handler   */
        /*********************/

        if (digitalRead(buttonUp) == LOW  || FS_BUTTON_DIR == 1) {
          menu_id--;
#ifdef USE_DIVERSITY
          if (!isDiversity() && menu_id == 6) { // make sure we back up two menu slots.
            menu_id--;
          }
#endif
        }
        else if (digitalRead(buttonDown) == LOW   || FS_BUTTON_DIR == 2) {
          menu_id++;
        }

        if (menu_id > MAX_MENU_COUNT)
        {
          menu_id = 0; // next state
        }
        if (menu_id < 0)
        {
          menu_id = MAX_MENU_COUNT;
        }

        in_menu_time_out = 50;
        beep(50); // beep & debounce
        delay(KEY_DEBOUNCE); // debounce
#ifdef USE_GC9N_OSD
        OSDParams[0] = 0; //this is main menu
        OSDParams[1] = menu_id + 1; //By defaut=lt goes over manual
        SendToOSD(); //UPDATE OSD
#endif
      }
    }
    while (in_menu);

    last_state = 255; // force redraw of current screen
    switch_count = 0;

  }
  else // key pressed
  { // reset debounce
    switch_count = 0;
  }

  /***********************/
  /*     Save buttom     */
  /***********************/
  // hardware save buttom support (if no display is used)
  if (digitalRead(buttonSave) == LOW)
  {
    system_state = STATE_SAVE;
  }

  /***************************************/
  /*   Draw screen if mode has changed   */
  /***************************************/
  if (force_menu_redraw || system_state != last_state)
  {
    force_menu_redraw = 0;
    /************************/
    /*   Main screen draw   */
    /************************/
    // changed state, clear an draw new screen

    // simple menu
    switch (system_state)
    {
      case STATE_SCAN: // Band Scanner
        state_last_used = system_state;
      case STATE_RSSI_SETUP: // RSSI setup
        // draw selected
        if (system_state == STATE_RSSI_SETUP)
        {
          // prepare new setup
          rssi_min_a = 50;
          rssi_max_a = 300; // set to max range
          rssi_setup_min_a = RSSI_MAX_VAL;
          rssi_setup_max_a = RSSI_MIN_VAL;
#ifdef USE_DIVERSITY
          rssi_min_b = 50;
          rssi_max_b = 300; // set to max range
          rssi_setup_min_b = RSSI_MAX_VAL;
          rssi_setup_max_b = RSSI_MIN_VAL;
#endif
          rssi_setup_run = RSSI_SETUP_RUN;
        }

        // trigger new scan from begin
        channel_sort_idx = CHANNEL_MIN;
        current_channel_index = getChannelSortTableEntry(channel_sort_idx);
        rssi_best = 0;
        scan_start = 1;
        drawScreen.bandScanMode(system_state);
        break;
      case STATE_SEEK: // seek mode
        rssi_seek_threshold = RSSI_SEEK_TRESHOLD;
        rssi_best = 0;
        force_seek = 1;
      case STATE_MANUAL: // manual mode
        if (system_state == STATE_MANUAL)
        {
          time_screen_saver = millis();
          stateManualInitFlag = true;       //indicate just entered state
        }
        else if (system_state == STATE_SEEK)
        {
          time_screen_saver = 0; // dont show screen saver until we found a channel.
          stateManualInitFlag = false;
        }
        drawScreen.seekMode(system_state);

        // return user to their saved channel after bandscan
        if (state_last_used == STATE_SCAN || state_last_used == STATE_FAVORITE ||
            state_last_used == STATE_FREQ_BYMHZ || last_state == STATE_RSSI_SETUP )
        {
          current_channel_index = EEPROM.read(EEPROM_ADR_CHANIDX);
        }
        state_last_used = system_state;
        break;
#ifdef USE_DIVERSITY
      case STATE_DIVERSITY:
        // diversity menu is below this is just a place holder.
        break;
#endif
      case STATE_SETUP_MENU:

        break;
      case STATE_SAVE:

        EEPROM.write(EEPROM_ADR_CHANIDX, current_channel_index);
        EEPROM.write(EEPROM_ADR_STATE, state_last_used);
        EEPROM.write(EEPROM_ADR_BEEP, settings_beeps);
        EEPROM.write(EEPROM_ADR_ORDERBY, settings_orderby_channel);
        // save call sign
        for (uint8_t i = 0; i < sizeof(call_sign); i++) {
          EEPROM.write(EEPROM_ADR_CALLSIGN + i, call_sign[i]);
        }
#ifdef USE_DIVERSITY
        EEPROM.write(EEPROM_ADR_DIVERSITY, diversity_mode);
#endif

        ///////////////////////FAVORITIES SAVE Gc9n
        if (last_state != STATE_SETUP_MENU  && state_last_used != STATE_FAVORITE ) // if you didnt came from menu setup  save favorite
        {
          if ( EEPROM.read(EEPROM_ADR_TUNE_FAV[9]) != 255) //ALL FAVS full gc9n
          {
            int lfav;
            lfav = EEPROM.read(EEPROM_ADR_TUNE_FAV_LAST);
            if (lfav == 9)
            {
              lfav = 0;
            }
            else
            {
              lfav = lfav + 1;
            }
            EEPROM.write(EEPROM_ADR_TUNE_FAV[lfav], 255); // rotate the favs if full
            EEPROM.write(EEPROM_ADR_TUNE_FAV_LAST, lfav);
          }
          for (int i = 0; i < 10; i++)
          {
            if ( EEPROM.read(EEPROM_ADR_TUNE_FAV[i]) == 255) //not used  gc9n
            {
              EEPROM.write(EEPROM_ADR_TUNE_FAV[i], current_channel_index);
              EEPROM.write(EEPROM_ADR_TUNE_FAV_LAST, i);
              i = 12; //exit loop
            }
          }
#ifdef USE_GC9N_OSD
          OSDParams[0] = -2; //this is save
          OSDParams[1] = 0; //By default goes over manual
          SendToOSD(); //UPDATE OSD
#endif
          drawScreen.save(state_last_used, current_channel_index, getCurrentChannelInMhz(), call_sign, EEPROM.read(EEPROM_ADR_TUNE_FAV_LAST) + 1);
        }
        if (last_state == STATE_SETUP_MENU)
        {
#ifdef USE_GC9N_OSD
          OSDParams[0] = -2; //this is save
          OSDParams[1] = 0; //By default goes over manual
          SendToOSD(); //UPDATE OSD
#endif
          drawScreen.save(state_last_used, current_channel_index, getCurrentChannelInMhz(), call_sign, -99);
        }

        ///LONG PRESS IN FAVORITES WILL DELETE THE CURRENT FAVORITE CHANNEL
        if  (state_last_used == STATE_FAVORITE && last_state == 255)
        {  
#ifdef USE_GC9N_OSD
          OSDParams[0] = -3; //this is delete
          OSDParams[1] = 0; //By default goes over manual
          SendToOSD(); //UPDATE OSD
#endif
          EEPROM.write(EEPROM_ADR_TUNE_FAV[lfavs], 255);
          drawScreen.FavDelete(   getCurrentChannelInMhz(), lfavs + 1);
          
          for (int i = 0; i < 10; i++) {
            temp_EEPROM_ADR_TUNE_FAV[i] = 255;    //empty temp
          }

          //--REORGANIZE FAVS  GC9n
          int MaxFav = 0;
          for (int i = 0; i < 10; i++)
          {

            if ( EEPROM.read(EEPROM_ADR_TUNE_FAV[i]) != 255) //not used  gc9n
            {
              FillTemp_Tune_fav(EEPROM.read(EEPROM_ADR_TUNE_FAV[i]));
              MaxFav++;
            }
          }

          for (int i = 0; i < 10; i++)
          {
            EEPROM.write(EEPROM_ADR_TUNE_FAV[i], temp_EEPROM_ADR_TUNE_FAV[i] );

          }

          //DELETED SUCCESFULLY AND GO TO FIRST FAV (IF EXISTS)
          //drawScreen.FavReorg(MaxFav);
          RefreshFav = false;
           delay(1000);
          EEPROM.write(EEPROM_ADR_TUNE_FAV_LAST, 0);
          current_channel_index = EEPROM.read(EEPROM_ADR_TUNE_FAV[0]) ;
          lfavs = 0;
          current_channel_index = EEPROM.read(EEPROM_ADR_TUNE_FAV[0]) ;
          drawScreen.FavSel(1);
          channel_sort_idx = getChannelSortTableIndex(current_channel_index);
          saveChannelToEEPROM();


          //drawScreen.screenSaver(diversity_mode, channelIndexToName(channelIndex), getCurrentChannelInMhz(), call_sign);
        }
        ///END LONG PRESS IN FAVORITES WILL DELETE THE CURRENT FAVORITE CHANNEL
        for (uint8_t loop = 0; loop < 5; loop++)
        {
          beep(100); // beep
          delay(100);
        }
        delay(800);
        system_state = state_last_used; // return to saved function
        force_menu_redraw = 1; // we change the state twice, must force redraw of menu

        // selection by inverted box
        break;

      case STATE_FAVORITE: // FAV mode
        time_screen_saver = millis();
        state_last_used = system_state;
        break;

      case STATE_FREQ_BYMHZ:
        time_screen_saver = millis();
        if (system_state != state_last_used)
        {
          state_last_used = system_state;
          // save so state is resumed after restart
          EEPROM.write(EEPROM_ADR_STATE, STATE_FREQ_BYMHZ);
        }
        break;

    } // end switch

    last_state = system_state;
  }

  // play 3 beeps to give feedback of correct start
  if (play_startup_beeps)
  {
    play_startup_beeps = 0;
#define UP_BEEP 50
    beep(UP_BEEP);
    delay(UP_BEEP);
    beep(UP_BEEP);
    delay(UP_BEEP);
    beep(UP_BEEP);
  }

  uint8_t rssi_value;

  /*************************************/
  /*   Processing depending of state   */
  /*************************************/
#ifndef TVOUT_SCREENS
  if (system_state == STATE_SCREEN_SAVER ||
      system_state == STATE_SCREEN_SAVER_LITE ||
      system_state == STATE_FREQ_BYMHZ)
  {

    uint16_t freqInMHz = getCurrentChannelInMhz();
              //if freq matches table entry then show channel name
    uint16_t chanName =
            (getChannelFreqTableEntry(current_channel_index) == freqInMHz) ?
                              channelIndexToName(current_channel_index) : 0;

#ifdef USE_DIVERSITY
    drawScreen.screenSaver(diversity_mode, chanName, freqInMHz, call_sign);
#else
    drawScreen.screenSaver(chanName, freqInMHz, call_sign);
#endif
    time_screen_saver = millis();
    do
    {
      rssi_value = readRSSI();

      if (chanChangedSaveFlag && time_screen_saver + 1000 < millis())
      {  //unsaved channel change and enough time elapsed
        chanChangedSaveFlag = false;
        saveChannelToEEPROM();
      }

      if ( ((time_screen_saver != 0 && time_screen_saver + (SCREENSAVER_TIMEOUT * 1000) < millis())) )
      {
#ifdef USE_GC9N_OSD
        OSDParams[0] = -99; // CLEAR OSD
        SendToOSD(); //UPDATE OSD
#endif
        time_screen_saver = 0;
      }

#ifdef USE_DIVERSITY
      drawScreen.updateScreenSaver(active_receiver, rssi_value, readRSSI(useReceiverA), readRSSI(useReceiverB));
#else
      drawScreen.updateScreenSaver(rssi_value);
#endif

      // if 'up' or 'down' button then exit loop
      if (digitalRead(buttonUp) == LOW || FS_BUTTON_DIR == 1 ||   // channel UP
         (digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2))  // channel DOWN
      {
        break;
      }
      else  // if not 'up' or 'down' button then clear tracking flag
        previousUpDnButtonFlag = false;
    }
    while (digitalRead(buttonMode) == HIGH);     // wait for next button press
    system_state = state_last_used;

    // if screen saver then exit 'loop()' function here
    // also exit if mode button to get to menu processing without delay
    if (system_state != STATE_FREQ_BYMHZ || digitalRead(buttonMode) == LOW)
      return;
  }
#endif

#ifdef USE_DIVERSITY
  if (system_state == STATE_DIVERSITY)
  {
    // simple menu
    char menu_id = diversity_mode;
    uint8_t in_menu = 1;
    do
    {
      diversity_mode = menu_id;
      drawScreen.diversity(diversity_mode);
      do
      {
        //delay(10); // timeout delay
        readRSSI();
        drawScreen.updateDiversity(active_receiver, readRSSI(useReceiverA), readRSSI(useReceiverB));
      }
      while ((digitalRead(buttonMode) == HIGH) && (digitalRead(buttonUp) == HIGH) && (digitalRead(buttonDown) == HIGH) &&  FSButtonDirection() == 0 ); // wait for next mode or time out

      if (digitalRead(buttonMode) == LOW)       // channel UP
      {
        in_menu = 0; // exit menu
      }
      else if (digitalRead(buttonUp) == LOW || FS_BUTTON_DIR == 1) {
        menu_id--;
      }
      else if (digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2) {
        menu_id++;
      }

      if (menu_id > useReceiverB) {
        menu_id = 0;
      }
      if (menu_id < 0) {
        menu_id = useReceiverB;
      }
      beep(50); // beep & debounce
      delay(KEY_DEBOUNCE); // debounce
    }
    while (in_menu);

    system_state = state_last_used;
  }
#endif
  if (system_state != STATE_FAVORITE)
  {
    RefreshFav = false;
  }

  /*****************************************/
  /*   Processing FAVORITES                */
  /*****************************************/
  if (system_state == STATE_FAVORITE ) //|| state == STATE_SEEK)
  {

    // read rssi
    wait_rssi_ready();
    rssi_value = readRSSI();
    rssi_best = (rssi_value > rssi_best) ? rssi_value : rssi_best;
    time_screen_saver = millis();


    if (!RefreshFav)
    {

      EEPROM.write(EEPROM_ADR_STATE, STATE_FAVORITE);
      HaveFav = false;
      for (int i = 0; i < 10; i++)
      {
        //NoFav
        if (EEPROM.read(EEPROM_ADR_TUNE_FAV[i]) != 255)
        { FirstFav = i;
          HaveFav = true;

        }
      }
      RefreshFav = true;
      //channel=channel_from_index(current_channel_index); // get 0...47 index depending of current channel
    }  // handling of keys

    if ( digitalRead(buttonUp) == LOW || FS_BUTTON_DIR == 1)      // channel UP
    {
      delay(KEY_DEBOUNCE); // debounce
      lfavs++;
      if (lfavs > FirstFav) {
        lfavs = 0;
      }
      current_channel_index = EEPROM.read(EEPROM_ADR_TUNE_FAV[lfavs]) ;
      if (current_channel_index != 255)
      {
        drawScreen.FavSel(lfavs + 1);
        channel_sort_idx = getChannelSortTableIndex(current_channel_index); // get 0...47 index depending of current channel
        time_screen_saver = millis();
        beep(50); // beep & debounce
        delay(KEY_DEBOUNCE); // debounce
        if (channel_sort_idx > CHANNEL_MAX)
          channel_sort_idx = CHANNEL_MIN;
        if (current_channel_index > CHANNEL_MAX_INDEX)
          current_channel_index = CHANNEL_MIN_INDEX;
        // drawScreen.seekMode(state);
        saveChannelToEEPROM();
        //Serial.println(current_channel_index);
      }
      else
      {
        lfavs--;
      }
    }


    if ( digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2) // channel DOWN
    {

      delay(KEY_DEBOUNCE); // debounce
      lfavs--;
      if (lfavs < 0)
      {
        lfavs = FirstFav;
      }

      current_channel_index = EEPROM.read(EEPROM_ADR_TUNE_FAV[lfavs]) ;
      if (current_channel_index != 255)
      {
        drawScreen.FavSel(lfavs + 1);
        channel_sort_idx = getChannelSortTableIndex(current_channel_index); // get 0...47 index depending of current channel
        time_screen_saver = millis();
        beep(50); // beep & debounce
        delay(KEY_DEBOUNCE); // debounce

        if (channel_sort_idx < CHANNEL_MIN)
          channel_sort_idx = CHANNEL_MAX;
        if (current_channel_index > CHANNEL_MAX_INDEX) // negative overflow
        {
          current_channel_index = CHANNEL_MAX_INDEX;
        }

        //drawScreen.seekMode(state);
        saveChannelToEEPROM();
      }
      else
      {
        lfavs++;
      }
    }

    if (HaveFav == false)
    { drawScreen.NoFav();

    }   // IF YOU DONT HAVE FAVS

    else
    {
      //delay(KEY_DEBOUNCE); // debounce
      system_state = STATE_SCREEN_SAVER;
    }// IF YOU HAVE FAVS

#ifdef USE_GC9N_OSD
    OSDParams[0] = 4; //this is FAV menu
    OSDParams[1] = getCurrentChannelInMhz();
    OSDParams[2] = lfavs + 1;
    SendToOSD(); //UPDATE OSD
#endif
  }


  /*****************************************/
  /*   Processing Set Freq by MHz          */
  /*****************************************/
  if (system_state == STATE_FREQ_BYMHZ)
  {
    static int buttonDelayValue = KEY_DEBOUNCE;

//    OSDParams[0] = 3;

    // if just entered mode then set initial MHz value via table index
    if (current_channel_mhz == 0)
      current_channel_mhz = getChannelFreqTableEntry(current_channel_index);

    // handling of keys
    bool upFlag = (digitalRead(buttonUp) == LOW || FS_BUTTON_DIR == 1);    // channel UP
    bool dnFlag = (digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2);  // channel DOWN

    if (upFlag || dnFlag)
    {
      time_screen_saver = millis();

      if (!previousUpDnButtonFlag)
      {  //button press is "new"
        beep(50);  // beep & debounce
        buttonDelayValue = KEY_DEBOUNCE;
        previousUpDnButtonFlag = true;
      }
      else
      {  //button is being held down, do progressive mod speedup
        if (buttonDelayValue > 20)
          buttonDelayValue -= 20;
        else if (buttonDelayValue > 0)
          --buttonDelayValue;
      }
      if (buttonDelayValue > 0)
        delay(buttonDelayValue);  // debounce

      FS_BUTTON_DIR = 0;
      if (upFlag)
      {
        if (buttonDelayValue > 0)
          ++current_channel_mhz;
        else
          current_channel_mhz = (current_channel_mhz + 10) / 10 * 10;
        if (current_channel_mhz > MAX_CHANNEL_MHZ)
          current_channel_mhz = MAX_CHANNEL_MHZ;
      }
      else
      {
        if (buttonDelayValue > 0)
          --current_channel_mhz;
        else
          current_channel_mhz = (current_channel_mhz - 1) / 10 * 10;
        if (current_channel_mhz < MIN_CHANNEL_MHZ)
          current_channel_mhz = MIN_CHANNEL_MHZ;
      }

      // set table index to nearest entry
      current_channel_index = freqInMhzToNearestFreqIdx(
                              current_channel_mhz, upFlag);

      // set tracking equal so tune is via 'current_channel_mhz'
      tracking_channel_index = current_channel_index;

      // indicate channel changed and needs to be saved
      chanChangedSaveFlag = true;

//#ifdef USE_GC9N_OSD
//      if (OSDParams[1]!=getCurrentChannelInMhz())
//      {
//        OSDParams[1]=getCurrentChannelInMhz();
//        SendToOSD(); //UPDATE OSD
//      }
//#endif

    }
    else
      previousUpDnButtonFlag = false;
  }
  else
    current_channel_mhz = 0;      // tune via 'current_channel_index'


  /*****************************************/
  /*   Processing MANUAL MODE / SEEK MODE  */
  /*****************************************/
  if (system_state == STATE_MANUAL || system_state == STATE_SEEK)
  {
    // read rssi
    wait_rssi_ready();
    rssi_value = readRSSI();
    FS_BUTTON_DIR = FSButtonDirection();
    
    
    channel_sort_idx = getChannelSortTableIndex(current_channel_index); // get 0...47 index depending of current channel
    if (system_state == STATE_MANUAL) // MANUAL MODE
    {

#ifdef USE_GC9N_OSD
      OSDParams[0] = 3; //this is MANUAL MODE
#endif

      // handling of keys
      if ( digitalRead(buttonUp) == LOW  || FS_BUTTON_DIR == 1 )       // channel UP
      {
        time_screen_saver = millis();
        beep(50); // beep & debounce
        delay(KEY_DEBOUNCE); // debounce
        FS_BUTTON_DIR=0;
        current_channel_index++;
        channel_sort_idx++;
        if (channel_sort_idx > CHANNEL_MAX)
          channel_sort_idx = CHANNEL_MIN;
        if (current_channel_index > CHANNEL_MAX_INDEX)
          current_channel_index = CHANNEL_MIN_INDEX;
        chanChangedSaveFlag = true;    //channel changed and needs to be saved
      }
      if ( digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2 ) // channel DOWN
      {
        time_screen_saver = millis();
        beep(50); // beep & debounce
        delay(KEY_DEBOUNCE); // debounce
        FS_BUTTON_DIR=0;
        current_channel_index--;
        channel_sort_idx--;
        if (channel_sort_idx < CHANNEL_MIN)
          channel_sort_idx = CHANNEL_MAX;
        if (current_channel_index > CHANNEL_MAX_INDEX) // negative overflow
          current_channel_index = CHANNEL_MAX_INDEX;
        chanChangedSaveFlag = true;    //channel changed and needs to be saved
      }

#ifdef USE_GC9N_OSD
      if (OSDParams[1] != getCurrentChannelInMhz())
      {
        OSDParams[1] = getCurrentChannelInMhz();
        SendToOSD(); //UPDATE OSD
      }
#endif

      if (!settings_orderby_channel)
      { // order by frequency
        current_channel_index = getChannelSortTableEntry(channel_sort_idx);
      }

    }

    // handling for seek mode after screen and RSSI has been fully processed
    if (system_state == STATE_SEEK) //
    { // SEEK MODE

#ifdef USE_GC9N_OSD
      OSDParams[0] = 2; //this is AUTO MODE
#endif

      // recalculate rssi_seek_threshold
      ((int)((float)rssi_best * (float)(RSSI_SEEK_TRESHOLD / 100.0)) > rssi_seek_threshold) ? (rssi_seek_threshold = (int)((float)rssi_best * (float)(RSSI_SEEK_TRESHOLD / 100.0))) : false;

      if (!seek_found) // search if not found
      {
        if ((!force_seek) && (rssi_value > rssi_seek_threshold)) // check for found channel
        {
          seek_found = 1;
          time_screen_saver = millis();
          // beep twice as notice of lock
         // beep(100);
         // delay(100);
        //  beep(100);
        }
        else
        { // seeking itself
          force_seek = 0;
          // next channel
          channel_sort_idx += seek_direction;
          if (channel_sort_idx > CHANNEL_MAX)
          {
            // calculate next pass new seek threshold
            rssi_seek_threshold = (int)((float)rssi_best * (float)(RSSI_SEEK_TRESHOLD / 100.0));
            channel_sort_idx = CHANNEL_MIN;
            rssi_best = 0;
          }
          else if (channel_sort_idx < CHANNEL_MIN)
          {
            // calculate next pass new seek threshold
            rssi_seek_threshold = (int)((float)rssi_best * (float)(RSSI_SEEK_TRESHOLD / 100.0));
            channel_sort_idx = CHANNEL_MAX;
            rssi_best = 0;
          }
          rssi_seek_threshold = rssi_seek_threshold < 5 ? 5 : rssi_seek_threshold; // make sure we are not stopping on everyting
          current_channel_index = getChannelSortTableEntry(channel_sort_idx);
        }
      }
      else
      { // seek was successful

      }
      FS_BUTTON_DIR = FSButtonDirection();
      if (digitalRead(buttonUp) == LOW || digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2 || FS_BUTTON_DIR == 1) // restart seek if key pressed
      {
        if (digitalRead(buttonUp) == LOW  || FS_BUTTON_DIR == 1 ) {
          seek_direction = 1;
        }
        else
        {
          seek_direction = -1;
        }
        beep(50); // beep & debounce
        FS_BUTTON_DIR = 0;
        delay(KEY_DEBOUNCE); // debounce
        force_seek = 1;
        seek_found = 0;
        time_screen_saver = 0;
      }

#ifdef USE_GC9N_OSD
      SendToOSD(); //UPDATE OSD
#endif
    }
    // change to screensaver after lock and 5 seconds has passed.
    if (((time_screen_saver + 5000 < millis()) && (time_screen_saver != 0) && (rssi_value > 50)) ||
        ((time_screen_saver != 0 && time_screen_saver + (SCREENSAVER_TIMEOUT * 1000) < millis()))) {
      system_state = STATE_SCREEN_SAVER;
#ifdef USE_GC9N_OSD
      OSDParams[0] = -99; // CLEAR OSD
      SendToOSD(); //UPDATE OSD
#endif
    }
  
    //teza
    if (last_channel_index != current_channel_index || stateManualInitFlag)
    {
      drawScreen.updateSeekMode(system_state, current_channel_index, channel_sort_idx, rssi_value, getCurrentChannelInMhz(), rssi_seek_threshold, seek_found);
#ifdef USE_GC9N_OSD
      OSDParams[1] = getCurrentChannelInMhz();
      SendToOSD(); //UPDATE OSD
#endif
      stateManualInitFlag = false;
    }
  }

  /****************************/
  /*   Processing SCAN MODE   */
  /****************************/
  else if (system_state == STATE_SCAN || system_state == STATE_RSSI_SETUP)
  {
#ifdef USE_GC9N_OSD
    OSDParams[0] = -1; //N/A for the moment
    SendToOSD(); //UPDATE OSD
#endif

    // force tune on new scan start to get right RSSI value
    if (scan_start)
    {
      scan_start = 0;
      setChannelByIdx(current_channel_index);
      last_channel_index = current_channel_index;
    }

    // print bar for spectrum
    wait_rssi_ready();
    // value must be ready
    rssi_value = readRSSI();

    if (system_state == STATE_SCAN)
    {
      if (rssi_value > RSSI_SEEK_TRESHOLD)
      {
        if (rssi_best < rssi_value) {
          rssi_best = rssi_value;
        }
      }
    }

    uint16_t bestChannelName = channelIndexToName(current_channel_index);
    uint16_t bestChannelFrequency = getCurrentChannelInMhz();

    drawScreen.updateBandScanMode((system_state == STATE_RSSI_SETUP), channel_sort_idx, rssi_value, bestChannelName, bestChannelFrequency, rssi_setup_min_a, rssi_setup_max_a);

    // next channel
    if (channel_sort_idx < CHANNEL_MAX)
    {
      channel_sort_idx++;
    }
    else
    {
      channel_sort_idx = CHANNEL_MIN;
      if (system_state == STATE_RSSI_SETUP)
      {
        if (!rssi_setup_run--)
        {
          // setup done
          rssi_min_a = rssi_setup_min_a;
          rssi_max_a = rssi_setup_max_a;
          if (rssi_max_a < 125) { // user probably did not turn on the VTX during calibration
            rssi_max_a = RSSI_MAX_VAL;
          }
          // save 16 bit
          EEPROM_WRITE_WORD(EEPROM_ADRW_RSSI_MIN_A, rssi_min_a);
          // save 16 bit
          EEPROM_WRITE_WORD(EEPROM_ADRW_RSSI_MAX_A, rssi_max_a);

#ifdef USE_DIVERSITY

          if (isDiversity()) { // only calibrate RSSI B when diversity is detected.
            rssi_min_b = rssi_setup_min_b;
            rssi_max_b = rssi_setup_max_b;
            if (rssi_max_b < 125) { // user probably did not turn on the VTX during calibration
              rssi_max_b = RSSI_MAX_VAL;
            }
            // save 16 bit
            EEPROM_WRITE_WORD(EEPROM_ADRW_RSSI_MIN_B, rssi_min_b);
            // save 16 bit
            EEPROM_WRITE_WORD(EEPROM_ADRW_RSSI_MAX_B, rssi_max_b);
          }
#endif
          system_state = EEPROM.read(EEPROM_ADR_STATE);
          beep(1000);
        }
      }
    }
    // new scan possible by press scan
    FS_BUTTON_DIR = FSButtonDirection();
    if (digitalRead(buttonUp) == LOW ||  FS_BUTTON_DIR == 1) // force new full new scan
    {
      beep(50); // beep & debounce
      delay(KEY_DEBOUNCE); // debounce
      last_state = 255; // force redraw by fake state change ;-)
      channel_sort_idx = CHANNEL_MIN;
      scan_start = 1;
      rssi_best = 0;
      FS_BUTTON_DIR = 0;

    }
    // update index after channel change
    current_channel_index = getChannelSortTableEntry(channel_sort_idx);
  }

  /****************************/
  /*      SETUP_MENU   */
  /****************************/
  if (system_state == STATE_SETUP_MENU)
  {
    // simple menu
    char menu_id = 0;
    in_menu = 1;
    drawScreen.setupMenu();
#ifdef USE_GC9N_OSD
    OSDParams[0] = 1; //this is main menu
    OSDParams[1] = menu_id + 1; //By defaut=lt goes over manual
    SendToOSD();
#endif
    int editing = -1;
    do {
      in_menu_time_out = 50;
      drawScreen.updateSetupMenu(menu_id, settings_beeps, settings_orderby_channel, call_sign, editing);
      while (--in_menu_time_out && ((digitalRead(buttonMode) == HIGH) && (digitalRead(buttonUp) == HIGH) && (digitalRead(buttonDown) == HIGH)  && FSButtonDirection() == 0)) // wait for next key press or time out
      {
        delay(100); // timeout delay
      }

      if (in_menu_time_out <= 0 ) {
        system_state = state_last_used;
        break; // Timed out, Don't save...
      }

      if (digitalRead(buttonMode) == LOW)       // channel UP
      {
        // do something about the users selection
        switch (menu_id) {
          case 0: // Channel Order Channel/Frequency
            settings_orderby_channel = !settings_orderby_channel;
            break;
          case 1:// Beeps enable/disable
            settings_beeps = !settings_beeps;
            break;

          case 2:// Edit Call Sign
            editing++;
            if (editing > 9) {
              editing = -1;
            }
            break;
          case 3:// Calibrate RSSI
            in_menu = 0;
            for (uint8_t loop = 0; loop < 10; loop++)
            {
#define RSSI_SETUP_BEEP 25
              beep(RSSI_SETUP_BEEP); // beep & debounce
              delay(RSSI_SETUP_BEEP); // debounce
            }
            system_state = STATE_RSSI_SETUP;
            break;
          case 4:
            in_menu = 0; // save & exit menu
            system_state = STATE_SAVE;
            break;
        }
      }
      else if (digitalRead(buttonUp) == LOW  || FS_BUTTON_DIR == 1) {
        if (editing == -1) {
          menu_id--;

        }
        else { // change current letter in place
          call_sign[editing]++;
          call_sign[editing] > '}' ? call_sign[editing] = ' ' : false; // loop to oter end
        }

      }
      else if (digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2) {
        if (editing == -1) {
          menu_id++;


        }
        else { // change current letter in place
          call_sign[editing]--;
          call_sign[editing] < ' ' ? call_sign[editing] = '}' : false; // loop to oter end
        }
      }

      if (menu_id > 4) {
        menu_id = 0;
      }
      if (menu_id < 0) {
        menu_id = 4;
      }

#ifdef USE_GC9N_OSD
      OSDParams[0] = 1; //this is main menu
      OSDParams[1] = menu_id + 1; //By defaut=lt goes over manual
      SendToOSD();
#endif
      beep(50); // beep & debounce
      do {
        delay(150);// wait for button release

      }
      while (editing == -1 && (digitalRead(buttonMode) == LOW || digitalRead(buttonUp) == LOW || digitalRead(buttonDown) == LOW || FSButtonDirection() == 1 || FSButtonDirection() == 2));
    }
    while (in_menu);
  }

  // tune to current channel
  setTunerToCurrentChannel();

}


/*###########################################################################*/
/*******************/
/*   SUB ROUTINES  */
/*******************/

//Sets the tuner to the value specified by the 'current_channel' variables.
void setTunerToCurrentChannel()
{
  if (current_channel_index != last_channel_index ||
      current_channel_mhz != last_channel_mhz)
  {
    if (current_channel_index == tracking_channel_index &&
        current_channel_mhz > 0)
    {  //tune is by freq in MHz
      setChannelByFreq(current_channel_mhz);
      last_channel_mhz = current_channel_mhz;
    }
    else
    {  //tune is by freq index (band/channel)
      setChannelByIdx(current_channel_index);
      tracking_channel_index = current_channel_index;
      current_channel_mhz = 0;
    }
    last_channel_index = current_channel_index;

    // keep time of tune to make sure that RSSI is stable when required
    set_time_of_tune();
  }
}

//Converts the given channel-index value to a 2-character band/channel
// text value encoded in a 16-bit integer.
// Returns a 16-bit integer with the high byte holding the 'band'
//  character and the low byte holding the 'channel' character.
uint16_t channelIndexToName(uint8_t idx)
{
  uint8_t tblVal = pgm_read_byte_near(channelNameCodes + idx);
  uint8_t chVal = (tblVal & (uint8_t)0x0F) + '0';     //channel
  tblVal >>= 4;                        //band
  if (tblVal >= (uint8_t)0x0A)
    tblVal += ('A'-(uint8_t)0x0A);     //band 'A' thru 'F'
  else
    tblVal += 'K';                     //band 'L' or 'R'
  return (((uint16_t)tblVal) << 8) + chVal;
}

//Returns the frequency in MHz corresponding to the current channel.
uint16_t getCurrentChannelInMhz()
{
  static uint8_t lastChanIdxVal = 255;
  static uint16_t lastChanInMHz = 0;

  // tune is set by MHz
  if (current_channel_mhz > 0)
    return current_channel_mhz;

  // tune is via 'current_channel_index'
  if (current_channel_index != lastChanIdxVal)
  {
    lastChanIdxVal = current_channel_index;
    lastChanInMHz = getChannelFreqTableEntry(lastChanIdxVal);
  }
  return lastChanInMHz;
}

//Saves the currently-tuned channel to EEPROM.
void saveChannelToEEPROM()
{
  static uint8_t lastSavedIdxVal = 255;
  static uint16_t lastSavedMHzVal = 65535;

  if (current_channel_index != lastSavedIdxVal)
  {
    lastSavedIdxVal = current_channel_index;
    EEPROM.write(EEPROM_ADR_CHANIDX, current_channel_index);
  }
  if (current_channel_mhz != lastSavedMHzVal)
  {
    lastSavedMHzVal = current_channel_mhz;
    EEPROM_WRITE_WORD(EEPROM_ADRW_FREQMHZ, current_channel_mhz);
  }
}

void beep(uint16_t time)
{
  digitalWrite(led, HIGH);
  if (settings_beeps) {
    digitalWrite(buzzer, LOW); // activate beep
  }
  delay(time / 2);
  digitalWrite(led, LOW);
  digitalWrite(buzzer, HIGH);
}

void FillTemp_Tune_fav(int FavChannelx)
{
  for (int i = 0; i < 10; i++)
  {
    if ( temp_EEPROM_ADR_TUNE_FAV[i] == 255 && FavChannelx != 255) //not used  gc9n
    {
      temp_EEPROM_ADR_TUNE_FAV[i] = FavChannelx;
      i = 12;
    }

  }
}

#ifdef USE_GC9N_OSD
void SendToOSD() //GC9N
{
  if (settings_OSD == true || OSDParams[0] == -99 )
  {
    char one[5]  ;
    char two[5] ;
    char three[5] ;
    char four[5] ;
    char blank[5] = " " ;
    char combined[20] = {0};


    itoa(OSDParams[0], one, 10);
    itoa(OSDParams[1], two, 10);
    itoa(OSDParams[2], three, 10);
    itoa(OSDParams[3], four, 10);

    char ClearScreen[20] = " CLEAR ";
    char RssiScreen[20] = " DRAWRSSI";

    strcat(combined, OSDCommand);

    if (OSDParams[0] == -99) // CLEARSCREEN
    {
      for (uint8_t i = 0; i < 20; i++)
      {
        combined[i] = ClearScreen[i];
      }
    }

    if (OSDParams[3] == -1) // DRAW_RSSI
    {
      for (uint8_t i = 0; i < 20; i++)
      {
        combined[i] = RssiScreen[i];
      }
    }

    strcat(combined, blank);
    strcat(combined, one);
    strcat(combined, blank);
    strcat(combined, two);
    strcat(combined, blank);
    strcat(combined, three);


    Serial.println(combined);
    OSDParams[3] = 0;

  }
}
#endif

int8_t FSButtonDirection () //gc9n
{
  char dir; //1 UP 2 DOWN
  while ( Serial.available() > 0) {

    dir = Serial.read();
    if (dir == ']')
    {
      FS_BUTTON_DIR = 2;
      return 2;
    }

    else if (dir == '[')
    {
      FS_BUTTON_DIR = 1;
      return 1;
    }

    else
    { FS_BUTTON_DIR = 0;
      return 0;
    }
  }
  return 0;
}
