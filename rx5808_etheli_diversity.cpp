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

  --------------------------------------------------------------------------

  The MIT License (MIT)

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
#define EEPROM_ADRW_FREQMHZ 16         // current freq in MHz, or 0 if chanIdx instead
#define EEPROM_ADRW_CHECKWORD 18       // integrity-check value for EEPROM
#define EEPROM_ADR_CALLSIGN 20
#define EEPROM_ADR_LAST_FAVIDX 60      // index of last favorite used

// address for favs list in EEPROM (array of 2-byte words)
#define EEPROM_ADRA_FAVLIST 64
#define FAV_NUMBER_OF_SLOTS 10         // number of favorite channels

#define CALL_SIGN_SIZE 10              // size of 'call_sign[]' array

#define EEPROM_CHECK_VALUE 0x2719      // EEPROM integrity-check value

// returns true if valid channel index or frequency-in-MHz value
#define IS_FAVENTRY_VALID(val) (((uint16_t)(val) <= CHANNEL_MAX_INDEX ||\
                    ((val) >= MIN_CHANNEL_MHZ && (val) <= MAX_CHANNEL_MHZ)))


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
void initializeFavorites();
bool addFreqOrIdxToFavs(uint16_t fVal);
boolean deleteCurrentFavEntry();
int getEntryForFavIndex(uint8_t fIdx);
void nextOrPrevFavEntry(boolean nextFlag);
int getFavIndexForFreqOrIdx(uint16_t fVal);
#ifdef USE_GC9N_OSD
void SendToOSD();
#endif
int8_t fsButtonDirection();
void writeWordToEeprom(int addr, uint16_t val);
uint16_t readWordFromEeprom(int addr);

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
static bool seek_forward_flag = true;
static unsigned long time_screen_saver = 0;
static uint8_t seek_found = 0;
static uint8_t last_seek_rssi = 0;
static uint8_t scan_start = 0;
static bool updateSeekScreenFlag = false;
static uint8_t play_startup_beeps = 1;

static char call_sign[CALL_SIGN_SIZE];
static bool settings_beeps = true;
#ifdef USE_GC9N_OSD
static bool settings_OSD = false;
#endif
static bool settings_orderby_channel = true;
static bool favModeInProgressFlag = false;
//static bool FATSHARK_BUTTON_PUSHED=false;
//static int  LAST_FATSHARK_BUTTON_STATE=0;
static byte FS_BUTTON_DIR = 0;

static uint8_t currentFavoritesCount = 0;
static uint8_t currentFavoritesIndex = 0;
static bool chanChangedSaveFlag = false;
static bool previousUpDnButtonFlag = false;
static bool fromScreenSaverFlag = false;


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

  // check EEPROM-integrity value; write defaults if mismatch
  if (readWordFromEeprom(EEPROM_ADRW_CHECKWORD) != EEPROM_CHECK_VALUE)
  {

    for (int i=0; i<=255; ++i)
      EEPROM.write(i, (uint8_t)255);

    EEPROM.write(EEPROM_ADR_STATE, START_STATE);
    EEPROM.write(EEPROM_ADR_CHANIDX, CHANNEL_MIN_INDEX);
    writeWordToEeprom(EEPROM_ADRW_FREQMHZ, 0);
    EEPROM.write(EEPROM_ADR_BEEP, settings_beeps);
    EEPROM.write(EEPROM_ADR_LAST_FAVIDX, 0);
#ifdef USE_GC9N_OSD
    EEPROM.write(EEPROM_ADR_OSD, settings_OSD);
#else
    EEPROM.write(EEPROM_ADR_OSD, false);
#endif
    EEPROM.write(EEPROM_ADR_ORDERBY, settings_orderby_channel);
    // save 16 bit
    writeWordToEeprom(EEPROM_ADRW_RSSI_MIN_A, RSSI_MIN_VAL);
    // save 16 bit
    writeWordToEeprom(EEPROM_ADRW_RSSI_MAX_A, RSSI_MAX_VAL);

    // save default call sign
    strncpy(call_sign, CALL_SIGN, CALL_SIGN_SIZE); // load callsign
    for (uint8_t i = 0; i < sizeof(call_sign); i++)
    {
      EEPROM.write(EEPROM_ADR_CALLSIGN + i, call_sign[i]);
    }

#ifdef USE_DIVERSITY
    // diversity
    EEPROM.write(EEPROM_ADR_DIVERSITY, diversity_mode);
    // save 16 bit
    writeWordToEeprom(EEPROM_ADRW_RSSI_MIN_B, RSSI_MIN_VAL);
    // save 16 bit
    writeWordToEeprom(EEPROM_ADRW_RSSI_MAX_B, RSSI_MAX_VAL);
#endif

    // write EEPROM-integrity check value
    writeWordToEeprom(EEPROM_ADRW_CHECKWORD, EEPROM_CHECK_VALUE);
  }

  // read saved settings from EEPROM
  system_state = EEPROM.read(EEPROM_ADR_STATE);
  state_last_used = system_state;
  current_channel_index = EEPROM.read(EEPROM_ADR_CHANIDX);
  if (current_channel_index > CHANNEL_MAX_INDEX)
  {
    current_channel_index = 0;
    EEPROM.write(EEPROM_ADR_CHANIDX, 0);
  }
  channel_sort_idx = getChannelSortTableIndex(current_channel_index);
  tracking_channel_index = current_channel_index;
  current_channel_mhz = readWordFromEeprom(EEPROM_ADRW_FREQMHZ);
  if (current_channel_mhz < MIN_CHANNEL_MHZ ||
      current_channel_mhz > MAX_CHANNEL_MHZ)
  {
    current_channel_mhz = 0;
    writeWordToEeprom(EEPROM_ADRW_FREQMHZ, 0);
  }

  // set the channel as soon as we can for faster boot up times
  setTunerToCurrentChannel();

  // initialize 'favorites' variables
  initializeFavorites();

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

  rssi_min_a = readWordFromEeprom(EEPROM_ADRW_RSSI_MIN_A);
  rssi_max_a = readWordFromEeprom(EEPROM_ADRW_RSSI_MAX_A);
#ifdef USE_DIVERSITY
  diversity_mode = EEPROM.read(EEPROM_ADR_DIVERSITY);
  rssi_min_b = readWordFromEeprom(EEPROM_ADRW_RSSI_MIN_B);
  rssi_max_b = readWordFromEeprom(EEPROM_ADRW_RSSI_MAX_B);
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

    // reset flag so favorites mode will "restart"
    favModeInProgressFlag = false;

    in_menu = 0;
    in_menu_time_out = 50; // 20x 100ms = 5 seconds
    /*
      Enter Mode menu
      Show current mode
      Change mode by MODE key
      Any Mode will refresh screen
      If not MODE changes in 2 seconds, it uses last used mode
    */
    char tracked_menu_id;
    do
    {
      // init tracker for item to be selected when menu resumed later on
      tracked_menu_id = last_state_menu_id;

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

      if (!in_menu)
      {  //first time through in-menu loop
              //if press while manual-mode showing then jump to screen saver:
        if (system_state == STATE_MANUAL && !fromScreenSaverFlag)
        {
          system_state = STATE_SCREEN_SAVER;
          fromScreenSaverFlag = true;
          break;
        }
        in_menu = 1;    //set flag to continue in-menu loop
      }

      switch (menu_id)
      {
        case 0: // AUTO MODE
          system_state = STATE_SEEK;
          tracked_menu_id = menu_id;
          break;
        case 1: // Band Scanner
          system_state = STATE_SCAN;
          scan_start = 1;
          break;
        case 2: // manual mode
          system_state = STATE_MANUAL;
          tracked_menu_id = menu_id;
          break;
        case 3: // Set freq by MHz
          system_state = STATE_FREQ_BYMHZ;
          tracked_menu_id = menu_id;
          break;
        case 4: // Favorites Menu       //gc9n
          system_state = STATE_FAVORITE;       //gc9n
          tracked_menu_id = menu_id;
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

      while (digitalRead(buttonMode) == LOW || digitalRead(buttonUp) == LOW || digitalRead(buttonDown) == LOW  || fsButtonDirection() == 1 || fsButtonDirection() == 2)
      {
        // wait for MODE release
        in_menu_time_out = 50;
      }
      while (--in_menu_time_out && ((digitalRead(buttonMode) == HIGH) && (digitalRead(buttonUp) == HIGH) && (digitalRead(buttonDown) == HIGH) &&  fsButtonDirection() == 0  )) // wait for next key press or time out
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
          if (digitalRead(buttonMode) != LOW)
          {  //button not pressed; timeout
            if (state_last_used != STATE_SCAN)
              system_state = state_last_used; // exit to last state on timeout.
            else
            {  //last state was bandscan; resume to manual or byMHz mode
              system_state = (last_state_menu_id != 3) ? STATE_MANUAL :
                                                         STATE_FREQ_BYMHZ;;
            }
          }
          else  //button pressed; set id for item to be selected when menu resumed
          {
            last_state_menu_id = tracked_menu_id;
                   //if auto/seek menu item selected then always start new seek
                   // (but if timeout then will depend on 'seek_found')
            if (system_state == STATE_SEEK)
            {
              force_seek = 1;
              if (seek_found)
                seek_found = 0;
              else  //if "new" seek then init 'last-rssi' value
                last_seek_rssi = 0;
            }
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
        scan_start = 1;
        drawScreen.bandScanMode(system_state);
        break;
      case STATE_SEEK:    // seek mode
      case STATE_MANUAL:  // manual mode
        if (system_state == STATE_MANUAL)
        {
          time_screen_saver = millis();
          if (system_state != state_last_used)
          {
            // save so state is resumed after restart
            EEPROM.write(EEPROM_ADR_STATE, system_state);
          }
              //if coming from scan or seek mode then restore previous channel:
          if (state_last_used == STATE_SCAN || state_last_used == STATE_SEEK ||
              last_state == STATE_RSSI_SETUP)
          {
            current_channel_index = EEPROM.read(EEPROM_ADR_CHANIDX);
            channel_sort_idx = getChannelSortTableIndex(current_channel_index);
            current_channel_mhz = 0;
          }
          else if (state_last_used == STATE_FREQ_BYMHZ)
          {     //if previous mode was BY-MHZ then tune now via channel index
            current_channel_mhz = 0;
          }
        }
        else //if (system_state == STATE_SEEK)
        {
          time_screen_saver = 0; // dont show screen saver until we found a channel.
        }
        drawScreen.seekMode(system_state);  //draw initial manual/seek screen
        updateSeekScreenFlag = true;        //update screen when entering mode
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
        if (last_state != STATE_SETUP_MENU  && state_last_used != STATE_FAVORITE )
        {  //didn't come from menu setup; save favorite
          bool newFlag = addFreqOrIdxToFavs((current_channel_mhz == 0) ?
                             current_channel_index : current_channel_mhz);
#ifdef USE_GC9N_OSD
          OSDParams[0] = -2; //this is save
          OSDParams[1] = 0; //By default goes over manual
          SendToOSD(); //UPDATE OSD
#endif
          if (newFlag)
          {  //new favorite added
            drawScreen.save(state_last_used, current_channel_index,
                          getCurrentChannelInMhz(), call_sign,
                          currentFavoritesIndex + 1);
          }
          else
          {  //duplicate favorite
            drawScreen.FavSel(currentFavoritesIndex + 1);
            delay(800);
            system_state = state_last_used; // return to saved function
            force_menu_redraw = 1; // we change the state twice, must force redraw of menu
            break;
          }
        }
        else if (last_state == STATE_SETUP_MENU)
        {
#ifdef USE_GC9N_OSD
          OSDParams[0] = -2; //this is save
          OSDParams[1] = 0; //By default goes over manual
          SendToOSD(); //UPDATE OSD
#endif
          drawScreen.save(state_last_used, current_channel_index,
                          getCurrentChannelInMhz(), call_sign, -99);
        }

        // long press in favorites will delete the current favorite channel
        if  (state_last_used == STATE_FAVORITE && last_state == 255)
        {
#ifdef USE_GC9N_OSD
          OSDParams[0] = -3; //this is delete
          OSDParams[1] = 0; //By default goes over manual
          SendToOSD(); //UPDATE OSD
#endif

          if (currentFavoritesIndex < currentFavoritesCount)
          {  //favorites list is not empty and index is OK
            drawScreen.FavDelete(getCurrentChannelInMhz(),
                                 currentFavoritesIndex + 1);
            delay(500);
            if (deleteCurrentFavEntry())
            {  //list still contains entries
              int fVal;     //get current favorite (freq or freq index)
              if ((fVal=getEntryForFavIndex(currentFavoritesIndex)) >= 0)
              {
                if (fVal <= 255)
                {  //frequency index
                  current_channel_index = (uint8_t)fVal;
                  current_channel_mhz = 0;
                  channel_sort_idx = getChannelSortTableIndex(current_channel_index);
                }
                else  //frequency in MHz
                  current_channel_mhz = fVal;
                saveChannelToEEPROM();
              }
              drawScreen.FavSel(currentFavoritesIndex + 1);
            }
            //drawScreen.screenSaver(diversity_mode, channelIndexToName(channelIndex), getCurrentChannelInMhz(), call_sign);
          }
        }

        for (uint8_t loop = 0; loop < 5; loop++)
        {
          beep(100); // beep
          delay(100);
        }
        delay(300);
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
          // save so state is resumed after restart
          EEPROM.write(EEPROM_ADR_STATE, system_state);

          // if coming from scan or seek mode then restore previous channel
          if (state_last_used == STATE_SCAN || state_last_used == STATE_SEEK ||
              last_state == STATE_RSSI_SETUP)
          {
            current_channel_index = EEPROM.read(EEPROM_ADR_CHANIDX);
                   //set tracking equal so tune is via 'current_channel_mhz':
            tracking_channel_index = current_channel_index;
            current_channel_mhz = readWordFromEeprom(EEPROM_ADRW_FREQMHZ);
          }
          state_last_used = system_state;
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
      uint8_t rssi_value = readRSSI();

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
    fromScreenSaverFlag = true;

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
    uint8_t in_div_menu = 1;
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
      while ((digitalRead(buttonMode) == HIGH) && (digitalRead(buttonUp) == HIGH) && (digitalRead(buttonDown) == HIGH) &&  fsButtonDirection() == 0 ); // wait for next mode or time out

      if (digitalRead(buttonMode) == LOW)       // channel UP
      {
        in_div_menu = 0; // exit menu
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
    while (in_div_menu);

    system_state = state_last_used;
  }
#endif

  /*****************************************/
  /*   Processing FAVORITES                */
  /*****************************************/
  if (system_state == STATE_FAVORITE )
  {
    if (currentFavoritesCount > (uint8_t)0)
    {  //favorites list is not empty
      if (favModeInProgressFlag)
      {  //did not just enter mode
        // read rssi
        wait_rssi_ready();
        uint8_t rssi_value = readRSSI();
        time_screen_saver = millis();

        // handling of keys
        bool upFlag = (digitalRead(buttonUp) == LOW || FS_BUTTON_DIR == 1);    // channel UP
        bool dnFlag = (digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2);  // channel DOWN

        if ((upFlag || dnFlag) && !(upFlag && dnFlag))
        {  //UP or DOWN key pressed (but not both)
          //switch to next or previous entries in favorites list
          nextOrPrevFavEntry(upFlag);
          time_screen_saver = millis();
          int fVal;       //get current favorite (freq or freq index)
          if ((fVal=getEntryForFavIndex(currentFavoritesIndex)) >= 0)
          {
            if (fVal <= 255)
            {  //frequency index
              current_channel_index = (uint8_t)fVal;
              current_channel_mhz = 0;
              channel_sort_idx = getChannelSortTableIndex(current_channel_index);
            }
            else
            {  //frequency in MHz
              current_channel_mhz = fVal;
                        //set table index to nearest entry:
              current_channel_index = freqInMhzToNearestFreqIdx(
                                               current_channel_mhz, upFlag);
                        //keep sort index in sync:
              channel_sort_idx = getChannelSortTableIndex(current_channel_index);
                        //set tracking equal so tune is via 'current_channel_mhz':
              tracking_channel_index = current_channel_index;
            }
            setTunerToCurrentChannel();     //tune now so not delayed
            saveChannelToEEPROM();
          }
          drawScreen.FavSel(currentFavoritesIndex + 1);
          delay(KEY_DEBOUNCE); // debounce
          beep(50); // beep & debounce
          delay(KEY_DEBOUNCE); // debounce
        }
        system_state = STATE_SCREEN_SAVER;
      }
      else
      {  //mode was just entered
        EEPROM.write(EEPROM_ADR_STATE, STATE_FAVORITE);
        favModeInProgressFlag = true;
              //get current channel index or frequency in MHz value:
        int fVal = (current_channel_mhz == 0) ?
                               current_channel_index : current_channel_mhz;
              //if current chan/freq matches an existing favorite then
              // use that favorite; otherwise use current/last favorite:
        int idx;
        if ((idx=getFavIndexForFreqOrIdx(fVal)) >= 0)
          currentFavoritesIndex = idx;
        else
          fVal = getEntryForFavIndex(currentFavoritesIndex);
              //set variables to tune to favorite:
        if (fVal >= 0)
        {
          if (fVal <= 255)
          {  //frequency index
            current_channel_index = (uint8_t)fVal;
            current_channel_mhz = 0;
            channel_sort_idx = getChannelSortTableIndex(current_channel_index);
          }
          else  //frequency in MHz
            current_channel_mhz = fVal;
          saveChannelToEEPROM();
        }
              //show favorite ID value to user:
        drawScreen.FavSel(currentFavoritesIndex + 1);
        delay(500);
      }
    }
    else
    {  //favorites list is empty
      drawScreen.NoFav();
      delay(1000);
      system_state = STATE_SCREEN_SAVER;
            //revert to non-Favorites mode:
      state_last_used = (current_channel_mhz == 0) ? STATE_MANUAL :
                                                     STATE_FREQ_BYMHZ;
      EEPROM.write(EEPROM_ADR_STATE, state_last_used);
      last_state_menu_id = (state_last_used == STATE_MANUAL) ? 2 : 3;
    }

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

    if ((upFlag || dnFlag) && !(upFlag && dnFlag))
    {  //UP or DOWN key pressed (but not both)
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
      // keep sort index in sync
      channel_sort_idx = getChannelSortTableIndex(current_channel_index);

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


  /*****************************************/
  /*   Processing MANUAL MODE / SEEK MODE  */
  /*****************************************/
  else if (system_state == STATE_MANUAL || system_state == STATE_SEEK)
  {
    if (current_channel_mhz > 0)
    {  //currently in ByMHz mode; need to tune via 'current_channel_index'
      current_channel_mhz = 0;
      setTunerToCurrentChannel();
      chanChangedSaveFlag = true;      //channel changed and needs to be saved
    }
    // read rssi
    wait_rssi_ready();
    uint8_t rssi_value = readRSSI();
    FS_BUTTON_DIR = fsButtonDirection();
    
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
    else if (system_state == STATE_SEEK) //
    { // SEEK MODE

#ifdef USE_GC9N_OSD
      OSDParams[0] = 2; //this is AUTO MODE
#endif

      if (!seek_found) // search if not found
      {
        // if seek was not just initiated then check if RSSI level is high
        //  enough for 'found' channel (and beyond previous 'found' channel)
        if ((!force_seek) && rssi_value > RSSI_SEEK_TRESHOLD &&
                                       last_seek_rssi <= RSSI_SEEK_TRESHOLD)
        {
          seek_found = 1;
              //check if next channels have higher RSSI
          char chkIdx = channel_sort_idx;
          uint8_t preChIdx = current_channel_index;
          uint8_t chkRssiVal;
          while (true)
          {
            if (seek_forward_flag)
            {
              if (++chkIdx > CHANNEL_MAX)
                break;
            }
            else
            {
              if (--chkIdx < CHANNEL_MIN)
                break;
            }
            current_channel_index = getChannelSortTableEntry(chkIdx);
            setTunerToCurrentChannel();
            wait_rssi_ready();
            chkRssiVal = readRSSI();
            if (chkRssiVal <= rssi_value)
            {  //next channel does not have higher RSSI
              current_channel_index = preChIdx;
              setTunerToCurrentChannel();
              break;
            }
              //next channel has higher RSSI; accept channel, and loop
            channel_sort_idx = chkIdx;
            preChIdx = current_channel_index;
            rssi_value = chkRssiVal;
            updateSeekScreenFlag = true;    //update channel shown on screen
          }
          time_screen_saver = millis();
          chanChangedSaveFlag = true;  //channel changed and needs to be saved
          // beep twice as notice of lock
         // beep(100);
         // delay(100);
        //  beep(100);
        }
        else
        { // seeking itself
          force_seek = 0;
          // next channel
          if (seek_forward_flag)
          {
            if (++channel_sort_idx > CHANNEL_MAX)
              channel_sort_idx = CHANNEL_MIN;
          }
          else
          {
            if (--channel_sort_idx < CHANNEL_MIN)
              channel_sort_idx = CHANNEL_MAX;
          }
          current_channel_index = getChannelSortTableEntry(channel_sort_idx);
        }
      }
      // else  //seek was successful

      // handling of keys
      bool upFlag = (digitalRead(buttonUp) == LOW || FS_BUTTON_DIR == 1);    // channel UP
      bool dnFlag = (digitalRead(buttonDown) == LOW || FS_BUTTON_DIR == 2);  // channel DOWN

      if ((upFlag || dnFlag) && !(upFlag && dnFlag))
      {  //UP or DOWN key pressed (but not both); restart seek
        seek_forward_flag = upFlag;
        beep(50); // beep & debounce
        FS_BUTTON_DIR = 0;
        delay(KEY_DEBOUNCE); // debounce
        force_seek = 1;
        seek_found = 0;
        time_screen_saver = 0;
      }

      last_seek_rssi = rssi_value;

#ifdef USE_GC9N_OSD
      SendToOSD(); //UPDATE OSD
#endif
    }

    // change to screensaver after lock and 5 seconds has passed.
    if (time_screen_saver != 0 &&
        ((time_screen_saver + 5000 < millis() && rssi_value > 50) ||
        time_screen_saver + (SCREENSAVER_TIMEOUT * 1000) < millis()))
    {
      system_state = STATE_SCREEN_SAVER;
#ifdef USE_GC9N_OSD
      OSDParams[0] = -99; // CLEAR OSD
      SendToOSD(); //UPDATE OSD
#endif
    }
  
    //teza
    if (last_channel_index != current_channel_index || updateSeekScreenFlag)
    {
      drawScreen.updateSeekMode(system_state, current_channel_index, channel_sort_idx, rssi_value, getCurrentChannelInMhz(), RSSI_SEEK_TRESHOLD, seek_found);
#ifdef USE_GC9N_OSD
      OSDParams[1] = getCurrentChannelInMhz();
      SendToOSD(); //UPDATE OSD
#endif
      updateSeekScreenFlag = false;
      if (seek_found)                  //cover case where seek screen restored
        time_screen_saver = millis();  // after fav save; setup timeout
    }

    fromScreenSaverFlag = false;
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
      current_channel_mhz = 0;      // tune via 'current_channel_index'
      setChannelByIdx(current_channel_index);
      last_channel_index = current_channel_index;
    }

    // print bar for spectrum
    wait_rssi_ready();
    // value must be ready
    uint8_t rssi_value = readRSSI();

    uint16_t scanChannelName = channelIndexToName(current_channel_index);
    uint16_t scanChannelFrequency = getCurrentChannelInMhz();

    drawScreen.updateBandScanMode((system_state == STATE_RSSI_SETUP), channel_sort_idx, rssi_value, scanChannelName, scanChannelFrequency, rssi_setup_min_a, rssi_setup_max_a);

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
        {  // setup done
          rssi_min_a = rssi_setup_min_a;
          writeWordToEeprom(EEPROM_ADRW_RSSI_MIN_A, rssi_min_a);
              //if 'max' is close to 'min' then user probably
              // did not turn on the VTX during calibration
          if (rssi_setup_max_a - rssi_setup_min_a >= rssi_setup_min_a/3)
          {  //difference is high enough to use
            rssi_max_a = rssi_setup_max_a;
            writeWordToEeprom(EEPROM_ADRW_RSSI_MAX_A, rssi_max_a);
          }

#ifdef USE_DIVERSITY
          if (isDiversity())
          {  // only calibrate RSSI B when diversity is detected.
            rssi_min_b = rssi_setup_min_b;
            writeWordToEeprom(EEPROM_ADRW_RSSI_MIN_B, rssi_min_b);
              //if 'max' is close to 'min' then user probably
              // did not turn on the VTX during calibration
            if (rssi_setup_max_b - rssi_setup_min_b >= rssi_setup_min_b/3)
            {  //difference is high enough to use
              rssi_max_b = rssi_setup_max_b;
              writeWordToEeprom(EEPROM_ADRW_RSSI_MAX_B, rssi_max_b);
            }
          }
#endif
          system_state = EEPROM.read(EEPROM_ADR_STATE);
          beep(1000);
        }
      }
    }
    // new scan possible by press scan
    FS_BUTTON_DIR = fsButtonDirection();
    if (digitalRead(buttonUp) == LOW ||  FS_BUTTON_DIR == 1) // force new full new scan
    {
      beep(50); // beep & debounce
      delay(KEY_DEBOUNCE); // debounce
      last_state = 255; // force redraw by fake state change ;-)
      channel_sort_idx = CHANNEL_MIN;
      scan_start = 1;
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
      while (--in_menu_time_out && ((digitalRead(buttonMode) == HIGH) && (digitalRead(buttonUp) == HIGH) && (digitalRead(buttonDown) == HIGH)  && fsButtonDirection() == 0)) // wait for next key press or time out
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
      while (editing == -1 && (digitalRead(buttonMode) == LOW || digitalRead(buttonUp) == LOW || digitalRead(buttonDown) == LOW || fsButtonDirection() == 1 || fsButtonDirection() == 2));
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
    }
    else
    {  //tune is by freq index (band/channel)
      setChannelByIdx(current_channel_index);
      tracking_channel_index = current_channel_index;
      current_channel_mhz = 0;
    }
    last_channel_index = current_channel_index;
    last_channel_mhz = current_channel_mhz;

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
    writeWordToEeprom(EEPROM_ADRW_FREQMHZ, current_channel_mhz);
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


//Initializes the 'currentFavoritesCount' and 'currentFavoritesIndex'
// variables.
void initializeFavorites()
{
  uint16_t wordVal;
  uint8_t btIdx = 0;
  do
  {           // find first unused slot
    wordVal = readWordFromEeprom(EEPROM_ADRA_FAVLIST + (btIdx*(uint16_t)2));
    if (!IS_FAVENTRY_VALID(wordVal))
      break;
  }
  while (++btIdx < FAV_NUMBER_OF_SLOTS);
  currentFavoritesCount = btIdx;
  if (currentFavoritesCount > (uint8_t)0)
  {
    currentFavoritesIndex = EEPROM.read(EEPROM_ADR_LAST_FAVIDX);
    if (currentFavoritesIndex >= currentFavoritesCount)
      currentFavoritesIndex = 0;
  }
}


//Adds the given frequency index or MHz value to the favorites list.
// Returns true if added; false if duplicate of existing favorite.
bool addFreqOrIdxToFavs(uint16_t fVal)
{
  int idx;

  // check if matches existing entry
  if ((idx=getFavIndexForFreqOrIdx(fVal)) >= 0)
  {  //match found; select as current favorite
    currentFavoritesIndex = (uint8_t)idx;
    EEPROM.write(EEPROM_ADR_LAST_FAVIDX, currentFavoritesIndex);
    return false;
  }

  // search for next unused favorites slot
  uint16_t wordVal;
  uint8_t btIdx;
  idx = 0;
  while (true)
  {
    wordVal = readWordFromEeprom(EEPROM_ADRA_FAVLIST + (idx*2));
    if (!IS_FAVENTRY_VALID(wordVal))
    {  //unused slot found; use it
      btIdx = (uint8_t)idx;
      ++currentFavoritesCount;         //keep track of favs count
      break;
    }
    if (++idx >= FAV_NUMBER_OF_SLOTS)
    {  //no unused slots found; reuse next slot after current favorite
      btIdx = currentFavoritesIndex;
      if (++btIdx >= FAV_NUMBER_OF_SLOTS)
        btIdx = 0;
      break;
    }
  }

  // enter given value into favorites slot
  writeWordToEeprom(EEPROM_ADRA_FAVLIST + (btIdx*(uint16_t)2), fVal);
  currentFavoritesIndex = btIdx;
  EEPROM.write(EEPROM_ADR_LAST_FAVIDX, btIdx);
  return true;
}


//Deletes the current favorite from the list.  Entries after the deleted
// entry are shifted down.
// Returns true if the favorites list still contains entries; false if
//  the list is now empty.
boolean deleteCurrentFavEntry()
{
  uint8_t btIdx = currentFavoritesIndex;
  if (btIdx < FAV_NUMBER_OF_SLOTS)
  {
    // scan entries after the one to be deleted
    uint16_t wordVal;
    uint16_t idx = btIdx;
    while(++idx < FAV_NUMBER_OF_SLOTS)
    {
      wordVal = readWordFromEeprom(EEPROM_ADRA_FAVLIST + (idx*2));
      if (IS_FAVENTRY_VALID(wordVal))
      {  //slot not empty; shift value down one
        writeWordToEeprom(EEPROM_ADRA_FAVLIST + ((idx-1)*2), wordVal);
      }
      else
        break;
    }
    --idx;
    // clear last slot that contained a value
    writeWordToEeprom(EEPROM_ADRA_FAVLIST + (idx*2), (uint16_t)0xFFFF);
    currentFavoritesCount = (uint8_t)idx;   //keep track of favs count

    // if last used slot was deleted then update current favorite index
    if (btIdx == idx)
    {
      if (btIdx == 0)
      {  //was on first slot; list is now empty
        currentFavoritesCount = 0;
        currentFavoritesIndex = 0;
        EEPROM.write(EEPROM_ADR_LAST_FAVIDX, currentFavoritesIndex);
        return false;
      }
      --btIdx;
      currentFavoritesIndex = btIdx;
      EEPROM.write(EEPROM_ADR_LAST_FAVIDX, btIdx);
    }
  }
  return true;
}


//Returns the frequency-index or frequency-in-MHz value for the favorites
// entry for the given favorites index.
int getEntryForFavIndex(uint8_t fIdx)
{
  if (fIdx < currentFavoritesCount)
  {
    uint16_t wordVal = readWordFromEeprom(
                                  EEPROM_ADRA_FAVLIST + (fIdx*(uint16_t)2));
    if (IS_FAVENTRY_VALID(wordVal))
      return (int)wordVal;
  }
  return -1;
}


//Switches the current favorite to the next or previous entry
// in the list.
void nextOrPrevFavEntry(boolean nextFlag)
{
  // if less than two entries in list then just return
  if (currentFavoritesCount < (uint8_t)2)
    return;
  uint8_t btIdx = currentFavoritesIndex;
  if (nextFlag)
  {  //next
    if (++btIdx >= currentFavoritesCount)
      btIdx = 0;
  }
  else
  {  //previous
    if (btIdx > (uint8_t)0)
      --btIdx;
    else
      btIdx = currentFavoritesCount - (uint8_t)1;
  }
  currentFavoritesIndex = btIdx;
  EEPROM.write(EEPROM_ADR_LAST_FAVIDX, currentFavoritesIndex);
}


//Returns the index for the favorites slot matching the given frequency
// index or MHz value, or -1 if no match.
int getFavIndexForFreqOrIdx(uint16_t fVal)
{
  for (int idx=0; idx<currentFavoritesCount; ++idx)
  {
    if (readWordFromEeprom(EEPROM_ADRA_FAVLIST+(idx*2)) == fVal)
      return idx;
  }
  return -1;
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

int8_t fsButtonDirection () //gc9n
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


//Writes 2-byte word to EEPROM at address.
void writeWordToEeprom(int addr, uint16_t val)
{
  EEPROM.write(addr, lowByte(val));
  EEPROM.write(addr+1, highByte(val));
}

//Reads 2-byte word at address from EEPROM.
uint16_t readWordFromEeprom(int addr)
{
  const uint8_t lb = EEPROM.read(addr);
  const uint8_t hb = EEPROM.read(addr+1);
  return (((uint16_t)hb) << 8) + lb;
}
