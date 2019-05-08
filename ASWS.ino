#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <RtcDS3231.h>

#define SERIAL true
#define DISPLAY true

#define U_SOL_PIN A0
#define I_SOL_PIN A1
#define U_BAT_PIN A2
#define BUTTON_PIN 5
#define LOAD_ENABLE_PIN 8
#define BUCK_ENABLE_PIN 9
#define DC_INIT 0.5*buckRes

/* v3-1: */
#define U_SOL_GAIN 20.29/394
#define U_BAT_GAIN 11.47/767
#define I_SOL_OFFSET 1
#define I_SOL_GAIN 430.0/145


/* v3-2:
  #define U_SOL_GAIN 12.63/836
  #define U_BAT_GAIN 11.41/762
  #define I_SOL_OFFSET 0
  #define I_SOL_GAIN 1 */

#define U_UNDERVOLTAGE_CUTOFF 9.0 /* 10.5 */
#define U_UNDERVOLTAGE_RECOVER 11.1 /* 10.8 */
#define U_OVERVOLTAGE_CUTOFF 12.5 /* 12.3 */
#define U_OVERVOLTAGE_RECOVER 11.1 /* 12.0 */
#define U_SOL_IDLE_DAY 12.0
#define U_SOL_IDLE_NIGHT 10.0
#define T_FILT .43
#define U_SOL_DESIRED_PERC_U_ID .8
#define I_SOL_CHARGE_SUFFICIENT_ENABLE 500
#define I_SOL_CHARGE_SUFFICIENT_DISABLE 400
//#define WAIT_AT_MPP 10
#define MIN_STEP_SIZE 1
#define MPPT true
#define U_MPPT_MIN 14.0
#define U_MPPT_MAX 19.0
#define BACK_TO_MPPT_RANGE_STEPSIZE 10

// TASK TIMES
#define DT 200L
#define DT_MPPT_UPDATE 1000L
#define DT_IDLE_VOLTAGE 120000L


boolean loadEnable = false;
boolean buttonActual = false;
boolean buttonPrevious = false;
boolean undervoltageProtection = false;
boolean overvoltageProtection = false;
boolean checkIdleVoltage = false;
boolean nightLoadOff = false;
boolean modeAscending = true;
boolean night = false;
boolean u_sol_idle_is_checked = false;
boolean positive_search_direction = true;

float u_sol = 0.0;
float u_sol_idle = 0.0;
float i_sol = 0.0;
float p_sol = 0.0;
float u_bat = 0.0;
const int numReadings = 5; // rolav length
int u_sol_readIndex = 0;
int u_bat_readIndex = 0;
int u_sol_readings[numReadings]; // for rolav
int u_bat_readings[numReadings]; // for rolav
unsigned long u_sol_total = 0; // for rolav
unsigned long u_bat_total = 0; // for rolav

//float i_bat = 0.0;
//float i_bat_previous = 0.0; // for 1st order low pass filter
float p_bat = 0.0;

unsigned long timerDTs;
unsigned long timerDTsNow;
unsigned long timerIdleVoltage;
unsigned long timerIdleVoltageNow;
unsigned long timerDCUpdate;
unsigned long timerDCUpdateNow;

int dc = 0;
int mode = 0;
int dc_previousIdleVoltageCheck = 0;
int mpptAction = 0;
int stepSize = MIN_STEP_SIZE;
int mpptWait = 0;
int buckRes = 0;

float p_sol_to_compare = 0.0;
float p_sol_dc_increased = 0.0;
float p_sol_dc_decreased = 0.0;

RtcDateTime rtcNow;
RtcTemperature rtcTemp;

//Adafruit_INA219 ina219;

U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 7, /* reset=*/ 6);

RtcDS3231<TwoWire> Rtc(Wire);


void setup() {
  // put your setup code here, to run once:
  dc = DC_INIT;
  dc_previousIdleVoltageCheck = dc;
  //Serial.begin(115200);

  pinMode(LOAD_ENABLE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  pinMode(9, OUTPUT);


  u8g2.begin();

  // initialize RTC

  Rtc.Begin();
  //RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  //Rtc.SetDateTime(compiled);

  // initialize Task Timer
  timerDTs = millis();
  timerIdleVoltage = timerDTs;
  timerDCUpdate = timerDTs;

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    u_sol_readings[thisReading] = 0;
    u_bat_readings[thisReading] = 0;
  }

  if (SERIAL) {
    Serial.begin(115200);
  }
}

void loop() {
  // READ INPUTS, WRITE OUTPUTS, UPDATE DISPLAY every DT seconds ///////////////////////////
  long timerDTsNow = millis();
  if (timerDTsNow - timerDTs >=  DT ) {
    timerDTs = timerDTsNow;

    if (checkIdleVoltage) { //night |
      dc = 0;
    }

    //dc = map(analogRead(A3),0,1023,0,buckRes);

    analogWrite(BUCK_ENABLE_PIN, dc);

    // Read Voltages, Currents, Power, loadEnableManualButton
    u_sol = u_sol_rolav(analogRead(U_SOL_PIN)) * U_SOL_GAIN;
    i_sol = (analogRead(I_SOL_PIN) - I_SOL_OFFSET) * I_SOL_GAIN;
    p_sol = u_sol * i_sol / 1000.0;
    u_bat = u_bat_rolav(analogRead(U_BAT_PIN)) * U_BAT_GAIN;

    buttonActual = digitalRead(BUTTON_PIN);

    // check if it's night
    if (u_sol_idle_is_checked) {
      if (u_sol_idle >= U_SOL_IDLE_DAY & night) {
        night = false;
        dc = DC_INIT;
      }
      if (u_sol_idle < U_SOL_IDLE_NIGHT & !night) {
        night = true;
      }
    }

    // Battery Voltage Check
    if (u_bat <= U_UNDERVOLTAGE_CUTOFF & !undervoltageProtection) {
      undervoltageProtection = true;
      mode = 0;
    }
    if (undervoltageProtection & u_bat > U_UNDERVOLTAGE_RECOVER) {
      undervoltageProtection = false;
    }

    if (u_bat >= U_OVERVOLTAGE_CUTOFF & !overvoltageProtection) {
      overvoltageProtection = true;
    }
    if (u_bat < U_OVERVOLTAGE_RECOVER & overvoltageProtection) {
      overvoltageProtection = false;
    }

    // Parse Enable Button & toggle modes

    if (buttonActual & !buttonPrevious) {
      /*if (mode < 3) {
        if (modeAscending) {
          mode++;
        }
        else {
          mode--;
        }
        }
        if (mode == 3 | mode == -1) {
        mode = 1;
        modeAscending = !modeAscending;
        }*/
      switch (mode) {
        case 0: {
            if (!undervoltageProtection) {
              mode = 2;
            }
            break;
          }
        case 2: {
            mode = 0;
            break;
          }
      }
    }
    buttonPrevious = buttonActual;

    if (mode == 0) {
      loadEnable = false;
    }
    if (mode == 1) {
      // Toggle Load Enable on Idle Solar Voltage
      /*if (!loadEnable  & !night) { //
        loadEnable = true;
        }
        if (loadEnable & night) { // & night
        loadEnable = false;
        }*/
    }
    if (mode == 2) {
      loadEnable = true;
    }

    // Write Load Enable
    if (loadEnable) {
      if (!undervoltageProtection) {
        digitalWrite(LOAD_ENABLE_PIN, HIGH);
      }
      else {
        digitalWrite(LOAD_ENABLE_PIN, LOW);
      }
    }
    else {
      digitalWrite(LOAD_ENABLE_PIN, LOW);
    }

  }

  // track MPP or Battery Maximum Voltage every DT_MPPT_UPDATE seconds /////////////////////
  long timerDCUpdateNow = millis();
  if (timerDCUpdateNow - timerDCUpdate >= DT_MPPT_UPDATE ) {
    timerDCUpdate = timerDCUpdateNow;

    /*if (!night) {
      if (overvoltageProtection & dc == buckRes) {
        overvoltageProtection = false;
      }*/
    //if (!overvoltageProtection) {
    ///  if (u_sol < U_MPPT_MAX & u_sol > U_MPPT_MIN) {
    switch (mpptAction) {
      case 0: {
          p_sol_to_compare = p_sol;
          if (positive_search_direction & dc < buckRes - stepSize) {
            dc += stepSize;
          }
          if (!positive_search_direction & dc > stepSize) {
            dc -= stepSize;
          }
          mpptAction++;
          break;
        }
      case 1: {
          if (p_sol < p_sol_to_compare | dc == buckRes | dc == 0) {
            positive_search_direction = !positive_search_direction;
            stepSize = MIN_STEP_SIZE;
          }
          mpptAction = 0;
          break;
        }
    }
    //}
    /*else {
      if (u_sol < U_MPPT_MIN & dc >= BACK_TO_MPPT_RANGE_STEPSIZE) {
        dc -= BACK_TO_MPPT_RANGE_STEPSIZE;
      }
      if (u_sol > U_MPPT_MAX & dc <= buckRes - BACK_TO_MPPT_RANGE_STEPSIZE) {
        dc += BACK_TO_MPPT_RANGE_STEPSIZE;
      }
      }
      }
      else {
      if (u_bat > U_OVERVOLTAGE_CUTOFF & dc > 0) {
      dc++;
      }
      if (u_bat < U_OVERVOLTAGE_CUTOFF & dc < buckRes) {
      dc--;
      }
      }*/
    //}

    // Get RTC Time and Temperature
    rtcNow = Rtc.GetDateTime();
    rtcTemp = Rtc.GetTemperature();

    // Display Update
    if (DISPLAY) {
      displayUpdate();
    }

    // Serial Update
    if (SERIAL) {
      serialUpdate();
    }

  }

  /*// check idle voltage every DT_IDLE_VOLTAGE seconds //////////////////////////////////////
    long timerIdleVoltageNow = millis();
    if (timerIdleVoltageNow - timerIdleVoltage >= DT_IDLE_VOLTAGE * 1e3 & !checkIdleVoltage) {
    timerIdleVoltage = timerIdleVoltageNow;
    checkIdleVoltage = true;
    dc_previousIdleVoltageCheck = dc;
    }
    if (millis() >= timerIdleVoltage + 1e3 & checkIdleVoltage) {
    checkIdleVoltage = false;
    u_sol_idle = u_sol;
    dc = dc_previousIdleVoltageCheck;
    u_sol_idle_is_checked = true;
    }*/
}

// END LOOP ////////////////////////////////////////////////////////////////////////////////

void serialUpdate() {
  Serial.print("U_SOL = ");
  Serial.print(u_sol);
  Serial.print(" V, I_SOL = ");
  Serial.print(i_sol);
  Serial.print(" mA, U_BAT = ");
  Serial.print(u_bat);
  Serial.print(" V, DC = ");
  Serial.print(dc);
  Serial.print(", P_SOL = ");
  Serial.print(p_sol);
  Serial.println(" W");
}

void displayUpdate() {
  int rows[7] = {7, 16, 25, 34, 43, 52, 61};
  int cols[6] = {0, 24, 50, 64, 88, 114};

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_logisoso32_tf);
    u8g2.setCursor(20, 32);
    u8g2.print(u_bat, 1);
    /*u8g2.setCursor(cols[0], rows[0]);
      u8g2.print("SOLAR:");

      u8g2.setCursor(cols[0], rows[1]);
      u8g2.print("U =");
      u8g2.setCursor(cols[1], rows[1]);
      u8g2.print(u_sol, 1);
      u8g2.setCursor(cols[2], rows[1]);
      u8g2.print("V");

      u8g2.setCursor(cols[0], rows[2]);
      u8g2.print("I =");
      u8g2.setCursor(cols[1], rows[2]);
      u8g2.print(i_sol, 0);
      u8g2.setCursor(cols[2], rows[2]);
      u8g2.print("mA");

      u8g2.setCursor(cols[0], rows[3]);
      u8g2.print("P =");
      u8g2.setCursor(cols[1], rows[3]);
      u8g2.print(p_sol, 1);
      u8g2.setCursor(cols[2], rows[3]);
      u8g2.print("W");

      u8g2.setCursor(cols[0], rows[4]);
      u8g2.print("UID=");
      u8g2.setCursor(cols[1], rows[4]);
      u8g2.print(u_sol_idle, 1);
      u8g2.setCursor(cols[2], rows[4]);
      u8g2.print("V");

      u8g2.setCursor(cols[3], rows[0]);
      u8g2.print("BATT:");

      u8g2.setCursor(cols[3], rows[1]);
      u8g2.print("U =");
      u8g2.setCursor(cols[4], rows[1]);
      u8g2.print(u_bat, 1);
      u8g2.setCursor(cols[5], rows[1]);
      u8g2.print("V");*/

    //u8g2.setCursor(cols[3], rows[2]);
    //u8g2.print(". =");
    //u8g2.setCursor(cols[4], rows[2]);
    //u8g2.print(buckRes);
    //u8g2.setCursor(cols[5], rows[2]);
    //u8g2.print("");

    /*u8g2.setCursor(cols[3], rows[3]);
      u8g2.print("P =");
      u8g2.setCursor(cols[4], rows[3]);
      u8g2.print(p_bat,1);
      u8g2.setCursor(cols[5], rows[3]);
      u8g2.print("W");*/

    /*u8g2.setCursor(cols[3], rows[2]);
      u8g2.print("I =");
      u8g2.setCursor(cols[4], rows[2]);
      u8g2.print(i_bat,0);
      u8g2.setCursor(cols[5], rows[2]);
      u8g2.print("mA");

      u8g2.setCursor(cols[3], rows[3]);
      u8g2.print("P =");
      u8g2.setCursor(cols[4], rows[3]);
      u8g2.print(p_bat,1);
      u8g2.setCursor(cols[5], rows[3]);
      u8g2.print("W");*/


    char datestring[20];

      #define countof(a) (sizeof(a) / sizeof(a[0]))

      snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            rtcNow.Month(),
            rtcNow.Day(),
            rtcNow.Year(),
            rtcNow.Hour(),
            rtcNow.Minute(),
            rtcNow.Second() );


    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(cols[0], rows[6]);
    //u8g2.print(u_bat,1);
    u8g2.print(datestring);


    if (undervoltageProtection) {
      u8g2.setCursor(cols[3], rows[4]);
      u8g2.print("BEMTY");
    }

    if (overvoltageProtection) {
      u8g2.setCursor(cols[3], rows[4]);
      u8g2.print("BFULL");
    }

    u8g2.setCursor(cols[0], rows[5]);
    if (night) {
      u8g2.print("NIGHT");
    }

    u8g2.setCursor(cols[3], rows[5]);
    switch (mode) {
      case 0: {
          u8g2.print("LOADOFF");
          break;
        }
      case 1: {
          u8g2.print("LOADAUTO");
          break;
        }
      case 2: {
          u8g2.print("LOADON");
          break;
        }
    }

    if (checkIdleVoltage) {
      u8g2.setCursor(cols[0], rows[5]);
      u8g2.print("CHECKUID");
    }

    /*// dc gauge at bootom of display
      u8g2.setCursor(max(min(map(dc, 0, buckRes, 0, 127) - 5, 117), 0), 62);
      u8g2.print(dc);
      u8g2.drawBox(0, 63, map(dc, 0, buckRes, 0, 127), 1);*/


  } while ( u8g2.nextPage() );
}

int u_sol_rolav(int newReading) {
  u_sol_total = u_sol_total - u_sol_readings[u_sol_readIndex];
  u_sol_readings[u_sol_readIndex] = newReading;
  u_sol_total = u_sol_total + u_sol_readings[u_sol_readIndex];
  u_sol_readIndex++;
  if (u_sol_readIndex >= numReadings) {
    u_sol_readIndex = 0;
  }
  return round(float(u_sol_total) / float(numReadings));
}

int u_bat_rolav(int newReading) {
  u_bat_total = u_bat_total - u_bat_readings[u_bat_readIndex];
  u_bat_readings[u_bat_readIndex] = newReading;
  u_bat_total = u_bat_total + u_bat_readings[u_bat_readIndex];
  u_bat_readIndex++;
  if (u_bat_readIndex >= numReadings) {
    u_bat_readIndex = 0;
  }
  return round(float(u_bat_total) / float(numReadings));
}
