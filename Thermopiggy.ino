//Nebarnix
#define USE_SERIAL //comment out if not using the serial port

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <Fonts/FreeSans12pt7b.h>

#include <ClickEncoder.h>
#include <TimerOne.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <toneAC.h>
#include "Thermocouple.h"

//Thermocouple definitions
#define TCPIN A1
//#define TC_OFFSET_VOLTAGE    -4.605    // mV compensation for system offset, fine tune this to match what is being read on the display
#define TC_OFFSET_VOLTAGE    -4.675    // mV compensation for system offset, fine tune this to match what is being read on the display
//#define TC_OFFSET_VOLTAGE    -4.530    // mV compensation for system offset, fine tune this to match what is being read on the display
#define GAIN_TC        76
//#define VREF     5000.0     //mV
#define VREF     3241.0     //mV (what's your 3.3V reference?? is directly proportional to temperture error)
//#define VREF     2500.0     //mV

#define MUX_STATE_PIN 2
#define OFFSET_MEASURE HIGH
#define NORMAL LOW
volatile bool currentMuxState = NORMAL;

//Thermistor
// which analog pin to connect
#define THERMISTORPIN A0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
//#define BCOEFFICIENT 3950 (default from sparkfun)
#define BCOEFFICIENT 4050 //custom attempts to fine tune, inversely proporantional to temp error
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

//click encoder stuff
ClickEncoder *encoder;
//int16_t last, value;

unsigned long lastRefreshTime = 0;

//these need to be persistant to hold the previous data points
float TrefPrev = 0;
float TCTempPrev = 0;
float TSetPoint = 35;
bool TSetPointReached = false;
bool firstFlag = true;
float TCMvOffset = TC_OFFSET_VOLTAGE;

//exponential filter params. Value between 1 and 0, the smaller the number the stronger the filter
#define ALPHA_TREF 0.1
#define ALPHA_TC 0.05
#define ALPHA_CAL 0.5
//#define ALPHA_TC 1

int16_t encoderValuePrev, encoderValue;

enum DispMode {
  statPage,
  tempPage,
  setPage,
  offsetPage,
  wrapPage
};


DispMode mode = statPage;


void timerIsr() {
  encoder->service();
}

float readThermistor()
{
  int samples[NUMSAMPLES];
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    analogRead(THERMISTORPIN); samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;

  //Serial.print("Average analog reading ");
  //Serial.println(average);

  //Serial.print(" ThermistorV=");
  //Serial.print((VREF * (average / 1024.0))/1000);
  

  // convert the value to resistance (this won't work because the voltage is against 3.3 not 5)
  average = average * (VREF / 3300); //scale by supply difference in mV
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  return average;
}

float readThermocouple()
{
  int samples[NUMSAMPLES];
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    analogRead(TCPIN); samples[i] = analogRead(TCPIN);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;

  //Serial.print("Average analog reading ");
  //Serial.println(average);

  return average; //gives raw ADC value
}

float convertThermistorToC(float resistance)
{
  float steinhart;
  steinhart = resistance / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  return steinhart;
}

float tuneOffsetV(float adcval)
{
  float thVoltage = -((VREF * (adcval / 1024.0)) / GAIN_TC);
  TCMvOffset = ALPHA_CAL * thVoltage + (1.0 - ALPHA_CAL) * TCMvOffset;
  return thVoltage;
}

float convertTCtoC(float cjTemp, float adcval, char type = TYPE_K)
{
  float cjVoltage, thVoltage, calc;
  const temp_range thCoeff = PROGMEM_getAnything(&thPolyCoeff[type]);

  //thVoltage = ((VREF * (adcval - _2_23)) / (_2_23 * GAIN_TC)) + TC_OFFSET_VOLTAGE;
  thVoltage = ((VREF * (adcval / 1024.0)) / GAIN_TC) + TCMvOffset;
  //Serial.print(" Vtc = "); Serial.print(thVoltage);
  //th_Voltage_read[ch] = thVoltage;

  if (cjTemp <  PROGMEM_getAnything(&cjTempRange[type][1])) {
    POLY_CALC(cjVoltage, cjTemp, thCoeff.neg_temp);
  } else {

    if (cjTemp <=  PROGMEM_getAnything(&cjTempRange[type][2])) {

      POLY_CALC(cjVoltage, cjTemp, thCoeff.pos_temp1);
      if (type == TYPE_K) {
        cjVoltage += COEFF_K_A0 * exp(COEFF_K_A1 * (cjTemp - COEFF_K_A2) * (cjTemp - COEFF_K_A2));
      }
    } else {
      POLY_CALC(cjVoltage, cjTemp, thCoeff.pos_temp2);
    }
  }
  //cj_Voltage[ch] = cjVoltage;
  //Serial.print(" Vcj = "); Serial.print(cjVoltage);
  //Serial.print(" VtcR = "); Serial.print(thVoltage);
  
  thVoltage += cjVoltage; //offset the C

  // Serial.print(" Vfinal = "); Serial.print(thVoltage);
  //th_Voltage[ch] = thVoltage;


  if (thVoltage >= PROGMEM_getAnything(&thVoltageRange[type][0]))
  {
    if (thVoltage < PROGMEM_getAnything(&thVoltageRange[type][1]))
    {
      POLY_CALC(calc, thVoltage, thCoeff.neg_voltage);
    }
    else
    {
      if (thVoltage <= PROGMEM_getAnything(&thVoltageRange[type][2]))
      {
        POLY_CALC(calc, thVoltage, thCoeff.pos_voltage1);
      }
      else
      {
        if ((thVoltage <= PROGMEM_getAnything(&thVoltageRange[type][3])) && (thCoeff.pos_voltage2[0] != 1.0f))
        {
          POLY_CALC(calc, thVoltage, thCoeff.pos_voltage2);
        }
        else
        {
          if (thCoeff.pos_voltage3[0] != 1.0f)
          {
            if (thVoltage <= PROGMEM_getAnything(&thVoltageRange[type][4]))
              POLY_CALC(calc, thVoltage, thCoeff.pos_voltage3);
          }
        }
      }
    }
  }
  return calc;
}

void splashScreen()
{
  //display splash screen
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  display.println(F("  Thermo  "));
  display.println(F("  Piggy   "));
  display.display();
  display.clearDisplay();
  delay(1500);

  display.setCursor(0, 0);            // Start at top-left corner
  display.println( F(" Nebarnix "));
  display.println(F("   2020"));
  display.display();
  display.clearDisplay();
  delay(1500);
}

void initOLED()
{
  //Init OLED display
  //FAIL BEEP IF FAIL
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
#ifdef USE_SERIAL
    Serial.println(F("SSD1306 allocation failed"));
#endif
    toneAC(2000); //pins 9 and 10 on a nano hooked to a buzzer
    delay(500);
    toneAC();
    for (;;); // Don't proceed, loop forever
  }
}

void initGPIO()
{

  //setup GPIO
  pinMode(7, OUTPUT); //this is the fake ground for the encoder
  digitalWrite(7, LOW); //make it 0

  pinMode(6, OUTPUT); //this is the fake 5V for the encoder
  digitalWrite(6, HIGH); //make it 1

  //encoder pins need pullups
  pinMode(3, INPUT);
  //digitalWrite(3, HIGH); //enable pullup
  pinMode(4, INPUT);
  //digitalWrite(4, HIGH); //enable pullup
  pinMode(5, INPUT);
  //digitalWrite(5, HIGH); //enable pullup

  pinMode(MUX_STATE_PIN, INPUT);
  //pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MUX_STATE_PIN), muxStateDetect, CHANGE);
}

void setup() {
  analogReference(EXTERNAL);
  // initialize serial communications at 9600 bps:
#ifdef USE_SERIAL
  Serial.begin(115200);
#endif
  initOLED();

  initGPIO();

  //setup encoder
  encoder = new ClickEncoder(4, 5, 3, 4); //CLK, DT, SW
  Timer1.initialize(1000); //1ms per read
  Timer1.attachInterrupt(timerIsr);

  //Boot-up beep
  toneAC(2600); //pins 9 and 10 on a nano hooked to a buzzer
  delay(50);
  toneAC();
  //splashScreen(); //make this nonblocking
}

void muxStateDetect()
{
  //read D2 without calling a function
  if (!(PIND & 0x4)) //D2 is LOW
    currentMuxState = NORMAL;
  else
    currentMuxState = OFFSET_MEASURE;
}

void chirp(int a, int b, int chirpTone)
{
  for (int i = 0; i < a; i++)
  {
    for (int j = 0; j < b; j++) //3 and 5 sounds like a cricket
    {
      toneAC(chirpTone);
      delay(10);
      toneAC();
      delay(10);
    }
    delay(100);
  }
}

void loop() {
  float TCTemp;

  if (lastRefreshTime > millis())
    lastRefreshTime = 0;

  encoderValue += encoder->getValue();
  /*
    if (encoderValue != encoderValuePrev)
    {
    encoderValuePrev = encoderValue;
    toneAC(2500);
    delay(10);
    toneAC();
    Timer1.initialize(1000); //1ms per read

    if(mode == setPage)
    {
    TCSetPoint+=encoderValue;
    encoderValue = 0;
    encoderValuePrev = encoderValue;
    }
    else if(mode == offsetPage)
    {

    }
    }*/

  //check to see if knob was clicked
  if (encoder->getButton() == ClickEncoder::Clicked)
  {
    toneAC(2000);
    delay(10);
    toneAC();
    Timer1.initialize(1000); //1ms per read
    mode = mode + 1;
    if (mode >= wrapPage)
      mode = statPage;
    encoderValue = 0; //reset knob on page change
    encoderValuePrev = encoderValue;
  }

  if ((millis() - lastRefreshTime) > 500) //update every second. If we sample too much, we'll throw off the piggy unit (ADC sample causes a glitch on the line)
    //if ((millis() - lastRefreshTime) > 100) //easy enough, update every second! hah)
  {
    float TRefValue = readThermistor(); //performs a burst of averages
    float Tref = convertThermistorToC(TRefValue);
    lastRefreshTime = millis();
    if (firstFlag == true)
      TrefPrev = Tref;

    Tref = ALPHA_TREF * Tref + (1.0 - ALPHA_TREF) * TrefPrev;
    TrefPrev = Tref;

#ifdef USE_SERIAL
    Serial.print("TrefF = ");
    Serial.print(Tref);
#endif

    bool checkMuxState = currentMuxState;
    float TCValue = readThermocouple(); //performs a burst of averages
    if (checkMuxState == currentMuxState && checkMuxState == NORMAL) //make sure it didn't change, skip if it did change mid-measure
    {
      TCTemp = convertTCtoC(Tref, TCValue, TYPE_K);
      if (firstFlag == true)
      {
        TCTempPrev = TCTemp;
        firstFlag = false;
      }
      TCTemp = ALPHA_TC * TCTemp + (1.0 - ALPHA_TC) * TCTempPrev;
      TCTempPrev = TCTemp;
    }
    else if (checkMuxState == OFFSET_MEASURE)
    {
      Serial.print(" CAL");
      Serial.print(tuneOffsetV(TCValue),4);
      
      TCTemp = TCTempPrev; //preserve the last temp for the rest of the loop, since we didn't update anything
    }
    else
    {
#ifdef USE_SERIAL
      Serial.print(" THROWAWAY!");
#endif
    TCTemp = TCTempPrev; //preserve the last temp for the rest of the loop, since we didn't update anything
    }

#ifdef USE_SERIAL
    Serial.print(" Tprobe = ");
    Serial.print(TCTemp);
    Serial.println();
#endif

    if (TSetPointReached == false && TCTemp > TSetPoint)
    {
      TSetPointReached = true;
      chirp(3, 5, 3500);
    }
    else if (TCTemp < (TSetPoint - 5) && TSetPointReached == true) //reset setpoint if we drop by 5
    {
      TSetPointReached = false;
      chirp(1, 3, 3000);
    }

    if (mode == statPage) //default display
    {
      display.setCursor(0, 0);            // Start at top-left corner
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.print(F("Tambient : "));
      display.println(Tref);
      display.print(F("Tprobe   : "));
      display.println(TCTemp);
      if (TSetPointReached == true)
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
      display.print(F("TSetPoint: "));
      display.println(TSetPoint);
      display.setTextColor(SSD1306_WHITE);
      //display.print(F("EncVal  : "));
      //display.println(encoderValue);
      display.println((millis() / 1000.0), 1);
      display.display();
    }
    else if (mode == tempPage) //view current temp large
    {
      display.setCursor(0, 0);            // Start at top-left corner
      display.setTextSize(4);             // Normal 1:1 pixel scale
      //display.print(F(""));
      display.println(TCTemp);
      display.display();
    }
    else if (mode == setPage) //view current set point temp
    {
      //encoderValue = encoder->getValue();
      if (encoderValue) //!= 0
      {
        TSetPoint += encoderValue;
        encoderValue = 0;
        TSetPointReached = false; //reset the setpoint if it changed
      }
      display.setCursor(0, 0);            // Start at top-left corner
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.println(F("Adjust Alarm"));
      display.print(F("Tprb: "));
      display.println(TCTemp);
      display.print(F("Tsp: "));
      display.println(TSetPoint);
      display.display();
    }
    else if (mode == offsetPage) //View current mV offset
    {
      //encoderValue = encoder->getValue();
      if (encoderValue) //!= 0
      {
        TCMvOffset += encoderValue / 1000.0;
        encoderValue = 0;
        //firstFlag = true; //reset filter
      }

      display.setCursor(0, 0);            // Start at top-left corner
      display.println(F("Adjust Offset"));
      display.setTextSize(1);
      display.print(F(  "Tprobe  : "));
      display.println(TCTemp);
      display.print(F("Offset  : "));
      display.println(TCMvOffset, 3);
      display.display();
    }

    display.clearDisplay();

  }
  //delay(1);
}
