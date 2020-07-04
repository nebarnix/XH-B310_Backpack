//Nebarnix 

#include "Thermocouple.h"

//Thermocouple definitions
#define TCPIN A1
#define TC_OFFSET_VOLTAGE    -4.605    // mV compensation for system offset, fine tune this to match what is being read on the display
#define GAIN_TC        76
#define VREF     5000.0     //mV

#define POLY_CALC(retVal, inVal, coeff_array) \
  { \
    float expVal = 1.0f; \
    const float* coeff = coeff_array; \
    retVal = 0.0f; \
    while(*coeff != 1.0f)\
    { \
      retVal += *coeff * expVal; \
      expVal *= inVal; \
      coeff++; \
    }\
  }

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
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

//microsmooth Filter Pointer

float readThermistor()
{
  int samples[NUMSAMPLES];
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
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

  // convert the value to resistance (this won't work because the voltage is against 3.3 not 5)
  average = average * (5.0 / 3.3); //scale by supply difference
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
    samples[i] = analogRead(TCPIN);
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

  // convert the value to resistance (this won't work because the voltage is against 3.3 not 5)
  //average = average * (5.0 / 3.3); //scale by supply difference
  //average = 1023 / average - 1;
  //average = SERIESRESISTOR / average;

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

float convertTCtoC(float cjTemp, float adcval, char type=TYPE_K)
{
  float cjVoltage, thVoltage,calc;
  const temp_range thCoeff = PROGMEM_getAnything(&thPolyCoeff[type]);

  //thVoltage = ((VREF * (adcval - _2_23)) / (_2_23 * GAIN_TC)) + TC_OFFSET_VOLTAGE;
  thVoltage = ((VREF * (adcval / 1024.0)) / GAIN_TC) + TC_OFFSET_VOLTAGE;
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
  thVoltage += cjVoltage;
  // Serial.print(" Vfinal = "); Serial.print(thVoltage);
  //th_Voltage[ch] = thVoltage;


  if (thVoltage >= PROGMEM_getAnything(&thVoltageRange[type][0]))  
  {
    if (thVoltage < PROGMEM_getAnything(&thVoltageRange[type][1]))
      {POLY_CALC(calc, thVoltage, thCoeff.neg_voltage);}
    else
    {
      if (thVoltage <= PROGMEM_getAnything(&thVoltageRange[type][2]))
        {POLY_CALC(calc, thVoltage, thCoeff.pos_voltage1);}
      else
      {
        if ((thVoltage <= PROGMEM_getAnything(&thVoltageRange[type][3])) && (thCoeff.pos_voltage2[0] != 1.0f))
          {POLY_CALC(calc, thVoltage, thCoeff.pos_voltage2);}
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

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

//these need to be persistant to hold the previous data points
float TrefPrev=0;
float TCTempPrev=0;
bool firstFlag = true;

//exponential filter params
#define ALPHA_TREF 0.1
#define ALPHA_TC 0.1

void loop() {
  
  float TRefValue = readThermistor(); //performs a burst of averages
  float Tref = convertThermistorToC(TRefValue); 
  if(firstFlag == true)
     TrefPrev = Tref;
  
  Tref = ALPHA_TREF*Tref +(1.0-ALPHA_TREF)*TrefPrev;
  TrefPrev = Tref;
     
  Serial.print("TrefF = ");
  Serial.print(Tref);
    
  float TCValue = readThermocouple(); //performs a burst of averages
  float TCTemp = convertTCtoC(Tref, TCValue, TYPE_K);  
  if(firstFlag == true)
     TCTempPrev = TCTemp;
  TCTemp = ALPHA_TC*TCTemp +(1.0-ALPHA_TC)*TCTempPrev;
  TCTempPrev = TCTemp;
  
  Serial.print(" Tprobe = ");
  Serial.print(TCTemp);
  Serial.println();

  firstFlag = false;
  delay(1000);
}
