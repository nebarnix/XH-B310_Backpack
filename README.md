# XH-B310_Backpack
Read the analog sensors on these cheap TC displays to (more or less) pull the temperature data into your arduino. 

This sketch is designed to run on an arduino nano. 

A0 to the reference thermistor (3.3V across a 10k/NTC 10k thermistor)
A1 to the output of the gain=1+(75k/1k)=76 non inverting thermocouple voltage amplifier. 

Methods to improve accuracy:
Easy: Can be improved by routing 3.3V to the Aref instead using the 5V (you will have to change the adc reference and one thermistor calc value)
Harder: Can be further improved by routing the onboard 2.5V precision voltage reference to the Aref pin (you will have to change the adc reference and one thermistor calc value)
