# XH-B310_Backpack
Wouldn't it have been nice if the makers of this inexpensive and high range (800C!) meter decided to dedicate an STM8 pin to providing us serial temperature data? 
Well, they didn't, because they don't love us. 

So we'll have to do the next best thing...
Read the analog sensors on these cheap TC displays to (more or less) pull the temperature data into your arduino. 

This sketch is designed to run on an arduino nano. 
* A0 to the reference thermistor (3.3V across a 10k/NTC 10k thermistor)
* A1 to the output of the gain=1+(75k/1k)=76 non inverting thermocouple voltage amplifier.
* D2 to the MUX pin A, which samples the TC ground every 67 seconds. This is the mV offset. 
* Ground wire

Methods to improve accuracy:
* Easy: Can be improved by routing 3.3V to the Aref instead using the 5V (you will have to change the adc reference and one thermistor calc value)
  Done - just install a jumper between 3.3V pin and Aref pin. 
* Harder: Can be further improved by routing the onboard 2.5V precision voltage reference to the Aref pin (you will have to change the adc reference and one thermistor calc value)

Overview of Board:
![Image of front of board](https://github.com/nebarnix/XH-B310_Backpack/blob/master/XH-B310%20Front.jpg)
![Image of back of board](https://github.com/nebarnix/XH-B310_Backpack/blob/master/XH-B310%20Back.JPG)

Signal interception points:
![Image of tap-off points](https://github.com/nebarnix/XH-B310_Backpack/blob/master/XH-B310%20Taps.JPG)

Fritzing of Arduino:
![Image of arduino layout](https://github.com/nebarnix/XH-B310_Backpack/blob/master/ThermoPiggy_bb.png)
