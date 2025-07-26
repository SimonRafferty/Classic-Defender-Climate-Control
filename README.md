# Classic-Defender-Climate-Control

I added Air Conditioning to my Defender - and wanted the HVAC to behave similarly to that on my 2005 Range Rover Sport.

I wanted to re-use the original controls on the Defender for the air temperature and fan speed - but I added two new switches (the same functions as the Range Rover) These are Auto / Manual and Defrost.

In Auto, the cab temperature is contolled with the original temperature slider.  It decides whether Air Conditioning is needed and also controls the duty cycle of the valve which allows hot water to flow through the heater matrix.  It also changes the fan speed based on the temperature difference between what you've asked for and the actual cab temperature.

In Manual mode, the fan speed is set with the original slider.  The temperature controls the heater valve duty cycle and in the lower part of it's travel, switches on the AC too.

I don't have a temperature readout in the cab nor is the temperature slider calibrated.  I just made it so, around the middle of it's range is a comfortable 21C.  Then, like the original I just nudge it up or down a little as required.

To re-use the original control sliders, I used Electric Go-Kart throttle potentiometers (very inexpensive) which accept a cable.  I think, if I were doing it again, I would use a better quality fly-by-wire throttle from a car.  These measure the slider positions & the analog signal is measured by the ESP32.

The Defrost button turns on the AC, sets the heater valve to 100% (Hot) and the fan to maximum.  If fitted, it also switches on the heated screen & mirrors.  This combination is VERY effective!

So far, it has worked flawlessly. 

It's designed to run on an Adafruit Huzzah32.  Why use a relatively expensive ESP32 when you can buy one for $3? I've found them the most reliable in automotive applications with wide temperature variations, RF and electrical noise. The first version used a TTGO T-Display but with largely the same code, it kept crashing and after a few days, bricked itself!
