## BLDC position control project ##

Three techniques have been implemented:

 - Position control with sinusoidal commutation -`sinus_control_V2`
 - Position control with simplified FOC (no current sensing) - `FOC`
 - Position control with special sinusoidal commutation(vectol angle limitation) - `combined_control_V3`
  
 ### Structure ###

Essential elements:

 - STM32F4-Discovery
 - DRV8313
 - AS5048A

![Alt-текст](https://github.com/ViktorAnchutin/BLDC_CONTROL/blob/master/graph/Structure.JPG?raw=true "Structural scheme")

### Electrical scheme ###
![Alt-текст](https://github.com/ViktorAnchutin/BLDC_CONTROL/blob/master/graph/El.JPG?raw=true "Electrical scheme")



[![Watch the video](https://raw.github.com/GabLeRoux/WebMole/master/ressources/WebMole_Youtube_Video.png)](http://youtu.be/vt5fpE0bzSY)
