# Usage Procedure (Uploaded Arduino Code)

This is an instructional guide on how to use the wave generator after code has been uploaded. 

## Definitions (Repeated Later): 

**Wedge Angle:** The bottom angle of the wedge (between the hypotenuse and mounting side). Should be labeled at the top of the wedge. 
**Starting Depth:** The starting depth of the wedge. Measured as the deepest point of the wedge to the surface of the still water. Can be approximated (+-5mm). 
**Tank Depth:** The water depth in the tank. Measured from the bottom of the tank to the surface of the still water. 
**Amplitude:** The amplitude of the generated wave in millimeters. This is not completely accurate at the moment. During hardcaps (capping of amplitude at certain frequencies due to mechanical limits), the LCD will notify what the new hardcapped frequency is. The stroke length of the wedge corresponds with amplitude (via calculations involving all of the aforementioned variables and frequency), but does **not directly resemble the amplitude**. 
**Frequency:** The frequency of the generated wave in Hz. Frequency will always take priority over amplitude if mechanical limits are encountered (calculated by system internals - these are either speed caps or mechanical limits). 

## Step 0: Confirm Electrical Systems

Confirm that all electrical systems (particularly the buck converter) are wired exactly as the diagram shows. 

Move the wedge to the top of the movement range (as close as possible to the motor)

## Step 1: Plug Battery In

Plug the battery (24v 2.8aH) into the power adapter. The Arduino Nano ESP32 should have a small green LED light up. The motor driver (DM556Y) should also have a green LED light up. 

The LCD should light up, and should display some brief introduction text. 

## Step 2: Setting Initial Settings

### ***NOTE: You will not be able to change wedge angle, starting depth, or tank depth after the initialization phase. To reset, turn the system on and off***

After the LCD turns on, read the introductory text. After all text prompts disappear, it will ask to set a wedge angle. For this setting, either dial works to adjust the value (adjustments are +-1 degree). To confirm the value (after set to the desired value), push down on either of the dials (there is a button within the encoder).

It will then ask to set starting depth and tank depth. The top dial corresponds with starting depth, while the bottom dial corresponds with the tank depth. Set starting depth (the distance of the deepest wedge point from the surface of still water) and tank depth (the depth of water within the tank) to the nearest millimeter (adjustments are +-1 mm). To confirm starting depth, push down on the top dial. To confirm tank depth, push down on the bottom dial. 

After both starting and tank depth are confirmed, it will then ask for a **starting frequency and starting amplitude**. The top dial corresponds with amplitude, and the bottom dial corresponds with frequency. To confirm amplitude, press down on the top dial. To confirm frequency, press down on the bottom dial. 

***NOTE: Amplitude refers to the target amplitude of the WAVE, not the stroke. The stroke and rotation of the motor are calculated using all inputs.***

After a few brief seconds, it should start making waves. 

## Step 3 (Optional): Wave Amplitude and Frequency Adjustments

To adjust the wave amplitude and frequency without going through the entire setup, push down on either dial. It will pause wave generation as new settings are entered. The top dial again corresponds with amplitude, and the bottom dial is again with frequency. To confirm either, push down on the corresponding dial. 

A few seconds after both settings are confirmed, wave generation will resume, but with the adjusted settings. 