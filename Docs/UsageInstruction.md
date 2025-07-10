# Usage Procedure (Uploaded Arduino Code)

This is an instructional guide on how to use the wave generator after code has been uploaded. 

## Step 0: Confirm Electrical Systems

Confirm that all electrical systems (particularly the buck converter) are wired exactly as the diagram shows. 

Move the wedge to the top of the movement range (as close as possible to the motor)

## Step 1: Plug Battery In

Plug the battery (24v 2.8aH) into the power adapter. The Arduino Nano ESP32 should have a small green LED light up. The motor driver (DM556Y) should also have a green LED light up. 

The LCD should light up, and should display some brief introduction text. 

## Step 2: Setting Initial Settings

### ***NOTE: You will not be able to change wedge angle, starting depth, or tank depth after the initialization phase. To reset, turn the system on and off***

After the LCD turns on, read the introductory text. After all text prompts disappear, it will ask to set a wedge angle. For this setting, either rotary encoder works to adjust the value (adjustments are +-1 degree). To confirm the value (after set to the desired value), push down on either of the rotary encoders (there is a button within the encoder).

It will then ask to set starting depth and tank depth. The amplitude rotary encoder corresponds with starting depth, while the frequency rotary encoder corresponds with the tank depth. Set starting depth (the distance of the deepest wedge point from the surface of still water) and tank depth (the depth of water within the tank) to the nearest millimeter (adjustments are +-1 mm). To confirm starting depth, push down on the amplitude rotary encoder. To confirm tank depth, push down on the frequency rotary encoder. 

After both starting and tank depth are confirmed, it will then ask for a **starting frequency and starting amplitude**. The amplitude rotary encoder corresponds with amplitude, and the frequency rotary encoder corresponds with frequency. To confirm amplitude, press down on the amplitude rotary encoder. To confirm frequency, press down on the frequency rotary encoder. 

***NOTE: Amplitude refers to the target amplitude of the WAVE, not the stroke. The stroke and rotation of the motor are calculated using all inputs***

After a few brief seconds, it should start making waves. 

## Step 3 (Optional): Wave Amplitude and Frequency Adjustments

To adjust the wave amplitude and frequency without going through the entire setup, push down on either rotary encoder. It will pause wave generation as new settings are entered. The amplitude rotary encoder again corresponds with amplitude, and the frequency rotary encoder is again with frequency. To confirm either, push down on the corresponding rotary encoder. 

A few seconds after both settings are confirmed, wave generation will resume, but with the adjusted settings. 