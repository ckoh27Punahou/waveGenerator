# Wave Generator Documentation

## Wave Generator Purpose

This project was developed to help support outreach efforts in Hawaii's educational sector, specifically in regards to further educating the use about waves and wave energy. 

This github page aims to provide adequate documentation for a desktop wave generator. It should provide **CAD Files, COTS BOM, Electrical Diagrams, and Source Code**. 

# CAD Files

## File Formatting

This wave generator is constructed using 3D printed and supplementary COTS parts (Commercial Off The Shelf). **The CAD files are posted as .STEP files**, meaning that they are ***NOT*** meshes (fully editable geometry). 

## Print Settings

The 3D printed parts (for working models) should be printed out of PLA or PETG. Settings are as follow: 3-5 walls, 25-45% infill. Prints should be within +-0.2mm of tolerance. Adjust to printer as necessary. 

## CAD BOM

The [Bill of Materials](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/waveGeneratorBOM.csv) to all of the COTS parts is posted as a .csv within the Docs folder. 

# Source Code

This wave generator uses code to take user input via rotary encoders and calculate the proper settings to generate the user specified waves. There are two options for source code: using Visual Studio's Platform IO, or using the official Arduino IDE. There are two versions of the code posted above (identical) - each is designed for the specific program. 

## Development Platforms

This code was developed specifically for the [Arduino Nano ESP32](https://store.arduino.cc/products/nano-esp32-with-headers?srsltid=AfmBOoobrec-nGruFs_-Y9yvGQ6Ko73yMo6Cp1iv4HudvSx1w5w5968Y) using the Arduino configuration. The development platform was Visual Studio Code, which utilized Platform IO to push the code to the Arduino and include the necessary libraries. Versions for the Arduino IDE and VS Code are both posted above (**Install Accordingly**). 

## Installing Libraries

The following library dependencies need to be installed (either using Platform IO or the Arduino IDE): AccelStepper.h, hd44780.h, hd44780_I2Cexp.h - this is done externally of the written software. 

[How to install libraries in Platform IO](https://community.platformio.org/t/adding-libraries-to-project/25807)

[How to install libaries in Arduino IDE](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries/)

# Electrical Wiring

[Electrical wiring diagram](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/waveGeneratorElectricalDiagram.pdf) is posted in the Docs folder ("waveGeneratorElectricalDiagram.pdf"). 

# Assembly

Assembly for this project is relatively simple. It goes as follows: electrical assembly and wiring, installing heat-set inserts, physical assembly, belt tensioning, and finally mounting. 

[Full assembly instructions](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/AssemblyInstructions.md) is posted in the Docs folder ("AssemblyInstructions.md")

# Usage

Usage for this project is relatively straightforward. After code has been uploaded and mechanical and electrical systems have been constructed, the following instructions can be used: 

[Usage Instructions](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/UsageInstruction.md)

# Developer Notes

I'm Charlie Koh. At the time of this development, I am a rising Junior at Punahou High School. This project was developed in collaboration with the lab of Dr. Troy Heitmann.  