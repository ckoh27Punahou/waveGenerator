# Wave Generator Documentation

This github page aims to provide adequate documentation for a desktop wave generator. It should provide **CAD Files, COTS BOM, Electrical Diagrams, and Source Code**. 

# CAD Files

## File Formatting

This wave generator is constructed using 3D printed and supplementary COTS parts (Commercial Off The Shelf). **The CAD files are posted as .STEP files**, meaning that they are ***NOT*** meshes (fully editable geometry). 

## Print Settings

The 3D printed parts (for working models) should be printed out of PLA or PETG. Settings are as follow: 3-5 walls, 25-45% infill. Prints should be within +-0.2mm of tolerance. Adjust to printer as necessary. 

## CAD BOM

The [Bill of Materials](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/waveGeneratorBOM.csv) to all of the COTS parts is posted as a .csv within the Docs folder. 

# Source Code

This wave generator uses code to take user input via rotary encoders and calculate the proper settings to generate the user specified waves. 

Links to papers used as resources are posted below: 

[Paper](https://google.com)

## Development Platforms

This code was developed specifically for the [Arduino Nano ESP32](https://store.arduino.cc/products/nano-esp32-with-headers?srsltid=AfmBOoobrec-nGruFs_-Y9yvGQ6Ko73yMo6Cp1iv4HudvSx1w5w5968Y) using the Arduino configuration. The development platform was Visual Studio Code, which utilized Platform IO to push the code to the Arduino and include the necessary libraries. Versions for the Arduino IDE and VS Code are both posted above (**Install Accordingly**). 

## Installing Libraries

The following libraries need to be installed (either using Platform IO or the Arduino IDE): AccelStepper.h, hd44780.h, hd44780_I2Cexp.h - this is done externally of the written software. 

# Electrical Wiring

[Electrical wiring diagram](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/waveGeneratorElectricalDiagram.pdf) is posted in the Docs folder ("waveGeneratorElectricalDiagram.pdf"). 

# Assembly

Assembly for this project is relatively simple. It goes as follows: electrical assembly and wiring, installing heat-set inserts, physical assembly, belt tensioning, and finally mounting. 

[Full assembly instructions](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/AssemblyInstructions.md) is posted in the Docs folder ("AssemblyInstructions.md")