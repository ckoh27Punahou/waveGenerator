# Assembly Instructions

## Required Materials

1. Soldering Iron
2. Solder
3. BOM COTS Parts ($134.51)
4. 3mm Hex Key
5. Phillips Head Screwdriver
6. 2mm Hex Key
7. PLA
8. 20 AWG Wire
9. Dupont Jumper Cables
10. Wire Cutters
11. Wire Stripper
12. Heat Shrink/Electrical Tape

## Step 1: Electrical Wiring

Construct the electrical diagram as shown ("waveGeneratorElectricalDiagram"). The junctions indicate soldered areas/wire electrical connections. Wires that cross but do not contain a junction on their intersection do not have any electrical connection. For safety, place electrical tape/heat shrink over soldered joints. 

## Step 2: Heat Set Inserts

**NOTE: For this step, use a soldering iron tip either specifically designed for heat set inserts, or be comfortable having potential PLA contamintation on the tip of the soldering iron.**

Using M4 Heat Set Inserts (included in the COTS BOM), place heat set inserts in all of the following designated locations (images shown below). Arrows with label "1" indicate heat set inserts. 

Heat set inserts ***MUST*** be put in straight. If they are not, and screws insertion is attempted, the insert may do one of the following:

1. Surface detachment (pulled out)
2. Cross threading (destroys thread)
3. Unable to insert (either due to misalignment or angling)

### Upper Diagram:

![Upper Diagram](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/heatSetInserts/upperDiagram.png)

### Lower Diagram: 

![Lower Diagram](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/heatSetInserts/lowerDiagram.png)

### Wedge Diagram:

![Wedge Diagram](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/heatSetInserts/wedgeDiagram.png)

## Step 3: Physical Assembly

### **Fully Exploded View**

![Exploded View](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/fullyExploded.png)

### Step 3a: Insert Motor

Insert the motor as shown in the image below. Screw the motor in using the countersunk M3 screws (phillips head). 

![Motor Insert](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/motorInsert.png)

### Step 3b: Insert Carbon Fiber Rod

Insert carbon fiber rod in the designated location of the motor mount. This won't be secured, and is used for structural support.

When inserting, make sure to simultaniously insert two linear bearings as shown below. These will be used to tension the belt.

![Carbon Fiber Rod Insert](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/carbonFiberRodInsert.png)

### Step 3c: Assemble Carrier

Press fit linear bearings into the central carrige, and then screw the top cover onto the central carrige (as displayed below) using M4x20mm screws. 

![Carrier Assembly Diagram](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/carrierAssembly.png)

### Step 3d: Assemble Lower Mount

Press fit 17mmx6mmx6mm bearing into lower pulley. Place pulley (with bearing) and pulley spacer onto 25mmx6mm steel dowel, and then insert into location on lower mount (displayed in image below).

Place lower seals to cover and secure 25mmx6mm steel dowel, and then secure with M4x20mm screws. 

![Lower Mount Assembly Diagram](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/lowerMountAssembly.png)

### Step 3e: Insert Vertical Carbon Fiber Rods

Insert two 300mmx8mm Carbon Fiber Rods into the lower mount. Proceed to use M4x16mm screws to secure the carbon fiber rods (displayed in the image below - similar to set screws).

![Carbon Fiber Rod Insert](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/verticalCarbonFiber.png)

### Step 3f: Attach Motor Mount to Carbon Fiber Rods

Place the motor mount on the top of the two previously mounted vertical carbon fiber rods. Attach the front plate, and secure to the lower and motor mounts using M4x16mm screws. Similarly to the previous step, secure the carbon fiber rods with side M4x16mm screws (again, similar to set screws). 

![Motor Mount Carbon Fiber Insert](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/motorMountCarbonFiber.png)

### Step 3g: Wedge Attachment

Place the front wedge stilts aligned with the central carrier. Use six M4x20mm screws to secure the wedge to the central carrier. 

![Wedge Attachment Image](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/wedgeMount.png)

### Step 3h: Tank Mount 1

Place one of the outer mounts on the motor side of the motor mount, and secure it with two M4x20mm bolts. 

![Tank Mount 1](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/tankMount1.png)

### Step 3i: Mounting GT2 Pulley

Place 20T GT2 pulley onto the stepper motor shaft. 

![Pulley Mounting](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/pulleyMount.png)

### Step 3j: Belt Mounting

Place belt around lower pulley and belt pulley, and place ends in the top and the bottom of the central carrier (not displayed). Place belt securer and M4x20mm bolt, and secure belt (with serrated side facing the belt and matching teeth) - make sure the belt is properly tensioned. 

![Belt Mounting](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/beltGrabber.png)

### Step 3k: Tank Mounting

Place full assembly into the tank. Place second outer mount on the non-motor side of the motor mount, and secure around the tank using two M4x20mm screws.

![Tank Mounting](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/imageDiagrams/assemblyImages/tankMount2.png)

***Assembly of wave generator is completed***

## Step 4: Uploading Code

Prior to uploading code, choose to use either Visual Studio Code's Platform IO or the official Arduino IDE (easier). Download the corresponding file from this repository. 

Plug a data-capable USB-C cable into the Arduino Nano ESP32. Using either Platform IO or the Arduino IDE, make sure to import the following libraries (***NOT #include***): 

1. AccelStepper
2. hd44780
3. (For PIO, identify path) hd44780

[How to install libraries in Platform IO](https://community.platformio.org/t/adding-libraries-to-project/25807)

[How to install libaries in Arduino IDE](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries/)

Using the computer port that the USB-C cable, upload the designed sketches (***Use the program either designed for Platform IO or Arduino IDE***). Once it states that uploding is complete, unplug the USB-C cable. 

***Transition to [User Manual](https://github.com/ckoh27Punahou/waveGenerator/blob/main/Docs/UsageInstruction.md) for usage instructions***

