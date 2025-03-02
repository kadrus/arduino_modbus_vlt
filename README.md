# arduino_modbus_vlt
Arduino Micro communication with several Danfoss VLT HVAC frequency Converters (FCs)

      The polypropylene mesh production line has three rolls for longitudinal orientation drawing in a hot water bath. Three Danfoss frequency converters control the motors that rotate these shafts. The shafts' operating speeds must be in a certain ratio and controlled synchronously for the line to operate properly.
      The manufacturer used an analog circuit with four variable resistors, one setting the reference voltage for the other three. The control voltages from the sliders of these three resistors are fed to the corresponding frequency converters.
      Over time, however, the contact between the sliders and the conductive layer in inadequate-quality resistors broke down. Undesirable random deviations of shaft speeds occurred, which led to grid breaks and accidents on the line.
      I implemented digital control of the frequency converters via Modbus protocol. For this purpose, I programmed an Arduino Micro Pro controller. The accidents on the line have stopped

There are several libraries needed to compile this project:
https://github.com/emelianov/modbus-esp8266
https://github.com/acksen/AcksenIntEEPROM
https://github.com/Dlloydev/Toggle
https://github.com/RobTillaart/RunningMedian
