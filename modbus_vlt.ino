/*
  Modbus RTU communication with several Danfoss VLT HVAC frequency Converters (FCs)
*/

#include <ModbusRTU.h>
#include <Toggle.h>
#include <RunningMedian.h>
#include <AcksenIntEEPROM.h>
#include <math.h>

// Buttons stuff is here: -------------------------------------------------

#define BT_NUM             8 // number of buttons
#define TIME_TO_REPEAT   700 // after this holding time, in ms, button starts to retrigger
#define TIME_TO_SET     4000 // after this time the frequency of each slave FCs will be set equal to such of "upper" FC by measuring its analog output signal
#define RETRIG_PER        70 // retriggering period in milliseconds
#define COUNT_TO_ACCEL    20 // after this number of button retriggering events parameter change starts to accelerate
#define INIT_STEP         12 // initial increment/decrement in parameter
#define ACCEL_STEP_UP      2 // each retrigger step rising
#define MAX_STEP         256 // maximum allowed step
#define MAXFRQ_LIM       100 // limit for the MaxFrq in Hertz

uint16_t   retriggerMs[BT_NUM] {}; // retriggering period for each button separately
byte    retriggerCount[BT_NUM] {}; // how many times this button is retriggered
uint16_t          Step[BT_NUM] {}; 
const byte         Pin[BT_NUM] { 2, 3, 4, 5, 6, 7, 8, 9 }; // corresponding pins of Arduino Pro Micro board

// Button objects declaration
Toggle *Bt = new Toggle[BT_NUM];

// Modbus stuff starts from here: -----------------------------------------

const byte ID[] = {
  1, 2, 3    // each FC slave IDs
};

#define FC_NUM         sizeof(ID)  // how many slave Danfoss FCs we have actually

// Coil addresses (N-1 because addresses actually start from zero)
#define CTW_CADR       1-1 // control word   16 bit bitwise (Master to Slave)
#define REF_CADR      17-1 // reference word 16 bit bitwise (Master to Slave)
#define STW_CADR      33-1 // status word    16 bit bitwise (Slave to Master)
#define FRQ_CADR      49-1 // frequency word 16 bit bitwise (Slave to Master)
#define WRC_CADR      65-1 // write control   1 bit         (Master to Slave)
#define WRD_LEN       16   // coils per each word

// Holding register addresses
#define ERR_HADR       7-1 // last error code          byte (Slave to Master)
#define CTW_HADR   50000-1 // control word         uint16_t (Master to Slave)
#define REF_HADR   50010-1 // bus reference        uint16_t (Master to Slave)
#define STW_HADR   50200-1 // status word          uint16_t (Slave to Master)
#define MAV_HADR   50210-1 // main actual value    uint16_t (Slave to Master)
#define FMN_HADR    3020-1 // min frequency limit  uint16_t (Master to Slave)
#define FMX_HADR    3030-1 // max frequency limit  uint16_t (Master to Slave)

//Modbus net
ModbusRTU mb;

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Callback to monitor errors
  if (event != Modbus::EX_SUCCESS) {
    Serial.print("Request result: 0x");
    Serial.println(event, HEX);
  }
  return true;
}
// ----------------------- modbus stuff is finished -------------------------------

// An array to store the status word (STW) bitwise
bool    Stw[FC_NUM][WRD_LEN]{};

// Variables to store input&output holding registers
uint16_t  REFreg[FC_NUM] {};     // reference should be no more than 2^14, it is a full scale
uint16_t  MAVreg[FC_NUM] {};     // maximum actual value register shows actual frequency, it is equal to reference when the motor has reached its nominal speed
//uint16_t  TRYreg[FC_NUM] {};

// Float variables for frequency limits
float     MaxFrq[FC_NUM] {}; // in order to reduce errors of many-times multiplication and division
float     MinFrq[FC_NUM] {}; // we have to operate with frequency limits as floats and storing them in EEPROM as floats too

// Integer variables for frequency limits
uint16_t  FreqMax[FC_NUM][2] {}; // two uint16_t variables (4 bytes) per each FC
//uint16_t  FreqMin[FC_NUM][2] {}; // a minimum frequency limit, two uint16_t variables (4 bytes) per each FC
uint32_t *FMax = (uint32_t*)(&FreqMax); // to have a normal access to variable as uint32_t in milliHertz
//uint32_t *FMin = (uint32_t*)(&FreqMin);
uint16_t  FMXreg[FC_NUM][2]; // a double register (4 bytes) just a swapped version of FreqMax[][] to convert MC "litle endian" to FC "Big endian"
//uint16_t  FMNreg[FC_NUM][2];


// Flags&timestamps
bool           parChanged;             // setting up this flag means that the parameters have changed affecting the actual FC output frequency
bool           doButtonTask = false;   // flag shows that we need to do the button task now
bool           cmnBtnsHold[2] {};      // these flags are especially to store the hold status of the two first common-related buttons
bool           cmnBtnsRlsd[2] {};      // anologously for they release event
bool           redLEDout    = false;   // sapienti sat :)
unsigned long  lastMeasTime;           // the time when the last measurement has been performed
unsigned long  chTimeStamp;            // the time at which the last change of either limit or reference is made 
unsigned long  fbTimeStamp;            // the time when the last reading of STW and MAV registers has occurred 
#define        FB_PERIOD     1000      // how frequently to repeat the reading providing the feedback
#define        EE_WR_DELAY   3000      // the delay between the last parameter change and writing parameters to EEPROM

// EEPROM stuff
#define EEPROM_SETTINGS                0 // start Address for EEPROM data
#define EEPROM_MAX_WRITES            300 // maximum number of writes to EEPROM in one session
AcksenIntEEPROM IntEEPROM(EEPROM_SETTINGS);  // Initialise EEPROM library with defined start address

// Some pins definition
#define RED_LED_PIN  14 //!!!!
#define ANALOG_PIN   A2

// Analog measurements
// Variable to store an analog value measured from the output voltage of the upper FC.
// We use it as a reference to roughly set the starting frequency for all slave FCs
int     uppOut            = 0;  // upper FC measured analog output, 0 -- 1023
byte    measCounter       = 0;  // the counter shows how many times the uppOut has been measured
float   uppFrq;                 // frequency of the upper FC, Hz
#define UPP_FRQ_MIN         0   // upper FC minimum frequency
#define UPP_FRQ_MAX        50   // upper FC maximum frequency
#define R_SENS             99.7 // sensing resistor in current loop from upper FC, Ohm
#define I_LOOP_MIN          4   // minimum current in the current loop, mA
#define I_LOOP_MAX         20   // maximum current in the current loop, mA
#define I_LOOP_SCLFACTOR    0.5 // scale factor for the analog output so that 20 mA corresponds to I_LOOP_SCLFACTOR from the full scale of the upper FC
#define MEAS_PERIOD        15   // ANALOG_PIN measurements period
#define BLINK_PERIOD       200  // LED state change period , ms

// Corresponding relative reduction coefs corrected with the shaft diameters relation
// {(1435/7.3)/(1435/7.3), (1435/18)/(1435/7.3), (1435/39)/(1435/7.3)*(39/18)}
const float RelCoef[FC_NUM]  = 
  {
    1, 0.405555555556, 0.405555555556
  };

// Initialize samples array to compute the median, should be at least (TIME_TO_SET - TIME_TO_REPEAT)/MEAS_PERIOD, adjustable
RunningMedian uppSamples = RunningMedian(250);
  
void alarm(byte i) {
  char *message[] = {"Status word alarm bit is HIGH!",
                     "Upper FC frequency is not between Max Reference and Min Reference of slave FC",
                     "FC isn't ready, stopping!"};

  while (1) { // a useless eternal loop, blinking the LED, indicating the problem until the board has been reset
    digitalWrite(RED_LED_PIN, HIGH);
    delay(500);
    digitalWrite(RED_LED_PIN, LOW);
    delay(500);
    Serial.println(message[i]);
  }
  
}

void sendFrqLim(byte k) {    // send MaxFrq to k-th FCs (k can change from 0 to FC_NUM)

  //FMin[k] = (uint32_t)round(1000*MinFrq[k]);// Serial.print(FMin[k]); Serial.print("  "); Serial.print(FreqMin[k][0]); Serial.print("  "); Serial.println(FreqMin[k][1]); delay(5); // correct conversion of float type into uint32_t
  FMax[k] = (uint32_t)round(1000*MaxFrq[k]);// Serial.print(FMax[k]); Serial.print("  "); Serial.print(FreqMax[k][0]); Serial.print("  "); Serial.println(FreqMax[k][1]); delay(5);
  
  //FMNreg[k][0] = FreqMin[k][1]; // making a swapped register copy to convert MC little endian to FC Big endian 
  //FMNreg[k][1] = FreqMin[k][0];
  FMXreg[k][0] = FreqMax[k][1];
  FMXreg[k][1] = FreqMax[k][0];

  /*
  if (!mb.slave()) {    // Check if no transaction in progress
    mb.writeHreg(ID[k], FMN_HADR,  FMNreg[k], 2, cb); // Writing double (4 byte) min limit register
  }
  while(mb.slave()) { // Check if transaction is active
    mb.task();
    delay(5);
  }
  */
  
  if (!mb.slave()) {    // Check if no transaction in progress
    mb.writeHreg(ID[k], FMX_HADR,  FMXreg[k], 2, cb); // Writing double (4 byte) max limit register
  }
  while(mb.slave()) { // Check if transaction is active
    mb.task();
    delay(5);
  }
  if (Serial) {
    Serial.print("MaxFrq[");
    Serial.print(k);
    Serial.print("] = ");
    Serial.print(MaxFrq[k]);
    Serial.println(" sent.");
  }
}

void readFrqLim(byte k) {    // read MaxFrq from k-th FCs (k can change from 0 to FC_NUM)

  /*
  if (!mb.slave()) {    // Check if no transaction in progress
    mb.readHreg(ID[k], FMN_HADR,  FMNreg[k], 2, cb); // Writing double (4 byte) min limit register
  }
  while(mb.slave()) { // Check if transaction is active
    mb.task();
    delay(10);
  }
  */
  
  if (!mb.slave()) {    // Check if no transaction in progress
    mb.readHreg(ID[k], FMX_HADR,  FMXreg[k], 2, cb); // Writing double (4 byte) max limit register
  }
  while(mb.slave()) { // Check if transaction is active
    mb.task();
    delay(5);
  }

  //FreqMin[k][1] = FMNreg[k][0]; // to convert FC Big endian to MC little endian
  //FreqMin[k][0] = FMNreg[k][1];
  FreqMax[k][1] = FMXreg[k][0];
  FreqMax[k][0] = FMXreg[k][1];

  //MinFrq[k] = (float)FMin[k]/1000;// correct conversion of uint32_t into the float
  MaxFrq[k] = (float)FMax[k]/1000;
  
  if (Serial) {
    Serial.print("MaxFrq[");
    Serial.print(k);
    Serial.print("] = ");
    Serial.print(MaxFrq[k]);
    Serial.println(" read.");
  }
}

void sendREFreg(byte k) {    // send REFreg to k-th FCs (k can change from 0 to FC_NUM)

  if (!mb.slave()) {    // Check if no transaction in progress
    mb.writeHreg(ID[k], REF_HADR,  &REFreg[k], 1, cb); // Writing single (2 byte) reference register
  }
  
  while(mb.slave()) { // Check if transaction is active
    mb.task();
    delay(5);
  }
  
  if (Serial) {
    Serial.print("REFreg[");
    Serial.print(k);
    Serial.print("] = ");
    Serial.print(REFreg[k]);
    Serial.println(" sent.");
  }
}

void readMAVreg(byte k) {
  
  if (!mb.slave()) {    // Check if no transaction in progress
      mb.readHreg(ID[k], MAV_HADR,  &MAVreg[k], 1, cb); // Reading single MAV register
  }
    
  while(mb.slave()) { // Check if transaction is active
      mb.task();
      delay(5);
  }
/*   
  Serial.print("MAVreg[");
  Serial.print(k);
  Serial.print("] = ");
  Serial.print(MAVreg[k]);
  Serial.println(" read.");
*/  
}

void readREFreg(byte k) {
  
  if (!mb.slave()) {    // Check if no transaction in progress
      mb.readHreg(ID[k], REF_HADR,  &REFreg[k], 1, cb); // Reading single MAV register
  }
    
  while(mb.slave()) { // Check if transaction is active
      mb.task();
      delay(5);
  }
  if (Serial) {
    Serial.print("REFreg[");
    Serial.print(k);
    Serial.print("] = ");
    Serial.print(REFreg[k]);
    Serial.println(" read.");
  }
}

void readSTWcoil(byte k) {
  
  if (!mb.slave()) {    // Check if no transaction in progress
    mb.readCoil(ID[k], STW_CADR, Stw[k], WRD_LEN, cb);
  }
    
  while(mb.slave()) { // Check if transaction is active
    mb.task();
    delay(5);
  }
  /* 
  for (byte n = 0; n < WRD_LEN; n++) {
    Serial.println(Stw[k][n]);
  }
  Serial.println(); */
}

void setup() {

  byte STWreadTries = 0; // A counter of the status word (STW) initial reading tries
  
  // Initialize button objects
  for (byte i = 0; i < BT_NUM; i++) Bt[i].begin(Pin[i]);

  // Initialize both serial ports
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8E1);
  
  // Initialize the analog pin
  analogReference(INTERNAL);    // a built-in 2.56 Volts reference for ATmega32U4

  // Initialize other digital pins which are non-associated with the buttons
  pinMode(RED_LED_PIN, OUTPUT); // Red LED blinking indicates the problem occurred
        
  // Initialize Modbus
  mb.begin(&Serial1);
  mb.setBaudrate(9600);
  mb.master();

  // Initialise EEPROM variables for EEPROMEx
  EEPROM.setMemPool(0, EEPROMSizeATmega32u4);  // see EEPROMex.h for other options
  EEPROM.setMaxAllowedWrites(EEPROM_MAX_WRITES);
  
  IntEEPROM.resetPresentAddress();  // Ensure that EEPROM Present Memory Address is reset to Starting Address
  
  // Initialize FCs
  for (byte fc = 0; fc < FC_NUM; fc++) { // for each FC

    // An initial status word reading
    do {
    
      delay(1000);
    
      readSTWcoil(fc);    // reading FC status word
      STWreadTries++;
    
      if ( (STWreadTries > 3) ) { // or so many tries and FC still isn't ready
        alarm(2);// FC isn't ready, stop!!!
      };
    
    } while (!Stw[fc][0] || !Stw[fc][1]); // while Control or FC are not ready

    STWreadTries = 0;
    
    if (Stw[fc][3]) { // if there is alarm
      alarm(0);
    };
      
    if (Stw[fc][11]) { // if this FC is running at the moment of the setup
      readREFreg(fc);  // then we are reading the current data from it to prevent any changes
      readFrqLim(fc);
      parChanged  = true;
    }
    else {  // if it is not running now so let's upload the data from controller EEPROM
      MinFrq[fc] = IntEEPROM.readEEPROMValueFloat();
      //MinFrq[fc] =  0.00;
      MaxFrq[fc] = IntEEPROM.readEEPROMValueFloat();
      //MaxFrq[fc] = 50;
      REFreg[fc] = IntEEPROM.readEEPROMValueInt();
      //REFreg[fc] = 8192;
      sendFrqLim(fc);      // send frequency limits from MaxFrq and MinFrq to fc-th FC
      sendREFreg(fc);      // send initial references from REFreg to fc-th FC
    }
    
    mb.writeCoil(ID[fc], WRC_CADR, true, cb); // Parameter changes are written both to the RAM and EEPROM of the FC
    while(mb.slave()) {
      mb.task();
      delay(5);
    }

    Serial.println(fc);
    Serial.println(MinFrq[fc]);
    Serial.println(MaxFrq[fc]);
    Serial.println(REFreg[fc]);
    Serial.println();

    // The time stamps initialization
    fbTimeStamp = millis();
    chTimeStamp = millis();
  }
  
}

void loop() {
  
  byte          i = 0; // button counter
  byte          f = 0; // FC index especially for out-of-cycle use
  uint32_t oldBlinkTime = 0; // it is to store an old measurement counter value
  
 // Button-related procedures
  do { // for i-th button
    
    Bt[i].poll(); // polling i-th button

    // When the button is just pressed
    if (Bt[i].onPress()) {
      Bt[i].clearTimer();
      retriggerMs[i] = TIME_TO_REPEAT;
      Step[i] = INIT_STEP;
      doButtonTask = true;
    }

    // When the button is pressed for more than TIME_TO_REPEAT milliseconds
    if (Bt[i].pressedFor(TIME_TO_REPEAT)) {
      retriggerMs[i] = RETRIG_PER;
      if (i == 0 || i == 1) cmnBtnsHold[i] = true; // remember this to detect the two first buttons being pressed at the same time
    }

    // When both two first common-related buttons are being held for a time more than TIME_TO_REPEAT
    if (cmnBtnsHold[0] && cmnBtnsHold[1]) {

      doButtonTask = false;   // all the button tasks stop, this is an exception
      parChanged   = false;   // don't write anything into EEPROM for a while
      fbTimeStamp = millis(); // to postpone the automatic feedback

      if (millis() > (lastMeasTime + MEAS_PERIOD)) {
        
        // Measure the output reference signal of the "upper" FC as a 10-bit unsigned integer from 0 to 2.56 Volts (should be divided from 10 Volts range)
        uppOut = analogRead(ANALOG_PIN); // the full scale is 0 -- 2.55 Volts, which corresponds to 0 -- 1023 or 0 -- 16383 upper FC reference
        lastMeasTime = millis();
        uppSamples.add(uppOut);
        measCounter++;
        Serial.println(uppOut);
         
        //if (millis() > (oldBlinkTime + BLINK_PERIOD)) {
          redLEDout = !redLEDout; //blinking the red LED
          digitalWrite(RED_LED_PIN, redLEDout);
        //  oldBlinkTime = millis();
        //}
        
      }
      
      if (Bt[0].pressedFor(TIME_TO_SET) && Bt[1].pressedFor(TIME_TO_SET)) {
        
        Bt[0].clearTimer();
        Bt[1].clearTimer();
        digitalWrite(RED_LED_PIN, LOW); // stop lightning on the LED
        
        // Upper FC frequency, Hz. It's calculated from the median of the reference analog output sample, 0 -- 1023
        uppOut = uppSamples.getMedian();
        uppFrq = UPP_FRQ_MIN
                +I_LOOP_SCLFACTOR*(UPP_FRQ_MAX - UPP_FRQ_MIN)
                *(2.56*((float)uppOut/1023)/R_SENS - 1e-3*I_LOOP_MIN)/(1e-3*(I_LOOP_MAX - I_LOOP_MIN));
        Serial.println(uppOut);
        Serial.println(measCounter);
        Serial.println(uppFrq);
        measCounter = 0;
                
        for (byte fc = 0; fc < FC_NUM; fc++) { // for each FC
          MaxFrq[fc] = 50.000; // the standard upper frequency limit
          MinFrq[fc] =  0.000; // the standard lower frequency limit
          // A computation procedure of the new REFreg value
          if (uppFrq >= MinFrq[fc] && uppFrq <= MaxFrq[fc]) {
            REFreg[fc] = (uint16_t)round( 0x3FFF*(RelCoef[fc]*uppFrq - MinFrq[fc])/(MaxFrq[fc] - MinFrq[fc]) );
            sendFrqLim(fc);
            sendREFreg(fc); // sending the frequency to FC so that the drawing speeds of all three shafts are equal
          }
          else alarm(1);          
        }
        chTimeStamp = millis();
        parChanged  = true;
        
        /*
        for (byte n = 0; n < 8; n++) { // fast blinking the LED
          redLEDout = !redLEDout;
          digitalWrite(RED_LED_PIN, redLEDout);
          delay(50);
        } */      
      }
    }
    else {
      // If there is no exception, check the button is retriggered
      if (Bt[i].isPressed() && Bt[i].retrigger(retriggerMs[i])) {
        retriggerCount[i]++;
        if (retriggerCount[i] > COUNT_TO_ACCEL) {
          if (Step[i] < MAX_STEP) {
            Step[i] += ACCEL_STEP_UP; // to change the parameter faster and faster while the button is long pressed
          }
          else {
            Step[i] = MAX_STEP;
          }
        }
        doButtonTask = true;
      }
    }

    // When the button is just released
    if (Bt[i].onRelease()) {
      if (i == 0 || i == 1) {
        cmnBtnsRlsd[i] = true;
        cmnBtnsHold[i] = false;              // we forget about it was held
        measCounter = 0;
        //Serial.println("HERE!!!");
    //    Bt[0].clearTimer();
    //    Bt[1].clearTimer();
      }
      //else {
        Bt[i].clearTimer();
      //}
      retriggerCount[i] = 0; 
      doButtonTask = false; //just to be safe 
    }

    if (cmnBtnsRlsd[0] && cmnBtnsRlsd[1]) { // if the buttons are _both_ released now
       cmnBtnsHold[0] = false;              // we forget about they were held
       cmnBtnsHold[1] = false;
       cmnBtnsRlsd[0] = false;              // and released then
       cmnBtnsRlsd[1] = false;
       digitalWrite(RED_LED_PIN, LOW); // stop lighting the LED
    }
    
    // At last, we are here to do the task assigned to the button
    if (doButtonTask) {
      
      fbTimeStamp = millis(); // to postpone the automatic feedback
      
      switch (i) {        
     
        case 0: // Maximum frequency for all FCs is about to increment
        //Serial.println("0");
        for (byte fc = 0; fc < FC_NUM; fc++) { // for each FC
          MaxFrq[fc] *= 1 + (float)Step[i]/1000;
          if (MaxFrq[fc] > MAXFRQ_LIM) MaxFrq[fc] = MAXFRQ_LIM;
          sendFrqLim(fc); // sending the frequency limits to fc-th FC
        }
        chTimeStamp = millis();
        parChanged  = true;
        i++; // the check for the next button state is needed to detect the special case of two buttons pressed at a time
        break;
        
        case 1: // Maximum frequency for all FCs is going to be decremented
        for (byte fc = 0; fc < FC_NUM; fc++) { // for each FC
          MaxFrq[fc] /= 1 + (float)Step[i]/1000;
          if (MaxFrq[fc] < 10) MaxFrq[fc] = 10; // to prevent zeroing
          sendFrqLim(fc);
        }
        chTimeStamp = millis();
        parChanged  = true;
        i++;
        break;

        case 2: // FC 0 reference increment
        case 4: // FC 1 
        case 6: // FC 2
        f = (i-2)/2; // the FC number depends on which button is pressed
        if (REFreg[f] < 0x4000 ) {
          REFreg[f] += Step[i];
        }
        else {
          REFreg[f] = 0x4000;
        }
        sendREFreg(f); // sending reference to corresponding FC
        chTimeStamp    = millis();
        parChanged     = true;
        i += 2; // skipping the next button check because there is no sense to do it this time
        break;

        case 3: // FC 0 reference decrement
        case 5: // FC 1 
        case 7: // FC 2 
        f = (i-3)/2;
        if (REFreg[f] >= Step[i] ) {     // to prevent overflow
          REFreg[f] -= Step[i];
        }
        else {
          REFreg[f] = 0;
        }
        sendREFreg(f); // sending reference to corresponding FC
        chTimeStamp    = millis();
        parChanged     = true;
        i++;
        
      }
      doButtonTask = false; // button task is done
    }
    else {
      i++; // even if there is no buttontask to do we just ought to increment a button counter
    }
  } while (i < BT_NUM);

  // Parameters writing to EEPROM
   if (parChanged) {
    
    if (millis() > (chTimeStamp + EE_WR_DELAY)) {
      
      IntEEPROM.resetPresentAddress();  // Ensure that EEPROM Present Memory Address is reset to Starting Address
       for (byte fc = 0; fc < FC_NUM; fc++) { 
        // rewriting the values into EEPROM passing EE_WR_DELAY milliseconds after any parameter last change
        IntEEPROM.writeEEPROMValueFloat(MinFrq[fc]); // write data out to EEPROM
        IntEEPROM.writeEEPROMValueFloat(MaxFrq[fc]);
        IntEEPROM.writeEEPROMValueInt(REFreg[fc]);
        Serial.println(fc);
        Serial.println(MinFrq[fc]);
        Serial.println(MaxFrq[fc]);
        Serial.println(REFreg[fc]);
        Serial.println();
      }
      
      parChanged = false; // All the parameters have been saved into EEPROM including the changed one
    
    }
    
  }
          
  // FCs feedback
  if (millis() > (fbTimeStamp + FB_PERIOD)) { //it's time to refresh the FC STWs & MAVs
    
    fbTimeStamp = millis();
    
    for (byte fc = 0; fc < FC_NUM; fc++) { // for each FC
      
      readSTWcoil(fc);
      if (Stw[fc][3]) alarm(0);
      
      //readMAVreg(fc);
        
    }

  }
 
}
