// Info:

// From CAN ZERO:

//  0x240
//       SOC = buf[6]  byte 6 is State Of Charge on dash in %
//           no Byte 7
//  BMS ----------------------------
//  BMS_PACK_ACTIVE_DATA ID             0x408
//           byte 0
//      BatteryTemp = buf[1];   byte 1 is battery temp in celcius
//           byte 2 (same as byte 1) could be second temp sensor ?? needs change in battery temp)
//      BatteryAmps = buf[3] + 256 * buf[4];  // byte 3 battery amps, both when charging and
//            byte 5 (fixed at 131)
//            byte 6 (fixed at 0)
//            byte 7 (fixed at 255)
//  BMS_PACK_STATUS                     0x188
//      SOC2 = buf[0];
//      ChargeCycles = buf[3] + 256 * buf[4]   byte 3 is number of charge cycles (probably byte 4 as well (highbyte)
//      CellBalance = buf[5];                  byte 5 could be Cel Balance value
//  BMS_PACK_CONFIG                     0x288
//  BMS_PACK_STATS                      0x308
//        ?? byte 0 (very random ?? 0-25 when speed = 0 , goes higher/lower when powering or regen)
//        ?? byte 1 (very reandom ?? 246-248 when speed = 0, goes higher/lower when powering or regen)
//        ?? byte 2 (always 15)
//      Vbatt = 0.001 * buf[3] + 0.256 * buf[4] + 65.535 * buf[5];  // byte 3, 4 and 5 are Vbatt in units of mV (divide by 1000 to get Volts)
//  BMS_CELL_VOLTAGE                    0x388
//  BMS_PACK_TEMP_DATA                  0x488
//  BMS_PACK_TIME                       0x508
//  BMS_CONTROL                         0x506
//
//  CALEX -----------------------------------
//  CALEX_CHARGE_CONTROL                0x0206
//           ?? byte 0 (batterypack temperature ??)
//           ?? byte 1 (fixed at 2 during charging)
//      Charger0Voltage = buf[2] * 0.001 + buf[3] * 0.256 + buf[4] * 65.535;   byte 2-3-4 Charger voltage in mV (division by 1000 gets Volts)
//           ?? byte 5 (fixed at 1)
//           ?? byte 6 (fixed at 0)
//           ?? byte 7 (fixed at 0)
//  CALEX_CHARGE_STATUS                 0x192
//  T_CALEX_MAX_CHARGE_VOLTAGE_CURRENT  0x292
//  R_CALEX_MAX_CHARGE_VOLTAGE_CURRENT  0x306
//  CALEX_TAPER_CUTOFF_CURRENT          0x406
//
// DASH -------------------------------------
// DASH_STATUS                          0x1C0
// DASH_ODO_FROM_DASH                   0x2C0
// DASH_STATUS2                         0x440
//    long trip 2 km * 100
//    short range km * 100 ( * MI_TO_KM )
//    short temp C * 100
// DASH_STATUS3                         0x540
//     ? empty
//    short  wh/km * 100 ( * KM_TO_MI )
//    short  trip av wh/km * 100
//    short  life av wh/km * 100
// DASH_ODO_TO_DASH                     0x3C0







#include <Adafruit_ADS1X15.h>

#include <mcp_can.h>
#include <SPI.h>
#include <SoftwareSerial.h>
//#include <SimpleTimer.h>

// bluetooth
const int rxPin = 5;  // Broche 11 en tant que RX, à raccorder sur TX du HC-05
const int txPin = 6;  // Broche 10 en tant que TX, à raccorder sur RX du HC-05
SoftwareSerial bluetooth(rxPin, txPin);

const int CALEX_RELAY_PIN = 3;  // Arduino pin connected to relay's pin
const int TYPE2_RELAY_PIN = 2;  // Arduino pin connected to relay's pin enable charge by TYPE

//Constants
const bool DEBUG = true;            // for debug without CAN connection
const int BATTERYCAPACITY = 14400;  //Wh

//const int  LED_BUILTIN;
const int ZERO_SPI_CS_PIN = 10;    //CS Pin for ZERO CAN
const int CHARGER_SPI_CS_PIN = 9;  //CS Pin for charger CAN


// Global

// CAN ADRESS for charger
unsigned long int sendId1 = 0x1806E7F4;
// unsigned long int sendId2 = 0x0;
// unsigned long int sendId3 = 0x0;
unsigned long int receiveId1 = 0x18FF50E7;
// unsigned long int receiveId2 = 0x0;
// unsigned long int receiveId3 = 0x0;

// variables for CAN
unsigned long int receiveId;  //ID of either charger
unsigned char buf[8];         //Buffer for data from CAN message of either charger
unsigned char buf0[8];        //Buffer for data from CAN message of charger 0
unsigned char buf1[8];        //Buffer for data from CAN message of charger 1
// unsigned char buf2[8];        //Buffer for data from CAN message of charger 2
// unsigned char buf3[8];        //Buffer for data from CAN message of charger 3

unsigned char len = 0;  //Length of received CAN message of either charger
unsigned long ChargeCycles;
unsigned long CellBalance;
unsigned long Charger0Voltage;
unsigned long BatterySOC;
unsigned long BatteryTemp;
unsigned long BatteryAmps;
unsigned long BatteryVoltage;

// STATUS APPLICATION
int status = 0;  // Status for control app 0 STOP 1 CANOK 2 RUN 3 WAIT 4 PENDING 5 PAUSE 6 PRECHARGE  > 80 ERROR

bool chargerStatusCAN;
bool zeroStatusCAN;
unsigned long previousTimeLoop;
unsigned long previousTimeSecond;
int hours = 0;
int minutes = 0;
int seconds = 0;
int myTimestamp = 0;
int memmyTimestamp = 0;

int charger0CurrentMax = 125;                                                                               // Values charger0 (Altex origne with sensor) current max A *10
int charger1CurrentMax = 320;                                                                               // Values charger2 current max A *10
int charger2CurrentMax = 0;                                                                                 // Values charger3 current max A *10
int charger3CurrentMax = 0;                                                                                 // Values charger4 current max A *10
long chargeCurrentMax = charger0CurrentMax + charger1CurrentMax + charger2CurrentMax + charger3CurrentMax;  // total Current charge Maximum
long chargeCurrentMin = charger0CurrentMax;                                                                 // curent charge minimun ( altex charger here )
float chargePowerMax = chargeCurrentMax * 0.103;                                                            // We assume the nominal voltage, 103 V to be the reference for power calculations.
float chargeEnergyInput = 0;                                                                                // Value used to know the amount of charge put into the battery in realtime.
int targetSOC = 80;
int targetCUR = 12;
long total_current = 0;
float maxvoltage = 1162;
word outputvoltage = 1162;              //set max voltage to 116,2V (offset = 0,1)
word outputcurrent = chargeCurrentMax;  //set max current to 320 (32A, offset = 0,1)

String inputString = "";
String command = "";
String value = "";
boolean stringComplete = false;

int toDifferHour = 0;  // To differ charge start

//Declare objects

MCP_CAN CANC(CHARGER_SPI_CS_PIN);  //Set CS pin for charger
MCP_CAN CANZ(ZERO_SPI_CS_PIN);     //Set CS pin for charger


//----------------------------------------------------------------------
// ----------------------------SETUP------------------------------------
//----------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Welcome to TCCharger Setup");
  pinMode(CALEX_RELAY_PIN, OUTPUT);  // set arduino pin to output mode
  pinMode(TYPE2_RELAY_PIN, OUTPUT);  // set arduino pin to output mode
  pinMode(LED_BUILTIN, OUTPUT);      //LED start
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  // bluetooth
  // define pin modes for tx, rx pins:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  bluetooth.begin(9600);
  Serial.println("with bluetooth control (HC-06)");

  delay(1000);

  // CAN connetions
  initCan();

  previousTimeLoop = millis();
  previousTimeSecond = millis();
}

void initCan() {

  chargerStatusCAN = false;

  // CAN init charger
  if (DEBUG != true) {
    while (CAN_OK != CANC.begin(CAN_250KBPS)) {  // CAN Bus initialisation
      Serial.println("CAN Charger Initialization failed, restart");
      delay(200);
    }
  } else {
    chargerStatusCAN = true;
  }
  // CAN init Zero
  if (DEBUG != true) {
    while (CAN_OK != CANZ.begin(CAN_250KBPS)) {  // CAN Bus initialisation
      Serial.println("CAN Zero Initialization failed, restart");
      delay(200);
    }
  } else {
    zeroStatusCAN = true;
    Serial.println("CAN initialization successful");
  }
}

//----------------------------------------------------------------------
// ----------------------------LOOP-------------------------------------
//----------------------------------------------------------------------
void loop() {

  if ((chargerStatusCAN && zeroStatusCAN) | DEBUG) {
    status = 4;
    digitalWrite(LED_BUILTIN, LOW);

    // read bluetooth command
    if (bluetooth.available()) {
      // get the new byte:
      char inChar = (char)bluetooth.read();
      inputString += inChar;
      //Serial.print(inChar);
      // newline or a carriage return, set a flag
      if (inChar == '\n' || inChar == '\r') {
        stringComplete = true;
        Serial.println("Data BT in");
        Serial.println(inputString);
      }
    }

    // VAlue before Bluetooth communication
    // calcul timelaps or search on CAN ZERO

    float currentMaxVolatge = voltage();
    float total_current = currentSum();
    char timelaps = timelapsEstimate(total_current, currentMaxVolatge, BatterySOC);
    int intValue = 0;
    // parse command from bluetooth
    if (stringComplete) {
      Serial.println(inputString);
      boolean stringOK = false;
      if (inputString.startsWith("CMD ")) {
        inputString = inputString.substring(4);
        int pos = inputString.indexOf('=');
        if (pos > -1) {
          command = inputString.substring(0, pos);
          value = inputString.substring(pos + 1, inputString.length() - 1);  // extract command up to \n exluded
          if (DEBUG) {
            Serial.println(command);
            Serial.println(value);
          }
          if (command.equals("STATUS")) {
            value.equals("ON") ? status = 3 : status = 6;  // ON > PRERUN : OFF > PREPAUSE
            stringOK = true;
          } else if (command.equals("ONLYCALEX")) {
            value.equals("ON") ? status = 4 : Serial.println("OnlyCalex OFF");
            stringOK = true;
          } else if (command.equals("TARGETSOC")) {  // TARGETSOC=value
            intValue = value.toInt();
            if (intValue > 0) {
              targetSOC = (float)intValue;
              stringOK = true;
            }
          } else if (command.equals("TARGETCUR")) {  // TARGETSOC=value
            intValue = value.toInt();
            if (intValue > 0) {
              targetCUR = (float)intValue;
              stringOK = true;
            }
          }
        }  // pos > -1
        else if (inputString.startsWith("STATUS")) {
          Serial.print("STATUS VOLTAGE=");
          Serial.println(currentMaxVolatge);
          Serial.print("STATUS CURRENT=");
          Serial.println(total_current);
          Serial.print("STATUS KWCHARGED=");
          Serial.println(chargeEnergyInput);
          Serial.print("STATUS BATSOC=");
          Serial.println(BatterySOC);
          Serial.print("STATUS TIMELAPS=");
          Serial.println(timelaps);
          stringOK = true;
        }  // inputString.startsWith("STATUS")
      }    // inputString.startsWith("CMD ")
      stringOK ? Serial.println("Command Executed") : Serial.println("Invalid Command");
      // clear the string for next iteration
      inputString = "";
      stringComplete = false;
    }  // stringComplete


    // loop 100ms for work
    if (millis() >= (previousTimeLoop)) {
      previousTimeLoop = previousTimeLoop + 100;
      // loop 1s for count Time
      if (millis() >= (previousTimeSecond)) {
        previousTimeSecond = previousTimeSecond + 1000;
        seconds = seconds + 1;
        myTimestamp = myTimestamp + 1;
        if (seconds == 60) {
          seconds = 0;
          minutes = minutes + 1;
        }
        if (minutes == 60) {
          minutes = 0;
          hours = hours + 1;
        }
        if (hours == 13) {
          hours = 1;
        }
        if (DEBUG) {
          Serial.println("---------------------");
          Serial.print(hours, DEC);
          Serial.print(":");
          Serial.print(minutes, DEC);
          Serial.print(":");
          Serial.println(seconds, DEC);
          Serial.print("Set charging current: ");
          Serial.print((float)targetCUR / 100);  //targetCurrent
          Serial.println(" A");
          Serial.print("Actual charging current: ");
          Serial.print((float)total_current / 10.0);  //Output current setpoint
          Serial.println(" A");
          Serial.print("Set max SOC: ");
          Serial.print(targetSOC);
          Serial.print(" % (Max voltage ");
          Serial.print((float)outputvoltage / 10.0);  //Target SOC to stop charge
          Serial.println(" V)");
          Serial.print(BatteryTemp);
          Serial.println(" °C )");
        }

        // STATUS
        // auto ONLYCALEX after
        // Status for control app 0 STOP 1 CANOK 2 RUN 3 PRERUN 4 ONLYCALEX 5 PAUSE 6 PREPAUSE > 80 ERROR
        if (status == 0) {  // STOP
          digitalWrite(CALEX_RELAY_PIN, LOW);
        } else if (status == 2 && memmyTimestamp == (myTimestamp + 2)) {  // RUN
          setCurrent(targetCUR);
          digitalWrite(TYPE2_RELAY_PIN, HIGH);
        } else if (status == 3) {  // PRERUN
          digitalWrite(CALEX_RELAY_PIN, HIGH);
          memmyTimestamp = myTimestamp;
          status = 2;
        } else if (status == 4) {  // ONLYCALEX
          digitalWrite(CALEX_RELAY_PIN, HIGH);
          setCurrent(charger0CurrentMax);
        } else if (status == 5 && memmyTimestamp == (myTimestamp + 2)) {  // PAUSE
          digitalWrite(CALEX_RELAY_PIN, LOW);
        } else if (status == 6) {  // PREPAUSE cut all charger before cut CALEX for cut corectly
          setCurrent(0);
          digitalWrite(TYPE2_RELAY_PIN, LOW);
          memmyTimestamp = myTimestamp;
          status = 5;
        } else if (status >= 80) {  // ERROR
          // stop charge
          setCurrent(0);
          delay(2000);
          digitalWrite(CALEX_RELAY_PIN, LOW);
          // TODO remonter l'erreur sur le telephone et le port serie
        }


        // can send and received message
        if (status < 80) {
          digitalWrite(LED_BUILTIN, HIGH);
          readCanMessage();
        }

        // WRITE seulement si run RUN or PREPAUSE or ONLYCALEX
        if (status == 2 || status == 4 || status == 6) {
          writeCanMessage();
        }

        if (DEBUG) {
          status = 3;
        }







        int toDifferHour;

        if (targetSOC >= 100) {
          targetSOC = 100;
        }
        setVoltage(socToMaxVoltage(targetSOC));



        // bluetooth send Status
        currentMaxVolatge = voltage();
        // current from charger obsoléte
        // total_current = currentSum();
        total_current = BatteryAmps;
        chargeEnergyInput = calculateEnergy(chargeEnergyInput);
        // Print real charging current as provided by the charger
        bluetooth.print("STATUS VOLTAGE=");
        bluetooth.println(currentMaxVolatge);
        bluetooth.print("STATUS CURRENT=");
        bluetooth.println(total_current);
        bluetooth.print("STATUS KWCHARGED=");
        bluetooth.println(chargeEnergyInput);
        bluetooth.print("STATUS BATSOC=");
        bluetooth.println(BatterySOC);
        bluetooth.print("STATUS BATTEMP=");
        bluetooth.println(BatteryTemp);
        bluetooth.print("STATUS TIMELAPS=");
        bluetooth.println(timelaps);
      }

      // ??????
      //delay(10);  //This will ensure one loop per second
    }
  }
  if (status >= 80) {
    // try reconnect CAN connections
    initCan();
  }
}

/************************************************
** Function name:           canRead
** Descriptions:            read CAN message of chargerID
*************************************************/

void chargerCanRead(unsigned long int chargerId) {

  if (CAN_MSGAVAIL == CANC.checkReceive()) {  //Check for messages

    CANC.readMsgBuf(&len, buf);  // read data, len: data length, buf: data buffer

    unsigned long int t_receiveId = CANC.getCanId();  //Reading CAN-ID from either charger

    if (t_receiveId == chargerId) {  //CAN Bus ID from TC charger protocol 998
      Serial.println("TC CAN Data received!");
      Serial.print("CAN ID: ");
      Serial.print(receiveId, HEX);  //Output ID

      Serial.print(" / CAN Data: ");
      for (int i = 0; i < len; i++) {  //Output data

        if (buf[i] < 0x10) {  // Displaying zero if only one digit
          Serial.print("0");
        }

        Serial.print(buf[i], HEX);
        Serial.print(" ");  // Spaces
      }

      Serial.println();  //Prints an empty paragraph

      Serial.print("Charging voltage: ");
      float currentMaxVolatge = (((float)buf[0] * 256.0) + ((float)buf[1])) / 10.0;  //highByte/lowByte + offset

      Serial.print(currentMaxVolatge);
      Serial.print(" V / Charging current: ");
      float total_current = (((float)buf[2] * 256.0) + ((float)buf[3])) / 10.0;  //highByte/lowByte + offset
      Serial.print(total_current);
      Serial.println(" A");  //Paragraph

      switch (buf[4]) {  //Read out error byte

        case B00000001:
          Serial.println("Error: hardware error");
          status = 92;
        case B00000010:
          Serial.println("Error: overheating");
          status = 93;
        case B00000100:
          Serial.println("Error: input voltage not allowed");
          status = 94;
        case B00001000:
          Serial.println("Error: battery not connected");
          status = 95;
        case B00010000:
          Serial.println("Error: CAN bus error");
          status = 96;
        case B00001100:
          Serial.println("Error: No input voltage");
          status = 97;
      }
    }
  }
}


/************************************************
** Function name:           canWrite
** Descriptions:            write CAN message
*************************************************/
String canWrite(unsigned char t_data[8], unsigned long int t_id, char t_chargerNumber) {

  byte sndStat = CANC.sendMsgBuf(t_id, 1, 8, t_data);  //Send message (ID, extended frame, data length, data)

  if (sndStat == CAN_OK) {  //Status byte for transmission
    return "CAN message sent successfully to charger " + t_chargerNumber;
  } else {
    status = 91;
    return "Error during message transmission to charger " + t_chargerNumber;
  }
}

/************************************************
** Function name:           setVoltage
** Descriptions:            set target voltage
*************************************************/

void setVoltage(int t_voltage) {  //can be used to set desired voltage to i.e. 80% SOC

  if (t_voltage >= 980 && t_voltage <= 1164) {

    outputvoltage = t_voltage;
  }
}

/************************************************
** Function name:           setCurrent
** Descriptions:            set target current
*************************************************/

void setCurrent(int t_current) {  //can be used to reduce or adjust charging speed

  if (t_current >= 0 && t_current <= 320) {

    outputcurrent = t_current;
  }
}


/************************************************
** Function name:           timelapsEstimate
** Descriptions:            calcul timelaps
*************************************************/

char timelapsEstimate(float t_current, float t_voltage, unsigned long t_SOC) {

  float nbHour = (BATTERYCAPACITY * ((100 - t_SOC) / 100)) / t_current * t_voltage;
  return '2';  // todo
}

/************************************************
** Function name:           refreshCanMessage
** Descriptions:            read CAN charger and CAN Zero
*************************************************/

void readCanMessage() {  //Cyclic function called by the timer

  if (DEBUG != true) {
    // send CAN message
    unsigned char voltamp[8] = { highByte(outputvoltage), lowByte(outputvoltage), highByte(outputcurrent), lowByte(outputcurrent), 0x00, 0x00, 0x00, 0x00 };  //Regenerate the message
    Serial.println(canWrite(voltamp, sendId1, '1'));                                                                                                          // charger 1
    // Serial.println(canWrite(voltamp, sendId2, '2'));                                                                                                          // charger 2
    // Serial.println(canWrite(voltamp, sendId3, '3'));

    //received CAN message
    chargerCanRead(receiveId1);  //Call read function of charger 1
    for (int i = 0; i < len; i++) {
      buf1[i] = buf[i];
    }
    // chargerCanRead(receiveId2);  //Call read function of charger 2
    // for (int i = 0; i < len; i++) {
    //   buf2[i] = buf[i];
    // }
    // chargerCanRead(receiveId3);  //Call read function of charger 3
    // for (int i = 0; i < len; i++) {
    //   buf3[i] = buf[i];
    // }

    zeroCanRead();
  }
  Serial.println();  //Print a blank line
}

/************************************************
** Function name:           writeCanMessage
** Descriptions:            write CAN charger
*************************************************/

void writeCanMessage() {

  if (DEBUG != true) {
    // send CAN message
    unsigned char voltamp[8] = { highByte(outputvoltage), lowByte(outputvoltage), highByte(outputcurrent), lowByte(outputcurrent), 0x00, 0x00, 0x00, 0x00 };  //Regenerate the message
    Serial.println(canWrite(voltamp, sendId1, '1'));                                                                                                          // charger 1
    // Serial.println(canWrite(voltamp, sendId2, '2'));                                                                                                          // charger 2
    // Serial.println(canWrite(voltamp, sendId3, '3'));
  }
  Serial.println();  //Print a blank line
}

/************************************************
** Function name:           Can Read ZERO
** Descriptions:            Only Read on Zero can bus ( SOC Voltage etc.)
*************************************************/
void zeroCanRead() {

  if (CAN_MSGAVAIL == CANZ.checkReceive()) {    //Check for messages
    CANZ.readMsgBuf(&len, buf);                 // read data, len: data length, buf: data buffer
    unsigned long int canId = CANZ.getCanId();  //Reading CAN-ID from zero

    if (canId == 0x0188) {
      BatterySOC = buf[0];
      // ?? byte 1 (always 0)
      // ?? byte 2 (always 66, but 3 while charging in another session)
      ChargeCycles = buf[3] + 256 * buf[4];  // byte 3 is number of charge cycles (probably byte 4 as well (highbyte)
      CellBalance = buf[5];                  // byte 5 could be Cel Balance value
                                             // ?? byte 6 (always 0)
                                             // ?? byte 7 (always 4 maybe number of bricks ?)
    }

    if (canId == 0x0206)  // this canID only appears during charging !! charger
    {
      // ?? byte 0 (batterypack temperature ??)
      // ?? byte 1 (fixed at 2 during charging)
      Charger0Voltage = buf[2] * 0.001 + buf[3] * 0.256 + buf[4] * 65.535;  // byte 2-3-4 Charger voltage in mV (division by 1000 gets Volts)
                                                                            // ?? byte 5 (fixed at 1)
                                                                            // ?? byte 6 (fixed at 0)
                                                                            // ?? byte 7 (fixed at 0)
    }

    // if (canId == 0x0240) {
    //   BatterySOC = buf[6];  // byte 6 is State Of Charge on dash in %
    //                         // no Byte 7
    // }

    if (canId == 0x0408) {
      // byte 0
      BatteryTemp = buf[1];  // byte 1 is battery temp in celcius
      // byte 2 (same as byte 1) could be second temp sensor ?? needs change in battery temp)
      BatteryAmps = buf[3] + 256 * buf[4];  // byte 3 battery amps, both when charging and
                                            // byte 5 (fixed at 131)
                                            // byte 6 (fixed at 0)
                                            // byte 7 (fixed at 255)
    }
    if (canId == 0x0388) {
      // ?? byte 0 (very random ?? 0-25 when speed = 0 , goes higher/lower when powering or regen)
      // ?? byte 1 (very reandom ?? 246-248 when speed = 0, goes higher/lower when powering or regen)
      // ?? byte 2 (always 15)
      BatteryVoltage = 0.001 * buf[3] + 0.256 * buf[4] + 65.535 * buf[5];  // byte 3, 4 and 5 are Vbatt in units of mV (divide by 1000 to get Volts)
    }
  }
}


/************************************************
** Function name:           socToMaxVoltage
** Descriptions:            convert SOC to voltage
*************************************************/
int socToMaxVoltage(int t_targetSOC) {

  if (t_targetSOC == 30) {
    outputvoltage = 999;
  }
  if (t_targetSOC == 40) {
    return 1009;
  }
  if (t_targetSOC == 50) {
    return 1025;
  }
  if (t_targetSOC == 60) {
    return 1046;
  }
  if (t_targetSOC == 70) {
    return 1070;
  }
  if (t_targetSOC == 75) {
    return 1084;
  }
  if (t_targetSOC == 80) {
    return 1097;
  }
  if (t_targetSOC == 85) {
    return 1111;
  }
  if (t_targetSOC == 90) {
    return 1126;
  }
  if (t_targetSOC == 95) {
    return 1136;
  }
  if (t_targetSOC == 100) {
    return 1162;
  }
}

/************************************************
** Function name:           currentSum
** Descriptions:            sum current of all charger
*************************************************/
float currentSum() {
  float t_current1 = (((float)buf1[2] * 256.0) + ((float)buf1[3])) / 10.0;  //highByte/lowByte + offset
  // float t_current2 = (((float)buf2[2] * 256.0) + ((float)buf2[3])) / 10.0;  //highByte/lowByte + offset
  // float t_current3 = (((float)buf3[2] * 256.0) + ((float)buf3[3])) / 10.0;  //highByte/lowByte + offset
  // messure originalChargerCurrent or all Charger current change this cacul
  // TODO etraire avce can ZERO
  float t_current0 = 0;
  //float t_current = t_current0 + t_current1 + t_current2 + t_current3;
  float t_current = t_current0 + t_current1;
  return t_current;
}

/************************************************
** Function name:           voltage
** Descriptions:            Voltage max of all charger
*************************************************/
float voltage() {
  float t_voltage1 = (((float)buf1[0] * 256.0) + ((float)buf1[1])) / 10.0;  //highByte/lowByte + offset
  // float t_voltage2 = (((float)buf2[0] * 256.0) + ((float)buf2[1])) / 10.0;  //highByte/lowByte + offset
  // float t_voltage3 = (((float)buf3[0] * 256.0) + ((float)buf3[1])) / 10.0;  //highByte/lowByte + offset

  // mesure Charge0 read CAN CALTEX
  float t_voltage0 = Charger0Voltage;

  float t_voltage = max(t_voltage0, t_voltage1);
  //float t2_voltage = max(t_voltage2, t_voltage3);
  //float t_voltage = max(t1_voltage, t2_voltage);
  return t_voltage;
}

/************************************************
** Function name:           calculateEnergy
** Descriptions:            Estimate Energy comming to battery
*************************************************/
float calculateEnergy(float t_chargeEnergyInput) {
  float t_voltage = voltage();
  float t_current = currentSum();
  float r_chargeEnergyInput = t_chargeEnergyInput + t_current * t_voltage / 1000 / 3600;
  return r_chargeEnergyInput;
}
