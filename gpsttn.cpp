//partly taken from Arduino Library example and group work

//define headers
#include <HardwareSerial.h>
HardwareSerial loraSerial(2);
#define WIMOD_IF    loraSerial
#define WIMOD_IF_RX 23
#define WIMOD_IF_TX 05
#define PC_IF    Serial

// gps
#include "TinyGPS++.h"

HardwareSerial gpsSerial (1); 
TinyGPSPlus gps;

//wimod/LoRa join
#include <WiMODLoRaWAN.h> // make sure to use only the WiMODLoRaWAN.h, the WiMODLR_BASE.h must not be used for LoRaWAN firmware.
WiMODLoRaWAN wimod(WIMOD_IF);

// Application ID: xxxxxxxxxx
// Device EUI: ** ** ** ** ** ** ** ** //hexa decimal format
const unsigned char APPEUI[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0xB9, 0x7E };
const unsigned char APPKEY[] = { 0x9B, 0x91, 0xFA, 0xC4, 0x8F, 0xF4, 0x29, 0x3D, 0x48, 0xC0, 0x72, 0xDD, 0xB4, 0xFA, 0x25, 0xA5 };

// begin LoRaWAN  
boolean sendLora;
unsigned long lastSent = 0;
#define loraBytesSize 11	// 12 for hello world
byte loraBytes[loraBytesSize];

// Typedefs
typedef enum TModemState {
    ModemState_Disconnected = 0,
    ModemState_ConnectRequestSent,
    ModemState_Connected,
    ModemState_FailedToConnect,
} TModemState;

typedef struct TRuntimeInfo {
    TModemState ModemState;
} TRuntimeInfo;

//section RAM: Create in instance of the interface to the WiMOD-LR-Base firmware
TRuntimeInfo RIB = {  };
static TWiMODLORAWAN_TX_Data txData; //contains UINT8 TWiMODLORAWAN_TX_Data.Payload[128]

//Helperfunctions
void debugMsg(String msg) { PC_IF.print(msg); }
void debugMsg(int a) { PC_IF.print(a, DEC); }
void debugMsgChar(char c) { PC_IF.print(c); }
void debugMsgHex(int a) {
    if (a < 0x10) {
        PC_IF.print(F("0"));
    }
    PC_IF.print(a, HEX);
}

void print_lora_config() {
  TWiMODLORAWAN_RadioStackConfig radioCfgnew;  
  wimod.GetRadioStackConfig(&radioCfgnew);
  PC_IF.print("DataRateIndex: "); PC_IF.println(radioCfgnew.DataRateIndex);
  PC_IF.print("TXPowerLevel: "); PC_IF.println(radioCfgnew.TXPowerLevel);
  PC_IF.print("Options: "); PC_IF.println(radioCfgnew.Options);
  PC_IF.print("-> ADR: "); PC_IF.println(radioCfgnew.Options & LORAWAN_STK_OPTION_ADR);
  PC_IF.print("-> DUTYCYCLE: "); PC_IF.println(radioCfgnew.Options & LORAWAN_STK_OPTION_DUTY_CYCLE_CTRL);
  PC_IF.print("-> CLASSC: "); PC_IF.println(radioCfgnew.Options & LORAWAN_STK_OPTION_DEV_CLASS_C);
  PC_IF.print("-> POWERUPIND: "); PC_IF.println(radioCfgnew.Options & LORAWAN_STK_OPTION_POWER_UP_IND);
  PC_IF.print("-> PRIVATENETWORK: "); PC_IF.println(radioCfgnew.Options & LORAWAN_STK_OPTION_PRIVATE_NETOWRK);
  PC_IF.print("-> EXTPKT: "); PC_IF.println(radioCfgnew.Options & LORAWAN_STK_OPTION_EXT_PKT_FORMAT);
  PC_IF.print("-> MACCMD: ");  PC_IF.println(radioCfgnew.Options & LORAWAN_STK_OPTION_MAC_CMD);
  PC_IF.print("PowerSavingMode: "); PC_IF.println(radioCfgnew.PowerSavingMode);
  PC_IF.print("Retransmissions: "); PC_IF.println(radioCfgnew.Retransmissions);
  PC_IF.print("BandIndex: "); PC_IF.println(radioCfgnew.BandIndex);
}

void config_lora_radio() {
  
  //print_lora_config();
  TWiMODLORAWAN_RadioStackConfig radioCfg;
  wimod.GetRadioStackConfig(&radioCfg);
  TWiMODLRResultCodes res;
  UINT8 sta;
  radioCfg.DataRateIndex = LoRaWAN_DataRate_EU868_LoRa_SF7_125kHz; //see TLoRaWANDataRate
  radioCfg.TXPowerLevel = 16; //from 0 to 20

  // ADR off
  radioCfg.Options = LORAWAN_STK_OPTION_DUTY_CYCLE_CTRL;
  
  radioCfg.PowerSavingMode = 1;
  radioCfg.Retransmissions = 7; //max number of retransmissions (for C-Data) to use
  radioCfg.BandIndex = LORAWAN_BAND_EU_868_RX2_SF9; //SF9BW125 is used for RX2 by TTN; 
  
  // set new radio config
  Serial.print("Radio config successful, when 100 shown: "); Serial.print(wimod.SetRadioStackConfig(&radioCfg, &res, &sta)); Serial.print(res); Serial.println(sta);

  
  //optional: wimod.Process();
  delay(500);
  print_lora_config();
}

//join tx indication callback
void onJoinTx(TWiMODLR_HCIMessage& rxMsg) {
    TWiMODLORAWAN_TxIndData txData;
    wimod.convert(rxMsg, &txData);
    debugMsg(F("joining attempt: "));
    debugMsg((int) txData.NumTxPackets);
    debugMsg(F("\n"));
}

//joined network indication
void onJoinedNwk(TWiMODLR_HCIMessage& rxMsg) {
    TWiMODLORAWAN_RX_JoinedNwkData joinedData;

    debugMsg(F("Join-Indication received.\n"));

    if (wimod.convert(rxMsg, &joinedData)) {
        if ((LORAWAN_JOIN_NWK_IND_FORMAT_STATUS_JOIN_OK == joinedData.StatusFormat)
                || (LORAWAN_JOIN_NWK_IND_FORMAT_STATUS_JOIN_OK_CH_INFO == joinedData.StatusFormat)){
            //Ok device is now joined to nwk (server)
            RIB.ModemState = ModemState_Connected;

            debugMsg(F("Device has joined a network.\n"));
            debugMsg(F("New Device address is: "));
            debugMsg((int) joinedData.DeviceAddress);
            debugMsg(F("\n"));
        } else {
            // error joining procedure did not succeed
            RIB.ModemState = ModemState_FailedToConnect;
            debugMsg(F("Failed to join a network.\n"));
        }
    }
}

//rx data callback
void onRxData(TWiMODLR_HCIMessage& rxMsg) {
  debugMsg("Rx-Data Indication received.\n");
  TWiMODLORAWAN_RX_Data radioRxMsg;
  int i;

  // convert/copy the raw message to RX radio buffer
  if (wimod.convert(rxMsg, &radioRxMsg)) {

  if (radioRxMsg.StatusFormat & LORAWAN_FORMAT_ACK_RECEIVED) { // this is an ack
    debugMsg(F("Ack-Packet received."));
  }
      // print out the received message as hex string
      if (radioRxMsg.Length > 0) {
          // print out the length
          debugMsg(F("Rx-Message: ["));
          debugMsg(radioRxMsg.Length);
          debugMsg(F("]: "));

          // print out the payload
          for (i = 0; i < radioRxMsg.Length; i++) {
              debugMsgHex(radioRxMsg.Payload[i]);
              debugMsg(F(" "));
          }
          debugMsg(F("\n"));
      } else {  // no payload included
        debugMsg(F("\n")); 
      }
  }
}

// end LoRaWAN 

void setup()
{
// serial debug

  pinMode(BUILTIN_LED, OUTPUT);
  PC_IF.begin(115200);

// LoRaWan setup

  //LoRa
  uint8_t devEUI[8];
  WIMOD_IF.begin(WIMOD_LORAWAN_SERIAL_BAUDRATE, SERIAL_8N1, WIMOD_IF_RX, WIMOD_IF_TX);
  //rx tx
  wimod.begin(); // init the communication stack
  delay(100); 
  wimod.Reset(); 
  delay(100); // do a software reset of the WiMOD
  wimod.DeactivateDevice(); // deactivate device in order to get a clean start

  // do a simple ping to check the local serial connection
  debugMsg(F("Ping WiMOD: "));
  if (wimod.Ping() != true) {
      debugMsg(F("FAILED\n"));
  } else {
      debugMsg(F("OK. Starting join OTAA procedure...\n"));
      config_lora_radio();
  
      //setup OTAA parameters
      TWiMODLORAWAN_JoinParams joinParams;
      memcpy(joinParams.AppEUI, APPEUI, 8);
      memcpy(joinParams.AppKey, APPKEY, 16);
      wimod.SetJoinParameter(joinParams);
  
      // Register callbacks for join related events
      wimod.RegisterJoinedNwkIndicationClient(onJoinedNwk);
      wimod.RegisterJoinTxIndicationClient(onJoinTx);
      wimod.RegisterRxUDataIndicationClient(onRxData);
      
      if (wimod.JoinNetwork()) {
        RIB.ModemState = ModemState_ConnectRequestSent;
        debugMsg(F("...waiting for nwk response...\n"));
      } else {
        debugMsg("Error sending join request: ");
        debugMsg((int) wimod.GetLastResponseStatus());
        debugMsg(F("\n"));
      }
  }
    
  if (wimod.GetDeviceEUI(devEUI)) {
    debugMsg(F("\r\n"));
    debugMsg(F("LoRaWAN Info:\r\n"));
    debugMsg(F("-------------\r\n"));
    debugMsg(F("DeviceEUI:       "));
    printPayload(devEUI, 0x08);
  }
  
// gps serial init 

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(100);   
}
//#define GPS_TEST
//#define LORA_TEST
#define GPS_TTN
void loop()
{
 
  #ifdef GPS_TTN

  if( ( gps.location.age() < 1000 ) && (gps.location.lat() != (double)0.0 ) )
  {
   // double lat /long -itude , 4 Byte , in degree (+ -90 Lat ; + -180 Long )
   uint32_t latitudeBinary = ( uint32_t ) (( gps.location.lat()+ 90) * 10000000);
   loraBytes[0] = ( latitudeBinary >> 24) & 0xFF ; // shift and take out 1 byte
   loraBytes[1] = ( latitudeBinary >> 16) & 0xFF ;
   loraBytes[2] = ( latitudeBinary >> 8) & 0xFF ;
   loraBytes[3] = latitudeBinary & 0xFF ;
   uint32_t longitudeBinary = ( uint32_t ) (( gps.location.lng()+180) * 10000000);
   loraBytes[4] = ( longitudeBinary >> 24) & 0xFF ;
   loraBytes[5] = ( longitudeBinary >> 16) & 0xFF ; 
   loraBytes[6] = ( longitudeBinary >> 8) & 0xFF ;
   loraBytes[7] = longitudeBinary & 0xFF ;
   // double altitude , 4 Byte , in meters
   uint16_t altitudeBinary = ( gps.altitude.meters() < 0) ? 0 : ( uint16_t ) gps.altitude.meters(); 
   loraBytes[8] = ( altitudeBinary >> 8) & 0xFF ;
   loraBytes[9] = altitudeBinary & 0xFF ;
   // double hdop , 4 Byte , quality of position data
   uint16_t hdopBinary = ( uint16_t ) gps.hdop.value()/10;
   // HDOP in /10 m, decode with /10 to 0m to 25.5 m
   loraBytes[10] = hdopBinary & 0xFF ;

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  PC_IF.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    PC_IF.println(F("No GPS data received: check wiring"));
  
  sendLora = true;
  if(millis() - lastSent < 15000) sendLora = false; //send every 15 sec
  if(RIB.ModemState != ModemState_Connected) sendLora = false; // check of OTAA procedure has finished
  if(sendLora) digitalWrite(BUILTIN_LED, HIGH); else digitalWrite(BUILTIN_LED, LOW);

  if(sendLora) {
    debugMsg(F("Sending...\n"));
    txData.Port = 0x01;
    txData.Length = loraBytesSize;
    memcpy(txData.Payload, loraBytes, txData.Length);
  
    // try to send a message
    if (false == wimod.SendUData(&txData)) { // an error occurred
         if (LORAWAN_STATUS_CHANNEL_BLOCKED == wimod.GetLastResponseStatus()) {// we have got a duty cycle problem
             debugMsg(F("TX failed: Blocked due to DutyCycle...\n"));
         }
    } else {
      lastSent = millis();
    }
  }
  // check for any pending data of the WiMOD
  wimod.Process();
  }
  delay(30000);
  #endif
