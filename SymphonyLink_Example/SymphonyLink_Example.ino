#include <SymphonyLink.h>

//Instantiate a SymphonyLink class using the SymphonyLink Arduino Library
SymphonyLink symlink; 

uint8_t txData[1];    //TX data buffer. Could be up to 256 bytes.
uint8_t rxData[128];   //RX data buffer. Could be up to 128 bytes.
uint8_t rxDataLength;
uint8_t radioPath = 1; // set 1 for U.FL, 2 for integrated trace antenna

sym_module_state_t currentSymphonyState;
sym_module_state_t lastSymphonyState;

void setup() 
{
  int ii;
  
  //Arduino Due allows debug and status signals out of the Serial port. UART communications with the SymphonyLink module are done using Serial1.
  Serial.begin(115200);

  //Configure the following to match your network and application tokens
  //Set desired network token 
  uint32_t netToken = 0x18ae3821; //The OPEN network token is 0x4f50454e.
                                   //A module can only talk to gateways using the same network token.
  //Set desired application token
  uint8_t appToken[APP_TOKEN_LEN] = {0xf3,0x9e,0x7f,0x5d,0x39,0x56,0x2b,0x5e,0xdb,0xae}; //Generate new application tokens in your Conductor account.
                                                                                          //The application token identifies this dataflow in Conductor.
  
  //Initialize the SymphonyLink object and open UART communications with the module on Serial1.
  symlink.begin(netToken, appToken, LL_DL_MAILBOX, 15);

  //Initialize the txData. This is the array of hex bytes to be sent over the air, as an example.
  for (ii = 0; ii < sizeof(txData); ii++)
  {
    txData[ii] = 0;
  }

  rxDataLength = sizeof(rxData);

  //Set RF path
  symlink.setAntenna(radioPath);
  
  //Update the state of the SymphonyLink module (aka Modem)
  lastSymphonyState = symlink.updateModemState();
}

void loop()
{
  //Update the state of the SymphonyLink module (aka Modem)
  currentSymphonyState = symlink.updateModemState();
  switch (currentSymphonyState)
  {
    case SYMPHONY_READY:                            
      if (SYMPHONY_TRANSMITTING != lastSymphonyState) //When SymphonyLink module is ready, send txData
      {
        txData[0]++;                                 //Increment payload
        symlink.write(txData, sizeof(txData), true); //Uplink the payload to Conductor
      
        //Print the payload to the Serial monitor for debug.
        Serial.print("\t... Outbound payload is ");
        symlink.printPayload(txData, sizeof(txData));
      }
      else 
      {
         // If last uplink failed, do not increment payload.
         if (LL_TX_STATE_SUCCESS != symlink.getTransmitState())
         {
          txData[0]--;
         }

         //Check for downlink data
         symlink.read(rxData, rxDataLength);
      }
      break;
    default:
      break;
  }
  lastSymphonyState = currentSymphonyState;
}
