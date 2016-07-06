/*
SymphonyLink.cpp - Arduino library for implementing a Symphony Link module.
Released into public domain.

This library is designed to control a Link Labs Symphony Link evaluation board connected to Arduino Due.

For wiring diagrams and additional information, please visit:
http://docs.link-labs.com/m/52162/l/554388-connecting-the-evaluation-board-to-arduino-due

The Arduino Due communicates with the Symphony Link evaluation board by connecting the Due's Serial1 interface to the Symphony Link module's UART interface. Use the Due's Serial interface to print debug information to the Arduino IDE's serial monitor.
*/
/*****************************************************/
/*****INCLUDES****************************************/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "SymphonyLink.h"
#include "ll_ifc_symphony.h"
#include "ll_ifc.h"
/*****************************************************/
/*****PUBLIC FUNCTIONS********************************/
SymphonyLink::SymphonyLink()
{
	// set network config variables to 0 or default
	_net_token = 0;
	memset(_app_token, 0, APP_TOKEN_LEN);
	_dl_mode = LL_DL_OFF;
	_qos = 0;

	// configure Arduino pins for Link Labs evaluation board
	pinMode(SYMPHONY_RESET_PIN, OUTPUT);
	pinMode(SYMPHONY_BOOT_PIN, OUTPUT);
	pinMode(SYMPHONY_IRQ_PIN, INPUT);

	// set nRESET and nBOOT pins low
	digitalWrite(SYMPHONY_BOOT_PIN, LOW);
	digitalWrite(SYMPHONY_RESET_PIN, LOW);
}
/*****************************************************/
int32_t SymphonyLink::setMac(ll_mac_type_t macMode)
{
	err_check_ret_t err = NO_ERROR;
	ll_mac_type_t mac_mode;
	int32_t ret = 0;

	//Query current MAC mode
	err = symphonyCheckInterfaceErrors(ll_mac_mode_get(&mac_mode));
	
	if (NO_ERROR != err)
	{
		ret = -1; // Error getting MAC mode
	}
	else
	{
		// if current MAC != desired MAC, set MAC
		if (mac_mode != macMode)
		{
			Serial.print("Setting MAC mode to ");
			Serial.println(macMode,DEC);

			err = symphonyCheckInterfaceErrors(ll_mac_mode_set(macMode));
			delay(SYMPHONY_POST_RESET_DELAY); // wait for module reset

			if (NO_ERROR != err)
			{
				ret = -2; // Error setting MAC mode
			}
			else
			{
				// MAC mode has been set. Verify new MAC mode matches intended.
				err = symphonyCheckInterfaceErrors(ll_mac_mode_get(&mac_mode));
				if (NO_ERROR != err)
				{
					ret = -3; // Error getting new MAC mode
				}
				else if (mac_mode != macMode)
				{
					// New MAC mode does not match intended MAC mode
					ret = -4;
					Serial.println("Set MAC mode does not match intended MAC mode");
				}
				else
				{
					// Success setting new MAC mode
					ret = 0;
				}
			}

		}
		else
		{
			// MAC mode already matches desired MAC mode. Don't waste time.
			Serial.print("MAC mode already set to ");
			Serial.println(macMode,DEC);
			ret = 0;
		}
	}
	
	return (ret);
}
/*****************************************************/
int32_t SymphonyLink::getMac(ll_mac_type_t* macMode)
{
	err_check_ret_t err = NO_ERROR;
	int32_t ret = 0;
	
	// Get current MAC mode	
	err = symphonyCheckInterfaceErrors(ll_mac_mode_get(macMode));
	
	if (NO_ERROR != err)
	{
		// Error getting MAC mode
		ret = -1;
	}

	return (ret);
}
/*****************************************************/
int32_t SymphonyLink::getModuleFirmware(ll_version_t* firmwareVersion)
{
	err_check_ret_t err = NO_ERROR;
	int32_t ret = 0;

	// Get module firmware type
	err = symphonyCheckInterfaceErrors(ll_version_get(firmwareVersion));

	if (NO_ERROR != err)
	{
		ret = -1;
	}

	return (ret);
}
/*****************************************************/
int32_t SymphonyLink::getAntenna(uint8_t* ant)
{
	err_check_ret_t err = NO_ERROR;
	int32_t ret = 0;

	// Get RF path. Returns 1 for U.FL, 2 for integrated trace antenna
	err = symphonyCheckInterfaceErrors(ll_antenna_get(ant));

	if (NO_ERROR != err)
	{
		ret = -1;
	}

	return (ret);
}
/*****************************************************/
int32_t SymphonyLink::setAntenna(uint8_t ant)
{
	err_check_ret_t err = NO_ERROR;
	int32_t ret = 0;

	// Set RF path. 1 for U.FL, 2 for integrated trace antenna
	err = symphonyCheckInterfaceErrors(ll_antenna_set(ant));
}
/*****************************************************/
int32_t SymphonyLink::begin(uint32_t net_token, uint8_t* app_token, enum ll_downlink_mode dl_mode, uint8_t qos)
{
	Serial.println("Starting system ...");

	// init network configuration variables
	_net_token = net_token; 			// net token local copy
	memcpy(_app_token, app_token, APP_TOKEN_LEN); 	// app token local copy
	_dl_mode = dl_mode;				// downlink mode local copy
	_qos = qos; 					// qos local copy
	
	// init state handling variables
	_err_count = 0;
	_tx_err_count = 0;
	_clear_flash_count = 0;
	_mailbox_checked = false;
	_current_state = SYMPHONY_RESET;

	printState();

	// configure Arduino Serial1 interface for UART comms with module, 115200 8-n-1
	Serial1.begin(115200);

	return (0);
}
/*****************************************************/
sym_module_state_t SymphonyLink::updateModemState(void)
{
	//Iterate the state machine
	switch (_current_state)
	{
		//Hard reset of module.
		case SYMPHONY_RESET:
			_next_state = resetState();
			break;
		//Set network config of module.
		case SYMPHONY_INITIALIZING:
			_next_state = initializeState();
			break;
		//Find compatible gateway and, if necessary, register network config.
		case SYMPHONY_SCANNING:
			_next_state = scanState();
			break;
		//Registration failed.
		//Reset the module and try again.
		//Registration only needs to succeed once; requires gateway to have internet connection.
		case SYMPHONY_NOT_REGISTERED:
			_next_state = registrationFailureState();
			break;
		//Module is ready to send and receive.
		case SYMPHONY_READY:
			break;
		//Uplink transmission in progress.
		case SYMPHONY_TRANSMITTING:
			_next_state = transmitState();
			break;
		case SYMPHONY_RECEIVING:
			_next_state = receiveState();
		default:
			break;
	}
	
	//Reset module if too many sequential errors
	if (_err_count > IFC_MAX_ERRORS || _tx_err_count > TX_MAX_ERRORS)
	{
		_next_state = SYMPHONY_RESET;
	}
	
	//Update state of module
	_last_state = _current_state;
	_current_state = _next_state;	
	
	//Print state to serial monitor if changed
	if (_current_state != _last_state)
	{
		printState();
	}

	delay(500); //prevents polling the module too frequently.

	return (_current_state);
}
/*****************************************************/
void SymphonyLink::resetModule(void)
{
	digitalWrite(SYMPHONY_RESET_PIN, HIGH); //Ground the module's nRESET pin (inverting logic on LL3B60)
	delay(100);
	digitalWrite(SYMPHONY_RESET_PIN, LOW); //Allow internal pull-up to reset module.
	delay(SYMPHONY_POST_RESET_DELAY); //Wait for module to boot. 2 seconds is sufficient.

	if (digitalRead(SYMPHONY_IRQ_PIN))
	{
		//Something is wrong. Module is potentially being held in reset. Inform user.
		Serial.println("\t... Symphony Link module is stuck in reset");
	}
	else
	{
		//Interrupt line should go low following reset (there is inverting logic on the evaluation board).
	}
}


/*****************************************************/
sym_module_state_t SymphonyLink::getState(void)
{
	return	(_current_state);
}
/*****************************************************/
sym_module_state_t SymphonyLink::write(uint8_t* msg, uint16_t msg_len, bool ack)
{
	_next_state = SYMPHONY_TRANSMITTING;
	err_check_ret_t err;

	_tx_rising_edge = false; //Allows _txState time to update
	_init_rising_edge = false; //Allows _symState time to update
	if (ack)
	{
		//Send acknowledged uplink message
		err = symphonyCheckInterfaceErrors(ll_message_send_ack(msg, msg_len));
	}
	else
	{
		//Send unacknowledged uplink message
		err = symphonyCheckInterfaceErrors(ll_message_send_unack(msg, msg_len));
	}
	
	//Update state
	_last_state = _current_state;
	_current_state = _next_state;	
	
	//Print state to serial monitor if state has changed
	if (_current_state != _last_state)
	{
		printState();
	}
	return (_current_state);	
}
/*******************************************************/
sym_module_state_t SymphonyLink::read(uint8_t* msg, uint8_t msg_len)
{
	_next_state = SYMPHONY_RECEIVING;
	err_check_ret_t err;

	_downlinkMessage = msg;
	_downlinkMessageLength = msg_len;

	switch (_dl_mode)
	{
		case LL_DL_OFF:
			_next_state = SYMPHONY_READY;
			break;
		case LL_DL_MAILBOX:
			if (_mailbox_checked)
			{
				_next_state = SYMPHONY_RECEIVING;
			}
			else
			{
				_next_state = SYMPHONY_TRANSMITTING;
				Serial.println("\t... Checking gateway's downlink mailbox ");
				err = symphonyCheckInterfaceErrors(ll_mailbox_request());
				if (NO_ERROR != err)
				{
					Serial.println("\t... Error during mailbox check ");
				}
			}
			_mailbox_checked = !_mailbox_checked;
			break;
		case LL_DL_ALWAYS_ON:
			_next_state = SYMPHONY_RECEIVING;
			break;
		default:
			break;
	}
	

	//Update state
	_last_state = _current_state;
	_current_state = _next_state;	
	
	//Print state to serial monitor if state has changed
	if (_current_state != _last_state)
	{
		printState();
	}
	return (_current_state);	
}
/*******************************************************/
void SymphonyLink::printPayload(uint8_t* msg, uint16_t msg_len)
{
	int a;

	for (a = 0; a < msg_len; a++)
	{
		Serial.print(msg[a], HEX);
	}
	Serial.println(" ");
}
/******************************************************/
void SymphonyLink::printDownlinkMessage(uint8_t* msg, uint8_t msg_len)
{
	int a;
	for (a = 0; a < msg_len; a++)
	{
		Serial.print(msg[a], HEX);
	}
	Serial.println(" ");
}
/******************************************************/
enum ll_tx_state SymphonyLink::getTransmitState(void)
{
	return (_txState);
}
/******************************************************/
enum ll_state SymphonyLink::getModuleState(void)
{
	return (_symState);
}
/******************************************************/
enum ll_rx_state SymphonyLink::getReceiveState(void)
{
	return (_rxState);
}
/*****************************************************/
/*****PRIVATE FUNCTIONS*******************************/
/*****state processing functions**********************/
sym_module_state_t SymphonyLink::resetState(void)
{
	resetModule(); //Hard reset of module

	//If too long since module has synced with gateway, clear network config from flash, re-write network config. This will cause the module to register with Conductor during next initialization.
	if (++_clear_flash_count >= IFC_CLEAR_FLASH)
	{
		Serial.println("\t... Too long since last successful sync to gateway.");
		Serial.println("\t... Re-writing module's network configuration to flash.");
		ll_settings_delete();
		_clear_flash_count = 0;
	}

	//Reset error counts
	_err_count = 0;
	_tx_err_count = 0;
	
	return(SYMPHONY_INITIALIZING);
}
/*****************************************************/
sym_module_state_t SymphonyLink::initializeState(void)
{
	err_check_ret_t err = NO_ERROR;
	ll_mac_type_t mac_mode;
	_next_state = SYMPHONY_INITIALIZING;

	//Check/set module is using Symphony Link mode
	symphonyCheckInterfaceErrors(ll_mac_mode_get(&mac_mode));
	if (mac_mode != SYMPHONY_LINK)
	{
		symphonyCheckInterfaceErrors(ll_mac_mode_set(SYMPHONY_LINK));
	}
	//Check/write network config
	else
	{
		err = symphonyCheckInterfaceErrors(ll_config_set(_net_token, _app_token, _dl_mode, _qos));
		_next_state = SYMPHONY_SCANNING;
	}

	return (_next_state);
}
/******************************************************/
sym_module_state_t SymphonyLink::scanState(void)
{
	uint8_t is_registered;
	_next_state = SYMPHONY_SCANNING;
	
	if (NO_ERROR == symphonyGetModuleStatus());
	{
		switch (_net_info.connection_status)
		{
			case LLABS_CONNECT_CONNECTED:
				_next_state = SYMPHONY_READY;
				break;
			case LLABS_CONNECT_INITIAL:
				_next_state = SYMPHONY_SCANNING;
				break;
			case LLABS_CONNECT_DISCONNECTED:
				if (_symState == LL_STATE_IDLE_DISCONNECTED)
				{
					ll_app_reg_get(&is_registered);
					if (is_registered)
					{
						_next_state = SYMPHONY_READY;
					}
					else
					{
						_next_state = SYMPHONY_NOT_REGISTERED;
					}
				}
				break;
			default:
				break;
		}
	}

	return (_next_state);
}
/******************************************************/
sym_module_state_t SymphonyLink::registrationFailureState(void)
{
	// One of the following has failed:
	// (1) Registration of the module's application token with Conductor, or
	// (2) registration of the module's downlink mode with the gateway, or
	// (3) crypto key exchange between the module and the gateway.
	//
	// In this example, we handle the failed registration by resetting the module and trying again.
	// Battery-powered applications may wish to deal with failed registration using less power-hungry methods.
	// It is okay for the module to remain in the SYMPHONY_NOT_REGISTERED state indefinitely, but will not be able to communicate until registration succeeds.
	_next_state = SYMPHONY_RESET;

	return (_next_state);
}
/******************************************************/
sym_module_state_t SymphonyLink::transmitState(void)
{
	_next_state = SYMPHONY_TRANSMITTING;
	
	if (LL_STATE_IDLE_CONNECTED == _symState)
	{
		switch (_txState)
		{
			case LL_TX_STATE_TRANSMITTING:
				_tx_rising_edge = true;
				break;
			case LL_TX_STATE_SUCCESS:
				if (true == _tx_rising_edge || true == _init_rising_edge)
				{
					_tx_err_count = 0;
					Serial.println("\t... TX Success");
					_next_state = SYMPHONY_READY;
				}
				else
				{
					//Wait for _tx_rising_edge or _init_rising_edge.
					//These booleans allow time for module to change state immediately following TX command.
				}
				break;
			case LL_TX_STATE_ERROR:
				if (true == _tx_rising_edge || true == _init_rising_edge)
				{
					_tx_err_count++;
					Serial.println("\t... TX Error ... unable to sync to gateway");
					_next_state = SYMPHONY_READY;
				}
				else
				{
					//Wait for _tx_rising_edge or _init_rising_edge.
					//This case implies _txState was LL_TX_STATE_ERROR at conclusion of last TX attempt. 
				}
				break;
			default:
				_tx_err_count++;
				Serial.println("\t... TX Error ... _txState not defined");
				_next_state = SYMPHONY_READY;
				break;
		}
	}
	else if (LL_STATE_IDLE_DISCONNECTED == _symState)
	{
		if (true == _init_rising_edge)
		{
			_tx_err_count++;
			Serial.println("\t... TX Error ... unable to sync to gateway");
			_next_state = SYMPHONY_READY;
		}
	}
	else if (LL_STATE_INITIALIZING == _symState)
	{
		_init_rising_edge = true;
	}
	else // _symState == LL_STATE_ERROR or NaN. Reset the module.
	{
		_next_state = SYMPHONY_RESET;
	}	

	symphonyGetModuleStatus();
	return (_next_state);
}
/******************************************************/
sym_module_state_t SymphonyLink::receiveState()
{

	err_check_ret_t err = NO_ERROR;
	int16_t rssi;
	uint8_t  snr;

	_next_state = SYMPHONY_RECEIVING;

	Serial.println("\t... Checking module for downloaded downlink messages ");

	if (LL_RX_STATE_NO_MSG == _rxState)
	{
		Serial.println("\t... None waiting ");
		_next_state = SYMPHONY_READY;
	}
	else if (LL_RX_STATE_RECEIVED_MSG == _rxState)
	{
		Serial.println("\t... Downlink message waiting ... ");
		
		err = symphonyCheckInterfaceErrors(ll_retrieve_message(_downlinkMessage, &_downlinkMessageLength, &rssi, &snr)); 
		if (NO_ERROR != err)
		{
		  // Do nothing.  Error counter will automatically increment and eventually reset the module.
		}
		else
		{
			Serial.print("\t... Downlink payload is ");
			printDownlinkMessage(_downlinkMessage, _downlinkMessageLength);
		}

		_next_state = SYMPHONY_READY;
	}
	else // _rxState error
	{
		_next_state = SYMPHONY_RESET;
	}

	return (_next_state);
}
/******************************************************/
err_check_ret_t SymphonyLink::symphonyCheckInterfaceErrors(int32_t ifc_ret)
{
	err_check_ret_t rst;
	if (ifc_ret < 0)
	{
		_err_count++;
		rst = ERROR_NO_RESET;
	}
	else
	{
		_err_count = 0;
		rst = NO_ERROR;
	}

	if (_err_count > IFC_MAX_ERRORS)
	{
		rst = ERROR_SO_RESET;
	}

	return (rst);
}
/******************************************************/
err_check_ret_t SymphonyLink::symphonyGetModuleStatus(void)
{
	err_check_ret_t err = NO_ERROR;
	uint32_t _irq_flags_tmp;

	// read module state & IRQ flags
	do{
		err = symphonyCheckInterfaceErrors(ll_get_state(&_symState, &_txState, &_rxState));
		if (NO_ERROR != err)
		{
			Serial.println("State error");
			break;
		}

		err = symphonyCheckInterfaceErrors(ll_irq_flags(0xFFFFFFFF, &_irq_flags_tmp));
		if (NO_ERROR != err)
		{
			Serial.println("IRQ error");
			break;
		}
		_irq_flags |= _irq_flags_tmp;

		err = symphonyCheckInterfaceErrors(ll_net_info_get(&_net_info));
		if (NO_ERROR != err)
		{
			Serial.println("net_info error");
		}
	} while (0);
	
	return (err);
}
/*******************************************************/
void SymphonyLink::printState(void)
{
	switch (_current_state)
	{
		case SYMPHONY_RESET:
			Serial.println("STATE: SYMPHONY_RESET");
			break;
		case SYMPHONY_INITIALIZING:
			Serial.println("STATE: SYMPHONY_INITIALIZING");
			break;
		case SYMPHONY_SCANNING:
			Serial.println("STATE: SYMPHONY_SCANNING");
			break;
		case SYMPHONY_NOT_REGISTERED:
			Serial.println("STATE: SYMPHONY_NOT_REGISTERED");
			break;
		case SYMPHONY_READY:
			Serial.println("STATE: SYMPHONY_READY");
			break;
		case SYMPHONY_TRANSMITTING:
			Serial.println("STATE: SYMPHONY_TRANSMITTING");
			break;
		case SYMPHONY_RECEIVING:
			Serial.println("STATE: SYMPHONY_RECEIVING");
			break;
		default:
			break;
	}
}
/*******************************************************/
void SymphonyLink::printModuleState(void)
{
	switch (_symState)
	{
		case LL_STATE_IDLE_CONNECTED:
			Serial.print("ll_state = IDLE_CONNECTED, ");
			break;
		case LL_STATE_IDLE_DISCONNECTED:
			Serial.print("ll_state = IDLE_DISCONNECTED, ");
			break;
		case LL_STATE_INITIALIZING:
			Serial.print("ll_state = INITIALIZING, ");
			break;
		case LL_STATE_ERROR:
			Serial.print("ll_state = ERROR, ");
		default:
			break;
	}
	switch (_txState)
	{
		case LL_TX_STATE_TRANSMITTING:
			Serial.print("tx_state = TRANSMITTING, ");
			break;
		case LL_TX_STATE_SUCCESS:
			Serial.print("tx_state = SUCCESS, ");
			break;
		case LL_TX_STATE_ERROR:
			Serial.print("tx_state = ERROR, ");
			break;
		default:
			break;
	}
	switch (_rxState)
	{
		case LL_RX_STATE_NO_MSG:
			Serial.println("rx_state = NO_MESSAGE");
			break;
		case LL_RX_STATE_RECEIVED_MSG:
			Serial.println("rx_state = RECEIVED_MSG");
			break;
		default:
			break;
	}
}
/*****************************************************/
/*****UART transport functions************************/
int32_t transport_write(uint8_t* buf, uint16_t len)
{
	int32_t ret;

	ret = Serial1.write(buf, len);

	if (ret < 0){
		return -1;
	}
}
/*****************************************************/
int32_t transport_read(uint8_t *buf, uint16_t len)
{
	uint8_t ret;

	uint32_t timeout_val = (500 * 1);

		Serial1.setTimeout(timeout_val);
		ret = Serial1.readBytes(buf, len);

		if (ret > 0)
		{
			return(ret);
		}
		else
		{
			return(-1);
		}
}


