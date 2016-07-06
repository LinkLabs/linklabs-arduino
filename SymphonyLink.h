/*
SymphonyLink.h - Arduino library for implementing a Symphony Link module.
Released into public domain.

This library is designed to control a Link Labs Symphony Link evaluation board connected to Arduino Due.

For wiring diagrams and additional information, please visit:
http://docs.link-labs.com/m/52162/l/554388-connecting-the-evaluation-board-to-arduino-due

The Arduino Due communicates with the Symphony Link evaluation board by connecting the Due's Serial1 interface to the Symhony Link module's UART interface. Use the Due's Serial interface to print debug information to the Arduino IDE's serial monitor.
*/
#ifndef SYMPHONYLINK_H
#define SYMPHONYLINK_H
/**************************************************************************/
/*****INCLUDES*************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Arduino.h"
#include "ll_ifc_consts.h"
#include "ll_ifc_symphony.h"
#include "ll_ifc.h"
/**************************************************************************/
/*****Defines**************************************************************/
#define IFC_MAX_ERRORS			3	// max number of sequential errors with ll_ifc before resetting module
#define TX_MAX_ERRORS			3	// max number of sequential tx errors before resetting module
#define IFC_CLEAR_FLASH			3	// max number of resets before clearing module's flash variables
#define SYMPHONY_POST_RESET_DELAY		3000	// ms delay after resetting module before using UART
// Arduino pin config
#define SYMPHONY_RESET_PIN	    	7	// Arduino pin for nRESET of module. Use '2' for Rev 4 and 5 eval boards. Use '7' for Rev 6 and later.
#define	SYMPHONY_BOOT_PIN		    	8	// Arduino pin for nBOOT of module
#define SYMPHONY_IRQ_PIN 			13	// Arduino pin to read module's host interrupt line
/**************************************************************************/
/*****Types****************************************************************/
// Symphony Link module states
typedef enum
{
	SYMPHONY_RESET = 0,
	SYMPHONY_INITIALIZING ,
	SYMPHONY_SCANNING ,
	SYMPHONY_NOT_REGISTERED ,
	SYMPHONY_READY ,
	SYMPHONY_TRANSMITTING ,
	SYMPHONY_RECEIVING
} sym_module_state_t;
// Error states
typedef enum
{
	NO_ERROR = 0,
	ERROR_NO_RESET ,
	ERROR_SO_RESET
} err_check_ret_t;
/**************************************************************************/
/*****Classes**************************************************************/
// SymphonyLink class
class SymphonyLink
{
	public:

		SymphonyLink();
		/********** high level functions ******************/
		int32_t begin(uint32_t net_token, uint8_t* app_token, enum ll_downlink_mode dl_mode, uint8_t qos);
		sym_module_state_t getState(void);
		sym_module_state_t write(uint8_t* msg, uint16_t msg_len, bool ack);
		sym_module_state_t read(uint8_t* msg, uint8_t msg_len);
		void resetModule(void);
		void printPayload(uint8_t* msg, uint16_t msg_len);
		void printDownlinkMessage(uint8_t* msg, uint8_t msg_len);
		sym_module_state_t updateModemState(void);
		err_check_ret_t symphonyGetModuleStatus(void);
		
		/********** some ll_ifc wrapper functions **************/
		int32_t getMac(ll_mac_type_t* macMode);
		int32_t setMac(ll_mac_type_t macMode);
		int32_t getModuleFirmware(ll_version_t* firmwareVersion);
		int32_t getAntenna(uint8_t* ant);
		int32_t setAntenna(uint8_t ant);
		enum ll_state getModuleState(void);
		enum ll_tx_state getTransmitState(void);
		enum ll_rx_state getReceiveState(void);

	private:

		//state processing functions
		sym_module_state_t resetState(void);
		sym_module_state_t initializeState(void);
		sym_module_state_t scanState(void);
		sym_module_state_t registrationFailureState(void);
		sym_module_state_t transmitState(void);
		sym_module_state_t receiveState(void);
		
		//utility functions
		err_check_ret_t symphonyCheckInterfaceErrors(int32_t ifc_ret);
		void printState(void);
		void printModuleState(void);

		/*****symphony variables**************************/
		//network config
		uint32_t 		_net_token;
		uint8_t			_app_token[APP_TOKEN_LEN];
		enum ll_downlink_mode 	_dl_mode;
		ll_mac_type_t		_macMode;
		uint8_t			_qos;
		//variables of state
		uint32_t 		_err_count;
		uint32_t		_tx_err_count;
		uint32_t 		_clear_flash_count;
		boolean			_tx_rising_edge;
		boolean			_init_rising_edge;
		boolean			_mailbox_checked;
		sym_module_state_t	_last_state;
		sym_module_state_t 	_current_state;
		sym_module_state_t 	_next_state;
		uint32_t 		_irq_flags;
		llabs_network_info_t 	_net_info;
		enum ll_state 		_symState;
		enum ll_tx_state 	_txState;
		enum ll_rx_state 	_rxState;
		uint8_t*  _downlinkMessage;
		uint8_t   _downlinkMessageLength;
};

#endif // SYMPHONYLINK_H
