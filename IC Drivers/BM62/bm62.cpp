/*
 * bm62.cpp
 *
 *  Created on: 01-May-2021
 *      Author: aninda
 */

#include "bm62.h"

#define BM62_MMI_COMMAND_DELAY 100
#define BM62_UART_COMMAND_DELAY 100

int ackNumber = 0;
int eventNumber = 0;

uint32_t bm62dataerror = 0;
extern uint8_t askedfor_bm62_silentreset;

BM62::BM62() {
	clearFlags();
	batteryMaxVoltage = 4.3;
}

/***
 *      /$$$$$$   /$$                 /$$     /$$                 /$$$$$$$$
 *     /$$__  $$ | $$                | $$    |__/                | $$_____/
 *    | $$  \__//$$$$$$    /$$$$$$  /$$$$$$   /$$  /$$$$$$$      | $$    /$$   /$$ /$$$$$$$   /$$$$$$$
 *    |  $$$$$$|_  $$_/   |____  $$|_  $$_/  | $$ /$$_____/      | $$$$$| $$  | $$| $$__  $$ /$$_____/
 *     \____  $$ | $$      /$$$$$$$  | $$    | $$| $$            | $$__/| $$  | $$| $$  \ $$| $$
 *     /$$  \ $$ | $$ /$$ /$$__  $$  | $$ /$$| $$| $$            | $$   | $$  | $$| $$  | $$| $$
 *    |  $$$$$$/ |  $$$$/|  $$$$$$$  |  $$$$/| $$|  $$$$$$$      | $$   |  $$$$$$/| $$  | $$|  $$$$$$$
 *     \______/   \___/   \_______/   \___/  |__/ \_______/      |__/    \______/ |__/  |__/ \_______/
 */

uint8_t BM62::calculateChecksum(uint8_t *data, uint16_t len) {
	uint8_t checksum = 0;
	for (uint16_t i = 0; i < len; i++) {
		checksum += *data;
		data++;
	}
	checksum = ~checksum + 1;
	return checksum;
}

void BM62::sendSPP(uint8_t *data, uint16_t len) {
	uint8_t i;
	uint16_t totallen = len; // Since we are incorporating 2 more bytes to show the start and the end of transmission
	uint8_t BM62_SPP_Buffer[250] = { 0 }; //Temporary buffer used to send the data
	BM62_SPP_Buffer[0] = SYNC_BYTE;
	BM62_SPP_Buffer[1] = (uint8_t) (totallen >> 8); //length lsb
	BM62_SPP_Buffer[2] = (uint8_t) (totallen + 7);  //length msb
	BM62_SPP_Buffer[3] = SEND_SPP_DATA;			   //op code
	BM62_SPP_Buffer[4] = DATABASE_INDEX;		   //database
	BM62_SPP_Buffer[5] = SINGLE_PACKET;			   //type
	BM62_SPP_Buffer[6] = (uint8_t) (totallen >> 8); //total
	BM62_SPP_Buffer[7] = (uint8_t) (totallen);	   //length
	BM62_SPP_Buffer[8] = (uint8_t) (totallen >> 8); //payload
	BM62_SPP_Buffer[9] = (uint8_t) (totallen);	   //length
	for (i = 10; i < len + 10; i++) {
		BM62_SPP_Buffer[i] = data[i - 10];
	}
	BM62_SPP_Buffer[i] = calculateChecksum(&BM62_SPP_Buffer[1], len + 10);
	USART3_Write(BM62_SPP_Buffer, len + 12); //Send the buffer using an UART channel
	virtualDelay();
}

void BM62::sendUARTCMD(uint8_t *param, uint16_t len) {
	uint16_t r = 0;
	uint8_t uartbuf[20] = { 0 };
	uartbuf[0] = SYNC_BYTE;
	uartbuf[1] = (uint8_t) (len >> 8); //Split the 16 bit length into 2 bytes
	uartbuf[2] = (uint8_t) (len);
	for (r = 3; r < len + 3; r++) {
		uartbuf[r] = param[r - 3];
	}
	uartbuf[r] = calculateChecksum(&uartbuf[1], len + 2); //Take into consideration of the Opcode and the Database index bytes
	USART3_Write(uartbuf, len + 4); //Send the buffer using an UART channel
	virtualDelay();
}

void BM62::sendMMI(uint8_t param, uint8_t len) {
	// MMI Action will take only 7 Bytes in total.
	uint8_t mmibuf[7];
	mmibuf[0] = SYNC_BYTE;
	mmibuf[1] = (uint8_t) (len >> 8); //Split the 16 bit length into 2 bytes
	mmibuf[2] = (uint8_t) (len + 2);
	mmibuf[3] = MMI_ACTION;
	mmibuf[4] = DATABASE_INDEX;
	mmibuf[5] = param;
	mmibuf[6] = calculateChecksum(&mmibuf[1], len + 4); //Take into consideration of the Opcode and the Database index bytes
	USART3_Write(mmibuf, 7); //Send the buffer using an UART channel
	virtualDelay();
}

void BM62::decipherVoltage(uint8_t voltage_status) {
	switch (voltage_status) {
	case 0x00:
		batteryVoltage = 3.1;
		break;
	case 0x01:
		batteryVoltage = 3.2;
		break;
	case 0x02:
		batteryVoltage = 3.3;
		break;
	case 0x03:
		batteryVoltage = 3.4;
		break;
	case 0x04:
		batteryVoltage = 3.5;
		break;
	case 0x05:
		batteryVoltage = 3.6;
		break;
	case 0x06:
		batteryVoltage = 3.7;
		break;
	case 0x07:
		batteryVoltage = 3.8;
		break;
	case 0x08:
		batteryVoltage = 3.9;
		break;
	case 0x09:
		batteryVoltage = 4.0;
		break;
	case 0x0A:
		batteryVoltage = 4.1;
		break;
	case 0x0B:
		batteryVoltage = 4.2;
		break;
	case 0x0C:
		batteryVoltage = 4.3;
		break;
	}
	batteryPercentage = (batteryVoltage / batteryMaxVoltage) * 100;
}

/***
 *     /$$$$$$$  /$$      /$$  /$$$$$$   /$$$$$$        /$$$$$$$$                              /$$
 *    | $$__  $$| $$$    /$$$ /$$__  $$ /$$__  $$      | $$_____/                             | $$
 *    | $$  \ $$| $$$$  /$$$$| $$  \__/|__/  \ $$      | $$    /$$    /$$ /$$$$$$  /$$$$$$$  /$$$$$$   /$$$$$$$
 *    | $$$$$$$ | $$ $$/$$ $$| $$$$$$$   /$$$$$$/      | $$$$$|  $$  /$$//$$__  $$| $$__  $$|_  $$_/  /$$_____/
 *    | $$__  $$| $$  $$$| $$| $$__  $$ /$$____/       | $$__/ \  $$/$$/| $$$$$$$$| $$  \ $$  | $$   |  $$$$$$
 *    | $$  \ $$| $$\  $ | $$| $$  \ $$| $$            | $$     \  $$$/ | $$_____/| $$  | $$  | $$ /$$\____  $$
 *    | $$$$$$$/| $$ \/  | $$|  $$$$$$/| $$$$$$$$      | $$$$$$$$\  $/  |  $$$$$$$| $$  | $$  |  $$$$//$$$$$$$/
 *    |_______/ |__/     |__/ \______/ |________/      |________/ \_/    \_______/|__/  |__/   \___/ |_______/
 */

void BM62::decodeEventsPacket(uint8_t *databuf) {
	uint16_t pointer = 0;
	if (databuf[pointer] == 0x00 && databuf[pointer + 1] == SYNC_BYTE) {
		uint16_t msg_length = ((uint16_t) databuf[pointer + 2] << 2)
				+ (uint16_t) databuf[pointer + 3];
		uint8_t event_opcode = databuf[pointer + 4];
		BM62_DEBUG("Valid Event Packet, msgLen[%d], opcode[0x%02X]\n",
				msg_length, event_opcode);

		if(event_opcode!=COMMAND_ACK) {
			eventAcknowledge(event_opcode);
		}

		switch (event_opcode) {
		case COMMAND_ACK: {
			uint8_t command_id = databuf[pointer + 5];
			uint8_t command_ack_status = databuf[pointer + 6];
			BM62_DEBUG("Command ACK [0x%02X, 0x%02X]\n", command_id, command_ack_status);
			break;
		}
		// Handling BTM Events
		case BTM_STATUS: {
			uint8_t params = databuf[pointer + 5];
			switch (params) {
			case SPP_LINK_CONNECTED: {
				SPPConn = true;
				flagWrite(BYTE_1, SPP_CONN_BIT, SET);
				break;
			}
			case SPP_LINK_DISCONNECTED: {
				SPPConn = false;
				flagWrite(BYTE_1, SPP_CONN_BIT, CLEAR);
				break;
			}
			case HF_LINK_ESTABLISHED: {
				HFConn = true;
				flagWrite(BYTE_1, HFP_CONN_BIT, SET);
				break;
			}
			case HF_LINK_DISCONNECTED: {
				HFConn = false;
				flagWrite(BYTE_1, HFP_CONN_BIT, CLEAR);
				break;
			}
			case A2DP_LINK_ESTABLISHED: {
				A2DPConn = true;
				flagWrite(BYTE_1, A2DP_CONN_BIT, SET);
				break;
			}
			case A2DP_LINK_DISCONNECTED: {
				A2DPConn = false;
				flagWrite(BYTE_1, A2DP_CONN_BIT, CLEAR);
				break;
			}
			case AVRCP_LINK_CONNECTED: {
				AVRCPConn = true;
				flagWrite(BYTE_1, AVRCP_CONN_BIT, SET);
				break;
			}
			case AVRCP_LINK_DISCONNECTED: {
				AVRCPConn = false;
				flagWrite(BYTE_1, AVRCP_CONN_BIT, CLEAR);
				break;
			}
			}
			break;
		}
			// Handling Call Events
		case CALL_STATUS: {
			uint8_t ch_index = databuf[pointer + 5];
			uint8_t params = databuf[pointer + 6];
			switch (params) {
			case IDLE: {
				CallState = IDLE_CALL;
				flagWrite(BYTE_1, INCOMING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, OUTGOING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, ACTIVE_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, IDLE_CALL_BIT, SET);
				SYSTEM_DEBUG("Idle Call\n");
				break;
			}
			case VOICE_DIAL: {
				SYSTEM_DEBUG("Voice Dial\n");
				break;
			}
			case CALL_INCOMING: {
				if(CallState!=INCOMING_CALL)
					generateTones(VP_INCOMING_CALL);
				CallState = INCOMING_CALL;
				flagWrite(BYTE_1, INCOMING_CALL_BIT, SET);
				flagWrite(BYTE_1, OUTGOING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, ACTIVE_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, IDLE_CALL_BIT, CLEAR);
				SYSTEM_DEBUG("Incoming Call using Call Status\n");
				break;
			}
			case CALL_OUTGOING: {
				CallState = OUTGOING_CALL;
				flagWrite(BYTE_1, INCOMING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, OUTGOING_CALL_BIT, SET);
				flagWrite(BYTE_1, ACTIVE_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, IDLE_CALL_BIT, CLEAR);
				SYSTEM_DEBUG("Outgoing Call\n");
				break;
			}
			case CALL_ACTIVE: {
				CallState = ACTIVE_CALL;
				flagWrite(BYTE_1, INCOMING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, OUTGOING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, ACTIVE_CALL_BIT, SET);
				flagWrite(BYTE_1, IDLE_CALL_BIT, CLEAR);
				SYSTEM_DEBUG("Active Call\n");
				break;
			}
			case CALL_ACTIVE_WAITING: {
				CallState = ACTIVE_CALL;
				flagWrite(BYTE_1, INCOMING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, OUTGOING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, ACTIVE_CALL_BIT, SET);
				flagWrite(BYTE_1, IDLE_CALL_BIT, CLEAR);
				SYSTEM_DEBUG("Idle Call Waiting\n");
				break;
			}
			case CALL_ACTIVE_HOLD: {
				CallState = ACTIVE_CALL;
				flagWrite(BYTE_1, INCOMING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, OUTGOING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, ACTIVE_CALL_BIT, SET);
				flagWrite(BYTE_1, IDLE_CALL_BIT, CLEAR);
				SYSTEM_DEBUG("Active Call on Hold\n");
				break;
			}
			}
			UNUSED(ch_index);
			break;
		}
		case CALLER_ID: {
			uint8_t ch_index = databuf[pointer + 5];
			uint8_t caller_id_byte0 = databuf[pointer + 6];
			if(caller_id_byte0){
				if(CallState!=INCOMING_CALL)
					generateTones(VP_INCOMING_CALL);
				CallState = INCOMING_CALL;
				flagWrite(BYTE_1, INCOMING_CALL_BIT, SET);
				flagWrite(BYTE_1, OUTGOING_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, ACTIVE_CALL_BIT, CLEAR);
				flagWrite(BYTE_1, IDLE_CALL_BIT, CLEAR);
				SYSTEM_DEBUG("Incoming Call using Caller ID\n");
			}
			UNUSED(ch_index);
			break;
		}
		case MISSED_CALL: {
			uint8_t ch_index = databuf[pointer + 5];
			CallState = IDLE_CALL;
			flagWrite(BYTE_1, INCOMING_CALL_BIT, CLEAR);
			flagWrite(BYTE_1, OUTGOING_CALL_BIT, CLEAR);
			flagWrite(BYTE_1, ACTIVE_CALL_BIT, CLEAR);
			flagWrite(BYTE_1, IDLE_CALL_BIT, SET);
			SYSTEM_DEBUG("Missed Call\n");
			UNUSED(ch_index);
			break;
		}
		// Handling SPP Data Events
		case SPP_DATA_EVENT: {
			/**
			 * SPP Events data format
			 * 0xAA
			 * 0x22
			 * <channel index>
			 * <type>
			 * <total payload length>
			 * <payload length in this packet>
			 * <payload>
			 */
			uint8_t ch_index = databuf[pointer + 5];
			uint8_t type = databuf[pointer + 6];
			uint16_t payload_len = ((uint16_t) databuf[pointer + 7] << 8)
					+ (uint16_t) databuf[pointer + 8];
			uint16_t payload_inpacket_len = ((uint16_t) databuf[pointer + 9]
					<< 8) + (uint16_t) databuf[pointer + 10];
			uint8_t params = databuf[pointer + 11];
			BM62_DEBUG("SPP_DATA EVENT [0x%02X]\n", params);
			switch (params) {
			case CODE_BASED_INVOCATION: {
				uint8_t sppCommand = databuf[pointer + 12];
				switch(sppCommand) {
				case GOOGLE_ASSISTANT_CONFIG: {
					uint8_t state = databuf[pointer + 13];
					flagWrite(BYTE_3, GOOGLE_ASSISTANT_BIT, state);
					BM62_DEBUG("Updating Google Assistant [%d]\n", state);
					break;
				}
				case REDIAL_CONFIG: {
					uint8_t state = databuf[pointer + 13];
					flagWrite(BYTE_3, REDIAL_BIT, state);
					BM62_DEBUG("Updating Redial [%d]\n", state);
					break;
				}
				}
				break;
			}
			}
			UNUSED(ch_index);
			UNUSED(type);
			UNUSED(payload_len);
			UNUSED(payload_inpacket_len);
			break;
		}
		case BTM_BATTERY_STATUS: {
			uint8_t voltage_status = databuf[pointer + 6];
			decipherVoltage(voltage_status);
			break;
		}
		case READ_LINK_STATUS_REPLY: {
			uint8_t link_profile_state = databuf[pointer + 5];
			uint8_t database0_state = databuf[pointer + 6];
			if (link_profile_state > 0x02) {
				//Checking A2DP state
				if (bitRead(&database0_state, 0)
						&& bitRead(&database0_state, 1)) {
					A2DPConn = true;
					flagWrite(BYTE_1, A2DP_CONN_BIT, SET);
				} else {
					A2DPConn = false;
					flagWrite(BYTE_1, A2DP_CONN_BIT, CLEAR);
				}

				//Checking AVRCP state
				if (bitRead(&database0_state, 2)) {
					AVRCPConn = true;
					flagWrite(BYTE_1, AVRCP_CONN_BIT, SET);
				} else {
					AVRCPConn = false;
					flagWrite(BYTE_1, AVRCP_CONN_BIT, CLEAR);
				}

				//Checking HF state
				if (bitRead(&database0_state, 3)) {
					HFConn = true;
					flagWrite(BYTE_1, HFP_CONN_BIT, SET);
				} else {
					HFConn = false;
					flagWrite(BYTE_1, HFP_CONN_BIT, CLEAR);
				}

				//Checking SPP state
				if (bitRead(&database0_state, 4)) {
					SPPConn = true;
					flagWrite(BYTE_1, SPP_CONN_BIT, SET);
				} else {
					SPPConn = false;
					flagWrite(BYTE_1, SPP_CONN_BIT, CLEAR);
				}
			} else {
				A2DPConn = false;
				AVRCPConn = false;
				HFConn = false;
				SPPConn = false;
				flagWrite(BYTE_1, A2DP_CONN_BIT, CLEAR);
				flagWrite(BYTE_1, SPP_CONN_BIT, CLEAR);
				flagWrite(BYTE_1, HFP_CONN_BIT, CLEAR);
				flagWrite(BYTE_1, AVRCP_CONN_BIT, CLEAR);
			}

			SYSTEM_DEBUG(
					"Link-Profile[0x%02X], database0[0x%02X], BM62[%d, %d, %d, %d]\n",
					link_profile_state, database0_state, A2DPConn,
					AVRCPConn, HFConn, SPPConn);
			break;
		}
		}
	}
}

/***
 *      /$$$$$$            /$$           /$$             /$$$$$$$$
 *     /$$__  $$          |__/          | $$            | $$_____/
 *    | $$  \ $$ /$$   /$$ /$$  /$$$$$$$| $$   /$$      | $$    /$$   /$$ /$$$$$$$   /$$$$$$$
 *    | $$  | $$| $$  | $$| $$ /$$_____/| $$  /$$/      | $$$$$| $$  | $$| $$__  $$ /$$_____/
 *    | $$  | $$| $$  | $$| $$| $$      | $$$$$$/       | $$__/| $$  | $$| $$  \ $$| $$
 *    | $$/$$ $$| $$  | $$| $$| $$      | $$_  $$       | $$   | $$  | $$| $$  | $$| $$
 *    |  $$$$$$/|  $$$$$$/| $$|  $$$$$$$| $$ \  $$      | $$   |  $$$$$$/| $$  | $$|  $$$$$$$
 *     \____ $$$ \______/ |__/ \_______/|__/  \__/      |__/    \______/ |__/  |__/ \_______/
 *          \__/
 */

void BM62::clearFlags(void) {
	CallState = IDLE_CALL;
	flagWrite(BYTE_1, INCOMING_CALL_BIT, CLEAR);
	flagWrite(BYTE_1, OUTGOING_CALL_BIT, CLEAR);
	flagWrite(BYTE_1, ACTIVE_CALL_BIT, CLEAR);
	flagWrite(BYTE_1, IDLE_CALL_BIT, SET);
	A2DPConn = false;
	SPPConn = false;
	AVRCPConn = false;
	HFConn = false;
	flagWrite(BYTE_1, A2DP_CONN_BIT, CLEAR);
	flagWrite(BYTE_1, SPP_CONN_BIT, CLEAR);
	flagWrite(BYTE_1, HFP_CONN_BIT, CLEAR);
	flagWrite(BYTE_1, AVRCP_CONN_BIT, CLEAR);
}

void BM62::powerOn(void) {
	/**
	 * BM62 Boot Process - Step 2
	 * Power on requires
	 *  - power on button press mmi(0x51)
	 *  - power on button release mmi(0x52)
	 */
	sendMMI(POWER_ON_BUTTON_PRESS, 1);
	sendMMI(POWER_ON_BUTTON_RELEASE, 1);
}

void BM62::powerOff(void) {
	//BM62 Boot Process - Step 1 - Switch off AT command
	sendMMI(POWER_OFF_BUTTON_PRESS, 1);
	sendMMI(POWER_OFF_BUTTON_RELEASE, 1);

	clearFlags();
}

void BM62::shutdown(void) {
	sendMMI(SWITCH_POWER_OFF, 1);
	sendMMI(NSPK_POWER_OFF_SPEAKERS, 1);
	gpioLow(BM62_RESET_GPIO_Port, BM62_RESET_Pin); //BM62 MFB is high for 500 ms to wake it up
	gpioLow(BM62_MFB_GPIO_Port, BM62_MFB_Pin);

	clearFlags();
}

void BM62::readBattVoltage(void) {
	uint8_t buf[] = { READ_BTM_BATT_CHG_STATUS, 0x00 };
	sendUARTCMD(buf, sizeof(buf));
}

void BM62::readLinkStatus(void) {
	uint8_t buf[] = { READ_LINK_STATUS, 0x00 };
	sendUARTCMD(buf, sizeof(buf));
}

void BM62::eventMask(void) {
	/**
	 * Byte 0: Current Cell Phone Battery Level | Max Cell phone Battery Level | Missed Call | SMS Received | Incoming Call | Call Status | Reserved | Reserved
	 * Byte 1: BTM DAC Gain Level | BTM Reset to Default settings OK | BTM Charging Status | BTM Battery Level | Cell Phone service status | Current CellPhone Signal Strength | Max CellPhone Signal Strength | Cell Phone Roaming
	 * Byte 2: Reserved | Reserved | Ringtone Status | Page Status | Unknown AT Command Result Code | AVC Vendor specific Response | Remote Device Friendly Name | EQ Mode
	 * Byte 3: All Reserved
	 */
	uint8_t buf[] = { EVENT_MASK_SETTING, 0b11010011, 0b11101111, 0b11111111,
			0b11111111 };
	sendUARTCMD(buf, sizeof(buf));
}

void BM62::eventAcknowledge(uint8_t eventid) {
	uint8_t buf[] = {EVENT_ACK,eventid};
	sendUARTCMD(buf, sizeof(buf));
}

void BM62::profileLinkBasedonExistingConnectionLinks(void) {
	if((A2DPConn + SPPConn + AVRCPConn + HFConn)>0 && (A2DPConn + SPPConn + AVRCPConn + HFConn)<4) {
		//Then try to connect with the existing other connections
		if(!A2DPConn) {
			uint8_t buf[] = {ADDITIONAL_PROFILES_LINK_SETUP, 0, 0x01};
			sendUARTCMD(buf, sizeof(buf));
		}
		if(!HFConn) {
			uint8_t buf[] = {ADDITIONAL_PROFILES_LINK_SETUP, 0, 0x00};
			sendUARTCMD(buf, sizeof(buf));
		}
		if(!SPPConn) {
			uint8_t buf[] = {ADDITIONAL_PROFILES_LINK_SETUP, 0, 0x02};
			sendUARTCMD(buf, sizeof(buf));
		}
	}
}

void BM62::disconnect(void) {
	// BM62 Boot Process - Step 3 - Exit Pairing mode
	sendMMI(EXIT_PAIRING_MODE, 1);
	// BM62 Boot Process - Step 4 - Disconnect all linked device
	sendMMI(DISCONNECT_ALL_LINK, 1);
}

void BM62::linkLastDevice(void) {
	sendMMI(LINK_LAST_DEVICE, 1);
}

void BM62::acceptIncomingCall(void) {
	sendMMI(ACCEPT_INCOMING_CALL, 1);
}

void BM62::rejectIncomingCall(void) {
	sendMMI(REJECT_INCOMING_CALL, 1);
}

void BM62::endOngoingCall(void) {
	sendMMI(END_CALL, 1);
}

void BM62::invokeGoogleAssistant(void) {
	sendMMI(START_VOICE_DIAL, 1);
}

void BM62::cancelGoogleAssistant(void) {
	sendMMI(CANCEL_VOICE_DIAL, 1);
}

void BM62::redial(void){
	sendMMI(LAST_NUMBER_REDIAL, 1);
}

void BM62::generateTones(uint8_t tonetype) {
	uint8_t buf[] = {BTM_UTILITY_FUNCTION, GENERATE_TONE, tonetype};
	sendUARTCMD(buf, sizeof(buf));
}

/***
 *     /$$$$$$$  /$$      /$$  /$$$$$$   /$$$$$$        /$$$$$$$                        /$$     /$$
 *    | $$__  $$| $$$    /$$$ /$$__  $$ /$$__  $$      | $$__  $$                      | $$    |__/
 *    | $$  \ $$| $$$$  /$$$$| $$  \__/|__/  \ $$      | $$  \ $$  /$$$$$$  /$$   /$$ /$$$$$$   /$$ /$$$$$$$   /$$$$$$   /$$$$$$$
 *    | $$$$$$$ | $$ $$/$$ $$| $$$$$$$   /$$$$$$/      | $$$$$$$/ /$$__  $$| $$  | $$|_  $$_/  | $$| $$__  $$ /$$__  $$ /$$_____/
 *    | $$__  $$| $$  $$$| $$| $$__  $$ /$$____/       | $$__  $$| $$  \ $$| $$  | $$  | $$    | $$| $$  \ $$| $$$$$$$$|  $$$$$$
 *    | $$  \ $$| $$\  $ | $$| $$  \ $$| $$            | $$  \ $$| $$  | $$| $$  | $$  | $$ /$$| $$| $$  | $$| $$_____/ \____  $$
 *    | $$$$$$$/| $$ \/  | $$|  $$$$$$/| $$$$$$$$      | $$  | $$|  $$$$$$/|  $$$$$$/  |  $$$$/| $$| $$  | $$|  $$$$$$$ /$$$$$$$/
 *    |_______/ |__/     |__/ \______/ |________/      |__/  |__/ \______/  \______/    \___/  |__/|__/  |__/ \_______/|_______/
 */

/**
 * BM62 initialization code for the system
 */
void BM62::init(void) {
	gpioInit(BM62_RESET_GPIO_Port, BM62_RESET_Pin);
	gpioInit(BM62_MFB_GPIO_Port, BM62_MFB_Pin);

	gpioHigh(BM62_RESET_GPIO_Port, BM62_RESET_Pin); //BM62 Reset pin is HIGH so it doesnt reset (Active Low)
	HAL_Delay(20);
	gpioHigh(BM62_MFB_GPIO_Port, BM62_MFB_Pin); //BM62 MFB is high for 500 ms to wake it up
	HAL_Delay(500);
	gpioLow(BM62_MFB_GPIO_Port, BM62_MFB_Pin); //MFB low again (MFB active high)

	powerOff(); // Start the Power off Sequence
	powerOn();	 // Start the Power on Sequence
	eventMask();
}

void BM62::reset(void) {
	gpioLow(BM62_RESET_GPIO_Port, BM62_RESET_Pin);
	gpioLow(BM62_MFB_GPIO_Port, BM62_MFB_Pin);
	HAL_Delay(100);
	gpioHigh(BM62_RESET_GPIO_Port, BM62_RESET_Pin);
	gpioHigh(BM62_MFB_GPIO_Port, BM62_MFB_Pin);
	HAL_Delay(100);
	gpioHigh(BM62_RESET_GPIO_Port, BM62_RESET_Pin); //BM62 Reset pin is HIGH so it doesnt reset (Active Low)
	HAL_Delay(20);
	gpioHigh(BM62_MFB_GPIO_Port, BM62_MFB_Pin); //BM62 MFB is high for 500 ms to wake it up
	HAL_Delay(500);
	gpioLow(BM62_MFB_GPIO_Port, BM62_MFB_Pin); //MFB low again (MFB active high)

	powerOff(); // Start the Power off Sequence
	HAL_Delay(200);
	powerOn();	 // Start the Power on Sequence
	disconnect();
	linkLastDevice();
	HAL_Delay(8000);
}

void BM62::reconnect(void) {
	profileLinkBasedonExistingConnectionLinks();
	HAL_Delay(4000);
}

/**
 * Process the bm62 events from the events buffer
 */
void BM62::processEvents(void) {
	uint16_t readPointerIdx = 0;
//	BM62_DEBUG("Items : [%d]\n", ringBuf.num_items());
	while(ringBuf.num_items() >= 5) {
		if(ringBuf.peek(readPointerIdx) == 0x00 && ringBuf.peek(readPointerIdx+1) == 0xAA) {
			uint16_t msg_length_high = ringBuf.peek(readPointerIdx+2) == -1 ? 0 : ringBuf.peek(readPointerIdx+2);
			uint16_t msg_length_low = ringBuf.peek(readPointerIdx+3) == -1 ? 0 : ringBuf.peek(readPointerIdx+3);
			uint16_t msg_length = ((uint16_t) msg_length_high << 8) + (uint16_t) msg_length_low;
			if(msg_length > 0){	//If we get atleast the message length bytes
				if(msg_length < 31) {
//					BM62_DEBUG("Msg Length [%d & %d -> %d], [%d]\n", msg_length_high, msg_length_low, msg_length, ringBuf.num_items());
					if(ringBuf.num_items() >= msg_length+5){	//If the remaining item number is greater than the required msg length
						uint8_t temp_buffer[30] = {0};
						for(int i=0; i<msg_length+5; i++){
							ringBuf.dequeue(&temp_buffer[i]);
						}
						decodeEventsPacket(temp_buffer);
					}else{
						break;
					}
				}else{
					//Since we peeked into 4 bytes dequeuing 4 bytes
					ringBuf.dequeue(NULL);
					ringBuf.dequeue(NULL);
					ringBuf.dequeue(NULL);
					ringBuf.dequeue(NULL);
					break;
				}
			}else{
				break;
			}
		}else{
			ringBuf.dequeue(NULL);
		}
		bm62dataerror = 0;
	}

	if(ringBuf.num_items() < 2) {
		bm62dataerror++;
		if(bm62dataerror > 50 && askedfor_bm62_silentreset!=true){	//Chek true if already reset is happening
			//Do a silent reset
			askedfor_bm62_silentreset = 1;
			bm62dataerror = 0;
		}
	}
}

/**
 * Pick data from the data queue and send via SPP
 */
void BM62::transmitSPP(void) {
	if (SPPConn == true) {
		uint8_t noOfPacketstoSend = 3;
		if (dataQueue.noOfElements() > noOfPacketstoSend) {
			uint8_t datacounter = 0;
			uint8_t singlePacketLength = sizeof(appSensor.bytes) + 3;
			uint8_t buffer[(singlePacketLength * noOfPacketstoSend)] = { 0 }; // Prepare buffer for 3 data packets
			while (datacounter < noOfPacketstoSend
					&& dataQueue.noOfElements() > 0) {
				dataQueue.peek(buffer + (datacounter * singlePacketLength),
						singlePacketLength - 3);
				buffer[((datacounter + 1) * singlePacketLength) - 1] = 0x5C;
				buffer[((datacounter + 1) * singlePacketLength) - 2] = 0x5C;
				buffer[((datacounter + 1) * singlePacketLength) - 3] = 0x5C;
				dataQueue.dequeue();
				datacounter++;
			}
			sendSPP(buffer, sizeof(buffer));
		}
	}
}
