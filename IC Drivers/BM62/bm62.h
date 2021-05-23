/*
 * bm62.h
 *
 *  Created on: 01-May-2021
 *      Author: aninda
 */

#ifndef INC_BM62_H_
#define INC_BM62_H_

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "queue.h"
#include "ringBuffer.h"
#include "amplifier.h"

extern Application application;
extern appSensor_Type appSensor;

extern RINGBUFFER ringBuf;

//extern QUEUE eventsQueue;
extern QUEUE dataQueue;

/**
 * BM62 definitions
 */
#define SYNC_BYTE (0xAA)
#define DATABASE_INDEX (0x00) // BM62 can't connect to 2 device at the same time. Since we are primarily concerned with 1 device hence 0
#define END_OF_SEQUENCE (0x5C) // End of text transmission via SPP

#define TIMESTAMP_SYNC						(0x2E)		// Syncing Timestamp
#define AT_COMMAND_FROM_PHONE				(0x2F)		// AT Commands being sent
#define CODE_BASED_INVOCATION				(0x30)		// it is currently in Don't care condition - Code Based Invocation
#define CONFIG_PUSH							(0x31)		// For pushing configuration

/**
 * @def Config settings
 * @brief
 *
 */
#define GOOGLE_ASSISTANT_CONFIG				(63)
#define REDIAL_CONFIG						(62)


/**
 * Call Definitions
 */
#define IDLE_CALL							(0x00)
#define OUTGOING_CALL						(0x01)
#define INCOMING_CALL						(0x02)
#define ACTIVE_CALL							(0x03)

/**
 * UART command sets to be used.
 */
#define MAKE_CALL 									0x00
#define MAKE_EXT_CALL 								0x01
#define MMI_ACTION 									0x02		//Frequently used for sending out MMI commands
#define EVENT_MASK_SETTING 							0x03
#define CHANGE_DEVICE_NAME							0x05
#define CHANGE_PIN_CODE								0x06
#define BTM_PARAMETER_SETTING						0x07
#define READ_BTM_VERSION							0x08
#define GET_PB_BY_AT_CMD							0x09
#define VENDOR_AT_CMD								0x0A
#define AVC_VENDOR_DEPENDENT_CMD					0x0B
#define AVC_GROUP_NAVIGATION						0x0C
#define READ_LINK_STATUS							0x0D
#define READ_PAIRED_DEVICE_RECORD 					0x0E
#define READ_LOCAL_BD_ADDR							0x0F
#define READ_LOCAL_DEVICE_NAME						0x10
#define SET_ACCESS_PB_METHOD						0x11
#define SEND_SPP_DATA								0x12
#define BTM_UTILITY_FUNCTION						0x13
#define EVENT_ACK									0x14
#define ADDITIONAL_PROFILES_LINK_SETUP				0x15
#define READ_LINKED_DEVICE_INFO						0x16
#define PROFILES_LINK_BACK							0x17
#define DISCONNECT									0x18
#define MCU_STATUS_INDICATION 						0x19
#define USER_CONFIRM_SPP_REQ_REPLY					0x1A
#define SET_HF_GAIN_LEVEL							0x1B
#define EQ_MODE_SETTING								0x1C
#define DSP_NR_CTRL									0x1D
#define GPIO_CNTRL									0x1E
#define MCU_UART_RX_BUFFER_SIZE						0x1F
#define VOICE_PROMT_CMD								0x20
#define MAP_REQUEST									0x21
#define SECURITY_BONDING_REQ						0x22
#define SET_OVERALL_GAIN							0x23
#define READ_BTM_SETTING							0x24
#define READ_BTM_BATT_CHG_STATUS					0x25
#define MCU_UPDATE_CMD								0x26
#define REPORT_BATTERY_CAPACITY						0x27
#define LE_ANCS_SERVICE_CMD							0x28
#define LE_SIGNALING_CMD							0x29
#define NSPK_VENDOR_CMD								0x2A
#define READ_NSPK_LINK_STATUS						0x2B
#define NSPK_SYNC_AUDIO_EFFECT						0x2C
#define LE_GATT_CMD									0x2D
#define LE_APP_CMD									0x2F
#define DSP_RUNTIME_PROGRAM							0x30
#define READ_VENDOR_EEPROM_DATA						0x31
#define QUERY_IC_VER_INFO							0x32
#define	VOICE_PROMT_IND_CMD							0x33
#define READ_BTM_LINK_MODE							0x34
#define CONFIGURE_VENDOR_PARAM						0x35
#define DSP_DEDICATED_CMD							0x36
#define NSPK_EXCHANGE_LINK_INFO_CMD					0x37
#define UART_CMD_NSPK_SET_GIAC						0x38
#define	READ_FEATURE_LIST							0x39
#define PERSONAL_MSPK_GROUP_CNTRL					0x3A
#define UART_CMD_TEST_DEVICE						0x3B

// MMI command sets to be used.
#define ADD_SCO_LINK								0x01
#define REMOVE_SCO_LINK								0x01
#define FORCE_END_ACTIVE_CALL						0x02
#define ACCEPT_INCOMING_CALL						0x04
#define REJECT_INCOMING_CALL						0x05
#define END_CALL									0x06
#define TRANSFER_AUDIO_TO_PHONE						0x06
#define TOGGLE_MICROPHONE							0x07
#define MUTE_MICROPHONE								0x08
#define ACTIVE_MICROPHONE							0x09
#define START_VOICE_DIAL							0x0A
#define CANCEL_VOICE_DIAL							0x0B
#define LAST_NUMBER_REDIAL							0x0C
#define SET_ACTIVE_ON_HOLD_ACTIVE_HOLD_CALL			0x0D
#define SWITCH_VOICE_BETWEEN_PHONE_HEADSET			0x0E
#define QUERY_CALL_LIST_INFORMATION					0x0F
#define THREE_WAY_CALL								0x10
#define RELEASE_WAITING_OR_HOLD_CALL				0x11
#define ACCEPT_WAITING_CALL							0x12
#define INITIATE_HF_CONN							0x16
#define DISCONNECT_HF_CONN							0x17
#define ENABLE_RX_NOICE_REDUCTION_SCO_READY			0x18
#define DISABLE_RX_NOICE_REDUCTION_SCO_READY		0x19
#define SWITCH_RX_NOICE_REDUCTION_SCO_READY			0x1A
#define ENABLE_TX_NOICE_REDUCTION_SCO_READY			0x1B
#define DISABLE_TX_NOICE_REDUCTION_SCO_READY		0x1C
#define SWITCH_TX_NOICE_REDUCTION_SCO_READY			0x1D
#define ENABLE_AEC_SCO_READY						0x1E
#define DISABLE_AEC_SCO_READY						0x1F
#define SWITCH_AEC_SCO_READY						0x20
#define ENABLE_AEC_RX_NOICE_REDUCTION_SCO_READY		0x21
#define DISABLE_AEC_RX_NOICE_REDUCTION_SCO_READY	0x22
#define SWITCH_AEC_RX_NOICE_REDUCTION_SCO_READY		0x23
#define INCREASE_MICROPHONE_GAIN					0x24
#define DECREASE_MICROPHONE_GAIN					0x25
#define SWITCH_PRIMARY_SECONDARY_HF_DEVICE			0x26
#define INCREASE_SPEAKER_GAIN						0x30
#define DECREASE_SPEAKER_GAIN						0x31
#define PLAY_PAUSE_MUSIC							0x32
#define STOP_MUSIC									0x33
#define NEXT_SONG									0x34
#define	PREVIOUS_SONG								0x35
#define FAST_FORWARD								0x36
#define REWIND										0x37
#define EQ_MODE_UP									0x38
#define EQ_MODE_DOWN								0x39
#define LOCK_BUTTON									0x3A
#define DISCONN_A2DP_LINK							0x3B
#define NEXT_AUDIO_EFFECT							0x3C
#define PREVIOUS_AUDIO_EFFECT						0x3D
#define TOGGLE_3D_EFFECT							0x3E
#define REPORT_CURR_EQ_MODE							0x3F
#define REPORT_CURR_AUDIO_EFFECT_STATUS				0x40
#define ENTER_PAIRING_MODE							0x50
#define POWER_ON_BUTTON_PRESS						0x51
#define POWER_ON_BUTTON_RELEASE						0x52
#define POWER_OFF_BUTTON_PRESS						0x53
#define POWER_OFF_BUTTON_RELEASE					0x54
#define REVERSE_PANEL								0x55
#define RESET_EEPROM_TO_DEFAULT						0x56
#define FORCE_SPEAKER_GAIN_TOGGLE					0x57
#define TOGGLE_BUTTON_INDICATION					0x58
#define COMBINE_FUNC_0						 		0x59
#define COMBINE_FUNC_1						 		0x5A
#define COMBINE_FUNC_2						 		0x5B
#define COMBINE_FUNC_3						 		0x5C
#define FAST_ENTER_PAIRING_MODE						0x5D
#define SWITCH_POWER_OFF							0x5E
#define DISABLE_LED						 			0x5F
#define TOGGLE_BUZZER						 		0x60
#define DISABLE_BUZZER						 		0x61
#define ENABLE_BUZZER						 		0x62
#define CHANGE_TONE_SET						 		0x63
#define RETRIEVE_PHONEBOOK				         	0x64
#define RETRIEVE_MCH						 		0x65
#define RETRIEVE_ICH						 		0x66
#define RETRIEVE_OCH								0x67
#define RETRIEVE_CCH						 		0x68
#define CANCEL_ACCESS_PBAP						 	0x69
#define BATTERY_STAT							 	0x6A
#define EXIT_PAIRING_MODE						 	0x6B
#define LINK_LAST_DEVICE 						 	0x6C
#define DISCONNECT_ALL_LINK						 	0x6D
#define OHS_EVENT_1						 			0x6E
#define OHS_EVENT_2						 			0x6F
#define OHS_EVENT_3						 			0x70
#define OHS_EVENT_4						 			0x71
#define SHS_SEND_USER_DATA1						 	0x72
#define SHS_SEND_USER_DATA2						 	0x73
#define SHS_SEND_USER_DATA3						 	0x74
#define SHS_SEND_USER_DATA4						 	0x75
#define SHS_SEND_USER_DATA5						 	0x76
#define CURRENT_RX_NR_STAT						 	0x77
#define CURRENT_TX_NR_STAT						 	0x78
#define FORCE_BUZZER_ALARM						 	0x79
#define CANCEL_ALL_BT_PAGING						0x7A
#define OHS_EVENT_5						 			0x7B
#define OHS_EVENT_6						 			0x7C
#define DISCONNECT_SPP_LINK						 	0x7D
#define OHS_EVENT_7						 			0xC0
#define OHS_EVENT_8						 			0xC1
#define OHS_EVENT_9						 			0xC2
#define OHS_EVENT_10						 		0xC3
#define OHS_EVENT_11						 		0xC4
#define OHS_EVENT_12						 		0xC5
#define OHS_EVENT_13						 		0xC6
#define OHS_EVENT_14						 		0xC7
#define OHS_EVENT_15						 		0xC8
#define OHS_EVENT_16						 		0xC9
#define OHS_EVENT_17						 		0xCA
#define SWITCH_DSP						 			0xCB
#define TRIGGER_NSPK_MASTER						 	0xE0
#define TRIGGER_NSPK_SLAVE						 	0xE1
#define NSPK_CONNECT_DISCONNECT						0xE2
#define CANCEL_NSPK_CREATION						0xE3
#define TERMINATE_NSPK_LINK						 	0xE4
#define TERMINATE_NSPK_CONNECTION				    0xE5
#define NSPK_MASTER_ENT_AUX_44_1K					0xE6
#define NSPK_MASTER_ENT_AUX_48K						0xE7
#define NSPK_MASTER_EXT_AUX						 	0xE8
#define NSPK_DYNAMIC_CREATION						0xEB
#define NSPK_SWITCH_CHANNEL						 	0xEC
#define NSPK_POWER_OFF_SPEAKERS						0xED
#define NSPK_AFH_SBCENCODING_AUDIOSYNC				0xEE
#define NSPK_MASTER_SLAVE_FOR_NEW_SLAVE				0xF0
#define NSPK_SLAVE_ENABLE_SCAN_FOR_NEW_MASTER		0xF1
#define NSPK_SLAVE_USE_SLOW_PAGE_SCAN				0xF2
#define NSPK_SLAVE_USE_FAST_PAGE_SCAN				0xF3
#define NSPK_ENTER_NSPK_MODE						0xF4
#define NSPK_ENTER_BROADCAST_MODE					0xF5
#define NSPK_ADD_THIRD_SPK						 	0xF6
#define NSPK_SOUND_SYNCHRONIZATION					0xF7
#define NSPK_CSB_CONNECTED_MODE_SWITCH				0xF8
#define NSPK_BACK_TO_LAST_MODE						0xF9


// Events generated by BM62.
#define COMMAND_ACK 								0x00
#define BTM_STATUS									0x01
#define CALL_STATUS									0x02
#define CALLER_ID									0x03
#define BTM_BATTERY_STATUS							0x0C
#define BTM_CHARGING_STATUS							0x0D
#define LE_ANCS_SERVICE_EVENT						0x31
#define LE_SIGNALING_EVENT							0x32
#define LE_GATT_EVENT								0x39
#define SPP_DATA_EVENT								0x22
#define READ_LINK_STATUS_REPLY						0x1E
#define MISSED_CALL									0x05

// BTM Status Extensions.
#define POWER_OFF_STATE								0x00
#define PAIRING_STATE								0x01
#define POWER_ON_STATE								0x02
#define PAIRING_SUCCESSFUL							0x03
#define PAIRING_FAILED								0x04
#define HF_LINK_ESTABLISHED							0x05
#define HF_LINK_DISCONNECTED						0x07
#define A2DP_LINK_ESTABLISHED						0x06
#define A2DP_LINK_DISCONNECTED						0x08
#define SCO_LINK_CONNECTED							0x09
#define SCO_LINK_DISCONNECTED						0x0A
#define AVRCP_LINK_CONNECTED						0x0B
#define AVRCP_LINK_DISCONNECTED						0x0C
#define SPP_LINK_CONNECTED							0x0D
#define SPP_LINK_DISCONNECTED						0x0E
#define STANDBY_STATE								0x0F
#define MAP_CONNECTED								0x12
#define MAP_DISCONNECTED							0x14
#define MAP_OPERATION_FORBIDDEN						0x13
#define ACL_DISCONNECTED							0x11
#define ACL_CONNECTED								0x15

// Call Status Extensions.
#define IDLE										0x00
#define VOICE_DIAL									0x01
#define CALL_INCOMING								0x02
#define CALL_OUTGOING 								0x03
#define CALL_ACTIVE									0x04
#define CALL_ACTIVE_WAITING							0x05
#define CALL_ACTIVE_HOLD							0x06


// LE GATT Event Extensions.
#define CLIENT_WRITE_CHAR_VAL						0x00
#define READ_LOCAL_CHAR_VAL							0x01
#define READ_LOCAL_ALL_PRIMARY_SERVICES				0x02
#define READ_LOCAL_SPECIFIC_PRIMARY_SERVICES		0x03

// SPP Packet Type
#define SINGLE_PACKET 								0x00
#define FRAGMENTED_START_PACKET						0x01
#define FRAGMENTED_CONTINUE_PACKET					0x02
#define FRAGMENTED_END_PACKET						0x03

// BTM Utility Extensions
#define GENERATE_TONE                               0x02
#define VOLUME_UP                                   0x01
#define VOLUME_DOWN                                 0x02


class BM62 {
private:
	uint8_t calculateChecksum(uint8_t *data, uint16_t len);

private:
	void sendSPP(uint8_t *data, uint16_t len);
	void sendUARTCMD(uint8_t *param, uint16_t len);
	void sendMMI(uint8_t param, uint8_t len);

private:
	void decipherVoltage(uint8_t voltage_status);
	void decodeEventsPacket(uint8_t *databuf);
	void clearFlags(void);

public:
	uint8_t A2DPConn;
	uint8_t SPPConn;
	uint8_t AVRCPConn;
	uint8_t HFConn;
	uint8_t CallState;
	float batteryVoltage;
	float batteryMaxVoltage;
	float batteryPercentage;

	BM62();
	void init(void);
	void reset(void);
	void processEvents(void);
	void transmitSPP(void);
	void powerOn(void);
	void powerOff(void);
	void shutdown(void);
	void disconnect(void);
	void reconnect(void);
	void linkLastDevice(void);
	void readBattVoltage(void);
	void readLinkStatus(void);
	void eventMask(void);
	void eventAcknowledge(uint8_t);
	void profileLinkBasedonExistingConnectionLinks(void);
	void acceptIncomingCall(void);
	void rejectIncomingCall(void);
	void generateTones(uint8_t);
	void endOngoingCall(void);
	void invokeGoogleAssistant(void);
	void cancelGoogleAssistant(void);
	void redial(void);
};

#endif /* INC_BM62_H_ */
