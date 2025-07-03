/**
  ******************************************************************************
  * @file    geostream_process.cpp
  * @brief   GeoStream protocol commands.
  * @author  Sergey P
  * @date    20 August 2024
  ******************************************************************************
  */

//******************************************************************************
//  Include section: this is where the header file is attached to the module
//******************************************************************************
#include <SIMA7600.h>
#include <time.h>

#include "geostream.h"
#include "script_manager.h"

#include "debug_log.h"
#include "logger.h"
#include "geostream_process.h"
#include "sms_commands.h"
#include "GSMClient.h"
#include "GSMClientSecure.h"
#include "gsm_sms.h"
#include "GeoStreamClient.h"
#include "indication.h"
//#include "j1939_error_processor.h"
#include "geostream_commands.h"
#include "settings_manager.h"
#include "system_utils.h"
#include "can_manager.h"
#include "flash_writer.h"
#include "gateway.h"
#include "common_types.h"
#include "rtc.h"
#include "gsm_watchdog.h"

//******************************************************************************
//  Local functions prototype section
//******************************************************************************

static uint8_t EnterServiceMode(geos_t* geo_ctx);
static uint8_t ExitFromServiceMode(geos_t* geo_ctx);
static uint8_t GetDeviceInfo(geos_t* geo_ctx);
static uint8_t GetDeviceInfoForServer(geos_t* geo_ctx);
static uint8_t SetupScript(geos_t* geo_ctx, uint8_t* data_ptr);
static uint8_t DeclareVariables(geos_t* geo_ctx, uint8_t *pData);
static uint8_t GetResponseAboutSavingData(geos_t* geo_ctx);
static bool WriteToMemory(uint32_t address, const uint8_t* data, size_t length);

static int DetermineAndSendData(geos_t* geo_ctx, bool *is_empty_after);
static int CollectAndSendJ1939Errors(geos_t* geo_ctx, bool *is_empty_after);
static int CollectAndSendLogData(geos_t* geo_ctx, bool *is_empty_after);
static int SendDataToServer(geos_t* geo_ctx, uint8_t cmd, uint8_t *data, uint32_t size, bool *is_empty_after);
static bool SendConfigRequest(geos_t* geo_ctx);
static void GsmProcess(void *pvArg);
static bool ReadAnswerFromServer(void *pvArg, uint32_t timeout);
static int SendToCan1(uint8_t *pData, uint16_t len);
static int SendToCan2(uint8_t *pData, uint16_t len);
static int SendToCan3(uint8_t *pData, uint16_t len);
static int SendToGsm(uint8_t *p_data, uint16_t len);
static uint8_t StartTransparentModeWrapper(geos_t *pGeo);
static uint32_t ExchangeCAN(geos_t* geo_ctx, bool has_new_msg);
static server_info_t MakeServerInfo(const uint8_t ip[4], const char* cn, uint16_t port);
static void HandleSms();

//******************************************************************************
//  Section for defining variables used in the module
//******************************************************************************

//------------------------------------------------------------------------------
//  Global
//------------------------------------------------------------------------------

/** Commands table for geoStream protocol via a GPRS connection. */
const geo_cmd_t gsm_commands[] = {
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_BOOT_OUT,         0, (uint8_t(*)())&UnknownCommand },
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_BOOT_REQ,         0, (uint8_t(*)())&UnknownCommand },
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_BOOT_IN,          0, (uint8_t(*)())&EnterFirmwareUpdate },
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_SRV_MD_IN,        0, (uint8_t(*)())&EnterServiceMode },
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_SRV_MD_OUT,       0, (uint8_t(*)())&ExitFromServiceMode },
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_CTRL_VAR,        16, (uint8_t(*)())&ControlVariable },
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_TRANSPARENT_IN,   8, (uint8_t(*)())&StartTransparentModeWrapper },
	{ SEQ_DATA,    CRYP_NONE, 0x01, CMD_DEV_INFO,         0, (uint8_t(*)())&GetDeviceInfoForServer },
	{ SEQ_DATA,    CRYP_NONE, 0x01, CMD_GET_CAN_INFO,     4, (uint8_t(*)())&RequestDevInfo },
	{ SEQ_DATA,    CRYP_NONE, 0x01, CMD_GET_VAR,         12, (uint8_t(*)())&GetVariableValue },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_PERITH,          -1, (uint8_t(*)())&ConfigurePeripheral },
	{ SEQ_CONFIG,  CRYP_NONE, 0x00, CMD_PERITH,          -1, (uint8_t(*)())&ConfigurePeripheral },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_LOGIC,           -1, (uint8_t(*)())&SetupScript },
	{ SEQ_CONFIG,  CRYP_NONE, 0x00, CMD_LOGIC,           -1, (uint8_t(*)())&SetupScript },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_VARIABLES,       -1, (uint8_t(*)())&DeclareVariables },
	{ SEQ_CONFIG,  CRYP_NONE, 0x00, CMD_VARIABLES,       -1, (uint8_t(*)())&DeclareVariables },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_GEOZONE,         -1, (uint8_t(*)())&ConfigureGeozones },
	{ SEQ_CONFIG,  CRYP_NONE, 0x00, CMD_GEOZONE,         -1, (uint8_t(*)())&ConfigureGeozones },
	{ SEQ_DATA_REQ,CRYP_NONE, 0x01, CMD_SEND_DATA,        0, (uint8_t(*)())&GetResponseAboutSavingData },
//	{ SEQ_DATA_REQ,CRYP_NONE, 0x01, CMD_J1939_ERRORS,     0, (uint8_t(*)())&GetResponseAboutSavingData },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_TLS_CERT,        -1, (uint8_t(*)())&ConfigureSecurity },
};

/** Commands table for geoStream protocol via a CAN bus. */
const geo_cmd_t can_commands[] = {
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_BOOT_OUT,         0, (uint8_t(*)())&UnknownCommand },
	{ SEQ_SERVICE, CRYP_NONE, 0x01, CMD_CTRL_VAR,        16, (uint8_t(*)())&ControlVariable },
	{ SEQ_DATA,    CRYP_NONE, 0x01, CMD_DEV_INFO,         0, (uint8_t(*)())&GetDeviceInfo },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_PERITH,          -1, (uint8_t(*)())&ConfigurePeripheral },
	{ SEQ_CONFIG,  CRYP_NONE, 0x00, CMD_PERITH,          -1, (uint8_t(*)())&ConfigurePeripheral },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_TLS_CERT,        -1, (uint8_t(*)())&ConfigureSecurity },
	{ SEQ_DATA,    CRYP_NONE, 0x01, CMD_GET_VAR,         12, (uint8_t(*)())&GetVariableValue },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_LOGIC,           -1, (uint8_t(*)())&SetupScript },
	{ SEQ_CONFIG,  CRYP_NONE, 0x00, CMD_LOGIC,           -1, (uint8_t(*)())&SetupScript },
	{ SEQ_CONFIG,  CRYP_NONE, 0x01, CMD_VARIABLES,       -1, (uint8_t(*)())&DeclareVariables },
	{ SEQ_CONFIG,  CRYP_NONE, 0x00, CMD_VARIABLES,       -1, (uint8_t(*)())&DeclareVariables },
};

const uint8_t GSM_COMMANDS_NUM = sizeof(gsm_commands) / sizeof(gsm_commands[0]);
const uint8_t CAN_COMMANDS_NUM = sizeof(can_commands) / sizeof(can_commands[0]);

//------------------------------------------------------------------------------
//  Local
//------------------------------------------------------------------------------

static uint32_t session_num = 0;
static tx_geos_t tx_message = {0};

SmsCommands sms_commands;

geos_t geo_gsm, geo_can[CAN_COUNT];
GeoStreamClient client(false);
FlashWriter script_writer(32, WriteToMemory);

//******************************************************************************
//  The section describing the functions (first global, then local)
//******************************************************************************

/**
 * @brief Initialization of the CAN protocol
 */
void Geostream_BeginCAN(void)
{
	SettingsManager::getInstance().LoadFromFlash();

	/** Configure the geostream protocol for CAN1 */
	geo_cfg_t geo_config = {
		ExchangeCAN,              // exchangeCallbackPtr
		SendToCan1,               // sendCallbackPtr
		NULL,                     // indicateCallbackPtr
		(geo_cmd_t*)can_commands, // array of commands
		(uint8_t)CAN_COMMANDS_NUM // number of commands
	};

	/** Initialize CAN1 */
	geo_can[0] = newGeo(geo_config);
	geo_can[0].protocol_ver = 0x02;
	geo_can[0].init(&geo_can[0]);

	/** Initialize CAN2 */
	geo_config.send = SendToCan2;
	geo_can[1] = newGeo(geo_config);
	geo_can[1].protocol_ver = 0x02;
	geo_can[1].init(&geo_can[1]);

	/** Initialize CAN3 */
	geo_config.send = SendToCan3;
	geo_can[2] = newGeo(geo_config);
	geo_can[2].protocol_ver = 0x02;
	geo_can[2].init(&geo_can[2]);
}

TaskHandle_t gsmHandle = NULL;

#define MONITOR_PERIOD_MS  5000

void GsmMonitorTask(void *pvParameters)
{
	TaskHandle_t gh = (TaskHandle_t) pvParameters;
	char info[256];
	for(;;) {
		vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
		// Состояние задачи
		eTaskState state = eTaskGetState(gh);
		// Хай-уотер-марк стека
		UBaseType_t hwm = uxTaskGetStackHighWaterMark(gh);

		snprintf(info, sizeof(info),
				"GsmProcess: state=%d, stack free=%u words\r\n", (int) state,
				(unsigned) hwm);
		DBG_PRINT_V("%s", info);
	}
}

/**
 * @brief Initialization of the GSM protocol
 */
void Geostream_BeginGSM(void)
{
	SettingsManager::getInstance().LoadFromFlash();

	/** Configure the geostream protocol for GSM */
	geo_cfg_t geo_config = {
		NULL,          // exchangeCallbackPtr (not used for GSM)
		SendToGsm,     // sendCallbackPtr
		NULL,          // indicateCallbackPtr
		(geo_cmd_t*)gsm_commands,  // array of commands
		(uint8_t)GSM_COMMANDS_NUM  // number of commands
	};

	geo_gsm = newGeo(geo_config);
	geo_gsm.protocol_ver = 0x02;
	geo_gsm.init(&geo_gsm);

	GsmWatchdog::Instance().Init();

	/** Start a separate task for GSM processing */
	xTaskCreate(
			GsmProcess,        // task function
			"GsmProcess",      // task name (for debugging)
			4096,              // stack size
			&geo_gsm,          // parameter passed to the task
			2,                 // task priority
			&gsmHandle
	);

//	xTaskCreate(GsmMonitorTask, "GsmMon", 1024, gsmHandle, 2, NULL);
}



/*
 * @brief Exchange process.
 * @param Pointer to the argument.
 */
static void GsmProcess(void *pvArg)
{
	const uint8_t RETRY_MAX = 5;

	geos_t *geo_ptr = (geos_t*)pvArg;
	DeviceSettingsData::V1* config = &g_config.data.v1;

	SettingsManager::getInstance().LoadFromFlash();

	Network.SetApn(String(config->apn_name), String(config->apn_user), String(config->apn_passw));
	GSM1.resetRestartTimer();
	GSM1.begin();
	GSM_Sms.configure();
	GSM_Sms.deleteAllMessages();

	geo_ptr->saveIMEI(geo_ptr, (char*)(_GSM_Base.GetIMEI().c_str()));

	uint32_t last_log_sending_millis = 0;
	uint32_t start_waiting_answer_millis = 0;
	uint32_t start_waiting_any_message_millis = 0;

	main_host = MakeServerInfo(config->mainServerIP, config->main_server_cn, config->mainServerPort);
	config_host = MakeServerInfo(config->serverIP, config->server_cn, config->serverPort);

	server_info_t host = main_host;

	int counter_of_attempts_to_connect = 0;
	int counter_of_attempts_to_send = 0;
	int counter_of_attempts_to_receive = 0;

	bool is_empty_after = true;
	bool is_handshake_passed = false;

	client.enableSecure(g_config.data.v1.use_tls);
	client.setCertificate(g_config.data.v1.client_CA);
	client.setCACert(g_config.data.v1.root_CA);
	client.setPrivateKey(g_config.data.v1.private_key);

	while(1) {
		delay(100);

		HandleSms();

		if(counter_of_attempts_to_connect >= RETRY_MAX) {
			DBG_PRINT_V("GSM.begin...");
			is_handshake_passed = false;
			Network.Close();
			GSM1.searchNetwork();
			GSM_Sms.configure();
			counter_of_attempts_to_connect = 0;
			host = main_host;
		}

		if(counter_of_attempts_to_send >= RETRY_MAX) {
			client.stop();
			server_connection_mode = SERVER_MODE_SEND_DATA;
			host = main_host;
			DBG_PRINT_V("Attemps to send are %d, disconnect from server", counter_of_attempts_to_send);
			counter_of_attempts_to_send = 0;
		}

		if(!client.connected() || !is_handshake_passed) {
			client.setServerName(host.server_name.c_str());
			DBG_PRINT_V("Heap size under: %d", xPortGetFreeHeapSize());
			Led_SetState(LINK_DISCONNECT);
			if(client.connect(host.ip_address.c_str(), host.port) <= 0) {
				client.stop();
				is_handshake_passed = false;
				counter_of_attempts_to_connect++;
				DBG_PRINT_V("Client isn't connected, attemps are %d", counter_of_attempts_to_connect);
				Led_SetState(LINK_CONNECT_ERR);
				vTaskDelay(5000);
			} else {
				DBG_PRINT_V("Client is connected");
				Led_SetState(LINK_CONNECT_OK);
				is_handshake_passed = true;
			}
			DBG_PRINT_V("Heap size after: %d", xPortGetFreeHeapSize());
			continue;

		}

		counter_of_attempts_to_connect = 0;
		GSM1.resetRestartTimer();

		if(server_connection_mode == SERVER_MODE_SEND_DATA) {

			if(host != main_host) {
				client.stop();
				DBG_PRINT_V("Change host to main server");
				host = main_host;
				continue;
			}

			if(millis() >= last_log_sending_millis + (is_empty_after ? 10000 : 1000)) {
				DBG_PRINT_V("Prepare sending log data...");
				int size_of_sent = DetermineAndSendData(geo_ptr, &is_empty_after);

				if(size_of_sent >= 0) {
					last_log_sending_millis = millis();
					counter_of_attempts_to_send = 0;
					if(size_of_sent == 0) {
						DBG_PRINT_V("It is not log data for sending", size_of_sent);
					} else {
						DBG_PRINT_V("SENT LOG %d bytes", size_of_sent);
						server_connection_mode = SERVER_MODE_WAIT_ANSWER;
						start_waiting_answer_millis = millis();
					}
				} else {
					counter_of_attempts_to_send++;
					DBG_PRINT_V("Send log data error #%d", counter_of_attempts_to_send);
				}
			}

		} else if(server_connection_mode == SERVER_MODE_GET_CONFIG) {

			if(host != config_host) {
				client.stop();
				host = config_host;
			}
			if(SendConfigRequest(geo_ptr)) {
				server_connection_mode = SERVER_MODE_SERVICE;
				start_waiting_any_message_millis = millis();
				counter_of_attempts_to_send = 0;
			} else {
				counter_of_attempts_to_send++;
			}

		} else if(server_connection_mode == SERVER_MODE_SERVICE) {
			/** Wait answer from server while 20 seconds */
			if(millis() >= start_waiting_any_message_millis + 90000) {
				DBG_PRINT_V("Exit from service mode");
				geo_ptr->exitFromTransparentMode(geo_ptr);
				server_connection_mode = SERVER_MODE_SEND_DATA;
			}
		} else if(server_connection_mode == SERVER_MODE_TRANSPARENT) {

		} else if(server_connection_mode == SERVER_MODE_WAIT_ANSWER) {
			/** Wait answer from server while 30 seconds */
			if(millis() >= start_waiting_answer_millis + 30000) {
				counter_of_attempts_to_receive++;
				server_connection_mode = SERVER_MODE_SEND_DATA;
				if(counter_of_attempts_to_receive >= RETRY_MAX) {
					DBG_PRINT_V("Server silent too long, scheduling reconnect");
					counter_of_attempts_to_receive = 0;
					counter_of_attempts_to_connect = RETRY_MAX; // to reset the modem
				}
			}
		} else {

		}

		if(ReadAnswerFromServer(geo_ptr, 3000)) {
			start_waiting_any_message_millis = start_waiting_answer_millis = millis();
			counter_of_attempts_to_receive = 0;
			GsmWatchdog::Instance().Reset();
		}
	}

	vTaskDelete(NULL);
}

static server_info_t MakeServerInfo(const uint8_t ip[4], const char* cn, uint16_t port)
{
	server_info_t info;
	info.ip_address = String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2])
			+ "." + String(ip[3]);
	info.port = port;
	info.server_name = String(cn);
	return info;
}

static void HandleSms()
{
	if(GSM_Sms.availabel() > 0) {
		sms_t sms;
		if(GSM_Sms.getSms(sms)) {
			sms_commands.Execute(sms);
			GsmWatchdog::Instance().Reset();
		}
	}
}

/*
 * @brief
 */
static bool ReadAnswerFromServer(void* arg_ptr, uint32_t timeout)
{
	geos_t* geo_ctx = (geos_t*)arg_ptr;

	uint32_t start_millis = millis();

	do {
		int read_size = client.available();

		if(read_size > 0) {
			uint8_t *data = (uint8_t*)pvPortMalloc(read_size);
			int real_data_read = client.read(data, read_size);
			geo_ctx->GetFrame(geo_ctx, data, real_data_read);
			vPortFree(data);
		}

		if (geo_ctx->rxSts == SESSION_PARSING) {
			geo_ctx->rxSts = SESSION_IDLE;
			DBG_PRINT_V("GOT %04X,%02X,%d,s:%d", geo_ctx->rxMsg.hdr.startSeq,\
					geo_ctx->rxMsg.hdr.cmd,\
					geo_ctx->error,\
					geo_ctx->rxMsg.hdr.sessionNum);

			if(geo_ctx->isTransparentMode) {
				geo_ctx->transparentObj->ex.send(geo_ctx->rx_buffer, geo_ctx->rxMsg.hdr.len);
			} else {
				Geo_parseSession(geo_ctx, 1);
				Geo_sendAnswer(geo_ctx);
			}

			if(geo_ctx->rxMsg.hdr.result == GEO_CMD) {
				DBG_PRINT_V("ANSWER %04X,%02X,%d", geo_ctx->tx_msg.hdr.startSeq,\
						geo_ctx->rxMsg.hdr.cmd,\
						geo_ctx->error);
			}

			Led_SetState(geo_ctx->error ? INT_ERR : INT_OK);
			return true;
		}

		delay(500);

	} while (millis() - start_millis < timeout);

	return false;
}

/*
 * @brief
 */
static uint8_t GetResponseAboutSavingData(geos_t *geo)
{
	if(geo->rxMsg.hdr.result == GEO_OK && server_connection_mode == SERVER_MODE_WAIT_ANSWER) {
		server_connection_mode = SERVER_MODE_SEND_DATA;
		tx_message.resetDataBlock(&tx_message);
	}
	return GEO_OK;
}

/**
  * @brief  Replaces the CAN0 data processing function for geostream protocol.
  * @param  Data array.
  * @param  Data size.
  * @retval Always zero (success).
  */
static int SendToCan1(uint8_t * data_ptr, uint16_t size)
{
  CAN_SendMultiMessage(0, data_ptr, size);
	return size;
}

/**
  * @brief  Replaces the CAN1 data processing function for geostream protocol.
  * @param  Data pointer.
  * @param  Data size.
  * @retval Always zero (success).
  */
static int SendToCan2(uint8_t* data_ptr, uint16_t size)
{
  CAN_SendMultiMessage(1, data_ptr, size);
	return size;
}

/**
  * @brief  Replaces the CAN2 data processing function for geostream protocol.
  * @param  Data pointer.
  * @param  Data size.
  * @retval Always zero (success).
  */
static int SendToCan3(uint8_t* data_ptr, uint16_t size)
{
  CAN_SendMultiMessage(2, data_ptr, size);
	return size;
}

/**
  * @brief Wrapper for the function of sending data via GSM module
  */
static int SendToGsm(uint8_t *p_data, uint16_t len)
{
	return client.write(p_data, len);
}

/*
 * @brief  Exchanges messages via a CAN bus.
 * @param  Pointer to the protocol structure.
 * @param  New message status.
 * @retval Waiting time for a new message.
 */
static uint32_t ExchangeCAN(geos_t* geo_ctx, bool has_new_msg)
{
	if(has_new_msg) {
		if(geo_ctx->isTransparentMode) {
			geo_ctx->transparentObj->ex.send(geo_ctx->rx_buffer, geo_ctx->rxMsg.hdr.len);
		} else {
			Geo_parseSession(geo_ctx, 1);
			Geo_sendAnswer(geo_ctx);
			Led_SetState(geo_ctx->error ? INT_ERR : INT_OK);
		}

		geo_ctx->rxSts = SESSION_IDLE;
	}

	return 60000; /* 1 minute */
}

/**
  * @brief  Call back function for CAN-bus data.
  * @param  CAN ID.
  * @param  CAN message ID.
  * @param  Data from CAN-bus
  * @param  Data size.
  * @retval None.
  */
void Geostream_CanBusCallBack(uint8_t can_id, uint32_t msgId, uint8_t *data, uint8_t size)
{
	static uint32_t last_id[CAN_COUNT] = { 0 };
	static uint8_t  last_status[CAN_COUNT] = { 0 };

	/** Check CAN NUMBER */
	if(can_id >= CAN_COUNT || geo_can[can_id].GetFrame == NULL) {
		return;
	}

	/** Check CAN message ID */
	if(msgId != g_config.data.v1.can_broadcast_msg_id[can_id]\
			&& msgId != g_config.data.v1.can_msg_id[can_id]) {
		return;
	}

//	DBG_PRINT_I("Valid message received on CAN channel %d.", can_id);

	/** Check and update last CAN message ID*/
	if(last_id[can_id] != g_config.data.v1.can_broadcast_msg_id[can_id]\
			&& last_id[can_id] != g_config.data.v1.can_msg_id[can_id]) {
		last_id[can_id] = msgId;
	}

	if(last_status[can_id] == 0 || (last_id[can_id] == msgId)) {
//		DBG_PRINT_I("Calling GetFrame for CAN channel %d with Message ID: %x", can_id, msgId);
		last_id[can_id] = msgId;
		last_status[can_id] = geo_can[can_id].GetFrame(&geo_can[can_id], data, size);
	}
}

/**
  * @brief   Switches to the service mode.
  * @param   Pointer to the protocol structure.
  * @retval  Error code.
  */
static uint8_t EnterServiceMode(geos_t *pGeo)
{
	pGeo->setServiceMode(pGeo, true);
	server_connection_mode = SERVER_MODE_SERVICE;
	return GEO_OK;
}

/**
  * @brief   Removes from the service mode.
  * @param   Pointer to the protocol structure.
  * @retval  Error code.
  */
static uint8_t ExitFromServiceMode(geos_t *pGeo)
{
	pGeo->setServiceMode(pGeo, false);
	server_connection_mode = SERVER_MODE_SEND_DATA;
	return GEO_OK;
}


/**
  * @brief   Gets device information.
  * @param   Pointer to the protocol structure.
  * @retval  Error code.
  */
static uint8_t GetDeviceInfo(geos_t* geo_ctx)
{
	if(geo_ctx->rxMsg.hdr.result == GEO_CMD) {

		uint32_t* UID_BASE_ADDR = (uint32_t*)MCU_UID_BASE;

		SettingsManager::getInstance().LoadFromFlash();

		DeviceInfo device_info = {
				.device_details = {
					.manufacture_date = {31, (1 - 1), 25},
//					.manufacture_date = {22, (10 - 1), 24}, // For rev. 4
					.revision = 6,
					.size_space = 0,
					.size_space_firmware = FLASH_APP_AREA_SIZE,
					.size_space_config = 0,
					.uptime = 0,
					.size_byte_log_fill = 0,
					.size_byte_log_free = 0
				},
				.sim1 = "0000000000000000000",
				.sim2 = "0000000000000000000"
		};

		_GSM_Base.GetICCIDIfRead().getBytes((uint8_t*)device_info.sim1, 20, 0);

	#ifdef BOOT_MODE
		device_info.device_mode = CMD_BOOT_IN;
	#else
		device_info.device_mode = CMD_BOOT_OUT;
	#endif

		memcpy(&device_info.boot_version, &g_config.data.v1.boot_version, sizeof(device_info.boot_version));
		memcpy(&device_info.firmware_version, &g_config.data.v1.firmware_version, sizeof(device_info.firmware_version));
		memcpy(&device_info.config_version, &g_config.data.v1.config_version, sizeof(device_info.config_version));
		memset(device_info.device_details.sn, 0, sizeof(device_info.device_details.sn));
		memcpy(device_info.device_details.sn, UID_BASE_ADDR, MCU_UID_SIZE);

		geo_ctx->tx_msg.addDataToBlock(&geo_ctx->tx_msg, (uint8_t*)&device_info, sizeof(device_info));
	} else {
		ReceivedDevInfo();
	}

	return GEO_OK;
}

/**
  * @brief   Gets device information.
  * @param   Pointer to the protocol structure.
  * @retval  Error code.
  */
static uint8_t GetDeviceInfoForServer(geos_t *p_geo)
{
	GetResponseAboutSavingData(p_geo);
	return GetDeviceInfo(p_geo);
}

/**
  * @brief   Configures logic.
  * @param   Pointer to the protocol structure.
  * @param   Pointer to the data.
  * @retval  Error code.
  */
static uint8_t SetupScript(geos_t* geo_ctx, uint8_t* data_ptr)
{
	static uint32_t last_session = 0;
	geo_block_t *pBlk = (geo_block_t*)data_ptr;
	const uint16_t HDR_SIZE = sizeof(pBlk->hdr.size) + sizeof(pBlk->hdr.settType);

	if(last_session != geo_ctx->sessionCnt) {
		/** If first session */
		if(geo_ctx->rxMsg.hdr.arg2 == 1) {
			if(!Script_Clear(SCR_LOGIC)) {
				return GEO_ERR_PROT;
			}
			script_writer.Reset();
			script_writer.SetWriteAddress(FLASH_SCRIPT_START_ADDR);
			session_num = geo_ctx->rxMsg.hdr.sessionNum;
		} else if((--session_num) != geo_ctx->rxMsg.hdr.sessionNum) {
			return GEO_INV_NB_SESS;
		}
		last_session = geo_ctx->sessionCnt;
	}

	if(!script_writer.Append((uint8_t*) &pBlk->buf[HDR_SIZE],
			(pBlk->hdr.size - HDR_SIZE))) {
		return GEO_ERR_WR_LOG;
	}

	/** If last session */
	if(geo_ctx->rxMsg.hdr.sessionNum == 0 && geo_ctx->isSessionEnded) {

		if(!script_writer.Finalize()) {
			return GEO_ERR_WR_LOG;
		}

		SettingsManager::getInstance().LoadFromFlash();
		g_config.data.v1.script_size = script_writer.GetTotalSize();
		SettingsManager::getInstance().SaveSettings();

		/** Run logic*/
		char *report = NULL;
		if(!Script_RunLogic(g_config.data.v1.script_variables_size,
				g_config.data.v1.script_size, &report)) {
			geo_ctx->error = GEO_INV_PAR_CFG;
			geoAddErrorReport(geo_ctx, report);
			return geo_ctx->error;
		}

		struct tm tim = {0};
		RTC_GetTime(&tim);
		if(!SettingsManager::getInstance().UpdateConfigVersionAndSave(tim)) {
			return GEO_ERR;
		}
	}

	return GEO_OK;
}

/**
  * @brief   Configures logic.
  * @param   Pointer to the protocol structure.
  * @param   Pointer to the data.
  * @retval  Error code.
  */
static uint8_t DeclareVariables(geos_t *pGeo, uint8_t *pData)
{
	static uint32_t lastSession = 0;
	geo_block_t *pBlk = (geo_block_t*) pData;

	if(lastSession != pGeo->sessionCnt) {
		/** If first session */
		if(pGeo->rxMsg.hdr.arg2 == 1) {
			if(!Script_Clear(SCR_VARIABLES)) {
				return GEO_ERR_PROT;
			}
			script_writer.Reset();
			script_writer.SetWriteAddress(FLASH_VARS_START_ADDR);
			session_num = pGeo->rxMsg.hdr.sessionNum;
		} else if((--session_num) != pGeo->rxMsg.hdr.sessionNum) {
			return GEO_INV_NB_SESS;
		}
		lastSession = pGeo->sessionCnt;
	}

	if(!script_writer.Append((uint8_t *) &pBlk->buf[0], pBlk->hdr.size)) {
		return GEO_ERR_WR_LOG;
	}


	/** If only last session */
	if(pGeo->rxMsg.hdr.sessionNum == 0 && pGeo->isSessionEnded) {

		if(!script_writer.Finalize()) {
			return GEO_ERR_WR_LOG;
		}

		SettingsManager::getInstance().LoadFromFlash();
		g_config.data.v1.script_variables_size = script_writer.GetTotalSize();
		SettingsManager::getInstance().SaveSettings();

		char *report = NULL;
		if(!Script_RunLogic(g_config.data.v1.script_variables_size,
				g_config.data.v1.script_size, &report)) {
			pGeo->error = GEO_INV_PAR_CFG;
			geoAddErrorReport(pGeo, report);
			return pGeo->error;
		}
		struct tm tim = {0};
		RTC_GetTime(&tim);
		if(!SettingsManager::getInstance().UpdateConfigVersionAndSave(tim)) {
			return GEO_ERR;
		}
	}

	return GEO_OK;
}

/*
 * @brief  Sends a request for device configuration.
 * @param  Pointer to the protocol structure.
 * @retval Operation result.
 */
static bool SendConfigRequest(geos_t *geo_ptr)
{
	tx_geos_t tx_message = newTxGeo(geo_ptr);
	tx_message.hdr.startSeq = SEQ_SERVICE;
	tx_message.hdr.cmd = CMD_CFG_MD;
	tx_message.hdr.result = GEO_CMD;
	tx_message.hdr.encrypType = CRYP_NONE;

	/** Send the message */
  return tx_message.send(geo_ptr, &tx_message);
}

/*
 * @brief  Common function to send data to server.
 * @param  geo_ptr: Pointer to the protocol structure.
 * @param  cmd: Command to be sent.
 * @param  data: Pointer to the data buffer.
 * @param  size: Size of the data.
 * @param  is_empty_after: Pointer to a boolean indicating if buffer is empty after operation.
 * @retval Operation result.
 */
static int SendDataToServer(geos_t *geo_ptr, uint8_t cmd, uint8_t *data, uint32_t size, bool *is_empty_after)
{
	if(size == 0 || size > sizeof(tx_message.data_of_blocks)) {
		return 0;
	}

	tx_message.hdr.startSeq = SEQ_DATA_REQ;
	tx_message.hdr.cmd = cmd;
	tx_message.addDataToNewBlock(&tx_message, data, size);

	int sending_result = tx_message.send(geo_ptr, &tx_message);
	return sending_result < 1 ? sending_result : tx_message.blocksSize;
}

/*
 * @brief  Collects and sends log data to the server using Journal.
 * @param  geo_ptr: Pointer to the protocol structure.
 * @param  is_empty_after: Pointer to a flag indicating if the journal is empty after processing.
 * @retval Operation result.
 */
static int CollectAndSendLogData(geos_t* geo_ptr, bool* is_empty_after)
{
	uint8_t buffer[1024] = { 0 };  // Buffer for reading journal entries.
	uint32_t read_size = 0;        // Size of the current journal entry.

	*is_empty_after = true;

	// Process journal entries until the message buffer is full or the journal is empty.
	while(true) {
		// Try to read the next journal entry.
		if(!Logger::journal.ReadNextEntry(buffer, sizeof(buffer), &read_size) || read_size == 0) {
			break; // Exit if there are no more entries or an error occurred.
		}

		const uint16_t block_data_size = read_size;

		// Check if adding this entry would exceed the message buffer size.
		if((tx_message.blocksSize + read_size + sizeof(block_data_size))
				>= sizeof(tx_message.data_of_blocks)) {
			*is_empty_after = false; // Journal is not empty after this iteration.
			break;
		}

		tx_message.addDataToNewBlock(&tx_message, buffer, read_size);

		// Move to the next journal entry.
		if(!Logger::journal.Advance()) {
			*is_empty_after = false; // Assume journal is not empty if advance fails.
			break;
		}
	}

	// If no data blocks were added, nothing to send.
	if(tx_message.blocksSize == 0) {
		return 0;
	}

	// Check if the journal still contains data after processing.
	*is_empty_after = (Logger::journal.GetUsedSpace() == 0);

	// Send the collected data to the server.
//	return SendDataToServer(geo_ptr, CMD_SEND_DATA, buffer, tx_message.blocksSize,
//			is_empty_after);

	tx_message.hdr.startSeq = SEQ_DATA_REQ;
	tx_message.hdr.cmd = CMD_SEND_DATA;

	int sending_result = tx_message.send(geo_ptr, &tx_message);
	return sending_result < 1 ? sending_result : tx_message.blocksSize;
}


/*
 * @brief  Collects and sends J1939 errors to server.
 * @param  geo_ptr: Pointer to the protocol structure.
 * @retval Operation result.
 */
static int CollectAndSendJ1939Errors(geos_t *geo_ptr, bool *is_empty_after)
{
//	uint8_t error_buffer[1024] = {0};
//	uint16_t errors_buffer_size = 0;
//
//	J1939Error errors = j1939_error_processor.GetErrors();
//	if(errors.count == 0) {
//		return 0;
//	}
//	uint32_t size = sizeof(uint16_t) * errors.count;
//
//	if(sizeof(error_buffer) < size + sizeof(track_point_t)) {
//		return 0;
//	}
//	GenerateGeoPositionPointByteArray(errors.geo_position_point, errors.rtc_timestamp, error_buffer, &errors_buffer_size);
//	memcpy(&error_buffer[errors_buffer_size], errors.errors, size);
//	errors_buffer_size += size;
//	return SendDataToServer(geo_ptr, CMD_J1939_ERRORS, &error_buffer[1], errors_buffer_size - 1, is_empty_after);
}

/*
 * @brief  Determines which data to send and sends it to server.
 * @param  geo_ptr: Pointer to the protocol structure.
 * @retval Operation result.
 */
static int DetermineAndSendData(geos_t* geo_ptr, bool* is_empty_after)
{
	if (tx_message.send == 0) {
		tx_message = newTxGeo(geo_ptr);
		tx_message.hdr.result = GEO_CMD; /** Command to server */
		tx_message.hdr.encrypType = CRYP_NONE;
		tx_message.resetDataBlock(&tx_message);
	}


	if(tx_message.blocksSize > 0) {
		int sending_result = tx_message.send(geo_ptr, &tx_message);
		return sending_result < 1 ? sending_result : tx_message.blocksSize;
//	} else if (j1939_error_processor.HaveErrorsChanged()) {
//		return CollectAndSendJ1939Errors(geo_ptr, is_empty_after);
	} else {
		return CollectAndSendLogData(geo_ptr, is_empty_after);
	}
}

/*
 * @brief
 */
static uint8_t StartTransparentModeWrapper(geos_t* geo_ctx)
{
	server_connection_mode = SERVER_MODE_SERVICE;
	return StartTransparentMode(geo_ctx);
}

/*
 * @brief
 */
static bool WriteToMemory(uint32_t address, const uint8_t* data, size_t length)
{
	return SettingsManager::WriteToFlash(address, data, length);
}

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
