#include "GSMClient.h"
#include "GSMNetwok.h"
#include "debug_log.h"
#include "GSMSocket.h"
#include "system_utils.h"

#include <cmsis_os.h>

EventGroupHandle_t _gsm_client_flags = NULL;

#define GSM_CLIENT_CONNECTED_FLAG          					(1 << 0)
#define GSM_CLIENT_CONNECT_FAIL_FLAG       					(1 << 1)
#define GSM_CLIENT_DISCONNAECTED_FLAG               (1 << 2)
#define GSM_CLIENT_DISCONNAECT_FAIL_FLAG            (1 << 3)
#define GSM_CLIENT_SEND_DATA_TO_MODULE_SUCCESS_FLAG (1 << 4)
#define GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG    (1 << 5)
#define GSM_CLIENT_SEND_FAIL_FLAG                   (1 << 6)
#define GSM_CLIENT_SEND_SUCCESS_FLAG                (1 << 7)
#define GSM_CLIENT_RECEIVE_DATA_SIZE_SUCCESS_FLAG   (1 << 8)
#define GSM_CLIENT_RECEIVE_DATA_SIZE_FAIL_FLAG      (1 << 9)
#define GSM_CLIENT_RECEIVE_DATA_SUCCESS_FLAG   			(1 << 10)
#define GSM_CLIENT_RECEIVE_DATA_FAIL_FLAG   				(1 << 11)
#define GSM_CLIENT_CONFIG_SUCCESS_FLAG   						(1 << 12)
#define GSM_CLIENT_CONFIG_FAIL_FLAG   							(1 << 13)

// Constants for retries and delays
const size_t MAX_RETRY_ATTEMPTS = 3;
const size_t DELAY_AFTER_FAIL = 2000;
const size_t DELAY_AFTER_SUCCESS = 300;
const size_t MAX_WRITE_SIZE = 1000;

static bool setupURC = false;

GSMClient::GSMClient()
{
	if (!_gsm_client_flags) {
		_gsm_client_flags = xEventGroupCreate();
		if (!_gsm_client_flags) {
			DBG_PRINT_E("Evant flag of GSM Client create fail");
		}
	}

	if (!setupURC) {
		_GSM_Base.URCRegister("+CIPCLOSE", [](String urcText) {
        int socket_id = -1, error = -1;
        if (sscanf(urcText.c_str(), "+CIPCLOSE: %d,%d", &socket_id, &error) != 2) {
            DBG_PRINT_E("Close status format fail");
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_DISCONNAECT_FAIL_FLAG);
            return;
        }

        if ((socket_id < 0) || (socket_id > 9)) {
            DBG_PRINT_E("Socket %d is out of range", socket_id);
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_DISCONNAECT_FAIL_FLAG);
            return;
        }

        if (error != 0) {
            DBG_PRINT_I("Socket %d close error code: %d", socket_id, error);
        }

        DBG_PRINT_I("Socket %d is close", socket_id);
        ClientSocketInfo.connected = false;
        xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_DISCONNAECTED_FLAG);
    });

		_GSM_Base.URCRegister("+IPCLOSE", [](String urcText) {
        volatile int socket_id = -1, close_because = -1;
        if (sscanf(urcText.c_str(), "+IPCLOSE: %d,%d", &socket_id, &close_because) != 2) {
            DBG_PRINT_E("Close status format fail");
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_DISCONNAECT_FAIL_FLAG);
            return;
        }

        if ((socket_id < 0) || (socket_id > 9)) {
            DBG_PRINT_E("Socket %d is out of range", socket_id);
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_DISCONNAECT_FAIL_FLAG);
            return;
        }

        DBG_PRINT_I("Socket %d is close bacause %d", socket_id, close_because);
        ClientSocketInfo.connected = false;
        xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_DISCONNAECTED_FLAG);
    });

		setupURC = true;
	}
	setup_Socket();
}

/**
 * @brief Establishes a TCP connection to the specified host and port.
 * @param host The hostname or IP address of the server.
 * @param port The server port number.
 * @param timeout Timeout in milliseconds for the connection attempt.
 * @return 1 if the connection is successful, a negative value for errors:
 *         -2 if the network cannot be opened,
 *         -3 for connection error,
 *         -5 if RX queue creation fails,
 *         -1 for timeout or command errors.
 */
int GSMClient::connect(const char *host, uint16_t port, int32_t timeout)
{
	// Stop any existing connection
	this->stop();

	// Open the network
	if(!Network.Open(timeout)) {
		DBG_PRINT_E("Network open fail");
		return -2;
	}

	// Clear event flags
	xEventGroupClearBits(_gsm_client_flags, GSM_CLIENT_CONFIG_SUCCESS_FLAG | GSM_CLIENT_CONFIG_FAIL_FLAG);

	// Deregister existing URC handler for "+CIPOPEN:"
	_GSM_Base.URCDeregister("+CIPOPEN:");

	// Declare static variables for URC data handling
	static int urc_link_num = -1;
	static int urc_error_code = -1;

	// Register a new URC handler for "+CIPOPEN:"
	_GSM_Base.URCRegister("+CIPOPEN:", [](const String &urcText) {
		int link_num = -1, error_code = -1;
		if(sscanf(urcText.c_str(), "+CIPOPEN: %d,%d", &link_num, &error_code) == 2) {
			urc_link_num = link_num;
			urc_error_code = error_code;
			if(error_code == 0) {
				xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_CONFIG_SUCCESS_FLAG);
			} else {
				xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_CONFIG_FAIL_FLAG);
			}
		} else {
			DBG_PRINT_E("Invalid URC response format: %s", urcText.c_str());
			xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_CONFIG_FAIL_FLAG);
		}
	});

	// Send the command to establish a TCP connection
	// Command format: AT+CIPOPEN=<link_num>,"TCP","<host>",<port>
	String command = "AT+CIPOPEN=0,\"TCP\",\"" + String(host) + "\"," + String(port);
	if(!_GSM_Base.sendCommandFindOK(command, timeout)) {
		DBG_PRINT_E("AT+CIPOPEN timeout");
		return -1; // Timeout
	}

	// Wait for event flags set by the URC handler
	EventBits_t flags = xEventGroupWaitBits(_gsm_client_flags,
																					GSM_CLIENT_CONFIG_SUCCESS_FLAG | GSM_CLIENT_CONFIG_FAIL_FLAG,
																					pdTRUE, pdFALSE, (timeout + 3000) / portTICK_PERIOD_MS);

	if(flags & GSM_CLIENT_CONFIG_SUCCESS_FLAG) {
		// Connection established successfully

		// Create RX queue if not already created
		if(!ClientSocketInfo.rxQueue) {
			ClientSocketInfo.rxQueue = xQueueCreate(GSM_TCP_BUFFER, sizeof(uint8_t));
		}

		if(!ClientSocketInfo.rxQueue) {
			DBG_PRINT_E("RX queue creation failed, insufficient memory");
			return -5;
		}

	} else if(flags & GSM_CLIENT_CONFIG_FAIL_FLAG) {
		DBG_PRINT_E("TCP/IP connection failed, error code: %d", urc_error_code);
		return -3; // Connection error
	} else {
		DBG_PRINT_E("Timeout while establishing TCP/IP connection");
		return -1; // Timeout
	}

	if(!_GSM_Base.sendCommandFindOK("AT+CIPRXGET=1")) {
		DBG_PRINT_E("Socket %d set get the network data manual fail", 0);
	}

	DBG_PRINT_I("Connected !");
	ClientSocketInfo.connected = true;
	return 1;
}

/*
 * @brief
 */
int GSMClient::write(uint8_t c)
{
	return this->write((const uint8_t *)&c, 1);
}

static int check_send_length = 0;
static uint8_t *_data_send_buf = NULL;
static uint16_t _data_send_size = 0;

size_t GSMClient::WriteMaxAllowed(const uint8_t *buf, size_t size)
{
  if(!ClientSocketInfo.connected  || !buf || size == 0) {
      return 0;
  }

//    check_socket_id = this->sock_id;
    check_send_length = size;

    xEventGroupClearBits(_gsm_client_flags, GSM_CLIENT_SEND_SUCCESS_FLAG | GSM_CLIENT_SEND_FAIL_FLAG);
    _GSM_Base.URCDeregister("+CIPSEND");
    _GSM_Base.URCRegister("+CIPSEND", [](String urcText) {
    	_GSM_Base.URCDeregister("+CIPSEND");

        int socket_id = -1, send_length = -1;
        if (sscanf(urcText.c_str(), "+CIPSEND: %d,%*d,%d", &socket_id, &send_length) != 2) {
            DBG_PRINT_E("Send respont format error");
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_FAIL_FLAG);
            return;
        }

        if (socket_id != 0) {
            DBG_PRINT_E("Send respont socket id wrong");
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_FAIL_FLAG);
            return;
        }

        if (send_length == -1) {
            DBG_PRINT_E("Socket %d is disconnect so can't send data", socket_id);
            ClientSocketInfo.connected = false;
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_FAIL_FLAG);
            return;
        }

        if (send_length != check_send_length) {
            DBG_PRINT_E("Socket %d send respont size wrong, Send %d but real send %d", socket_id, check_send_length, send_length);
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_FAIL_FLAG);
            return;
        }

        xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_SUCCESS_FLAG);
    });

    xEventGroupClearBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_SUCCESS_FLAG | GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG);
    _GSM_Base.URCDeregister(">");
    _GSM_Base.URCRegister(">", [](String urcText) {
    	_GSM_Base.URCDeregister(">");

        uint8_t *buff_out = (uint8_t*)pvPortMalloc(_data_send_size);

        delay(50);
        _GSM_Base.send(_data_send_buf, _data_send_size);


        uint32_t beforeTimeout = _GSM_Base.getTimeout();
        _GSM_Base.setTimeout(500);

        uint16_t res_size = 0;
        while(1) { // Try to fix bug
            uint16_t n = _GSM_Base.readBytes(&buff_out[0], 1);
            if (n != 1) {
                DBG_PRINT_E("GSM reply timeout");
                vPortFree(buff_out);
                xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG);
                return;
            }
            if (buff_out[0] == _data_send_buf[0]) {
                res_size += 1;
                break;
            } else if (buff_out[0] == '\r') { // I think i found URC return
                if (_GSM_Base.find('\n')) {
                    String urc = _GSM_Base.readStringUntil('\r');
                    if (_GSM_Base.find('\n')) {
                        urc = urc.substring(0, urc.length() - 1);
                        DBG_PRINT_E("Try to process URC: %s, ", urc.c_str());
                        extern void URCProcess_(const char * data);
                        URCProcess_(urc.c_str());
                    } else {
                        DBG_PRINT_E("Try to read URC but fail (2)");
                        vPortFree(buff_out);
                        xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG);
                        return;
                    }
                } else {
                    DBG_PRINT_E("Try to read URC but fail (1)");
                    vPortFree(buff_out);
                    xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG);
                    return;
                }
            }
        }

        _GSM_Base.setTimeout(3000);
        res_size += _GSM_Base.readBytes(&buff_out[1], _data_send_size - 1);

        // uint16_t res_size = _SIM_Base.readBytes(buffOut, _data_send_size);

        if (res_size != _data_send_size) {
            DBG_PRINT_E("GSM reply data size wrong, Send : %d, Rev: %d", _data_send_size, res_size);
            vPortFree(buff_out);
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG);
            return;
        }

        if (memcmp(_data_send_buf, buff_out, _data_send_size) != 0) {
            DBG_PRINT_E("GSM reply data wrong");
            vPortFree(buff_out);
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG);
            return;
        }

        _GSM_Base.setTimeout(beforeTimeout);

        vPortFree(buff_out);

        xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_SUCCESS_FLAG);
    });

  	_GSM_Base.URCDeregister("+CIPERROR:");
    _GSM_Base.URCRegister("+CIPERROR:", [](String urcText) {
    	_GSM_Base.URCDeregister("+CIPERROR:");

        int error = -1;
        if (sscanf(urcText.c_str(), "+CIPERROR: %d", &error) != 1) {
            DBG_PRINT_E("+CIPERROR: response format error");
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG);
            return;
        }

        if (error == 4) {
//            DBG_PRINT_E("Socket %d is disconnect so can't send data", check_socket_id);
            ClientSocketInfo.connected = false;
            xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG);
        }
    });

    _data_send_buf = (uint8_t*)buf;
    _data_send_size = size;

    if (!_GSM_Base.sendCommand("AT+CIPSEND=0," + String(size), 1000)) {
        DBG_PRINT_E("Send req send data TCP/IP error timeout");
        return 0; // Timeout
    }

    EventBits_t flags;
    flags = xEventGroupWaitBits(_gsm_client_flags, GSM_CLIENT_SEND_DATA_TO_MODULE_SUCCESS_FLAG | GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG, pdTRUE, pdFALSE, 3000 / portTICK_PERIOD_MS);
    _GSM_Base.URCDeregister("+CIPERROR:");
    if (flags & GSM_CLIENT_SEND_DATA_TO_MODULE_SUCCESS_FLAG) {
        DBG_PRINT_I("Socket %d send data to module", 0);
    } else if (flags & GSM_CLIENT_SEND_DATA_TO_MODULE_FAIL_FLAG) {
        DBG_PRINT_I("Socket %d send data to module fail", 0);
        return 0;
    } else {
        DBG_PRINT_E("Socket %d send data to module wait respont timeout", this->sock_id);
        return 0;
    }

    if (!_GSM_Base.waitOKorERROR(300)) {
        DBG_PRINT_E("Wait OK timeout");
        return 0; // Timeout
    }

    flags = xEventGroupWaitBits(_gsm_client_flags, GSM_CLIENT_SEND_SUCCESS_FLAG | GSM_CLIENT_SEND_FAIL_FLAG, pdTRUE, pdFALSE, 3000 / portTICK_PERIOD_MS);
    if (flags & GSM_CLIENT_SEND_SUCCESS_FLAG) {
        DBG_PRINT_I("Socket %d send data success", this->sock_id);
    } else if (flags & GSM_CLIENT_SEND_FAIL_FLAG) {
        DBG_PRINT_I("Socket %d send data fail", this->sock_id);
        return 0;
    } else {
        DBG_PRINT_E("Socket %d send data wait respont timeout", this->sock_id);
        return 0;
    }

    return size;
}

/*
 * @brief Write data to some location, with retry attempts and delay after each attempt.
 * @param buffer The data to write.
 * @param size The size of the data.
 * @return The number of bytes successfully written.
 */
int GSMClient::write(const uint8_t * buffer, size_t size)
{
	size_t real_wrote_size = 0;
	size_t attempts = 0;

	while(real_wrote_size < size && attempts < MAX_RETRY_ATTEMPTS) {
		size_t need_write_size = min(size - real_wrote_size, MAX_WRITE_SIZE);
		size_t ret_size = this->WriteMaxAllowed(&buffer[real_wrote_size], need_write_size);
		if(ret_size == 0) {
			attempts++;
			vTaskDelay(DELAY_AFTER_FAIL);

			if(attempts == MAX_RETRY_ATTEMPTS) {
				return MAX_WRITE_ERROR_CODE;
			}

			continue;
		}
		real_wrote_size += ret_size;
		vTaskDelay(DELAY_AFTER_SUCCESS);
	}
	return real_wrote_size;
}

static int data_in_buffer_length = 0;

/*
 * @brief
 */
int GSMClient::available()
{
	if(!ClientSocketInfo.connected) {
		return -1;
	}

	if (!ClientSocketInfo.rxQueue) {
		DBG_PRINT_E("No queue !");
		return 0;
	}

	size_t dataWaitRead = uxQueueMessagesWaiting(ClientSocketInfo.rxQueue);
	if (uxQueueSpacesAvailable(ClientSocketInfo.rxQueue) <= 0) {
		DBG_PRINT_I("No spaces !");
		return dataWaitRead;
	}

	if (ClientSocketInfo.read_request_counter <= 0) {
//			DBG_PRINT_I("No flage set !");
		return dataWaitRead;
	}

//    check_socket_id = this->sock_id;

  xEventGroupClearBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_SIZE_SUCCESS_FLAG | GSM_CLIENT_RECEIVE_DATA_SIZE_FAIL_FLAG);
  _GSM_Base.URCRegister("+CIPRXGET: 4", [](String urcText) {
  	_GSM_Base.URCDeregister("+CIPRXGET: 4");

		int socket_id = -1;
		if (sscanf(urcText.c_str(), "+CIPRXGET: 4,%d,%d", &socket_id, &data_in_buffer_length) != 2) {
				DBG_PRINT_E("+CIPRXGET: 4: Respont format error");
				xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_SIZE_FAIL_FLAG);
				return;
		}

		if (socket_id != 0) {
				DBG_PRINT_E("+CIPRXGET: 4: Send respont socket id wrong");
				xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_SIZE_FAIL_FLAG);
				return;
		}

		xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_SIZE_SUCCESS_FLAG);
  });

	DBG_PRINT_I("Send request recv data size");
  if (!_GSM_Base.sendCommand("AT+CIPRXGET=4,0")) {
		DBG_PRINT_E("Send req recv data size error (1)");
		return 0; // Timeout
  }

  EventBits_t flags;
  flags = xEventGroupWaitBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_SIZE_SUCCESS_FLAG | GSM_CLIENT_RECEIVE_DATA_SIZE_FAIL_FLAG, pdTRUE, pdFALSE, 300 / portTICK_PERIOD_MS);
  if (flags & GSM_CLIENT_RECEIVE_DATA_SIZE_SUCCESS_FLAG) {
      DBG_PRINT_I("Socket %d recv data size in buffer is %d", this->sock_id, data_in_buffer_length);
  } else if (flags & GSM_CLIENT_RECEIVE_DATA_SIZE_FAIL_FLAG) {
      DBG_PRINT_I("Socket %d recv data size in buffer fail", this->sock_id);
      return 0; // Timeout
  } else {
      DBG_PRINT_E("Socket %d recv data size in buffer timeout", this->sock_id);
      return 0; // Timeout
  }

  if (data_in_buffer_length > 0) {
//      check_socket_id = this->sock_id;

      xEventGroupClearBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_SUCCESS_FLAG | GSM_CLIENT_RECEIVE_DATA_FAIL_FLAG);
      _GSM_Base.URCRegister("+CIPRXGET: 2", [](String urcText) {
      	_GSM_Base.URCDeregister("+CIPRXGET: 2");

          int socket_id = -1, real_data_can_read = 0;
          if (sscanf(urcText.c_str(), "+CIPRXGET: 2,%d,%d,%*d", &socket_id, &real_data_can_read) != 2) {
              DBG_PRINT_E("+CIPRXGET: 2: Respont format error");
              xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_FAIL_FLAG);
              return;
          }

          if (socket_id != 0) {
              DBG_PRINT_E("+CIPRXGET: 2: Send respont socket id wrong");
              xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_FAIL_FLAG);
              return;
          }

//          uint8_t* buff = (uint8_t*)malloc(real_data_can_read + 2); // Add \r\n
          uint8_t* buff = (uint8_t*) pvPortMalloc(real_data_can_read + 2);

          uint16_t res_size = _GSM_Base.getDataAfterIt(buff, real_data_can_read + 2); // Add \r\n

          for (int i=0;i<res_size - 2;i++) { // remove \r\n
              if (xQueueSend(ClientSocketInfo.rxQueue, &buff[i], 0) != pdTRUE) {
                  DBG_PRINT_E("Queue is full ?");
                  break;
              }
          }
//          free(buff);
      		if (buff != NULL) {
      			vPortFree(buff);
      		}

          xEventGroupSetBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_SUCCESS_FLAG);
      });

      int bufferFree = uxQueueSpacesAvailable(ClientSocketInfo.rxQueue);
      uint16_t read_size = min(min(data_in_buffer_length, bufferFree), 1500);
      DBG_PRINT_I("Data in GSM %d , Buffer free: %d, Read size %d", data_in_buffer_length, bufferFree, read_size);
      if (!_GSM_Base.sendCommand("AT+CIPRXGET=2," + String(this->sock_id) + "," + String(read_size), 300)) {
          DBG_PRINT_E("Send req recv data error");
          return 0; // Timeout
      }

      EventBits_t flags;
      flags = xEventGroupWaitBits(_gsm_client_flags, GSM_CLIENT_RECEIVE_DATA_SUCCESS_FLAG | GSM_CLIENT_RECEIVE_DATA_FAIL_FLAG, pdTRUE, pdFALSE, 300 / portTICK_PERIOD_MS);
      if (flags & GSM_CLIENT_RECEIVE_DATA_SUCCESS_FLAG) {
          DBG_PRINT_I("Socket %d recv data in buffer", this->sock_id);
      } else if (flags & GSM_CLIENT_RECEIVE_DATA_FAIL_FLAG) {
          DBG_PRINT_I("Socket %d recv data in buffer fail", this->sock_id);
          return 0;
      } else {
          DBG_PRINT_E("Socket %d recv data in buffer timeout", this->sock_id);
          return 0;
      }

      if (!_GSM_Base.waitOKorERROR(300)) {
          DBG_PRINT_E("Socket %d recv data wait OK timeout", this->sock_id);
      }

      if ((data_in_buffer_length - read_size) > 0) {
//          ClientSocketInfo[this->sock_id].read_request_flag = true;
      } else {
  			ClientSocketInfo.read_request_counter--;
      }
  } else {
		ClientSocketInfo.read_request_counter--;
  }

  return uxQueueMessagesWaiting(ClientSocketInfo.rxQueue);
}

/*
 * @brief
 */
int GSMClient::read()
{
	char c;
	if(this->read((uint8_t*) &c, 1) >= 0) {
		return c;
	}

	return -1;
}

/*
 * @brief
 */
int GSMClient::read(uint8_t *buf, size_t size)
{
	if(!ClientSocketInfo.connected) {
		return -1;
	}

	if(!ClientSocketInfo.rxQueue) {
		return -1;
	}

	size_t data_wait_read = uxQueueMessagesWaiting(ClientSocketInfo.rxQueue);
	if((data_wait_read == 0) || ClientSocketInfo.read_request_counter > 0) {
		this->available();
	}

	data_wait_read = uxQueueMessagesWaiting(ClientSocketInfo.rxQueue);

	if(data_wait_read == 0) {
		return -1;
	}

	int real_read = 0;
	for (size_t i = 0; i < min(data_wait_read, size); i++) {
		if (xQueueReceive(ClientSocketInfo.rxQueue, &buf[i], 0) == pdTRUE) {
			real_read++;
		} else {
			break;
		}
	}

	return real_read;
}

/*
 * @brief
 */
int GSMClient::peek()
{
	if(!ClientSocketInfo.connected) {
		return -1;
	}

	if(!ClientSocketInfo.rxQueue) {
		return -1;
	}

	size_t dataWaitRead = uxQueueMessagesWaiting(ClientSocketInfo.rxQueue);
	if(dataWaitRead == 0) {
		dataWaitRead = this->available();
	}

	if(dataWaitRead == 0) {
		return -1;
	}

	uint8_t c;
	if(xQueuePeek(ClientSocketInfo.rxQueue, &c, 0) != pdTRUE) {
		return -1;
	}

	return c;
}

void GSMClient::flush() { // Not support
    
}

bool GSMClient::connected()
{
	return ClientSocketInfo.connected;
}

GSMClient::operator bool()
{
	return this->connected();
}

void GSMClient::stop()
{
	if(ClientSocketInfo.connected) {
		ClientSocketInfo.connected = false;
		String command = "AT+CIPCLOSE=0";
		if(!_GSM_Base.sendCommandFindOK(command, 1000)) {
			DBG_PRINT_E("Send disconnect TCP/IP error");
			return;
		}
	}
}

GSMClient::~GSMClient()
{
	this->stop();
}
