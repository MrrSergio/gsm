#ifndef __GSM_SOCKET_H__
#define __GSM_SOCKET_H__

#include <cmsis_os.h>

//#define SOCKET_NUM_MAX 1

struct Socket_TCPUDP {
//    bool itUsing = false;
    bool connected = false;
    QueueHandle_t rxQueue = NULL;
    int read_request_counter = 0;
};

extern struct Socket_TCPUDP ClientSocketInfo;

void setup_Socket() ;

#endif
