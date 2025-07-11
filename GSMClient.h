#ifndef __GSM_CLIENT_H__
#define __GSM_CLIENT_H__

#include "Client.h"
#include "GSMBase.h"

#define GSM_TCP_BUFFER (16 * 1024)

class GSMClient : public Client {   
    private:
        int8_t sock_id = 0;
        bool _connected = false;
        
    public:
        GSMClient();
        ~GSMClient();

//        int connect(IPAddress ip, uint16_t port, int32_t timeout) ;
        int connect(const char *host, uint16_t port, int32_t timeout) ;

        // From Client class
//        inline int connect(IPAddress ip, uint16_t port) {
//            return connect(ip, port, 30 * 1000); // 30 sec
//        }
        inline int connect(const char *host, uint16_t port) {
            return connect(host, port, 30 * 1000); // 30 sec
        }

        int GetSizeForRead(void);
        size_t WriteMaxAllowed(const uint8_t *buf, size_t size);
        int write(uint8_t) ;
        int write(const uint8_t *buf, size_t size) ;
        int available() ;
        int read() ;
        int read(uint8_t *buf, size_t size) ;
        int peek() ;
        void flush() ;
        void stop() ;
        bool connected() ;
        operator bool() ;

};

#endif
