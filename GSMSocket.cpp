
#include "GSMSocket.h"
#include "GSMBase.h"
#include "debug_log.h"

struct Socket_TCPUDP ClientSocketInfo;

void setup_Socket()
{
	static bool setup_socket = false;

	if(setup_socket) {
		return;
	}

	_GSM_Base.URCRegister("+CIPRXGET: 1", [](String urcText) {
		volatile int socket_id = -1;
		volatile int read_values = sscanf(urcText.c_str(), "+CIPRXGET: 1,%d", &socket_id);

		if(read_values != 1 || socket_id != 0 ) {
			return;
		}

		ClientSocketInfo.read_request_counter++;
  });

	setup_socket = true;
}
