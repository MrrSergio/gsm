/*
 * GeoStreamClient.h
 *
 *  Created on: 18 нояб. 2022 г.
 *      Author: Sergey
 */

#ifndef PROTOCOLS_GEOSTREAM_GEOSTREAMCLIENT_H_
#define PROTOCOLS_GEOSTREAM_GEOSTREAMCLIENT_H_

#include "Client.h"
#include "GSMClient.h"
#include "GSMClientSecure.h"

class GeoStreamClient : public Client {
public:
	explicit GeoStreamClient(bool is_secure);
	virtual ~GeoStreamClient();

	void enableSecure(bool is_secure);
  int connect(const char *host, uint16_t port);
  int write(uint8_t);
  int write(const uint8_t *buf, size_t size);
  int available();
  int read();
  int read(uint8_t *buf, size_t size);
  int peek() { return 0; }
  void flush() { }
  void stop();
  bool connected();
  operator bool();

  void setCACert(const char *rootCA);
  void setCertificate(const char *client_ca);
  void setPrivateKey (const char *private_key);
  void setServerName (const char *sn);

private:
  bool _is_secure;
  const char *CA_cert = NULL;
  const char *cert = NULL;
  const char *private_key = NULL;
  const char *pskIdent = NULL;
  const char *psKey = NULL;
  const char *server_name = NULL;

  GSMClient *client = NULL;
  GSMClientSecure *tls_client = NULL;
};

#endif /* PROTOCOLS_GEOSTREAM_GEOSTREAMCLIENT_H_ */
