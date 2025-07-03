#include "GeoStreamClient.h"

/*
 * @brief
 */
GeoStreamClient::GeoStreamClient(bool is_secure) : _is_secure(is_secure)
{
	if(_is_secure) {
    if (tls_client == NULL) {
			tls_client = new GSMClientSecure();
    }
	} else {
    if (client == NULL) {
			client = new GSMClient();
    }
	}
}

/*
 * @brief
 */
void GeoStreamClient::enableSecure(bool is_secure)
{
	if(is_secure == _is_secure) {
		return;
	}

	if(_is_secure) {
    if (tls_client != NULL) {
    	if(tls_client->connected()) {
    		tls_client->stop();
    	}
			delete tls_client;
			tls_client = NULL;
    }
    if (client == NULL) {
			client = new GSMClient();
    }
	} else {
    if (client != NULL) {
    	if(client->connected()) {
    		client->stop();
    	}
			delete client;
			client = NULL;
    }
    if (tls_client == NULL) {
			tls_client = new GSMClientSecure();
    }
	}

	_is_secure = is_secure;

}

/*
 * @brief
 */
GeoStreamClient::~GeoStreamClient()
{

}

/*
 * @brief
 */
void GeoStreamClient::setCACert(const char *rootCA)
{
	this->CA_cert = rootCA;
}

/*
 * @brief
 */
void GeoStreamClient::setCertificate(const char *client_ca)
{
	this->cert = client_ca;
}

/*
 * @brief
 */
void GeoStreamClient::setPrivateKey (const char *private_key)
{
	this->private_key = private_key;
}

/*
 * @brief
 */
void GeoStreamClient:: setServerName (const char *sn)
{
	this->server_name = sn;
}

/*
 *@brief
 */
int GeoStreamClient::connect(const char *host, uint16_t port)
{
	int result = -1;

	if (tls_client != NULL) {
		result = tls_client->connect(host, port, server_name, CA_cert, cert, private_key);
	} else if(client != NULL){
		result = client->connect(host, port);
	}
	return result;
}

/*
 *@brief
 */
int GeoStreamClient::write(uint8_t c)
{
    return this->write((const uint8_t *)&c, 1);
}

/*
 * @brief Write data to some location, with retry attempts and delay after each attempt.
 * @param buffer The data to write.
 * @param size The size of the data.
 * @return The number of bytes successfully written, or an error code if the writing fails.
 */
int GeoStreamClient::write(const uint8_t *buf, size_t size)
{
	size_t result = 0;

	if (tls_client != NULL) {
		result = tls_client->write(buf, size);
	} else if(client != NULL){
		result = client->write(buf, size);
	}
	return result;
}

/*
 * @brief
 */
int GeoStreamClient::available()
{
	int result = 0;

	if (tls_client != NULL) {
		result = tls_client->available();
	} else if(client != NULL){
		result = client->available();
	}
	return result;
}

/*
 * @brief
 */
int GeoStreamClient::read()
{
    char c;
    if (this->read((uint8_t*)&c, 1) > 0) {
        return c;
    } else {
        return -1;
    }
}

/*
 * @brief
 */
int GeoStreamClient::read(uint8_t *buf, size_t size)
{
	int result = -1;

	if (tls_client != NULL) {
		result = tls_client->read(buf, size);
	} else if(client != NULL){
		result = client->read(buf, size);
	}
	return result;
}

/*
 * @brief
 */
void GeoStreamClient::stop()
{
	if (tls_client != NULL) {
		tls_client->stop();
	} else if(client != NULL) {
		client->stop();
	}
}

/*
 * @brief
 */
bool GeoStreamClient::connected()
{
	bool result = 0;

	if (tls_client != NULL) {
		result = tls_client->connected();
	} else if(client != NULL){
		result = client->connected();
	}
	return result;
}

GeoStreamClient::operator bool()
{
	return this->connected();
}
