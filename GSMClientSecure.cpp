#include "GSMClientSecure.h"
#include "GSMNetwok.h"
//#include "GSM_LOG.h"

GSMClientSecure::GSMClientSecure() {
    sslclient = new gsm_sslclient_context;
    gsm_ssl_init(sslclient);
    sslclient->handshake_timeout = 120000;
}

void GSMClientSecure::setInsecure() {
    this->insecure = true;
}

void GSMClientSecure::setPreSharedKey(const char *pskIdent, const char *psKey) {
    this->pskIdent = pskIdent;
    this->psKey = psKey;
}

void GSMClientSecure::setCACert(const char *rootCA) {
    this->CA_cert = rootCA;
}

void GSMClientSecure::setCertificate(const char *client_ca) {
    this->cert = client_ca;
}
void GSMClientSecure::setPrivateKey (const char *private_key) {
    this->private_key = private_key;
}

bool GSMClientSecure::verify(const char* fingerprint, const char* domain_name) {
    if (!this->sslclient) {
        return false;
    }

    return gsm_verify_ssl_fingerprint(sslclient, fingerprint, domain_name);
}

void GSMClientSecure::setHandshakeTimeout(unsigned long handshake_timeout) {
    sslclient->handshake_timeout = handshake_timeout * 1000;
}

int GSMClientSecure::connect(const char *host, uint16_t port, const char *sn, const char *rootCABuff, const char *cli_cert, const char *cli_key)
{
    this->CA_cert = rootCABuff;
    this->cert = cli_cert;
    this->private_key = cli_key;
    this->server_name = sn;
    return this->connect(host, port);
}

int GSMClientSecure::connect(const char *host, uint16_t port, const char *pskIdent, const char *psKey)
{
    this->pskIdent = pskIdent;
    this->psKey = psKey;
    return this->connect(host, port);
}

int GSMClientSecure::connect(const char *host, uint16_t port, int32_t timeout)
{
    return gsm_start_ssl_client(
        this->sslclient, 
        host, port, 
        timeout, 
        this->server_name, this->CA_cert, this->cert, this->private_key,
        this->pskIdent, this->psKey,
        this->insecure
    );
}

int GSMClientSecure::write(uint8_t c)
{
    return this->write((const uint8_t *)&c, 1);
}

/*
 * @brief Write data to some location, with retry attempts and delay after each attempt.
 * @param buffer The data to write.
 * @param size The size of the data.
 * @return The number of bytes successfully written, or an error code if the writing fails.
 */
int GSMClientSecure::write(const uint8_t *buf, size_t size)
{
    if (!this->sslclient) {
        return -1;
    }

    if (!this->sslclient->client) {
        return -1;
    }

    int ret_size = gsm_send_ssl_data(this->sslclient, buf, size);
    return ret_size < 0 ? MAX_WRITE_ERROR_CODE : ret_size;
}

int GSMClientSecure::available() {
    if (!this->sslclient) {
        return 0;
    }

    if (!this->sslclient->client) {
        return 0;
    }

    int data_in_buffer = gsm_data_to_read(this->sslclient);
    if (data_in_buffer < 0) {
        if (data_in_buffer == -0x7880) { // MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY
            this->stop();
        }
        return 0;
    }

    return data_in_buffer;
}

int GSMClientSecure::read() {
    char c;
    if (this->read((uint8_t*)&c, 1) > 0) {
        return c;
    } else {
        return -1;
    }
}

int GSMClientSecure::read(uint8_t *buf, size_t size) {
    if (!this->sslclient) {
        return -1;
    }

    if (!this->sslclient->client) {
        return -1;
    }

    int in_buffer = this->available();
    if (in_buffer <= 0) {
        return -1;
    }

    return gsm_get_ssl_receive(this->sslclient, buf, min(size, (size_t)in_buffer));
}

int GSMClientSecure::peek() {
    return -1; // Not support
}

void GSMClientSecure::flush() { // Not support
    
}

bool GSMClientSecure::connected()
{
  if (this->available() > 0) {
		return 1;
	}

	if (!this->sslclient) {
		return 0;
	}

	if (!this->sslclient->client) {
		return 0;
	}

	return this->sslclient->client->connected();
}

GSMClientSecure::operator bool()
{
	return this->connected();
}

void GSMClientSecure::stop()
{
	if(!this->sslclient) {
		return;
	}

	if(!this->sslclient->client) {
		return;
	}

	gsm_stop_ssl_socket(this->sslclient);
}

GSMClientSecure::~GSMClientSecure() {
    this->stop();
    free(sslclient);
}
