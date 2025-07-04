/* Provide SSL/TLS functions to ESP32 with Arduino IDE
*
* Adapted from the ssl_client1 example of mbedtls.
*
* Original Copyright (C) 2006-2015, ARM Limited, All Rights Reserved, Apache 2.0 License.
* Additions Copyright (C) 2017 Evandro Luis Copercini, Apache 2.0 License.
*/

//#include "Arduino.h"
#include "mbedtls/sha256.h"
#include "mbedtls/oid.h"
#include <algorithm>
#include <string>
#include "gsm_ssl_client.h"
#include "mbedtls/error.h"
#include "mbedtls_config.h"
#include "debug_log.h"
#include "system_utils.h"


//#define DEBUG_LEVEL 4

//#include "GSM_LOG.h"

#if !defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED) && !defined(MBEDTLS_KEY_EXCHANGE_SOME_PSK_ENABLED)
#  warning "Please configure IDF framework to include mbedTLS -> Enable pre-shared-key ciphersuites and activate at least one cipher"
#else

static const char *pers = "esp32-tls";


#if DEBUG_LEVEL > 0
/*
 *@brief Debug callback for mbed TLS
 *@brief Just prints on the USB serial port
*/
static void my_debug(void *ctx, int level, const char *file, int line,
										 const char *str)
{
	const char *p, *basename;
	(void) ctx;

	/* Extract basename from file*/
	for(p = basename = file; *p != '\0'; p++) {
			if(*p == '/' || *p == '\\') {
					basename = p + 1;
			}
	}

	DBG_PRINT_I("%s:%04d: |%d| %s", basename, line, level, str);
}
#endif

/*
 * @brief
 */
static int _handle_error(int err, const char * function, int line)
{
    if(err == -30848){
        return err;
    }
#ifdef MBEDTLS_ERROR_C
    char error_buf[100];
//    mbedtls_strerror(err, error_buf, 100);
//    DBG_PRINT_E("[%s():%d]: (%d) %s", function, line, err, error_buf);
#else
    DBG_PRINT_E("[%s():%d]: code %d", function, line, err);
#endif
    return err;
}

#define handle_error(e) _handle_error(e, __FUNCTION__, __LINE__)

// Imprement Client lib
int gsm_mbedtls_net_send(void *ctx, const unsigned char *buf, size_t len)
{
	GSMClient *client = (GSMClient*)ctx;

	if(!client->connected()) {
		return MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY;
	}

	return client->write(buf, len);
}

/*
 * @brief
 */
int gsm_mbedtls_net_recv_timeout(void *ctx, unsigned char *buf, size_t len, uint32_t timeout)
{
     DBG_PRINT_V("mbedtls read %d bytes with timeout is %d", len, timeout);
    timeout = timeout == 0 ? 10000 : timeout;

    GSMClient *client = (GSMClient*)ctx;
    size_t read_len = 0;
    uint32_t startTime = millis();
    while (((millis() - startTime) < timeout)) {
			if (!client->connected()) {
				DBG_PRINT_E("Connected break !");
				break;
			}
			int wait_read = len - read_len;
			if (wait_read <= 0) {
				break;
			}
			int available_read = client->available();
			int will_read = available_read < wait_read ? available_read : wait_read;
			if (will_read > 0) {
				int real_read = client->read(&buf[read_len], will_read);
				if(real_read >= 0) {
					read_len += real_read;
				}
				startTime = millis();
			}
			vTaskDelay(10); // delay(1);
    }

    if (read_len == 0) {
			if (!client->connected()) {
				DBG_PRINT_E("MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY !");
				return MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY;
			} else {
				// return MBEDTLS_ERR_SSL_TIMEOUT;
				DBG_PRINT_E("MBEDTLS_ERR_SSL_WANT_READ !");
				return MBEDTLS_ERR_SSL_WANT_READ;
			}
    }

    return read_len;
}

/*
 * @brief
 */
int gsm_mbedtls_net_recv(void *ctx, unsigned char *buf, size_t len)
{
	GSMClient *client = (GSMClient*)ctx;
	if(!client->connected()) {
		return MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY;
	}
	int read_len = client->read(buf, len);
	DBG_PRINT_I("mbedtls read %d bytes, return %d", len, read_len);
	return read_len >= 0 ? read_len : 0;
}

/*
 * @brief
 */
void gsm_ssl_init(gsm_sslclient_context *ssl_client)
{
	mbedtls_ssl_init(&ssl_client->ssl_ctx);
	mbedtls_ssl_config_init(&ssl_client->ssl_conf);
	mbedtls_ctr_drbg_init(&ssl_client->drbg_ctx);
}

/*
 * @brief
 */
int gsm_start_ssl_client(gsm_sslclient_context *ssl_client, const char *host, uint32_t port, int timeout,\
		 const char *server_name, const char *rootCABuff, const char *cli_cert, const char *cli_key,\
		const char *pskIdent, const char *psKey, bool insecure)
{
    char buf[512];
    int ret, flags;
//    DBG_PRINT_V("Free internal heap before TLS %u", ESP.getFreeHeap());

    if (rootCABuff == NULL && pskIdent == NULL && psKey == NULL && !insecure) {
        return -1;
    }

    DBG_PRINT_V("Starting socket");

    if (ssl_client->client.connect(host, port, timeout) != 1) {
        DBG_PRINT_E("Connect fail");
        return -1;
    }

//    MX_MBEDTLS_Init();

    DBG_PRINT_V("Seeding the random number generator");
    mbedtls_entropy_init(&ssl_client->entropy_ctx);

    ret = mbedtls_ctr_drbg_seed(&ssl_client->drbg_ctx, mbedtls_entropy_func,
                                &ssl_client->entropy_ctx, (const unsigned char *) pers, strlen(pers));
    if (ret < 0) {
			return handle_error(ret);
    }

    DBG_PRINT_V("Setting up the SSL/TLS structure...");

    if ((ret = mbedtls_ssl_config_defaults(&ssl_client->ssl_conf,
                                           MBEDTLS_SSL_IS_CLIENT,
                                           MBEDTLS_SSL_TRANSPORT_STREAM,
                                           MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
        return handle_error(ret);
    }

    mbedtls_x509_crt_profile crt_profile_for_test = mbedtls_x509_crt_profile_default;
    crt_profile_for_test.rsa_min_bitlen = 1024;
    mbedtls_ssl_conf_cert_profile(&ssl_client->ssl_conf, &crt_profile_for_test);

    if(rootCABuff == NULL || cli_cert == NULL || cli_key == NULL) {
    	return -1;
    }

		mbedtls_x509_crt_init(&ssl_client->ca_cert);
		mbedtls_x509_crt_init(&ssl_client->client_cert);
		mbedtls_pk_init(&ssl_client->client_key);

		DBG_PRINT_V("Loading CA cert");
		mbedtls_ssl_conf_authmode(&ssl_client->ssl_conf, MBEDTLS_SSL_VERIFY_REQUIRED);
		ret = mbedtls_x509_crt_parse(&ssl_client->ca_cert, (const unsigned char *)rootCABuff, strlen(rootCABuff) + 1);
		mbedtls_ssl_conf_ca_chain(&ssl_client->ssl_conf, &ssl_client->ca_cert, NULL);
		if (ret < 0) {
			// free the ca_cert in the case parse failed, otherwise, the old ca_cert still in the heap memory, that lead to "out of memory" crash.
			mbedtls_x509_crt_free(&ssl_client->ca_cert);
			return handle_error(ret);
		}

		DBG_PRINT_V("Loading CRT cert");

		ret = mbedtls_x509_crt_parse(&ssl_client->client_cert, (const unsigned char *)cli_cert, strlen(cli_cert) + 1);
		if (ret < 0) {
		// free the client_cert in the case parse failed, otherwise, the old client_cert still in the heap memory, that lead to "out of memory" crash.
		mbedtls_x509_crt_free(&ssl_client->client_cert);
			return handle_error(ret);
		}

		DBG_PRINT_V("Loading private key");
		ret = mbedtls_pk_parse_key(&ssl_client->client_key, (const unsigned char *)cli_key, strlen(cli_key) + 1, NULL, 0);

		if (ret != 0) {
			return handle_error(ret);
		}

		mbedtls_ssl_conf_own_cert(&ssl_client->ssl_conf, &ssl_client->client_cert, &ssl_client->client_key);

    DBG_PRINT_V("Setting hostname for TLS session...");

    // Hostname set here should match CN in server certificate
    if((ret = mbedtls_ssl_set_hostname(&ssl_client->ssl_ctx, /*host"alpha.vke.su""dmz_geoc"*/server_name)) != 0){
    	return handle_error(ret);
    }

    mbedtls_ssl_conf_rng(&ssl_client->ssl_conf, mbedtls_ctr_drbg_random, &ssl_client->drbg_ctx);

    if ((ret = mbedtls_ssl_setup(&ssl_client->ssl_ctx, &ssl_client->ssl_conf)) != 0) {
        return handle_error(ret);
    }

    mbedtls_ssl_set_bio(&ssl_client->ssl_ctx, &ssl_client->client, gsm_mbedtls_net_send, NULL, gsm_mbedtls_net_recv_timeout);
    // mbedtls_ssl_set_bio(&ssl_client->ssl_ctx, &ssl_client->client, gsm_mbedtls_net_send, gsm_mbedtls_net_recv, NULL);

    DBG_PRINT_V("Performing the SSL/TLS handshake...");

//    mbedtls_debug_set_threshold(1);

#if DEBUG_LEVEL > 0
//        mbedtls_ssl_conf_verify(&_ssl_conf, my_verify, NULL);
		mbedtls_ssl_conf_dbg(&ssl_client->ssl_conf, my_debug, NULL);
//		mbedtls_debug_set_threshold(DEBUG_LEVEL);
#endif

    unsigned long handshake_start_time=millis();
    while ((ret = mbedtls_ssl_handshake(&ssl_client->ssl_ctx)) != 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            return handle_error(ret);
        }
        if((millis()-handshake_start_time)>ssl_client->handshake_timeout)
			return -1;
	    vTaskDelay(2); // 2 ticks
    }


    if (cli_cert != NULL && cli_key != NULL) {
//        log_d("Protocol is %s Ciphersuite is %s", mbedtls_ssl_get_version(&ssl_client->ssl_ctx), mbedtls_ssl_get_ciphersuite(&ssl_client->ssl_ctx));
        if ((ret = mbedtls_ssl_get_record_expansion(&ssl_client->ssl_ctx)) >= 0) {
//            log_d("Record expansion is %d", ret);
        } else {
//            log_w("Record expansion is unknown (compression)");
        }
    }

    DBG_PRINT_V("Verifying peer X.509 certificate...");

    if ((flags = mbedtls_ssl_get_verify_result(&ssl_client->ssl_ctx)) != 0) {
			memset(buf, 0, sizeof(buf));
			mbedtls_x509_crt_verify_info(buf, sizeof(buf), "  ! ", flags);
			DBG_PRINT_E("Failed to verify peer certificate! verification info: %s", buf);
			gsm_stop_ssl_socket(ssl_client);  //It's not safe continue.
			return handle_error(ret);
    } else {
			DBG_PRINT_V("Certificate verified.");
    }
    
    if (rootCABuff != NULL) {
			mbedtls_x509_crt_free(&ssl_client->ca_cert);
    }

    if (cli_cert != NULL) {
			mbedtls_x509_crt_free(&ssl_client->client_cert);
    }

    if (cli_key != NULL) {
			mbedtls_pk_free(&ssl_client->client_key);
    }

//    DBG_PRINT_V("Heap size 8: %d", xPortGetFreeHeapSize());

//    DBG_PRINT_V("Free internal heap after TLS %u", ESP.getFreeHeap());

    return 1;
}

/*
 * @brief
 */
void gsm_stop_ssl_socket(gsm_sslclient_context *ssl_client)
{
    DBG_PRINT_V("Cleaning SSL connection.");

    ssl_client->client.stop();

		mbedtls_x509_crt_free(&ssl_client->ca_cert);
		mbedtls_x509_crt_free(&ssl_client->client_cert);
		mbedtls_pk_free(&ssl_client->client_key);

    mbedtls_ssl_free(&ssl_client->ssl_ctx);
    mbedtls_ssl_config_free(&ssl_client->ssl_conf);
    mbedtls_ctr_drbg_free(&ssl_client->drbg_ctx);
    mbedtls_entropy_free(&ssl_client->entropy_ctx);
}

/*
 * @brief
 */
int gsm_data_to_read(gsm_sslclient_context *ssl_client)
{
	volatile int ret, res;
	ret = mbedtls_ssl_read(&ssl_client->ssl_ctx, NULL, 0);
//	DBG_PRINT_E("RET: %i",ret);   //for low level debug
	res = mbedtls_ssl_get_bytes_avail(&ssl_client->ssl_ctx);
//	DBG_PRINT_E("RES: %i",res);    //for low level debug
	if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE && ret < 0) {
		return handle_error(ret);
	}

	return res;
}

/*
 * @brief
 */
int gsm_send_ssl_data(gsm_sslclient_context *ssl_client, const uint8_t *data, size_t len)
{
    DBG_PRINT_V("Writing TCP request with %d bytes...", len); //for low level debug
    int ret = -1;

    while ((ret = mbedtls_ssl_write(&ssl_client->ssl_ctx, data, len)) <= 0) {
			if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE && ret < 0) {
				DBG_PRINT_V("Handling error %d", ret); //for low level debug
				return handle_error(ret);
			}
			//wait for space to become available
			vTaskDelay(2);
    }

    return ret;
}

/*
 * @brief
 */
int gsm_get_ssl_receive(gsm_sslclient_context *ssl_client, uint8_t *data, int length)
{
	//log_d( "Reading HTTP response...");   //for low level debug
	int ret = -1;

	ret = mbedtls_ssl_read(&ssl_client->ssl_ctx, data, length);

//	DBG_PRINT_V( "%d bytes read", ret);   //for low level debug
	return ret;
}

/*
 * @brief
 */
static bool parseHexNibble(char pb, uint8_t* res)
{
    if (pb >= '0' && pb <= '9') {
        *res = (uint8_t) (pb - '0'); return true;
    } else if (pb >= 'a' && pb <= 'f') {
        *res = (uint8_t) (pb - 'a' + 10); return true;
    } else if (pb >= 'A' && pb <= 'F') {
        *res = (uint8_t) (pb - 'A' + 10); return true;
    }
    return false;
}

// Compare a name from certificate and domain name, return true if they match
static bool matchName(const std::string& name, const std::string& domainName)
{
    size_t wildcardPos = name.find('*');
    if (wildcardPos == std::string::npos) {
        // Not a wildcard, expect an exact match
        return name == domainName;
    }

    size_t firstDotPos = name.find('.');
    if (wildcardPos > firstDotPos) {
        // Wildcard is not part of leftmost component of domain name
        // Do not attempt to match (rfc6125 6.4.3.1)
        return false;
    }
    if (wildcardPos != 0 || firstDotPos != 1) {
        // Matching of wildcards such as baz*.example.com and b*z.example.com
        // is optional. Maybe implement this in the future?
        return false;
    }
    size_t domainNameFirstDotPos = domainName.find('.');
    if (domainNameFirstDotPos == std::string::npos) {
        return false;
    }
    return domainName.substr(domainNameFirstDotPos) == name.substr(firstDotPos);
}

// Verifies certificate provided by the peer to match specified SHA256 fingerprint
bool gsm_verify_ssl_fingerprint(gsm_sslclient_context *ssl_client, const char* fp, const char* domain_name)
{
    // Convert hex string to byte array
    uint8_t fingerprint_local[32];
    int len = strlen(fp);
    int pos = 0;
    for (size_t i = 0; i < sizeof(fingerprint_local); ++i) {
        while (pos < len && ((fp[pos] == ' ') || (fp[pos] == ':'))) {
            ++pos;
        }
        if (pos > len - 2) {
//            log_d("pos:%d len:%d fingerprint too short", pos, len);
            return false;
        }
        uint8_t high, low;
        if (!parseHexNibble(fp[pos], &high) || !parseHexNibble(fp[pos+1], &low)) {
//            log_d("pos:%d len:%d invalid hex sequence: %c%c", pos, len, fp[pos], fp[pos+1]);
            return false;
        }
        pos += 2;
        fingerprint_local[i] = low | (high << 4);
    }

    // Get certificate provided by the peer
    const mbedtls_x509_crt* crt = mbedtls_ssl_get_peer_cert(&ssl_client->ssl_ctx);

    if (!crt)
    {
//        log_d("could not fetch peer certificate");
        return false;
    }

    // Calculate certificate's SHA256 fingerprint
    uint8_t fingerprint_remote[32];
    mbedtls_sha256_context sha256_ctx;
    mbedtls_sha256_init(&sha256_ctx);
    mbedtls_sha256_starts(&sha256_ctx, false);
    mbedtls_sha256_update(&sha256_ctx, crt->raw.p, crt->raw.len);
    mbedtls_sha256_finish(&sha256_ctx, fingerprint_remote);

    // Check if fingerprints match
    if (memcmp(fingerprint_local, fingerprint_remote, 32))
    {
//        log_d("fingerprint doesn't match");
        return false;
    }

    // Additionally check if certificate has domain name if provided
    if (domain_name)
        return gsm_verify_ssl_dn(ssl_client, domain_name);
    else
        return true;
}

// Checks if peer certificate has specified domain in CN or SANs
bool gsm_verify_ssl_dn(gsm_sslclient_context *ssl_client, const char* domain_name)
{
//    log_d("domain name: '%s'", (domain_name)?domain_name:"(null)");
    std::string domain_name_str(domain_name);
    std::transform(domain_name_str.begin(), domain_name_str.end(), domain_name_str.begin(), ::tolower);

    // Get certificate provided by the peer
    const mbedtls_x509_crt* crt = mbedtls_ssl_get_peer_cert(&ssl_client->ssl_ctx);

    // Check for domain name in SANs
    const mbedtls_x509_sequence* san = &crt->subject_alt_names;
    while (san != nullptr)
    {
        std::string san_str((const char*)san->buf.p, san->buf.len);
        std::transform(san_str.begin(), san_str.end(), san_str.begin(), ::tolower);

        if (matchName(san_str, domain_name_str))
            return true;

//        log_d("SAN '%s': no match", san_str.c_str());

        // Fetch next SAN
        san = san->next;
    }

    // Check for domain name in CN
    const mbedtls_asn1_named_data* common_name = &crt->subject;
    while (common_name != nullptr)
    {
        // While iterating through DN objects, check for CN object
        if (!MBEDTLS_OID_CMP(MBEDTLS_OID_AT_CN, &common_name->oid))
        {
            std::string common_name_str((const char*)common_name->val.p, common_name->val.len);

            if (matchName(common_name_str, domain_name_str))
                return true;

//            log_d("CN '%s': not match", common_name_str.c_str());
        }

        // Fetch next DN object
        common_name = common_name->next;
    }

    return false;
}
#endif
