#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

// Configuration minimale pour utiliser SHA1 et Base64
#define MBEDTLS_SHA1_C
#define MBEDTLS_BASE64_C

// Requis pour le bon fonctionnement général
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_NO_PLATFORM_ENTROPY

// #include "mbedtls/check_config.h"

#endif // MBEDTLS_CONFIG_H