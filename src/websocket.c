#include "PAMI_2026.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

uint64_t WS_BuildPacket(char* buffer, uint64_t bufferLen,
                        enum WebSocketOpCode opcode,
                        char* payload, uint64_t payloadLen, int mask)
{
    WebsocketPacketHeader_t header;
    int payloadIndex = 2;

    // Remplir les bits de métadonnées
    header.meta.bits.FIN = 1;
    header.meta.bits.RSV = 0;
    header.meta.bits.OPCODE = opcode;
    header.meta.bits.MASK = mask;

    // Longueur du payload
    if (payloadLen < 126) {
        header.meta.bits.PAYLOADLEN = payloadLen;
    } else if (payloadLen < 0x10000) {
        header.meta.bits.PAYLOADLEN = 126;
    } else {
        header.meta.bits.PAYLOADLEN = 127;
    }

    buffer[0] = header.meta.bytes.byte0;
    buffer[1] = header.meta.bytes.byte1;

    // Générer le masque
    header.mask.maskKey = (uint32_t)rand();

    // Masquage du payload
    if (header.meta.bits.MASK) {
        for (uint64_t i = 0; i < payloadLen; i++) {
            payload[i] = payload[i] ^ header.mask.maskBytes[i % 4];
        }
    }

    // Remplissage de la longueur étendue
    if (header.meta.bits.PAYLOADLEN == 126) {
        buffer[2] = (payloadLen >> 8) & 0xFF;
        buffer[3] = payloadLen & 0xFF;
        payloadIndex = 4;
    }

    if (header.meta.bits.PAYLOADLEN == 127) {
        buffer[2] = (payloadLen >> 56) & 0xFF;
        buffer[3] = (payloadLen >> 48) & 0xFF;
        buffer[4] = (payloadLen >> 40) & 0xFF;
        buffer[5] = (payloadLen >> 32) & 0xFF;
        buffer[6] = (payloadLen >> 24) & 0xFF;
        buffer[7] = (payloadLen >> 16) & 0xFF;
        buffer[8] = (payloadLen >> 8)  & 0xFF;
        buffer[9] = payloadLen & 0xFF;
        payloadIndex = 10;
    }

    // Ajouter la clé de masquage
    if (header.meta.bits.MASK) {
        buffer[payloadIndex]     = header.mask.maskBytes[0];
        buffer[payloadIndex + 1] = header.mask.maskBytes[1];
        buffer[payloadIndex + 2] = header.mask.maskBytes[2];
        buffer[payloadIndex + 3] = header.mask.maskBytes[3];
        payloadIndex += 4;
    }

    // Vérification du buffer
    if ((payloadLen + payloadIndex) > bufferLen) {
        printf("WEBSOCKET BUFFER OVERFLOW\r\n");
        return 1;
    }

    // Copier le payload
    for (uint64_t i = 0; i < payloadLen; i++) {
        buffer[payloadIndex + i] = payload[i];
    }

    return (payloadIndex + payloadLen);
}

int WS_ParsePacket(WebsocketPacketHeader_t* header, char* buffer, uint32_t len)
{
    header->meta.bytes.byte0 = (uint8_t)buffer[0];
    header->meta.bytes.byte1 = (uint8_t)buffer[1];

    int payloadIndex = 2;
    header->length = header->meta.bits.PAYLOADLEN;

    if (header->meta.bits.PAYLOADLEN == 126) {
        header->length = ((uint64_t)buffer[2] << 8) | buffer[3];
        payloadIndex = 4;
    }

    if (header->meta.bits.PAYLOADLEN == 127) {
        header->length = ((uint64_t)buffer[6] << 24) |
                         ((uint64_t)buffer[7] << 16) |
                         ((uint64_t)buffer[8] << 8) |
                         (uint64_t)buffer[9];
        payloadIndex = 10;
    }

    if (header->meta.bits.MASK) {
        header->mask.maskBytes[0] = buffer[payloadIndex + 0];
        header->mask.maskBytes[1] = buffer[payloadIndex + 1];
        header->mask.maskBytes[2] = buffer[payloadIndex + 2];
        header->mask.maskBytes[3] = buffer[payloadIndex + 3];
        payloadIndex += 4;

        // Démasquer
        for (uint64_t i = 0; i < header->length; i++) {
            buffer[payloadIndex + i] ^= header->mask.maskBytes[i % 4];
        }
    }

    header->start = payloadIndex;
    return 0;
}