#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "aes.h"
#include "cmac.h"

#define LORA_CYPHERKEYBYTES             16
#define LORA_AUTHENTICATIONBLOCKBYTES   16
#define LORA_FRAMEMICBYTES              4

uint8_t* Write4ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}

void LoRa_GenerateDataFrameIntegrityCode(const uint8_t key[], uint8_t const input[], uint16_t dataLength, uint32_t address, bool up, uint32_t sequenceNumber, uint8_t* output)
{
    /*
    Generate artificial B[0] block
    Encrypt B[0] to give X[1]

    for n = 1 to number of blocks
        exclusive OR B[n] with X[n] to give Y[n]
        encrypt Yi using key to give X[n+1]
    */
    uint8_t b0[LORA_AUTHENTICATIONBLOCKBYTES];
    memset(b0, 0 , LORA_AUTHENTICATIONBLOCKBYTES);

    b0[ 0] = 0x49; //authentication flags

    b0[ 5] = up ? 0 : 1;
    Write4ByteValue(&b0[6], address);
    Write4ByteValue(&b0[10], sequenceNumber);

    b0[15] = (uint8_t)dataLength;

    AES_CMAC_CTX cmacctx;
    AES_CMAC_Init(&cmacctx);
    AES_CMAC_SetKey(&cmacctx, key);

    AES_CMAC_Update(&cmacctx, b0, LORA_AUTHENTICATIONBLOCKBYTES);
    AES_CMAC_Update(&cmacctx, input, dataLength);

    uint8_t temp[LORA_AUTHENTICATIONBLOCKBYTES];
    AES_CMAC_Final(temp, &cmacctx);

    memcpy(output, temp, LORA_FRAMEMICBYTES);
}

void LoRa_GenerateJoinFrameIntegrityCode(const uint8_t key[], uint8_t const input[], uint16_t dataLength, uint8_t* output)
{
    AES_CMAC_CTX cmacctx;
    AES_CMAC_Init(&cmacctx);
    AES_CMAC_SetKey(&cmacctx, key);

    AES_CMAC_Update(&cmacctx, input, dataLength);
    uint8_t temp[LORA_AUTHENTICATIONBLOCKBYTES];
    AES_CMAC_Final(temp, &cmacctx);
    memcpy(output, temp, LORA_FRAMEMICBYTES);
}

const uint8_t network_session_key[LORA_CYPHERKEYBYTES] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00,
};

const uint32_t devaddr = 0xcafebabe;

void
join_test()
{

    const uint8_t app_key[LORA_CYPHERKEYBYTES] = {
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff 
    };
    const uint8_t payload[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x37, 0x6a, 0x30, 0x9e, 0x30, 0x34, 0x51,
        0x19, 0x9a, 0x53, 0x4b, 0x41, 0x67, 0x38 
    };
    uint32_t rx_mic;
    uint32_t calculated_mic;

    LoRa_GenerateJoinFrameIntegrityCode(app_key, payload, sizeof(payload)-LORA_FRAMEMICBYTES, (uint8_t*)&calculated_mic);
    printf("calculated_mic:%08x\n", calculated_mic);
    rx_mic = payload[sizeof(payload)-1] << 24;
    rx_mic += payload[sizeof(payload)-2] << 16;
    rx_mic += payload[sizeof(payload)-3] << 8;
    rx_mic += payload[sizeof(payload)-4];
    printf("rx_mic:%08x\n", rx_mic);
}

int main(int argc, char **argv)
{
    uint32_t calculated_mic;
    bool uplink = true;
    uint32_t seq_num = 1;
    unsigned int i;

    if (argc < 2) {
        printf("give payload arg\n");
        return -1;
    }

    if (argv[1][0] == 'J') {
        join_test();
        return 0;
    }

    printf("payload: ");
    for (i = 0; i < strlen(argv[1]); i++)
        printf("%02x ", (unsigned int)argv[1][i]);

    LoRa_GenerateDataFrameIntegrityCode(
        network_session_key,
        (uint8_t*)argv[1],
        strlen(argv[1]),
        devaddr,
        uplink,
        seq_num,
        (uint8_t*)&calculated_mic
    );

    printf("\ncalculated_mic:%x\n", calculated_mic);
    return 0;
}
