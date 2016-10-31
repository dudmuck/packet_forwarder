#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>         /* atoi, exit */
#include <time.h>
#include <unistd.h>
#include <sys/select.h>
#include "loragw_hal.h"
#include "lorawan.h"
#include "aes.h"
#include "cmac.h"
#include "parson.h"
#include "trace.h"
#include "lorawan_bands.h"

#define JOIN_ACCEPT_DELAY1_us                          5000000
#define JOIN_ACCEPT_DELAY2_us                          6000000
#define RECEIVE_DELAY1_us                              1000000
#define RECEIVE_DELAY2_us                              2000000



#define MIN_LORA_PREAMB 6 /* minimum Lora preamble length for this application */
#define STD_LORA_PREAMB 8

#define LORA_ENCRYPTIONBLOCKBYTES       16
#define LORA_CYPHERKEYBYTES             16
#define LORA_AUTHENTICATIONBLOCKBYTES   16
#define LORA_EUI_LENGTH                 8
#define LORA_FRAMEMICBYTES              4

#define LORA_MAXFRAMELENGTH             235
#define LORA_MACHEADERLENGTH            1
#define LORA_MINDATAHEADERLENGTH        7
#define LORA_PORTLENGTH                 1
#define LORA_MAXDATABYTES    (LORA_MAXFRAMELENGTH - (LORA_MACHEADERLENGTH + LORA_MINDATAHEADERLENGTH + LORA_PORTLENGTH + LORA_FRAMEMICBYTES)) //excluding port
#define LORA_NETWORKADDRESSBITS         25

typedef enum eLoRaMacMoteCmd
{
    /*!
     * LinkCheckReq
     */
    MOTE_MAC_LINK_CHECK_REQ          = 0x02,
    /*!
     * LinkADRAns
     */
    MOTE_MAC_LINK_ADR_ANS            = 0x03,
    /*!
     * DutyCycleAns
     */
    MOTE_MAC_DUTY_CYCLE_ANS          = 0x04,
    /*!
     * RXParamSetupAns
     */
    MOTE_MAC_RX_PARAM_SETUP_ANS      = 0x05,
    /*!
     * DevStatusAns
     */
    MOTE_MAC_DEV_STATUS_ANS          = 0x06,
    /*!
     * NewChannelAns
     */
    MOTE_MAC_NEW_CHANNEL_ANS         = 0x07,
    /*!
     * RXTimingSetupAns
     */
    MOTE_MAC_RX_TIMING_SETUP_ANS     = 0x08,
    /*!
     * PingSlotInfoReq
     */
    MOTE_MAC_PING_SLOT_INFO_REQ      = 0x10,
    /*!
     * PingSlotFreqAns
     */
    MOTE_MAC_PING_SLOT_FREQ_ANS      = 0x11,
    /*!
     * BeaconTimingReq
     */
    MOTE_MAC_BEACON_TIMING_REQ       = 0x12,
    /*!
     * BeaconFreqAns
     */
    MOTE_MAC_BEACON_FREQ_ANS         = 0x13,
}LoRaMacMoteCmd_t;

typedef enum eLoRaMacSrvCmd
{
    /*!
     * LinkCheckAns
     */
    SRV_MAC_LINK_CHECK_ANS           = 0x02,
    /*!
     * LinkADRReq
     */
    SRV_MAC_LINK_ADR_REQ             = 0x03,
    /*!
     * DutyCycleReq
     */
    SRV_MAC_DUTY_CYCLE_REQ           = 0x04,
    /*!
     * RXParamSetupReq
     */
    SRV_MAC_RX_PARAM_SETUP_REQ       = 0x05,
    /*!
     * DevStatusReq
     */
    SRV_MAC_DEV_STATUS_REQ           = 0x06,
    /*!
     * NewChannelReq
     */
    SRV_MAC_NEW_CHANNEL_REQ          = 0x07,
    /*!
     * RXTimingSetupReq
     */
    SRV_MAC_RX_TIMING_SETUP_REQ      = 0x08,
    /*!
     * PingSlotInfoAns
     */
    SRV_MAC_PING_SLOT_INFO_ANS       = 0x10,
    /*!
     * PingSlotChannelReq
     */
    SRV_MAC_PING_SLOT_CHANNEL_REQ    = 0x11,
    /*!
     * BeaconTimingAns
     */
    SRV_MAC_BEACON_TIMING_ANS        = 0x12,
    /*!
     * BeaconFreqReq
     */
    SRV_MAC_BEACON_FREQ_REQ          = 0x13,
}LoRaMacSrvCmd_t;

typedef union {
    struct {
        uint8_t major   : 2;    // 0 1
        uint8_t rfu     : 3;    // 2 3 4
        uint8_t MType   : 3;    // 5 6 7
    } bits;
    uint8_t octet;
} mhdr_t;

typedef struct {
    mhdr_t mhdr;
    uint8_t AppEUI[LORA_EUI_LENGTH];
    uint8_t DevEUI[LORA_EUI_LENGTH];
    uint16_t DevNonce;
} __attribute__((packed)) join_req_t;

struct _devNonce_list {
    uint16_t value;
    time_t last_seen;
    struct _devNonce_list* next;
};

typedef struct {
    uint8_t app_eui[LORA_EUI_LENGTH];
    uint8_t dev_eui[LORA_EUI_LENGTH];
    uint8_t app_key[LORA_CYPHERKEYBYTES];

    uint32_t dev_addr;
    uint8_t network_session_key[LORA_CYPHERKEYBYTES];
    uint8_t app_session_key[LORA_CYPHERKEYBYTES];

    uint16_t FCntDown;

    struct _devNonce_list* devNonce_list;
} mote_t;

struct _mote_list {
    mote_t mote;
    struct _mote_list* next;
};

struct _mote_list* mote_list = NULL;

typedef enum {
    MTYPE_JOIN_REQ = 0,
    MTYPE_JOIN_ACC,//1
    MTYPE_UNCONF_UP,//2
    MTYPE_UNCONF_DN,//3
    MTYPE_CONF_UP,//4
    MTYPE_CONF_DN,//5
    MTYPE_RFU,//6
    MTYPE_P,//7
} mtype_e;


typedef union {
    struct {
        uint8_t FOptsLen        : 4;    // 0 1 2 3
        uint8_t RFU_FPending    : 1;    // 4 FPending only for downlink frames
        uint8_t ACK             : 1;    // 5
        uint8_t ADCACKReq       : 1;    // 6
        uint8_t ADR             : 1;    // 7
    } bits;
    uint8_t octet;
} FCtrl_t;

#define DEVADDR_NONE        0xffffffff
typedef struct {
    uint32_t DevAddr;
    FCtrl_t FCtrl;
    uint16_t FCnt;
} __attribute__((packed)) fhdr_t;

uint32_t network_id;
uint32_t networkAddress = 0;  // bits 24..0 of DevAddr, for join accept
uint8_t tx_rf_chain;    // written by sx1301 conf
unsigned int join_ignore_count = 0;
unsigned int current_join_attempt = 0;
unsigned int devnonce_experation = 600; // in seconds
#define DEVNONCE_HISTORY_SIZE   20  /* how many devNonce to keep */

uint8_t user_downlink[128];
int user_downlink_length = -1;

void print_octets(char const* label, uint8_t const* buf, uint8_t buf_len)
{
    int i;
    printf("%s:", label);
    for (i = 0; i < buf_len; i++)
        printf(" %02x", buf[i]);
    printf("\n");
}

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


uint8_t* Write3ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}

uint8_t* Write2ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}

uint8_t* Write1ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input;

    return ptr;
}


void BlockExOr(uint8_t const l[], uint8_t const r[], uint8_t out[], uint16_t bytes)
{
    uint8_t const* lptr = l;
    uint8_t const* rptr = r;
    uint8_t* optr = out;
    uint8_t const* const end = out + bytes;

    for (;optr < end; lptr++, rptr++, optr++)
        *optr = *lptr ^ *rptr;
}

static uint16_t FindBlockOverhang(uint16_t inputDataLength)
{
    return inputDataLength & (LORA_ENCRYPTIONBLOCKBYTES - 1);
}

uint16_t CountBlocks(uint16_t inputDataLength)
{
    uint16_t blockSizeMinus1 = LORA_ENCRYPTIONBLOCKBYTES - 1;
    uint16_t inRoundDown = inputDataLength & ~blockSizeMinus1;
    uint16_t roundUp = (FindBlockOverhang(inputDataLength) > 0) ? 1 : 0;
    uint16_t result = inRoundDown / LORA_ENCRYPTIONBLOCKBYTES + roundUp;

    return result;
}

//void LoRa_EncryptPayload(CypherKey const& key, uint8 const in[], uint16 inputDataLength, uint32 address, bool up, uint32 sequenceNumber, uint8 out[])
void LoRa_EncryptPayload(const uint8_t key[], uint8_t const in[], uint16_t inputDataLength, uint32_t address, bool up, uint32_t sequenceNumber, uint8_t out[])
{
    if (inputDataLength == 0)
        return;

    uint8_t A[LORA_ENCRYPTIONBLOCKBYTES];

    memset(A, 0, LORA_ENCRYPTIONBLOCKBYTES);

    A[ 0] = 0x01; //encryption flags
    A[ 5] = up ? 0 : 1;

    Write4ByteValue(&A[6], address);
    Write4ByteValue(&A[10], sequenceNumber);

    uint16_t const blocks = CountBlocks(inputDataLength);
    uint16_t const overHangBytes = FindBlockOverhang(inputDataLength);

    uint8_t const* blockInput = in;
    uint8_t* blockOutput = out;
    for (uint16_t i = 1; i <= blocks; i++, blockInput += LORA_ENCRYPTIONBLOCKBYTES, blockOutput += LORA_ENCRYPTIONBLOCKBYTES)
    {
        A[15] = (uint8_t)i;

        aes_context aesContext;
        aes_set_key(key, LORA_CYPHERKEYBYTES, &aesContext);

        uint8_t S[LORA_CYPHERKEYBYTES];
        aes_encrypt(A, S, &aesContext);

        uint16_t bytesToExOr;
        if ((i < blocks) || (overHangBytes == 0))
            bytesToExOr = LORA_CYPHERKEYBYTES;
        else
            bytesToExOr = overHangBytes;

        BlockExOr(S, blockInput, blockOutput, bytesToExOr);
    }
}

// LoRa::GenerateDataFrameIntegrityCode(key, Data(), AuthenticatedLength(true), Address(), true, sequenceNumber, calculatedMic);
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

static void
send_downlink_(struct lgw_pkt_tx_s* tx_pkt, mote_t* mote, uint32_t rx_count_us)
{
    fhdr_t* fhdr = (fhdr_t*)&tx_pkt->payload[1];
    uint8_t* mic_ptr;
    fhdr->DevAddr = mote->dev_addr; // if different address, then multicast
    fhdr->FCnt = mote->FCntDown++;
    // tx_pkt->size must have already FRMPayload length or zero
    tx_pkt->size += LORA_MACHEADERLENGTH + sizeof(fhdr_t) + fhdr->FCtrl.bits.FOptsLen;
    mic_ptr = &tx_pkt->payload[tx_pkt->size];
    LoRa_GenerateDataFrameIntegrityCode(mote->network_session_key, tx_pkt->payload, tx_pkt->size, fhdr->DevAddr, false, fhdr->FCnt, mic_ptr);
    tx_pkt->size += LORA_FRAMEMICBYTES;

    tx_pkt->tx_mode = TIMESTAMPED;
    if (dl_rxwin == 1)
        tx_pkt->count_us = rx_count_us + RECEIVE_DELAY1_us;
    else if (dl_rxwin == 2)
        tx_pkt->count_us = rx_count_us + RECEIVE_DELAY2_us;

    tx_pkt->rf_chain = tx_rf_chain;
    tx_pkt->rf_power = 20;   // TODO
    tx_pkt->modulation = MOD_LORA;
    tx_pkt->coderate = CR_LORA_4_5;
    tx_pkt->invert_pol = true;
    tx_pkt->preamble = STD_LORA_PREAMB;
    tx_pkt->no_crc = true;
    tx_pkt->no_header = false;
    if (lgw_send(*tx_pkt) == LGW_HAL_ERROR) {
        printf("lgw_send() failed\n");
    }
}

static void
parse_mac_command(struct lgw_pkt_rx_s *rx_pkt, mote_t* mote, uint8_t* cmd_buf, uint8_t cmd_buf_len)
{
    int i;
    printf("mac_command:");
    for (i = 0; i < cmd_buf_len; i++)
        printf("%02x ", cmd_buf[i]);
    printf("\n");

    if (cmd_buf[0] != MOTE_MAC_PING_SLOT_INFO_REQ) {
        printf("TODO mac cmd %02x\n", cmd_buf[0]);
        return;
    }

    printf("MOTE_MAC_PING_SLOT_INFO_REQ\n");

    /* generate ack reply */
    struct lgw_pkt_tx_s tx_pkt;
    uint8_t* fopts_ptr = &tx_pkt.payload[sizeof(mhdr_t) + sizeof(fhdr_t)];
    tx_pkt.size = 0;
    fhdr_t* fhdr = (fhdr_t*)&tx_pkt.payload[1];
    mhdr_t *mhdr_rx = (mhdr_t*)&rx_pkt->payload[0];
    mhdr_t *mhdr_tx = (mhdr_t*)&tx_pkt.payload[0];
    if (mhdr_rx->bits.MType == MTYPE_UNCONF_UP)
        mhdr_tx->bits.MType = MTYPE_UNCONF_DN;
    else if (mhdr_rx->bits.MType == MTYPE_CONF_UP)
        mhdr_tx->bits.MType = MTYPE_CONF_DN;

    fhdr->FCtrl.octet = 0;
    fhdr->FCtrl.bits.FOptsLen = 1;
    fhdr->FCtrl.bits.ACK = 1;
    *fopts_ptr = SRV_MAC_PING_SLOT_INFO_ANS;

    band_conv(rx_pkt, &tx_pkt);
    send_downlink_(&tx_pkt, mote, rx_pkt->count_us);

}

void print_hal_bw(uint8_t bandwidth)
{
    printf("bw");
    switch (bandwidth) {
        case BW_125KHZ: printf("125 "); break;
        case BW_250KHZ: printf("250 "); break;
        case BW_500KHZ: printf("500 "); break;
        default: printf("? "); break;
    }
}
void print_hal_sf(uint8_t datarate)
{
    printf("sf");
    switch (datarate) {
        case DR_LORA_SF7: printf("7 "); break;
        case DR_LORA_SF8: printf("8 "); break;
        case DR_LORA_SF9: printf("9 "); break;
        case DR_LORA_SF10: printf("10 "); break;
        case DR_LORA_SF11: printf("11 "); break;
        case DR_LORA_SF12: printf("12 "); break;
    }
}

static void
parse_uplink(mote_t* mote, struct lgw_pkt_rx_s *rx_pkt)
{
    uint8_t decrypted[256];
    uint32_t calculated_mic;
    uint32_t* rx_mic;
    mhdr_t *mhdr = (mhdr_t*)&rx_pkt->payload[0];
    fhdr_t *fhdr = (fhdr_t*)&rx_pkt->payload[1];
    int o = sizeof(mhdr_t) + sizeof(fhdr_t) + fhdr->FCtrl.bits.FOptsLen;
    uint8_t FRMPayload_length = (rx_pkt->size - LORA_FRAMEMICBYTES) - (o + 1);
    uint8_t* FRMPayload = &rx_pkt->payload[o+1];
    uint8_t* rx_fport_ptr = &rx_pkt->payload[o];

    printf("o:%d fport:%d FRMPayload_length:%d\n", o, *rx_fport_ptr, FRMPayload_length);
    for (o = 0; o < FRMPayload_length; o++) {
        printf("%02x ", FRMPayload[o]);
    }
    printf("\n");

    LoRa_GenerateDataFrameIntegrityCode(mote->network_session_key, rx_pkt->payload, rx_pkt->size-LORA_FRAMEMICBYTES, fhdr->DevAddr, true, fhdr->FCnt, (uint8_t*)&calculated_mic);
    rx_mic = (uint32_t*)&rx_pkt->payload[rx_pkt->size-LORA_FRAMEMICBYTES];
    if (calculated_mic != *rx_mic) {
        printf("genMic:%08x, rxMic:%08x\n", calculated_mic, *rx_mic);
        printf("mic fail\n");
        return;
    }

    // FRMPayload
    printf("FRMPayload:");
    for (o = 0; o < FRMPayload_length; o++) {
        printf("%02x ", FRMPayload[o]);
    }
    printf("\n");
    LoRa_EncryptPayload(mote->app_session_key, FRMPayload, FRMPayload_length, fhdr->DevAddr, true, fhdr->FCnt, decrypted);
    printf("decrypt:");
    for (o = 0; o < FRMPayload_length; o++) {
        printf("%02x ", decrypted[o]);
    }

    if (mhdr->bits.MType != MTYPE_CONF_UP && user_downlink_length == -1) {
        printf("\n");
        return;
    }

    /* schedule downlink tx confirm */
    /* 4byte devaddr, 1byte fCtrl, 2byte sequenceCounter, 4byte MIC */
    struct lgw_pkt_tx_s tx_pkt;
    tx_pkt.size = 0;
    if (mhdr->bits.MType == MTYPE_CONF_UP)
        tx_pkt.payload[0] = MTYPE_CONF_DN << 5; // MHDR
    else if (mhdr->bits.MType == MTYPE_UNCONF_UP)
        tx_pkt.payload[0] = MTYPE_UNCONF_DN << 5; // MHDR

    fhdr = (fhdr_t*)&tx_pkt.payload[1];
    fhdr->FCtrl.octet = 0;
    fhdr->FCtrl.bits.FOptsLen = 0;
    fhdr->FCtrl.bits.ACK = 1;

    if (user_downlink_length != -1) {
        /* add user payload */
        int o = sizeof(mhdr_t) + sizeof(fhdr_t) + fhdr->FCtrl.bits.FOptsLen;
        uint8_t* tx_fport_ptr = &tx_pkt.payload[o];
        uint8_t* FRMPayload = &tx_pkt.payload[o+1];
        LoRa_EncryptPayload(mote->app_session_key, user_downlink, user_downlink_length, mote->dev_addr, false, mote->FCntDown, FRMPayload);
        *tx_fport_ptr = *rx_fport_ptr;
        tx_pkt.size = user_downlink_length + 1; // +1 for fport
        user_downlink_length = -1;  // mark as sent
    }

    band_conv(rx_pkt, &tx_pkt);
    send_downlink_(&tx_pkt, mote, rx_pkt->count_us);

    printf("\n");
}

void GenerateSessionKey(bool generateNetworkKey, const uint8_t* applicationKey, uint32_t networkId, uint32_t applicationNonce, uint16_t deviceNonce, uint8_t* output)
{
    uint8_t input[LORA_ENCRYPTIONBLOCKBYTES];

    input[0] = generateNetworkKey ? 0x01 : 0x02;
    uint8_t* ptr = &input[1];

    ptr = Write3ByteValue(ptr, applicationNonce);
    ptr = Write3ByteValue(ptr, networkId);
    ptr = Write2ByteValue(ptr, deviceNonce);
    memset(ptr, 0, LORA_ENCRYPTIONBLOCKBYTES - (ptr - input));

    aes_context aesContext;
    aes_set_key(applicationKey, LORA_CYPHERKEYBYTES, &aesContext);

    aes_encrypt(input, output, &aesContext);
}

void CryptJoinServer(uint8_t const* key, uint8_t const* input, uint16_t length, uint8_t* output)
{
    aes_context aesContext;
    aes_set_key(key, (length_type)length, &aesContext);

    aes_decrypt(input, output, &aesContext);
}


void SendJoinComplete(uint16_t deviceNonce, uint8_t firstReceiveWindowDataRateoffset, uint8_t secondReceiveWindowDataRateNibble, uint8_t classARxWindowDelay_s, mote_t* mote, struct lgw_pkt_rx_s *rx_pkt)
{
    struct lgw_pkt_tx_s tx_pkt;
    uint8_t networkSessionKey[LORA_CYPHERKEYBYTES];
    uint8_t uncyphered[LORA_MAXDATABYTES];
    uint8_t* current = uncyphered;
    uint32_t applicationNonce = rand() & 0xffffff;  // 24bit

    GenerateSessionKey(true, mote->app_key, network_id, applicationNonce, deviceNonce, networkSessionKey);
    memcpy(mote->network_session_key, networkSessionKey, LORA_CYPHERKEYBYTES);
    //print_octets("network_session_key", mote->network_session_key, LORA_CYPHERKEYBYTES);
    
    printf("SendJoinComplete(): mote->dev_addr:%x\n", mote->dev_addr);
    if (mote->dev_addr == DEVADDR_NONE) {
        // new mote joining
        printf("new-mote\n");
        mote->dev_addr = ++networkAddress | (network_id << LORA_NETWORKADDRESSBITS);
    } else
        printf("rejoin\n");

    printf("networkAddress:%d classARxWindowDelay_s:%d, \n", networkAddress, classARxWindowDelay_s);
    *(current++) = MTYPE_JOIN_ACC << 5; // MHDR     0
    current = Write3ByteValue(current, applicationNonce);// 1 2 3
    current = Write3ByteValue(current, network_id);// 4 5 6
    current = Write4ByteValue(current, mote->dev_addr); // 7 8 9 10
    current = Write1ByteValue(current, (firstReceiveWindowDataRateoffset << 4) | (secondReceiveWindowDataRateNibble & 0xf)); // 11 
    current = Write1ByteValue(current, classARxWindowDelay_s - 1); // 12

    uint16_t authenticatedBytes = current - uncyphered;
    LoRa_GenerateJoinFrameIntegrityCode(mote->app_key, uncyphered, authenticatedBytes, current);
    current += LORA_FRAMEMICBYTES;

    print_octets("join-acc uncyphered", uncyphered, current-uncyphered);

    tx_pkt.payload[0] = MTYPE_JOIN_ACC << 5; // MHDR
    //encrypt
    uint16_t cypherBytes = (current - uncyphered) - LORA_MACHEADERLENGTH;
    CryptJoinServer(mote->app_key, &uncyphered[LORA_MACHEADERLENGTH], cypherBytes, &tx_pkt.payload[LORA_MACHEADERLENGTH]);

    /**** RF TX ********/
    tx_pkt.coderate = CR_LORA_4_5;
    tx_pkt.modulation = MOD_LORA;

    band_conv(rx_pkt, &tx_pkt);

    tx_pkt.tx_mode = TIMESTAMPED;
    if (dl_rxwin == 1)
        tx_pkt.count_us = rx_pkt->count_us + JOIN_ACCEPT_DELAY1_us;
    else if (dl_rxwin == 2)
        tx_pkt.count_us = rx_pkt->count_us + JOIN_ACCEPT_DELAY2_us;
    tx_pkt.rf_chain = tx_rf_chain;
    tx_pkt.rf_power = 20;   // TODO
    tx_pkt.invert_pol = true;
    //tx_pkt.f_dev
    tx_pkt.preamble = STD_LORA_PREAMB;
    tx_pkt.no_crc = true;
    tx_pkt.no_header = false;
    tx_pkt.size = current - uncyphered;

    if (tx_pkt.modulation == MOD_LORA) {
        printf("gw tx lora at %uhz ", tx_pkt.freq_hz);
        print_hal_bw(tx_pkt.bandwidth);
        print_hal_sf(tx_pkt.datarate);
        printf("\n");
    } else if (tx_pkt.modulation == MOD_FSK) {
        printf("gw tx FSK f_dev:%d datarate:%d\n", tx_pkt.f_dev, tx_pkt.datarate);
    }
    int i = lgw_send(tx_pkt);
    if (i == LGW_HAL_ERROR) {
        printf("lgw_send() failed\n");
    }

    GenerateSessionKey(false, mote->app_key, network_id, applicationNonce, deviceNonce, mote->app_session_key);
    //print_octets("app_session_key", mote->app_session_key, LORA_CYPHERKEYBYTES);
}

static int
check_devnonce(mote_t* mote, uint16_t DevNonce)
{

    if (mote->devNonce_list == NULL) {
        /* first join attempt seen from this mote */
        mote->devNonce_list = malloc(sizeof(struct _devNonce_list));
        memset(mote->devNonce_list, 0, sizeof(struct _devNonce_list));
        mote->devNonce_list->value = DevNonce;
        mote->devNonce_list->last_seen = time(NULL);
        return 0;
    }
    /* subsequent join attempt */
    int history_cnt = 0;
    struct _devNonce_list* my_devNonce_list = mote->devNonce_list;
    while (my_devNonce_list != NULL) {
        time_t now = time(NULL);
        history_cnt++;
        /* check this devnonce against previous */
        //printf("%d: %04x, %lu\n", history_cnt, my_devNonce_list->value, now-my_devNonce_list->last_seen);
        if (my_devNonce_list->value == DevNonce && (now-my_devNonce_list->last_seen < devnonce_experation)) {
            printf("devNonce %04x seen %lu seconds ago\n", DevNonce, now - my_devNonce_list->last_seen);
            return -1;
        }
        if (my_devNonce_list->next == NULL)
            break;
        my_devNonce_list = my_devNonce_list->next;
    }
    if (history_cnt < DEVNONCE_HISTORY_SIZE) {
        /* add new */
        my_devNonce_list->next = malloc(sizeof(struct _devNonce_list));
        my_devNonce_list = my_devNonce_list->next;
        memset(my_devNonce_list , 0, sizeof(struct _devNonce_list));
        my_devNonce_list->value = DevNonce;
        my_devNonce_list->last_seen = time(NULL);
    } else {
        /* overwrite oldest */
        struct _devNonce_list* oldest_devNonce_list = NULL;
        time_t oldest_time = time(NULL);
        my_devNonce_list = mote->devNonce_list;
        while (my_devNonce_list != NULL) {
            if (my_devNonce_list->last_seen < oldest_time) {
                oldest_time = my_devNonce_list->last_seen;
                oldest_devNonce_list = my_devNonce_list;
            }
            my_devNonce_list = my_devNonce_list->next;
        }
        oldest_devNonce_list->value = DevNonce;
        oldest_devNonce_list->last_seen = time(NULL);
    }
    return 0;
}

static void
parse_join_req(mote_t* mote, struct lgw_pkt_rx_s *rx_pkt)
{
    uint32_t calculated_mic;
    uint32_t* rx_mic = (uint32_t*)&rx_pkt->payload[rx_pkt->size-LORA_FRAMEMICBYTES];
    join_req_t* jreq_ptr = (join_req_t*)&rx_pkt->payload[0];

    LoRa_GenerateJoinFrameIntegrityCode(mote->app_key, rx_pkt->payload, rx_pkt->size-LORA_FRAMEMICBYTES, (uint8_t*)&calculated_mic);
    if (calculated_mic != *rx_mic) {
        printf("join_req mic fail: %08x, %08x\n", calculated_mic, *rx_mic);
        return;
    }

    printf("join-DevNonce:%04x ", jreq_ptr->DevNonce);
    if (check_devnonce(mote, jreq_ptr->DevNonce) < 0) {
        printf("devNonce fail\n");
        return;
    }
    
    if (join_ignore_count > 0) {
        /* simulate failed join attempts (force retries) */
        ++current_join_attempt;
        printf("current_join_attempt:%d, ignore:%d {%d} ", current_join_attempt, join_ignore_count, current_join_attempt % join_ignore_count);
        if ((current_join_attempt % join_ignore_count) != 0) {
            printf("block\n");
            return;
        }
            printf("allow\n");
    }

    Rx2ChannelParams_t Rx2Channel = RX_WND_2_CHANNEL;
    SendJoinComplete(jreq_ptr->DevNonce, 0, Rx2Channel.Datarate, 1, mote, rx_pkt);

}


void
lorawan_parse_uplink(struct lgw_pkt_rx_s *p)
{
    struct _mote_list* mote_list_ptr = mote_list;
    mhdr_t *mhdr = (mhdr_t*)&p->payload[0];
    int o;
    printf("lorawan_parse_uplink(,%d):", p->size);

    for (o = 0; o < p->size; o++) {
        printf("%02x ", p->payload[o]);
    }
    printf("\n");

    // FHDR: devAddr, FCtrl, FCnt, FOpts
    // FHDR:     4     1      2     n
    // FPort after FOpts
    if (mhdr->bits.MType == MTYPE_JOIN_REQ) {
        //int i;
        join_req_t* join_req = (join_req_t*)&p->payload[0];
        printf("MTYPE_JOIN_REQ ");

        while (mote_list_ptr != NULL) {
            if ((memcmp(join_req->AppEUI, mote_list_ptr->mote.app_eui, LORA_EUI_LENGTH) == 0) &&
                (memcmp(join_req->DevEUI, mote_list_ptr->mote.dev_eui, LORA_EUI_LENGTH) == 0))
            {
                break;
            }
            mote_list_ptr = mote_list_ptr->next;
        }

        if (mote_list_ptr == NULL) {
            printf("mote not in .json:\n");
            printf("AppEUI:");
            for (o = LORA_EUI_LENGTH-1; o >= 0; o--)
                printf("%02x ", join_req->AppEUI[o]);
            printf("\nDevEUI:");
            for (o = LORA_EUI_LENGTH-1; o >= 0; o--)
                printf("%02x ", join_req->DevEUI[o]);
            printf("\n");
        } else
            parse_join_req(&mote_list_ptr->mote, p);

    } else if (mhdr->bits.MType == MTYPE_UNCONF_UP || mhdr->bits.MType == MTYPE_CONF_UP) {
        fhdr_t *fhdr = (fhdr_t*)&p->payload[1];
        if (mhdr->bits.MType == MTYPE_UNCONF_UP)
            printf("MTYPE_UNCONF_UP\n");
        else if (mhdr->bits.MType == MTYPE_CONF_UP)
            printf("MTYPE_CONF_UP\n");

        printf("devaddr:0x%08x FCtrl.FOptsLen:%d, FCnt:%d\n", fhdr->DevAddr, fhdr->FCtrl.bits.FOptsLen, fhdr->FCnt);

        while (mote_list_ptr != NULL) {
            if (mote_list_ptr->mote.dev_addr == fhdr->DevAddr)
                break;
            mote_list_ptr = mote_list_ptr->next;
        }
        if (mote_list_ptr == NULL) {
            printf("mote:%08x not found\n", fhdr->DevAddr);
            return;
        }
        if (fhdr->FCtrl.bits.FOptsLen > 0) {
            uint8_t* fopts_ptr = &p->payload[sizeof(mhdr_t) + sizeof(fhdr_t)];
            parse_mac_command(p, &mote_list_ptr->mote, fopts_ptr, fhdr->FCtrl.bits.FOptsLen);
        }
        // fport==0: mac command
 
        parse_uplink(&mote_list_ptr->mote, p);
    } else
        printf("%02x mtype:%d", p->payload[0], mhdr->bits.MType);

}

bool inputAvailable()  
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

void
lorawan_kbd_input()
{
    if (inputAvailable()) {
        user_downlink_length = read(STDIN_FILENO, user_downlink, sizeof(user_downlink));
        printf("user_downlink:%d,%s\n", user_downlink_length, user_downlink);
    }
}


#if 0
void
show_motes()
{
    int i;
    struct _mote_list* mote_list_ptr = mote_list;
    printf("\nshow_motes()\n");

    while (mote_list_ptr != NULL) {
        printf("dev_addr:%08x\n", mote_list_ptr->mote.dev_addr);
        printf("network_session_key:");
        for (i = 0; i < 16; i++)
            printf("%02x ", mote_list_ptr->mote.network_session_key[i]);
        printf("\n");
        printf("app_session_key:");
        for (i = 0; i < 16; i++)
            printf("%02x ", mote_list_ptr->mote.app_session_key[i]);
        printf("\n");

        mote_list_ptr = mote_list_ptr->next;
    };
}
#endif /* #if 0 */

bool ishexchar(char ch)
{
    if (ch >= 'a' && ch <= 'f')
        return true;

    if (ch >= 'A' && ch <= 'F')
        return true;

    if (ch >= '0' && ch <= '9')
        return true;

    return false;
}

void
parse_dev_addr(uint32_t* dst, const char* str)
{
    int n;
    uint8_t* u8_ptr = (uint8_t*)dst;

    for (n = 3; n >= 0; n--) {
        int i;
        sscanf(str, "%02x", &i);
        u8_ptr[n] = i;
        str++;
        str++;
        while (!ishexchar(*str))
            str++;
    }
}

void
parse_hex_string(uint8_t* dst, const char* str, uint8_t nbytes)
{
    int n = 0, i;
    while (n < nbytes) {
        sscanf(str, "%02x", &i);
        dst[n++] = i;
        str++;
        str++;
        while (!ishexchar(*str))
            str++;
    }
}

void
parse_hex_string_reverse(uint8_t* dst, const char* str, uint8_t nbytes)
{
    int n = nbytes-1, i;
    while (n >= 0) {
        sscanf(str, "%02x", &i);
        dst[n--] = i;
        str++;
        str++;
        while (!ishexchar(*str))
            str++;
    }
}

int parse_lorawan_configuration(const char * conf_file)
{
    const char conf_obj_name[] = "lorawan";
    JSON_Value *root_val = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
    const char *str; /* pointer to sub-strings in the JSON data */
    struct _mote_list* my_mote_list = NULL;
    unsigned int i;

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing lorawan parameters\n", conf_file, conf_obj_name);
    }

    JSON_Array* mote_array = json_object_get_array(conf_obj, "motes");
    size_t n_motes = json_array_get_count(mote_array);
    printf("n_motes:%lu\n", n_motes);
    for (i = 0; i < n_motes; i++) {
        JSON_Object* mote_obj = json_array_get_object (mote_array, i);

        if (mote_list == NULL) {    // first time
            mote_list = malloc(sizeof(struct _mote_list));
            memset(mote_list, 0, sizeof(struct _mote_list));
            my_mote_list = mote_list;
        } else {
            my_mote_list->next = malloc(sizeof(struct _mote_list));
            my_mote_list = my_mote_list->next;
            memset(my_mote_list, 0, sizeof(struct _mote_list));
        }

        str = json_object_get_string(mote_obj, "dev_addr");
        if (str != NULL)
            parse_dev_addr(&my_mote_list->mote.dev_addr, str);  // ABP
        else
            my_mote_list->mote.dev_addr = DEVADDR_NONE; // OTA: mote will join

        str = json_object_get_string(mote_obj, "app_eui");
        if (str != NULL)
            parse_hex_string_reverse(my_mote_list->mote.app_eui, str, LORA_EUI_LENGTH);

        str = json_object_get_string(mote_obj, "dev_eui");
        if (str != NULL)
            parse_hex_string_reverse(my_mote_list->mote.dev_eui, str, LORA_EUI_LENGTH);

        str = json_object_get_string(mote_obj, "app_key");
        if (str != NULL)
            parse_hex_string(my_mote_list->mote.app_key, str, LORA_CYPHERKEYBYTES);

        str = json_object_get_string(mote_obj, "network_session_key");
        if (str != NULL)
            parse_hex_string(my_mote_list->mote.network_session_key, str, LORA_CYPHERKEYBYTES);

        str = json_object_get_string(mote_obj, "app_session_key");
        if (str != NULL)
            parse_hex_string(my_mote_list->mote.app_session_key, str, LORA_CYPHERKEYBYTES);
    }

    str = json_object_get_string(conf_obj, "network_id");
    if (str != NULL) {
        sscanf(str, "%x", &network_id);
    }

    val = json_object_get_value(conf_obj, "Rx1DrOffset");
    if (val != NULL) {
        Rx1DrOffset = (uint32_t)json_value_get_number(val);
    }

    val = json_object_get_value(conf_obj, "dl_rxwin");
    if (val != NULL) {
        dl_rxwin = (uint32_t)json_value_get_number(val);
    }

    val = json_object_get_value(conf_obj, "join_ignore_count");
    if (val != NULL) {
        join_ignore_count = (uint32_t)json_value_get_number(val);
    }


    //show_motes();
    srand(time(NULL));
    return 0;
}
