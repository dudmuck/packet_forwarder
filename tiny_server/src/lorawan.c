/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>         /* atoi, exit */
#include <unistd.h>
#include <sys/select.h>
#include <time.h>   // for struct timespec
#include <sys/time.h>
#include <math.h>
#include "loragw_hal.h"
#include "lorawan.h"
#include "aes.h"
#include "cmac.h"
#include "parson.h"
#include "trace.h"
#include "lorawan_bands.h"

#define DEFAULT_DOWNLINK_PORT       2

#define JOIN_ACCEPT_DELAY1_us                          5000000
#define JOIN_ACCEPT_DELAY2_us                          6000000
#define RECEIVE_DELAY1_us                              1000000
#define RECEIVE_DELAY2_us                              2000000

#define BEACON_WINDOW_SLOTS                         4096

#define MIN_LORA_PREAMB 6 /* minimum Lora preamble length for this application */
#define STD_LORA_PREAMB 8

#define LORA_ENCRYPTIONBLOCKBYTES       16
#define LORA_AUTHENTICATIONBLOCKBYTES   16
#define LORA_FRAMEMICBYTES              4

#define LORA_MAXFRAMELENGTH             235
#define LORA_MACHEADERLENGTH            1
#define LORA_MINDATAHEADERLENGTH        7
#define LORA_PORTLENGTH                 1
#define LORA_MAXDATABYTES    (LORA_MAXFRAMELENGTH - (LORA_MACHEADERLENGTH + LORA_MINDATAHEADERLENGTH + LORA_PORTLENGTH + LORA_FRAMEMICBYTES)) //excluding port
#define LORA_NETWORKADDRESSBITS         25


#define NULL_SNR        -30

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

#define DEVADDR_NONE        0xffffffff

uint32_t network_id;
uint32_t networkAddress = 0;  // bits 24..0 of DevAddr, for join accept
uint8_t tx_rf_chain;    // written by sx1301 conf
unsigned int join_ignore_count = 0;
unsigned int current_join_attempt = 0;
time_t devnonce_experation = 600; // in seconds
#define DEVNONCE_HISTORY_SIZE   20  /* how many devNonce to keep */

uint8_t user_downlink[128];
uint32_t lgw_trigcnt_at_next_beacon;

mtype_e user_dowlink_mtype = MTYPE_UNCONF_DN;
uint8_t ping_downlink_port = 2; // non-zero default
mote_t* tx_busy_mote = NULL;    // if !NULL, downlink scheduled for this mote

#ifdef ENABLE_CLASS_B
BeaconContext_t BeaconCtx;
int skip_beacon_cnt = 0;
#endif  /* ENABLE_CLASS_B */

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


/*!
 * AES computation context variable
 */
static aes_context AesContext;

void LoRaMacBeaconComputePingOffset( uint64_t beaconTime, uint32_t address, uint16_t pingPeriod, uint16_t *pingOffset )
{
    uint8_t zeroKey[16];
    uint8_t buffer[16];
    uint8_t cipher[16];
    uint32_t result = 0;
    /* Refer to chapter 15.2 of the LoRaWAN specification v1.1. The beacon time
     * GPS time in seconds modulo 2^32
     */
    uint32_t time = ( beaconTime % ( ( ( uint64_t ) 1 ) << 32 ) );

    memset( zeroKey, 0, 16 );
    memset( buffer, 0, 16 );
    memset( cipher, 0, 16 );
    memset( AesContext.ksch, '\0', 240 );

    buffer[0] = ( time ) & 0xFF;
    buffer[1] = ( time >> 8 ) & 0xFF;
    buffer[2] = ( time >> 16 ) & 0xFF;
    buffer[3] = ( time >> 24 ) & 0xFF;

    buffer[4] = ( address ) & 0xFF;
    buffer[5] = ( address >> 8 ) & 0xFF;
    buffer[6] = ( address >> 16 ) & 0xFF;
    buffer[7] = ( address >> 24 ) & 0xFF;

    aes_set_key( zeroKey, 16, &AesContext );
    aes_encrypt( buffer, cipher, &AesContext );

    result = ( ( ( uint32_t ) cipher[0] ) + ( ( ( uint32_t ) cipher[1] ) * 256 ) );

    *pingOffset = ( uint16_t )( result % pingPeriod );
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

static int
_send_downlink(struct lgw_pkt_tx_s* tx_pkt, mote_t* mote)
{
    int i;
    fhdr_t* fhdr = (fhdr_t*)&tx_pkt->payload[1];
    uint8_t* mic_ptr;

#ifdef ENABLE_CLASS_B
    if (beacon_guard) {
        printf("send blocked by beacon_guard\n");
        return -1;
    }
#endif  /* ENABLE_CLASS_B */

    fhdr->DevAddr = mote->dev_addr; // if different address, then multicast
    fhdr->FCnt = mote->FCntDown++;
    // tx_pkt->size must have already FRMPayload length or zero
    tx_pkt->size += LORA_MACHEADERLENGTH + sizeof(fhdr_t) + fhdr->FCtrl.dlBits.FOptsLen;
    mic_ptr = &tx_pkt->payload[tx_pkt->size];
    LoRa_GenerateDataFrameIntegrityCode(mote->network_session_key, tx_pkt->payload, tx_pkt->size, fhdr->DevAddr, false, fhdr->FCnt, mic_ptr);
    tx_pkt->size += LORA_FRAMEMICBYTES;

    tx_pkt->tx_mode = TIMESTAMPED;

    tx_pkt->rf_chain = tx_rf_chain;
    tx_pkt->rf_power = 20;   // TODO
    tx_pkt->modulation = MOD_LORA;
    tx_pkt->coderate = CR_LORA_4_5;
    tx_pkt->invert_pol = true;
    tx_pkt->preamble = STD_LORA_PREAMB;
    tx_pkt->no_crc = true;
    tx_pkt->no_header = false;

    pthread_mutex_lock(&mx_concent);
    i = lgw_send(*tx_pkt);
    pthread_mutex_unlock(&mx_concent);
    if (i == LGW_HAL_ERROR) {
        printf("send_downlink(): lgw_send() failed\n");
        return -1;
    }

    return 0;
}

#ifdef ENABLE_CLASS_B
uint64_t last_beaconTime;

void
lorawan_update_ping_offsets(uint64_t beaconTime)
{
    struct _mote_list* mote_list_ptr = mote_list;

    while (mote_list_ptr != NULL) {
        mote_t* mote = &mote_list_ptr->mote;
        if (mote->class_b_en) {
            LoRaMacBeaconComputePingOffset(
                beaconTime,
                mote->dev_addr,
                mote->ping_period,
                &mote->ping_offset
            );
#if UINTPTR_MAX == 0xffffffff
/* 32-bit */
    printf("ping offset: %llu, %08x, %u, %u\n", beaconTime, mote->dev_addr, mote->ping_period, mote->ping_offset);
#elif UINTPTR_MAX == 0xffffffffffffffff
/* 64-bit */
    printf("ping offset: %lu, %08x, %u, %u\n", beaconTime, mote->dev_addr, mote->ping_period, mote->ping_offset);
#else
/* wtf */
#endif
        }
        mote_list_ptr = mote_list_ptr->next;
    }

    last_beaconTime = beaconTime;
}
#endif  /* ENABLE_CLASS_B */

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

static int
send_downlink_classA(struct lgw_pkt_tx_s* tx_pkt, mote_t* mote, uint32_t rx_count_us)
{
    int ret;
    printf(" send_downlink_classA rx%d\n", dl_rxwin);
    if (dl_rxwin == 1) {
        tx_pkt->count_us = rx_count_us + RECEIVE_DELAY1_us;
    } else if (dl_rxwin == 2) {
        tx_pkt->count_us = rx_count_us + RECEIVE_DELAY2_us;
    }

    ret = _send_downlink(tx_pkt, mote);
    if (ret == 0)
        tx_busy_mote = mote;

    return ret;
}

void put_queue_mac_cmds(mote_t* mote, uint8_t cmd_len, uint8_t* cmd_buf)
{
    int i;
    uint8_t* this_cmd_buf = mote->macCmd_queue[mote->macCmd_queue_in_idx];
    this_cmd_buf[0] = cmd_len;

    printf("put_queue_mac_cmds %u: ", cmd_len);
    for (i = 0; i < cmd_len; i++) {
        this_cmd_buf[i+1] = cmd_buf[i];
        printf("%02x ", cmd_buf[i]);
    }
    printf("\r\n");

    if (++mote->macCmd_queue_in_idx == MAC_CMD_QUEUE_SIZE)
        mote->macCmd_queue_in_idx = 0;

    if (mote->macCmd_queue_in_idx == mote->macCmd_queue_out_idx) {
        printf("macCmd_queue full\n");
        exit(EXIT_FAILURE);
    }
}

void get_queued_mac_cmds(mote_t* mote, struct lgw_pkt_tx_s* tx_pkt)
{
    fhdr_t* tx_fhdr = (fhdr_t*)&tx_pkt->payload[1];
    uint8_t* tx_fopts_ptr;

    tx_fhdr->FCtrl.octet = 0;
    tx_fhdr->FCtrl.dlBits.FOptsLen = 0;

    if (mote->macCmd_queue_out_idx == mote->macCmd_queue_in_idx)
        return; // nothing to put

    tx_fopts_ptr = &tx_pkt->payload[sizeof(mhdr_t) + sizeof(fhdr_t)];
    printf("get_queued_mac_cmds ");

    while (mote->macCmd_queue_out_idx != mote->macCmd_queue_in_idx) {
        int i;
        uint8_t* this_cmd_buf = mote->macCmd_queue[mote->macCmd_queue_out_idx];
        uint8_t cmd_len = this_cmd_buf[0];
        if ((tx_fhdr->FCtrl.dlBits.FOptsLen + cmd_len) > 15)
            break;  // no more room

        printf("%u:[", cmd_len);
        for (i = 0; i < cmd_len; i++) {
            *tx_fopts_ptr++ = this_cmd_buf[i+1];
            tx_fhdr->FCtrl.dlBits.FOptsLen++;
            printf("%02x ", this_cmd_buf[i+1]);
        }
        printf("] ");

        if (++mote->macCmd_queue_out_idx == MAC_CMD_QUEUE_SIZE)
            mote->macCmd_queue_out_idx = 0;
    }
    printf("\n");
}

static void
parse_mac_command(struct lgw_pkt_rx_s *rx_pkt, mote_t* mote, uint8_t* rx_cmd_buf, uint8_t rx_cmd_buf_len)
{
    uint8_t cmd_buf[MAC_CMD_SIZE];
    uint8_t rx_cmd_buf_idx = 0;
    int i;
    printf("rx_mac_command(s):");
    for (i = 0; i < rx_cmd_buf_len; i++)
        printf("%02x ", rx_cmd_buf[i]);
    printf("\n");

    while (rx_cmd_buf_idx < rx_cmd_buf_len) {

        if (mote->session_start) {
            band_parse_start_mac_command(rx_cmd_buf[rx_cmd_buf_idx], mote);
        }

        switch (rx_cmd_buf[rx_cmd_buf_idx++]) {
#ifdef ENABLE_CLASS_B
            float diff;
            uint16_t i_diff;
#endif  /* ENABLE_CLASS_B */
            case MOTE_MAC_LINK_CHECK_REQ:   // 0x02
                printf("MOTE_MAC_LINK_CHECK_REQ\n");
                /* no payload in request */
                cmd_buf[0] = SRV_MAC_LINK_CHECK_ANS;
                cmd_buf[1] = 20;  // db margin above noise floor
                cmd_buf[2] = 1;  // gateway count
                put_queue_mac_cmds(mote, 3, cmd_buf);
                break;
            case MOTE_MAC_LINK_ADR_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("LINK_ADR_ANS status:0x%02x\n", i);
                break;
#ifdef ENABLE_CLASS_B
            case MOTE_MAC_PING_SLOT_INFO_REQ:   // 0x10
                printf("MOTE_MAC_PING_SLOT_INFO_REQ\n");
                 // one payload byte: periocity and datarate
                mote->ping_slot_info.octet = rx_cmd_buf[rx_cmd_buf_idx++];
                mote->class_b_en = true;
                uint32_t ping_nb = 128 / (1 << mote->ping_slot_info.bits.periodicity);
                mote->ping_period = BEACON_WINDOW_SLOTS / ping_nb;
                LoRaMacBeaconComputePingOffset(last_beaconTime, mote->dev_addr, mote->ping_period, &mote->ping_offset);
                cmd_buf[0] = SRV_MAC_PING_SLOT_INFO_ANS;    // 0x10
                put_queue_mac_cmds(mote, 1, cmd_buf);
                break;
            case MOTE_MAC_BEACON_TIMING_REQ:    // 0x12
                /* no payload in request */
                diff = (float)(lgw_trigcnt_at_next_beacon - rx_pkt->count_us) / 30000.0;
                i_diff = (int)floor(diff);
                printf("MOTE_MAC_BEACON_TIMING_REQ slots:%.1f=%.1fms (int:%u,%u)", diff, diff*30.0, i_diff, i_diff*30);
                cmd_buf[0] = SRV_MAC_BEACON_TIMING_ANS;   // 0x12
                cmd_buf[1] = i_diff & 0xff; //lsbyte first byte
                cmd_buf[2] = (i_diff >> 8) & 0xff;
                cmd_buf[3] = 0;   // beacon channel index
                put_queue_mac_cmds(mote, 4, cmd_buf);
                printf("%02x %02x %02x\n", cmd_buf[1], cmd_buf[2], cmd_buf[3]);
                break;
            case MOTE_MAC_PING_SLOT_FREQ_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("PING_SLOT_FREQ_ANS status:0x%02x\n", i);
                break;
            case MOTE_MAC_BEACON_FREQ_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("BEACON_FREQ_ANS status:0x%02x\n", i);
                break;
#endif  /* ENABLE_CLASS_B */
            case MOTE_MAC_RX_PARAM_SETUP_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("RX_PARAM_SETUP_ANS status:0x%02x\n", i);
                break;
            case MOTE_MAC_NEW_CHANNEL_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                printf("NEW_CHANNEL_ANS status:0x%02x\n", i);
                break;
            default:
                printf("[31mTODO mac cmd %02x[0m\n", rx_cmd_buf[rx_cmd_buf_idx-1]);
                return;
        } // ..switch (<mac_command>)
    } // .. while have mac comannds

#ifndef ENABLE_CLASS_B
    (void)rx_pkt;
    (void)mote;
#endif  /* ENABLE_CLASS_B */
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

int8_t
get_dr(uint8_t lgw_datarate, uint8_t lgw_bw)
{
    int dr;

    for (dr = 0; data_rates[dr].bw != 0; dr++)  {
        if (data_rates[dr].bw == lgw_bw && data_rates[dr].bps_sf == lgw_datarate) {
            return dr;
        }
    }

    return -1;  // DR not found
}

static void
adr(mote_t* mote, struct lgw_pkt_rx_s *rx_pkt)
{
    int8_t dr, i, hist_idx;
    bool dr_done = false;
    uint32_t datarate = rx_pkt->datarate;
    uint32_t saved_datarate = datarate;
    int8_t saved_txpwr_idx = mote->txpwr_idx;
    float snr = 0, max_snr = -40, min_snr = 40;

    hist_idx = mote->snr_history_idx;
    for (i = 0; i < SNR_HISTORY_SIZE; i++) {
        if (mote->snr_history[hist_idx] > max_snr)
            max_snr = mote->snr_history[hist_idx];
        if (mote->snr_history[hist_idx] < min_snr)
            min_snr = mote->snr_history[hist_idx];

        snr += mote->snr_history[hist_idx];

        if (hist_idx == 0)
            hist_idx = SNR_HISTORY_SIZE-1;
        else
            hist_idx--;
    }
    snr += rx_pkt->snr;
    snr /= (SNR_HISTORY_SIZE+1);


    /*printf("[33m");
    printf("{%.1f %.1f %.1f} ", min_snr, snr, max_snr);*/
    if ((max_snr - min_snr) > 4.0) {
        /* unstable snr */
        //printf("[0m\n");
        return;
    }
    print_hal_sf(rx_pkt->datarate);

    /* spreading-factor reduction if S/N ratio permits */
    while (!dr_done) {
        if (datarate == DR_LORA_SF8 || datarate == DR_LORA_SF9 || datarate == DR_LORA_SF10 ||
            datarate == DR_LORA_SF11 || datarate == DR_LORA_SF12)
        {
            snr -= 3.0;
            if (datarate == DR_LORA_SF8)
                datarate = DR_LORA_SF7;
            else if (datarate == DR_LORA_SF9)
                datarate = DR_LORA_SF8;
            else if (datarate == DR_LORA_SF10)
                datarate = DR_LORA_SF9;
            else if (datarate == DR_LORA_SF11)
                datarate = DR_LORA_SF10;
            else if (datarate == DR_LORA_SF12)
                datarate = DR_LORA_SF11;

            if (snr < 0)
                dr_done = true;

        } else {
            dr_done = true;
        }
    }
    //printf("->");
    print_hal_sf(rx_pkt->datarate);

    dr = get_dr(datarate, rx_pkt->bandwidth);
    /*if (dr < 0)
        printf("[41m");
    printf("to dr%u ", dr);*/

    /* tx-power reduction if S/N ratio permits */
    while (snr > 3.0) {
        /* txpwr_idx: higher number is lower power */
        if (mote->txpwr_idx >= LORAMAC_MIN_TX_POWER)
            break;
        else {
            mote->txpwr_idx++;
            snr -= 3.0;
        }
    }

    /*printf("(%u %u, %u %u) ",
        saved_datarate,
        datarate,
        saved_txpwr_idx,
        mote->txpwr_idx
    );*/
    if (saved_datarate != datarate || saved_txpwr_idx != mote->txpwr_idx) {
        uint8_t cmd_buf[MAC_CMD_SIZE];
        cmd_buf[0] = SRV_MAC_LINK_ADR_REQ;
        cmd_buf[1] = (dr << 4) | mote->txpwr_idx; /* datarate:hi-nibble txpower:lo-nibble */
#if defined( USE_BAND_915_HYBRID )
        cmd_buf[2] = mote->ChMask[0] & 0xff; /* chmask lo-byte */
        cmd_buf[3] = mote->ChMask[0] >> 8; /* chmask hi-byte */
#elif defined( USE_BAND_915 )
        #error todo_64ch_us915
#else
        cmd_buf[2] = mote->ChMask & 0xff; /* chmask lo-byte */
        cmd_buf[3] = mote->ChMask >> 8; /* chmask hi-byte */
#endif
        cmd_buf[4] = 0x00; /* redundancy: RFU + ChMaskCntl:hi-nibble NbTrans:lo-nibble */
        put_queue_mac_cmds(mote, 5, cmd_buf);
    }

    //printf("[0m\n");
}

static void
parse_uplink(mote_t* mote, struct lgw_pkt_rx_s *rx_pkt)
{
    struct lgw_pkt_tx_s tx_pkt;
    uint8_t decrypted[256];
    uint32_t calculated_mic;
    uint32_t rx_mic;
    mhdr_t *rx_mhdr = (mhdr_t*)&rx_pkt->payload[0];
    fhdr_t *rx_fhdr = (fhdr_t*)&rx_pkt->payload[1];
    fhdr_t* tx_fhdr = (fhdr_t*)&tx_pkt.payload[1];
    int rxofs = sizeof(mhdr_t) + sizeof(fhdr_t) + rx_fhdr->FCtrl.ulBits.FOptsLen;
    int rxFRMPayload_length = 0;
    uint8_t* rxFRMPayload = NULL;
    uint8_t* rx_fport_ptr = NULL;

    printf("rxAck:%u fCtrl:0x%02x rxofs:%d ", rx_fhdr->FCtrl.ulBits.ACK, rx_fhdr->FCtrl.octet, rxofs);
    if ((rx_pkt->size - LORA_FRAMEMICBYTES) > rxofs) {
        rxFRMPayload_length = (rx_pkt->size - LORA_FRAMEMICBYTES) - (rxofs + 1);
        rxFRMPayload = &rx_pkt->payload[rxofs+1];
        rx_fport_ptr = &rx_pkt->payload[rxofs];
        printf("rx_fport:%d rxFRMPayload_length:%d\n", *rx_fport_ptr, rxFRMPayload_length);
        for (rxofs = 0; rxofs < rxFRMPayload_length; rxofs++) {
            printf("%02x ", rxFRMPayload[rxofs]);
        }
        printf("\n");
    } else
        printf("no-payload\n");

    tx_pkt.size = 0;

    if (verbose) {
        int i;
        printf("\nnskey:");
        for (i = 0; i < LORA_CYPHERKEYBYTES; i++)
            printf("%02x ", mote->network_session_key[i]);
        printf("rx_pkt->payload:");
        for (i = 0; i < rx_pkt->size; i++)
            printf("%02x ", rx_pkt->payload[i]);
        printf("\n");
        printf("rx_fhdr->DevAddr:%08x\n", rx_fhdr->DevAddr);
        printf("rx_fhdr->FCnt:%u\n", rx_fhdr->FCnt);
    }

    LoRa_GenerateDataFrameIntegrityCode(mote->network_session_key, rx_pkt->payload, rx_pkt->size-LORA_FRAMEMICBYTES, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, (uint8_t*)&calculated_mic);
    rx_mic = rx_pkt->payload[rx_pkt->size-1] << 24;
    rx_mic += rx_pkt->payload[rx_pkt->size-2] << 16;
    rx_mic += rx_pkt->payload[rx_pkt->size-3] << 8;
    rx_mic += rx_pkt->payload[rx_pkt->size-4];
    if (calculated_mic != rx_mic) {
        printf("[31mgenMic:%08x, rxMic:%08x\n", calculated_mic, rx_mic);
        printf("mic fail[0m\n");
        return;
    }

    if (rx_fport_ptr != NULL && *rx_fport_ptr == 0) {
        /* mac commands are encrypted onto port 0 */
        LoRa_EncryptPayload(mote->network_session_key, rxFRMPayload, rxFRMPayload_length, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, decrypted);
        printf("mac commands encrypted on port 0\n");
        parse_mac_command(rx_pkt, mote, decrypted, rxFRMPayload_length);
    } else {
        if (rx_fhdr->FCtrl.ulBits.FOptsLen > 0) {
            /* mac commands are in header */
            printf("mac commands in header\n");
            rxofs = sizeof(mhdr_t) + sizeof(fhdr_t);
            parse_mac_command(rx_pkt, mote, &rx_pkt->payload[rxofs], rx_fhdr->FCtrl.ulBits.FOptsLen);
        }
        if (rxFRMPayload != NULL) {
            LoRa_EncryptPayload(mote->app_session_key, rxFRMPayload, rxFRMPayload_length, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, decrypted);
            printf("app-decrypt:");
            for (rxofs = 0; rxofs < rxFRMPayload_length; rxofs++) {
                printf("%02x ", decrypted[rxofs]);
            }
            printf("  ");
            for (rxofs = 0; rxofs < rxFRMPayload_length; rxofs++) {
                if (decrypted[rxofs] >= ' ' && decrypted[rxofs] < 0x7f)
                    printf("%c", decrypted[rxofs]);
            }
            printf("\n");
        }
    }

    if (rx_fhdr->FCtrl.ulBits.ADR || mote->force_adr) {
        adr(mote, rx_pkt);
        mote->force_adr = false;
    }

    /* any pending mac commands to send? */
    get_queued_mac_cmds(mote, &tx_pkt);

/*    {
        int n;
        uint8_t* ptr = &tx_pkt.payload[sizeof(mhdr_t) + sizeof(fhdr_t)];
        printf("[36mtx foptslen %u: ", tx_fhdr->FCtrl.dlBits.FOptsLen);
        for (n = 0; n < tx_fhdr->FCtrl.dlBits.FOptsLen; n++) {
            printf("%02x ", *ptr++);
        }
        printf("[0m\n");
    }*/
    if (tx_fhdr->FCtrl.dlBits.FOptsLen > 0 || rx_mhdr->bits.MType == MTYPE_CONF_UP ||
        mote->user_downlink_length > 0 || rx_fhdr->FCtrl.ulBits.ADCACKReq)
    {
        /* something to send via downlink */
        if (rx_mhdr->bits.MType == MTYPE_CONF_UP)
            tx_fhdr->FCtrl.dlBits.ACK = 1;
        else
            tx_fhdr->FCtrl.dlBits.ACK = 0;

        if (mote->user_downlink_length > 0) {
            /* add user payload */
            int txo = sizeof(mhdr_t) + sizeof(fhdr_t) + tx_fhdr->FCtrl.dlBits.FOptsLen;
            uint8_t* tx_fport_ptr = &tx_pkt.payload[txo];
            uint8_t* txFRMPayload = &tx_pkt.payload[txo+1];
            LoRa_EncryptPayload(mote->app_session_key, user_downlink, mote->user_downlink_length, mote->dev_addr, false, mote->FCntDown, txFRMPayload);
            if (rx_fport_ptr != NULL)
                *tx_fport_ptr = *rx_fport_ptr;
            else
                *tx_fport_ptr = DEFAULT_DOWNLINK_PORT;

            tx_pkt.size = tx_fhdr->FCtrl.dlBits.FOptsLen + mote->user_downlink_length + 1; // +1 for fport
            tx_pkt.payload[0] = user_dowlink_mtype << 5; // MHDR
        } else {
            /* downlink not triggered by user_downlink */
            /* downlink triggered by FOpotsLen > 0 or conf_uplink */
            tx_pkt.payload[0] = MTYPE_UNCONF_DN << 5; // MHDR
        }

        band_conv(rx_pkt, &tx_pkt);
        if (send_downlink_classA(&tx_pkt, mote, rx_pkt->count_us) < 0)
            printf("send_downlink_classA() failed\n");
        else
            mote->user_downlink_length = 0;  // mark as sent
    }
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
    memset(aesContext.ksch, '\0', 240);
    aes_set_key(key, LORA_CYPHERKEYBYTES, &aesContext);

    aes_decrypt(input, output, &aesContext);
    if (length >= 16)
        aes_decrypt(input + 16, output + 16, &aesContext);
}


void SendJoinComplete(uint16_t deviceNonce, uint8_t firstReceiveWindowDataRateoffset, uint8_t secondReceiveWindowDataRateNibble, uint8_t classARxWindowDelay_s, mote_t* mote, struct lgw_pkt_rx_s *rx_pkt)
{
    struct lgw_pkt_tx_s tx_pkt;
    uint8_t networkSessionKey[LORA_CYPHERKEYBYTES];
    uint8_t uncyphered[LORA_MAXDATABYTES];
    uint8_t* current = uncyphered;
    uint32_t applicationNonce = rand() & 0xffffff;  // 24bit

    if (tx_busy_mote != NULL) {
        printf("[31mSendJoinComplete(): tx busy[0m\n");
        return;
    }

    GenerateSessionKey(true, mote->app_key, network_id, applicationNonce, deviceNonce, networkSessionKey);
    mote->session_start = true;
    band_init_session(mote);

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

    current = band_cflist(current, mote);

    uint16_t authenticatedBytes = current - uncyphered;
    LoRa_GenerateJoinFrameIntegrityCode(mote->app_key, uncyphered, authenticatedBytes, current);
    print_octets("mic", current, LORA_FRAMEMICBYTES);
    current += LORA_FRAMEMICBYTES;

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
    pthread_mutex_lock(&mx_concent);
    int i = lgw_send(tx_pkt);
    pthread_mutex_unlock(&mx_concent);
    if (i == LGW_HAL_ERROR) {
        printf("[31mSendJoinComplete(): lgw_send() failed[0m\n");
    } else
        tx_busy_mote = mote;

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
    int i;
    uint32_t calculated_mic;
    uint32_t rx_mic;
    join_req_t* jreq_ptr = (join_req_t*)&rx_pkt->payload[0];

    if (verbose) {
        printf("\nappkey:");
        for (i = 0; i < LORA_CYPHERKEYBYTES; i++)
            printf("%02x ", mote->app_key[i]);
        printf("\nrx_pkt->payload:");
        for (i = 0; i < rx_pkt->size; i++)
            printf("%02x ", rx_pkt->payload[i]);
        printf("\n");
    }

    LoRa_GenerateJoinFrameIntegrityCode(mote->app_key, rx_pkt->payload, rx_pkt->size-LORA_FRAMEMICBYTES, (uint8_t*)&calculated_mic);
    rx_mic = rx_pkt->payload[rx_pkt->size-1] << 24;
    rx_mic += rx_pkt->payload[rx_pkt->size-2] << 16;
    rx_mic += rx_pkt->payload[rx_pkt->size-3] << 8;
    rx_mic += rx_pkt->payload[rx_pkt->size-4];
    if (calculated_mic != rx_mic) {
        printf("join_req mic fail: %08x, %08x\n", calculated_mic, rx_mic);
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

    mote->macCmd_queue_in_idx = mote->macCmd_queue_out_idx = 0;
    mote->txpwr_idx = LORAMAC_DEFAULT_TX_POWER;
    band_init_channel_mask(mote);

    /* reset snr history */
    for (i = 0; i < SNR_HISTORY_SIZE; i++)
        mote->snr_history[i] = NULL_SNR;

    Rx2ChannelParams_t Rx2Channel = RX_WND_2_CHANNEL;
    SendJoinComplete(jreq_ptr->DevNonce, 0, Rx2Channel.Datarate, 1, mote, rx_pkt);

}

void
get_soonest_mote(mote_t** m)
{
    struct _mote_list* mote_list_ptr;

    /* get current pingslot */
    struct timespec host_time_now;
    double seconds_since_beacon;
    int16_t now_pingslot;

    if (host_time_at_beacon.tv_sec == 0) {
        //printf("get_soonest_mote(): beacon not sent\n");
        *m = NULL;
        return;
    }
    if (clock_gettime (CLOCK_MONOTONIC, &host_time_now) == -1)
        perror ("clock_gettime");
    seconds_since_beacon = difftimespec(host_time_now, host_time_at_beacon);
    printf("seconds_since_beacon:%.2f\n", seconds_since_beacon);
    now_pingslot = (int16_t)ceil((seconds_since_beacon - 2.12) / 0.03);
    //printf("now_pingslot:%d\r\n", now_pingslot);

    /* update next occurring ping in motes */
    for (mote_list_ptr = mote_list; mote_list_ptr != NULL; mote_list_ptr = mote_list_ptr->next) {
        mote_t* mote = &mote_list_ptr->mote;
        if (mote->class_b_en) {
            printf("class_b mote %08x ", mote->dev_addr);
            if (mote->ping_offset > 0) {
                uint16_t _use_pingslot = mote->ping_offset;
                printf("ping_offset:%u ",  mote->ping_offset);
                while (_use_pingslot < now_pingslot)
                    _use_pingslot += mote->ping_period;
                printf(" use_pingslot:%u", _use_pingslot);
                mote->next_occurring_ping = _use_pingslot;
            } else
                printf("ping_offset==0");
            printf("\n");
        }
    }

    /* find class-B mote with soonest occurring ping slot */
    uint16_t soonest_occurring_ping = 4096;
    *m = NULL;
    for (mote_list_ptr = mote_list; mote_list_ptr != NULL; mote_list_ptr = mote_list_ptr->next) {
        mote_t* mote = &mote_list_ptr->mote;
        if (mote->class_b_en && mote->user_downlink_length != 0 && mote->next_occurring_ping < soonest_occurring_ping)  {
            *m = &mote_list_ptr->mote;
            soonest_occurring_ping = mote->next_occurring_ping;
            //printf("soonest mote %08x: %u\r\n", (*m)->dev_addr, soonest_occurring_ping);
        }
        
    }
}

void
send_downlink_ping(mote_t *m)
{
    /*** prepare downlink tx ***/
    struct lgw_pkt_tx_s ping_tx_pkt;
    ping_tx_pkt.freq_hz = PINGSLOT_CHANNEL_FREQ(0); // TODO use address in idle state
    ping_tx_pkt.bandwidth = data_rates[PING_SLOT_DATARATE].bw;
    ping_tx_pkt.datarate = data_rates[PING_SLOT_DATARATE].bps_sf;
    ping_tx_pkt.payload[0] = user_dowlink_mtype << 5; // MHDR
    fhdr_t* tx_fhdr = (fhdr_t*)&ping_tx_pkt.payload[1];

    get_queued_mac_cmds(m, &ping_tx_pkt);
    ping_tx_pkt.size = tx_fhdr->FCtrl.dlBits.FOptsLen + m->user_downlink_length + 1; // +1 for fport

    int txo = sizeof(mhdr_t) + sizeof(fhdr_t) + tx_fhdr->FCtrl.dlBits.FOptsLen;
    uint8_t* tx_fport_ptr = &ping_tx_pkt.payload[txo];
    uint8_t* txFRMPayload = &ping_tx_pkt.payload[txo+1];

    LoRa_EncryptPayload(
        /* const uint8_t key[] */ m->app_session_key,
        /* uint8_t const in[] */ user_downlink,
        /* uint16_t inputDataLength */ m->user_downlink_length,
        /* uint32_t address */ m->dev_addr,
        /* bool up */ false,
        /* uint32_t sequenceNumber */ m->FCntDown,
        /* uint8_t out[] */ txFRMPayload
    );

    *tx_fport_ptr = ping_downlink_port;
    uint32_t ping_offset_us = m->next_occurring_ping * 30000;
    float ping_offset_s = m->next_occurring_ping * 0.03;
    ping_tx_pkt.count_us = trigcnt_pingslot_zero;
    ping_tx_pkt.count_us += ping_offset_us + (g_sx1301_ppm_err * ping_offset_s);
    printf("use_pingslot:%u, ping_offset_us:%u, ping_offset_s:%f\n", m->next_occurring_ping, ping_offset_us, ping_offset_s);
    //printf("err_us:%d
    if (m->next_occurring_ping >= BEACON_WINDOW_SLOTS) {
        /* cant be sent in this beacon period */
        printf("send in next beacon window\n");
        return;
    }

    if (_send_downlink(&ping_tx_pkt, m) == 0) {
        tx_busy_mote = m;
    } else
        printf("ping _send_downlink() failed\n");
}

void
lorawan_parse_uplink(struct lgw_pkt_rx_s *p)
{
    struct _mote_list* mote_list_ptr = mote_list;
    mhdr_t *mhdr = (mhdr_t*)&p->payload[0];
    int o;
    printf("lorawan_parse_uplink(,%d) %u:", p->size, p->count_us);

    if (p->size <= (sizeof(fhdr_t) + LORA_FRAMEMICBYTES)) {
        printf("too small\n");
        return;
    }
    if (mhdr->bits.major != 0) {
        printf("unsupported major:%u\n", mhdr->bits.major);
        return;
    }
#ifdef ENABLE_CLASS_B
    if (beacon_guard) {
        printf("beacon_guard\n");
        return; /* drop uplinks during beacon guard period */
    }
#endif  /* ENABLE_CLASS_B */

/*    raw-rf payload print:
    for (o = 0; o < p->size; o++) {
        printf("%02x ", p->payload[o]);
    }
    printf("\n");*/

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
            printf("unknown mote:\n");
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
            printf("MTYPE_UNCONF_UP\nup-");
        else if (mhdr->bits.MType == MTYPE_CONF_UP)
            printf("MTYPE_CONF_UP\nup-");

        if (fhdr->FCtrl.ulBits.classB)
            printf("classB ");
        else
            printf("classA ");
        printf("devaddr:0x%08x FCtrl.FOptsLen:%d, FCnt:%d\n", fhdr->DevAddr, fhdr->FCtrl.ulBits.FOptsLen, fhdr->FCnt);

        while (mote_list_ptr != NULL) {
            if (mote_list_ptr->mote.dev_addr == fhdr->DevAddr)
                break;
            mote_list_ptr = mote_list_ptr->next;
        }
        if (mote_list_ptr == NULL) {
            printf("mote:%08x not found\n", fhdr->DevAddr);
            return;
        }
        mote_list_ptr->mote.class_b_en = fhdr->FCtrl.ulBits.classB;
 
        parse_uplink(&mote_list_ptr->mote, p);
    } else
        printf(" [31m%02x mtype:%d[0m\n", p->payload[0], mhdr->bits.MType);


    if (mote_list_ptr != NULL) {
        mote_list_ptr->mote.snr_history[mote_list_ptr->mote.snr_history_idx] = p->snr;
        if (++mote_list_ptr->mote.snr_history_idx == SNR_HISTORY_SIZE)
            mote_list_ptr->mote.snr_history_idx = 0;
    }
}

// return: 1=cmd parsed, 0=user payload
static int cmd_parse(uint8_t const* cmd_buf, unsigned int cmd_buf_len)
{
    //printf("%c, len:%d\n", cmd_buf[0], cmd_buf_len);
    if (cmd_buf_len == 1) {
        switch (cmd_buf[0]) {
            case 'C':
                printf("Confirmed\n");
                user_dowlink_mtype = MTYPE_CONF_DN;
                return 1;
            case 'U':
                printf("Unconfirmed\n");
                user_dowlink_mtype = MTYPE_UNCONF_DN;
                return 1;
            case '.':
                if (user_dowlink_mtype == MTYPE_UNCONF_DN)
                    printf("user_dowlink_mtype:MTYPE_UNCONF_DN\n");
                else if (user_dowlink_mtype == MTYPE_CONF_DN)
                    printf("user_dowlink_mtype:MTYPE_CONF_DN\n");
                return 1;
            case '?':
                printf("C       set ping dowlink to Confirmed\n");
                printf("U       set ping dowlink to Unconfirmed\n");
                printf("sb%%d       skip sending N beacons\n");
                return 1;
            default: return 0;
        }
    } else if (cmd_buf[0] == 's' && cmd_buf[1] == 'b') {
        int n = sscanf((char*)cmd_buf+2, "%d", &skip_beacon_cnt);
        printf("%d = sscanf() skip_beacon_cnt:%d\n", n, skip_beacon_cnt);
        return 1;
    }
    return 0;
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

void _print_tx_status(uint8_t tx_status)
{
    printf("TX_STATUS: ");
    switch (tx_status) {
        case TX_OFF:
            printf("TX_OFF\n");
            break;
        case TX_FREE:
            printf("TX_FREE\n");
            if (tx_busy_mote != NULL) {
                mote_t* soonest_mote;
                tx_busy_mote->user_downlink_length = 0; // flag as tx complete
                tx_busy_mote = NULL;
                /* is there another class-b mote with downlink to send? */
                get_soonest_mote(&soonest_mote);
                if (soonest_mote != NULL)
                    send_downlink_ping(soonest_mote);
            }
            break;
        case TX_EMITTING:
            printf("TX_EMITTING\n");
            break;
        case TX_SCHEDULED:
            printf("TX_SCHEDULED\n");
            break;
        default:
            printf("lgw_status returned UNKNOWN (%d)\n", tx_status);
            break;
    }
}

void
lorawan_kbd_input()
{
    int stdin_len;
    uint8_t tx_status;
    static uint8_t prev_tx_status = TX_OFF;
#ifdef ENABLE_CLASS_B
    //uint32_t trig_tstamp;
#endif    /* ENABLE_CLASS_B */

    lgw_status(TX_STATUS, &tx_status);
    if (prev_tx_status != tx_status) {
        _print_tx_status(tx_status);
        prev_tx_status = tx_status;
    }
    
    if (!inputAvailable())
        return;

    stdin_len = read(STDIN_FILENO, user_downlink, sizeof(user_downlink));
    printf("user_downlink:%d,%s\n", stdin_len, user_downlink);

    if (cmd_parse(user_downlink, stdin_len-1))
        return; // user command, not payload

#ifdef ENABLE_CLASS_B
    struct _mote_list* mote_list_ptr;
    //send_ping_single_mote();

    if (tx_busy_mote != NULL) {
        printf("tx busy\n");
        return;
    }

    /* flag downlink into all class_b motes */
    for (mote_list_ptr = mote_list; mote_list_ptr != NULL; mote_list_ptr = mote_list_ptr->next) {
        mote_t* mote = &mote_list_ptr->mote;
        if (mote->class_b_en)
            mote->user_downlink_length = stdin_len;
    }


    mote_t* soonest_mote = NULL;
    get_soonest_mote(&soonest_mote);

    if (soonest_mote == NULL) {
        printf("no mote to ping\r\n");
        return;
    }

    //printf("soonest mote:%08x, ping at %u\r\n", soonest_mote->dev_addr, soonest_mote->next_occurring_ping);

    send_downlink_ping(soonest_mote);
#endif    /* ENABLE_CLASS_B */
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
    unsigned int n_motes = json_array_get_count(mote_array);
    printf("n_motes:%u\n", n_motes);
    for (i = 0; i < n_motes; i++) {
        int h;
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

        my_mote_list->mote.txpwr_idx = LORAMAC_DEFAULT_TX_POWER;
        band_init_channel_mask(&my_mote_list->mote);
        for (h = 0; h < SNR_HISTORY_SIZE; h++)
            my_mote_list->mote.snr_history[h] = NULL_SNR;

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

    val = json_object_get_value(conf_obj, "ping_downlink_port");
    if (val != NULL) {
        ping_downlink_port = (uint32_t)json_value_get_number(val);
    }

    //show_motes();
    srand(time(NULL));
    return 0;
}
