#ifndef _LORAWAN_H_
#define _LORAWAN_H_

#include <pthread.h>
#include "loragw_hal.h"

extern pthread_mutex_t mx_concent; /* control access to the concentrator */

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
#ifdef ENABLE_CLASS_B
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
#endif  /* ENABLE_CLASS_B */
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
#ifdef ENABLE_CLASS_B
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
#endif  /* ENABLE_CLASS_B */
}LoRaMacSrvCmd_t;

typedef struct sBeaconContext
{
    struct sBeaconCfg {
        uint32_t Interval;
    } Cfg;
    uint32_t BeaconTime;
} BeaconContext_t;

typedef union {
    struct {
        uint8_t major   : 2;    // 0 1
        uint8_t rfu     : 3;    // 2 3 4
        uint8_t MType   : 3;    // 5 6 7
    } bits;
    uint8_t octet;
} mhdr_t;

typedef union {
    struct {
        uint8_t FOptsLen        : 4;    // 0 1 2 3
        uint8_t FPending        : 1;    // 4 
        uint8_t ACK             : 1;    // 5
        uint8_t ADCACKReq       : 1;    // 6
        uint8_t ADR             : 1;    // 7
    } dlBits;   // downlink  (gwtx)
    struct {
        uint8_t FOptsLen        : 4;    // 0 1 2 3
        uint8_t classB          : 1;    // 4    unused in classA
        uint8_t ACK             : 1;    // 5
        uint8_t ADCACKReq       : 1;    // 6
        uint8_t ADR             : 1;    // 7
    } ulBits;   // uplink   (gwrx)
    uint8_t octet;
} FCtrl_t;

typedef struct {
    uint32_t DevAddr;
    FCtrl_t FCtrl;
    uint16_t FCnt;
} __attribute__((packed)) fhdr_t;

#define MAC_CMD_QUEUE_SIZE      6
#define MAC_CMD_SIZE            8

#define SNR_HISTORY_SIZE        3

#define LORA_EUI_LENGTH                 8
#define LORA_CYPHERKEYBYTES             16
typedef struct {
    uint8_t app_eui[LORA_EUI_LENGTH];
    uint8_t dev_eui[LORA_EUI_LENGTH];
    uint8_t app_key[LORA_CYPHERKEYBYTES];

    bool session_start;
    bool force_adr;

    uint8_t macCmd_queue[MAC_CMD_QUEUE_SIZE][MAC_CMD_SIZE];
    uint8_t macCmd_queue_in_idx, macCmd_queue_out_idx;

    float snr_history[SNR_HISTORY_SIZE];
    uint8_t snr_history_idx;

    uint32_t dev_addr;
    uint8_t network_session_key[LORA_CYPHERKEYBYTES];
    uint8_t app_session_key[LORA_CYPHERKEYBYTES];

    uint16_t FCntDown;

    uint8_t txpwr_idx;
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    uint16_t ChMask[6];
#else
    uint16_t ChMask;
#endif

    union {
        struct {
            uint8_t periodicity: 3;    // 0 to 2
            uint8_t rfu: 5;    // 3 to 7
        } bits;
        uint8_t octet;
    } ping_slot_info;
    bool class_b_en;
    uint16_t ping_offset;
    uint16_t ping_period;
    uint16_t next_occurring_ping;

    uint8_t user_downlink_length;   // 0 = no downlink to be sent

    struct _devNonce_list* devNonce_list;
} mote_t;

extern BeaconContext_t BeaconCtx;

void print_hal_bw(uint8_t bandwidth);
void print_hal_sf(uint8_t datarate);
void lorawan_kbd_input(void);
void lorawan_parse_uplink(struct lgw_pkt_rx_s *p);
int parse_lorawan_configuration(const char * conf_file);

extern uint8_t tx_rf_chain;    // written by sx1301 conf

#ifdef ENABLE_CLASS_B
void lorawan_update_ping_offsets(uint64_t beaconTime);
extern uint32_t lgw_trigcnt_at_next_beacon;
extern float g_sx1301_ppm_err;  // from tiny server
extern uint32_t trigcnt_pingslot_zero;  // from tiny server
extern bool beacon_guard;  // from tiny server
extern int skip_beacon_cnt;
#endif	/* ENABLE_CLASS_B */

extern bool verbose;    // from tiny server
extern struct timespec host_time_at_beacon;    // from tiny server


double difftimespec(struct timespec end, struct timespec beginning);  // from tiny server

/*!
 * Computes the LoRaMAC join frame decryption
 *
 * \param [IN]  beaconTime      - Time of the recent received beacon
 * \param [IN]  address         - Frame address
 * \param [IN]  pingPeriod      - Ping period of the node
 * \param [OUT] pingOffset      - Pseudo random ping offset
 */
void LoRaMacBeaconComputePingOffset( uint64_t beaconTime, uint32_t address, uint16_t pingPeriod, uint16_t *pingOffset );

uint8_t* Write4ByteValue(uint8_t output[], uint32_t input);
uint8_t* Write3ByteValue(uint8_t output[], uint32_t input);
uint8_t* Write2ByteValue(uint8_t output[], uint32_t input);
uint8_t* Write1ByteValue(uint8_t output[], uint32_t input);

void put_queue_mac_cmds(mote_t* mote, uint8_t cmd_len, uint8_t* cmd_buf);

#endif /* _LORAWAN_H_ */
