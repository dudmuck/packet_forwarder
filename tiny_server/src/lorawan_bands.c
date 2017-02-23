/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "loragw_hal.h"
#include "lorawan_bands.h"


typedef union uDrRange
{
    /*!
     * Byte-access to the bits
     */
    int8_t Value;
    /*!
     * Structure to store the minimum and the maximum datarate
     */
    struct sFields
    {
         /*!
         * Minimum data rate
         *
         * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
         *
         * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4]
         */
        int8_t Min : 4;
        /*!
         * Maximum data rate
         *
         * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
         *
         * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4]
         */
        int8_t Max : 4;
    }Fields;
}DrRange_t;

typedef struct sChannelParams
{
    /*!
     * Frequency in Hz
     */
    uint32_t Frequency;
    /*!
     * Data rate definition
     */
    DrRange_t DrRange;
    /*!
     * Band index
     */
    uint8_t Band;
}ChannelParams_t;

uint8_t Rx1DrOffset;
uint8_t dl_rxwin = 1;

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

uint8_t BeaconChannel( uint32_t devAddr )
{
    uint32_t frequency = 0;

    if (BeaconCtx.BeaconTime == 0) {
        printf("BeaconCtx.BeaconTime == 0\n");
        return 0;
    }
    if (BeaconCtx.Cfg.Interval == 0) {
        printf("BeaconCtx.Cfg.Interval == 0\n");
        return 0;
    }

    frequency = devAddr + ( BeaconCtx.BeaconTime / ( BeaconCtx.Cfg.Interval / 1000 ) );
    printf("[31mBeaconChannel() frequency:%u, devAddr:%08x, BeaconCtx.BeaconTime:%x, BeaconCtx.Cfg.Interval:%u[0m\n", frequency, devAddr, BeaconCtx.BeaconTime, BeaconCtx.Cfg.Interval);
    return ( ( uint8_t )( frequency % 8 ) );
}

#define LORAMAC_FIRST_RX1_CHANNEL   923300000
#define LORAMAC_STEPWIDTH_RX1_CHANNEL   600000

const data_rate_t data_rates[] = {
    { BW_125KHZ, DR_LORA_SF10 },    // DR0
    { BW_125KHZ, DR_LORA_SF9 },    // DR1
    { BW_125KHZ, DR_LORA_SF8 },    // DR2
    { BW_125KHZ, DR_LORA_SF7 },    // DR3
    { BW_500KHZ, DR_LORA_SF8 },    // DR4
    { BW_UNDEFINED, DR_UNDEFINED },// DR5
    { BW_UNDEFINED, DR_UNDEFINED },// DR6
    { BW_UNDEFINED, DR_UNDEFINED },// DR7
    { BW_500KHZ, DR_LORA_SF12 },    // DR8
    { BW_500KHZ, DR_LORA_SF11 },    // DR9
    { BW_500KHZ, DR_LORA_SF10 },    // DR10
    { BW_500KHZ, DR_LORA_SF9 },    // DR11
    { BW_500KHZ, DR_LORA_SF8 },    // DR12
    { BW_500KHZ, DR_LORA_SF7 },    // DR13
    { BW_UNDEFINED, DR_UNDEFINED },// DR14
    { BW_UNDEFINED, DR_UNDEFINED } // DR15
};

const int8_t datarateOffsets[5][4] =
{
    { DR_10, DR_9 , DR_8 , DR_8  }, // DR_0
    { DR_11, DR_10, DR_9 , DR_8  }, // DR_1
    { DR_12, DR_11, DR_10, DR_9  }, // DR_2
    { DR_13, DR_12, DR_11, DR_10 }, // DR_3
    { DR_13, DR_13, DR_12, DR_11 }, // DR_4
};

void
get_rx2_config(struct lgw_pkt_tx_s* tx_pkt) // us915
{
    Rx2ChannelParams_t Rx2Channel = RX_WND_2_CHANNEL;

    tx_pkt->bandwidth = BW_500KHZ;
    tx_pkt->freq_hz = Rx2Channel.Frequency;
    switch (Rx2Channel.Datarate) {
        case DR_8 : tx_pkt->datarate = DR_LORA_SF12; break;
        case DR_9 : tx_pkt->datarate = DR_LORA_SF11; break;
        case DR_10: tx_pkt->datarate = DR_LORA_SF10; break;
        case DR_11: tx_pkt->datarate = DR_LORA_SF9; break;
        case DR_12: tx_pkt->datarate = DR_LORA_SF8; break;
        case DR_13: tx_pkt->datarate = DR_LORA_SF7; break;
    }
}

void band_conv(struct lgw_pkt_rx_s* rx_pkt, struct lgw_pkt_tx_s* tx_pkt)
{
    int hz;
    float fch, fraction;
    int ich;
    int dl_dr = -1, upl_dr = -1;

    printf("us915-band_conv(): rx:%uhz ", rx_pkt->freq_hz);
    if (rx_pkt->bandwidth == BW_500KHZ) {
        hz = rx_pkt->freq_hz - 903000000;
        //printf("500khz:%dhz ", hz);
        fch = 64 + (hz / 1600000.0);  // stepping
        //printf("ch:%.1f ", fch);
    } else if (rx_pkt->bandwidth == BW_125KHZ) {
        hz = rx_pkt->freq_hz - 902300000;
        //printf("125khz:%dhz ", hz);
        fch = hz / 200000.0;  // stepping
        //printf("ch:%.1f ", fch);
    } else {
        return;
    }

    fraction = fmodf(fch, 1.0);
    //printf("fraction:%f ", fraction);
    if (fraction >= 0.5)
        ich = ceil(fch);
    else
        ich = floor(fch);

    tx_pkt->bandwidth = BW_500KHZ;

    if (dl_rxwin == 1) {
        tx_pkt->freq_hz = LORAMAC_FIRST_RX1_CHANNEL + (ich % 8) * LORAMAC_STEPWIDTH_RX1_CHANNEL;
        /* get uplink datarate */
        if (rx_pkt->bandwidth == BW_500KHZ) {
            if (rx_pkt->datarate == DR_LORA_SF8)
                upl_dr = 4;
        } else if (rx_pkt->bandwidth == BW_125KHZ) {
            switch (rx_pkt->datarate) {
                case DR_LORA_SF10: upl_dr = 0; break;
                case DR_LORA_SF9: upl_dr = 1; break;
                case DR_LORA_SF8: upl_dr = 2; break;
                case DR_LORA_SF7: upl_dr = 3; break;
            }
        }
        dl_dr = datarateOffsets[upl_dr][Rx1DrOffset];
        printf("upl_dr%d, Rx1DrOffset:%d -> dl_dr%d\n", upl_dr, Rx1DrOffset, dl_dr);
        switch (dl_dr) {
            case DR_8 : tx_pkt->datarate = DR_LORA_SF12; break;
            case DR_9 : tx_pkt->datarate = DR_LORA_SF11; break;
            case DR_10: tx_pkt->datarate = DR_LORA_SF10; break;
            case DR_11: tx_pkt->datarate = DR_LORA_SF9; break;
            case DR_12: tx_pkt->datarate = DR_LORA_SF8; break;
            case DR_13: tx_pkt->datarate = DR_LORA_SF7; break;
        }
    } else if (dl_rxwin == 2) {
        get_rx2_config(tx_pkt);
    }
}

int check_band_config(uint32_t cf)
{
    if (cf < 902000000 || cf > 928000000) {
        printf("USE_BAND_915 rx cf %uhz out of range\n", cf);
        return -1;
    }
    return 0;
}

void band_parse_start_mac_command(uint8_t cmd, mote_t* mote)
{
    (void)cmd;
    (void)mote;
}

void band_init_session(mote_t* mote)
{
    (void)mote;
}

uint8_t* band_cflist(uint8_t* ptr, mote_t* mote)
{
    (void)mote;
    return ptr;
}

void band_init_channel_mask(mote_t* mote)
{
#if defined( USE_BAND_915 )
    mote->ChMask[0] = 0xFFFF;
    mote->ChMask[1] = 0xFFFF;
    mote->ChMask[2] = 0xFFFF;
    mote->ChMask[3] = 0xFFFF;
    mote->ChMask[4] = 0x00FF;
    mote->ChMask[5] = 0x0000;
#elif defined( USE_BAND_915_HYBRID )
    mote->ChMask[0] = 0x00FF;
    mote->ChMask[1] = 0x0000;
    mote->ChMask[2] = 0x0000;
    mote->ChMask[3] = 0x0000;
    mote->ChMask[4] = 0x0001;
    mote->ChMask[5] = 0x0000;
#endif
}

/* end USE_BAND_915 */
#elif defined(USE_BAND_868)

const data_rate_t data_rates[] = {
    { BW_125KHZ, DR_LORA_SF12 },    // DR0
    { BW_125KHZ, DR_LORA_SF11 },    // DR1
    { BW_125KHZ, DR_LORA_SF10 },    // DR2
    { BW_125KHZ, DR_LORA_SF9 },    // DR3
    { BW_125KHZ, DR_LORA_SF8 },    // DR4
    { BW_125KHZ, DR_LORA_SF7 },    // DR5
    { BW_250KHZ, DR_LORA_SF7 },    // DR6
    { 0, 0},    // DR7  TODO FSK
};

// Channel = { Frequency [Hz], { ( ( DrMax << 4 ) | DrMin ) }, Band }
#define LC1                { 868100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 1 }
#define LC2                { 868300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 1 }
#define LC3                { 868500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 1 }

#define LORA_MAX_NB_CHANNELS                        16

// Channel = { Frequency [Hz], Datarate }

static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS] =
{
    LC1,
    LC2,
    LC3,
};

void
get_rx2_config(struct lgw_pkt_tx_s* tx_pkt) // eu868
{
    Rx2ChannelParams_t Rx2Channel = RX_WND_2_CHANNEL;

    tx_pkt->freq_hz = Rx2Channel.Frequency;
    tx_pkt->modulation = MOD_LORA;
    tx_pkt->bandwidth = BW_125KHZ;
    switch (Rx2Channel.Datarate) {
        case DR_0: tx_pkt->datarate = DR_LORA_SF12; break;
        case DR_1: tx_pkt->datarate = DR_LORA_SF11; break;
        case DR_2: tx_pkt->datarate = DR_LORA_SF10; break;
        case DR_3: tx_pkt->datarate = DR_LORA_SF9; break;
        case DR_4: tx_pkt->datarate = DR_LORA_SF8; break;
        case DR_5: tx_pkt->datarate = DR_LORA_SF7; break;
        case DR_6:
            tx_pkt->datarate = DR_LORA_SF7;
            tx_pkt->bandwidth = BW_250KHZ;
            break;
        case DR_7:
            tx_pkt->modulation = MOD_FSK;
            tx_pkt->datarate = 9999;//TODO
            tx_pkt->f_dev = 50;//TODO
            break;
    }
}

void band_conv(struct lgw_pkt_rx_s* rx_pkt, struct lgw_pkt_tx_s* tx_pkt)
{
    printf("eu868-band_conv(): rx:%uhz ", rx_pkt->freq_hz);
    //uint32_t tol_hz;
    int i/*, min_i*/, min = Channels[0].Frequency;

    if (rx_pkt->bandwidth == BW_125KHZ) {
        //tol_hz = 125000 / 4;
    } else if (rx_pkt->bandwidth == BW_250KHZ) {
        //tol_hz = 250000 / 4;
    }

    for (i = 0; i < LORA_MAX_NB_CHANNELS; i++) {
        int diff_hz = abs(rx_pkt->freq_hz - Channels[i].Frequency);
        if (diff_hz < min) {
            //min_i = i;
            min = diff_hz;
        }
    }

    if (dl_rxwin == 1) {
        tx_pkt->freq_hz = rx_pkt->freq_hz;
        tx_pkt->datarate = rx_pkt->datarate;
        tx_pkt->bandwidth = rx_pkt->bandwidth;
    } else if (dl_rxwin == 2) {
        get_rx2_config(tx_pkt);
    }

    printf("\n");
}

int check_band_config(uint32_t cf)
{
    if (cf < 865000000 || cf > 878000000) {
        printf("USE_BAND_868 rx cf %uhz out of range\n", cf);
        return -1;
    }
    return 0;
}

void band_parse_start_mac_command(uint8_t cmd, mote_t* mote)
{
    (void)cmd;
    (void)mote;
}

void band_init_session(mote_t* mote)
{
    (void)mote;
}

uint8_t* band_cflist(uint8_t* ptr, mote_t* mote)
{
    (void)mote;
    return ptr;
}

void band_init_channel_mask(mote_t* mote)
{
    mote->ChMask = 0x0007;  /* LC1, LC2, LC3 */
}

/* end USE_BAND_868 */
#elif defined(USE_BAND_ARIB_8CH)

const data_rate_t data_rates[] = {
    { BW_125KHZ, DR_LORA_SF12 },    // DR0
    { BW_125KHZ, DR_LORA_SF11 },    // DR1
    { BW_125KHZ, DR_LORA_SF10 },    // DR2
    { BW_125KHZ, DR_LORA_SF9 },    // DR3
    { BW_125KHZ, DR_LORA_SF8 },    // DR4
    { BW_125KHZ, DR_LORA_SF7 },    // DR5
    { BW_250KHZ, DR_LORA_SF7 },    // DR6
    { 0, 0},    // DR7  TODO FSK
};

void
get_rx2_config(struct lgw_pkt_tx_s* tx_pkt)     // arb-actility
{
    Rx2ChannelParams_t Rx2Channel = RX_WND_2_CHANNEL;
    /* TODO: join accept Rx2 */

    tx_pkt->freq_hz = Rx2Channel.Frequency;
    tx_pkt->modulation = MOD_LORA;
    tx_pkt->bandwidth = BW_125KHZ;
    switch (Rx2Channel.Datarate) {
        case DR_0: tx_pkt->datarate = DR_LORA_SF12; break;
        case DR_1: tx_pkt->datarate = DR_LORA_SF11; break;
        case DR_2: tx_pkt->datarate = DR_LORA_SF10; break;
        case DR_3: tx_pkt->datarate = DR_LORA_SF9; break;
        case DR_4: tx_pkt->datarate = DR_LORA_SF8; break;
        case DR_5: tx_pkt->datarate = DR_LORA_SF7; break;
        case DR_6:
            tx_pkt->datarate = DR_LORA_SF7;
            tx_pkt->bandwidth = BW_250KHZ;
            break;
        case DR_7:
            tx_pkt->modulation = MOD_FSK;
            tx_pkt->datarate = 9999;//TODO
            tx_pkt->f_dev = 50;//TODO
            break;
    }
}

void band_conv(struct lgw_pkt_rx_s* rx_pkt, struct lgw_pkt_tx_s* tx_pkt)
{
    printf("arib-band_conv(): rx:%uhz ", rx_pkt->freq_hz);
    if (dl_rxwin == 1) {
        tx_pkt->freq_hz = rx_pkt->freq_hz;
        tx_pkt->datarate = rx_pkt->datarate;
        tx_pkt->bandwidth = rx_pkt->bandwidth;
    } else if (dl_rxwin == 2) {
        get_rx2_config(tx_pkt);
    }
}

int check_band_config(uint32_t cf)
{
    if (cf < 920600000 || cf > 923600000) {
        printf("ARIB_8CH rx cf %uhz out of range\n", cf);
        return -1;
    }
    return 0;
}

uint8_t* band_cflist(uint8_t* ptr, mote_t* mote)
{
    uint32_t freq;
    /*
     * LoRaMacChannelAdd(3) 921800000hz dr:0x53 add ch3 mask:000b
     * LoRaMacChannelAdd(4) 921600000hz dr:0x53 add ch4 mask:001b
     * LoRaMacChannelAdd(5) 921400000hz dr:0x53 add ch5 mask:003b*/

    freq = 921800000 / 100; // ch4
    ptr = Write3ByteValue(ptr, freq);

    freq = 921600000 / 100; // ch5
    ptr = Write3ByteValue(ptr, freq);

    freq = 921400000 / 100; // ch6
    ptr = Write3ByteValue(ptr, freq);

    freq = 0;   // ch7
    ptr = Write3ByteValue(ptr, freq);

    freq = 0;   // ch8
    ptr = Write3ByteValue(ptr, freq);

    ptr = Write1ByteValue(ptr, 0);  // CFListType

    /* sending these 3 channels enabled 3 bits in channel mask */
    mote->ChMask |= 0x0038;

    return ptr;
}

void band_parse_start_mac_command(uint8_t cmd, mote_t* mote)
{
    uint8_t cmd_buf[MAC_CMD_SIZE];
    uint32_t freq;

    switch (cmd) {
        case MOTE_MAC_RX_PARAM_SETUP_ANS:
            freq = 922000000 / 100;

            cmd_buf[0] = SRV_MAC_NEW_CHANNEL_REQ;
            cmd_buf[1] = 2; // channel index 
            cmd_buf[2] = freq & 0xff;
            cmd_buf[3] = (freq >> 8) & 0xff;
            cmd_buf[4] = (freq >> 16) & 0xff;
            cmd_buf[5] = (DR_5 << 4) | DR_3;  // DrRange
            put_queue_mac_cmds(mote, 6, cmd_buf);

            mote->ChMask |= 0x0004;  // ch2 enable
            mote->force_adr = true;  // ensure ch_mask is sent
            printf("put NEW_CHANNEL_REQ\n");
            break;
        case MOTE_MAC_NEW_CHANNEL_ANS:
            mote->ChMask &= ~0x0003;  // ch0,1 disable
            mote->force_adr = true;  // ensure ch_mask is sent
            mote->session_start = false;   // done
            printf("session_start = false\n");
            break;
    } // ...switch (cmd)
}

void band_init_session(mote_t* mote)
{
    uint8_t cmd_buf[MAC_CMD_SIZE];
    uint32_t freq;
    Rx2ChannelParams_t Rx2Channel = RX_WND_2_CHANNEL;

    printf("PING_SLOT_CHANNEL_REQ ");

    freq = PINGSLOT_CHANNEL_FREQ(0) / 100;
    cmd_buf[0] = SRV_MAC_PING_SLOT_CHANNEL_REQ;
    cmd_buf[1] = freq & 0xff;
    cmd_buf[2] = (freq >> 8) & 0xff;
    cmd_buf[3] = (freq >> 16) & 0xff;
    cmd_buf[4] = PING_SLOT_DATARATE;
    put_queue_mac_cmds(mote, 5, cmd_buf);


    freq = BEACON_CHANNEL_FREQ() / 100;
    cmd_buf[0] = SRV_MAC_BEACON_FREQ_REQ;
    cmd_buf[1] = freq & 0xff;
    cmd_buf[2] = (freq >> 8) & 0xff;
    cmd_buf[3] = (freq >> 16) & 0xff;
    put_queue_mac_cmds(mote, 4, cmd_buf);


    freq = Rx2Channel.Frequency / 100;
    cmd_buf[0] = SRV_MAC_RX_PARAM_SETUP_REQ;   // Rx2 window config 
    cmd_buf[1] = 0 | Rx2Channel.Datarate; // drOffset:hi-nibble, datarate:lo-nibble 
    cmd_buf[2] = freq & 0xff;
    cmd_buf[3] = (freq >> 8) & 0xff;
    cmd_buf[4] = (freq >> 16) & 0xff;
    put_queue_mac_cmds(mote, 5, cmd_buf);

}

void band_init_channel_mask(mote_t* mote)
{
    mote->ChMask = 0x0003;  /* LC1, LC2 */
}

/* end ARIB_8CH */
#endif /* USE_BAND */

