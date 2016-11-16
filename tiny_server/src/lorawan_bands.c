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
#include "lorawan.h"


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

#elif defined(USE_BAND_JPN920)

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
get_rx2_config(struct lgw_pkt_tx_s* tx_pkt)     // jpn920
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
    printf("jpn920-band_conv(): rx:%uhz ", rx_pkt->freq_hz);

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
        printf("USE_BAND_JPN920 rx cf %uhz out of range\n", cf);
        return -1;
    }
    return 0;
}

#endif /* USE_BAND */

