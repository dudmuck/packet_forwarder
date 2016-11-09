extern uint8_t Rx1DrOffset;
extern uint8_t dl_rxwin;

void band_conv(struct lgw_pkt_rx_s* rx_pkt, struct lgw_pkt_tx_s* tx_pkt);
int check_band_config(uint32_t cf);
void get_rx2_config(struct lgw_pkt_tx_s* tx_pkt);

/******************************************************************/

typedef struct {
    uint8_t bw; // bandwidth
    uint8_t bps_sf; // bps for fsk, sf for lora
} data_rate_t;
extern const data_rate_t data_rates[];

/*!
 * LoRaMAC receive window 2 channel parameters
 */
typedef struct sRx2ChannelParams
{
    /*!
     * Frequency in Hz
     */
    uint32_t Frequency;
    /*!
     * Data rate
     *
     * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
     *
     * US915 - [DR_8, DR_9, DR_10, DR_11, DR_12, DR_13]
     */
    uint8_t  Datarate;
} Rx2ChannelParams_t;

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define BEACON_CHANNEL_FREQ( )                      ( 923.3e6 + ( BeaconChannel( 0 ) * 600e3 ) )
uint8_t BeaconChannel( uint32_t devAddr );

/*!
 * LoRaMac datarates definition
 */
#define DR_0                                        0  // SF10 - BW125 |
#define DR_1                                        1  // SF9  - BW125 |
#define DR_2                                        2  // SF8  - BW125 +-> Up link
#define DR_3                                        3  // SF7  - BW125 |
#define DR_4                                        4  // SF8  - BW500 |
#define DR_5                                        5  // RFU
#define DR_6                                        6  // RFU
#define DR_7                                        7  // RFU
#define DR_8                                        8  // SF12 - BW500 |
#define DR_9                                        9  // SF11 - BW500 |
#define DR_10                                       10 // SF10 - BW500 |
#define DR_11                                       11 // SF9  - BW500 |
#define DR_12                                       12 // SF8  - BW500 +-> Down link
#define DR_13                                       13 // SF7  - BW500 |
#define DR_14                                       14 // RFU          |
#define DR_15                                       15 // RFU          |


#define RX_WND_2_CHANNEL                                  { 923300000, DR_8 }

#define BEACON_SIZE         19

#define BEACON_BW           BW_500KHZ
#define BEACON_SF           DR_LORA_SF10

#define PINGSLOT_CHANNEL_FREQ( x )                  ( 923.3e6 + ( BeaconChannel( x ) * 600e3 ) )

#elif defined(USE_BAND_JPN920)

#define DR_0                                        0  // SF12 - BW125
#define DR_1                                        1  // SF11 - BW125
#define DR_2                                        2  // SF10 - BW125
#define DR_3                                        3  // SF9  - BW125
#define DR_4                                        4  // SF8  - BW125
#define DR_5                                        5  // SF7  - BW125
#define DR_6                                        6  // SF7  - BW250
#define DR_7                                        7  // FSK


#define BEACON_SIZE         19
#define BEACON_CHANNEL_FREQ( )                      ( 922000000 )
#define BEACON_BW           BW_125KHZ
#define BEACON_SF           DR_LORA_SF10

#define PINGSLOT_CHANNEL_FREQ( x )                  922000000

#define RX_WND_2_CHANNEL                                  { 923200000, DR_2 }

#elif defined(USE_BAND_868)

#define DR_0                                        0  // SF12 - BW125
#define DR_1                                        1  // SF11 - BW125
#define DR_2                                        2  // SF10 - BW125
#define DR_3                                        3  // SF9  - BW125
#define DR_4                                        4  // SF8  - BW125
#define DR_5                                        5  // SF7  - BW125
#define DR_6                                        6  // SF7  - BW250
#define DR_7                                        7  // FSK

#define RX_WND_2_CHANNEL                                  { 869525000, DR_0 }

#define BEACON_SIZE         17
#define BEACON_CHANNEL_FREQ( )                      ( 869525000 )
#define BEACON_BW           BW_125KHZ   /* DR3 */
#define BEACON_SF           DR_LORA_SF9

#define PINGSLOT_CHANNEL_FREQ( x )                  869525000

#endif
