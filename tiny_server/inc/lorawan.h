#include <pthread.h>
#include "loragw_hal.h"

extern pthread_mutex_t mx_concent; /* control access to the concentrator */

typedef struct sBeaconContext
{
    struct sBeaconCfg {
        uint32_t Interval;
    } Cfg;
    uint32_t BeaconTime;
} BeaconContext_t;

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
extern bool beacon_valid;
extern float g_sx1301_ppm_err;  // from tiny server
extern uint32_t trigcnt_pingslot_zero;  // from tiny server
void lorawan_service_ping(void);
extern bool beacon_guard;  // from tiny server
#endif	/* ENABLE_CLASS_B */


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
