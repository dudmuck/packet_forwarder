#include "loragw_hal.h"

void lorawan_parse_uplink(struct lgw_pkt_rx_s *p);
int parse_lorawan_configuration(const char * conf_file);

extern uint8_t tx_rf_chain;    // written by sx1301 conf
