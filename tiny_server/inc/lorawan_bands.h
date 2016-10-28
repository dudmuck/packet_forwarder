extern uint8_t Rx1DrOffset;
extern uint8_t dl_rxwin;

void band_conv(struct lgw_pkt_rx_s* rx_pkt, struct lgw_pkt_tx_s* tx_pkt);
int check_band_config(uint32_t cf);
