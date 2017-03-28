#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <signal.h>
#include <sys/param.h>

#include "gw_server.h"
#include "trace.h"
#include "parson.h"
#include "loragw_hal.h"
#include "loragw_gps.h"

#define DEFAULT_PORT     5555

#define STRINGIFY(x)    #x
#define STR(x)          STRINGIFY(x)

/* sx1301 configuration */
struct lgw_conf_board_s _boardconf;
struct lgw_conf_lbt_s _lbtconf;
struct lgw_conf_rxrf_s _rfconfs[LGW_RF_CHAIN_NB];
struct lgw_conf_rxif_s _ifconfs[LGW_MULTI_NB];
struct lgw_conf_rxif_s _ifconf_std;
struct lgw_conf_rxif_s _ifconf_fsk;
bool tx_rf_chain;   // written by sx1301 config
bool lgw_started = false;

/* Gateway specificities */
static int8_t antenna_gain = 0;
unsigned char mac_address[6];   /* unique identifier */
long run_rate_usec;

/* TX capabilities */
static struct lgw_tx_gain_lut_s txlut; /* TX gain table */
static uint32_t tx_freq_min[LGW_RF_CHAIN_NB]; /* lowest frequency supported by TX chain */
static uint32_t tx_freq_max[LGW_RF_CHAIN_NB]; /* highest frequency supported by TX chain */

static char serv_addr[64] = STR(DEFAULT_SERVER); /* address of the server (host name or IPv4/IPv6) */
static uint16_t serv_port = DEFAULT_PORT;
bool connected_to_server = false;
int _sock = -1;

/* packets filtering configuration variables */
static bool fwd_valid_pkt = true; /* packets with PAYLOAD CRC OK are forwarded */
static bool fwd_nocrc_pkt = false; /* packets with NO PAYLOAD CRC are NOT forwarded */
static bool fwd_error_pkt = false; /* packets with PAYLOAD CRC ERROR are NOT forwarded */

/* GPS configuration and synchronization */
static char gps_tty_path[64] = "\0"; /* path of the TTY port GPS is connected on */
bool gps_time_valid = false;
uint8_t pps_cnt;
uint32_t prev_trig_tstamp;
struct timespec host_time_at_pps;

/* Reference coordinates, for broadcasting (beacon) */
static struct coord_s reference_coord;

uint32_t beacon_period = 128;   // TODO: put into beacon_info
struct {
    uint32_t hz;
    uint8_t size;
    uint8_t sf;
    uint8_t bw;
    uint8_t rfuOffset1;
    uint8_t rfuOffset2;
    bool us915;
} beacon_info;
uint8_t skip_beacon_cnt = 0;    // TODO: implement user write
bool beacon_guard = false;

#define BILLION     1000000000
double difftimespec(struct timespec end, struct timespec beginning) {
    double x;

    x = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    x += (double)(end.tv_sec - beginning.tv_sec);

    return x;
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

int get_host_unique_id()
{
    struct ifreq ifr;
    struct ifconf ifc;
    char buf[1024];
    int success = 0;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock == -1) { return -1; };

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) { return -1; }

    struct ifreq* it = ifc.ifc_req;
    const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));

    for (; it != end; ++it) {
        strcpy(ifr.ifr_name, it->ifr_name);
        if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
            if (! (ifr.ifr_flags & IFF_LOOPBACK)) { // don't count loopback
                if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
                    success = 1;
                    break;
                }
            }
        }
        else { return -1; }
    }


    if (success) {
        memcpy(mac_address, ifr.ifr_hwaddr.sa_data, 6);
        int i;
        for (i = 0; i<6; i++)
            printf("%02x ", mac_address[i]);
        printf("\n");
    }
    return 0;
}

static int _parse_SX1301_configuration(char* const conf_file, JSON_Value *root_val)
{
    int i;
    char param_name[32]; /* used to generate variable parameter names */
    const char *str; /* used to store string value from JSON object */
    const char conf_obj_name[] = "SX1301_conf";
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_lbt_obj = NULL;
    JSON_Object *conf_lbtchan_obj = NULL;
    JSON_Value *val = NULL;
    JSON_Array *conf_array = NULL;
    uint32_t sf, bw, fdev;

    printf("parse_SX1301_configuration()\n");

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing SX1301 parameters\n", conf_file, conf_obj_name);
    }

    /* set board configuration */
    val = json_object_get_value(conf_obj, "lorawan_public"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        _boardconf.lorawan_public = (bool)json_value_get_boolean(val);
    }

    val = json_object_get_value(conf_obj, "clksrc"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONNumber) {
        _boardconf.clksrc = (uint8_t)json_value_get_number(val);
    }

    /* set LBT configuration */
    conf_lbt_obj = json_object_get_object(conf_obj, "lbt_cfg"); /* fetch value (if possible) */
    if (conf_lbt_obj == NULL) {
        MSG("INFO: no configuration for LBT\n");
    } else {
        val = json_object_get_value(conf_lbt_obj, "enable"); /* fetch value (if possible) */
        if (val != NULL) {
            if (json_value_get_type(val) == JSONBoolean) {
                _lbtconf.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG("WARNING: Data type for lbt_cfg.enable seems wrong, please check (val:%p)\n", val);
                _lbtconf.enable = false;
            }
        }


        if (_lbtconf.enable == true) {
            val = json_object_get_value(conf_lbt_obj, "rssi_target"); /* fetch value (if possible) */
            if (val != NULL) {
                if (json_value_get_type(val) == JSONNumber) {
                    _lbtconf.rssi_target = (int8_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for lbt_cfg.rssi_target seems wrong, please check (val:%p)\n", val);
                    _lbtconf.rssi_target = 0;
                }
            }
            val = json_object_get_value(conf_lbt_obj, "sx127x_rssi_offset"); /* fetch value (if possible) */
            if (val != NULL) {
                if (json_value_get_type(val) == JSONNumber) {
                    _lbtconf.rssi_offset = (int8_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for lbt_cfg.sx127x_rssi_offset seems wrong, please check (val:%p)\n", val);
                    _lbtconf.rssi_offset = 0;
                }
            }
            /* set LBT channels configuration */
            conf_array = json_object_get_array(conf_lbt_obj, "chan_cfg");
            if (conf_array != NULL) {
                _lbtconf.nb_channel = json_array_get_count( conf_array );
                MSG("INFO: %u LBT channels configured ", _lbtconf.nb_channel);
            }
            for (i = 0; i < (int)_lbtconf.nb_channel; i++) {
                /* Sanity check */
                if (i >= LBT_CHANNEL_FREQ_NB)
                {
                    MSG("ERROR: LBT channel %d not supported, skip it\n", i );
                    break;
                }
                /* Get LBT channel configuration object from array */
                conf_lbtchan_obj = json_array_get_object(conf_array, i);

                /* Channel frequency */
                val = json_object_dotget_value(conf_lbtchan_obj, "freq_hz"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    _lbtconf.channels[i].freq_hz = (uint32_t)json_value_get_number(val);
                    MSG("%uhz ", _lbtconf.channels[i].freq_hz);
                } else {
                    MSG("WARNING: Data type for lbt_cfg.channels[%d].freq_hz seems wrong, please check (val:%p)\n", i, val);
                    _lbtconf.channels[i].freq_hz = 0;
                }

                /* Channel scan time */
                val = json_object_dotget_value(conf_lbtchan_obj, "scan_time_us"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    _lbtconf.channels[i].scan_time_us = (uint16_t)json_value_get_number(val);
                    MSG("%uus ", _lbtconf.channels[i].scan_time_us );
                } else {
                    MSG("WARNING: Data type for lbt_cfg.channels[%d].scan_time_us seems wrong, please check (val:%p)\n", i, val);
                    _lbtconf.channels[i].scan_time_us = 0;
                }
            }
            MSG("\n");
        } else {
            MSG("INFO: LBT is disabled\n");
        }
    }

    /* set antenna gain configuration */
    val = json_object_get_value(conf_obj, "antenna_gain"); /* fetch value (if possible) */
    if (val != NULL) {
        if (json_value_get_type(val) == JSONNumber) {
            antenna_gain = (int8_t)json_value_get_number(val);
        } else {
            MSG("WARNING: Data type for antenna_gain seems wrong, please check\n");
            antenna_gain = 0;
        }
    }
    MSG("INFO: antenna_gain %d dBi\n", antenna_gain);

    /* set configuration for tx gains */
    for (i = 0; i < TX_GAIN_LUT_SIZE_MAX; i++) {
        snprintf(param_name, sizeof param_name, "tx_lut_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (val == NULL) {
            continue;
        }
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for tx gain lut %i (val:%p)\n", i, val);
            continue;
        }
        txlut.size++; /* update TX LUT size based on JSON object found in configuration file */
        /* there is an object to configure that TX gain index, let's parse it */
        snprintf(param_name, sizeof param_name, "tx_lut_%i.pa_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            txlut.lut[i].pa_gain = (uint8_t)json_value_get_number(val);
        } /*else {
            MSG("WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
            txlut.lut[i].pa_gain = 0;
        }*/
        snprintf(param_name, sizeof param_name, "tx_lut_%i.dac_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            txlut.lut[i].dac_gain = (uint8_t)json_value_get_number(val);
        } else {
            txlut.lut[i].dac_gain = 3; /* This is the only dac_gain supported for now */
        }
        snprintf(param_name, sizeof param_name, "tx_lut_%i.dig_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            txlut.lut[i].dig_gain = (uint8_t)json_value_get_number(val);
        } /*else {
            MSG("WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
            txlut.lut[i].dig_gain = 0;
        }*/
        snprintf(param_name, sizeof param_name, "tx_lut_%i.mix_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            txlut.lut[i].mix_gain = (uint8_t)json_value_get_number(val);
        } /*else {
            MSG("WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
            txlut.lut[i].mix_gain = 0;
        }*/
        snprintf(param_name, sizeof param_name, "tx_lut_%i.rf_power", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            txlut.lut[i].rf_power = (int8_t)json_value_get_number(val);
        } /*else {
            MSG("WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
            txlut.lut[i].rf_power = 0;
        }*/
    }

    /* set configuration for RF chains */
    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        snprintf(param_name, sizeof param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for radio %i\n", i);
            continue;
        }
        /* there is an object to configure that radio, let's parse it */
        snprintf(param_name, sizeof param_name, "radio_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (val != NULL) {
            if (json_value_get_type(val) == JSONBoolean) {
                _rfconfs[i].enable = (bool)json_value_get_boolean(val);
            } else {
                MSG("ERR: radio_%d.enable not boolean\n", i);
                _rfconfs[i].enable = false;
            }
        }

        if (_rfconfs[i].enable == false) { /* radio disabled, nothing else to parse */
            MSG("INFO: radio %i disabled\n", i);
        } else  { /* radio enabled, will parse the other parameters */

            snprintf(param_name, sizeof param_name, "radio_%i.rssi_offset", i);
            if (json_object_dotget_value(conf_obj, param_name) != NULL) {
                _rfconfs[i].rssi_offset = (float)json_object_dotget_number(conf_obj, param_name);
            }

            snprintf(param_name, sizeof param_name, "radio_%i.freq", i);
            if (json_object_dotget_value(conf_obj, param_name) != NULL) {
                _rfconfs[i].freq_hz = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            }

            snprintf(param_name, sizeof param_name, "radio_%i.type", i);
            str = json_object_dotget_string(conf_obj, param_name);
            if (str != NULL) {
                if (!strncmp(str, "SX1255", 6)) {
                    _rfconfs[i].type = LGW_RADIO_TYPE_SX1255;
                } else if (!strncmp(str, "SX1257", 6)) {
                    _rfconfs[i].type = LGW_RADIO_TYPE_SX1257;
                } else {
                    MSG("WARNING: invalid radio type: %s (should be SX1255 or SX1257)\n", str);
                }
            }
            snprintf(param_name, sizeof param_name, "radio_%i.tx_enable", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (val != NULL) {
                if (json_value_get_type(val) == JSONBoolean) {
                    _rfconfs[i].tx_enable = (bool)json_value_get_boolean(val);
                    if (_rfconfs[i].tx_enable == true) {
                        tx_rf_chain = i;
                        /* tx is enabled on this rf chain, we need its frequency range */
                        snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_min", i);
                        tx_freq_min[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                        snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_max", i);
                        tx_freq_max[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                        if ((tx_freq_min[i] == 0) || (tx_freq_max[i] == 0)) {
                            MSG("WARNING: no frequency range specified for TX rf chain %d\n", i);
                        }
                        /* ... and the notch filter frequency to be set */
                        snprintf(param_name, sizeof param_name, "radio_%i.tx_notch_freq", i);
                        _rfconfs[i].tx_notch_freq = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    }
                } else {
                    _rfconfs[i].tx_enable = false;
                }
            }
            if (_rfconfs[i].type == LGW_RADIO_TYPE_SX1255)
                MSG("INFO SX1255: ");
            else if (_rfconfs[i].type == LGW_RADIO_TYPE_SX1257)
                MSG("INFO SX1257: ");
            MSG("radio %i enabled, center frequency %u, RSSI offset %f, tx enabled %d, tx_notch_freq %u\n", i, _rfconfs[i].freq_hz, _rfconfs[i].rssi_offset, _rfconfs[i].tx_enable, _rfconfs[i].tx_notch_freq);
        }
    } // ..for (i = 0; i < LGW_RF_CHAIN_NB; ++i)

    /* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
    for (i = 0; i < LGW_MULTI_NB; ++i) {
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (val == NULL)
            continue;
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for Lora multi-SF channel %i\n", i);
            continue;
        }
        /* there is an object to configure that Lora multi-SF channel, let's parse it */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            _ifconfs[i].enable = (bool)json_value_get_boolean(val);
        } else {
            _ifconfs[i].enable = false;
        }
        if (_ifconfs[i].enable == false) { /* Lora multi-SF channel disabled, nothing else to parse */
            MSG("INFO: Lora multi-SF channel %i disabled\n", i);
        } else  { /* Lora multi-SF channel enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
            _ifconfs[i].rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
            _ifconfs[i].freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);
            // TODO: handle individual SF enabling and disabling (spread_factor)
            MSG("INFO: Lora multi-SF channel %i>  radio %i, IF %i Hz, 125 kHz bw, SF 7 to 12\n", i, _ifconfs[i].rf_chain, _ifconfs[i].freq_hz);
        }
    }

    /* set configuration for Lora standard channel */
    val = json_object_get_value(conf_obj, "chan_Lora_std"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for Lora standard channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_Lora_std.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            _ifconf_std.enable = (bool)json_value_get_boolean(val);
        } else {
            _ifconf_std.enable = false;
        }
        if (_ifconf_std.enable == false) {
            MSG("INFO: Lora standard channel %i disabled\n", i);
        } else  {
            _ifconf_std.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.radio");
            _ifconf_std.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.bandwidth");
            switch(bw) {
                case 500000: _ifconf_std.bandwidth = BW_500KHZ; break;
                case 250000: _ifconf_std.bandwidth = BW_250KHZ; break;
                case 125000: _ifconf_std.bandwidth = BW_125KHZ; break;
                default: _ifconf_std.bandwidth = BW_UNDEFINED;
            }
            sf = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.spread_factor");
            switch(sf) {
                case  7: _ifconf_std.datarate = DR_LORA_SF7;  break;
                case  8: _ifconf_std.datarate = DR_LORA_SF8;  break;
                case  9: _ifconf_std.datarate = DR_LORA_SF9;  break;
                case 10: _ifconf_std.datarate = DR_LORA_SF10; break;
                case 11: _ifconf_std.datarate = DR_LORA_SF11; break;
                case 12: _ifconf_std.datarate = DR_LORA_SF12; break;
                default: _ifconf_std.datarate = DR_UNDEFINED;
            }
            MSG("INFO: Lora std channel> radio %i, IF %i Hz, %u Hz bw, SF %u\n", _ifconf_std.rf_chain, _ifconf_std.freq_hz, bw, sf);
        }
    }

    /* set configuration for FSK channel */
    val = json_object_get_value(conf_obj, "chan_FSK"); /* fetch value (if possible) */
    printf("chan_FSK object: %p\n", val);
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for FSK channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_FSK.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            _ifconf_fsk.enable = (bool)json_value_get_boolean(val);
        } else {
            _ifconf_fsk.enable = false;
        }
        if (_ifconf_fsk.enable == false) {
            MSG("INFO: FSK channel %i disabled\n", i);
        } else  {
            _ifconf_fsk.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.radio");
            _ifconf_fsk.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_FSK.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.bandwidth");
            fdev = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.freq_deviation");
            _ifconf_fsk.datarate = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.datarate");

            /* if chan_FSK.bandwidth is set, it has priority over chan_FSK.freq_deviation */
            if ((bw == 0) && (fdev != 0)) {
                bw = 2 * fdev + _ifconf_fsk.datarate;
            }
            if      (bw == 0)      _ifconf_fsk.bandwidth = BW_UNDEFINED;
            else if (bw <= 7800)   _ifconf_fsk.bandwidth = BW_7K8HZ;
            else if (bw <= 15600)  _ifconf_fsk.bandwidth = BW_15K6HZ;
            else if (bw <= 31200)  _ifconf_fsk.bandwidth = BW_31K2HZ;
            else if (bw <= 62500)  _ifconf_fsk.bandwidth = BW_62K5HZ;
            else if (bw <= 125000) _ifconf_fsk.bandwidth = BW_125KHZ;
            else if (bw <= 250000) _ifconf_fsk.bandwidth = BW_250KHZ;
            else if (bw <= 500000) _ifconf_fsk.bandwidth = BW_500KHZ;
            else _ifconf_fsk.bandwidth = BW_UNDEFINED;

            MSG("INFO: FSK channel> radio %i, IF %i Hz, %u Hz bw, %u bps datarate\n", _ifconf_fsk.rf_chain, _ifconf_fsk.freq_hz, bw, _ifconf_fsk.datarate);
        }
    }

    return 0;
}

static int parse_gateway_configuration(const char * conf_file)
{
    const char conf_obj_name[] = "gateway_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
    const char *str; /* pointer to sub-strings in the JSON data */

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
        MSG("INFO: %s does contain a JSON object named %s, parsing gateway parameters\n", conf_file, conf_obj_name);
    }

    /* server hostname or IP address (optional) */
    str = json_object_get_string(conf_obj, "server_address");
    if (str != NULL) {
        strncpy(serv_addr, str, sizeof serv_addr);
        MSG("INFO: server hostname or IP address is configured to \"%s\"\n", serv_addr);
    } else {
        printf("%s: missing server_address\n", conf_obj_name);
        return -1;
    }

    /* get tcp port to server  */
    val = json_object_get_value(conf_obj, "server_port");
    if (val != NULL) {
        serv_port = json_value_get_number(val);
        MSG("INFO: server port is configured to %d\n", serv_port);
    } else {
        printf("%s: missing server_port\n", conf_obj_name);
        return -1;
    }

    val = json_object_get_value(conf_obj, "run_rate_ms");
    if (val != NULL) {
        unsigned int run_rate_ms = json_value_get_number(val);
        MSG("INFO: run rate configured to %dms\n", run_rate_ms);
        run_rate_usec = run_rate_ms * 1000;  /* milliseconds to microseconds */
    } else {
        printf("%s: missing run_rate_ms\n", conf_obj_name);
        run_rate_usec = 100000;  /* default run rate */
    }

    /* packet filtering parameters */
    val = json_object_get_value(conf_obj, "forward_crc_valid");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_valid_pkt = (bool)json_value_get_boolean(val);
    }
    MSG("INFO: packets received with a valid CRC will%s be forwarded\n", (fwd_valid_pkt ? "" : " NOT"));
    val = json_object_get_value(conf_obj, "forward_crc_error");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_error_pkt = (bool)json_value_get_boolean(val);
    }
    MSG("INFO: packets received with a CRC error will%s be forwarded\n", (fwd_error_pkt ? "" : " NOT"));
    val = json_object_get_value(conf_obj, "forward_crc_disabled");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_nocrc_pkt = (bool)json_value_get_boolean(val);
    }
    MSG("INFO: packets received with no CRC will%s be forwarded\n", (fwd_nocrc_pkt ? "" : " NOT"));

    /* GPS module TTY path (optional) */
    str = json_object_get_string(conf_obj, "gps_tty_path");
    if (str != NULL) {
        strncpy(gps_tty_path, str, sizeof gps_tty_path);
        MSG("INFO: GPS serial port path is configured to \"%s\"\n", gps_tty_path);
    } else {
        printf("%s: missing gps_tty_path\n", conf_obj_name);
        return -1;  /* this is class-B forwarder */
    }

    /* get reference coordinates */
    val = json_object_get_value(conf_obj, "ref_latitude");
    if (val != NULL) {
        reference_coord.lat = (double)json_value_get_number(val);
        MSG("INFO: Reference latitude is configured to %f deg\n", reference_coord.lat);
    }
    val = json_object_get_value(conf_obj, "ref_longitude");
    if (val != NULL) {
        reference_coord.lon = (double)json_value_get_number(val);
        MSG("INFO: Reference longitude is configured to %f deg\n", reference_coord.lon);
    }
    val = json_object_get_value(conf_obj, "ref_altitude");
    if (val != NULL) {
        reference_coord.alt = (short)json_value_get_number(val);
        MSG("INFO: Reference altitude is configured to %i meters\n", reference_coord.alt);
    }

    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

typedef enum {
    GST_OUTSIDE = 0,
    GST_UBX_SYNC,
    GST_NMEA,
    GST_UBX_CLASS_1,
    GST_UBX_CLASS_2,
    GST_UBX_LENGTH_LO,
    GST_UBX_LENGTH_HI,
    GST_UBX_END
} gps_uart_state_e;
struct timespec g_utc_time; /* UTC time associated with PPS pulse */
struct timespec g_gps_time; /* UBX time associated with PPS pulse */
struct timespec* time_ptr_ = &g_gps_time;    /* ? using UTC or UBX ? */
struct coord_s coord;   /* physical location */

void
gps_service(int fd)
{
    static gps_uart_state_e gst = GST_OUTSIDE;
    static uint8_t gps_buf_idx;
    static char gps_buf[256];
    static uint16_t ubx_length;
    uint8_t serial_buff[128];
    ssize_t nb_char;
    int i;

    nb_char = read(fd, serial_buff, sizeof(serial_buff));

    for (i = 0; i < nb_char; i++) {
        struct timespec this_gps_time;
        switch (gst) {
            case GST_OUTSIDE:
                if (serial_buff[i] == LGW_GPS_UBX_SYNC_CHAR) {
                    gps_buf_idx = 0;
                    gps_buf[gps_buf_idx++] = serial_buff[i];
                    gst = GST_UBX_SYNC;
                    break;
                } else if (serial_buff[i] == LGW_GPS_NMEA_SYNC_CHAR) {
                    gps_buf_idx = 0;
                    gps_buf[gps_buf_idx++] = serial_buff[i];
                    gst = GST_NMEA;
                    break;
                }
                break;
            case GST_UBX_SYNC:
                if (serial_buff[i] == 0x62) {
                    gps_buf[gps_buf_idx++] = serial_buff[i];
                    gst = GST_UBX_CLASS_1;
                }
                break;
            case GST_UBX_CLASS_1:
                gps_buf[gps_buf_idx++] = serial_buff[i];
                gst = GST_UBX_CLASS_2;
                break;
            case GST_UBX_CLASS_2:
                gps_buf[gps_buf_idx++] = serial_buff[i];
                gst = GST_UBX_LENGTH_LO;
                break;
            case GST_UBX_LENGTH_LO:
                gps_buf[gps_buf_idx++] = serial_buff[i];
                ubx_length = serial_buff[i];
                gst = GST_UBX_LENGTH_HI;
                break;
            case GST_UBX_LENGTH_HI:
                gps_buf[gps_buf_idx++] = serial_buff[i];
                ubx_length |= serial_buff[i] << 8;
                ubx_length += 6 + 2;    /* header + payload + checksum */
                gst = GST_UBX_END;
                break;
            case GST_UBX_END:
                if (gps_buf_idx < sizeof(gps_buf))
                    gps_buf[gps_buf_idx++] = serial_buff[i];

                if (gps_buf_idx >= ubx_length) {
                    size_t frame_size = 0;
                    enum gps_msg latest_msg; /* keep track of latest NMEA message parsed */
                    latest_msg = lgw_parse_ubx(gps_buf, gps_buf_idx, &frame_size);
                    if (latest_msg == INCOMPLETE) {
                        printf("ubx-INCOMPLETE\n");
                    } else if (latest_msg == IGNORED) {
                        printf("ubx-IGNORED\n");
                    } else if (latest_msg == INVALID) {
                        printf("ubx-INVALID\n");
                    } else if (latest_msg == UBX_NAV_TIMEGPS) {
                        int i = lgw_gps_get(&g_utc_time, &this_gps_time, NULL, NULL);
                        //MSG("UBX_NAV_TIMEGPS ");
                        if (i != LGW_GPS_SUCCESS) {
                            MSG("WARNING: [gps] could not get GPS time from GPS\n");
                            gps_time_valid = false;
                        } else {
                            //printf("gps_time:%lu.%09lu ->", this_gps_time.tv_sec, this_gps_time.tv_nsec);
                            /* round to second */
                            g_gps_time.tv_sec = this_gps_time.tv_sec;
                            if (this_gps_time.tv_nsec > 500000000) {
                                g_gps_time.tv_sec++;
                            }
                            g_gps_time.tv_nsec = 0;
                            //printf("gps_time:%lu.%09lu\n", g_gps_time.tv_sec, g_gps_time.tv_nsec);
                            gps_time_valid = true;
                        }
                    } else {
                        printf("latest_msg:%d\n", latest_msg);
                    }
                    gst = GST_OUTSIDE;
                }
                break;
            case GST_NMEA:
                if (gps_buf_idx < sizeof(gps_buf))
                    gps_buf[gps_buf_idx++] = serial_buff[i];
                else
                    printf("nmea overflow\n");

                if (serial_buff[i] == 0x0a) {
                    enum gps_msg latest_msg;
                    gps_buf[gps_buf_idx] = 0; // null terminate
                    latest_msg = lgw_parse_nmea(gps_buf, gps_buf_idx);
                    if (latest_msg == INVALID) {
                        /* returns INVALID for ignored sentence */
                    } else if (latest_msg == UNKNOWN) {
                        printf("nmea-UNKNOWN\n");
                    } else if (latest_msg == IGNORED) {
                        printf("nmea-IGNORED\n");
                    } else if (latest_msg == NMEA_RMC || latest_msg == NMEA_GGA) {
                        int i = lgw_gps_get(&g_utc_time, &this_gps_time, &coord, NULL);
                        if (i != LGW_GPS_SUCCESS) {
                            MSG("WARNING: [gps] could not get GPS time from GPS\n");
                            gps_time_valid = false;
                        } else {
                            //printf("# GPS coordinates: latitude %.5f, longitude %.5f, altitude %i m\n", coord.lat, coord.lon, coord.alt);
                            //printf("utc:%lu.%09lu\n", g_utc_time.tv_sec, g_utc_time.tv_nsec);
                            gps_time_valid = true;
                        }
                    } else {
                        printf("nmea latest_msg:%d\n", latest_msg);
                    }
                    gst = GST_OUTSIDE;
                }
                break;
    
        }
    }
}

void
init_sockaddr (struct sockaddr_in *name, const char *hostname, uint16_t port)
{
    struct hostent *hostinfo;

    name->sin_family = AF_INET;
    name->sin_port = htons (port);
    hostinfo = gethostbyname (hostname);
    if (hostinfo == NULL)
    {
        fprintf (stderr, "Unknown host %s.\n", hostname);
        exit (EXIT_FAILURE);
    }
    name->sin_addr = *(struct in_addr *) hostinfo->h_addr;
}

int connect_to_server()
{
    extern void init_sockaddr (struct sockaddr_in *name, const char *hostname, uint16_t port);
    struct sockaddr_in servername;
    int sock;

    /* Create the socket. */
    sock = socket (PF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        perror ("socket (client)");
        return -1;
    }

    /* Connect to the server. */
    printf("attempting server %s\n", serv_addr);
    init_sockaddr (&servername, serv_addr, serv_port);
    if (0 > connect (sock, (struct sockaddr *) &servername, sizeof (servername)))
    {
        perror ("connect to server");
        return -1;
    }

    return sock;
}

#define TX_PRELOAD_US           200000
#define NUM_TX_PKTS     4
struct lgw_pkt_tx_s tx_pkts[NUM_TX_PKTS];

void gw_tx_service(bool im)
{
    double seconds_since_pps;
    struct timespec now;
    uint32_t count_us_now;
    int i;

    if (clock_gettime (CLOCK_MONOTONIC, &now) == -1)
        perror ("clock_gettime");

    /* get time difference between last host_time_at_pps and now */
    seconds_since_pps = difftimespec(now, host_time_at_pps);

    /* get now sx1301 counter */
    count_us_now = prev_trig_tstamp + (seconds_since_pps * 1000000);
    //printf("count_us_now:%u ", count_us_now);

    /* compare count_us of waiting tx packets to count_us_now */
    int32_t min_us_to_tx_start = 0x7fffffff;
    int ftbs_i = -1;
    for (i = 0; i < NUM_TX_PKTS; i++) {
        if (tx_pkts[i].freq_hz != 0) {
            int32_t us_to_tx_start = tx_pkts[i].count_us - count_us_now;
            //printf("us_to_tx_start:%d\n", us_to_tx_start);
            if (us_to_tx_start < min_us_to_tx_start) {
                min_us_to_tx_start = us_to_tx_start;
                ftbs_i = i;
            }
        }
    }
    if (ftbs_i == -1) {
        /* nothing to be transmitted */
        return;
    }

    //printf("min_us_to_tx_start:%d\n", min_us_to_tx_start);
    if (min_us_to_tx_start < TX_PRELOAD_US) {
        if (beacon_guard) {
            printf("[31mskipping tx due to beacon_guard[0m\n");
        } else {
            if (min_us_to_tx_start < 10000)
                printf("[31m");
            if (im)
                printf("immediatly ");
            printf("sending tx_pkts[%d] at %u (%dus before tx)[0m\n", ftbs_i, count_us_now, min_us_to_tx_start);
            i = lgw_send(tx_pkts[ftbs_i]);
            tx_pkts[ftbs_i].freq_hz = 0;    // mark as sent
            if (i == LGW_HAL_ERROR) {
                printf("lgw_send() failed\n");
            }
        }
    }
}

void put_server_downlink(const uint8_t* const user_buf)
{
    int i;
    uint32_t* u32_ptr;
    uint16_t* u16_ptr;
    unsigned int idx = 3;   // skip past cmd and length
    struct lgw_pkt_tx_s tx_pkt;

    u32_ptr = (uint32_t*)&user_buf[idx];
    tx_pkt.freq_hz = *u32_ptr;
    idx += sizeof(tx_pkt.freq_hz);

    tx_pkt.tx_mode = user_buf[idx++];

    u32_ptr = (uint32_t*)&user_buf[idx];
    tx_pkt.count_us = *u32_ptr;
    idx += sizeof(tx_pkt.count_us);

    idx++;  // rf_chain from server ignored
    tx_pkt.rf_chain = tx_rf_chain;
    tx_pkt.rf_power = user_buf[idx++];
    tx_pkt.modulation = user_buf[idx++];
    tx_pkt.bandwidth = user_buf[idx++];

    u32_ptr = (uint32_t*)&user_buf[idx];
    tx_pkt.datarate = *u32_ptr;
    idx += sizeof(tx_pkt.datarate);

    tx_pkt.coderate = user_buf[idx++];
    tx_pkt.invert_pol = user_buf[idx++];
    tx_pkt.f_dev = user_buf[idx++];

    u16_ptr = (uint16_t*)&user_buf[idx];
    tx_pkt.preamble = *u16_ptr;
    idx += sizeof(tx_pkt.preamble);

    tx_pkt.no_crc = user_buf[idx++];
    tx_pkt.no_header = user_buf[idx++];

    u16_ptr = (uint16_t*)&user_buf[idx];
    tx_pkt.size = *u16_ptr;
    idx += sizeof(tx_pkt.size);

    if (tx_pkt.size > sizeof(tx_pkt.payload)) {
        printf("tx_pkt.size too large:%u\n", tx_pkt.size);
        return;
    }

    memcpy(&tx_pkt.payload, user_buf+idx, tx_pkt.size);
    idx += tx_pkt.size;

    printf("downlink_tx() ");
    switch (tx_pkt.tx_mode) {
        case IMMEDIATE: printf("IMMEDIATE"); break;
        case TIMESTAMPED: printf("TIMESTAMPED"); break;
        case ON_GPS: printf("ON_GPS"); break;
    }
    if (tx_pkt.modulation == MOD_LORA)
        printf(" MOD_LORA ");
    else if (tx_pkt.modulation == MOD_FSK) {
        printf(" MOD_FSK:%d ", tx_pkt.f_dev);
    } else
        printf(" MOD_%d", tx_pkt.modulation);

    print_hal_bw(tx_pkt.bandwidth);
    if (tx_pkt.modulation == MOD_LORA) {
        print_hal_sf(tx_pkt.datarate);
        switch (tx_pkt.coderate) {
            case CR_LORA_4_5: printf("cr4/5"); break;
            case CR_LORA_4_6: printf("cr4/6"); break;
            case CR_LORA_4_7: printf("cr4/7"); break;
            case CR_LORA_4_8: printf("cr4/8"); break;
            default: printf("cr?"); break;
        }
    }

    /* check dBm transmit level */
    for (i = 0; i < txlut.size; i++) {
        if (tx_pkt.rf_power == txlut.lut[i].rf_power) {
            printf(" txlut%d ", i);
            break;  // exact dBm found
        }
    }
    if (i == txlut.size) {
        /* dBm not found, use closest power availble */
        printf("[31m%ddBm not found[0m ", tx_pkt.rf_power);
        if (tx_pkt.rf_power < txlut.lut[0].rf_power)
            tx_pkt.rf_power = txlut.lut[0].rf_power;
        else {
            for (i = txlut.size-1; i >= 0; i--) {
                if (tx_pkt.rf_power < txlut.lut[i].rf_power) {
                    tx_pkt.rf_power = txlut.lut[i].rf_power;
                    break;
                }
            }
        }
    }

    printf(" %uhz %uus %ddBm iq_inv:%d preamble:%d nocrc:%d noheader:%d size:%u\n",
        tx_pkt.freq_hz,
        tx_pkt.count_us,
        tx_pkt.rf_power,
        tx_pkt.invert_pol,
        tx_pkt.preamble,
        tx_pkt.no_crc,
        tx_pkt.no_header,
        tx_pkt.size
    );

    if (tx_pkt.tx_mode == TIMESTAMPED) {
        for (i = 0; i < NUM_TX_PKTS; i++) {
            if (tx_pkts[i].freq_hz == 0)
                break;
        }
        if (i == NUM_TX_PKTS) {
            printf("tx_pkts full\n");
            return;
        }

        memcpy(&tx_pkts[i], &tx_pkt, sizeof(struct lgw_pkt_tx_s));
        printf("putting tx_pkts[%d] to be sent at %u\n", i, tx_pkts[i].count_us);
        /* service immediately in case its late */
        gw_tx_service(true);
    } else if (tx_pkt.tx_mode == IMMEDIATE) {
        i = lgw_send(tx_pkt);
        if (i == LGW_HAL_ERROR) {
            printf("lgw_send() failed\n");
        }
    }
}

int
downlink_service()
{
    uint8_t user_buf[512];
    int nbytes;//, bytes_available = 0;
    unsigned int len;

    nbytes = read(_sock, user_buf, sizeof(user_buf));
    if (nbytes < 0) {
        perror("sock-read");
        return -1;
    }
    if (nbytes == 0) {
        /* likely server disconnected */
        printf("server read nbytes == 0\n");
        return -1;
    }
    //printf("server_downlink_service() read nbytes:%d\n", nbytes);

    len = user_buf[1];
    len |= user_buf[2] << 8;
    if (len != nbytes) {
        printf("given len:%d, read len:%d\n", len, nbytes);
        return 0;
    }

    switch (user_buf[0]) {
        case DOWNLINK_TX:
            put_server_downlink(user_buf);
            break;
        default:
            printf("unhandled server cmd:%d, nbytes:%d\n", user_buf[0], nbytes);
            break;
    }

    return 0;
}

static int parse_lorawan_configuration(JSON_Value *root_val)
{
    const char conf_obj_name[] = "lorawan";
    JSON_Object *conf_obj;
    JSON_Value *val;

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("ERROR: does not contain a JSON object named %s\n", conf_obj_name);
        return -1;
    } else {
        MSG("INFO: does contain a JSON object named %s, parsing lorawan parameters\n", conf_obj_name);
    }

    val = json_object_get_value(conf_obj, "beacon_period");
    if (json_value_get_type(val) == JSONNumber) {
        beacon_period = (uint8_t)json_value_get_number(val);
        MSG("beacon_period:%u\n", beacon_period);
    } else
        return -1;

    val = json_object_get_value(conf_obj, "beacon_size");
    if (json_value_get_type(val) == JSONNumber) {
        beacon_info.size = (uint8_t)json_value_get_number(val);
        MSG("beacon_size:%u\n", beacon_info.size);
    } else
        return -1;

    val = json_object_get_value(conf_obj, "us915_beacon");
    if (json_value_get_type(val) == JSONBoolean) {
        beacon_info.us915 = json_value_get_boolean(val);
        MSG("beacon us915:%u\n", beacon_info.us915);
    } else
        beacon_info.us915 = false;

    val = json_object_get_value(conf_obj, "beacon_hz");
    if (json_value_get_type(val) == JSONNumber) {
        beacon_info.hz = json_value_get_number(val);
        MSG("beacon_hz:%u\n", beacon_info.hz);
    } else if (!beacon_info.us915) {
        return -1;
    }

    val = json_object_get_value(conf_obj, "beacon_sf");
    if (json_value_get_type(val) == JSONNumber) {
        uint8_t sf = (uint8_t)json_value_get_number(val);
        MSG("beacon_sf:%u\n", sf);
        switch (sf) {
            case 7: beacon_info.sf = DR_LORA_SF7; break;
            case 8: beacon_info.sf = DR_LORA_SF8; break;
            case 9: beacon_info.sf = DR_LORA_SF9; break;
            case 10: beacon_info.sf = DR_LORA_SF10; break;
            case 11: beacon_info.sf = DR_LORA_SF11; break;
            case 12: beacon_info.sf = DR_LORA_SF12; break;
        }
    } else
        return -1;

    val = json_object_get_value(conf_obj, "beacon_bw");
    if (json_value_get_type(val) == JSONNumber) {
        unsigned int bw_khz = json_value_get_number(val);
        MSG("beacon_bw:%u\n", bw_khz);
        switch (bw_khz) {
            case 7: beacon_info.bw = BW_7K8HZ; break;
            case 16: beacon_info.bw = BW_15K6HZ; break;
            case 31: beacon_info.bw = BW_31K2HZ; break;
            case 62: beacon_info.bw = BW_62K5HZ; break;
            case 125: beacon_info.bw = BW_125KHZ; break;
            case 250: beacon_info.bw = BW_250KHZ; break;
            case 500: beacon_info.bw = BW_500KHZ; break;
            default: beacon_info.bw = BW_UNDEFINED; break;
        }
    } else
        return -1;

    val = json_object_get_value(conf_obj, "beacon_rfuOffset1");
    if (json_value_get_type(val) == JSONNumber) {
        beacon_info.rfuOffset1 = (uint8_t)json_value_get_number(val);
    } else
        return -1;

    val = json_object_get_value(conf_obj, "beacon_rfuOffset2");
    if (json_value_get_type(val) == JSONNumber) {
        beacon_info.rfuOffset2 = (uint8_t)json_value_get_number(val);
    } else
        return -1;


    return 0;
}

int
get_server_config()
{
    char file_buf[4096];
    int file_length;
    int nbytes;
    int n_read;
    JSON_Value *root_val = NULL;

    /* configuration available */
    printf("have configuration\n");
    /* first 4 bytes is length */
    nbytes = read(_sock, &file_length, sizeof(int));
    printf("conf file_length got nbytes %d: %d\n", nbytes, file_length);

    for (n_read = 0; n_read < file_length; ) {
        nbytes = read(_sock, file_buf+n_read, sizeof(file_buf)-n_read);
        n_read += nbytes;
        printf("n_read:%d\n", n_read);
    }

    root_val = json_parse_string_with_comments(file_buf);
    if (root_val == NULL) {
        MSG("ERROR: server-conf not a valid JSON string\n");
        return -1;
    } else {
        int x = _parse_SX1301_configuration("(from-server)", root_val);
        if (x != 0)
            return -1;

        printf("sx1301 config from server ok\n");
        x = parse_lorawan_configuration(root_val);
        if (x != 0)
            return -1;

        printf("lorawan config from server ok\n");
        json_value_free(root_val);
    }

    return 0;
}

int
write_to_server(uint8_t cmd, uint8_t* user_buf, uint16_t user_buf_length)
{
    uint8_t msg[512];
    uint16_t* msg_len = (uint16_t*)&msg[1];
    int nbytes;

    if (!connected_to_server) {
        printf("write_to_server() not connected\n");
        return -1;
    }

    msg[0] = cmd;
    *msg_len = user_buf_length + 3 + 6; // + header + mac-addr length
    msg[3] = mac_address[0];
    msg[4] = mac_address[1];
    msg[5] = mac_address[2];
    msg[6] = mac_address[3];
    msg[7] = mac_address[4];
    msg[8] = mac_address[5];

    if (*msg_len > sizeof(msg)) {
        printf("msg[] to small\n");
        return -1;
    }

    if (user_buf_length > 0 && user_buf != NULL)
        memcpy(msg+9, user_buf, user_buf_length);

    nbytes = write (_sock, msg, *msg_len);
    if (nbytes < 0) {
        if (errno == EPIPE) {
            printf("got EPIPE\n");
            connected_to_server = false;
            return -1;
        }
        perror ("sock-write");
        return -1;
    }
    if (nbytes != *msg_len) {
        fprintf(stderr, "incomplete server write:%d,%d\n", nbytes, *msg_len);
        return -1;
    }
    return 0;
}

volatile uint32_t wbr;

/* return true if done getting first pps */
bool
_first_pps(uint32_t trig_tstamp)
{
    uint32_t lgw_trigcnt_at_next_beacon;
    uint8_t beacon_period_mask = beacon_period - 1;
    uint32_t seconds_to_beacon = 0;

    if (gps_time_valid && time_ptr_->tv_sec != 0) {
        seconds_to_beacon = ((time_ptr_->tv_sec | beacon_period_mask) + 1) - time_ptr_->tv_sec;
    } else
        printf("gps_time_valid %d, %lu\n", gps_time_valid, time_ptr_->tv_sec);

    if (seconds_to_beacon < 4)
        return false; /* not too close to beacon ocurring */

    if (clock_gettime (CLOCK_MONOTONIC, &host_time_at_pps) == -1)
        perror ("clock_gettime");

    prev_trig_tstamp = trig_tstamp;

    pps_cnt = (beacon_period - seconds_to_beacon) + 1;
    printf("seconds_to_beacon:%u pps_cnt:%d\n", seconds_to_beacon, pps_cnt);
    if (pps_cnt >= beacon_period) {
        printf("%u = (%u - %u) + 1\n", pps_cnt, beacon_period, seconds_to_beacon);
        return -1;
    }
    lgw_trigcnt_at_next_beacon = trig_tstamp + (seconds_to_beacon * 1000000);
    printf(" lgw_trigcnt_at_next_beacon:%u\n", lgw_trigcnt_at_next_beacon);


    {   /* send initial beacon info to server */
        uint32_t buf_idx = 0;
        uint32_t lgw_trigcnt_at_next_beacon;
        uint32_t* u32_ptr;
        double* dptr;
        short* sptr;
        uint8_t buf[sizeof(uint32_t)+sizeof(struct coord_s)];
        uint32_t seconds_to_beacon = ((time_ptr_->tv_sec | beacon_period_mask) + 1) - time_ptr_->tv_sec;
        lgw_trigcnt_at_next_beacon = trig_tstamp + (seconds_to_beacon * 1000000);
        u32_ptr = (uint32_t*)buf;
        *u32_ptr = lgw_trigcnt_at_next_beacon;
        buf_idx += sizeof(uint32_t);

        dptr = (double*)&buf[buf_idx];
        *dptr = coord.lat;
        buf_idx += sizeof(double);

        dptr = (double*)&buf[buf_idx];
        *dptr = coord.lon;
        buf_idx += sizeof(double);

        sptr = (short*)&buf[buf_idx];
        *sptr = coord.alt;
        buf_idx += sizeof(short);

        if (beacon_info.us915) {
            unsigned int sp = time_ptr_->tv_sec / beacon_period;
            buf[buf_idx++] = sp & 7;
        } else
            buf[buf_idx++] = 0;

        write_to_server(BEACON_INIT, buf, buf_idx);
    }

    return true;
}

#if 0
int
first_pps()
{
    uint8_t beacon_period_mask = beacon_period - 1;
    uint32_t first_trig_tstamp;
    uint32_t trig_tstamp;
    uint32_t seconds_to_beacon = 0;

    do {
        lgw_get_trigcnt(&first_trig_tstamp);
        do {
            gps_ubx_uart_service();
            usleep(10000);
            lgw_get_trigcnt(&trig_tstamp);
        } while (first_trig_tstamp == trig_tstamp);
        if (clock_gettime (CLOCK_MONOTONIC, &host_time_at_pps) == -1)
            perror ("clock_gettime");

        /* first write to prev_trig_tstamp */
        prev_trig_tstamp = trig_tstamp;

        if (gps_time_valid && time_ptr_->tv_sec != 0) {
            seconds_to_beacon = ((time_ptr_->tv_sec | beacon_period_mask) + 1) - time_ptr_->tv_sec;
        } else
            printf("gps_time_valid %d, %lu\n", gps_time_valid, time_ptr_->tv_sec);
    } while (seconds_to_beacon < 4);    // cant start too close to beacon

    pps_cnt = (beacon_period - seconds_to_beacon) + 1;
    printf("seconds_to_beacon:%u pps_cnt:%d\n", seconds_to_beacon, pps_cnt);
    if (pps_cnt >= beacon_period) {
        printf("%u = (%u - %u) + 1\n", pps_cnt, beacon_period, seconds_to_beacon);
        return -1;
    }
    lgw_trigcnt_at_next_beacon = trig_tstamp + (seconds_to_beacon * 1000000);
    printf(" lgw_trigcnt_at_next_beacon:%u\n", lgw_trigcnt_at_next_beacon);


    {   /* send initial beacon info to server */
        uint32_t buf_idx = 0;
        uint32_t lgw_trigcnt_at_next_beacon;
        uint32_t* u32_ptr;
        double* dptr;
        short* sptr;
        uint8_t buf[sizeof(uint32_t)+sizeof(struct coord_s)];
        uint32_t seconds_to_beacon = ((time_ptr_->tv_sec | beacon_period_mask) + 1) - time_ptr_->tv_sec;
        lgw_trigcnt_at_next_beacon = trig_tstamp + (seconds_to_beacon * 1000000);
        u32_ptr = (uint32_t*)buf;
        *u32_ptr = lgw_trigcnt_at_next_beacon;
        buf_idx += sizeof(uint32_t);

        dptr = (double*)&buf[buf_idx];
        *dptr = coord.lat;
        buf_idx += sizeof(double);

        dptr = (double*)&buf[buf_idx];
        *dptr = coord.lon;
        buf_idx += sizeof(double);

        sptr = (short*)&buf[buf_idx];
        *sptr = coord.alt;
        buf_idx += sizeof(short);

        if (beacon_info.us915) {
            unsigned int sp = time_ptr_->tv_sec / beacon_period;
            buf[buf_idx++] = sp & 7;
        } else
            buf[buf_idx++] = 0;

        write_to_server(BEACON_INIT, buf, buf_idx);
    }

    return 0;
}
#endif /* #if 0 */

static uint16_t BeaconCrc( uint8_t *buffer, uint16_t length )
{
    uint16_t i;
    // The CRC calculation follows CCITT
    const uint16_t polynom = 0x1021;
    // CRC initial value
    uint16_t crc = 0x0000;

    if( buffer == NULL )
    {
        return 0;
    }

    for( i = 0; i < length; ++i )
    {
        uint16_t j;
        crc ^= ( uint16_t ) buffer[i] << 8;
        for( j = 0; j < 8; ++j )
        {
            crc = ( crc & 0x8000 ) ? ( crc << 1 ) ^ polynom : ( crc << 1 );
        }
    }

    return crc;
}

uint8_t
load_beacon(uint32_t seconds)
{
    struct lgw_pkt_tx_s tx_pkt;
    uint16_t crc0, crc1;
    uint8_t rfuOffset1 = beacon_info.rfuOffset1;
    uint8_t rfuOffset2 = beacon_info.rfuOffset2;
    uint8_t chan_ret;

    if (beacon_info.us915) {
        unsigned int sp = seconds / beacon_period;
        chan_ret = sp & 7;
        beacon_info.hz = 923300000 + (600000 * chan_ret);
    } else
        chan_ret = 0;

    tx_pkt.size = beacon_info.size;

    tx_pkt.payload[2 + rfuOffset1] = seconds;
    tx_pkt.payload[3 + rfuOffset1] = seconds >> 8;
    tx_pkt.payload[4 + rfuOffset1] = seconds >> 16;
    tx_pkt.payload[5 + rfuOffset1] = seconds >> 24;

    crc0 = BeaconCrc( tx_pkt.payload, 6 + rfuOffset1 );
    //printf("crc0:%04x to [%d],[%d]\n", crc0, 6+rfuOffset1, 7+rfuOffset1);
    tx_pkt.payload[6 + rfuOffset1] = crc0;
    tx_pkt.payload[7 + rfuOffset1] = crc0 >> 8;

    crc1 = BeaconCrc( &tx_pkt.payload[8 + rfuOffset1], 7 + rfuOffset2 );
    tx_pkt.payload[15 + rfuOffset1 + rfuOffset2] = crc1;
    tx_pkt.payload[16 + rfuOffset1 + rfuOffset2] = crc1 >> 8;

    tx_pkt.modulation = MOD_LORA;
    tx_pkt.coderate = CR_LORA_4_5;
    tx_pkt.bandwidth = beacon_info.bw;
    tx_pkt.datarate = beacon_info.sf;
    if (skip_beacon_cnt > 0) {
        /* cause beacon to fail by sending on wrong frequency */
        printf("skip_beacon_cnt:%d\n", skip_beacon_cnt);
        skip_beacon_cnt--;
        tx_pkt.freq_hz = beacon_info.hz + 450000;
    } else {
        tx_pkt.freq_hz = beacon_info.hz;
    }

    print_hal_sf(tx_pkt.datarate);
    print_hal_bw(tx_pkt.bandwidth);
    printf(" %.1fMHz\n", tx_pkt.freq_hz / 1e6);

    tx_pkt.tx_mode = ON_GPS;
    tx_pkt.rf_chain = tx_rf_chain;
    tx_pkt.rf_power = 20;   // TODO
    tx_pkt.invert_pol = false;
    tx_pkt.preamble = 10;
    tx_pkt.no_crc = true;
    tx_pkt.no_header = true;   // beacon is fixed length

    printf("BEACON: ");
    if (lgw_send(tx_pkt) == LGW_HAL_ERROR) {
        printf("lgw_send() failed\n");
    }

    return chan_ret;
}

void
notify_server(uint32_t tstamp_at_beacon_tx, uint32_t beacon_sent_seconds, uint8_t ch)
{
    double* dptr;
    short* sptr;
    uint32_t* u32_ptr;
    uint8_t user_buf[8 + sizeof(struct coord_s)];
    unsigned int buf_idx = 0;

    printf("notify_server(%u, %u)\n", tstamp_at_beacon_tx, beacon_sent_seconds);

    u32_ptr = (uint32_t*)&user_buf[buf_idx];
    *u32_ptr = tstamp_at_beacon_tx;
    buf_idx += sizeof(uint32_t);

    u32_ptr = (uint32_t*)&user_buf[buf_idx];
    *u32_ptr = beacon_sent_seconds;
    buf_idx += sizeof(uint32_t);

    dptr = (double*)&user_buf[buf_idx];
    *dptr = coord.lat;
    buf_idx += sizeof(double);

    dptr = (double*)&user_buf[buf_idx];
    *dptr = coord.lon;
    buf_idx += sizeof(double);

    sptr = (short*)&user_buf[buf_idx];
    *sptr = coord.alt;
    buf_idx += sizeof(short);

    user_buf[buf_idx++] = ch;   // which channel was beacon sent on

    if (write_to_server(BEACON_INDICATION, user_buf, buf_idx) < 0) {
        printf("notify server write failed\n");
    }
}

int pps(uint32_t trig_tstamp)
{
    static bool save_next_tstamp = false;
    uint8_t beacon_period_mask = beacon_period - 1;
    static struct timespec beacon_sent_seconds;
    static uint8_t ch = 0;

    if (++pps_cnt > beacon_period) {
        printf("pps_cnt:%u\n", pps_cnt);
        return -1;
    }

    if (!gps_time_valid)
        printf("[31mgps sec:%lu[0m\n", time_ptr_->tv_sec);

    if ((time_ptr_->tv_sec & beacon_period_mask) == beacon_period_mask) {
        printf("load_beacon pps sec:%lx   tstamp:%u, ", time_ptr_->tv_sec, trig_tstamp);
        if (gps_time_valid)
            printf(" pps_cnt:%u\n", pps_cnt);
        else
            printf(" [31mpps_cnt:%u[0m\n", pps_cnt);

        pps_cnt = 0;
        beacon_sent_seconds.tv_sec = time_ptr_->tv_sec + 1;
        ch = load_beacon(beacon_sent_seconds.tv_sec);
        save_next_tstamp = true;
    } else if ((time_ptr_->tv_sec & beacon_period_mask) == (beacon_period_mask-2)) {
        beacon_guard = true;
        printf("beacon_guard @ %d, sec:%lx\n", pps_cnt, time_ptr_->tv_sec);
    } else if (save_next_tstamp) {
        uint32_t tstamp_at_beacon_tx;
        save_next_tstamp = false;
        tstamp_at_beacon_tx = trig_tstamp;
        printf("tstamp_at_beacon_tx:%u, pps_cnt:%d sec:%lx\n",  tstamp_at_beacon_tx, pps_cnt, time_ptr_->tv_sec);

        notify_server(tstamp_at_beacon_tx, beacon_sent_seconds.tv_sec, ch);

        beacon_guard = false;
    }


    return 0;
}

void
service_pps()
{
    uint32_t trig_tstamp;
    struct timespec saved;

    /*int i =*/ lgw_get_trigcnt(&trig_tstamp);
    if (prev_trig_tstamp == trig_tstamp)
        return;

    saved.tv_sec = host_time_at_pps.tv_sec;
    saved.tv_nsec = host_time_at_pps.tv_nsec;
    if (clock_gettime (CLOCK_MONOTONIC, &host_time_at_pps) == -1)
        perror ("clock_gettime");

    if (!gps_time_valid)
        printf("pps diff:%f\n", difftimespec(host_time_at_pps, saved));

    prev_trig_tstamp = trig_tstamp;
    /* pps pulse occurred */
    //printf("pps %u\n", trig_tstamp);
    pps(trig_tstamp);
}

int check_sx1301_config()
{
    int i;

    MSG("#### check_sx1301_config(): lorawan_public %d, clksrc %d txlut.size:%d #####\n", _boardconf.lorawan_public, _boardconf.clksrc, txlut.size);

    /* all parameters parsed, submitting configuration to the HAL */
    if (txlut.size > 0) {
        MSG("INFO: Configuring TX LUT with %u indexes\n", txlut.size);
        if (lgw_txgain_setconf(&txlut) != LGW_HAL_SUCCESS) {
            MSG("ERROR: Failed to configure concentrator TX Gain LUT\n");
            return -1;
        }
    } else {
        MSG("WARNING: No TX gain LUT defined\n");
    }

    /* all parameters parsed, submitting configuration to the HAL */
    if (lgw_board_setconf(_boardconf) != LGW_HAL_SUCCESS) {
        MSG("ERROR: Failed to configure board\n");
        return -1;
    }

    if (_lbtconf.enable == true) {
       /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_lbt_setconf(_lbtconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: Failed to configure LBT\n");
            return -1;
        }
    }

    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        if (_rfconfs[i].enable) {
            if (_rfconfs[i].type == LGW_RADIO_TYPE_SX1255)
                MSG("INFO SX1255: ");
            else if (_rfconfs[i].type == LGW_RADIO_TYPE_SX1257)
                MSG("INFO SX1257: ");
            MSG("radio %i enabled, center frequency %u, RSSI offset %f, tx enabled %d, tx_notch_freq %u\n", i, _rfconfs[i].freq_hz, _rfconfs[i].rssi_offset, _rfconfs[i].tx_enable, _rfconfs[i].tx_notch_freq);
            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_rxrf_setconf(i, _rfconfs[i]) != LGW_HAL_SUCCESS) {
                MSG("ERROR: invalid configuration for radio %i\n", i);
                return -1;
            }
        }
    }

    for (i = 0; i < LGW_MULTI_NB; ++i) {
        if (_ifconfs[i].enable) {
            int r = _ifconfs[i].rf_chain;
            MSG("INFO: Lora multi-SF ch%i>  radio %i, IF %iHz=%uhz, 125 kHz bw, SF 7 to 12\n", i, r, _ifconfs[i].freq_hz, _rfconfs[r].freq_hz + _ifconfs[i].freq_hz);
            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_rxif_setconf(i, _ifconfs[i]) != LGW_HAL_SUCCESS) {
                MSG("ERROR: invalid configuration for Lora multi-SF channel %i\n", i);
                return -1;
            }
        }
    }

    if (_ifconf_std.enable) {
        unsigned int r, bw = 0, sf;
        switch (_ifconf_std.bandwidth) {
            case BW_500KHZ: bw = 500; break;
            case BW_250KHZ: bw = 250; break;
            case BW_125KHZ: bw = 125; break;
            case BW_UNDEFINED: bw = 0; break;
        }
        switch (_ifconf_std.datarate) {
            case DR_LORA_SF7: sf = 7; break;
            case DR_LORA_SF8: sf = 8; break;
            case DR_LORA_SF9: sf = 9; break;
            case DR_LORA_SF10: sf = 10; break;
            case DR_LORA_SF11: sf = 11; break;
            case DR_LORA_SF12: sf = 12; break;
            default: sf = 0; break;
        }
        r = _ifconf_std.rf_chain;
        MSG("INFO: Lora std channel> radio %i, IF %iHz=%u, %uKHz bw, SF %u\n", r, _ifconf_std.freq_hz, _rfconfs[r].freq_hz + _ifconf_std.freq_hz,bw, sf);
        if (lgw_rxif_setconf(8, _ifconf_std) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for Lora standard channel\n");
            return -1;
        }
    }

    if (_ifconf_fsk.enable) {
        MSG("INFO: FSK channel> radio %i, IF %i Hz, bw:0x%x, %u bps datarate\n", _ifconf_fsk.rf_chain, _ifconf_fsk.freq_hz, _ifconf_fsk.bandwidth, _ifconf_fsk.datarate);
        if (lgw_rxif_setconf(9, _ifconf_fsk) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for FSK channel\n");
            return -1;
        }
    }

    return 0;
}

#define NB_PKT_MAX      8 /* max number of packets per fetch/send cycle */
int
uplink_service()
{
    int i;
    int nb_pkt;
    struct lgw_pkt_rx_s rxpkt[NB_PKT_MAX]; /* array containing inbound packets + metadata */

    if (!lgw_started)
        return 0;

    nb_pkt = lgw_receive(NB_PKT_MAX, rxpkt);
    if (nb_pkt == LGW_HAL_ERROR) {
        MSG("ERROR: [up] failed packet fetch\n");
        return -1;
    } else if (nb_pkt == 0) {
        /* nothing received at this time */
        return 0;
    }

    for (i = 0; i < nb_pkt; ++i) {
        uint16_t* u16_ptr;
        uint32_t* u32_ptr;
        unsigned int buf_idx = 0;
        uint8_t user_buf[sizeof(struct lgw_pkt_rx_s)];
        struct lgw_pkt_rx_s *p = &rxpkt[i];
#if 0
    uint32_t    freq_hz;        /*!> central frequency of the IF chain */
    uint8_t     if_chain;       /*!> by which IF chain was packet received */
    uint8_t     status;         /*!> status of the received packet */
    uint32_t    count_us;       /*!> internal concentrator counter for timestamping, 1 microsecond resolution */
    uint8_t     rf_chain;       /*!> through which RF chain the packet was received */
    uint8_t     modulation;     /*!> modulation used by the packet */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint32_t    datarate;       /*!> RX datarate of the packet (SF for LoRa) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
    float       rssi;           /*!> average packet RSSI in dB */
    float       snr;            /*!> average packet SNR, in dB (LoRa only) */
    float       snr_min;        /*!> minimum packet SNR, in dB (LoRa only) */
    float       snr_max;        /*!> maximum packet SNR, in dB (LoRa only) */
    uint16_t    crc;            /*!> CRC that was received in the payload */
    uint16_t    size;           /*!> payload size in bytes */
    uint8_t     payload[256];   /*!> buffer containing the payload */
#endif /* #if 0 */
        switch(p->status) {
            case STAT_CRC_OK:
                printf("rx STAT_CRC_OK\n");
                if (!fwd_valid_pkt) {
                    continue;
                }
                break;
            case STAT_CRC_BAD:
                if (p->snr > -3)
                    printf("rx STAT_CRC_BAD snr:%.1f rssi:%.0f\n", p->snr, p->rssi);
                if (!fwd_error_pkt) {
                    continue;
                }
                break;
            case STAT_NO_CRC:
                printf("rx STAT_NO_CRC\n");
                if (!fwd_nocrc_pkt) {
                    continue;
                }
                break;
            default:
                printf("rx status:<%d>\n", p->status);
                continue;
        } // ..switch(p->status)

        u32_ptr = (uint32_t*)&user_buf[buf_idx];
        *u32_ptr = p->freq_hz;
        buf_idx += sizeof(p->freq_hz);
        printf("rx freq_hz:%u ", *u32_ptr);

        user_buf[buf_idx++] = p->if_chain;
        user_buf[buf_idx++] = p->status;

        u32_ptr = (uint32_t*)&user_buf[buf_idx];
        *u32_ptr = p->count_us;
        buf_idx += sizeof(p->count_us);

        user_buf[buf_idx++] = p->rf_chain;
        user_buf[buf_idx++] = p->modulation;
        user_buf[buf_idx++] = p->bandwidth;

        u32_ptr = (uint32_t*)&user_buf[buf_idx];
        *u32_ptr = p->datarate;
        buf_idx += sizeof(p->datarate);

        user_buf[buf_idx++] = p->coderate;

        memcpy(&user_buf[buf_idx], &p->rssi, sizeof(p->rssi));
        buf_idx += sizeof(p->rssi);

        memcpy(&user_buf[buf_idx], &p->snr, sizeof(p->snr));
        buf_idx += sizeof(p->snr);
        printf("snr:%.1f rssi:%.0f ", p->snr, p->rssi);

        memcpy(&user_buf[buf_idx], &p->snr_min, sizeof(p->snr_min));
        buf_idx += sizeof(p->snr_min);

        memcpy(&user_buf[buf_idx], &p->snr_max, sizeof(p->snr_max));
        buf_idx += sizeof(p->snr_max);

        u16_ptr = (uint16_t*)&user_buf[buf_idx];
        *u16_ptr = p->crc;
        buf_idx += sizeof(p->crc);

        u16_ptr = (uint16_t*)&user_buf[buf_idx];
        *u16_ptr = p->size;
        buf_idx += sizeof(p->size);

        if (p->size > sizeof(p->payload)) {
            printf("oversized %u\n", p->size);
            continue;
        } else
            printf("size:%u\n", p->size);


        memcpy(user_buf+buf_idx, p->payload, p->size);
        buf_idx += p->size;

        write_to_server(UPLINK, user_buf, buf_idx);
    } // ..for (i = 0; i < nb_pkt; ++i)

    return 0;
}

int
main (int argc, char **argv)
{
    uint32_t first_trig_tstamp;
    bool get_first_pps = false;
    bool receive_config = false;
    unsigned int server_retry_wait_cnt = 0;
    fd_set active_fd_set;
    int maxfd, gps_tty_fd = -1; /* file descriptor of the GPS TTY port */
    int i, x;
    JSON_Value *root_val;
    char *global_cfg_path= "../global_conf.json"; /* contain global (typ. network-wide) configuration */
    char *local_cfg_path = "../local_conf.json"; /* contain node specific configuration, overwrite global parameters for parameters that are defined in both */
    char *debug_cfg_path = "../debug_conf.json"; /* if present, all other configuration files are ignored */

    if (get_host_unique_id() < 0) {
        return -1;
    }

    /* get broken pipe errno instead of SIGPIPE */
    signal(SIGPIPE, SIG_IGN);

    /* clear gateway configuration */
    memset(&_boardconf, 0, sizeof _boardconf); /* initialize configuration structure */
    memset(&_lbtconf, 0, sizeof _lbtconf); /* initialize configuration structure */
    memset(&_rfconfs, 0, sizeof _rfconfs); /* initialize configuration structure */
    memset(&_ifconfs, 0, sizeof _ifconfs); /* initialize configuration structure */
    memset(&_ifconf_std, 0, sizeof _ifconf_std); /* initialize configuration structure */
    memset(&_ifconf_fsk, 0, sizeof _ifconf_fsk); /* initialize configuration structure */

    /* load configuration files */
    if (access(debug_cfg_path, R_OK) == 0) { /* if there is a debug conf, parse only the debug conf */
        MSG("INFO: found debug configuration file %s, parsing it\n", debug_cfg_path);
        MSG("INFO: other configuration files will be ignored\n");
        root_val = json_parse_file_with_comments(debug_cfg_path);
        if (root_val == NULL) {
            MSG("ERROR: %s is not a valid JSON file\n", debug_cfg_path);
            exit(EXIT_FAILURE);
        }
        x = _parse_SX1301_configuration(debug_cfg_path, root_val);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        json_value_free(root_val);

        x = parse_gateway_configuration(debug_cfg_path);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
    } else if (access(global_cfg_path, R_OK) == 0) { /* if there is a global conf, parse it and then try to parse local conf  */
        MSG("INFO: found global configuration file %s, parsing it\n", global_cfg_path);
        root_val = json_parse_file_with_comments(global_cfg_path);
        if (root_val == NULL) {
            MSG("ERROR: %s is not a valid JSON file\n", global_cfg_path);
            exit(EXIT_FAILURE);
        }
        x = _parse_SX1301_configuration(global_cfg_path, root_val);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        json_value_free(root_val);

        x = parse_gateway_configuration(global_cfg_path);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        if (access(local_cfg_path, R_OK) == 0) {
            MSG("INFO: found local configuration file %s, parsing it\n", local_cfg_path);
            MSG("INFO: redefined parameters will overwrite global parameters\n");
            root_val = json_parse_file_with_comments(local_cfg_path);
            if (root_val == NULL) {
                MSG("ERROR: %s is not a valid JSON file\n", local_cfg_path);
                exit(EXIT_FAILURE);
            }
            _parse_SX1301_configuration(local_cfg_path, root_val);
            json_value_free(root_val);

            parse_gateway_configuration(local_cfg_path);
        }
    } else if (access(local_cfg_path, R_OK) == 0) { /* if there is only a local conf, parse it and that's all */
        MSG("INFO: found local configuration file %s, parsing it\n", local_cfg_path);
        root_val = json_parse_file_with_comments(local_cfg_path);
        if (root_val == NULL) {
            MSG("ERROR: %s is not a valid JSON file\n", local_cfg_path);
            exit(EXIT_FAILURE);
        }
        x = _parse_SX1301_configuration(local_cfg_path, root_val);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        json_value_free(root_val);

        x = parse_gateway_configuration(local_cfg_path);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
    } else {
        MSG("ERROR: [main] failed to find any configuration file named %s, %s OR %s\n", global_cfg_path, local_cfg_path, debug_cfg_path);
        exit(EXIT_FAILURE);
    }

    FD_ZERO(&active_fd_set);

    /* Start GPS a.s.a.p., to allow it to lock */
    if (gps_tty_path[0] != '\0') { /* do not try to open GPS device if no path set */
        i = lgw_gps_enable(gps_tty_path, "ubx7", 0, &gps_tty_fd); /* HAL only supports u-blox 7 for now */
        if (i != LGW_GPS_SUCCESS) {
            printf("WARNING: [main] impossible to open %s for GPS sync (check permissions)\n", gps_tty_path);
            return -1;  /* this is class-B forwarder */
        } else {
            printf("INFO: [main] TTY port %s open for GPS synchronization\n", gps_tty_path);
            FD_SET(gps_tty_fd, &active_fd_set);
            maxfd = gps_tty_fd + 1;
            printf("gps_tty_fd:%d\n", gps_tty_fd);
        }
    } else {
        printf("WARNING: [main] GPS not opened\n");
        return -1;  /* this is class-B forwarder */
    }

    for (;;) {
        struct timeval timeout;
        int retval;

        FD_SET(gps_tty_fd, &active_fd_set);
        if (connected_to_server)
            FD_SET(_sock, &active_fd_set);

        timeout.tv_sec = 0;
        timeout.tv_usec = run_rate_usec;
        retval = select(maxfd, &active_fd_set, NULL, NULL, &timeout);
        if (retval < 0) { 
            perror("select");
            return -1;
        } else if (FD_ISSET(gps_tty_fd, &active_fd_set)) {
            gps_service(gps_tty_fd);
        } else if (connected_to_server) {
            if (FD_ISSET(_sock, &active_fd_set)) {
                if (receive_config) {
                    if (get_server_config() == 0) {
                        printf("get_server_config() OK\n");
                        receive_config = false;

                        if (check_sx1301_config() < 0) {
                            /* check failed */
                            printf("config check failed\n");
                            FD_CLR(_sock, &active_fd_set);
                            close (_sock);
                            connected_to_server = false;
                            server_retry_wait_cnt = 400;
                        } else {
                            /* check ok */
                            if (lgw_start() == LGW_HAL_SUCCESS) {
                                MSG("INFO: [main] concentrator started, packet can now be received\n");
                                lgw_started = true;
                                get_first_pps = true;
                                lgw_get_trigcnt(&first_trig_tstamp);
                            } else {
                                lgw_started = false;
                                MSG("ERROR: [main] failed to start the concentrator\n");
                                FD_CLR(_sock, &active_fd_set);
                                close (_sock);
                                connected_to_server = false;
                                server_retry_wait_cnt = 400;
                            }
                        }
                    } else {
                        printf("get_server_config() failed\n");
                    }
                } else {
                    if (downlink_service() < 0) {
                        lgw_stop();
                        lgw_started = false;
                        FD_CLR(_sock, &active_fd_set);
                        close (_sock);
                        connected_to_server = false;
                        server_retry_wait_cnt = 400;
                    }
                }
            }
        } else if (retval != 0) {
            printf("TODO other fd\n");
        }

        if (!connected_to_server) {
            if (server_retry_wait_cnt > 0) {
                if (--server_retry_wait_cnt > 0)
                    continue;
            }
            _sock = connect_to_server();
            if (_sock >= 0) {
                uint32_t u32 = PROT_VER;
                printf("server-connected\n");
                connected_to_server = true;

                /* request config from server */
                write_to_server(REQ_CONF, (uint8_t*)&u32, sizeof(uint32_t));

                receive_config = true;
                FD_SET(_sock, &active_fd_set);
                maxfd = MAX(gps_tty_fd, _sock) + 1;
            } else {
                server_retry_wait_cnt = 400;
            }
        } else if (get_first_pps) {
            uint32_t trig_tstamp;
            lgw_get_trigcnt(&trig_tstamp);
            if (trig_tstamp != first_trig_tstamp) {
                if (_first_pps(trig_tstamp)) {
                    get_first_pps = false; // done getting first pps
                    printf("got first pps\n");
                }
            }
        } else if (lgw_started) {
            /* connected to server and concentrator running */
            service_pps();

            uplink_service();
            gw_tx_service(false);

        }
    } // ..for (;;)

}

