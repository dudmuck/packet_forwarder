/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Configure Lora concentrator and forward packets to a server
    Use GPS for packet timestamping.
    Send a becon at a regular interval without server intervention

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>         /* C99 types */
#include <stdbool.h>        /* bool type */
#include <stdio.h>          /* printf, fprintf, snprintf, fopen, fputs */

#include <string.h>         /* memset */
#include <signal.h>         /* sigaction */
#include <time.h>           /* time, clock_gettime, strftime, gmtime */
#include <sys/time.h>       /* timeval */
#include <unistd.h>         /* getopt, access */
#include <stdlib.h>         /* atoi, exit */
#include <errno.h>          /* error messages */
#include <math.h>           /* modf */
#include <assert.h>

#include <sys/socket.h>     /* socket specific definitions */
#include <netinet/in.h>     /* INET constants and stuff */
#include <arpa/inet.h>      /* IP address conversion stuff */
#include <netdb.h>          /* gai_strerror */


#include "trace.h"
#include "parson.h"
#include "base64.h"
#include "lorawan.h"
#include "lorawan_bands.h"
#ifdef ENABLE_CLASS_B
    #include "loragw_gps.h"
#endif
#include "loragw_aux.h"
#include "loragw_reg.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#define STRINGIFY(x)    #x
#define STR(x)          STRINGIFY(x)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#ifndef VERSION_STRING
  #define VERSION_STRING "undefined"
#endif

#define DEFAULT_SERVER      127.0.0.1   /* hostname also supported */
#define DEFAULT_PORT_UP     1780
#define DEFAULT_PORT_DW     1782
#define DEFAULT_KEEPALIVE   5           /* default time interval for downstream keep-alive packet */
#define DEFAULT_STAT        30          /* default time interval for statistics */
#define PUSH_TIMEOUT_MS     100
#define PULL_TIMEOUT_MS     200
#define GPS_REF_MAX_AGE     30          /* maximum admitted delay in seconds of GPS loss before considering latest GPS sync unusable */
#define FETCH_SLEEP_MS      10          /* nb of ms waited when a fetch return no packets */
#define BEACON_POLL_MS      50          /* time in ms between polling of beacon TX status */

#define PROTOCOL_VERSION    2           /* v1.3 */

#define XERR_INIT_AVG       128         /* nb of measurements the XTAL correction is averaged on as initial value */
#define XERR_FILT_COEF      256         /* coefficient for low-pass XTAL error tracking */

#define PKT_PUSH_DATA   0
#define PKT_PUSH_ACK    1
#define PKT_PULL_DATA   2
#define PKT_PULL_RESP   3
#define PKT_PULL_ACK    4
#define PKT_TX_ACK      5

#define NB_PKT_MAX      8 /* max number of packets per fetch/send cycle */

#define MIN_LORA_PREAMB 6 /* minimum Lora preamble length for this application */
#define STD_LORA_PREAMB 8
#define MIN_FSK_PREAMB  3 /* minimum FSK preamble length for this application */
#define STD_FSK_PREAMB  4

#define STATUS_SIZE     200
#define TX_BUFF_SIZE    ((540 * NB_PKT_MAX) + 30 + STATUS_SIZE)

/* -------------------------------------------------------------------------- */
bool verbose = false;
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
volatile bool exit_sig = false; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
volatile bool quit_sig = false; /* 1 -> application terminates without shutting down the hardware */

/* packets filtering configuration variables */
static bool fwd_valid_pkt = true; /* packets with PAYLOAD CRC OK are forwarded */
static bool fwd_error_pkt = false; /* packets with PAYLOAD CRC ERROR are NOT forwarded */
static bool fwd_nocrc_pkt = false; /* packets with NO PAYLOAD CRC are NOT forwarded */

/* statistics collection configuration variables */
static unsigned stat_interval = DEFAULT_STAT; /* time interval (in sec) at which statistics are collected and displayed */

/* hardware access control and correction */
pthread_mutex_t mx_concent = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */

#ifdef ENABLE_CLASS_B
/* Reference coordinates, for broadcasting (beacon) */
static struct coord_s reference_coord;

static char gps_tty_path[64] = "\0"; /* path of the TTY port GPS is connected on */
static int gps_tty_fd = -1; /* file descriptor of the GPS TTY port */
static bool gps_enabled = false; /* is GPS enabled on that gateway ? */
static bool gps_ref_valid; /* is GPS reference acceptable (ie. not too old) */

void thread_gps(void);
#endif    /* ENABLE_CLASS_B */

/* beacon parameters */
static uint32_t beacon_period = 0; /* set beaconing period, must be a sub-multiple of 86400, the nb of sec in a day */
static uint32_t beacon_freq_hz = 0; /* TX beacon frequency, in Hz */

/* Gateway specificities */
static int8_t antenna_gain = 0;

/* TX capabilities */
static struct lgw_tx_gain_lut_s txlut; /* TX gain table */
static uint32_t tx_freq_min[LGW_RF_CHAIN_NB]; /* lowest frequency supported by TX chain */
static uint32_t tx_freq_max[LGW_RF_CHAIN_NB]; /* highest frequency supported by TX chain */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

static int parse_SX1301_configuration(const char * conf_file);

static int parse_gateway_configuration(const char * conf_file);

//static uint16_t crc_ccit(const uint8_t * data, unsigned size);

double difftimespec(struct timespec end, struct timespec beginning);

/* threads */
void thread_up(void);
bool beacon_guard = false;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = true;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = true;
    }
    return;
}

static int parse_SX1301_configuration(const char * conf_file) {
    int i;
    char param_name[32]; /* used to generate variable parameter names */
    const char *str; /* used to store string value from JSON object */
    const char conf_obj_name[] = "SX1301_conf";
    JSON_Value *root_val = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_lbt_obj = NULL;
    JSON_Value *val = NULL;
    JSON_Array *conf_array = NULL;
    JSON_Object *conf_lbtchan_obj = NULL;
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_lbt_s lbtconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;
    uint32_t sf, bw, fdev, cf_hz[2];

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
        MSG("INFO: %s does contain a JSON object named %s, parsing SX1301 parameters\n", conf_file, conf_obj_name);
    }

    /* set board configuration */
    memset(&boardconf, 0, sizeof boardconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "lorawan_public"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.lorawan_public = (bool)json_value_get_boolean(val);
    } else {
        MSG("WARNING: Data type for lorawan_public seems wrong, please check\n");
        boardconf.lorawan_public = false;
    }
    val = json_object_get_value(conf_obj, "clksrc"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONNumber) {
        boardconf.clksrc = (uint8_t)json_value_get_number(val);
    } else {
        MSG("WARNING: Data type for clksrc seems wrong, please check\n");
        boardconf.clksrc = 0;
    }
    MSG("INFO: lorawan_public %d, clksrc %d\n", boardconf.lorawan_public, boardconf.clksrc);
    /* all parameters parsed, submitting configuration to the HAL */
    if (lgw_board_setconf(boardconf) != LGW_HAL_SUCCESS) {
        MSG("WARNING: Failed to configure board\n");
    }

    /* set LBT configuration */
    memset(&lbtconf, 0, sizeof lbtconf); /* initialize configuration structure */
    conf_lbt_obj = json_object_get_object(conf_obj, "lbt_cfg"); /* fetch value (if possible) */
    if (conf_lbt_obj == NULL) {
        MSG("INFO: no configuration for LBT\n");
    } else {
        val = json_object_get_value(conf_lbt_obj, "enable"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONBoolean) {
            lbtconf.enable = (bool)json_value_get_boolean(val);
        } else {
            MSG("WARNING: Data type for lbt_cfg.enable seems wrong, please check\n");
            lbtconf.enable = false;
        }
        if (lbtconf.enable == true) {
            val = json_object_get_value(conf_lbt_obj, "rssi_target"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONNumber) {
                lbtconf.rssi_target = (int8_t)json_value_get_number(val);
            } else {
                MSG("WARNING: Data type for lbt_cfg.rssi_target seems wrong, please check\n");
                lbtconf.rssi_target = 0;
            }
            val = json_object_get_value(conf_lbt_obj, "sx127x_rssi_offset"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONNumber) {
                lbtconf.rssi_offset = (int8_t)json_value_get_number(val);
            } else {
                MSG("WARNING: Data type for lbt_cfg.sx127x_rssi_offset seems wrong, please check\n");
                lbtconf.rssi_offset = 0;
            }
            /* set LBT channels configuration */
            conf_array = json_object_get_array(conf_lbt_obj, "chan_cfg");
            if (conf_array != NULL) {
                lbtconf.nb_channel = json_array_get_count( conf_array );
                MSG("INFO: %u LBT channels configured\n", lbtconf.nb_channel);
            }
            for (i = 0; i < (int)lbtconf.nb_channel; i++) {
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
                    lbtconf.channels[i].freq_hz = (uint32_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for lbt_cfg.channels[%d].freq_hz seems wrong, please check\n", i);
                    lbtconf.channels[i].freq_hz = 0;
                }

                /* Channel scan time */
                val = json_object_dotget_value(conf_lbtchan_obj, "scan_time_us"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    lbtconf.channels[i].scan_time_us = (uint16_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for lbt_cfg.channels[%d].scan_time_us seems wrong, please check\n", i);
                    lbtconf.channels[i].scan_time_us = 0;
                }
            }

            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_lbt_setconf(lbtconf) != LGW_HAL_SUCCESS) {
                MSG("ERROR: Failed to configure LBT\n");
                return -1;
            }
        } else {
            MSG("INFO: LBT is disabled\n");
        }
    }

    /* set antenna gain configuration */
    val = json_object_get_value(conf_obj, "antenna_gain"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONNumber) {
        antenna_gain = (int8_t)json_value_get_number(val);
    } else {
        MSG("WARNING: Data type for antenna_gain seems wrong, please check\n");
        antenna_gain = 0;
    }
    MSG("INFO: antenna_gain %d dBi\n", antenna_gain);

    /* set configuration for tx gains */
    memset(&txlut, 0, sizeof txlut); /* initialize configuration structure */
    for (i = 0; i < TX_GAIN_LUT_SIZE_MAX; i++) {
        snprintf(param_name, sizeof param_name, "tx_lut_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for tx gain lut %i\n", i);
            continue;
        }
        txlut.size++; /* update TX LUT size based on JSON object found in configuration file */
        /* there is an object to configure that TX gain index, let's parse it */
        snprintf(param_name, sizeof param_name, "tx_lut_%i.pa_gain", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONNumber) {
            txlut.lut[i].pa_gain = (uint8_t)json_value_get_number(val);
        } else {
            MSG("WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
            txlut.lut[i].pa_gain = 0;
        }
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
                } else {
            MSG("WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
                        txlut.lut[i].dig_gain = 0;
                }
                snprintf(param_name, sizeof param_name, "tx_lut_%i.mix_gain", i);
                val = json_object_dotget_value(conf_obj, param_name);
                if (json_value_get_type(val) == JSONNumber) {
                        txlut.lut[i].mix_gain = (uint8_t)json_value_get_number(val);
                } else {
            MSG("WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
                        txlut.lut[i].mix_gain = 0;
                }
                snprintf(param_name, sizeof param_name, "tx_lut_%i.rf_power", i);
                val = json_object_dotget_value(conf_obj, param_name);
                if (json_value_get_type(val) == JSONNumber) {
                        txlut.lut[i].rf_power = (int8_t)json_value_get_number(val);
                } else {
            MSG("WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
                        txlut.lut[i].rf_power = 0;
                }
    }
    /* all parameters parsed, submitting configuration to the HAL */
    MSG("INFO: Configuring TX LUT with %u indexes\n", txlut.size);
        if (lgw_txgain_setconf(&txlut) != LGW_HAL_SUCCESS) {
                MSG("WARNING: Failed to configure concentrator TX Gain LUT\n");
    }

    /* set configuration for RF chains */
    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        memset(&rfconf, 0, sizeof rfconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for radio %i\n", i);
            continue;
        }
        /* there is an object to configure that radio, let's parse it */
        snprintf(param_name, sizeof param_name, "radio_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            rfconf.enable = (bool)json_value_get_boolean(val);
        } else {
            rfconf.enable = false;
        }
        if (rfconf.enable == false) { /* radio disabled, nothing else to parse */
            MSG("INFO: radio %i disabled\n", i);
        } else  { /* radio enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "radio_%i.freq", i);
            rfconf.freq_hz = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_offset", i);
            rfconf.rssi_offset = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.type", i);
            str = json_object_dotget_string(conf_obj, param_name);
            if (!strncmp(str, "SX1255", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1255;
            } else if (!strncmp(str, "SX1257", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1257;
            } else {
                MSG("WARNING: invalid radio type: %s (should be SX1255 or SX1257)\n", str);
            }
            snprintf(param_name, sizeof param_name, "radio_%i.tx_enable", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.tx_enable = (bool)json_value_get_boolean(val);
                if (rfconf.tx_enable == true) {
                    tx_rf_chain = i;
                    /* tx is enabled on this rf chain, we need its frequency range */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_min", i);
                    tx_freq_min[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_max", i);
                    tx_freq_max[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    if ((tx_freq_min[i] == 0) || (tx_freq_max[i] == 0)) {
                        MSG("WARNING: no frequency range specified for TX rf chain %d\n", i);
                    }
                }
            } else {
                rfconf.tx_enable = false;
            }
            MSG("INFO: radio %i enabled (type %s), center frequency %u, RSSI offset %f, tx enabled %d\n", i, str, rfconf.freq_hz, rfconf.rssi_offset, rfconf.tx_enable);
            cf_hz[i] = rfconf.freq_hz;
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxrf_setconf(i, rfconf) != LGW_HAL_SUCCESS) {
            MSG("WARNING: invalid configuration for radio %i\n", i);
        }
    }

    /* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
    for (i = 0; i < LGW_MULTI_NB; ++i) {
        memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for Lora multi-SF channel %i\n", i);
            continue;
        }
        /* there is an object to configure that Lora multi-SF channel, let's parse it */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) { /* Lora multi-SF channel disabled, nothing else to parse */
            MSG("INFO: Lora multi-SF channel %i disabled\n", i);
        } else  { /* Lora multi-SF channel enabled, will parse the other parameters */
            float MHz;
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);
            // TODO: handle individual SF enabling and disabling (spread_factor)
            MHz = (cf_hz[ifconf.rf_chain] + ifconf.freq_hz) / 1e6;
            MSG("INFO: Lora multi-SF channel %i>  radio %i, IF %i Hz (%.1f), 125 kHz bw, SF 7 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz, MHz);
            uint32_t cf = cf_hz[ifconf.rf_chain] + ifconf.freq_hz;
            if (check_band_config(cf) < 0) {
                exit(EXIT_FAILURE);
            }
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxif_setconf(i, ifconf) != LGW_HAL_SUCCESS) {
            MSG("WARNING: invalid configuration for Lora multi-SF channel %i\n", i);
        }
    }

    /* set configuration for Lora standard channel */
    memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_Lora_std"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for Lora standard channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_Lora_std.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            MSG("INFO: Lora standard channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.bandwidth");
            switch(bw) {
                case 500000: ifconf.bandwidth = BW_500KHZ; break;
                case 250000: ifconf.bandwidth = BW_250KHZ; break;
                case 125000: ifconf.bandwidth = BW_125KHZ; break;
                default: ifconf.bandwidth = BW_UNDEFINED;
            }
            sf = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.spread_factor");
            switch(sf) {
                case  7: ifconf.datarate = DR_LORA_SF7;  break;
                case  8: ifconf.datarate = DR_LORA_SF8;  break;
                case  9: ifconf.datarate = DR_LORA_SF9;  break;
                case 10: ifconf.datarate = DR_LORA_SF10; break;
                case 11: ifconf.datarate = DR_LORA_SF11; break;
                case 12: ifconf.datarate = DR_LORA_SF12; break;
                default: ifconf.datarate = DR_UNDEFINED;
            }
            MSG("INFO: Lora std channel> radio %i, IF %i Hz, %u Hz bw, SF %u\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf);
            uint32_t cf = cf_hz[ifconf.rf_chain] + ifconf.freq_hz;
            if (check_band_config(cf) < 0) {
                exit(EXIT_FAILURE);
            }
        }
        if (lgw_rxif_setconf(8, ifconf) != LGW_HAL_SUCCESS) {
            MSG("WARNING: invalid configuration for Lora standard channel\n");
        }
    }

    /* set configuration for FSK channel */
    memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_FSK"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for FSK channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_FSK.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            MSG("INFO: FSK channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_FSK.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.bandwidth");
            fdev = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.freq_deviation");
            ifconf.datarate = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.datarate");

            /* if chan_FSK.bandwidth is set, it has priority over chan_FSK.freq_deviation */
            if ((bw == 0) && (fdev != 0)) {
                bw = 2 * fdev + ifconf.datarate;
            }
            if      (bw == 0)      ifconf.bandwidth = BW_UNDEFINED;
            else if (bw <= 7800)   ifconf.bandwidth = BW_7K8HZ;
            else if (bw <= 15600)  ifconf.bandwidth = BW_15K6HZ;
            else if (bw <= 31200)  ifconf.bandwidth = BW_31K2HZ;
            else if (bw <= 62500)  ifconf.bandwidth = BW_62K5HZ;
            else if (bw <= 125000) ifconf.bandwidth = BW_125KHZ;
            else if (bw <= 250000) ifconf.bandwidth = BW_250KHZ;
            else if (bw <= 500000) ifconf.bandwidth = BW_500KHZ;
            else ifconf.bandwidth = BW_UNDEFINED;

            MSG("INFO: FSK channel> radio %i, IF %i Hz, %u Hz bw, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
        }
        if (lgw_rxif_setconf(9, ifconf) != LGW_HAL_SUCCESS) {
            MSG("WARNING: invalid configuration for FSK channel\n");
        }
    }
    json_value_free(root_val);
    return 0;
}

static int parse_gateway_configuration(const char * conf_file) {
    const char conf_obj_name[] = "gateway_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
#ifdef ENABLE_CLASS_B
    const char *str; /* pointer to sub-strings in the JSON data */
#endif    /* ENABLE_CLASS_B */
    //unsigned long long ull = 0;

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

    /* get interval (in seconds) for statistics display (optional) */
    val = json_object_get_value(conf_obj, "stat_interval");
    if (val != NULL) {
        stat_interval = (unsigned)json_value_get_number(val);
        MSG("INFO: statistics display interval is configured to %u seconds\n", stat_interval);
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

#ifdef ENABLE_CLASS_B
    /* GPS module TTY path (optional) */
    str = json_object_get_string(conf_obj, "gps_tty_path");
    if (str != NULL) {
        strncpy(gps_tty_path, str, sizeof gps_tty_path);
        MSG("INFO: GPS serial port path is configured to \"%s\"\n", gps_tty_path);
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
#endif    /* ENABLE_CLASS_B */

    /* Beacon signal period (optional) */
    val = json_object_get_value(conf_obj, "beacon_period");
    if (val != NULL) {
        beacon_period = (uint32_t)json_value_get_number(val);
        MSG("INFO: Beaconing period is configured to %u seconds\n", beacon_period);
    }

    /* Beacon TX frequency (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_hz");
    if (val != NULL) {
        beacon_freq_hz = (uint32_t)json_value_get_number(val);
        MSG("INFO: Beaconing signal will be emitted at %u Hz\n", beacon_freq_hz);
    }

    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

static uint16_t BeaconCrc( uint8_t *buffer, uint16_t length )
{
    // The CRC calculation follows CCITT
    const uint16_t polynom = 0x1021;
    // CRC initial value
    uint16_t crc = 0x0000;

    if( buffer == NULL )
    {
        return 0;
    }

    for( uint16_t i = 0; i < length; ++i )
    {
        crc ^= ( uint16_t ) buffer[i] << 8;
        for( uint16_t j = 0; j < 8; ++j )
        {
            crc = ( crc & 0x8000 ) ? ( crc << 1 ) ^ polynom : ( crc << 1 );
        }
    }

    return crc;
}

double difftimespec(struct timespec end, struct timespec beginning) {
    double x;

    x = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    x += (double)(end.tv_sec - beginning.tv_sec);

    return x;
}

#if 0
void thread_test(void)
{
    int ret;
    uint32_t cnt = 0;
    struct timespec beacon_start_ts, rem;
    uint32_t lgw_trigcnt;

    if (clock_gettime (CLOCK_MONOTONIC, &beacon_start_ts) == -1)
        perror ("clock_gettime");

    beacon_start_ts.tv_sec++;
    beacon_start_ts.tv_nsec = 0;

    while (!exit_sig && !quit_sig)
    {
        ret = lgw_get_trigcnt(&lgw_trigcnt);
        if (ret == LGW_HAL_SUCCESS)
            printf("cnt:%u, %u\n", ++cnt, lgw_trigcnt);
        else
            printf("cnt:%u, FAIL\n", ++cnt);

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &beacon_start_ts, &rem);
        beacon_start_ts.tv_sec++;
    }
}
#endif /* #if 0 */

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
    int i; /* loop variable and temporary variable for return value */
    int opt;

    /* configuration file related */
    char *global_cfg_path= "global_conf.json"; /* contain global (typ. network-wide) configuration */
    char *local_cfg_path = "local_conf.json"; /* contain node specific configuration, overwrite global parameters for parameters that are defined in both */
    char *debug_cfg_path = "debug_conf.json"; /* if present, all other configuration files are ignored */
    //char cnt_test = 0;

    /* threads */
    pthread_t thrid_up;
#ifdef ENABLE_CLASS_B
    pthread_t thrid_gps;
#endif    /* ENABLE_CLASS_B */
    //pthread_t thrid_test;

    while ((opt = getopt(argc, argv, "vnt:")) != -1) {
        switch (opt) {
            case 'v':
                verbose = true;
                printf("### verbose mode ###\n");
                break;
            case 'n':
                //cnt_test = 1;
                break;
            case 't':
                //nsecs = atoi(optarg);
                //tfnd = 1;
                break;
            default: /* '?' */
                break;
        }
    }


    /* variables to get local copies of measurements */
    /* display version informations */
    MSG("*** Beacon Packet Forwarder for Lora Gateway ***\nVersion: " VERSION_STRING "\n");
    MSG("*** Lora concentrator HAL library version info ***\n%s\n***\n", lgw_version_info());

    /* display host endianness */
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        MSG("INFO: Little endian host\n");
    #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        MSG("INFO: Big endian host\n");
    #else
        MSG("INFO: Host endianness unknown\n");
    #endif

    /* load configuration files */
    if (access(debug_cfg_path, R_OK) == 0) { /* if there is a debug conf, parse only the debug conf */
        MSG("INFO: found debug configuration file %s, parsing it\n", debug_cfg_path);
        MSG("INFO: other configuration files will be ignored\n");
        parse_SX1301_configuration(debug_cfg_path);
        parse_gateway_configuration(debug_cfg_path);
        parse_lorawan_configuration(debug_cfg_path);
    } else if (access(global_cfg_path, R_OK) == 0) { /* if there is a global conf, parse it and then try to parse local conf  */
        MSG("INFO: found global configuration file %s, parsing it\n", global_cfg_path);
        parse_SX1301_configuration(global_cfg_path);
        parse_gateway_configuration(global_cfg_path);
        parse_lorawan_configuration(global_cfg_path);
        if (access(local_cfg_path, R_OK) == 0) {
            MSG("INFO: found local configuration file %s, parsing it\n", local_cfg_path);
            MSG("INFO: redefined parameters will overwrite global parameters\n");
            parse_SX1301_configuration(local_cfg_path);
            parse_gateway_configuration(local_cfg_path);
            parse_lorawan_configuration(local_cfg_path);
        }
    } else if (access(local_cfg_path, R_OK) == 0) { /* if there is only a local conf, parse it and that's all */
        MSG("INFO: found local configuration file %s, parsing it\n", local_cfg_path);
        parse_SX1301_configuration(local_cfg_path);
        parse_gateway_configuration(local_cfg_path);
        parse_lorawan_configuration(local_cfg_path);
    } else {
        MSG("ERROR: [main] failed to find any configuration file named %s, %s OR %s\n", global_cfg_path, local_cfg_path, debug_cfg_path);
        exit(EXIT_FAILURE);
    }

#ifdef ENABLE_CLASS_B
    /* Start GPS a.s.a.p., to allow it to lock */
    if (gps_tty_path[0] != '\0') { /* do not try to open GPS device if no path set */
        i = lgw_gps_enable(gps_tty_path, "ubx7", 0, &gps_tty_fd); /* HAL only supports u-blox 7 for now */
        if (i != LGW_GPS_SUCCESS) {
            printf("WARNING: [main] impossible to open %s for GPS sync (check permissions)\n", gps_tty_path);
            gps_enabled = false;
            gps_ref_valid = false;
#ifdef ENABLE_CLASS_B
            return -1;  // impossible to run class-B without GPS
#endif
        } else {
            printf("INFO: [main] TTY port %s open for GPS synchronization\n", gps_tty_path);
            gps_enabled = true;
            gps_ref_valid = false;
        }
    }
#endif    /* ENABLE_CLASS_B */

    /* get timezone info */
    tzset();

    /* sanity check on configuration variables */
    // TODO

    /* starting the concentrator */
    i = lgw_start();
    if (i == LGW_HAL_SUCCESS) {
        MSG("INFO: [main] concentrator started, packet can now be received\n");
    } else {
        MSG("ERROR: [main] failed to start the concentrator\n");
        exit(EXIT_FAILURE);
    }

    /* spawn threads to manage upstream and downstream */
    i = pthread_create( &thrid_up, NULL, (void * (*)(void *))thread_up, NULL);
    if (i != 0) {
        MSG("ERROR: [main] impossible to create upstream thread\n");
        exit(EXIT_FAILURE);
    }
    /*if (cnt_test) {
        i = pthread_create( &thrid_test, NULL, (void * (*)(void *))thread_test, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create downstream thread\n");
            exit(EXIT_FAILURE);
        }
    } else*/
#ifdef ENABLE_CLASS_B
    {
        i = pthread_create( &thrid_gps, NULL, (void * (*)(void *))thread_gps, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create downstream thread\n");
            exit(EXIT_FAILURE);
        }
    }
#endif    /* ENABLE_CLASS_B */

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL); /* Ctrl-\ */
    sigaction(SIGINT, &sigact, NULL); /* Ctrl-C */
    sigaction(SIGTERM, &sigact, NULL); /* default "kill" command */

    /* main loop task : statistics collection */
    while (!exit_sig && !quit_sig) {
        /* wait for next reporting interval */
        wait_ms(1000 * stat_interval);
    }

    /* wait for upstream thread to finish (1 fetch cycle max) */
    pthread_join(thrid_up, NULL);
#ifdef ENABLE_CLASS_B
    pthread_cancel(thrid_gps); /* don't wait for downstream thread */
#endif    /* ENABLE_CLASS_B */

    /* if an exit signal was received, try to quit properly */
    if (exit_sig) {
        /* stop the hardware */
        i = lgw_stop();
        if (i == LGW_HAL_SUCCESS) {
            MSG("INFO: concentrator stopped successfully\n");
        } else {
            MSG("WARNING: failed to stop concentrator successfully\n");
        }
    }

    MSG("INFO: Exiting packet forwarder program\n");
    exit(EXIT_SUCCESS);
}

uint32_t trigcnt_pingslot_zero;

void
load_beacon(uint32_t seconds)
{
    uint16_t crc0, crc1;
    struct lgw_pkt_tx_s tx_pkt;
    uint8_t rfuOffset1 = 0;
    uint8_t rfuOffset2 = 0;

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    rfuOffset1 = 1;
    rfuOffset2 = 1;
#endif

    tx_pkt.size = BEACON_SIZE;
    
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
    tx_pkt.bandwidth = BEACON_BW;
    tx_pkt.datarate = BEACON_SF;
    tx_pkt.size = BEACON_SIZE;
    if (skip_beacon_cnt > 0) {
        /* cause beacon to fail by sending on wrong frequency */
        printf("skip_beacon_cnt:%d\n", skip_beacon_cnt);
        skip_beacon_cnt--;
        tx_pkt.freq_hz = BEACON_CHANNEL_FREQ() + 500000;
    } else {
        tx_pkt.freq_hz = BEACON_CHANNEL_FREQ();
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
}

#ifdef ENABLE_CLASS_B
struct timespec g_utc_time; /* UTC time associated with PPS pulse */
struct timespec g_gps_time; /* UBX time associated with PPS pulse */

#ifdef ENABLE_HAL_UBX
struct timespec* time_ptr = &g_gps_time;
#else
struct timespec* time_ptr = &g_utc_time;
#endif

uint32_t prev_tstamp_at_beacon_tx;
bool send_saved_ping = false;
bool first_diff = true;
uint32_t prev_trig_tstamp;
struct timespec prev_utc_sec;
uint8_t pps_cnt;

void pps_init(uint32_t trig_tstamp)
{
    prev_trig_tstamp = trig_tstamp;
    first_diff = true;
    prev_utc_sec.tv_sec = time_ptr->tv_sec;
    pps_cnt = 0;
}

void pps(uint32_t trig_tstamp)
{
    static bool save_next_tstamp = false;
    uint8_t beacon_period_mask = beacon_period - 1;
    static uint32_t tstamp_at_beacon_tx;
    int million = trig_tstamp - prev_trig_tstamp;
    int dev = abs(million - 1000000);

    if (prev_utc_sec.tv_sec+1 != time_ptr->tv_sec) {
        printf("[31mmissing second %lu -> %lu[0m\n", prev_utc_sec.tv_sec, time_ptr->tv_sec);
        //exit(EXIT_FAILURE);
        prev_utc_sec.tv_sec = time_ptr->tv_sec;
        return;
    }
    prev_utc_sec.tv_sec = time_ptr->tv_sec;

    if (dev > 100) {
        printf("dev > 100: %d = %d - %d\n", million, trig_tstamp, prev_trig_tstamp);
        exit(EXIT_FAILURE);
    } 

    prev_trig_tstamp = trig_tstamp;

    if ((time_ptr->tv_sec & beacon_period_mask) == beacon_period_mask) {
        printf("pps utc:%lx   tstamp:%u, pps_cnt:%u\n", time_ptr->tv_sec, trig_tstamp, pps_cnt);
        pps_cnt = 0;
        load_beacon(time_ptr->tv_sec+1);
        save_next_tstamp = true;
    } else if ((time_ptr->tv_sec & beacon_period_mask) == (beacon_period_mask-2)) {
        beacon_guard = true;
    } else if (save_next_tstamp) {
        save_next_tstamp = false;
        prev_tstamp_at_beacon_tx = tstamp_at_beacon_tx;
        tstamp_at_beacon_tx = trig_tstamp;
        printf("tstamp_at_beacon_tx:%u, predicted:%u\n",  tstamp_at_beacon_tx, lgw_trigcnt_at_next_beacon);
        if (!first_diff) {
            uint32_t measured_us = tstamp_at_beacon_tx - prev_tstamp_at_beacon_tx;
            int trigcnt_error_over_beacon_period = (beacon_period * 1000000) - measured_us;
            /* ppm_error:
                    positive = sx1301 is slow
                    negative = sx1301 is fast
            */
            g_sx1301_ppm_err = trigcnt_error_over_beacon_period / (float)beacon_period;
            printf("measured_us:%u  trigcnt_error:%d, g_sx1301_ppm_err:%f\n", measured_us, trigcnt_error_over_beacon_period, g_sx1301_ppm_err);
        } else {
            first_diff = false;
        }

        uint32_t reserved_trigcnt_offset = 2120000 + (g_sx1301_ppm_err * 2.12);
        trigcnt_pingslot_zero = trig_tstamp + reserved_trigcnt_offset;

        lorawan_update_ping_offsets(time_ptr->tv_sec);

        int error_us = g_sx1301_ppm_err * beacon_period;
        uint32_t beacon_period_us = beacon_period * 1000000;
        lgw_trigcnt_at_next_beacon = trig_tstamp + (beacon_period_us - error_us);
        beacon_valid = true;
        printf("error_us:%d, beacon_period_us:%u, lgw_trigcnt_at_next_beacon:%u\n", error_us, beacon_period_us, lgw_trigcnt_at_next_beacon);

        BeaconCtx.BeaconTime = time_ptr->tv_sec - beacon_period;
        send_saved_ping = true;
    } else if (send_saved_ping) {
        uint8_t status;
        beacon_guard = false;
        lgw_status(TX_STATUS, &status);
        if (status == TX_FREE) {
            lorawan_service_ping();
            send_saved_ping = false;
        }
    }

}
#endif    /* ENABLE_CLASS_B */

struct timespec uart_host_time, pps_host_time;

/* -------------------------------------------------------------------------- */
/* --- THREAD 1: RECEIVING PACKETS AND FORWARDING THEM ---------------------- */

void thread_up(void) {
    int i;

    /* allocate memory for packet fetching and processing */
    struct lgw_pkt_rx_s rxpkt[NB_PKT_MAX]; /* array containing inbound packets + metadata */
    struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
    int nb_pkt;
#ifdef ENABLE_CLASS_B
    uint32_t first_trig_tstamp = 0; /* concentrator timestamp associated with PPM pulse */
    uint32_t trig_tstamp = 0; /* concentrator timestamp associated with PPM pulse */
#endif    /* ENABLE_CLASS_B */

    /* report management variable */
    bool send_report = false;

#ifdef ENABLE_CLASS_B
    /* block here until first pps */
    pthread_mutex_lock(&mx_concent);
    i = lgw_get_trigcnt(&first_trig_tstamp);
    pthread_mutex_unlock(&mx_concent);
    do {
        wait_ms(FETCH_SLEEP_MS);
        pthread_mutex_lock(&mx_concent);
        i = lgw_get_trigcnt(&trig_tstamp);
        pthread_mutex_unlock(&mx_concent);
    } while (first_trig_tstamp == trig_tstamp);
    pps_init(trig_tstamp);
#endif    /* ENABLE_CLASS_B */

    while (!exit_sig && !quit_sig) {

        /* fetch packets */
        pthread_mutex_lock(&mx_concent);
        nb_pkt = lgw_receive(NB_PKT_MAX, rxpkt);
        pthread_mutex_unlock(&mx_concent);
        if (nb_pkt == LGW_HAL_ERROR) {
            MSG("ERROR: [up] failed packet fetch, exiting\n");
            exit(EXIT_FAILURE);
        }

        lorawan_kbd_input();
        /* check if there are status report to send */
        /* no mutex, we're only reading */

        /* wait a short time if no packets, nor status report */
        if ((nb_pkt == 0) && (send_report == false)) {
#ifdef ENABLE_CLASS_B
            prev_trig_tstamp = trig_tstamp;
            pthread_mutex_lock(&mx_concent);
            i = lgw_get_trigcnt(&trig_tstamp);
            pthread_mutex_unlock(&mx_concent);
            if (prev_trig_tstamp != trig_tstamp) {
                double secs;
                pps_cnt++;
                //struct timespec now;
                if (clock_gettime (CLOCK_MONOTONIC, &pps_host_time) == -1)
                    perror ("clock_gettime");
                //printf("pps @ %lu.%09lu\n", pps_host_time.tv_sec, pps_host_time.tv_nsec);
                secs = difftimespec(pps_host_time, uart_host_time);
                /*if (secs > 0.97 || secs < 0.85)
                    printf("[31m");
                printf("pps: %f since uart, time:%lu.%09lu, %02x[0m\n", secs, time_ptr->tv_sec, time_ptr->tv_nsec, pps_cnt);*/
                pps(trig_tstamp);
            }
#endif    /* ENABLE_CLASS_B */

            wait_ms(FETCH_SLEEP_MS);
            continue;
        }

        /* serialize Lora packets metadata and payload */
        for (i=0; i < nb_pkt; ++i) {
        uint8_t sf;
            unsigned int bw;

            p = &rxpkt[i];

            switch (p->datarate) {
                case DR_LORA_SF7: sf = 7; break;
                case DR_LORA_SF8: sf = 8; break;
                case DR_LORA_SF9: sf = 9; break;
                case DR_LORA_SF10: sf = 10; break;
                case DR_LORA_SF11: sf = 11; break;
                case DR_LORA_SF12: sf = 12; break;
                default: sf = 0; break;
            }

            switch (p->bandwidth) {
                case BW_500KHZ: bw = 500; break;
                case BW_250KHZ: bw = 250; break;
                case BW_125KHZ: bw = 125; break;
                case BW_62K5HZ: bw = 62; break;
                case BW_31K2HZ: bw = 31; break;
                case BW_15K6HZ: bw = 16; break;
                case BW_7K8HZ: bw = 8; break;
                default: bw = 0; break;
            }

            printf("RX %.2fMHz %+.0fdBm sf%ubw%u ", (float)p->freq_hz / 1000000.0, p->rssi, sf, bw);

            lorawan_parse_uplink(p);
        } // ...for (i=0; i < nb_pkt; ++i)

    }
    MSG("\nINFO: End of upstream thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 2: POLLING SERVER AND ENQUEUING PACKETS IN JIT QUEUE ---------- */


#ifdef ENABLE_CLASS_B
float g_sx1301_ppm_err = 0; // sx1301 timestamp error in a single second

#ifdef ENABLE_HAL_UBX
void thread_gps(void)
{
    /* serial variables */
    char serial_buff[128]; /* buffer to receive GPS data */
    size_t wr_idx = 0;     /* pointer to end of chars in buffer */

    /* variables for PPM pulse GPS synchronization */
    enum gps_msg latest_msg; /* keep track of latest NMEA message parsed */

    /* initialize some variables before loop */
    memset(serial_buff, 0, sizeof serial_buff);

    while (!exit_sig && !quit_sig) {
        size_t rd_idx = 0;
        size_t frame_end_idx = 0;

        /* blocking non-canonical read on serial port */
        ssize_t nb_char = read(gps_tty_fd, serial_buff + wr_idx, LGW_GPS_MIN_MSG_SIZE);
        if (nb_char <= 0) {
            MSG("WARNING: [gps] read() returned value %d\n", nb_char);
            continue;
        }
        wr_idx += (size_t)nb_char;

        /*******************************************
         * Scan buffer for UBX/NMEA sync chars and *
         * attempt to decode frame if one is found *
         *******************************************/
        while(rd_idx < wr_idx) {
            size_t frame_size = 0;

            /* Scan buffer for UBX sync char */
            if(serial_buff[rd_idx] == (char)LGW_GPS_UBX_SYNC_CHAR) {

                /***********************
                 * Found UBX sync char *
                 ***********************/
                latest_msg = lgw_parse_ubx(&serial_buff[rd_idx], (wr_idx - rd_idx), &frame_size);

                if (frame_size > 0) {
                    if (latest_msg == INCOMPLETE) {
                        /* UBX header found but frame appears to be missing bytes */
                        frame_size = 0;
                    } else if (latest_msg == INVALID) {
                        /* message header received but message appears to be corrupted */
                        MSG("WARNING: [gps] could not get a valid message from GPS (no time)\n");
                        frame_size = 0;
                    } else if (latest_msg == UBX_NAV_TIMEGPS) {
                        double secs;
                        if (clock_gettime (CLOCK_MONOTONIC, &uart_host_time) == -1)
                            perror ("clock_gettime");
                        //printf("ubx @ %lu.%09lu\n", uart_host_time.tv_sec, uart_host_time.tv_nsec);

                        int i = lgw_gps_get(&g_utc_time, &g_gps_time, NULL, NULL);
                        if (i != LGW_GPS_SUCCESS) {
                            MSG("WARNING: [gps] could not get GPS time from GPS\n");
                        }
                        secs = difftimespec(uart_host_time, pps_host_time);
                        if (secs < 0.02 || secs > 0.17)
                            printf("[31m");
                        printf("uart: %f since pps, ubx:%lu.%09lu[0m\n", secs, g_gps_time.tv_sec, g_gps_time.tv_nsec);
                    }
                }
            } else if(serial_buff[rd_idx] == LGW_GPS_NMEA_SYNC_CHAR) {
                /************************
                 * Found NMEA sync char *
                 ************************/
                /* scan for NMEA end marker (LF = 0x0a) */
                char* nmea_end_ptr = memchr(&serial_buff[rd_idx],(int)0x0a, (wr_idx - rd_idx));

                if(nmea_end_ptr) {
                    /* found end marker */
                    frame_size = nmea_end_ptr - &serial_buff[rd_idx] + 1;
                    latest_msg = lgw_parse_nmea(&serial_buff[rd_idx], frame_size);

                    if(latest_msg == INVALID || latest_msg == UNKNOWN) {
                        /* checksum failed */
                        frame_size = 0;
                    } else if (latest_msg == NMEA_RMC) { /* Get location from RMC frames */
                        //gps_process_coords();
                    }
                }
            }

            if(frame_size > 0) {
                /* At this point message is a checksum verified frame
                   we're processed or ignored. Remove frame from buffer */
                rd_idx += frame_size;
                frame_end_idx = rd_idx;
            } else {
                rd_idx++;
            }
        } /* ...for(rd_idx = 0... */

        if(frame_end_idx) {
          /* Frames have been processed. Remove bytes to end of last processed frame */
          memcpy(serial_buff,&serial_buff[frame_end_idx],wr_idx - frame_end_idx);
          wr_idx -= frame_end_idx;
        } /* ...for(rd_idx = 0... */

        /* Prevent buffer overflow */
        if((sizeof(serial_buff) - wr_idx) < LGW_GPS_MIN_MSG_SIZE) {
            memcpy(serial_buff,&serial_buff[LGW_GPS_MIN_MSG_SIZE],wr_idx - LGW_GPS_MIN_MSG_SIZE);
            wr_idx -= LGW_GPS_MIN_MSG_SIZE;
        }
    }
    MSG("\nINFO: End of GPS thread\n");
}
#else
void thread_gps(void)
{
    while (!exit_sig && !quit_sig)
    {
        int i;
        enum gps_msg latest_msg; /* keep track of latest NMEA message parsed */
        char serial_buff[128]; /* buffer to receive GPS data */
        ssize_t nb_char;

        /* blocking canonical read on serial port */
        nb_char = read(gps_tty_fd, serial_buff, sizeof(serial_buff)-1);
        if (nb_char < 0) {
            MSG("WARNING: [gps] read() returned value < 0\n");
            continue;
        } else if (nb_char <= 0) {
            MSG("WARNING: [gps] read() returned value == 0\n");
            continue;
        } else {
            serial_buff[nb_char] = 0; /* add null terminator, just to be sure */
        }

        /* parse the received NMEA */
        latest_msg = lgw_parse_nmea(serial_buff, sizeof(serial_buff));

        if (latest_msg == NMEA_RMC || latest_msg == NMEA_GGA ) { 
            double secs;
            if (clock_gettime (CLOCK_MONOTONIC, &uart_host_time) == -1)
                perror ("clock_gettime");
            /* get UTC time for synchronization */
            i = lgw_gps_get(&g_utc_time, NULL, NULL);
            if (i != LGW_GPS_SUCCESS) {
                MSG("WARNING: [gps] could not get UTC time from GPS\n");
                continue;
            }
            secs = difftimespec(uart_host_time, pps_host_time);
            //printf("uart: %f since pps, ubx:%lu.%09lu[0m\n", secs, g_utc_time.tv_sec, g_utc_time.tv_nsec);
        } 

    } // ...while (!exit_sig && !quit_sig)
}
#endif /* !ENABLE_HAL_UBX */

#endif    /* ENABLE_CLASS_B */

void print_tx_status(uint8_t tx_status) {
    switch (tx_status) {
        case TX_OFF:
            MSG("INFO: [jit] lgw_status returned TX_OFF\n");
            break;
        case TX_FREE:
            MSG("INFO: [jit] lgw_status returned TX_FREE\n");
            break;
        case TX_EMITTING:
            MSG("INFO: [jit] lgw_status returned TX_EMITTING\n");
            break;
        case TX_SCHEDULED:
            MSG("INFO: [jit] lgw_status returned TX_SCHEDULED\n");
            break;
        default:
            MSG("INFO: [jit] lgw_status returned UNKNOWN (%d)\n", tx_status);
            break;
    }
}

/* --- EOF ------------------------------------------------------------------ */
