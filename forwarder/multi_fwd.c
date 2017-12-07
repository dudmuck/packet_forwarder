#include <errno.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <net/if.h>
#include <signal.h>
#include <string.h>
#include <sys/param.h>
#include <json-c/json.h>
#include <netdb.h>
#include "loragw_hal.h"
#include "loragw_gps.h"
#include "gw_server.h"
#include "trace.h"

typedef struct {
    char hostname[64];
    uint16_t port;
    bool connected_to_server;
    int sock;
    bool receive_config;
    int server_retry_cnt_down;
    float rssiOffset;
    float snrOffset;
} srv_t;

struct _srv_list {
    srv_t srv;    /**< this */
    struct _srv_list * next;  /**< next */
};

struct _srv_list* srv_list;

/* packets filtering configuration variables */
static bool fwd_valid_pkt = true; /* packets with PAYLOAD CRC OK are forwarded */
static bool fwd_nocrc_pkt = false; /* packets with NO PAYLOAD CRC are NOT forwarded */
static bool fwd_error_pkt = false; /* packets with PAYLOAD CRC ERROR are NOT forwarded */

static int8_t antenna_gain = 0;
unsigned server_retry_wait_us;  /* how long to wait before connect retry */

/* sx1301 configuration */
struct lgw_conf_board_s _boardconf;
struct lgw_conf_lbt_s _lbtconf;
struct lgw_conf_rxrf_s _rfconfs[LGW_RF_CHAIN_NB];
struct lgw_conf_rxif_s _ifconfs[LGW_MULTI_NB];
struct lgw_conf_rxif_s _ifconf_std;
struct lgw_conf_rxif_s _ifconf_fsk;
bool tx_rf_chain;   // written by sx1301 config
bool lgw_started = false;
bool tx_invert;

/* TX capabilities */
static struct lgw_tx_gain_lut_s txlut; /* TX gain table */
static uint32_t tx_freq_min[LGW_RF_CHAIN_NB]; /* lowest frequency supported by TX chain */
static uint32_t tx_freq_max[LGW_RF_CHAIN_NB]; /* highest frequency supported by TX chain */

/* Gateway specificities */
unsigned char mac_address[6];   /* unique identifier */
unsigned long run_rate_usec;

uint32_t beacon_period = 128;   // TODO: put into beacon_info
struct {
    uint32_t hz;
    uint8_t size;
    uint8_t sf;
    uint8_t bw;
    uint8_t rfuOffset1;
    uint8_t rfuOffset2;
    bool us915;
    int8_t dbm;
} beacon_info;
unsigned int skip_beacon_cnt = 0;
unsigned int skip_downlink_cnt = 0;
bool beacon_guard = false;

/* GPS configuration and synchronization */
bool gps_pps_valid = false;
bool gps_time_valid = false;
uint32_t prev_trig_tstamp;
struct timespec g_gps_time; /* UBX time associated with PPS pulse */
struct timespec* time_ptr_ = &g_gps_time;    /* ? using UTC or UBX ? */
struct timespec host_time_at_pps;
uint8_t pps_cnt;
struct coord_s coord;   /* physical location */
char gps_tty_path[96] = "\0";

#define BILLION     1000000000
double difftimespec(struct timespec end, struct timespec beginning) {
    double x;

    x = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    x += (double)(end.tv_sec - beginning.tv_sec);

    return x;
}

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
        printf("ch%u ", chan_ret);
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
    tx_pkt.freq_hz = beacon_info.hz;
    if (skip_beacon_cnt > 0) {
        /* cause beacon to be not received */
        printf("skip_beacon_cnt:%d\n", skip_beacon_cnt);
        skip_beacon_cnt--;
        tx_pkt.invert_pol = true;  // wrong transmit
    } else {
        tx_pkt.invert_pol = false;  // correct transmit
    }

    if (tx_invert)
        tx_pkt.invert_pol = !tx_pkt.invert_pol;

    print_hal_sf(tx_pkt.datarate);
    print_hal_bw(tx_pkt.bandwidth);
    printf(" %.1fMHz\n", tx_pkt.freq_hz / 1e6);

    tx_pkt.tx_mode = ON_GPS;
    tx_pkt.rf_chain = tx_rf_chain;
    tx_pkt.rf_power = beacon_info.dbm;
    tx_pkt.preamble = 10;
    tx_pkt.no_crc = true;
    tx_pkt.no_header = true;   // beacon is fixed length

    printf("BEACON: ");
    if (lgw_send(tx_pkt) == LGW_HAL_ERROR) {
        printf("lgw_send() failed\n");
    }

    return chan_ret;
}

int
write_to_server(srv_t* srv, uint8_t cmd, uint8_t* user_buf, uint16_t user_buf_length)
{
    uint8_t msg[512];
    uint16_t* msg_len = (uint16_t*)&msg[1];
    int nbytes;

    if (!srv->connected_to_server) {
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

    nbytes = write (srv->sock, msg, *msg_len);
    if (nbytes < 0) {
        if (errno == EPIPE) {
            printf("got EPIPE\n");
            srv->connected_to_server = false;
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

    struct _srv_list *sl;
    for (sl = srv_list; sl != NULL; sl = sl->next) {
        if (write_to_server(&sl->srv, BEACON_INDICATION, user_buf, buf_idx) < 0) {
            printf("notify server write failed\n");
        }
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
        //exit(EXIT_FAILURE);
        return -1;
    }

    if (!gps_time_valid)
        printf("\e[31mgps sec:%lu\e[0m\n", time_ptr_->tv_sec);

    if ((time_ptr_->tv_sec & beacon_period_mask) == beacon_period_mask) {
        printf("load_beacon pps sec:%lx   tstamp:%u, ", time_ptr_->tv_sec, trig_tstamp);
        if (gps_time_valid)
            printf(" pps_cnt:%u\n", pps_cnt);
        else
            printf(" \e[31mpps_cnt:%u\e[0m\n", pps_cnt);

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

    } else
        beacon_guard = false;


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

/* return true if done getting first pps */
bool
first_pps(uint32_t trig_tstamp)
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
    printf("seconds_to_beacon:%u pps_cnt:%d, %lu\n", seconds_to_beacon, pps_cnt, time_ptr_->tv_sec);
    if (pps_cnt >= beacon_period) {
        printf("%u = (%u - %u) + 1\n", pps_cnt, beacon_period, seconds_to_beacon);
        return -1;
    }
    lgw_trigcnt_at_next_beacon = trig_tstamp + (seconds_to_beacon * 1000000);
    printf(" lgw_trigcnt_at_next_beacon:%u\n", lgw_trigcnt_at_next_beacon);
    gps_pps_valid = true;

    {   /* send initial beacon info to server */
        uint32_t buf_idx = 0;
        uint32_t lgw_trigcnt_at_next_beacon;
        uint32_t* u32_ptr;
        double* dptr;
        short* sptr;
        uint8_t buf[sizeof(uint32_t)+sizeof(uint32_t)+sizeof(struct coord_s)];
        uint32_t seconds_to_beacon = ((time_ptr_->tv_sec | beacon_period_mask) + 1) - time_ptr_->tv_sec;
        lgw_trigcnt_at_next_beacon = trig_tstamp + (seconds_to_beacon * 1000000);
        u32_ptr = (uint32_t*)buf;
        *u32_ptr = lgw_trigcnt_at_next_beacon;
        buf_idx += sizeof(uint32_t);

        u32_ptr = (uint32_t*)&buf[buf_idx];
        // when beacon would have been sent if gateway was already running
        *u32_ptr = time_ptr_->tv_sec + (seconds_to_beacon - beacon_period);
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

        buf[buf_idx++] = 0; /* which modem is sending beacons */

        struct _srv_list *sl;
        for (sl = srv_list; sl != NULL; sl = sl->next) {
            write_to_server(&sl->srv, BEACON_INIT, buf, buf_idx);
        }
    }

    return true;
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

void
stdin_read()
{
    int i;
    char line[64];
    int stdin_len = read(STDIN_FILENO, line, sizeof(line));

    if (stdin_len == 2) {   // single char and newline
        switch (line[0]) {
            case '.':
                printf("eui:");
                for (i = 0; i<6; i++)
                    printf("%02x ", mac_address[i]);
                printf("\npps_cnt:%u\n", pps_cnt);
                printf("gps_pps_valid:%d\n", gps_pps_valid);
                break;
            default:
                printf(".       print status\n");
                printf("sb%%u       skip beacons\n");
                printf("sd%%u       skip downlinks\n");
                break;
        } // ..switch (line[0])
    } else {
        if (line[0] == 's' && line[1] == 'b') {
            sscanf(line+2, "%u", &skip_beacon_cnt);
            printf("skip_beacon_cnt:%u\n", skip_beacon_cnt);
        } else if (line[0] == 's' && line[1] == 'd') {
            sscanf(line+2, "%u", &skip_downlink_cnt);
            printf("skip_downlink_cnt:%u\n", skip_downlink_cnt);
        }
    }
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
                            //printf("gps_time:%lx.%09lx\n", g_gps_time.tv_sec, g_gps_time.tv_nsec);
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
parse_lorawan_conf(json_object* obj)
{
    json_object* ob;

    if (json_object_object_get_ex(obj, "beacon_period", &ob))
        beacon_period = json_object_get_int(ob);

    if (json_object_object_get_ex(obj, "beacon_size", &ob))
        beacon_info.size = json_object_get_int(ob);

    if (json_object_object_get_ex(obj, "us915_beacon", &ob))
        beacon_info.us915 = json_object_get_boolean(ob);

    if (json_object_object_get_ex(obj, "beacon_hz", &ob))
        beacon_info.hz = json_object_get_int(ob);

    if (json_object_object_get_ex(obj, "beacon_sf", &ob)) {
        switch (json_object_get_int(ob)) {
            case 7: beacon_info.sf = DR_LORA_SF7; break;
            case 8: beacon_info.sf = DR_LORA_SF8; break;
            case 9: beacon_info.sf = DR_LORA_SF9; break;
            case 10: beacon_info.sf = DR_LORA_SF10; break;
            case 11: beacon_info.sf = DR_LORA_SF11; break;
            case 12: beacon_info.sf = DR_LORA_SF12; break;
        }
    }

    if (json_object_object_get_ex(obj, "beacon_bw", &ob)) {
        switch (json_object_get_int(ob)) {
            case 7: beacon_info.bw = BW_7K8HZ; break;
            case 16: beacon_info.bw = BW_15K6HZ; break;
            case 31: beacon_info.bw = BW_31K2HZ; break;
            case 62: beacon_info.bw = BW_62K5HZ; break;
            case 125: beacon_info.bw = BW_125KHZ; break;
            case 250: beacon_info.bw = BW_250KHZ; break;
            case 500: beacon_info.bw = BW_500KHZ; break;
            default: beacon_info.bw = BW_UNDEFINED; break;
        }
    }

    if (json_object_object_get_ex(obj, "beacon_rfuOffset1", &ob))
        beacon_info.rfuOffset1 = json_object_get_int(ob);
    if (json_object_object_get_ex(obj, "beacon_rfuOffset2", &ob))
        beacon_info.rfuOffset2 = json_object_get_int(ob);

    if (json_object_object_get_ex(obj, "beacon_dbm", &ob))
        beacon_info.dbm = json_object_get_int(ob);
}

void
json_print_type(json_type t)
{
    switch (t) {
        case json_type_null: printf("null"); break;
        case json_type_boolean: printf("bool"); break;
        case json_type_double: printf("double"); break;
        case json_type_int: printf("int"); break;
        case json_type_object: printf("object"); break;
        case json_type_array: printf("array"); break;
        case json_type_string: printf("string"); break;
    }
}

void
parse_sx1301_conf(json_object* obj)
{
    char param_name[32];
    int i;
    json_object* ob;

    printf("parse_sx1301_conf() ");

    json_object_object_foreach(obj, key0, val0) {
        int type = json_object_get_type(obj);
        printf("key:\"%s\" -> %s ", key0, json_object_to_json_string(val0));
        json_print_type(type);
        printf("\n");
    }

    if (json_object_object_get_ex(obj, "lorawan_public", &ob))
        _boardconf.lorawan_public = json_object_get_boolean(ob);

    if (json_object_object_get_ex(obj, "tx_invert", &ob))
        tx_invert = json_object_get_boolean(ob);

    if (json_object_object_get_ex(obj, "clksrc", &ob))
        _boardconf.clksrc = json_object_get_int(ob);

    _lbtconf.enable = false;
    if (json_object_object_get_ex(obj, "lbt_cfg", &ob)) {
        json_object* o;
        if (json_object_object_get_ex(ob, "enable", &o))
            _lbtconf.enable = json_object_get_boolean(o);
        if (_lbtconf.enable) {
            if (json_object_object_get_ex(ob, "rssi_target", &o))
                _lbtconf.rssi_target = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "sx127x_rssi_offset", &o))
                _lbtconf.rssi_offset = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "chan_cfg", &o)) {
                _lbtconf.nb_channel = json_object_array_length(o);
                for (i = 0; i < _lbtconf.nb_channel; i++) {
                    json_object *ajo = json_object_array_get_idx(o, i);
                    if (json_object_object_get_ex(ajo, "freq_hz", &o))
                        _lbtconf.channels[i].freq_hz = json_object_get_int(o);
                    if (json_object_object_get_ex(ajo, "scan_time_us", &o))
                        _lbtconf.channels[i].scan_time_us = json_object_get_int(o);

                }
            }
        } // ..if lbt enabled
    } // ..lbt_cfg
    else printf("lbt-disabled ");

    if (json_object_object_get_ex(obj, "antenna_gain", &ob))
        antenna_gain = json_object_get_int(ob);

    for (i = 0; i < TX_GAIN_LUT_SIZE_MAX; i++) {
        snprintf(param_name, sizeof param_name, "tx_lut_%i", i);
        if (json_object_object_get_ex(obj, param_name, &ob)) {
            json_object* o;
            if (json_object_object_get_ex(ob, "pa_gain", &o))
                txlut.lut[i].pa_gain = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "mix_gain", &o))
                txlut.lut[i].mix_gain = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "rf_power", &o))
                txlut.lut[i].rf_power = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "dig_gain", &o))
                txlut.lut[i].dig_gain = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "dac_gain", &o))
                txlut.lut[i].dac_gain = json_object_get_int(o);
            else
                txlut.lut[i].dac_gain = 3;
            printf("%s\n", param_name);
        }
    } // ..for (i = 0; i < TX_GAIN_LUT_SIZE_MAX; i++)

    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        snprintf(param_name, sizeof param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
        if (json_object_object_get_ex(obj, param_name, &ob)) {
            json_object* o;
            if (!_rfconfs[i].enable) {
                if (json_object_object_get_ex(ob, "enable", &o))
                    _rfconfs[i].enable = json_object_get_boolean(o);
            }
            if (_rfconfs[i].enable) {
                if (json_object_object_get_ex(ob, "rssi_offset", &o)) {
                    _rfconfs[i].rssi_offset = json_object_get_int(o);
                }
                if (json_object_object_get_ex(ob, "freq", &o)) {
                    _rfconfs[i].freq_hz = json_object_get_int(o);
                    printf("rfchain%u freq_hz:%u\n", i, _rfconfs[i].freq_hz);
                } else
                    printf("rfchain%u no freq_hz\n", i);

                if (json_object_object_get_ex(ob, "type", &o)) {
                    const char* str = json_object_get_string(o);
                    if (strcmp(str, "SX1255") == 0)
                        _rfconfs[i].type = LGW_RADIO_TYPE_SX1255;
                    else if (strcmp(str, "SX1257") == 0)
                        _rfconfs[i].type = LGW_RADIO_TYPE_SX1257;
                }
                if (json_object_object_get_ex(ob, "tx_enable", &o))
                    _rfconfs[i].tx_enable = json_object_get_boolean(o);
                if (_rfconfs[i].tx_enable) {
                    if (json_object_object_get_ex(ob, "tx_freq_min", &o))
                        tx_freq_min[i] = json_object_get_int(o);
                    if (json_object_object_get_ex(ob, "tx_freq_max", &o))
                        tx_freq_max[i] = json_object_get_int(o);
                    if (json_object_object_get_ex(ob, "tx_notch_freq", &o))
                        _rfconfs[i].tx_notch_freq = json_object_get_int(o);
                }
                if (_rfconfs[i].type == LGW_RADIO_TYPE_SX1255)
                    MSG("INFO SX1255: ");
                else if (_rfconfs[i].type == LGW_RADIO_TYPE_SX1257)
                    MSG("INFO SX1257: ");
                MSG("radio %i enabled, center frequency %u, RSSI offset %f, tx enabled %d, tx_notch_freq %u\n", i, _rfconfs[i].freq_hz, _rfconfs[i].rssi_offset, _rfconfs[i].tx_enable, _rfconfs[i].tx_notch_freq);
            } // ..if (_rfconfs[i].enable)
            else
                printf("%s disabled\n", param_name);
        } else
            printf("no %s\n", param_name);
    } // ..for (i = 0; i < LGW_RF_CHAIN_NB; ++i)

    for (i = 0; i < LGW_MULTI_NB; ++i) {
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
        if (json_object_object_get_ex(obj, param_name, &ob)) {
            json_object* o;
            if (!_ifconfs[i].enable) {
                if (json_object_object_get_ex(ob, "enable", &o))
                    _ifconfs[i].enable = json_object_get_boolean(o);
            }
            if (_ifconfs[i].enable) {
                if (json_object_object_get_ex(ob, "radio", &o))
                    _ifconfs[i].rf_chain = json_object_get_int(o);
                if (json_object_object_get_ex(ob, "if", &o))
                    _ifconfs[i].freq_hz = json_object_get_int(o);
            } else
                printf("%s disabled\n", param_name);
        } else
            printf("no %s\n", param_name);
    } // ..for (i = 0; i < LGW_MULTI_NB; ++i)

    if (json_object_object_get_ex(obj, "chan_Lora_std", &ob)) {
        json_object* o;
        if (!_ifconf_std.enable) {
            if (json_object_object_get_ex(ob, "enable", &o))
                _ifconf_std.enable = json_object_get_boolean(o);
        }
        if (_ifconf_std.enable) {
            if (json_object_object_get_ex(ob, "radio", &o))
                _ifconf_std.rf_chain = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "if", &o))
                _ifconf_std.freq_hz = json_object_get_int(o);
            MSG("INFO: Lora std channel> radio %i, IF %i Hz ", _ifconf_std.rf_chain, _ifconf_std.freq_hz);
            if (json_object_object_get_ex(ob, "bandwidth", &o)) {
                MSG("bw:%uHz ", json_object_get_int(o));
                switch (json_object_get_int(o)) {
                    case 500000: _ifconf_std.bandwidth = BW_500KHZ; break;
                    case 250000: _ifconf_std.bandwidth = BW_250KHZ; break;
                    case 125000: _ifconf_std.bandwidth = BW_125KHZ; break;
                    default: _ifconf_std.bandwidth = BW_UNDEFINED;
                }
            }
            if (json_object_object_get_ex(ob, "spread_factor", &o)) {
                MSG("sf%u ", json_object_get_int(o));
                switch (json_object_get_int(o)) {
                    case  7: _ifconf_std.datarate = DR_LORA_SF7;  break;
                    case  8: _ifconf_std.datarate = DR_LORA_SF8;  break;
                    case  9: _ifconf_std.datarate = DR_LORA_SF9;  break;
                    case 10: _ifconf_std.datarate = DR_LORA_SF10; break;
                    case 11: _ifconf_std.datarate = DR_LORA_SF11; break;
                    case 12: _ifconf_std.datarate = DR_LORA_SF12; break;
                    default: _ifconf_std.datarate = DR_UNDEFINED;
                }
            }
            MSG("\n");
        }
    }

    if (json_object_object_get_ex(obj, "chan_FSK", &ob)) {
        json_object* o;
        if (!_ifconf_fsk.enable) {
            if (json_object_object_get_ex(ob, "enable", &o))
                _ifconf_fsk.enable = json_object_get_boolean(o);
        }
        if (_ifconf_fsk.enable) {
            unsigned fdev = 0;
            if (json_object_object_get_ex(ob, "radio", &o))
                _ifconf_fsk.rf_chain = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "if", &o))
                _ifconf_fsk.freq_hz = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "freq_deviation", &o))
                fdev = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "datarate", &o))
                _ifconf_fsk.datarate = json_object_get_int(o);
            if (json_object_object_get_ex(ob, "bandwidth", &o)) {
                unsigned bw = json_object_get_int(o);
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
    }
    printf("\nparse_sx1301_conf() done\n");
}


int
parse_config(const char* conf_file)
{
    int len, ret = -1;
    json_object *obj, *jobj;
    enum json_tokener_error jerr;
    struct json_tokener *tok = json_tokener_new();
    FILE *file = fopen(conf_file, "r");
    if (file == NULL) {
        perror(conf_file);
        return -1;
    }

    do {
        char line[96];
        if (fgets(line, sizeof(line), file) == NULL) {
            fprintf(stderr, "NULL == fgets()\n");
            goto pEnd;
        }
        len = strlen(line);
        jobj = json_tokener_parse_ex(tok, line, len);
        //printf("jobj:%p, len%u\n", jobj, len);
    } while ((jerr = json_tokener_get_error(tok)) == json_tokener_continue);

    if (jerr != json_tokener_success) {
        printf("parse_server_config() json error: %s\n", json_tokener_error_desc(jerr));
        goto pEnd;
    }
    if (tok->char_offset < len) {
        printf("json extra chars\n");
    }

/*
    json_object_object_foreach(jobj, key0, val0) {
        int type = json_object_get_type(jobj);
        printf("key:\"%s\" -> %s ", key0, json_object_to_json_string(val0));
        json_print_type(type);
        printf("\n");
    }
*/

    if (json_object_object_get_ex(jobj, "SX1301_conf", &obj)) {
        parse_sx1301_conf(obj);
    } else {
        printf("\e[31mmissing sx1301_conf\e[0m\n");
        ret = -1;
        goto pEnd;
    }

    if (json_object_object_get_ex(jobj, "gateway_conf", &obj)) {
        json_object *ob;
        if (json_object_object_get_ex(obj, "gps_tty_path", &ob)) {
            const char* str = json_object_get_string(ob);
            strncpy(gps_tty_path, str, sizeof(gps_tty_path));
        }
        if (json_object_object_get_ex(obj, "run_rate_ms", &ob))
            run_rate_usec = json_object_get_int(ob) * 1000;
        else
            run_rate_usec = 50000;  // default run rate

        if (json_object_object_get_ex(obj, "server_retry_wait_ms", &ob))
            server_retry_wait_us = json_object_get_int(ob) * 1000;
        else
            server_retry_wait_us = 4000000; // default retry

        if (json_object_object_get_ex(obj, "servers", &ob)) {
            struct _srv_list* sl;
            int i, alen = json_object_array_length(ob);
            srv_list = calloc(1, sizeof(struct _srv_list));
            sl = srv_list;
            for (i = 0; i < alen; ) {
                json_object* o;
                json_object *ajo = json_object_array_get_idx(ob, i);
                if (json_object_object_get_ex(ajo, "hostname", &o))
                    strncpy(sl->srv.hostname, json_object_get_string(o), sizeof(sl->srv.hostname)); 
                if (json_object_object_get_ex(ajo, "port", &o))
                    sl->srv.port = json_object_get_int(o);
                if (json_object_object_get_ex(ajo, "rssiOffset", &o))
                    sl->srv.rssiOffset = json_object_get_double(o);
                if (json_object_object_get_ex(ajo, "snrOffset", &o))
                    sl->srv.snrOffset = json_object_get_double(o);

                if (++i < alen) {
                    sl->next = calloc(1, sizeof(struct _srv_list));
                    sl = sl->next;
                }
            } // ..for (i = 0; i < alen; )
        }

        if (json_object_object_get_ex(obj, "forward_crc_valid", &ob)) 
            fwd_valid_pkt = json_object_get_boolean(ob);
        if (json_object_object_get_ex(obj, "forward_crc_error", &ob)) 
            fwd_error_pkt = json_object_get_boolean(ob);
        if (json_object_object_get_ex(obj, "forward_crc_disabled", &ob)) 
            fwd_nocrc_pkt = json_object_get_boolean(ob);
    } else {
        printf("\e[31mmissing gateway_conf\e[0m\n");
        ret = -1;
        goto pEnd;
    }

    ret = 0;

pEnd:
    if (tok)
        json_tokener_free(tok);
    fclose(file);
    return ret;
} // ..parse_config()


int
get_server_config(int sock)
{
    char file_buf[4096];
    int file_length;
    int nbytes;
    int n_read;

    /* configuration available */
    /* first 4 bytes is length */
    nbytes = read(sock, &file_length, sizeof(int));
    if (nbytes < 0) {
        perror("get_server_config() read()");
        return -1;
    }
    if (nbytes == 0) {
        fprintf(stderr, "get_server_config() read file_length 0\n");
        return -1;
    }
    printf("conf file_length got nbytes %d: %d\n", nbytes, file_length);

    for (n_read = 0; n_read < file_length; ) {
        nbytes = read(sock, file_buf+n_read, sizeof(file_buf)-n_read);
        if (nbytes < 0) {
            perror("read");
            return -1;
        }
        if (nbytes == 0) {
            printf("nbytes==0\n");
            return -1;
        }
        n_read += nbytes;
        printf("n_read:%d\n", n_read);
    }

    printf("have configuration\n");

    struct json_tokener *tok = json_tokener_new();
    json_object *jo = json_tokener_parse_ex(tok, file_buf, n_read);
    enum json_tokener_error jerr = json_tokener_get_error(tok);
    if (jerr == json_tokener_success) {
        json_object* obj;
        if (tok->char_offset < n_read)
            printf("\e[31mjson extra chars\e[0m\n");
        if (json_object_object_get_ex(jo, "SX1301_conf", &obj)) {
            parse_sx1301_conf(obj);
        } 
        if (json_object_object_get_ex(jo, "lorawan", &obj)) {
            parse_lorawan_conf(obj);
        } 
    } else
        printf("\e[31mjerr %s\e[0m\n", json_tokener_error_desc(jerr));

    json_object_put(jo);
    json_tokener_free(tok);

#if 0
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
#endif /* if 0 */

    return 0;
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

#define TX_PRELOAD_US           200000
#define NUM_TX_PKTS     4
struct lgw_pkt_tx_s tx_pkts[NUM_TX_PKTS];

/* return true for gps fix re-aquired */
bool gw_tx_service(bool im)
{
    double seconds_since_pps;
    struct timespec now;
    uint32_t count_us_now;
    int i;
    bool ret = false;

    if (clock_gettime (CLOCK_MONOTONIC, &now) == -1)
        perror ("clock_gettime");

    /* get time difference between last host_time_at_pps and now */
    seconds_since_pps = difftimespec(now, host_time_at_pps);
    //printf("seconds_since_pps:%f\n", seconds_since_pps);
    if (gps_pps_valid) {
        if (seconds_since_pps > 3.0) {
            /* send invalid GPS time value to server indicating loss of GPS fix */
            coord.lat = 0;
            coord.lon = 0;
            coord.alt = 0;
            notify_server(0, 0xffffffff, 0);
            printf("GPS PPS loss\n");
            gps_pps_valid = false;
        }
    } else if (seconds_since_pps < 1.0) {
        printf("GPS restored\n");
        gps_pps_valid = true;
        ret = true;
    }

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
        return ret;
    }

    //printf("min_us_to_tx_start:%d\n", min_us_to_tx_start);
    if (min_us_to_tx_start < TX_PRELOAD_US) {
        if (beacon_guard) {
            printf("\e[31mskipping tx due to beacon_guard\e[0m\n");
        } else {
            if (min_us_to_tx_start < 10000)
                printf("\e[31m");
            if (im)
                printf("immediatly ");

            if (skip_downlink_cnt > 0) {
                skip_downlink_cnt--;
                /* drop this downlink */
                tx_pkts[ftbs_i].invert_pol = !tx_pkts[ftbs_i].invert_pol;
                printf("\e[33m");
            }

            printf("sending tx_pkts[%d] at %u (%dus before tx)\e[0m\n", ftbs_i, count_us_now, min_us_to_tx_start);
            i = lgw_send(tx_pkts[ftbs_i]);
            tx_pkts[ftbs_i].freq_hz = 0;    // mark as sent
            if (i == LGW_HAL_ERROR) {
                printf("lgw_send() failed\n");
            }
        }
    }

    return ret;
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

    if (tx_invert)
        tx_pkt.invert_pol = !tx_pkt.invert_pol;

    memcpy(&tx_pkt.payload, user_buf+idx, tx_pkt.size);
    idx += tx_pkt.size;

    uint8_t tx_modem_idx = user_buf[idx++];
    if (tx_modem_idx != 0) {
        /* only one modem exists */
        fprintf(stderr, "FAIL modem_idx:%u ", tx_modem_idx);
    }

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
        printf("\e[31m%ddBm not found\e[0m ", tx_pkt.rf_power);
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
downlink_service(const srv_t* srv)
{
    uint8_t user_buf[512];
    int nbytes;//, bytes_available = 0;
    unsigned int len;

    nbytes = read(srv->sock, user_buf, sizeof(user_buf));
    if (nbytes < 0) {
        perror("sock-read");
        return -1;
    }
    if (nbytes == 0) {
        /* likely server disconnected */
        printf("\e[33mserver read nbytes == 0\e[0m\n");
        return -1;
    }
    //printf("server_downlink_service() read nbytes:%d\n", nbytes);
    printf("from %s:%u ", srv->hostname, srv->port);

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
        case SKIP_BEACON_COUNT:
            {
                unsigned int idx = 3;   // skip past cmd and length
                unsigned int* ui_ptr = (unsigned int*)&user_buf[idx];
                skip_beacon_cnt = *ui_ptr;
                printf("skip_beacon_cnt:%u\n", skip_beacon_cnt);
                idx += sizeof(unsigned int);
            }
            break;
        default:
            printf("unhandled server cmd:%d, nbytes:%d\n", user_buf[0], nbytes);
            break;
    }

    return 0;
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

int connect_to_server(const char* serv_addr, uint16_t serv_port)
{
    extern void init_sockaddr (struct sockaddr_in *name, const char *hostname, uint16_t port);
    struct sockaddr_in servername;
    int sock;
    char str[128];

    /* Create the socket. */
    sock = socket (PF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        perror ("socket (client)");
        return -1;
    }

    /* Connect to the server. */
    printf("attempting %s:%u\n", serv_addr, serv_port);
    init_sockaddr (&servername, serv_addr, serv_port);
    if (0 > connect (sock, (struct sockaddr *) &servername, sizeof (servername)))
    {
        sprintf(str, "connect %s:%u ", serv_addr, serv_port);
        perror(str);
        close(sock);
        return -1;
    }

    return sock;
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

    for (i = 0; i < nb_pkt; ++i) {
        struct _srv_list *sl;
        struct lgw_pkt_rx_s *p = &rxpkt[i];
        for (sl = srv_list; sl != NULL; sl = sl->next) {
            uint16_t* u16_ptr;
            uint32_t* u32_ptr;
            unsigned int buf_idx = 0;
            uint8_t user_buf[sizeof(struct lgw_pkt_rx_s)];
            float rssi = p->rssi + sl->srv.rssiOffset;
            float snr = p->snr + sl->srv.snrOffset;

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

            /* rssi and snr both float */
            memcpy(&user_buf[buf_idx], &rssi, sizeof(p->rssi));
            buf_idx += sizeof(p->rssi);

            memcpy(&user_buf[buf_idx], &snr, sizeof(p->snr));
            buf_idx += sizeof(p->snr);
            printf("(%u snr:%.1f rssi:%.0f)  ", sl->srv.port, snr, rssi);

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

            user_buf[buf_idx++] = 0;    /* which modem receiving (always 0) */

            write_to_server(&sl->srv, UPLINK, user_buf, buf_idx);
        }
    } // ..for (i = 0; i < nb_pkt; ++i)

    return 0;
}

int
main (int argc, char **argv)
{
    uint32_t first_trig_tstamp;
    bool get_first_pps = false;
    int maxfd, i, gps_tty_fd = -1;
    fd_set active_fd_set;

    /* clear gateway configuration */
    memset(&_boardconf, 0, sizeof _boardconf); /* initialize configuration structure */
    memset(&_lbtconf, 0, sizeof _lbtconf); /* initialize configuration structure */
    memset(&_rfconfs, 0, sizeof _rfconfs); /* initialize configuration structure */
    memset(&_ifconfs, 0, sizeof _ifconfs); /* initialize configuration structure */
    memset(&_ifconf_std, 0, sizeof _ifconf_std); /* initialize configuration structure */
    memset(&_ifconf_fsk, 0, sizeof _ifconf_fsk); /* initialize configuration structure */

    if (parse_config("../multi_conf.json") < 0) {
        printf("\e[31mparse_config() failed\e[0m\n");
        return -1;
    }
/*
    while ((opt = getopt(argc, argv, "")) != -1) {
        switch (opt) {
        }
    }
*/
    if (get_host_unique_id() < 0) {
        return -1;
    }
    /* get broken pipe errno instead of SIGPIPE */
    signal(SIGPIPE, SIG_IGN);


    FD_ZERO(&active_fd_set);
    FD_SET(STDIN_FILENO, &active_fd_set);

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
            printf("gps_tty_fd:%d, maxfd:%d\n", gps_tty_fd, maxfd);
        }
    } else {
        maxfd = STDIN_FILENO + 1;
        printf("WARNING: [main] GPS not opened\n");
        return -1;  /* this is class-B forwarder */
    }

    for (;;) {
        struct _srv_list *sl;
        struct timeval timeout;
        int retval;

        FD_SET(STDIN_FILENO, &active_fd_set);
        FD_SET(gps_tty_fd, &active_fd_set);
        for (sl = srv_list; sl != NULL; sl = sl->next) {
            if (sl->srv.connected_to_server)
                FD_SET(sl->srv.sock, &active_fd_set);
        }

        timeout.tv_sec = 0;
        timeout.tv_usec = run_rate_usec;
        retval = select(maxfd, &active_fd_set, NULL, NULL, &timeout);
        if (retval < 0) { 
            perror("select");
            return -1;
        } else if (FD_ISSET(STDIN_FILENO, &active_fd_set)) {
            stdin_read();
        } else if (FD_ISSET(gps_tty_fd, &active_fd_set)) {
            gps_service(gps_tty_fd);
        } else /*if (connected_to_server)*/ {
            for (sl = srv_list; sl != NULL; sl = sl->next) {
                srv_t* srv;
                if (!sl->srv.connected_to_server)
                    continue;
                srv = &sl->srv;
                if (FD_ISSET(srv->sock, &active_fd_set)) {
                    if (srv->receive_config) {
                        if (get_server_config(srv->sock) == 0) {
                            printf("get_server_config() OK\n");
                            srv->receive_config = false;

                            if (&sl->srv == &srv_list->srv) {  // if primary server
                                if (check_sx1301_config() < 0) {
                                    /* check failed */
                                    printf("config check failed\n");
                                    FD_CLR(srv->sock, &active_fd_set);
                                    close (srv->sock);
                                    srv->connected_to_server = false;
                                    srv->server_retry_cnt_down = server_retry_wait_us;
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
                                        FD_CLR(srv->sock, &active_fd_set);
                                        close (srv->sock);
                                        srv->connected_to_server = false;
                                        srv->server_retry_cnt_down = server_retry_wait_us;
                                    }
                                }
                            } 
                        } else {
                            printf("get_server_config() failed\n");
                            FD_CLR(srv->sock, &active_fd_set);
                            close (srv->sock);
                            srv->connected_to_server = false;
                            sleep(4);
                        }
                    } else {
                        if (downlink_service(srv) < 0) {
                            if (&sl->srv == &srv_list->srv) {  // if primary server
                                lgw_stop();
                                lgw_started = false;
                            }
                            FD_CLR(srv->sock, &active_fd_set);
                            close (srv->sock);
                            srv->connected_to_server = false;
                            srv->server_retry_cnt_down = server_retry_wait_us;
                        }
                    }
                }
            } // ..for (sl = srv_list; sl != NULL; sl = sl->next)
        } /*else if (retval != 0) {
            printf("TODO other fd ");
            if (FD_ISSET(_sock, &active_fd_set)) {
                printf("sock");
            }
            printf("\n");
            sleep(1);
        }*/

        for (sl = srv_list; sl != NULL; sl = sl->next) {
            srv_t* srv;
            if (sl->srv.connected_to_server)
                continue;
            srv = &sl->srv;

            if (srv->server_retry_cnt_down > 0) {
                srv->server_retry_cnt_down -= run_rate_usec;
                if (srv->server_retry_cnt_down > 0)
                    continue;
            }
            srv->sock = connect_to_server(srv->hostname, srv->port);
            if (srv->sock >= 0) {
                uint8_t buf[8];
                uint32_t* u32_ptr = (uint32_t*)buf;
                *u32_ptr = PROT_VER;
                buf[4] = 1; /* num_modems: one sx1301 */
                printf("server-connected\n");
                srv->connected_to_server = true;

                /* request config from server */
                write_to_server(srv, REQ_CONF, buf, sizeof(uint32_t)+1);

                srv->receive_config = true;
                FD_SET(srv->sock, &active_fd_set);
                maxfd = MAX(gps_tty_fd, srv->sock) + 1;
            } else {
                srv->server_retry_cnt_down = server_retry_wait_us;
            }
        } // ..for (sl = srv_list; sl != NULL; sl = sl->next)

        if (srv_list->srv.connected_to_server) {     /* GPS on primary server only */
            if (get_first_pps) {
                uint32_t trig_tstamp;
                lgw_get_trigcnt(&trig_tstamp);
                if (trig_tstamp != first_trig_tstamp) {
                    if (first_pps(trig_tstamp)) {
                        get_first_pps = false; // done getting first pps
                        printf("got first pps\n");
                    }
                }
            } else if (lgw_started) {
                /* connected to server and concentrator running */
                service_pps();

                uplink_service();
                if (gw_tx_service(false))
                    get_first_pps = true;   // lost GPS restored

            }
        }
    } // ..for (;;)

    return 0;
}
