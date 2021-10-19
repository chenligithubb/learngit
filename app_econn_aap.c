#define ECONN_AAP 1

#pragma GCC diagnostic ignored "-Wundef"

#if ECONN == ECONN_AAP
#include "types.h"
#include "stdio.h"
#include "string.h"
#include "os_timer.h"
#include "os_mem.h"
#include "app_btn.h"
#include "app_evt.h"
#include "app_l2cap.h"
#include "app_bt.h"
#include "app_wws.h"
#include "app_bat.h"
#include "app_inear.h"
#include "app_adv.h"
#include "app_audio.h"
#include "app_charger.h"
#include "app_sdp.h"
#include "app_econn.h"
#include "app_main.h"
#include "usr_cfg.h"
#include "inear_sensor.h"
#include "app_doap.h"

/**
 * please add aap_enc.s to source list before enable AAP_ENC_ENABLED
 */
#define AAP_ADV_ENC_ENABLED

#define AAP_DEBUG

#define AAP_MSG_ID_ENABLE_ADV   0
#define AAP_MSG_ID_STOP_ADV     1
#define AAP_MSG_ID_TIP_RESPONE  2
#define AAP_MSG_ID_GATT_CONNECT 3

#define AAP_DISABLE_ADV_AFTER_BOOT_MS 1500
#define AAP_CONNECTED_ADV_DURATION_MS 60000
#define AAP_TIP_PLAY_DURATION_MS      3000

#define MODULE_ID_I_1   0x2002
#define MODULE_ID_I_2   0x200F
#define MODULE_ID_I_PRO 0x200E

#define MODULE_ID         MODULE_ID_I_PRO
#define FEATURE_MODULE_ID MODULE_ID_I_PRO

#define ARRAY_LEN(x) sizeof(x) / sizeof(x[0])

#define BT_UNPACK_LE_2_BYTE(dst, src)                 \
    *((uint16_t *)(dst)) = *((src) + 1);              \
    *((uint16_t *)(dst)) = *((uint16_t *)(dst)) << 8; \
    *((uint16_t *)(dst)) |= *((src) + 0);
#define BT_UNPACK_LE_4_BYTE(dst, src)                   \
    *((uint32_t *)(dst)) = *((src) + 3);                \
    *((uint32_t *)(dst)) = (*((uint32_t *)(dst))) << 8; \
    *((uint32_t *)(dst)) |= *((src) + 2);               \
    *((uint32_t *)(dst)) = (*((uint32_t *)(dst))) << 8; \
    *((uint32_t *)(dst)) |= *((src) + 1);               \
    *((uint32_t *)(dst)) = (*((uint32_t *)(dst))) << 8; \
    *((uint32_t *)(dst)) |= *((src) + 0);

#ifndef AAP_DEBUG
#undef DBGLOG_ECONN_DBG
static inline int no_printf(const char *fmt, ...)
{
    return 0;
}   //suppress unused variable warning
#define DBGLOG_ECONN_DBG(...) no_printf(__VA_ARGS__)
#endif

#define AAP_PSM 0x1001

typedef struct {
    uint8_t value : 7;
    uint8_t charging : 1;
} __attribute__((packed)) adv_battery_t;

//00 00 04 00 01 00 02 00 7F 5A 04 01 00 00 00 00
static const uint8_t APP_CONNECT_REQUEST[] = {0x00, 0x00, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t APP_CONNECT_RESPONSE[] = {0x01, 0x00, 0x04, 0x00, 0x00, 0x00,
                                               0x01, 0x00, 0x02, 0x00, 0x07, 0x00,
                                               0xe1, 0x84, 0x07, 0x00, 0x3d, 0x86};
//{0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t AAP_DISCONNECT_REQUEST[] = {0x02, 0x00, 0x04, 0x00, 0x00, 0x00};
static const uint8_t AAP_DISCONNECT_RESPONSE[] = {0x03, 0x00, 0x04, 0x00, 0x00, 0x00};
static const uint8_t AACP_HEADER[] = {0x04, 0x00, 0x04, 0x00};

static bool_t aap_connected = false;
static bool_t in_ear_detection_enabled = true;
static bool_t is_enc_key_sent = false;
static BD_ADDR_T remote_addr = {0};
static uint32_t current_sys_state = 0;
static bool_t is_ag_pairing_adv = false;
static bool_t adv_enabled = false;
static char g_app_serial_l[] = "GX1D66X6JQH4";
static char g_app_serial_r[] = "GX7D5R5HJQH3";

/*this flag makesure inear not trigger streaming start as it was paused by AG side*/
static bool_t pause_streaming_by_outear = false;
static void aap_start_adv(void);
static void app_handle_inear_action_for_android(bool_t in_ear);
static uint8_t get_left_battery(void)
{
    if (app_wws_is_left()) {
        return app_bat_get_level();
    } else {
        return app_wws_peer_get_battery_level() == 0 ? app_bat_get_level()
                                                     : app_wws_peer_get_battery_level();
    }
}

static uint8_t get_left_charging(void)
{
    if (app_wws_is_left()) {
        return app_charger_is_charging() ? 1 : 0;
    } else {
        if (app_wws_peer_get_battery_level() == 0) {
            return app_charger_is_charging() ? 1 : 0;
        } else {
            return app_wws_peer_is_charging() ? 1 : 0;
        }
    }
}

static uint8_t get_left_in_ear(void)
{
    if (app_wws_is_left()) {
        return app_inear_get();
    } else {
        return app_wws_peer_is_inear();
    }
}

static uint8_t get_right_battery(void)
{
    if (app_wws_is_right()) {
        return app_bat_get_level();
    } else {
        return app_wws_peer_get_battery_level() == 0 ? app_bat_get_level()
                                                     : app_wws_peer_get_battery_level();
    }
}

static uint8_t get_right_charging(void)
{
    if (app_wws_is_right()) {
        return app_charger_is_charging() ? 1 : 0;
    } else {
        if (app_wws_peer_get_battery_level() == 0) {
            return app_charger_is_charging() ? 1 : 0;
        } else {
            return app_wws_peer_is_charging() ? 1 : 0;
        }
    }
}

static uint8_t get_right_in_ear(void)
{
    if (app_wws_is_right()) {
        return app_inear_get();
    } else {
        return app_wws_peer_is_inear();
    }
}

static uint8_t get_box_battery(void)
{
    return app_charger_get_box_battery();
}

static uint8_t get_box_charging(void)
{
    if (app_charger_is_box_charging()) {
        return 1;
    } else {
        return 0;
    }
}

static void aap_adv_pairing(void)
{
    static uint8_t adv_paring[] = {
        0x14, 0xFF, 0x4C, 0x00, 0x07, 0x0F,
        0x00,                                 // status 0:paring 1:paired ...
        0x02, 0x20,                           //module id 0x2002=I1,1 0x200F=I1,3/I2 0x200E=I Pro
        0x20, 0x19, 0x06, 0x03, 0x29, 0x56,   //bt addr
        0x35,                                 //hsStatus 13
        0x64,                                 //right batt_charge 14
        0xE4,                                 //left batt_charge 15
        0x40,                                 //box batt_charge 16
        0x01,                                 //ST=UCCLsBo?1s;  &0x08:0=bo=Box opened; &0x07=1:1s 17
        0x00                                  // CC=0 18
    };

    adv_paring[7] = MODULE_ID & 0xFF;
    adv_paring[8] = MODULE_ID >> 8;

    const BD_ADDR_T *local_addr = app_bt_get_local_address();
    adv_paring[9] = local_addr->addr[5];
    adv_paring[10] = local_addr->addr[4];
    adv_paring[11] = local_addr->addr[3];
    adv_paring[12] = local_addr->addr[2];
    adv_paring[13] = local_addr->addr[1];
    adv_paring[14] = local_addr->addr[0];

    adv_battery_t left = {get_left_battery(), get_left_charging()};
    adv_battery_t right = {get_right_battery(), get_right_charging()};
    adv_battery_t box = {get_left_charging() || get_right_charging() ? get_box_battery() : 0xFF,
                         get_left_charging() || get_right_charging() ? get_box_charging() : 1};

    adv_paring[17] = *((uint8_t *)&right);
    adv_paring[16] = *((uint8_t *)&left);
    adv_paring[18] = *((uint8_t *)&box);

    /**
     * index differ with paired
     * 0-1 used for paired
     * 5-7 used for paring
     * same index probably not trigger popup on phone
     */
    static uint8_t index = 5;

    index += 1;
    if (index > 7) {
        index = 5;
    }

    if (app_charger_is_box_open()) {
        adv_paring[19] = index;
    } else {
        adv_paring[19] = 0x08 + index;
    }

    if ((MODULE_ID == MODULE_ID_I_2) || (MODULE_ID == MODULE_ID_I_1)) {
        adv_paring[19] |= 0x30;   //box type: button center + led
    }

    app_adv_set_adv_data(adv_paring, sizeof(adv_paring));
    app_adv_set_enabled(true);

    DBGLOG_ECONN_DBG("aap_adv_pairing left:%d.%d right:%d.%d\n", left.value, left.charging,
                     right.value, right.charging);
}

//I2 <4c000719010f2075aab2010000c637eca98a2890db4c3d1373d38006be>,
//ST Status, Nm '?', Md 'I1,3', Paired no, Cnx no, ST=UCCLsbo?1s, CC=0, Batt C -20%; L +100%; R +100%
//4c 00 07 19 01 0f 20 75 aa b2 01 00 00 c6 37 ec a9 8a 28 90 db 4c 3d 13 73 d3 80 06 be

static unsigned int aap_status_to_headset_status(uint8_t status)
{
    unsigned int result = status & 1;   //1:U 0:u
    unsigned char b12 = (status >> 1) & 3;
    unsigned char b34 = (status >> 3) & 3;
    if (b12 == 1) {    //left
        result |= 4;   //E:in ear
    } else if (b12 == 2) {
        result |= 2;   //C:in case
    } else if (b12 == 3) {
        result |= 8;   //A
    }

    if (b34 == 1) {       // right
        result |= 0x20;   //E
    } else if (b34 == 2) {
        result |= 0x10;   //C
    } else if (b34 == 3) {
        result |= 0x40;   //A
    }
    //bit5: 1:L 0:r
    //bit6 not: 1:P 0:s
    //bit7: 1:B 0:b
    result |= ((status << 2) & 0x380) ^ 0x100;

    return result;
}

//<4c000719010f2075aab201000414e4e417b8de5a99fa5f000088c123b5>,
//ST Status, Nm 'BX\M-g\M^Z\M^DI', Md 'I1,3', Paired yes, Cnx yes, ST=UCCLsbo11s, CC=0, Batt C -23%; L +100%; R +100%
//4c 00 07 19 01 0f 20 75 aa b2 01 00 04 14 e4 e4 17 b8 de 5a 99 fa 5f 00 00 88 c1 23 b5
static void aap_adv_paired(void)
{
    //4c00071901022055aa3a00000095e4e42932e7507d0d3a1046fd980b4d
    static uint8_t adv_paired[] = {
        0x1E, 0xFF, 0x4C, 0x00,   //0
        0x07, 0x19,               //2
        0x01,                     //status  0:paring 1:paired ...   4
        0x02, 0x20,               //module id  0x2002=I1,1    5
        0x55,                     //HsStatus 7
        0xaa,                     //8  right/left battLevel*10, 0x0F:none
        0x3a,   //9  0x10:right_charging 0x20:left_charging 0x40:case_charging 0x0F:caseBattLevel*10 0x0F=outOfBox
        0x00,                    //10  0x07:locS 0x08:lc,case_closed 0x10:csVS 0x60:csLC 0x80:csLS
        0x00,                    //cc 11
        0x00,                    //12
        0x95,                    //13  0x03:aState 0x0C:asCount
        0xe4,                    //right batt_charge 14
        0xe4,                    //left batt_charge 15
        0x29,                    //box batt_charge 16
        0x32, 0xe7, 0x50,        // lch  17
        0x7d, 0x0d, 0x3a,        // lbic 20
        0x10, 0x46,              // matID big endian  23
        0xfd, 0x98, 0x0b, 0x4d   //25
    };

    //lc: 1=C:close 0=o:open
    //na: NeedsAWDL
    //nk: NeedsKeyboard
    //ns: NeedsSetup
    //pf: Problems flags
    //CnSv: Cnx

    uint8_t hs_status = 0x01;

    if (get_left_charging()) {
        hs_status |= (0x02 << 1);
    } else if (get_left_in_ear()) {
        hs_status |= (0x01 << 1);
    }

    if (get_right_charging()) {
        hs_status |= (0x02 << 3);
    } else if (get_right_in_ear()) {
        hs_status |= (0x01 << 3);
    }

    if (app_wws_is_left()) {
        hs_status |= 1 << 5;
    }

    uint8_t data8 = (((get_left_battery() / 10) & 0x0F) << 4) | ((get_right_battery() / 10) & 0x0F);
    uint8_t data9 = 0x80;
    uint8_t data10 = 0x00;
    if (get_right_charging()) {
        data9 |= 0x10;
    }
    if (get_left_charging()) {
        data9 |= 0x20;
    }

    if (get_left_charging() || get_right_charging()) {
        if (get_box_charging()) {
            data9 |= 0x40;
        }
        data9 |= get_box_battery() / 10;
    } else {
        data9 |= 0x0F;
    }

    if (!app_charger_is_box_open()) {
        data10 |= 0x08;
    } else if (get_left_charging() && get_right_charging()) {
        data10 |= 0x01;   //locP
    }

    if ((MODULE_ID == MODULE_ID_I_2) || (MODULE_ID == MODULE_ID_I_1)) {
        data10 |= 0x30;   //box type: button center + led
    }

    adv_battery_t left = {get_left_battery(), get_left_charging()};
    adv_battery_t right = {get_right_battery(), get_right_charging()};
    adv_battery_t box = {get_left_charging() || get_right_charging() ? get_box_battery() : 0xFF,
                         get_left_charging() || get_right_charging() ? get_box_charging() : 1};

    adv_paired[7] = MODULE_ID & 0xFF;
    adv_paired[8] = MODULE_ID >> 8;
    adv_paired[9] = hs_status;
    adv_paired[10] = data8;
    adv_paired[11] = data9;
    adv_paired[12] = data10;

    //matID
    adv_paired[25] = 0x00;
    adv_paired[26] = 0x00;

    if (app_wws_is_left()) {
        adv_paired[17] = *((uint8_t *)&right);
        adv_paired[16] = *((uint8_t *)&left);
    } else {
        adv_paired[16] = *((uint8_t *)&right);
        adv_paired[17] = *((uint8_t *)&left);
    }

    adv_paired[18] = *((uint8_t *)&box);

#ifdef AAP_ADV_ENC_ENABLED
    extern void aap_enc(uint8_t * data);
    aap_enc(adv_paired);
#endif

    app_adv_set_adv_data(adv_paired, sizeof(adv_paired));
    app_adv_set_enabled(true);

    DBGLOG_ECONN_DBG("aap_adv_paired left:%d.%d right:%d.%d\n", left.value, left.charging,
                     right.value, right.charging);
}

static void send_aacp_response(uint16_t cmd, const uint8_t *param, uint16_t param_len)
{
    uint8_t buffer[128];

    if (!aap_connected) {
        DBGLOG_ECONN_DBG("send_aacp_response aap no connected\n");
        return;
    }

    if (app_wws_is_slave()) {
        DBGLOG_ECONN_DBG("send_aacp_response by slave, ignored\n");
        return;
    }

    if (param_len > 100) {
        DBGLOG_ECONN_DBG("send_aacp_response param_len:%d too long\n", param_len);
        return;
    }
    memcpy_s(buffer, sizeof(buffer), AACP_HEADER, sizeof(AACP_HEADER));
    memcpy_s(buffer + sizeof(AACP_HEADER),
             sizeof(AACP_HEADER) >= sizeof(buffer) ? 0 : sizeof(buffer) - sizeof(AACP_HEADER), &cmd,
             sizeof(uint16_t));
    if (param && param_len) {
        memcpy_s(buffer + sizeof(AACP_HEADER) + sizeof(uint16_t),
                 sizeof(AACP_HEADER) + sizeof(uint16_t) >= sizeof(buffer)
                     ? 0
                     : sizeof(buffer) - sizeof(AACP_HEADER) - sizeof(uint16_t),
                 param, param_len);
    }

    app_l2cap_send_data(&remote_addr, AAP_PSM, buffer,
                        sizeof(AACP_HEADER) + sizeof(uint16_t) + param_len);
    DBGLOG_ECONN_DBG("aacp send response cmd:%02X param_len:%d\n", cmd, param_len);
}

/*
AACP_MSG_CAPABILITIES_RESP  04 00 04 00 02 00 03 01 0e 02 02 04 88 06 00 01
04 00 aap header
04 00 aacp header
02 00 AACP_MSG_CAPABILITIES_RESP
03 01 0e 02 02 04 88 06 00 01 param
*/
static void aap_send_capabilities(void)
{
    static const uint8_t capabilities[] = {0x03, 0x01, 0x0e, 0x02, 0x02,
                                           0x04, 0x88, 0x06, 0x00, 0x01};
    static const uint8_t capabilities_pro[] = {0x06, 0x01, 0x0e, 0x02, 0x02, 0x04, 0x01, 0x00,
                                               0x00, 0x00, 0x10, 0x01, 0x40, 0x01, 0x80, 0x01};
    if (MODULE_ID == MODULE_ID_I_PRO) {
        send_aacp_response(0x0002, capabilities_pro, sizeof(capabilities_pro));
    } else {
        send_aacp_response(0x0002, capabilities, sizeof(capabilities));
    }
}

#if 0
/**
 * @brief: marketing version to display version
 * @param in: Marketing Version
 * @param out: display version
 * @param out_len: max length of display version
 */
static void marketing_2_display(const char *in, char *out, int out_len)
{
    uint32_t num;

    num = atoi(in);
    snprintf(out, out_len, "%d%c%d", (num >> 20) & 0xF, ((num >> 16) & 0xF) + 'A',
             (uint16_t)num >> 4);
}

/**
 * @brief: display version to marketing version
 * @param in: display version
 * @param out: Marketing Version
 * @param out_len: max length of marketing version
 */
static void display_2_marketing(const char *in, char *out, int out_len)
{
    uint32_t num1, num2, num3, total;

    num1 = in[0] - '0';
    num2 = in[1] - 'A';
    num3 = atoi(in + 2);

    total = (num1 << 20) + (num2 << 16) + (num3 << 4);
    snprintf(out, out_len, "%d", total);
}
#endif

/*
 AccessoryVersionInfo.txt
"Name" I
"Model Identifier" A2031
"Manufacturer" XXX Inc.
"Serial Number (Right Bud)" GKXZKS6ELX2Y
"Firmware Version (Active)" 620002032000000.287
"Firmware Version (Pending)" 20.620002032000000.287
"Hardware Version" 1.0.0
"EA Protocol Name" com.xxx.accessory.updater.app.61
"Serial Number (Left Bud)" 0
"Serial Number (Case)" 0
"Marketing Version" 2102976
*/
static void aap_send_version_info(void)
{
    uint8_t buffer[256];
    char *ptr;

    memset(buffer, 0, sizeof(buffer));
    buffer[0] = 0x04;   //header
    buffer[1] = 0x00;
    buffer[2] = 0x04;
    buffer[3] = 0x00;
    buffer[4] = 0x1d;   //cmd
    buffer[5] = 0x00;
    buffer[6] = 0x01;    //ver
    buffer[7] = 0x00;    //param_len_low
    buffer[8] = 0x00;    //param_len_high
    buffer[9] = 0x04;    //session_index_low
    buffer[10] = 0x00;   //session_index_high

    ptr = (char *)buffer + 11;

    const char *dev_name;
    int name_len;
    dev_name = app_bt_get_local_name();
    name_len = strlen(dev_name);

    if (name_len > 32) {
        memcpy_s(ptr,
                 (uint8_t *)ptr >= buffer + sizeof(buffer)
                     ? 0
                     : buffer + sizeof(buffer) - (uint8_t *)ptr,
                 dev_name, 32);
        ptr += 64 + 1;
    } else {
        memcpy_s(ptr,
                 (uint8_t *)ptr >= buffer + sizeof(buffer)
                     ? 0
                     : buffer + sizeof(buffer) - (uint8_t *)ptr,
                 dev_name, name_len);
        ptr += name_len + 1;
    }

    if (MODULE_ID == MODULE_ID_I_PRO) {
        strcpy(ptr, "A2084");
    } else {
        strcpy(ptr, "A2031");
    }
    ptr += strlen(ptr) + 1;

    if (FEATURE_MODULE_ID == MODULE_ID_I_1) {
        strcpy(ptr, "Company Inc.");
    } else {
        strcpy(ptr, "Apple Inc.");
    }
    ptr += strlen(ptr) + 1;

    const uint8_t *addr = app_bt_get_local_address()->addr;
    snprintf(ptr,
             (uint8_t *)ptr >= buffer + sizeof(buffer) ? 0
                                                       : buffer + sizeof(buffer) - (uint8_t *)ptr,
             "%02X%02X%02X%02X%02X%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    ptr += strlen(ptr) + 1;

    strcpy(ptr, "620002032000000.287");
    ptr += strlen(ptr) + 1;

    strcpy(ptr, "20.620002032000000.287");
    ptr += strlen(ptr) + 1;

    strcpy(ptr, "1.0.0");
    ptr += strlen(ptr) + 1;

    strcpy(ptr, "com.company.accessory.updater.app.61");
    ptr += strlen(ptr) + 1;

    strcpy(ptr, g_app_serial_l);
    ptr += strlen(ptr) + 1;

    strcpy(ptr, g_app_serial_r);
    ptr += strlen(ptr) + 1;

    if (MODULE_ID == MODULE_ID_I_PRO) {
        // strcpy(ptr, "3150256");   //3A283
        strcpy(ptr, "3419888");   //3E751

    } else {
        strcpy(ptr, "2102976");
    }
    ptr += strlen(ptr) + 1;

    int param_len = (uint8_t *)ptr - buffer - 9;
    if (param_len >= 200) {
        DBGLOG_ECONN_DBG("aap_send_version_info data too long:%d\n", param_len);
        DBGLOG_ECONN_DBG("----buffer overflow------\n");
    }
    buffer[7] = param_len;
    app_l2cap_send_data(&remote_addr, AAP_PSM, buffer, param_len + 9);

    DBGLOG_ECONN_DBG("aap_send_version_info %d done\n", (uint32_t)ptr - (uint32_t)buffer);
}

#if 0
static const char *const IN_EAR_STR[] = {
    "in ear",  "out of ear", "in case", "unknown", "unknown", "unknown", "unknown",
    "unknown", "unknown",    "unknown", "On Ear",  "Off Ear", "On Neck",
};
#endif

typedef enum {
    IN_EAR = 0,
    OUT_OF_EAR,
    IN_CASE,
    UNKNOWN_IN_EAR_STATUS3,
    UNKNOWN_INEAR_STATUS4,
    UNKNOWN_INEAR_STATUS5,
    UNKNOWN_INEAR_STATUS6,
    UNKNOWN_INEAR_STATUS7,
    UNKNOWN_INEAR_STATUS8,
    UNKNOWN_INEAR_STATUS9,
    ON_EAR,
    OFF_EAR,
    ON_NECK,
} in_ear_status_t;

/*
InEarStatus 04 00 04 00 06 00 02 02
04 00 aap header
04 00 aacp header
06 00 InEarStatus
02 primary  0=in ear,1=out of ear,2=in case,10=on ear,11=off ear,12=on neck
02 secondary
 */
static void aap_send_in_ear_status(in_ear_status_t primary, in_ear_status_t secondary)
{
    uint8_t inEarStatusParam[] = {(uint8_t)primary, (uint8_t)secondary};
    DBGLOG_ECONN_DBG("aacp send in ear status: primary=%d secondary=%d\n", primary, secondary);
    send_aacp_response(0x0006, inEarStatusParam, sizeof(inEarStatusParam));
}

static void aap_send_current_in_ear_status(void)
{
    in_ear_status_t primary;
    in_ear_status_t secondary;

    if (app_charger_is_charging()) {
        primary = IN_CASE;
    } else if (app_inear_get()) {
        primary = IN_EAR;
    } else {
        primary = OUT_OF_EAR;
    }

    if (app_wws_peer_is_charging()) {
        secondary = IN_CASE;
    } else if (app_wws_peer_is_inear()) {
        secondary = IN_EAR;
    } else {
        secondary = OUT_OF_EAR;
    }

    aap_send_in_ear_status(primary, secondary);
}

static void aap_send_box_version(void)
{
    static uint8_t param[] = {0x01, 0x4c, 0x00, 0x00, 0x06, 0x07, 0x00, 0x01,
                              0x00, 0x00, 0x05, 0x01, 0x01, 0x30, 0x00};
    send_aacp_response(0x0023, param, sizeof(param));
}

typedef enum {
    CHARGE_FULL = 0,
    CHARGING,
    CHARGE_OFF,
} charge_status_t;

/*
BATT_INFO   04 00 04 00 04 00 03 02 01 64 02 01 04 01 64 01 01 08 01 11 02 01
04 00 aap header
04 00 aacp header
04 00 battery info
03 info count
info 1
02 which: 1=Single 2=Right 4=Left 8=Case
01 unknown
64 level:100
02 charge status: 0=full 1=charging 2=no charge
01 unknown

info 2
04 01 64 01 01 Left,100,charging
info 3
08 01 11 02 01 Case,17,no charge
 */
static void aap_send_battery_info(charge_status_t left_charging, uint8_t left_batt,
                                  charge_status_t right_charging, uint8_t right_batt,
                                  charge_status_t case_charging, uint8_t case_batt)
{
    if (left_charging > CHARGE_OFF) {
        DBGLOG_ECONN_DBG("aap_send_battery_info left_charging:%d error\n", left_charging);
        return;
    }
    if (right_charging > CHARGE_OFF) {
        DBGLOG_ECONN_DBG("aap_send_battery_info right_charging:%d error\n", right_charging);
        return;
    }
    if (case_charging > CHARGE_OFF) {
        DBGLOG_ECONN_DBG("aap_send_battery_info case_charging:%d error\n", case_charging);
        return;
    }
    uint8_t batteryParam[] = {0x03, 0x02, 0x01, right_batt, (uint8_t)right_charging,
                              0x01, 0x04, 0x01, left_batt,  (uint8_t)left_charging,
                              0x01, 0x08, 0x01, case_batt,  (uint8_t)case_charging,
                              0x01};
    DBGLOG_ECONN_DBG("aacp send battery info: left=%d,%d right=%d,%d case=%d,%d\n",
                     (int)left_charging, left_batt, (int)right_charging, right_batt,
                     (int)case_charging, case_batt);
    send_aacp_response(0x0004, batteryParam, sizeof(batteryParam));
}

static void aap_send_current_battery_info(void)
{
    aap_send_battery_info(get_left_charging() ? CHARGING : CHARGE_OFF, get_left_battery(),
                          get_right_charging() ? CHARGING : CHARGE_OFF, get_right_battery(),
                          get_box_charging() ? CHARGING : CHARGE_OFF, get_box_battery());
}

/*
roleState 04 00 04 00 08 00 02 00 01 00
04 00 aap header
04 00 aacp header
08 00 roleState
02 primary is,1=left,2=right
00 unknown(magnet connected??)
01 secondary connected,bool
00 quick connect enabled,bool
 */
static void aap_send_role_state(bool_t primary_left, bool_t secondary_connected,
                                bool_t quick_connect_enabled)
{
    //roleState:prim=2right second connected,notQuickDis 04 00 04 00 08 00 02 00 01 00
    uint8_t roleStateParam[] = {
        primary_left ? 0x01 : 0x02,
        0x00,
        secondary_connected ? 0x01 : 0x00,
        quick_connect_enabled ? 0x01 : 0x00,
    };
    DBGLOG_ECONN_DBG(
        "aacp send role state: primary_left:%d secondary_connected:%d quick_connect_enabled:%d\n",
        primary_left, secondary_connected, quick_connect_enabled);
    send_aacp_response(0x0008, roleStateParam, sizeof(roleStateParam));
}

/**
EasyPairResponse 04 00 04 00 0c 00 ee 7d 70 bb 94 10 01 02
04 00 aap header
04 00 aacp header
0c 00 EasyPairResponse
ee 7d 70 bb 94 10 addr
01 pair succeed:bool
02 unknown
 */
static void aap_send_eary_pair_response(uint8_t *other_bdaddr, bool_t paired)
{
    uint8_t param[] = {0xee, 0x7d, 0x70, 0xbb, 0x94, 0x10, 0x01, 0x02};
    if (other_bdaddr) {
        memcpy_s(param, sizeof(param), other_bdaddr, 6);
    }
    if (paired) {
        param[6] = 0x01;
    } else {
        param[6] = 0x00;
    }
    send_aacp_response(0x000C, param, sizeof(param));
}

/*
TriangleStatus 04 00 04 00 0e 00 0a 77 4e ca 57 4c 00
04 00 aap header
04 00 aacp header
0e 00 TriangleStatus
0a 77 4e ca 57 4c addr
00 paired:bool
 */
static void aap_send_triangle_status(uint8_t *bdaddr, bool_t paired)
{
    uint8_t param[] = {0x0a, 0x77, 0x4e, 0xca, 0x57, 0x4c, 0x00};
    if (bdaddr) {
        memcpy_s(param, sizeof(param), bdaddr, 6);
    }
    if (paired) {
        param[6] = 0x01;
    } else {
        param[6] = 0x00;
    }
    send_aacp_response(0x000E, param, sizeof(param));
}

#if 0
static const char *const CONTROL_CMD_STR[] = {
    "unknown",                   //0
    "Mic mode",                  //1
    "Scan",                      //2
    "Reset",                     //3
    "basic Double Tap mode",     //4
    "Button Send mode",          //5
    "Ownership state",           //6
    "Tap Interval",              //7
    "Bud Role",                  //8
    "Debug Get Data",            //9
    "In Ear detection",          //0x0A
    "Jitter Buffer",             //0x0B
    "Double Tap mode",           //0x0C
    "Listen mode",               //0x0D
    "unknown",                   //0x0E
    "unknown",                   //0x0F
    "unknown",                   //0x10
    "Switch Control",            //0x11
    "VoiceTrigger state",        //0x12
    "Siri mode",                 //0x13
    "Single Click",              //0x14
    "Double Click",              //0x15
    "Click And Hold",            //0x16
    "Double Click Interval",     //0x17
    "Click And Hold Interval",   //0x18
    "unknown",                   //0x19
    "Listening Mode Configs",    //0x1A
    "One Bud ANC Mode",          //0x1B
};
#endif

enum control_cmd {
    CONTROL_UNKNOWN_0 = 0,             //0
    CONTROL_MIC_MODE,                  //1
    CONTROL_SCAN,                      //2
    CONTROL_RESET,                     //3
    CONTROL_BASIC_DOUBLE_TAP_MODE,     //4
    CONTROL_BUTTON_SEND_MODE,          //5
    CONTROL_OWNERSHIP_STATE,           //6
    CONTROL_TAP_INTERVAL,              //7
    CONTROL_BUD_ROLE,                  //8
    DEBUG_GET_DATA,                    //9
    CONTROL_IN_EAR_DETECTION,          //0x0A
    CONTROL_JITTER_BUFFER,             //0x0B
    CONTROL_DOUBLE_TAP_MODE,           //0x0C
    CONTROL_LISTEN_MODE,               //0x0D
    CONTROL_UNKNOWN_E,                 //0x0E
    CONTROL_UNKNOWN_F,                 //0x0F
    CONTROL_UNKNOWN_10,                //0x10
    CONTROL_SWITCH_CONTROL,            //0x11
    CONTROL_VOICE_TRIGGER_STATE,       //0x12
    CONTROL_SIRI_MODE,                 //0x13
    CONTROL_SINGLE_CLICK,              //0x14
    CONTROL_DOUBLE_CLICK,              //0x15
    CONTROL_CLICK_AND_HOLD,            //0x16
    CONTROL_DOUBLE_CLICK_INTERVAL,     //0x17
    CONTROL_CLICK_AND_HOLD_INTERVAL,   //0x18
    CONTROL_UNKNOWN_19,                //0x19
    CONTRL_LISTENING_MODE_CONFIGS,     //0x1A
    CONTROL_ONE_BUD_ANC_MODE,          //0x1B
};

#if 0
static const char *get_double_tap_mode_str(uint8_t mode)
{

    switch (mode) {
        case 1:
            return "siri";
            break;
        case 2:
            return "media";
            break;
        case 3:
            return "next track";
            break;
        case 4:
            return "prev track";
            break;
        default:
            return "off";
            break;
    }
    return "unknown";
}
#endif

typedef enum {
    DOUBLE_TAP_OFF,
    DOUBLE_TAP_SIRI,
    DOUBLE_TAP_MEDIA,
    DOUBLE_TAP_NEXT_TRACK,
    DOUBLE_TAP_PREV_TRACK,
    DOUBLE_TAP_OFF1
} double_tap_mode_t;

static const char *get_listen_mode_str(uint8_t mode)
{
    switch (mode) {
        case LISTEN_MODE_UNKNOWN:
            return "unknown";
        case LISTEN_MODE_NORMAL:
            return "Normal";
        case LISTEN_MODE_ANC:
            return "ANC";
        case LISTEN_MODE_TRANSPARENCY:
            return "Transparency";
        default:
            return "unknown";
    }
}

typedef enum {
    CLICK_AND_HOLD_UNKNOWN,
    CLICK_AND_HOLD_SIRI,
    CLICK_AND_HOLD_PLAY_PAUSE,
    CLICK_AND_HOLD_NEXT_TRACK,
    CLICK_AND_HOLD_PREV_TRACK,
    CLICK_AND_HOLD_NOISE_MANAGEMENT,
} click_and_hold_mode_t;

static const char *get_click_and_hold_mode_str(uint8_t mode)
{
    switch (mode) {
        case CLICK_AND_HOLD_UNKNOWN:
            return "unknown";
        case CLICK_AND_HOLD_SIRI:
            return "Siri";
        case CLICK_AND_HOLD_PLAY_PAUSE:
            return "Play/Pause";
        case CLICK_AND_HOLD_NEXT_TRACK:
            return "Next Track";
        case CLICK_AND_HOLD_PREV_TRACK:
            return "Prev Track";
        case CLICK_AND_HOLD_NOISE_MANAGEMENT:
            return "Noise Management";
        default:
            return "unknown";
    }
}

#define NOISE_MANAGEMENT_NORMAL       0x01
#define NOISE_MANAGEMENT_ANC          0x02
#define NOISE_MANAGEMENT_TRANSPARENCY 0x04

static const char *get_listen_mode_configs_str(uint8_t config)   //Noise Management
{
    config &= 0x07;
    if (config == 0x01) {
        return "Normal";
    } else if (config == 0x02) {
        return "ANC";
    } else if (config == 0x03) {
        return "Normal/ANC";
    } else if (config == 0x04) {
        return "Transparency";
    } else if (config == 0x05) {
        return "Normal/Transparency";
    } else if (config == 0x06) {
        return "ANC/Transparency";
    } else if (config == 0x07) {
        return "Normal/ANC/Transparency";
    } else {
        return "unknown";
    }
}

//command 0x0009:control command
static void handle_control_command(uint8_t *param, uint16_t param_len)
{
    uint8_t control_command = param[0];
    uint32_t control_value = 0;
    param += 1;
    param_len -= 1;

    if (param_len != 4) {
        DBGLOG_ECONN_DBG("aacp error control command, param_len:%d\n", param_len);
        return;
    }

    BT_UNPACK_LE_4_BYTE(&control_value, param);

    if (control_command > 0x1B) {
        DBGLOG_ECONN_DBG("aacp recevied unknown control command:0x%X\n", control_command);
        return;
    }

    DBGLOG_ECONN_DBG("aacp recevied control command:0x%X value:0x%X\n", control_command,
                     control_value);
    switch (control_command) {
        case CONTROL_UNKNOWN_0:   //0
            break;
        case CONTROL_MIC_MODE:   //1
            DBGLOG_ECONN_DBG("mic mode:");
            if (control_value == 0) {
                DBGLOG_ECONN_DBG("auto\n");
            } else if (control_value == 1) {
                DBGLOG_ECONN_DBG("fixed_right\n");
            } else if (control_value == 2) {
                DBGLOG_ECONN_DBG("fixed_left\n");
            } else {
                DBGLOG_ECONN_DBG("unknown\n");
            }
            break;
        case CONTROL_SCAN:   //2
            break;
        case CONTROL_RESET:   //3
            break;
        case CONTROL_BASIC_DOUBLE_TAP_MODE:   //4
            break;
        case CONTROL_BUTTON_SEND_MODE:   //5
            break;
        case CONTROL_OWNERSHIP_STATE:   //6
            DBGLOG_ECONN_DBG("ownership state:");
            if (control_value) {
                DBGLOG_ECONN_DBG("owned\n");
            } else {
                DBGLOG_ECONN_DBG("not owned\n");
            }
            break;
        case CONTROL_TAP_INTERVAL:   //7
            break;
        case CONTROL_BUD_ROLE:   //8
            break;
        case DEBUG_GET_DATA:   //9
            break;
        case CONTROL_IN_EAR_DETECTION:   //0x0A
            DBGLOG_ECONN_DBG("in ear detection:");
            if (control_value == 2) {
                in_ear_detection_enabled = true;
                app_inear_set_enabled(true);
                DBGLOG_ECONN_DBG("enabled\n");
                aap_send_current_in_ear_status();
            } else if (control_value == 1) {
                in_ear_detection_enabled = false;
                app_inear_set_enabled(false);
                DBGLOG_ECONN_DBG("disabled\n");
            } else {
                in_ear_detection_enabled = false;
                app_inear_set_enabled(false);
                DBGLOG_ECONN_DBG("unknown\n");
            }
            break;
        case CONTROL_JITTER_BUFFER:   //0x0B
            break;
        case CONTROL_DOUBLE_TAP_MODE:   //0x0C
        {
            uint8_t left_mode = (control_value >> 8) & 0xFF;
            uint8_t right_mode = control_value & 0xFF;
            DBGLOG_ECONN_DBG("double tap mode left:%d right:%d\n", left_mode, right_mode);

            key_pressed_type_t type = BTN_TYPE_DOUBLE;
            uint16_t state = STATE_CONNECTED | STATE_A2DP_STREAMING;
            uint16_t left_event_id = 0;
            uint16_t right_event_id = 0;

            switch (left_mode) {
                case DOUBLE_TAP_OFF:
                    left_event_id = 0;
                    break;
                case DOUBLE_TAP_SIRI:
                    left_event_id = EVTUSR_VOICE_RECOGNITION_TOGGLE;
                    break;
                case DOUBLE_TAP_MEDIA:
                    left_event_id = EVTUSR_MUSIC_PLAY_PAUSE;
                    break;
                case DOUBLE_TAP_NEXT_TRACK:
                    left_event_id = EVTUSR_MUSIC_FORWARD;
                    break;
                case DOUBLE_TAP_PREV_TRACK:
                    left_event_id = EVTUSR_MUSIC_BACKWARD;
                    break;
                case DOUBLE_TAP_OFF1:
                    left_event_id = 0;
                    break;
                default:
                    left_event_id = 0;
                    break;
            }

            switch (right_mode) {
                case DOUBLE_TAP_OFF:
                    right_event_id = 0;
                    break;
                case DOUBLE_TAP_SIRI:
                    right_event_id = EVTUSR_VOICE_RECOGNITION_TOGGLE;
                    break;
                case DOUBLE_TAP_MEDIA:
                    right_event_id = EVTUSR_MUSIC_PLAY_PAUSE;
                    break;
                case DOUBLE_TAP_NEXT_TRACK:
                    right_event_id = EVTUSR_MUSIC_FORWARD;
                    break;
                case DOUBLE_TAP_PREV_TRACK:
                    right_event_id = EVTUSR_MUSIC_BACKWARD;
                    break;
                case DOUBLE_TAP_OFF1:
                    right_event_id = 0;
                    break;
                default:
                    right_event_id = 0;
                    break;
            }
            if (app_wws_is_left()) {
                app_btn_cus_key_add(type, state, left_event_id);
                app_wws_send_set_cus_key(type, state, right_event_id);
            } else {
                app_btn_cus_key_add(type, state, right_event_id);
                app_wws_send_set_cus_key(type, state, left_event_id);
            }
            break;
        }
        case CONTROL_LISTEN_MODE:   //0x0D
            DBGLOG_ECONN_DBG("listen mode:%d\n", (uint8_t)control_value);
            app_audio_listen_mode_set(control_value);
            break;
        case CONTROL_UNKNOWN_E:   //0x0E
            break;
        case CONTROL_UNKNOWN_F:   //0x0F
            break;
        case CONTROL_UNKNOWN_10:   //0x10
            break;
        case CONTROL_SWITCH_CONTROL:   //0x11
            break;
        case CONTROL_VOICE_TRIGGER_STATE:   //0x12
            DBGLOG_ECONN_DBG("voice trigger state:");
            if (control_value == 1) {
                DBGLOG_ECONN_DBG("enabled\n");
            } else if (control_value == 2) {
                DBGLOG_ECONN_DBG("disabled\n");
            } else {
                DBGLOG_ECONN_DBG("unknown\n");
            }
            break;
        case CONTROL_SIRI_MODE:   //0x13
            DBGLOG_ECONN_DBG("siri is %d\n", control_value);
            break;
        case CONTROL_SINGLE_CLICK:   //0x14
            break;
        case CONTROL_DOUBLE_CLICK:   //0x15
            break;
        case CONTROL_CLICK_AND_HOLD:   //0x16
        {
            uint8_t right_mode = control_value & 0xFF;
            uint8_t left_mode = (control_value >> 8) & 0xFF;
            DBGLOG_ECONN_DBG("click and hold mode: left=%d right=%d\n", left_mode, right_mode);

            key_pressed_type_t type = BTN_TYPE_LONG;
            uint16_t state = STATE_IDLE | STATE_CONNECTABLE | STATE_AG_PAIRING | STATE_CONNECTED
                | STATE_A2DP_STREAMING;
            uint16_t left_event_id = 0;
            uint16_t right_event_id = 0;

            switch (left_mode) {
                case CLICK_AND_HOLD_UNKNOWN:
                    left_event_id = 0;
                    break;
                case CLICK_AND_HOLD_SIRI:
                    left_event_id = EVTUSR_VOICE_RECOGNITION_TOGGLE;
                    break;
                case CLICK_AND_HOLD_PLAY_PAUSE:
                    left_event_id = EVTUSR_MUSIC_PLAY_PAUSE;
                    break;
                case CLICK_AND_HOLD_NEXT_TRACK:
                    left_event_id = EVTUSR_MUSIC_FORWARD;
                    break;
                case CLICK_AND_HOLD_PREV_TRACK:
                    left_event_id = EVTUSR_MUSIC_BACKWARD;
                    break;
                case CLICK_AND_HOLD_NOISE_MANAGEMENT:
                    left_event_id = EVTUSR_LISTEN_MODE_TOGGLE;
                    break;
                default:
                    left_event_id = 0;
                    break;
            }

            switch (right_mode) {
                case CLICK_AND_HOLD_UNKNOWN:
                    right_event_id = 0;
                    break;
                case CLICK_AND_HOLD_SIRI:
                    right_event_id = EVTUSR_VOICE_RECOGNITION_TOGGLE;
                    break;
                case CLICK_AND_HOLD_PLAY_PAUSE:
                    right_event_id = EVTUSR_MUSIC_PLAY_PAUSE;
                    break;
                case CLICK_AND_HOLD_NEXT_TRACK:
                    right_event_id = EVTUSR_MUSIC_FORWARD;
                    break;
                case CLICK_AND_HOLD_PREV_TRACK:
                    right_event_id = EVTUSR_MUSIC_BACKWARD;
                    break;
                case CLICK_AND_HOLD_NOISE_MANAGEMENT:
                    right_event_id = EVTUSR_LISTEN_MODE_TOGGLE;
                    break;
                default:
                    right_event_id = 0;
                    break;
            }
            if (app_wws_is_left()) {
                app_btn_cus_key_add(type, state, left_event_id);
                app_wws_send_set_cus_key(type, state, right_event_id);
            } else {
                app_btn_cus_key_add(type, state, right_event_id);
                app_wws_send_set_cus_key(type, state, left_event_id);
            }
            break;
        }
        case CONTROL_DOUBLE_CLICK_INTERVAL:   //0x17
            break;
        case CONTROL_CLICK_AND_HOLD_INTERVAL:   //0x18
            break;
        case CONTROL_UNKNOWN_19:   //0x19
            break;
        case CONTRL_LISTENING_MODE_CONFIGS:   //0x1A
            DBGLOG_ECONN_DBG("listening mode configs:%d\n", (uint8_t)control_value);
            app_audio_listen_mode_set_toggle_cfg((uint8_t)control_value);
            break;
        case CONTROL_ONE_BUD_ANC_MODE:   //0x1B
            break;
        default:
            break;
    }
}

static void handle_name_update(uint8_t *param, uint16_t param_len)
{
    if (param_len < 3) {
        DBGLOG_ECONN_DBG("handle_name_update param_len error\n");
        return;
    }

    uint8_t type = *param;
    param += 1;
    param_len -= 1;

    uint16_t name_len = 0;
    BT_UNPACK_LE_2_BYTE(&name_len, param);
    param += 2;
    param_len -= 2;

    char name[128];
    memset(name, 0, sizeof(name));
    if (param_len > 127) {
        memcpy_s(name, sizeof(name), param, 127);
    } else {
        memcpy_s(name, sizeof(name), param, param_len);
    }
    usr_cfg_set_local_name(name);
    app_bt_set_local_name(name);

    DBGLOG_ECONN_DBG("name update type:0x%X name:%x\n", type, name);
}

#ifdef AAP_ADV_ENC_ENABLED
static void aap_send_enc_key(void)
{
    if (is_enc_key_sent) {
        return;
    }

    is_enc_key_sent = true;
    static const uint8_t data0[] = {0x04, 0x00, 0x04, 0x00, 0x31, 0x00, 0x01, 0x00,
                                    0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00};
    static const uint8_t data[] = {0x04, 0x00, 0x04, 0x00, 0x31, 0x00, 0x02, 0x01, 0x00, 0x10,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x10,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    app_l2cap_send_data(&remote_addr, AAP_PSM, data0, sizeof(data0));
    app_l2cap_send_data(&remote_addr, AAP_PSM, data, sizeof(data));
}
#endif

static void aap_notify_listen_mode(uint8_t mode)
{
    uint8_t param[5] = {0};
    param[0] = CONTROL_LISTEN_MODE;
    param[1] = mode;

    send_aacp_response(0x0009, param, sizeof(param));
}

static void aap_send_ear_tip_respone(void)
{
    static const uint8_t response_data[] = {
        0x04, 0x00, 0x04, 0x00, 0x21, 0x00, 0x01, 0x03, 0xb2, 0x00, 0x00, 0x00, 0x84, 0x43, 0xd1,
        0x41, 0xce, 0xb8, 0x5e, 0x3e, 0x58, 0xff, 0xe2, 0x3c, 0x9d, 0xdb, 0x0e, 0x40, 0x37, 0xf5,
        0x29, 0x40, 0xe6, 0x69, 0x1e, 0x40, 0xf5, 0xf5, 0x9f, 0x3f, 0x1f, 0xaf, 0xac, 0x3f, 0xb1,
        0xfa, 0x74, 0x3f, 0x78, 0xb6, 0x65, 0x3f, 0xce, 0xd7, 0x55, 0x3f, 0xaa, 0x5a, 0x28, 0x3f,
        0xc5, 0xb1, 0x2c, 0x3f, 0xa3, 0xb0, 0x25, 0x3f, 0x25, 0x20, 0x19, 0x3f, 0xb6, 0xaa, 0xd4,
        0x3e, 0x17, 0x43, 0x28, 0x3f, 0x94, 0x1d, 0x0e, 0x3f, 0xa1, 0x6e, 0xc8, 0x3e, 0x36, 0x82,
        0xf1, 0x3e, 0x03, 0xa1, 0xdf, 0x3e, 0x82, 0x52, 0x0d, 0x3f, 0x4e, 0xad, 0xd9, 0x41, 0x4a,
        0x69, 0x43, 0x3e, 0x80, 0x30, 0x05, 0x3b, 0x05, 0x3f, 0x1d, 0x3e, 0x18, 0x47, 0x9a, 0x3e,
        0xfa, 0x2a, 0x01, 0x3f, 0x8c, 0x7e, 0x0d, 0x3f, 0x62, 0xff, 0x49, 0x3f, 0x06, 0xe2, 0x62,
        0x3f, 0x6a, 0xab, 0x6c, 0x3f, 0xc1, 0x62, 0x75, 0x3f, 0xa6, 0xff, 0x70, 0x3f, 0x28, 0x03,
        0x6c, 0x3f, 0xfd, 0x2a, 0x6d, 0x3f, 0x73, 0xf3, 0x76, 0x3f, 0x87, 0x88, 0x4f, 0x3f, 0x0a,
        0xc8, 0x4a, 0x3f, 0x17, 0x40, 0x57, 0x3f, 0xc1, 0x97, 0x22, 0x3f, 0x3b, 0xee, 0x48, 0x3f,
        0x2a, 0xf4, 0x41, 0x3f, 0xb0, 0x67, 0x1d, 0x3f};
    app_l2cap_send_data(&remote_addr, AAP_PSM, response_data, sizeof(response_data));
}

static void aacp_handle_command(uint16_t cmd, uint8_t *param, uint16_t param_len)
{
    DBGLOG_ECONN_DBG("received aacp cmd:%02X param_len:%d\n", cmd, param_len);

    if (cmd == 0x0001) {   //getCapabilities
        aap_send_capabilities();
        aap_send_role_state(1, 1, 0);
        aap_send_current_battery_info();
        aap_send_version_info();
        aap_notify_listen_mode(app_audio_listen_mode_get());
        aap_send_box_version();
    } else if (cmd == 0x0009) {   //control command
        handle_control_command(param, param_len);
    } else if (cmd == 0x000F) {
        //battery state notification param: -1=in ear detection enabled other:?
        DBGLOG_ECONN_DBG("battery state notification:0x%02X%02X\n", param[3], param[2]);
        aap_send_current_battery_info();
    } else if (cmd == 0x001A) {   //name update
        handle_name_update(param, param_len);
    } else if (cmd == 0x001B) {   //datetime
        if (param_len <= 10) {
            return;
        }
        param += 10;
        param_len -= 10;
        char str[128];
        memset(str, 0, sizeof(str));
        if (param_len < 127) {
            memcpy_s(str, sizeof(str), param, param_len);
        } else {
            memcpy_s(str, sizeof(str), param, 127);
        }
        //DBGLOG_ECONN_DBG("datetime:%s\n", str);
#ifdef AAP_ADV_ENC_ENABLED
        aap_send_enc_key();
#endif
    } else if (cmd == 0x0021) {   //ear tip fit test
        app_cancel_msg(MSG_TYPE_ECONN, AAP_MSG_ID_TIP_RESPONE);
        app_send_msg_delay(MSG_TYPE_ECONN, AAP_MSG_ID_TIP_RESPONE, NULL, 0,
                           AAP_TIP_PLAY_DURATION_MS);
    } else if (cmd == 0x0031) {   //custom command
        DBGLOG_ECONN_DBG("custom command\n");
#ifdef AAP_ADV_ENC_ENABLED
        aap_send_enc_key();
#endif
    }
}

static void l2cap_data_callback(BD_ADDR_T *addr, uint16_t psm, uint8_t *buf, uint16_t len)
{
    UNUSED(addr);
    UNUSED(psm);
    if (app_wws_is_slave()) {
        DBGLOG_ECONN_DBG("aap slave recv:%d, ignored\n", len);
        os_mem_free(buf);
        return;
    }

    if ((len == sizeof(APP_CONNECT_REQUEST)) && (!memcmp(buf, APP_CONNECT_REQUEST, 4))) {
        DBGLOG_ECONN_DBG("received aap connect request\n");
        app_l2cap_send_data(&remote_addr, AAP_PSM, APP_CONNECT_RESPONSE,
                            sizeof(APP_CONNECT_RESPONSE));
        os_mem_free(buf);
        return;
    } else if (len == sizeof(AAP_DISCONNECT_REQUEST) && (!memcmp(buf, AAP_DISCONNECT_REQUEST, 4))) {
        DBGLOG_ECONN_DBG("received aap disconnect request\n");
        app_l2cap_send_data(&remote_addr, AAP_PSM, AAP_DISCONNECT_RESPONSE,
                            sizeof(AAP_DISCONNECT_RESPONSE));
        os_mem_free(buf);
        return;
    } else if (len == sizeof(AAP_DISCONNECT_RESPONSE)
               && (!memcmp(buf, AAP_DISCONNECT_RESPONSE, 4))) {
        DBGLOG_ECONN_DBG("recevied aap disconnect response\n");
        os_mem_free(buf);
        return;
    }

    if (len < sizeof(AACP_HEADER) + sizeof(uint16_t)) {
        DBGLOG_ECONN_DBG("unknown aap data,len error\n");
        os_mem_free(buf);
        return;
    }

    if (memcmp(buf, AACP_HEADER, sizeof(AACP_HEADER))) {
        DBGLOG_ECONN_DBG("unknown aap data, header error");
        os_mem_free(buf);
        return;
    }

    uint16_t cmd;
    uint8_t *param = buf + sizeof(AACP_HEADER) + sizeof(uint16_t);
    uint16_t param_len = len - sizeof(AACP_HEADER) - sizeof(uint16_t);
    BT_UNPACK_LE_2_BYTE(&cmd, buf + sizeof(AACP_HEADER));

    aacp_handle_command(cmd, param, param_len);
    os_mem_free(buf);
}

static void l2cap_connection_callback(BD_ADDR_T *addr, uint16_t psm, bool_t connected)
{
    UNUSED(psm);

    remote_addr = *addr;
    aap_connected = connected;
    is_enc_key_sent = false;

    DBGLOG_ECONN_DBG("l2cap_connection_callback connected:%d\n", connected);
    if (connected && app_wws_is_master()) {
        app_cancel_msg(MSG_TYPE_ECONN, AAP_MSG_ID_GATT_CONNECT);
        app_send_msg_delay(MSG_TYPE_ECONN, AAP_MSG_ID_GATT_CONNECT, NULL, 0, 5 * 1000);
    }
    if (connected) {
        aap_start_adv();
    }
}

static void aap_disconnect(void)
{
    if (!aap_connected) {
        return;
    }
    DBGLOG_ECONN_DBG("aap start disconnect\n");
    app_l2cap_send_data(&remote_addr, AAP_PSM, AAP_DISCONNECT_REQUEST,
                        sizeof(AAP_DISCONNECT_REQUEST));
}

static void aap_add_sdp_record(void)
{
    static const uint8_t did_attribute_1[] = {
        0x09, 0x00, 0x01, 0x35, 0x03, 0x19, 0x12, 0x00,
    };
    static const uint8_t did_attribute_2[] = {
        0x09, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00, 0x00,
    };
    static const uint8_t did_attribute_3[] = {
        0x09, 0x00, 0x05, 0x35, 0x03, 0x19, 0x10, 0x02,
    };
    static const uint8_t did_attribute_4[] = {
        0x09, 0x00, 0x06, 0x35, 0x24, 0x09, 0x65, 0x6E, 0x09, 0x00, 0x6A, 0x09, 0x01, 0x00,
        0x09, 0x66, 0x72, 0x09, 0x00, 0x6A, 0x09, 0x01, 0x10, 0x09, 0x64, 0x65, 0x09, 0x00,
        0x6A, 0x09, 0x01, 0x20, 0x09, 0x6A, 0x61, 0x09, 0x00, 0x6A, 0x09, 0x01, 0x30,
    };
    static const uint8_t did_attribute_5[] = {
        0x09, 0x00, 0x08, 0x08, 0xFF,
    };
    static const uint8_t did_attribute_6[] = {
        0x09, 0x01, 0x01, 0x25, 0x0F, 0x50, 0x6E, 0x50, 0x20, 0x49,
        0x6E, 0x66, 0x6F, 0x72, 0x6D, 0x61, 0x74, 0x69, 0x6F, 0x6E,
    };
    static const uint8_t did_attribute_7[] = {
        0x09, 0x02, 0x00, 0x09, 0x01, 0x02,
    };
    static const uint8_t did_attribute_8[] = {
        0x09, 0x02, 0x01, 0x09, 0x00, 0x4C,
    };

    static const uint8_t did_attribute_9[] = {
        0x09, 0x02, 0x02, 0x09, (FEATURE_MODULE_ID >> 8) & 0xFF, FEATURE_MODULE_ID & 0xFF,
    };

    static const uint8_t did_attribute_A_1[] = {
        0x09, 0x02, 0x03, 0x09, 0x06, 0x78,
    };
    static const uint8_t did_attribute_A_2[] = {0x09, 0x02, 0x03, 0x09, 0x41, 0x6C};
    static const uint8_t did_attribute_A_pro[] = {
        0x09, 0x02, 0x03, 0x09, 0x61, 0x1B,
    };
    UNUSED(did_attribute_A_1);
    UNUSED(did_attribute_A_2);
    UNUSED(did_attribute_A_pro);
    static const uint8_t did_attribute_B[] = {
        0x09, 0x02, 0x04, 0x28, 0x01,
    };
    static const uint8_t did_attribute_C[] = {
        0x09, 0x02, 0x05, 0x09, 0x00, 0x01,
    };
    static const uint8_t did_attribute_D_1[] = {
        0x09,
        0xA0,
        0x00,
        /**
 * features:
 *  0x02: mic settings/left,right,auto/in_ear detection/+VGS=0
 *  0x4000:enhanced double tap
 *  0x400000: lea audio,siri
 *  0x08000000: ShareAudio
 */
        0x0A,
        0x08,
        0x00 & (~0x40),
        0x40,
        0xFF,
    };
    static const uint8_t did_attribute_D_2[] = {
        0x09,
        0xA0,
        0x00,
        /**
 * features:
 *  0x02: mic settings/left,right,auto/in_ear detection/+VGS=0
 *  0x4000:enhanced double tap
 *  0x400000: lea audio,siri
 *  0x08000000: ShareAudio
 */
        0x0A,
        0x08,
        0x00 & (~0x40),
        0x40,
        0xFF,
    };
    static const uint8_t did_attribute_D_pro[] = {
        0x09,
        0xA0,
        0x00,
        /**
 * features:
 *  0x02: mic settings/left,right,auto/in_ear detection/+VGS=0
 *  0x4000:enhanced double tap
 *  0x400000: lea audio,siri
 *  0x08000000: ShareAudio
 */
        0x0A,
        0x7F,
        0XFB,
        0x30,
        0xFF,
    };
    UNUSED(did_attribute_D_1);
    UNUSED(did_attribute_D_2);
    UNUSED(did_attribute_D_pro);
    static const uint8_t did_attribute_E[] = {
        0x09, 0xA0, 0x01, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
    };
    static const uint8_t did_attribute_F[] = {
        0x09, 0xAF, 0xFF, 0x09, 0x00, 0x02,
    };
    static const uint8_t aap_attribute_1[] = {0x09, 0x00, 0x01, 0x35, 0x11, 0x1c, 0x74, 0xec,
                                              0x21, 0x72, 0x0b, 0xad, 0x4d, 0x01, 0x8f, 0x77,
                                              0x99, 0x7b, 0x2b, 0xe0, 0x72, 0x2a};
    static const uint8_t aap_attribute_2[] = {
        0x09, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, 0x00,
    };
    static const uint8_t aap_attribute_3[] = {
        0x09, 0x00, 0x04, 0x35, 0x08, 0x35, 0x06, 0x19, 0x01, 0x00, 0x09, 0x10, 0x01,
    };
    static const uint8_t aap_attribute_4[] = {0x09, 0x00, 0x05, 0x35, 0x03, 0x19, 0x10, 0x02};
    static const uint8_t aap_attribute_5[] = {
        0x09, 0x00, 0x06, 0x35, 0x24, 0x09, 0x65, 0x6e, 0x09, 0x00, 0x6a, 0x09, 0x01, 0x00,
        0x09, 0x66, 0x72, 0x09, 0x00, 0x6a, 0x09, 0x01, 0x10, 0x09, 0x64, 0x65, 0x09, 0x00,
        0x6a, 0x09, 0x01, 0x20, 0x09, 0x6a, 0x61, 0x09, 0x00, 0x6a, 0x09, 0x01, 0x30};
    static const uint8_t aap_attribute_6[] = {0x09, 0x00, 0x08, 0x08, 0xff};
    static const uint8_t aap_attribute_7[] = {0x09, 0x00, 0x09, 0x35, 0x16, 0x35, 0x14, 0x1c, 0x4b,
                                              0x6f, 0x7c, 0x74, 0x07, 0xf4, 0x49, 0xde, 0xb0, 0xb9,
                                              0xab, 0x43, 0x04, 0x72, 0x8f, 0x29, 0x09, 0x01, 0x00};
    static const uint8_t aap_attribute_8[] = {0x09, 0x01, 0x00, 0x25, 0x0a, 0x41, 0x41, 0x50,
                                              0x20, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72};

    static const bt_service_attribute_t did_service_attribute[] = {
        {did_attribute_1, sizeof(did_attribute_1)},
        {did_attribute_2, sizeof(did_attribute_2)},
        {did_attribute_3, sizeof(did_attribute_3)},
        {did_attribute_4, sizeof(did_attribute_4)},
        {did_attribute_5, sizeof(did_attribute_5)},
        {did_attribute_6, sizeof(did_attribute_6)},
        {did_attribute_7, sizeof(did_attribute_7)},
        {did_attribute_8, sizeof(did_attribute_8)},
        {did_attribute_9, sizeof(did_attribute_9)},
#if (MODULE_ID == MODULE_ID_I_1)
        {did_attribute_A_1, sizeof(did_attribute_A_1)},
#elif (MODULE_ID == MODULE_ID_I_2)
        {did_attribute_A_2, sizeof(did_attribute_A_2)},
#else
        {did_attribute_A_pro, sizeof(did_attribute_A_pro)},
#endif
        {did_attribute_B, sizeof(did_attribute_B)},
        {did_attribute_C, sizeof(did_attribute_C)},
#if (MODULE_ID == MODULE_ID_I_1)
        {did_attribute_D_1, sizeof(did_attribute_D_1)},
#elif (MODULE_ID == MODULE_ID_I_2)
        {did_attribute_D_2, sizeof(did_attribute_D_2)},
#else
        {did_attribute_D_pro, sizeof(did_attribute_D_pro)},
#endif
        {did_attribute_E, sizeof(did_attribute_E)},
        {did_attribute_F, sizeof(did_attribute_F)},
    };

    static const bt_service_attribute_t aap_service_attribute[] = {
        {aap_attribute_1, sizeof(aap_attribute_1)}, {aap_attribute_2, sizeof(aap_attribute_2)},
        {aap_attribute_3, sizeof(aap_attribute_3)}, {aap_attribute_4, sizeof(aap_attribute_4)},
        {aap_attribute_5, sizeof(aap_attribute_5)}, {aap_attribute_6, sizeof(aap_attribute_6)},
        {aap_attribute_7, sizeof(aap_attribute_7)}, {aap_attribute_8, sizeof(aap_attribute_8)},
    };

    static const uint8_t aap_uuid[] = {0x74, 0xec, 0x21, 0x72, 0x0b, 0xad, 0x4d, 0x01,
                                       0x8f, 0x77, 0x99, 0x7b, 0x2b, 0xe0, 0x72, 0x2a};
    static const uint8_t l2cap_uuid[] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00,
                                         0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
    static const uint8_t did_uuid[] = {0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x10, 0x00,
                                       0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
    uint8_t realated_profile[] = {app_sdp_get_uuid_index(did_uuid)};
    uint8_t custom_realated_profile[] = {app_sdp_get_uuid_index(l2cap_uuid),
                                         app_sdp_add_uuid(aap_uuid)};
    int result;
    result = app_sdp_register_record(ARRAY_LEN(realated_profile), realated_profile,
                                     ARRAY_LEN(did_service_attribute), did_service_attribute);
    if (result) {
        DBGLOG_ECONN_DBG("add sdp record failed:%d.\n", result);
        return;
    }
    result = app_sdp_register_record(ARRAY_LEN(custom_realated_profile), custom_realated_profile,
                                     ARRAY_LEN(aap_service_attribute), aap_service_attribute);
    if (result) {
        DBGLOG_ECONN_DBG("add sdp record failed:%d.\n", result);
        return;
    }
}

static void aap_start_adv(void)
{
    uint32_t sys_state;

    if (app_wws_is_slave()) {
        app_adv_set_enabled(false);
        return;
    }

    sys_state = app_bt_get_sys_state();

    if (sys_state <= STATE_IDLE) {
        app_adv_set_enabled(false);
        return;
    }

    if (!adv_enabled) {
        app_adv_set_enabled(false);
        return;
    }

    /*Should not pop while not in tws connected state*/
    if (!app_wws_is_connected_master()) {
        return;
    }

    if (usr_cfg_pdl_is_empty()
        && ((sys_state == STATE_CONNECTABLE) || (sys_state == STATE_AG_PAIRING))) {
        DBGLOG_ECONN_DBG("pdl empty set to ag pairing mode\n");
        is_ag_pairing_adv = true;
    }

    if (is_ag_pairing_adv) {
        aap_adv_pairing();
        return;
    }

    if (sys_state < STATE_CONNECTED) {
        aap_adv_paired();
    } else {   // sys_state >= STATE_CONNECTED
        if (!aap_connected) {
            app_adv_set_enabled(false);
        } else {
            aap_adv_paired();
            app_cancel_msg(MSG_TYPE_ECONN, AAP_MSG_ID_STOP_ADV);
            app_send_msg_delay(MSG_TYPE_ECONN, AAP_MSG_ID_STOP_ADV, NULL, 0,
                               AAP_CONNECTED_ADV_DURATION_MS);
        }
    }
}

static void app_econn_aap_handle_msg(uint16_t msg_id, void *param)
{
    UNUSED(param);

    switch (msg_id) {
        case AAP_MSG_ID_ENABLE_ADV:
            adv_enabled = true;
            DBGLOG_ECONN_DBG("aap adv enabled\n");
            aap_start_adv();
            break;
        case AAP_MSG_ID_STOP_ADV:
            app_adv_set_enabled(false);
            break;
        case AAP_MSG_ID_TIP_RESPONE:
            aap_send_ear_tip_respone();
            break;
        case AAP_MSG_ID_GATT_CONNECT:
            if (app_wws_is_master() && app_bt_get_sys_state() >= STATE_CONNECTED) {
                app_doap_connect(&remote_addr);
            } else {
                DBGLOG_ECONN_DBG("doap_connect failed,wws_master %d sys_state %d\n",
                                 app_wws_is_master(), app_bt_get_sys_state());
            }
            break;
        default:
            app_doap_handle_msg(msg_id, param);
            break;
    }
}

void app_econn_init(void)
{
    DBGLOG_ECONN_DBG("app_econn_aap\n");
    app_register_msg_handler(MSG_TYPE_ECONN, app_econn_aap_handle_msg);
    app_l2cap_register_service(AAP_PSM, l2cap_connection_callback, l2cap_data_callback);
    app_send_msg_delay(MSG_TYPE_ECONN, AAP_MSG_ID_ENABLE_ADV, NULL, 0,
                       AAP_DISABLE_ADV_AFTER_BOOT_MS);
    app_doap_init();
}

void app_econn_deinit(void)
{
    UNUSED(aap_send_eary_pair_response);
    UNUSED(aap_status_to_headset_status);
    UNUSED(get_listen_mode_str);
    UNUSED(get_listen_mode_configs_str);
    UNUSED(get_click_and_hold_mode_str);
    UNUSED(aap_disconnect);
    UNUSED(aap_send_triangle_status);
}

bool_t app_econn_exists(void)
{
    return true;
}

void app_econn_enter_ag_pairing(void)
{
}

void app_econn_enable_ag_pairing_adv(void);
void app_econn_enable_ag_pairing_adv(void)
{
    is_ag_pairing_adv = true;
    aap_start_adv();
}

void app_econn_handle_sys_state(uint32_t sys_state)
{
    UNUSED(sys_state);

    if ((current_sys_state < STATE_WWS_PAIRING)
        && (sys_state >= STATE_WWS_PAIRING)) {   //bt power on
        aap_add_sdp_record();
    }

    if ((current_sys_state < STATE_CONNECTED) && (sys_state >= STATE_CONNECTED)) {
        is_ag_pairing_adv = false;
    }

    current_sys_state = sys_state;

    aap_start_adv();
}

void app_econn_handle_listen_mode_changed(uint8_t mode)
{
    uint8_t param[5] = {0};
    param[0] = CONTROL_LISTEN_MODE;
    param[1] = mode;

    send_aacp_response(0x0009, param, sizeof(param));
}

static void app_handle_inear_action_for_android(bool_t in_ear)
{
    if (in_ear) {
        if (pause_streaming_by_outear && app_bt_get_sys_state() == STATE_CONNECTED) {
            app_evt_send(EVTUSR_MUSIC_PLAY);
            pause_streaming_by_outear = false;
            DBGLOG_ECONN_DBG("app_handle_inear_action_for_android - play\n");
        }
    } else {
        if (app_bt_get_sys_state() == STATE_A2DP_STREAMING) {
            pause_streaming_by_outear = true;
            app_evt_send(EVTUSR_MUSIC_PAUSE);
            DBGLOG_ECONN_DBG("app_handle_inear_action_for_android - pause\n");
        }
    }
}

void app_econn_handle_in_ear_changed(bool_t in_ear)
{
    if (in_ear_detection_enabled) {
        if (aap_connected) {
            aap_send_current_in_ear_status();
        } else {
            app_handle_inear_action_for_android(in_ear);
        }
    }
}

void app_econn_handle_peer_in_ear_changed(bool_t in_ear)
{
    if (in_ear_detection_enabled) {
        if (aap_connected) {
            aap_send_current_in_ear_status();
        } else {
            app_handle_inear_action_for_android(in_ear);
        }
    }
}

void app_econn_handle_battery_level_changed(uint8_t level)
{
    UNUSED(level);
    aap_start_adv();
    aap_send_current_battery_info();
}

void app_econn_handle_peer_battery_level_changed(uint8_t level)
{
    UNUSED(level);
    aap_start_adv();
}

void app_econn_handle_tws_state_changed(bool_t connected)
{
    if (connected && app_wws_is_master()) {
        aap_send_current_in_ear_status();
    }
}

bool_t app_econn_handle_usr_evt(uint16_t event, bool_t from_peer)
{
    UNUSED(from_peer);

    bool_t ret = false;

    switch (event) {
        case EVTUSR_ENTER_FACTORY_TEST_MODE:
            //hy2751 gpio conflict with uart tx
            inear_sensor_deinit();
            ret = false;
            break;
        case EVTUSR_LISTEN_MODE_TOGGLE:
            /*Listen mode toggle invalid while not in tws connected-state*/
            if (!app_wws_is_connected()) {
                ret = true;
            }
            break;
        case EVTUSR_VOICE_RECOGNITION_TOGGLE:
            DBGLOG_ECONN_DBG("customer EVTUSR_VOICE_RECOGNITION_TOGGLE\n");
            app_doap_toggle();
            ret = true;
            break;
        default:
            ret = false;
            break;
    }

    return ret;
}

bool_t app_econn_handle_sys_evt(uint16_t event, void *param)
{
    UNUSED(param);

    bool_t ret = false;

    switch (event) {
        case EVTSYS_IN_EAR:
        case EVTSYS_OUT_OF_EAR:
            if (app_wws_is_connected()
                && (app_wws_peer_is_inear() || app_bt_get_sys_state() == STATE_A2DP_STREAMING)) {
                DBGLOG_ECONN_DBG("filter tone ...peer: %d,state:%d \n", app_wws_peer_is_inear(),
                                 app_bt_get_sys_state());
                ret = true;
            } else if (!app_wws_is_connected())
                ret = false;
            break;
        case EVTSYS_BT_POWER_OFF:
            if (app_charger_is_charging()) {
                DBGLOG_ECONN_DBG("customer EVTSYS_BT_POWER_OFF ignored when charging\n");
                ret = true;
            }
            break;
        case EVTSYS_WWS_PAIRED:
            aap_start_adv();
            ret = false;
            break;
        case EVTSYS_WWS_PAIR_FAILED:
            app_bt_set_discoverable(false);
            ret = false;
            break;
        case EVTSYS_WWS_CONNECTED:
            aap_start_adv();
            ret = false;
            break;
        case EVTSYS_CONNECTED:
            is_ag_pairing_adv = false;
            aap_start_adv();
            ret = false;
            break;
        case EVTSYS_INEAR_DISABLE:
        case EVTSYS_INEAR_ENABLE:
            /*do not play this sys tone when trigger on iphone*/
            if (aap_connected) {
                ret = true;
            }
            break;

        default:
            break;
    }

    return ret;
}

void app_econn_handle_ntc_value(int8_t value)
{
    UNUSED(value);
}

void app_econn_handle_tws_role_changed(bool_t is_master)
{
    UNUSED(is_master);
}

void app_econn_handle_charging_changed(bool_t charging)
{
    UNUSED(charging);
    aap_start_adv();
    aap_send_current_battery_info();
}

void app_econn_handle_peer_charging_changed(bool_t charging)
{
    UNUSED(charging);
    aap_start_adv();
    aap_send_current_battery_info();
}

void app_econn_handle_box_state_changed(box_state_t state)
{
    UNUSED(state);
    aap_start_adv();
    aap_send_current_battery_info();
}

void app_econn_handle_peer_box_state_changed(box_state_t state)
{
    UNUSED(state);
    aap_start_adv();
    aap_send_current_battery_info();
}
#endif
