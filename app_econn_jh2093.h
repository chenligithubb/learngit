#ifndef _APP_ECONN_JH2093_H__
#define _APP_ECONN_JH2093_H__

#include "storage_controller.h"

#define ECONN_MSG_ID_AT_TEST_CMD                  1
#define ECONN_MSG_ID_AT_TEST_BTN                  2
#define ECONN_MSG_ID_FACTORY_RESET                3
#define ECONN_REMOTE_MSG_ID_BTN_RESET             4
#define ECONN_REMOTE_MSG_ID_UPDATE_AUTO_POWER_OFF 5
#define ECONN_REMOTE_MSG_ID_UPDATE_EQ             6
#define ECONN_REMOTE_MSG_ID_UPDATE_BTN            7
#define ECONN_REMOTE_MSG_ID_UPDATE_TOUCH_TONE     8
#define ECONN_REMOTE_MSG_ID_DELIVER_CMD           9
#define ECONN_MSG_ID_FORCE_DISCOVERABLE           10
#define ECONN_MSG_ID_RESTART_AUTO_POWER_OFF       11
#define ECONN_MSG_ID_CLAER_LAST_BATTERY_LOW       12
#define ECONN_MSG_ID_REPORT_TWS_STATE             13
#define ECONN_MSG_ID_CHECK_SPP_STATE              14
#define ECONN_REMOTE_MSG_ID_GET_UI_STATISTICS     15
#define ECONN_REMOTE_MSG_ID_GET_UI_STATISTICS_RSP 16
#define ECONN_REMOTE_MSG_ID_CLAER_UI_STATISTICS   17

void app_at_test_msg_handler(uint16_t msg_id, void *param);
/**
 * @brief handle box battery low
 *
 */
void econn_handle_box_battery_low(bool_t battery_low);

#endif
