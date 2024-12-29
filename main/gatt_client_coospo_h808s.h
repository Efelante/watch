#ifndef __GATT__H808S__
#define __GATT__H808S__

#define GATTC_TAG "GATTC_H808S"
#define REMOTE_SERVICE_UUID        0x180D
#define REMOTE_NOTIFY_CHAR_UUID    0x2A37
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

#define COOSPO_H808S_NAME "808S 0006976"

/* Declare functions */
void coospo_connect(void);

#endif //__GATT__H808S__

