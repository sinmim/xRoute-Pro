#pragma once

/************************************************
 *   Safe Custom NimBLE Configuration
 ************************************************/

// ---- Basic ----
#ifndef CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS     4
#endif

#ifndef CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU
#define CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU    500
#endif

#ifndef CONFIG_BT_NIMBLE_MAX_BONDS
#define CONFIG_BT_NIMBLE_MAX_BONDS            4
#endif

#ifndef CONFIG_BT_NIMBLE_MAX_CCCDS
#define CONFIG_BT_NIMBLE_MAX_CCCDS            8
#endif

#ifndef CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME
#define CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME  "xRoutePro"
#endif

// ---- Roles ----
#ifndef CONFIG_BT_NIMBLE_ROLE_CENTRAL
#define CONFIG_BT_NIMBLE_ROLE_CENTRAL
#endif

#ifndef CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
#define CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
#endif

#ifndef CONFIG_BT_NIMBLE_ROLE_OBSERVER
#define CONFIG_BT_NIMBLE_ROLE_OBSERVER
#endif

#ifndef CONFIG_BT_NIMBLE_ROLE_BROADCASTER
#define CONFIG_BT_NIMBLE_ROLE_BROADCASTER
#endif

// ---- Security ----
#ifndef CONFIG_BT_NIMBLE_SECURITY
#define CONFIG_BT_NIMBLE_SECURITY             1
#endif

#ifndef CONFIG_BT_NIMBLE_NVS_PERSIST
#define CONFIG_BT_NIMBLE_NVS_PERSIST           1
#endif

#ifndef CONFIG_BT_NIMBLE_SM_LEGACY
#define CONFIG_BT_NIMBLE_SM_LEGACY             1
#endif

#ifndef CONFIG_BT_NIMBLE_SM_SC
#define CONFIG_BT_NIMBLE_SM_SC                 1
#endif

// ---- Buffers ----
#ifndef CONFIG_BT_NIMBLE_MSYS1_BLOCK_COUNT
#define CONFIG_BT_NIMBLE_MSYS1_BLOCK_COUNT    24
#endif

// ---- Host/Stack ----
#ifndef CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE
#define CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE 8192 //TODO chon jsonharo tooye parseram estefade mikonam fek konam baese stack ovf mishe bekhatere in az 4k be 8k bordam bala . badan bayad dorostesh konama ba ye taske jodagoone va gheyre
#endif

#ifndef CONFIG_BT_NIMBLE_USE_ESP_TIMER
#define CONFIG_BT_NIMBLE_USE_ESP_TIMER         1
#endif

// ---- Logging ----
#ifndef CONFIG_BT_NIMBLE_LOG_LEVEL
#define CONFIG_BT_NIMBLE_LOG_LEVEL             5
#endif

#ifndef CONFIG_NIMBLE_CPP_LOG_LEVEL
#define CONFIG_NIMBLE_CPP_LOG_LEVEL            0
#endif

// ---- Optional (Speed Optimizations) ----
#ifndef CONFIG_BT_NIMBLE_HS_FLOW_CTRL
#define CONFIG_BT_NIMBLE_HS_FLOW_CTRL           1
#endif

#ifndef CONFIG_BT_NIMBLE_HS_FLOW_CTRL_ITVL
#define CONFIG_BT_NIMBLE_HS_FLOW_CTRL_ITVL      500
#endif

#ifndef CONFIG_BT_NIMBLE_HS_FLOW_CTRL_THRESH
#define CONFIG_BT_NIMBLE_HS_FLOW_CTRL_THRESH    1
#endif

#ifndef CONFIG_BT_NIMBLE_HS_FLOW_CTRL_TX_ON_DISCONNECT
#define CONFIG_BT_NIMBLE_HS_FLOW_CTRL_TX_ON_DISCONNECT 1
#endif

// ---- Important Note ----
// NOT enabling Extended Advertising because ESP32 WROOM chips DO NOT support it.
