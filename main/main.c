/****************************************************************************************
* Includes
 ****************************************************************************************/

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

static char       peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static const char remote_device_name[] = "Pawpaw";

/****************************************************************************************
 * Global Variables
 ****************************************************************************************/

esp_bd_addr_t peer_bd_addr     = {0};
uint8_t       peer_bdname_len  = 0;
uint8_t       water_level      = 0;
bool          server_connected = false;

/****************************************************************************************
 * Function Definitions
 ****************************************************************************************/

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
  uint8_t *rmt_bdname     = NULL;
  uint8_t  rmt_bdname_len = 0;

  if (!eir)
  {
    return false;
  }

  rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
  if (!rmt_bdname)
  {
    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
  }

  if (rmt_bdname)
  {
    if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN)
    {
      rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
    }

    if (bdname)
    {
      memcpy(bdname, rmt_bdname, rmt_bdname_len);
      bdname[rmt_bdname_len] = '\0';
    }
    if (bdname_len)
    {
      *bdname_len = rmt_bdname_len;
    }
    return true;
  }

  return false;
}

/****************************************************************************************
 * Callbacks
 ****************************************************************************************/

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
  switch (event)
  {
    case ESP_BT_GAP_DISC_RES_EVT:
      ESP_LOGI("Bluetooth", "ESP_BT_GAP_DISC_RES_EVT");
      /* Find the target peer device name in the EIR data */
      for (int i = 0; i < param->disc_res.num_prop; i++)
      {
        if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len))
        {
          if (strlen(remote_device_name) == peer_bdname_len && strncmp(peer_bdname, remote_device_name, peer_bdname_len) == 0)
          {
            memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
            /* Have found the target peer device, cancel the previous GAP discover procedure. And go on
                     * dsicovering the SPP service on the peer device */
            esp_bt_gap_cancel_discovery();
            esp_spp_start_discovery(peer_bd_addr);
          }
        }
      }
      break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
      if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
      {
        ESP_LOGI("Bluetooth", "authentication success: %s", param->auth_cmpl.device_name);
        ESP_LOG_BUFFER_HEX("Bluetooth", param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
      }
      else
      {
        ESP_LOGE("Bluetooth", "authentication failed, status:%d", param->auth_cmpl.stat);
      }
      break;
    }

    default:
      break;
  }
}

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  switch (event)
  {
    case ESP_SPP_INIT_EVT:
      if (param->init.status == ESP_SPP_SUCCESS)
      {
        ESP_LOGI("Bluetooth", "ESP_SPP_INIT_EVT");
        esp_bt_gap_set_device_name("tron-receiver");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
      }
      else
      {
        ESP_LOGE("Bluetooth", "ESP_SPP_INIT_EVT status:%d", param->init.status);
      }
      break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
      if (param->disc_comp.status == ESP_SPP_SUCCESS)
      {
        ESP_LOGI("Bluetooth", "ESP_SPP_DISCOVERY_COMP_EVT scn_num:%d", param->disc_comp.scn_num);
        for (uint8_t i = 0; i < param->disc_comp.scn_num; i++)
        {
          ESP_LOGI("Bluetooth", "-- [%d] scn:%d service_name:%s", i, param->disc_comp.scn[i],
                   param->disc_comp.service_name[i]);
        }
        /* We only connect to the first found server on the remote SPP acceptor here */
        esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, param->disc_comp.scn[0], peer_bd_addr);
      }
      else
      {
        ESP_LOGE("Bluetooth", "ESP_SPP_DISCOVERY_COMP_EVT status=%d", param->disc_comp.status);
      }
      break;
    case ESP_SPP_OPEN_EVT:
      if (param->open.status == ESP_SPP_SUCCESS)
      {
        /* Start to write the first data packet */
        ESP_LOGI("Bluetooth", "ESP_SPP_OPEN_EVT: successfully connected to remote device");
        server_connected = true;
      }
      else
      {
        ESP_LOGE("Bluetooth", "ESP_SPP_OPEN_EVT status:%d", param->open.status);
      }
      break;
    case ESP_SPP_CLOSE_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_CLOSE_EVT status:%d handle:%" PRIu32 " close_by_remote:%d", param->close.status,
               param->close.handle, param->close.async);
      server_connected = false;
      break;
    case ESP_SPP_START_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_START_EVT");
      break;
    case ESP_SPP_DATA_IND_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_DATA_IND_EVT");
      if (param->data_ind.len == 1)
      {
        water_level = param->data_ind.data[0];
        ESP_LOGI("Bluetooth", "Water Level: %d", water_level);
      }
      break;
    default:
      break;
  }
}

/****************************************************************************************
 * Tasks
 ****************************************************************************************/

void spp_uart_task(void *pvParameters)
{
  while (1)
  {
    if (server_connected)
    {
      ESP_LOGI("UART", "water level: %d", water_level);
    }
    else
    {
      ESP_LOGI("UART", "Not connected to server");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/****************************************************************************************
 * Main
 ****************************************************************************************/

void app_main(void)
{
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(ret);
  esp_bt_controller_config_t bt_cfg        = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bluedroid_config_t     bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  esp_spp_cfg_t              spp_cfg       = {
                             .mode              = ESP_SPP_MODE_CB,
                             .enable_l2cap_ertm = true,
                             .tx_buffer_size    = 0,
  };
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
  ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
  ESP_ERROR_CHECK(esp_bluedroid_enable());
  ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));
  ESP_ERROR_CHECK(esp_spp_register_callback(spp_callback));
  ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

  xTaskCreate(spp_uart_task, "spp_uart_task", 2048, NULL, 10, NULL);

  while (1)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
