#include <driver/i2s.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <esp_a2dp_api.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include "BTSubsystems.h"
#include "BTCaster.h"

static esp_a2d_audio_state_t s_audio_state;
static uint32_t s_pkt_cnt;

RingbufHandle_t hI2SRingBuffer_;

esp_err_t BTSubsystems::initializeBT()
{
    esp_log_level_set(BTSUBSYSTEMS_TAG, ESP_LOG_DEBUG);
    
    ESP_LOGI(BTSUBSYSTEMS_TAG, " Preparing BT...");

    esp_err_t err = ESP_ERR_NOT_SUPPORTED;

    err = esp_bt_mem_release(ESP_BT_MODE_BLE);
    ESP_ERROR_CHECK(err);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(err);

    err = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    ESP_ERROR_CHECK(err);

    err = esp_bluedroid_init();
    ESP_ERROR_CHECK(err);

    err = esp_bluedroid_enable();
    ESP_ERROR_CHECK(err);

    esp_bt_pin_code_t pin_code; 
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';

    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);

    s_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;

    ESP_LOGI(BTSUBSYSTEMS_TAG, "Registering subsystem callbacks");

    esp_bt_dev_set_device_name("BT Caster");

    esp_bt_gap_register_callback([](esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
                                                 { BTCaster::postMessage(DispatchTarget_t::BT_GAP, (uint16_t)(event), (void *)(param), sizeof(esp_bt_gap_cb_param_t)); });

    esp_a2d_sink_init();
    esp_a2d_register_callback([](esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
                                              { BTCaster::postMessage(DispatchTarget_t::A2DP, (uint16_t)(event), (void *)(param), sizeof(esp_a2d_cb_param_t)); });

    esp_a2d_sink_register_data_callback([](const uint8_t *data, uint32_t len)
                                        {
        if (NULL != hI2SRingBuffer_)
            xRingbufferSend(hI2SRingBuffer_, (void *)data, len, (TickType_t)portMAX_DELAY); 
        else
            ESP_LOGW(BTSUBSYSTEMS_TAG, "Data callback with no ringbuffer handle!!"); });

    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    btInitialized_ = true;

    return err;
}

void BTSubsystems::a2dCallback(uint16_t event, void *param)
{
    if (!btInitialized_)
    {
        ESP_LOGE(BTSUBSYSTEMS_TAG, "BT NOT INITIALIZED, NO WORK");
        return;
    }

    esp_a2d_cb_param_t *a2d = NULL;
    a2d = (esp_a2d_cb_param_t *)(param);

    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(BTSUBSYSTEMS_TAG, "Dispatching A2D connection state");
        handleA2DConnectionStates(a2d->conn_stat);
        break;

    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(BTSUBSYSTEMS_TAG, "A2DP audio state: %s", BTSubsystems::A2DAudioStateStrings[a2d->audio_stat.state]);
        s_audio_state = a2d->audio_stat.state;
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state)
        {
            s_pkt_cnt = 0;
        }
        break;

    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(BTSUBSYSTEMS_TAG, "A2DP audio stream configuration, codec type: %d", a2d->audio_cfg.mcc.type);
        /* for now only SBC stream is supported */
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC)
        {
            int sample_rate = 16000;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6))
            {
                sample_rate = 32000;
            }
            else if (oct0 & (0x01 << 5))
            {
                sample_rate = 44100;
            }
            else if (oct0 & (0x01 << 4))
            {
                sample_rate = 48000;
            }

            //post message for dac to (re)configure parameters
            BTCaster::postMessage(DispatchTarget_t::DAC, 0, NULL, 0);

            i2s_set_clk(i2s_port_t(0), sample_rate, i2s_bits_per_sample_t(16), i2s_channel_t(2));

            ESP_LOGI(BTSUBSYSTEMS_TAG, "Configure audio player: %x-%x-%x-%x",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            ESP_LOGI(BTSUBSYSTEMS_TAG, "Audio player configured, sample rate: %d", sample_rate);
        }
        break;

    case ESP_A2D_PROF_STATE_EVT:
        if (ESP_A2D_INIT_SUCCESS == a2d->a2d_prof_stat.init_state)
        {
            ESP_LOGI(BTSUBSYSTEMS_TAG, "ESP A2D state : INIT");
        }
        else if (ESP_A2D_DEINIT_SUCCESS == a2d->a2d_prof_stat.init_state)
        {
            ESP_LOGI(BTSUBSYSTEMS_TAG, "ESP A2D state : DEINIT");
        }

        break;

    default:
        ESP_LOGW(BTSUBSYSTEMS_TAG, "Unhandled A2D event %d", event);
        break;
    }
}

void BTSubsystems::gapCallback(uint16_t event, void *param)
{
    if (!btInitialized_)
    {
        ESP_LOGE(BTSUBSYSTEMS_TAG, "BT NOT INITIALIZED, NO WORK");
        return;
    }

    esp_bt_gap_cb_event_t evt = esp_bt_gap_cb_event_t(event);
    esp_bt_gap_cb_param_t *prm = (esp_bt_gap_cb_param_t *)(param);

    switch (evt)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (prm->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGD(BTSUBSYSTEMS_TAG, "GAP Authenticated client: %s", prm->auth_cmpl.device_name);
            esp_log_buffer_hex(BTSUBSYSTEMS_TAG, prm->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGD(BTSUBSYSTEMS_TAG, "GAP Pairing code: %d", prm->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(prm->cfm_req.bda, true);
        esp_log_buffer_hex(BTSUBSYSTEMS_TAG, prm->cfm_req.bda, ESP_BD_ADDR_LEN);
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGD(BTSUBSYSTEMS_TAG, "Passkey notification: %d", prm->key_notif.passkey);
        esp_log_buffer_hex(BTSUBSYSTEMS_TAG, prm->key_notif.bda, ESP_BD_ADDR_LEN);
        break;

    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGD(BTSUBSYSTEMS_TAG, "Passkey request, enter key");
        esp_log_buffer_hex(BTSUBSYSTEMS_TAG, prm->key_req.bda, ESP_BD_ADDR_LEN);
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGD(BTSUBSYSTEMS_TAG, "GAP Mode change [%d]", prm->mode_chg.mode);
        esp_log_buffer_hex(BTSUBSYSTEMS_TAG, prm->mode_chg.bda, ESP_BD_ADDR_LEN);
        break;

    default:
        ESP_LOGW(BTSUBSYSTEMS_TAG, "Unhandled BT Gap CB event [%d]", evt);
        break;
    }
}

void BTSubsystems::handleA2DConnectionStates(esp_a2d_cb_param_t::a2d_conn_stat_param StateParam)
{
    uint8_t *bda = StateParam.remote_bda;
    ESP_LOGD(BTSUBSYSTEMS_TAG, "* A2DP CONN event: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
             BTSubsystems::A2SStateStrings[StateParam.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    if (ESP_A2D_CONNECTION_STATE_DISCONNECTED == StateParam.state)
    {
        ESP_LOGD(BTSUBSYSTEMS_TAG, "Disconnection");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        endI2SProcessing();
        uninstallI2SDriver();
    }
    else if (ESP_A2D_CONNECTION_STATE_CONNECTED == StateParam.state)
    {
        ESP_LOGD(BTSUBSYSTEMS_TAG, "Connected");
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        beginI2SProcessing();
    }
    else if (ESP_A2D_CONNECTION_STATE_CONNECTING == StateParam.state)
    {
        ESP_LOGD(BTSUBSYSTEMS_TAG, "New connection");
        installI2SDriver();
    }
}

void BTSubsystems::beginI2SProcessing()
{
    if ((hI2SRingBuffer_ = xRingbufferCreate(8 * 1024, RINGBUF_TYPE_BYTEBUF)) == NULL)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    taskI2SFeed_.begin("I2SFeedTask", &hI2SRingBuffer_);
}

void BTSubsystems::endI2SProcessing()
{
    taskI2SFeed_.end();
    vRingbufferDelete(hI2SRingBuffer_);
    hI2SRingBuffer_ = NULL;
}

void BTSubsystems::installI2SDriver()
{
    i2s_config_t i2sconfig = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 6,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0};

    ESP_LOGD(BTSUBSYSTEMS_TAG, "Installing I2S driver");
    i2s_driver_install(i2s_port_t(0), &i2sconfig, 0, NULL);

    i2s_pin_config_t pinconfig = {
        .bck_io_num = GPIO_NUM_27,
        .ws_io_num = GPIO_NUM_26,
        .data_out_num = GPIO_NUM_25,
        .data_in_num = GPIO_NUM_35};
    
    ESP_LOGD(BTSUBSYSTEMS_TAG, "Setting I2S pin config");
    i2s_set_pin(i2s_port_t(0), &pinconfig);
}

void BTSubsystems::uninstallI2SDriver()
{
    ESP_LOGD(BTSUBSYSTEMS_TAG, "Uninstalling I2S driver");
    i2s_driver_uninstall(i2s_port_t(0));
}
