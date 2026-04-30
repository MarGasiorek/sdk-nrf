#include <nfc_t4t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>
#include <nfc/tnep/tag.h>
#include <nfc/ndef/msg_parser.h>
#include <nfc/ndef/record_parser.h>
#include <nfc/t4t/ndef_file.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_power.h>
#if !NRF_POWER_HAS_RESETREAS
#include <hal/nrf_reset.h>
#endif
#include <zephyr/sys/poweroff.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(NFC_test);

K_SEM_DEFINE(l_continueStartSemaphore, 0, 1);

#define MAX_REC_COUNT (2)
#define MAX_SVC_COUNT (1)
#define NDEF_BUF_SIZE (128)

#define ON_OFF_RECORD_INDEX (0)
#define TIME_RECORD_INDEX   (1)

#define APP_NFC_SYSTEM_OFF_DELAY_S                 3  ///< Delay before shutting off when device is turned off from NFC
#define APP_MAIN_SYSTEM_OFF_FROM_RESTART_DELAY_S    10 

static struct k_poll_event events[NFC_TNEP_EVENTS_NUMBER];

static uint8_t ndef_msg_buf[NDEF_BUF_SIZE] = {0};
static uint8_t tnep_swap_buf[NDEF_BUF_SIZE] = {0};
static uint8_t test_buf[NDEF_BUF_SIZE] = {0};
static uint8_t l_aOnOffRecordText[100] = "OFF";
static uint8_t l_aTimeRecordText[100] = "0";


static size_t nfc_data_len = 0;
static const uint8_t en_code[] = {'e', 'n'};
static const uint8_t aSvcUri[] = "LXS";
static struct k_work work_test;

NFC_TNEP_TAG_SERVICE_DEF(nfcService, aSvcUri, sizeof(aSvcUri), NFC_TNEP_COMM_MODE_SINGLE_RESPONSE, 20, 10, 0, NULL, NULL, NULL, NULL);


static struct k_work_delayable work_systemOff;

static void                   printResetReason(void);
static void                   cbWorkerSystemOff(struct k_work* pWork);
static void cbNfc(void *pContext, nfc_t4t_event_t event, const uint8_t *pBuffer, size_t bufferLength, uint32_t flags);
static int  tnep_initial_msg_encode(struct nfc_ndef_msg_desc *msg);;
static void cbWorkerHandlePayload();
static int  getUsefulPayloadFromRecord(const struct nfc_ndef_record_desc *pRecord, uint8_t *pBuffer);

/// @brief Turn on blue LED
#define APP_led_blueOn()    dk_set_led_on(DK_LED1);
/// @brief Turn off blue LED
#define APP_led_blueOff()     dk_set_led_off(DK_LED1);
/// @brief Turn on red LED
#define APP_led_redOn()     dk_set_led_on(DK_LED2);
/// @brief Turn off red LED
#define APP_led_redOff()     dk_set_led_off(DK_LED2);


int APP_nfc_init()
{
    int err = 0;
    size_t len = sizeof(ndef_msg_buf);

    k_work_init(&work_test, cbWorkerHandlePayload);

    err = nfc_tnep_tag_tx_msg_buffer_register(ndef_msg_buf, tnep_swap_buf, len);
    if (0 != err)
    {
        LOG_ERR("Cannot register tnep buffer, err: %d\n", err);
        return 0;
    }

    err = nfc_tnep_tag_init(events, ARRAY_SIZE(events), nfc_t4t_ndef_rwpayload_set);
    if (0 != err)
    {
        LOG_ERR("Cannot initialize TNEP protocol, err: %d\n", err);
        return 0;
    }



    err = nfc_t4t_setup(cbNfc, NULL);

    if(0 == err)
    {
        /* Set created message as the NFC payload */
        err = nfc_tnep_tag_initial_msg_create(MAX_REC_COUNT + MAX_SVC_COUNT, tnep_initial_msg_encode);
        if (0 == err) {
            err = nfc_t4t_emulation_start();
            if (0 != err)
            {
                LOG_ERR("NFC T4T emulation start failed with %d", err);
            }
        }
        else
        {
            LOG_ERR("Cannot set payload! %d", err);
        }
    }
    else
    {
        LOG_ERR("NFC T4T SETUP failed with %d", err);
    }

    return err;
}

static void cbNfc(void *pContext, nfc_t4t_event_t event, const uint8_t *pBuffer, size_t bufferLength, uint32_t flags)
{
    switch (event) {
        case NFC_T4T_EVENT_NDEF_UPDATED:
            LOG_INF("NFC NDEF_UPDT");
            if(0 < bufferLength)
            {
                nfc_data_len = bufferLength;
                nfc_tnep_tag_rx_msg_indicate(nfc_t4t_ndef_file_msg_get(pBuffer), bufferLength);
                memcpy(test_buf, nfc_t4t_ndef_file_msg_get(pBuffer), bufferLength);
                k_work_submit(&work_test);
            }
            break;
        case NFC_T4T_EVENT_FIELD_ON:
            LOG_INF("NFC field on");
            break;
        case NFC_T4T_EVENT_FIELD_OFF:
            LOG_INF("NFC field off");
            break;
        default:
            break;
    }
}

static int tnep_initial_msg_encode(struct nfc_ndef_msg_desc *msg)
{
    
    // Use Text Record to test with generic app
    NFC_NDEF_TEXT_RECORD_DESC_DEF(onOffRecord, UTF_8, en_code,
                      sizeof(en_code), l_aOnOffRecordText,
                      strlen(l_aOnOffRecordText));

    NFC_NDEF_TEXT_RECORD_DESC_DEF(timeRecord, UTF_8, en_code,
                      sizeof(en_code), l_aTimeRecordText,
                      strlen(l_aTimeRecordText));

    struct nfc_ndef_record_desc records[] = { NFC_NDEF_TEXT_RECORD_DESC(onOffRecord), NFC_NDEF_TEXT_RECORD_DESC(timeRecord) };
    

    return nfc_tnep_initial_msg_encode(msg,
                       records,
                       MAX_REC_COUNT);

}

static void cbWorkerHandlePayload()
{
    uint32_t data_len = nfc_data_len;
    uint8_t res_buf[NDEF_BUF_SIZE * MAX_REC_COUNT];
    uint32_t res_buf_size = sizeof(res_buf);
    
    int err = nfc_ndef_msg_parse(res_buf, &res_buf_size, test_buf, &data_len);
    if (0 == err)
    {
        struct nfc_ndef_msg_desc* msg_desc = (struct nfc_ndef_msg_desc*)(res_buf);
        nfc_ndef_msg_printout(msg_desc); //DEBUG
        if(0 != getUsefulPayloadFromRecord(msg_desc->record[ON_OFF_RECORD_INDEX], l_aOnOffRecordText))
        {
            if (0 == strcmp(l_aOnOffRecordText, "OFF"))
            {
                LOG_INF("NFC REC OFF");
                APP_main_scheduleSystemOff(APP_NFC_SYSTEM_OFF_DELAY_S);
            }
            else if (0 == strcmp(l_aOnOffRecordText, "ON"))
            {
                LOG_INF("NFC REC ON");
                APP_main_cancelSystemOff();
            }
        }
        if(0 != getUsefulPayloadFromRecord(msg_desc->record[TIME_RECORD_INDEX], l_aTimeRecordText))
        {
            LOG_INF("NFC REC TIME");
            //APP_time_setReferenceTime(strtoul(l_aTimeRecordText, NULL, 10));
        }
        
    }
    else
    {
        LOG_ERR("nfc ndef msg parse failed with %d", err);
    }
}

static int getUsefulPayloadFromRecord(const struct nfc_ndef_record_desc *pRecord, uint8_t *pBuffer)
{
    const struct nfc_ndef_bin_payload_desc* pBinPayloadDescriptor = pRecord->payload_descriptor;
    const uint8_t* pUsefulRecBuf = pBinPayloadDescriptor->payload + sizeof(en_code) + 1;
    const uint8_t usefulRecSize = pBinPayloadDescriptor->payload_length - sizeof(en_code) - 1;
    const uint8_t currentSize   = strlen(pBuffer);
    int comparisonValue = 0xFF;

    if (usefulRecSize == currentSize){ comparisonValue = memcmp(pBuffer, pUsefulRecBuf, currentSize); }

    LOG_HEXDUMP_INF(pBuffer      , currentSize  , "existing record");
    LOG_HEXDUMP_INF(pUsefulRecBuf, usefulRecSize, "incoming record");

    memcpy(pBuffer, pUsefulRecBuf, usefulRecSize);
    pBuffer[usefulRecSize] = 0x00;
    LOG_HEXDUMP_INF(pBuffer, strlen(pBuffer) , "new record");

    return comparisonValue;
}

void APP_main_scheduleSystemOff(int delay_s)
{
	k_work_reschedule(&work_systemOff, K_SECONDS(delay_s));
}

void APP_main_cancelSystemOff()
{
    LOG_INF("SystemOff canceled");
	k_work_cancel_delayable(&work_systemOff); 
    k_sem_give(&l_continueStartSemaphore);
}

static void cbWorkerSystemOff(struct k_work* pWork)
{
	LOG_INF("Powering off system...");
	dk_set_led_on(DK_LED3);
	sys_poweroff();
}

static void printResetReason(void)
{
	uint32_t reas;

#if NRF_POWER_HAS_RESETREAS

	reas = nrf_power_resetreas_get(NRF_POWER);
	nrf_power_resetreas_clear(NRF_POWER, reas);
	if (reas & NRF_POWER_RESETREAS_NFC_MASK) {
		LOG_INF("Wake up by NFC field detect\n");
	} else if (reas & NRF_POWER_RESETREAS_RESETPIN_MASK) {
		LOG_INF("Reset by pin-reset\n");
	} else if (reas & NRF_POWER_RESETREAS_SREQ_MASK) {
		LOG_INF("Reset by soft-reset\n");
	} else if (reas) {
		LOG_INF("Reset by a different source (0x%08X)\n", reas);
	} else {
		LOG_INF("Power-on-reset\n");
	}

#else

	reas = nrf_reset_resetreas_get(NRF_RESET);
	nrf_reset_resetreas_clear(NRF_RESET, reas);
	if (reas & NRF_RESET_RESETREAS_NFC_MASK) {
		printk("Wake up by NFC field detect\n");
	} else if (reas & NRF_RESET_RESETREAS_RESETPIN_MASK) {
		printk("Reset by pin-reset\n");
	} else if (reas & NRF_RESET_RESETREAS_SREQ_MASK) {
		printk("Reset by soft-reset\n");
	} else if (reas) {
		printk("Reset by a different source (0x%08X)\n", reas);
	} else {
		printk("Power-on-reset\n");
	}

#endif
}

#include <debug/ppi_trace.h>
#include <nrfx_nfct.h>
#include <haly/nrfy_gpio.h>

static void ppi_trace_pin_setup(uint32_t pin, uint32_t evt)
{
   void *handle;

   handle = ppi_trace_config(pin, evt);
   __ASSERT(handle != NULL,
       "Failed to initialize trace pin, no PPI or GPIOTE resources?");

   ppi_trace_enable(handle);
}

int main(void)
{
    int status;

	ppi_trace_pin_setup(NRF_GPIO_PIN_MAP(0, 27), nrf_nfct_event_address_get(NRF_NFCT, NRF_NFCT_EVENT_FIELDDETECTED));
    ppi_trace_pin_setup(NRF_GPIO_PIN_MAP(0, 26), nrf_nfct_event_address_get(NRF_NFCT, NRF_NFCT_EVENT_FIELDLOST));

    printResetReason();
    
    k_work_init_delayable(&work_systemOff, cbWorkerSystemOff);

    APP_main_scheduleSystemOff(APP_MAIN_SYSTEM_OFF_FROM_RESTART_DELAY_S);

    status = APP_nfc_init();
    if (0 != status)
    {
        LOG_ERR("NFC initialization failed");
        return status;
    }
    LOG_DBG("NFC initialized successfuly");

	status = dk_leds_init();
	if (status) {
		printk("led init error %d", status);
		return status;
	}
	dk_set_led_off(DK_LED3);
    LOG_DBG("Leds initialized successfuly");

    APP_led_blueOn();
    k_sleep(K_MSEC(500));
    APP_led_blueOff();

    k_sem_take(&l_continueStartSemaphore, K_FOREVER); // Wait until the correct values have been written in nfc records before resuming the start sequence

    LOG_INF("START APP");
    
    // LEDS
    APP_led_blueOn();
    k_sleep(K_MSEC(1000));
    APP_led_blueOff();
    k_sleep(K_MSEC(800));
    for(int i = 0; i < 3; i++)
    {
        k_sleep(K_MSEC(200));
        APP_led_blueOn();
        k_sleep(K_MSEC(200));
        APP_led_blueOff();
    }

    for (;;) {
        k_sleep(K_FOREVER);
    }
    return status;
}
