/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "deep_sleep_example.h"
#include "sntp.h"
#include "lvgl_watch.h"
#include "gatt_client_coospo_h808s.h"

#include "circular_buffer.h"
#include "max30100.h"
#include "max30100_pulseoximeter.h"

#define I2C_MASTER_SCL_IO           22 //CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21 //CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0  //I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000 //CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MAX30100_SENSOR_ADDR       	0x57        /*!< Address of the MAX30100 sensor */

#define REPORTING_PERIOD_MS     	1000


#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

static const char *TAG = "watch";
extern uint8_t pulse_value;

#define I2C_HOST  0

//////////////////////////////////////////////////////////////////////////
// Please update the following configuration according to your LCD spec //
//////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ	(400 * 1000)
#define EXAMPLE_PIN_NUM_SDA        	21 
#define EXAMPLE_PIN_NUM_SCL        	22 
#define EXAMPLE_PIN_NUM_RST         -1
#define EXAMPLE_I2C_HW_ADDR         0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES			128
#define EXAMPLE_LCD_V_RES			64
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES			64
#define EXAMPLE_LCD_V_RES			128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS		8
#define EXAMPLE_LCD_PARAM_BITS		8

#if SOC_RTC_FAST_MEM_SUPPORTED
static RTC_DATA_ATTR struct timeval sleep_enter_time;
#else
static struct timeval sleep_enter_time;
#endif

static int show_timer = 0;

static void 						periodic_timer_callback(void* arg);
static void 						deep_sleep_task(void *args);
static void 						__attribute__((unused)) example_deep_sleep_register_rtc_timer_wakeup(void);
static void 						i2c_init();
static void 						i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
//static esp_lcd_panel_io_handle_t 	install_panel_io(void);
static esp_lcd_panel_io_handle_t 	install_panel_io(i2c_master_bus_handle_t *bus_handle);
static esp_lcd_panel_handle_t 		install_ssd1306_panel_driver(esp_lcd_panel_io_handle_t io_handle);
static bool 						notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
static lv_disp_t* 					lvgl_init(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t panel_handle);


void app_main(void)
{
	//i2c_init();
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    printf("I2C initialized successfully");

	esp_lcd_panel_io_handle_t io_handle = install_panel_io(&bus_handle);

    esp_lcd_panel_handle_t panel_handle = install_ssd1306_panel_driver(io_handle);

	lv_disp_t *disp = lvgl_init(io_handle, panel_handle);

    // Rotate the screen 
    lv_disp_set_rotation(disp, LV_DISP_ROT_180);

	// Create a periodic timer which will run every second and update the screen
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "scrn_upd_periodic"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

	char timebuf[128] = {0};

	char connect_status_str[128] = {0};
	sprintf(connect_status_str, "%s %s", LV_SYMBOL_WIFI, CONFIG_EXAMPLE_WIFI_SSID);
	set_lvgl_ui(disp, connect_status_str);
	sntp_app_main(timebuf);

	// Connect to heart rate monitor
	coospo_connect();
	
    /* Start the timers */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000));
	
	set_lvgl_ui(disp, timebuf);

    /* Enable wakeup from deep sleep by rtc timer */
    //example_deep_sleep_register_rtc_timer_wakeup();

#if CONFIG_EXAMPLE_GPIO_WAKEUP
    /* Enable wakeup from deep sleep by gpio */
    example_deep_sleep_register_gpio_wakeup();
#endif

#if CONFIG_EXAMPLE_EXT0_WAKEUP
    /* Enable wakeup from deep sleep by ext0 */
    example_deep_sleep_register_ext0_wakeup();
#endif

#if CONFIG_EXAMPLE_EXT1_WAKEUP
    /* Enable wakeup from deep sleep by ext1 */
    example_deep_sleep_register_ext1_wakeup();
#endif

#if CONFIG_EXAMPLE_TOUCH_WAKEUP
    /* Enable wakeup from deep sleep by touch */
    example_deep_sleep_register_touch_wakeup();
#endif

    xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, (void *) &panel_handle, 6, NULL);
}

static void deep_sleep_task(void *args)
{
	printf("ENTER DEEP SLEEP TASK\n");
	char timebuf[128] = {0};
    esp_lcd_panel_handle_t *panel_handle = (esp_lcd_panel_handle_t *) args;

	//sntp_app_main(timebuf);
	//update_time_label(timebuf);

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(*panel_handle, true));
	printf("timebuf: %s\n", timebuf);
    /**
     * Prefer to use RTC mem instead of NVS to save the deep sleep enter time, unless the chip
     * does not support RTC mem(such as esp32c2). Because the time overhead of NVS will cause
     * the recorded deep sleep enter time to be not very accurate.
     */
#if !SOC_RTC_FAST_MEM_SUPPORTED
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Open NVS done\n");
    }

    // Get deep sleep enter time
    nvs_get_i32(nvs_handle, "slp_enter_sec", (int32_t *)&sleep_enter_time.tv_sec);
    nvs_get_i32(nvs_handle, "slp_enter_usec", (int32_t *)&sleep_enter_time.tv_usec);
#endif

    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

	printf("Check wakeup cause\n");

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }

#if CONFIG_EXAMPLE_GPIO_WAKEUP
        case ESP_SLEEP_WAKEUP_GPIO: {
            uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
#endif //CONFIG_EXAMPLE_GPIO_WAKEUP

#if CONFIG_EXAMPLE_EXT0_WAKEUP
        case ESP_SLEEP_WAKEUP_EXT0: {
            printf("Wake up from ext0\n");
            break;
        }
#endif // CONFIG_EXAMPLE_EXT0_WAKEUP

#ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from EXT1: pin %d\n", pin);
            } else {
                printf("Wake up from EXT1\n");
            }
            break;
        }
#endif // CONFIG_EXAMPLE_EXT1_WAKEUP

#ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
        case ESP_SLEEP_WAKEUP_TOUCHPAD: {
            printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
            break;
        }
#endif // CONFIG_EXAMPLE_TOUCH_WAKEUP

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

#if CONFIG_IDF_TARGET_ESP32
    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
#endif

	while(1)
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
		;
	}

	// Wait 10 seconds before entring deep sleep
	// show_timer is increased by the periodic 1s timer
	while (show_timer < 10) {
	}
	show_timer = 0;

    printf("Entering deep sleep\n");

    // get deep sleep enter time
    gettimeofday(&sleep_enter_time, NULL);

#if !SOC_RTC_FAST_MEM_SUPPORTED
    // record deep sleep enter time via nvs
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_sec", sleep_enter_time.tv_sec));
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_usec", sleep_enter_time.tv_usec));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
#endif

	//TODO: Some bug is here. If uncomment the app restarts
    //ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(*panel_handle, false));
	
	//TODO: see esp_lcd_panel_disp_sleep() that can reduce the power
	//consumption of the LCD screen by entering the sleep mode. The internal
	//frame buffer is still retained.
	
    //enter deep sleep
    esp_deep_sleep_start();
}

static void __attribute__((unused)) example_deep_sleep_register_rtc_timer_wakeup(void) 
{
    const int wakeup_time_sec = 20;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
}

static void i2c_init()
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = (gpio_num_t) I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t) I2C_MASTER_SCL_IO,
        .flags.enable_internal_pullup = true
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30100_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static esp_lcd_panel_io_handle_t install_panel_io(i2c_master_bus_handle_t *bus_handle)
{
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
        .dc_bit_offset = 0,                     // According to SH1107 datasheet
        .flags =
        {
            .disable_control_phase = 1,
        }
#endif
    };
    //ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(*bus_handle, &io_config, &io_handle));
	return io_handle;
}

static esp_lcd_panel_handle_t install_ssd1306_panel_driver(esp_lcd_panel_io_handle_t io_handle)
{
    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
//#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
//    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
//#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
//    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
//#endif
//
//    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
//    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
//    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
//
//#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
//    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
//#endif
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif
	return panel_handle;
}

/* The LVGL port component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called
lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done
callback from esp_lcd IO config structure. In IDF 5.1 and higher, it is solved inside LVGL port component. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

static lv_disp_t* lvgl_init(esp_lcd_panel_io_handle_t io_handle, 
					 esp_lcd_panel_handle_t panel_handle)
{
    ESP_LOGI(TAG, "Initialize LVGL");

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();

    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };

    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

	return disp;
}

static void periodic_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
	char timebuf[128] = {0};
	char pulsebuf[128] = {0};
	char pulsebuf_2[128] = {0};
#if 1
	sntp_app_main(timebuf);
#else
	//if (show_timer < 8) {
	//	sntp_app_main(timebuf);
	//} else {
	//	timebuf[0] = '\0';
	//}
#endif
	update_time_label(timebuf);

	sprintf(pulsebuf, "%i %i", pulse_value, pulse_value);
	update_pulse_label(pulsebuf);

	//sprintf(pulsebuf_2, "%i", pulse_value);
	//update_pulse_label_2(pulsebuf_2);

	show_timer++;
}

