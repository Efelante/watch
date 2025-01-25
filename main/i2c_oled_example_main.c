/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <math.h>

// Time headers
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "lvgl.h"

// Wi-fi
#include "nvs_flash.h"
#include "nvs.h"
#include "sntp.h"

// Bluetooth heart rate monitor
#include "gatt_client_coospo_h808s.h"
extern uint8_t bt_pulse_value;


#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

#define I2C_BUS_PORT  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           21
#define EXAMPLE_PIN_NUM_SCL           22 
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES              64
#define EXAMPLE_LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    5
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2
#define EXAMPLE_LVGL_PALETTE_SIZE      8

// User lvgl headers
#include "lvgl_watch.h"

// User max30100 headers
#include "circular_buffer.h"
#include "max30100.h"
#include "max30100_pulseoximeter.h"
#define MAX30100_SENSOR_ADDR       	0x57        /*!< Address of the MAX30100 sensor */
#define I2C_MASTER_FREQ_HZ          EXAMPLE_LCD_PIXEL_CLOCK_HZ /*!< I2C master clock frequency */
#define REPORTING_PERIOD_MS     	1000
static int show_timer = 0;
static float max30100_hr = 0;
static int max30100_spo2 = 0;

// User mpu6050 headers
#include "mpu6050.h"

// User sd-card headers
#include "sd_card.h"
#include <string.h>
sdmmc_card_t card;
const char mount_point[] = MOUNT_POINT;
sdmmc_host_t host;

// User buttons
#include "buttons.h"

static const char *TAG = "example";

//---------------------MPU6050-----------------------------//
struct mpu6050 mpu6050;
static int steps = 0;
static int step = 0;

//--------------End of MPU6050-----------------------------//

//---------------------MAX30100-----------------------------//
// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
    ESP_LOGI(TAG, "Beat!");
}

void setup(struct pulseOximeter *pulseOximeter)
{
    ESP_LOGI(TAG, "Initializing pulse oximeter..");

    // Initialize the PulseOximeter instance
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip
    if (!pulseOximeter_begin(pulseOximeter, PULSEOXIMETER_DEBUGGINGMODE_NONE)) {
		ESP_LOGI(TAG, "FAILED");
        for(;;);
    } else {
		ESP_LOGI(TAG, "SUCCESS");
    }

    // The default current for the IR LED is 50mA and it could be changed
    //   by uncommenting the following line. Check max30100_registers.h for all the
    //   available options.
    //pulseOximeter_setIRLedCurrent(pulseOximeter, MAX30100_LED_CURR_7_6MA);

    // Register a callback for the beat detection
    pulseOximeter_setOnBeatDetectedCallback(pulseOximeter, &onBeatDetected);
}

static unsigned long millis()
{
	return (esp_timer_get_time() / 1000);
}

struct max30100_data buffer[RINGBUFFER_SIZE];
struct max30100 max30100;
// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
struct pulseOximeter pulseOximeter;

static void pulse_update_task(void *arg)
{
    ESP_LOGI(TAG, "Starting Pulse Update task");
    while (1) {
		pulseOximeter_update(&pulseOximeter);
        usleep(1000 * 5);

		mpu6050_getAccel(&mpu6050);
		//ESP_LOGI(TAG, "ACCEL_XOUT = %f, ACCEL_YOUT = %f, ACCEL_ZOUT = %f", 
		//		mpu6050.accel_x_out, 
		//		mpu6050.accel_y_out, 
		//		mpu6050.accel_z_out);
		float accel = sqrt(
				mpu6050.accel_x_out * mpu6050.accel_x_out + 
				mpu6050.accel_y_out * mpu6050.accel_y_out + 
				mpu6050.accel_z_out * mpu6050.accel_z_out 
				);
		if (!step) {
			if (accel > 1.1)
			{
				step = 1;
				steps++;
			}
		} else {
			if (accel < 1)
			{
				step = 0;
			}
		}
		ESP_LOGI(TAG, "Accel is %f Steps is %i", accel, steps);

#if 0
		// mpu6050
		mpu6050_getAccel(&mpu6050);
		//ESP_LOGI(TAG, "ACCEL_XOUT = %u, ACCEL_YOUT = %u, ACCEL_ZOUT = %u", 
		//		(uint16_t) (mpu6050.accel_x_out + 0.5), 
		//		(uint16_t) (mpu6050.accel_y_out + 0.5), 
		//		(uint16_t) (mpu6050.accel_z_out + 0.5));
		float accel = sqrt(
				mpu6050.accel_x_out * mpu6050.accel_x_out + 
				mpu6050.accel_y_out * mpu6050.accel_y_out + 
				mpu6050.accel_z_out * mpu6050.accel_z_out 
				);
		if (accel > PEDOMETER_ACCEL_THRESHOLD)
		{
			steps++;
		}
		//ESP_LOGI(TAG, "ACCEL is %i", (int) accel);

		static int stop = 0;
		if (!stop) {
			if (show_timer < 60) {
				// write to sd-card
				char data[EXAMPLE_MAX_CHAR_SIZE];
				const char *file_nihao = MOUNT_POINT"/nihao.txt";
				memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
				snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%i ", (int) accel);
				esp_err_t ret = s_example_write_file(file_nihao, data);
				if (ret != ESP_OK) {
					return;
				}
			} else {
				sd_card_deinitialize(mount_point, &card, &host);
				stop = 1;
			}
		}
#endif
    }
}

//--------------End of MAX30100-----------------------------//



// To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
static uint8_t oled_buffer[EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8];
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

extern void example_lvgl_demo_ui(lv_disp_t *disp);

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette. Skip the palette here
    // More information about the monochrome, please refer to https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
    px_map += EXAMPLE_LVGL_PALETTE_SIZE;

    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(disp);
    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            /* The order of bits is MSB first
                        MSB           LSB
               bits      7 6 5 4 3 2 1 0
               pixels    0 1 2 3 4 5 6 7
                        Left         Right
            */
            bool chroma_color = (px_map[(hor_res >> 3) * y  + (x >> 3)] & 1 << (7 - x % 8));

            /* Write to the buffer as required for the display.
            * It writes only 1-bit for monochrome displays mapped vertically.*/
            uint8_t *buf = oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color) {
                (*buf) &= ~(1 << (y % 8));
            } else {
                (*buf) |= (1 << (y % 8));
            }
        }
    }
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, oled_buffer);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        usleep(1000 * time_till_next_ms);
    }
}


static void lcd_update_task(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
	char timebuf[128] = {0};
	char pulsebuf[128] = {0};
	char spo2buf[128] = {0};
	char stepsbuf[128] = {0};
	char pulsebuf_2[128] = {0};

	max30100_hr = pulseOximeter_getHeartRate(&pulseOximeter);
	max30100_spo2 = pulseOximeter_getSpO2(&pulseOximeter);
	ESP_LOGI(TAG, "HR = %f bpm SPO2 = %i", max30100_hr, max30100_spo2);

	sntp_app_main(timebuf);
	//sprintf(timebuf,  "Time : %i", show_timer);
	sprintf(pulsebuf, "HR: %3i | %3u", (int) (max30100_hr + 0.5), bt_pulse_value);
	sprintf(spo2buf,  "SPO2 : %i", max30100_spo2);
	sprintf(stepsbuf,  "St : %i", steps);
	_lock_acquire(&lvgl_api_lock);
	update_time_label(timebuf);
	update_pulse_label(pulsebuf);
	update_spo2_label(spo2buf);
	update_steps_label(stepsbuf);
	_lock_release(&lvgl_api_lock);

	//mpu6050_getAccel(&mpu6050);
	//ESP_LOGI(TAG, "ACCEL_XOUT = %f, ACCEL_YOUT = %f, ACCEL_ZOUT = %f", 
	//		mpu6050.accel_x_out, 
	//		mpu6050.accel_y_out, 
	//		mpu6050.accel_z_out);
	//float accel = sqrt(
	//		mpu6050.accel_x_out * mpu6050.accel_x_out + 
	//		mpu6050.accel_y_out * mpu6050.accel_y_out + 
	//		mpu6050.accel_z_out * mpu6050.accel_z_out 
	//		);
	//ESP_LOGI(TAG, "ACCEL is %f", accel);
//
//
//#if 1
//	sntp_app_main(timebuf);
//#else
//	//if (show_timer < 8) {
//	//	sntp_app_main(timebuf);
//	//} else {
//	//	timebuf[0] = '\0';
//	//}
//#endif
//	update_time_label(timebuf);
//
//
//	//sprintf(pulsebuf_2, "%i", pulse_value);
//	//update_pulse_label_2(pulsebuf_2);

	//ESP_LOGI(TAG, "BT HRM: %u", bt_pulse_value);

	show_timer++;
	ESP_LOGI(TAG, "Show timer is %i", show_timer);
}


void app_main(void)
{
	// Buttons init
	buttons_init();	
	
	// SD-card test
	sd_card_initialize(&card, &host);
    ESP_LOGI(TAG, "%s %s!\n", "Hello", card.cid.name);
	sd_card_test(&card, &host);	
	//sd_card_deinitialize(mount_point, &card, &host);
	
	// Wi-Fi
	char timebuf[128] = {0};
	sntp_app_main(timebuf);

	// Connect to bluetooth Heart Rate Monitor
	coospo_connect();

	// Connect I2C devices:
	// * lcd: +
	// * pulseoximeter (max30100): +
	// * accelerometer, ...: -

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

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
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
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

    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();
    // create a lvgl display
    lv_display_t *display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    // associate the i2c panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // create draw buffer
    void *buf = NULL;
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers");
    // LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette.
    size_t draw_buffer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8 + EXAMPLE_LVGL_PALETTE_SIZE;
    buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(buf);

    // LVGL9 suooprt new monochromatic format.
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display);

    ESP_LOGI(TAG, "Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    example_lvgl_demo_ui(display);
    _lock_release(&lvgl_api_lock);

	// Screen update task

	// Create a periodic timer which will run every second and update the screen
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &lcd_update_task,
            /* name is optional, but may help identify the timer when debugging */
            .name = "scrn_upd_periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000));

	// MPU6050
    i2c_device_config_t mpu6050_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t mpu6050_dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &mpu6050_dev_config, &mpu6050_dev_handle));

	mpu6050_init(&mpu6050, mpu6050_dev_handle);

	uint8_t mpu6050_pwr_mgmt_1_reg = mpu6050_readRegister(&mpu6050, MPU6050_REG_PWR_MGMT_1);
    ESP_LOGI(TAG, "MPU6050 PWR_MGMT_1 reg is %x", mpu6050_pwr_mgmt_1_reg);
	mpu6050_writeRegister(&mpu6050, MPU6050_REG_PWR_MGMT_1, 0x00);	// Disable Sleep
	mpu6050_pwr_mgmt_1_reg = mpu6050_readRegister(&mpu6050, MPU6050_REG_PWR_MGMT_1);
    ESP_LOGI(TAG, "MPU6050 PWR_MGMT_1 reg is %x", mpu6050_pwr_mgmt_1_reg);

	uint8_t mpu6050_who_am_i_reg = mpu6050_readRegister(&mpu6050, MPU6050_REG_WHO_AM_I);
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I reg is %x", mpu6050_who_am_i_reg);

	// Set accel full range to 8g
	mpu6050_writeRegister(&mpu6050, MPU6050_REG_ACCEL_CONFIG, 0x10);

	// Set filter bandwidth to 21 Hz
	mpu6050_writeRegister(&mpu6050, MPU6050_REG_CONFIG, 0x04);

	// MAX30100
	//
	// Add max30100 i2c device to the bus
    i2c_device_config_t max30100_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30100_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t max30100_dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &max30100_dev_config, &max30100_dev_handle));

	cbuf_handle_t cbuf_handle = circular_buf_init(buffer, RINGBUFFER_SIZE);

	max30100_init(&max30100, max30100_dev_handle, cbuf_handle);
	pulseOximeter_init(&pulseOximeter, &max30100);
	setup(&pulseOximeter);

    ESP_LOGI(TAG, "Create pulse update task");
    xTaskCreate(pulse_update_task, "PULSE UPDATE", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
}
