| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# I2C OLED example

[esp_lcd](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html) supports I2C interfaced OLED LCD, whose color depth is usually 1bpp.

This example shows how to make use of the SSD1306 panel driver from `esp_lcd` component to facilitate the porting of LVGL library. In the end, example will display a scrolling text on the OLED screen. For more information about porting the LVGL library, you can also refer to another [lvgl porting example](../i80_controller/README.md).

## How to use the example

### Hardware Required

* An ESP development board
* An SSD1306 OLED LCD, with I2C interface
* An USB cable for power supply and programming

### Hardware Connection

The connection between ESP Board and the LCD is as follows:

```
      ESP Board                       OLED LCD (I2C)
+------------------+              +-------------------+
|               GND+--------------+GND                |
|                  |              |                   |
|               3V3+--------------+VCC                |
|                  |              |                   |
|               SDA+--------------+SDA                |
|                  |              |                   |
|               SCL+--------------+SCL                |
+------------------+              +-------------------+
```

The GPIO number used by this example can be changed in [lvgl_example_main.c](main/i2c_oled_example_main.c). Please pay attention to the I2C hardware device address as well, you should refer to your module's spec and schematic to determine that address.

### Build and Flash

Run `idf.py -p PORT build flash monitor` to build, flash and monitor the project. A scrolling text will show up on the LCD as expected.

The first time you run `idf.py` for the example will cost extra time as the build system needs to address the component dependencies and downloads the missing components from registry into `managed_components` folder.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

### Example Output

```bash
...
I (0) cpu_start: Starting scheduler on APP CPU.
I (345) example: Initialize I2C bus
I (345) example: Install panel IO
I (345) example: Install SSD1306 panel driver
I (455) example: Initialize LVGL library
I (455) example: Register display driver to LVGL
I (455) example: Install LVGL tick timer
I (455) example: Display LVGL Scroll Text
...
```

## Troubleshooting

For any technical queries, please open an [issue] (https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.

## WIFI

About the `example_connect()` Function

Protocols examples use a simple helper function, `example_connect()`, to establish Wi-Fi and/or Ethernet connection. This function is implemented in [examples/common_components/protocol_examples_common/include/protocol_examples_common.h](../common_components/protocol_examples_common/include/protocol_examples_common.h), and has a very simple behavior: block until connection is established and IP address is obtained, then return. This function is used to reduce the amount of boilerplate and to keep the example code focused on the protocol or library being demonstrated.

The simple `example_connect()` function does not handle timeouts, does not gracefully handle various error conditions, and is only suited for use in examples. When developing real applications, this helper function needs to be replaced with full Wi-Fi / Ethernet connection handling code. Such code can be found in [examples/wifi/getting_started/](../wifi/getting_started) and [examples/ethernet/basic/](../ethernet/basic) examples.

