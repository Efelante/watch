/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"

static lv_obj_t *time_label;
static lv_obj_t *pulse_label;

void example_lvgl_demo_ui(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);

	// Circular label
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "Hello Espressif, Hello LVGL.");
    /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
    lv_obj_set_width(label, lv_display_get_horizontal_resolution(disp));
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

	// Time label
	time_label = lv_label_create(scr);
    lv_label_set_text(time_label, "--");
    /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
    lv_obj_set_width(time_label, lv_display_get_horizontal_resolution(disp));
    lv_obj_align(time_label, LV_ALIGN_TOP_MID, 0, 20);

	// Pulse label
	pulse_label = lv_label_create(scr);
    lv_label_set_text(pulse_label, "--");
    /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
    lv_obj_set_width(pulse_label, lv_display_get_horizontal_resolution(disp));
    lv_obj_align(pulse_label, LV_ALIGN_TOP_MID, 0, 40);
}

void update_time_label(const char *str)
{
    lv_label_set_text(time_label, str);
}

void update_pulse_label(const char *str)
{
    lv_label_set_text(pulse_label, str);
}

//void update_pulse_label_2(const char *str)
//{
//    lv_label_set_text(pulse_label_2, str);
//}
//
//void clrscr()
//{
//	lv_obj_clean(lv_scr_act());
//}
