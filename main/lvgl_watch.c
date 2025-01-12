/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"

//#define PULSE_SYMBOL "\xEF\x80\x84"
#define PULSE_SYMBOL "BPM: "

//#include "lv_conf.h"

static lv_obj_t *time_label;
static lv_obj_t *pulse_label;
static lv_obj_t *pulse_label_2;
static lv_obj_t *pulse_symbol_label;

void set_lvgl_ui(lv_disp_t *disp, const char *str)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

	lv_obj_clean(scr);

	// Time label
    time_label = lv_label_create(scr);

	// Set time label style
	lv_style_t time_label_style;
	lv_style_init(&time_label_style);
	lv_style_set_text_font(&time_label_style, &lv_font_montserrat_18);
	lv_obj_add_style(time_label, &time_label_style, 0);
	
    lv_label_set_text(time_label, str);
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(time_label, disp->driver->hor_res);
    lv_obj_align(time_label, LV_ALIGN_TOP_MID, 0, 0);

	// Pulse symbol label
    //pulse_symbol_label = lv_label_create(scr);
	//lv_label_set_text(pulse_symbol_label, PULSE_SYMBOL);
    //lv_obj_set_width(pulse_symbol_label, disp->driver->hor_res);
    //lv_obj_align(pulse_symbol_label, LV_ALIGN_TOP_MID, 0, 40);

	// Pulse label
    pulse_label = lv_label_create(scr);

	// Set label style
	lv_style_t pulse_label_style;
	lv_style_init(&pulse_label_style);
	lv_style_set_text_font(&pulse_label_style, &lv_font_montserrat_18);
	lv_obj_add_style(pulse_label, &pulse_label_style, 0);
	

    lv_label_set_text(pulse_label, "--");
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(pulse_label, disp->driver->hor_res);
    lv_obj_align(pulse_label, LV_ALIGN_TOP_MID, 0, 40);

	//// Pulse label 2
    //pulse_label_2 = lv_label_create(scr);

	//// Set label style
	//lv_style_t pulse_label_2_style;
	//lv_style_init(&pulse_label_2_style);
	//lv_style_set_text_font(&pulse_label_2_style, &lv_font_montserrat_18);
	//lv_obj_add_style(pulse_label_2, &pulse_label_2_style, 0);
	//

    //lv_label_set_text(pulse_label_2, "--");
    ///* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    //lv_obj_set_width(pulse_label_2, disp->driver->hor_res);
    //lv_obj_align(pulse_label_2, LV_ALIGN_TOP_MID, 40, 40);
}

void update_time_label(const char *str)
{
    lv_label_set_text(time_label, str);
}

void update_pulse_label(const char *str)
{
    lv_label_set_text(pulse_label, str);
}

void update_pulse_label_2(const char *str)
{
    lv_label_set_text(pulse_label_2, str);
}

void clrscr()
{
	lv_obj_clean(lv_scr_act());
}
