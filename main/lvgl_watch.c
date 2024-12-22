/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"
//#include "lv_conf.h"

static lv_obj_t *label;

void example_lvgl_demo_ui(lv_disp_t *disp, const char *str)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
	lv_obj_clean(scr);
    label = lv_label_create(scr);

	// Set label style
	lv_style_t style;
	lv_style_init(&style);
	lv_style_set_text_font(&style, &lv_font_montserrat_14);
	lv_obj_add_style(label, &style, 0);
	
    //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, str);
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}

void update_label(const char *str)
{
    lv_label_set_text(label, str);
}

void clrscr()
{
	lv_obj_clean(lv_scr_act());
}
