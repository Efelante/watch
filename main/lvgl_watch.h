#ifndef __LVGL_WATCH_H__
#define __LVGL_WATCH_H__

void set_lvgl_ui(lv_disp_t *disp, const char *str);
void update_time_label(const char *str);
void update_pulse_label(const char *str);
void update_pulse_label_2(const char *str);
void clrscr();

#endif //__LVGL_WATCH_H__
