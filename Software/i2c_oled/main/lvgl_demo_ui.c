/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "lvgl.h"

// lv_obj_t *panel_top;
// lv_obj_t *panel_mid;
// lv_obj_t *panel_bot;
lv_obj_t *label_top;
lv_obj_t *label_mid;
lv_obj_t *label_bot;

typedef struct
{
    lv_obj_t *scr;
    int count_val;
    double *freq_1;
    double *freq_2;
} my_timer_context_t;
my_timer_context_t my_tim_ctx;

lv_timer_t *lvgl_ui_timer;

void example_lvgl_demo_ui_start(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    label_top = lv_label_create(scr);
    lv_label_set_text(label_top, "Freq (kHz)");
    lv_obj_align(label_top, LV_ALIGN_TOP_MID, 0, 0);

    label_mid = lv_label_create(scr);
    lv_label_set_text(label_mid, "1:\t");
    lv_obj_align(label_mid, LV_ALIGN_LEFT_MID, 0, 0);

    label_bot = lv_label_create(scr);
    lv_label_set_text(label_bot, "2:\t");
    lv_obj_align(label_bot, LV_ALIGN_BOTTOM_LEFT, 0, 0);
}



/// @brief 
/// @param loop1 
/// @param loop2 
void lvgl_ui_counter_update(double loop1, double loop2)
{
	char temp_digit_string[32] = {0};

	sprintf(temp_digit_string, "1: %10.6lf", loop1);

	lv_label_set_text(label_mid, temp_digit_string);

	sprintf(temp_digit_string, "M: %10.6lf", loop2);

	lv_label_set_text(label_bot, temp_digit_string);
}

static void example_lvgl_ui_count_up_timer_cb(lv_timer_t *timer)
{
	my_timer_context_t *timer_ctx = (my_timer_context_t *) timer->user_data;

	timer_ctx->count_val += 4;

    lvgl_ui_counter_update(*timer_ctx->freq_1, *timer_ctx->freq_2);

	if(timer_ctx->count_val <= 0)
	{
		lv_timer_del(timer);
	}

}

void example_lvgl_ui_create_timer(double* f_loop1, double* f_loop2)
{
	memset(&my_tim_ctx, 0, sizeof(my_tim_ctx));

    my_tim_ctx.freq_1 = f_loop1;
    my_tim_ctx.freq_2 = f_loop2;

	lvgl_ui_timer = lv_timer_create(example_lvgl_ui_count_up_timer_cb, 1, &my_tim_ctx);
}
