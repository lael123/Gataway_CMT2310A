/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-21     Rick       the first version
 */
#ifndef APPLICATIONS_LED_H_
#define APPLICATIONS_LED_H_

void led_Init(void);
void wifi_led(uint8_t type);
void beep_start(uint8_t count);
void beep_power(uint8_t count);
void learn_success(void);
void learn_fail(void);
void rf_led(uint8_t type);

#endif /* APPLICATIONS_LED_H_ */
