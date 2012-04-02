/* arch/arm/mach-msm/board-bravo.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
 * Copyright (C) 2010 Giulio Cervera <giulio.cervera@gmail.com>
 * Copyright (C) 2010 Diogo Ferreira <diogo@underdev.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/socinfo.h>

#include "board-bravo.h"
#include "devices.h"
#include "proc_comm.h"
#include "timer.h"

static uint debug_uart;

module_param_named(debug_uart, debug_uart, uint, 0);

static void bravo_reset(void)
{
    printk("bravo_reset()\n");
    gpio_set_value(BRAVO_GPIO_PS_HOLD, 0);
};

static void __init bravo_init(void)
{
    printk("bravo_init() revision=%d\n", system_rev);
    msm_clock_init(&qsd8x50_clock_init_data);
    //qsd8x50_init_gpiomux(board_data->gpiomux_cfgs);
    bravo_reset();
}

static void __init bravo_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
    printk("bravo_fixup(...)\n");
	mi->nr_banks = 2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
	mi->bank[1].start = MSM_EBI1_BANK1_BASE;
	mi->bank[1].size = MSM_EBI1_BANK1_SIZE;
}

static void __init bravo_map_io(void)
{
    printk("bravo_map_io()\n");
    msm_map_qsd8x50_io();

    if (socinfo_init() < 0)
        pr_err("socinfo_init() failed!\n");
}

static void __init bravo_init_early(void)
{
    printk("bravo_init_early()\n");
    //qsd8x50_allocate_memory_regions();
}

extern struct sys_timer msm_timer;

#ifdef CONFIG_MACH_BRAVO
MACHINE_START(BRAVO, "bravo")
#else
MACHINE_START(BRAVOC, "bravoc")
#endif
    .boot_params = 0x20000100,
    .fixup = bravo_fixup,
    .map_io = bravo_map_io,
//    .reserve = bravo_reserve,
    .init_irq	= msm_init_irq,
//    .handle_irq = gic_handle_irq,
    .init_machine = bravo_init,
    .timer = &msm_timer,
    .init_early = bravo_init_early,
MACHINE_END
