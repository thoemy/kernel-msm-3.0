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
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>

#include <linux/synaptics_i2c_rmi.h>
#include <linux/capella_cm3602_htc.h>
#include <linux/akm8973.h>
#include <linux/regulator/machine.h>
#include <linux/ds2784_battery.h>
#include <../../../drivers/w1/w1.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/socinfo.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_smd.h>
#include <mach/gpiomux.h>
#include <mach/board-bravo-microp-common.h>

#include "board-bravo.h"
#include "devices.h"
#include "proc_comm.h"
#include "board-bravo-tpa2018d1.h"
#include "board-bravo-smb329.h"

#include "timer.h"
#include "acpuclock.h"
#include "pm.h"
#include "gpiomux-8x50.h"

#define SMEM_SPINLOCK_I2C	"S:6"

void (*msm_hw_reset_hook)(void);

static uint debug_uart;

module_param_named(debug_uart, debug_uart, uint, 0);

extern void notify_usb_connected(int);
extern void msm_init_pmic_vibrator(void);
extern void __init bravo_audio_init(void);

extern int microp_headset_has_mic(void);

static int bravo_phy_init_seq[] = {
	0x0C, 0x31,
	0x31, 0x32,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1
};

static void bravo_usb_phy_reset(void)
{
	u32 id;
	int ret;

	id = PCOM_CLKRGM_APPS_RESET_USB_PHY;
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}

	msleep(1);

	id = PCOM_CLKRGM_APPS_RESET_USB_PHY;
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
}

static struct msm_hsusb_gadget_platform_data msm_hsusb_gadget_pdata = {
	.phy_init_seq		= bravo_phy_init_seq,
	.phy_reset		= bravo_usb_phy_reset,
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_ACCESSORY
static char *usb_functions_accessory[] = { "accessory" };
static char *usb_functions_accessory_adb[] = { "accessory", "adb" };
#endif

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	"accessory",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0ff9,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c87,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0FFE,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	/*
	XXX: there isn't a equivalent in htc's kernel
	{
		.product_id	= 0x4e14,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	}, */
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	{
		.vendor_id  = USB_ACCESSORY_VENDOR_ID,
		.product_id  = USB_ACCESSORY_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_accessory),
		.functions  = usb_functions_accessory,
	},
	{
		.vendor_id  = USB_ACCESSORY_VENDOR_ID,
		.product_id  = USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_accessory_adb),
		.functions  = usb_functions_accessory_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x0c07,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
#endif
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c02,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static int bravo_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		/* level shifter should be off */
		gpio_set_value(BRAVO_GPIO_TP_EN, 1);
		msleep(120);
		/* enable touch panel level shift */
		gpio_set_value(BRAVO_GPIO_TP_LS_EN, 1);
		msleep(3);
	} else {
		gpio_set_value(BRAVO_GPIO_TP_LS_EN, 0);
		gpio_set_value(BRAVO_GPIO_TP_EN, 0);
		udelay(50);
	}

	return 0;
}

static struct synaptics_i2c_rmi_platform_data bravo_synaptics_ts_data[] = {
	{
		.version = 0x100,
		.power = bravo_ts_power,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -1 * 0x10000 / 480,
		.inactive_right = -1 * 0x10000 / 480,
		.inactive_top = -5 * 0x10000 / 800,
		.inactive_bottom = -5 * 0x10000 / 800,
		.sensitivity_adjust = 12,
	}
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = BRAVO_LAYOUTS,
	.project_name = BRAVO_PROJECT_NAME,
	.reset = BRAVO_GPIO_COMPASS_RST_N,
	.intr = BRAVO_GPIO_COMPASS_INT_N,
};

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] = {
	{
		.supply = "acpu_vcore",
	},
};

static struct regulator_init_data tps65023_data[5] = {
	{
		.constraints = {
			.name = "dcdc1", /* VREG_MSMC2_1V29 */
			.min_uV = BRAVO_MIN_UV_MV * 1000,
			.max_uV = BRAVO_MAX_UV_MV * 1000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		},
		.consumer_supplies = tps65023_dcdc1_supplies,
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
	},
	/* dummy values for unused regulators to not crash driver: */
	{
		.constraints = {
			.name = "dcdc2", /* VREG_MSMC1_1V26 */
			.min_uV = 1260000,
			.max_uV = 1260000,
		},
	},
	{
		.constraints = {
			.name = "dcdc3", /* unused */
			.min_uV = 800000,
			.max_uV = 3300000,
		},
	},
	{
		.constraints = {
			.name = "ldo1", /* unused */
			.min_uV = 1000000,
			.max_uV = 3150000,
		},
	},
	{
		.constraints = {
			.name = "ldo2", /* V_USBPHY_3V3 */
			.min_uV = 3300000,
			.max_uV = 3300000,
		},
	},
};

static void ds2482_set_slp_n(unsigned n)
{
	gpio_direction_output(BRAVO_GPIO_DS2482_SLP_N, n);
}

static int capella_cm3602_power(int pwr_device, uint8_t enable);
static struct microp_function_config microp_functions[] = {
	{
		.name = "light_sensor",
		.category = MICROP_FUNCTION_LSENSOR,
		.levels = { 0x000, 0x001, 0x00F, 0x01E, 0x03C, 0x121, 0x190, 0x2BA, 0x35C, 0x3FF },
		.channel = 6,
		.int_pin = IRQ_LSENSOR,
		.golden_adc = 0xC0,
		.ls_power = capella_cm3602_power,
	},
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_functions[0],
	.irq = MSM_uP_TO_INT(9),
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = BRAVO_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_GSENSOR,
};

static struct tpa2018d1_platform_data tpa2018_data = {
	.gpio_tpa2018_spk_en = BRAVO_CDMA_GPIO_AUD_SPK_AMP_EN,
};

static struct i2c_board_info base_i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x40),
		.platform_data = bravo_synaptics_ts_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_TP_INT_N)
	},
	{
		I2C_BOARD_INFO("bravo-microp", 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO("ds2482", 0x30 >> 1),
		.platform_data = ds2482_set_slp_n,
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
};

static struct i2c_board_info rev_CX_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tpa2018d1", 0x58),
		.platform_data = &tpa2018_data,
	},
	{
		I2C_BOARD_INFO("smb329", 0x6E >> 1),
	},
};


static int __capella_cm3602_power(int on)
{
	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");
	if (on) {
		gpio_direction_output(BRAVO_GPIO_LS_EN_N, 0);
		gpio_direction_output(BRAVO_GPIO_PROXIMITY_EN, 1);
	} else {
		gpio_direction_output(BRAVO_GPIO_LS_EN_N, 1);
	}
	return 0;
};

static DEFINE_MUTEX(capella_cm3602_lock);
static int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	/* TODO eolsen Add Voltage reg control */
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
};

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en = BRAVO_GPIO_PROXIMITY_EN,
	.p_out = BRAVO_GPIO_PROXIMITY_INT_N,
	.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_PROXIMITY_INT_N),
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};

static int ds2784_charge(int on, int fast)
{
	if (is_cdma_version(system_rev)) {
		if (!on)
			smb329_set_charger_ctrl(SMB329_DISABLE_CHG);
		else
			smb329_set_charger_ctrl(fast ? SMB329_ENABLE_FAST_CHG : SMB329_ENABLE_SLOW_CHG);
	}
	else
		gpio_direction_output(BRAVO_GPIO_BATTERY_CHARGER_CURRENT, !!fast);
	gpio_direction_output(BRAVO_GPIO_BATTERY_CHARGER_EN, !on);
	return 0;
}

static int w1_ds2784_add_slave(struct w1_slave *sl)
{
	struct dd {
		struct platform_device pdev;
		struct ds2784_platform_data pdata;
	} *p;

	int rc;

	p = kzalloc(sizeof(struct dd), GFP_KERNEL);
	if (!p) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	rc = gpio_request(BRAVO_GPIO_BATTERY_CHARGER_EN, "charger_en");
	if (rc < 0) {
		pr_err("%s: gpio_request(%d) failed: %d\n", __func__,
			BRAVO_GPIO_BATTERY_CHARGER_EN, rc);
		kfree(p);
		return rc;
	}

	if (!is_cdma_version(system_rev)) {
		rc = gpio_request(BRAVO_GPIO_BATTERY_CHARGER_CURRENT, "charger_current");
		if (rc < 0) {
			pr_err("%s: gpio_request(%d) failed: %d\n", __func__,
					BRAVO_GPIO_BATTERY_CHARGER_CURRENT, rc);
			gpio_free(BRAVO_GPIO_BATTERY_CHARGER_EN);
			kfree(p);
			return rc;
		}
	}

	p->pdev.name = "ds2784-battery";
	p->pdev.id = -1;
	p->pdev.dev.platform_data = &p->pdata;
	p->pdata.charge = ds2784_charge;
	p->pdata.w1_slave = sl;

	platform_device_register(&p->pdev);

	return 0;
}

static struct w1_family_ops w1_ds2784_fops = {
	.add_slave = w1_ds2784_add_slave,
};

static struct w1_family w1_ds2784_family = {
	.fid = W1_FAMILY_DS2784,
	.fops = &w1_ds2784_fops,
};

static int __init ds2784_battery_init(void)
{
	return w1_register_family(&w1_ds2784_family);
}

static struct platform_device *devices[] __initdata = {
    &ram_console_device,
    &msm_device_smd,
    &msm_device_i2c,
    &capella_cm3602,
};

static struct msm_gpio bt_gpio_table[] = {
    {GPIO_CFG(BRAVO_GPIO_BT_UART1_RTS, 2, GPIO_CFG_OUTPUT,
              GPIO_CFG_PULL_UP, GPIO_CFG_8MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_UART1_CTS, 2, GPIO_CFG_INPUT,
              GPIO_CFG_PULL_UP, GPIO_CFG_8MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_UART1_RX, 2, GPIO_CFG_INPUT,
              GPIO_CFG_PULL_UP, GPIO_CFG_8MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_UART1_TX, 2, GPIO_CFG_OUTPUT,
              GPIO_CFG_PULL_UP, GPIO_CFG_8MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_RESET_N, 0, GPIO_CFG_OUTPUT,
              GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_SHUTDOWN_N, 0, GPIO_CFG_OUTPUT,
              GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_WAKE, 0, GPIO_CFG_OUTPUT,
              GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_HOST_WAKE, 0, GPIO_CFG_INPUT,
              GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA)},
};

static struct msm_gpio bt_gpio_table_rev_CX[] = {
    {GPIO_CFG(BRAVO_GPIO_BT_UART1_RTS, 2, GPIO_CFG_OUTPUT,
              GPIO_CFG_PULL_UP, GPIO_CFG_8MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_UART1_CTS, 2, GPIO_CFG_INPUT,
              GPIO_CFG_PULL_UP, GPIO_CFG_8MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_UART1_RX, 2, GPIO_CFG_INPUT,
             GPIO_CFG_PULL_UP, GPIO_CFG_8MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_UART1_TX, 2, GPIO_CFG_OUTPUT,
             GPIO_CFG_PULL_UP, GPIO_CFG_8MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_RESET_N, 0, GPIO_CFG_OUTPUT,
             GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_SHUTDOWN_N, 0, GPIO_CFG_OUTPUT,
             GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA)},
    {GPIO_CFG(BRAVO_CDMA_GPIO_BT_WAKE, 0, GPIO_CFG_OUTPUT,
             GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA)},
    {GPIO_CFG(BRAVO_GPIO_BT_HOST_WAKE, 0, GPIO_CFG_INPUT,
             GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA)},
};

static struct msm_gpio misc_gpio_table[] = {
    { GPIO_CFG(BRAVO_GPIO_LCD_RST_N, 0, GPIO_CFG_OUTPUT,
               GPIO_CFG_NO_PULL, GPIO_CFG_2MA)},
    { GPIO_CFG(BRAVO_GPIO_LED_3V3_EN, 0, GPIO_CFG_OUTPUT,
               GPIO_CFG_NO_PULL, GPIO_CFG_2MA)},
    { GPIO_CFG(BRAVO_GPIO_DOCK, 0, GPIO_CFG_OUTPUT,
               GPIO_CFG_NO_PULL, GPIO_CFG_4MA)},
};

static struct msm_gpio key_int_shutdown_gpio_table[] = {
    {GPIO_CFG(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0, GPIO_CFG_OUTPUT,
              GPIO_CFG_NO_PULL, GPIO_CFG_2MA)},
};

static void bravo_headset_init(void)
{
	if (is_cdma_version(system_rev))
		return;
	msm_gpios_enable(key_int_shutdown_gpio_table,
                         ARRAY_SIZE(key_int_shutdown_gpio_table));
	gpio_set_value(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0);
}


#define ATAG_BDADDR 0x43294329  /* bravo bluetooth address tag */
#define ATAG_BDADDR_SIZE 4
#define BDADDR_STR_SIZE 18

static char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0400);
MODULE_PARM_DESC(bdaddr, "bluetooth address");

static int __init parse_tag_bdaddr(const struct tag *tag)
{
	unsigned char *b = (unsigned char *)&tag->u;

	if (tag->hdr.size != ATAG_BDADDR_SIZE)
		return -EINVAL;

	snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
			b[0], b[1], b[2], b[3], b[4], b[5]);

	return 0;
}

__tagtable(ATAG_BDADDR, parse_tag_bdaddr);

static int __init bravo_board_serialno_setup(char *serialno)
{
#ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
#endif

	android_usb_pdata.serial_number = serialno;
	//msm_hsusb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", bravo_board_serialno_setup);

#ifdef CONFIG_PERFLOCK
static unsigned bravo_perf_acpu_table[] = {
	128000000,
	576000000,
	998400000,
};

static struct perflock_platform_data bravo_perflock_data = {
	.perf_acpu_table = bravo_perf_acpu_table,
	.table_size = ARRAY_SIZE(bravo_perf_acpu_table),
};
#endif

static void bravo_reset(void)
{
    printk("bravo_reset()\n");
	gpio_set_value(BRAVO_GPIO_PS_HOLD, 0);
};

static const struct smd_tty_channel_desc smd_cdma_default_channels[] = {
	{ .id = 0, .name = "SMD_DS" },
	{ .id = 19, .name = "SMD_DATA3" },
	{ .id = 27, .name = "SMD_GPSNMEA" }
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 60;
		gpio_sda = 61;
	} else {
		gpio_scl = 95;
		gpio_sda = 96;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.rsl_id = SMEM_SPINLOCK_I2C,
	.pri_clk = 95,
	.pri_dat = 96,
	.aux_clk = 60,
	.aux_dat = 61,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(95, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(96, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (gpio_request(60, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(61, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");

	msm_i2c_pdata.rmutex = 1;
	msm_i2c_pdata.pm_lat =
		msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static void __init bravo_init(void)
{
	int ret;

	printk("bravo_init() revision=%d\n", system_rev);

	if (is_cdma_version(system_rev))
		smd_set_channel_list(smd_cdma_default_channels,
				ARRAY_SIZE(smd_cdma_default_channels));

	msm_hw_reset_hook = bravo_reset;

	bravo_board_serialno_setup(board_serialno());

	msm_clock_init(&qsd8x50_clock_init_data);

	qsd8x50_init_gpiomux(qsd8x50_gpiomux_cfgs);

        /* TODO: CDMA version */
	acpuclk_init(&acpuclk_8x50_soc_data);

        msm_gpios_enable(misc_gpio_table, ARRAY_SIZE(misc_gpio_table));

        if (is_cdma_version(system_rev)) {
            //bcm_bt_lpm_pdata.gpio_wake = BRAVO_CDMA_GPIO_BT_WAKE;
            //bravo_flashlight_data.torch = BRAVO_CDMA_GPIO_FLASHLIGHT_TORCH;
            msm_gpios_enable(bt_gpio_table_rev_CX, ARRAY_SIZE(bt_gpio_table_rev_CX));
	} else {
            msm_gpios_enable(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));
	}

        gpio_request(BRAVO_GPIO_TP_LS_EN, "tp_ls_en");
	gpio_direction_output(BRAVO_GPIO_TP_LS_EN, 0);
	gpio_request(BRAVO_GPIO_TP_EN, "tp_en");
	gpio_direction_output(BRAVO_GPIO_TP_EN, 0);
//	gpio_request(BRAVO_GPIO_PROXIMITY_EN, "proximity_en");
//	gpio_direction_output(BRAVO_GPIO_PROXIMITY_EN, 1);
	gpio_request(BRAVO_GPIO_LS_EN_N, "ls_en");
	gpio_request(BRAVO_GPIO_COMPASS_RST_N, "compass_rst");
	gpio_direction_output(BRAVO_GPIO_COMPASS_RST_N, 1);
	gpio_request(BRAVO_GPIO_COMPASS_INT_N, "compass_int");
	gpio_direction_input(BRAVO_GPIO_COMPASS_INT_N);

	gpio_request(BRAVO_GPIO_DS2482_SLP_N, "ds2482_slp_n");

        //msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	//msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;

	platform_add_devices(devices, ARRAY_SIZE(devices));

	//platform_add_devices(msm_footswitch_devices,
        //                     msm_num_footswitch_devices);

        msm_device_i2c_init();

	i2c_register_board_info(0, base_i2c_devices,
                                ARRAY_SIZE(base_i2c_devices));

        if (is_cdma_version(system_rev)) {
            i2c_register_board_info(0, rev_CX_i2c_devices,
                                    ARRAY_SIZE(rev_CX_i2c_devices));
	}

        bravo_headset_init();

        ds2784_battery_init();
        //bravo_reset();
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

static void __init bravo_init_irq(void)
{
    printk("bravo_init_irq()\n");
    msm_init_irq();
    msm_init_sirc();
}

static void __init bravo_map_io(void)
{
    printk("bravo_map_io()\n");
    msm_map_qsd8x50_io();

    if (socinfo_init() < 0)
        pr_err("socinfo_init() failed!\n");
}

#ifdef CONFIG_MACH_BRAVO
MACHINE_START(BRAVO, "bravo")
#else
MACHINE_START(BRAVOC, "bravoc")
#endif
    .boot_params = 0x20000100,
    .fixup = bravo_fixup,
    .map_io = bravo_map_io,
    .init_irq = bravo_init_irq,
    .init_machine = bravo_init,
    .timer = &msm_timer,
MACHINE_END
