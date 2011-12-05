/*
 * arch/arm/mach-tegra/board-smba1002.c
 *
 * Copyright (C) 2011 Eduardo José Tagle <ejtagle@tutopia.com>
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

#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/pda_power.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/i2c-tegra.h>
#include <linux/memblock.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/io.h>
#include <mach/w1.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/nand.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/gpio.h>
#include <mach/clk.h>
#include <mach/usb_phy.h>
#include <mach/system.h>
#include <mach/nvmap.h>

#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>

#include "board.h"
#include "board-smba1002.h"
#include "clock.h"
#include "gpio-names.h"
#include "devices.h"
#include "pm.h"
#include "wakeups-t2.h"


/* NVidia bootloader tags */
#define ATAG_NVIDIA		0x41000801

#define ATAG_NVIDIA_RM			0x1
#define ATAG_NVIDIA_DISPLAY		0x2
#define ATAG_NVIDIA_FRAMEBUFFER		0x3
#define ATAG_NVIDIA_CHIPSHMOO		0x4
#define ATAG_NVIDIA_CHIPSHMOOPHYS	0x5
#define ATAG_NVIDIA_PRESERVED_MEM_0	0x10000
#define ATAG_NVIDIA_PRESERVED_MEM_N	2
#define ATAG_NVIDIA_FORCE_32		0x7fffffff

struct tag_tegra {
	__u32 bootarg_key;
	__u32 bootarg_len;
	char bootarg[1];
};

static int __init parse_tag_nvidia(const struct tag *tag)
{
	return 0;
}
__tagtable(ATAG_NVIDIA, parse_tag_nvidia);

static atomic_t smba1002_gps_mag_powered = ATOMIC_INIT(0);
static struct {                                                          
    struct regulator *regulator[2];                                      
} gps_gsm_ctx = {                                                        
    {NULL,NULL}                                                          
};

void smba1002_gps_mag_poweron(void)
{
	if (atomic_inc_return(&smba1002_gps_mag_powered) == 1) {
		pr_info("Enabling GPS/Magnetic module\n");

        regulator_enable(gps_gsm_ctx.regulator[0]);
        regulator_enable(gps_gsm_ctx.regulator[1]);

		/* 3G/GPS power on sequence */
		gpio_set_value(SMBA1002_GPSMAG_DISABLE, 1); /* Enable power */
		msleep(2);
	}
}
EXPORT_SYMBOL_GPL(smba1002_gps_mag_poweron);

void smba1002_gps_mag_poweroff(void)
{
	if (atomic_dec_return(&smba1002_gps_mag_powered) == 0) {
		pr_info("Disabling GPS/Magnetic module\n");
		/* 3G/GPS power on sequence */
		gpio_set_value(SMBA1002_GPSMAG_DISABLE, 0); /* Disable power */
		msleep(2);

        regulator_enable(gps_gsm_ctx.regulator[0]);
        regulator_enable(gps_gsm_ctx.regulator[1]);

	}
}
EXPORT_SYMBOL_GPL(smba1002_gps_mag_poweroff);

static atomic_t smba1002_gps_mag_inited = ATOMIC_INIT(0);
int smba1002_gps_mag_init(void)
{
	if (atomic_inc_return(&smba1002_gps_mag_inited) == 1) {
		gpio_request(SMBA1002_GPSMAG_DISABLE, "gps_disable");
		gpio_direction_output(SMBA1002_GPSMAG_DISABLE, 1);

        gps_gsm_ctx.regulator[0] = regulator_get(NULL, "avdd_usb_pll");  
        if (IS_ERR(gps_gsm_ctx.regulator[0])) {                          
            pr_err("unable to get regulator for usb pll\n");             
            atomic_dec(&smba1002_gps_mag_inited);                          
            return -ENODEV;                                              
        }                                                                

        gps_gsm_ctx.regulator[1] = regulator_get(NULL, "avdd_usb");      
        if (IS_ERR(gps_gsm_ctx.regulator[1])) {                          
            pr_err("unable to get regulator for usb\n");                 
            regulator_put(gps_gsm_ctx.regulator[0]);                     
            atomic_dec(&smba1002_gps_mag_inited);                          
            return -ENODEV;
        }
	}
    return 0;
}
EXPORT_SYMBOL_GPL(smba1002_gps_mag_init);

void smba1002_gps_mag_deinit(void)
{
	atomic_dec(&smba1002_gps_mag_inited);
    if (atomic_dec_return(&smba1002_gps_mag_inited) == 0) {                
        regulator_put(gps_gsm_ctx.regulator[1]);                         
        regulator_put(gps_gsm_ctx.regulator[0]);                         
        memset(&gps_gsm_ctx,0,sizeof(gps_gsm_ctx));                      
    }
}
EXPORT_SYMBOL_GPL(smba1002_gps_mag_deinit);


////////

static atomic_t smba1002_wlan_bt_powered = ATOMIC_INIT(0);
static struct {
	struct clk *	  clk;
	struct regulator *regulator[3];
} wlan_bt_ctx = {
	NULL, {NULL,NULL,NULL}
};

void smba1002_wlan_bt_poweron(void)
{
	if (atomic_inc_return(&smba1002_wlan_bt_powered) == 1) {
		pr_info("Enabling WLAN/BT module\n");

		/* Wlan power on sequence - Taken from the AR6002 datasheet */
		gpio_set_value(SMBA1002_WLAN_POWER, 0); /* Powerdown */
		gpio_set_value(SMBA1002_WLAN_RESET, 0); /* Assert reset */
		gpio_set_value(SMBA1002_BT_RESET, 0); /* Assert reset */
		
		msleep(1);

		/* Turn on voltages properly sequencing them */
		regulator_enable(wlan_bt_ctx.regulator[0]); /* 1.8v */
		regulator_enable(wlan_bt_ctx.regulator[1]); /* 1.2v */
		regulator_enable(wlan_bt_ctx.regulator[2]);
		clk_enable(wlan_bt_ctx.clk);
		msleep(1);
		
		gpio_set_value(SMBA1002_WLAN_RESET, 1); /* Deassert Sys reset */
		msleep(5);
		
		gpio_set_value(SMBA1002_WLAN_POWER, 1); /* Take out the Wlan adapter from powerdown */
		msleep(5);

		/* just in case, reset the adapter again */
		gpio_set_value(SMBA1002_WLAN_RESET, 0); /* Assert Sys reset */
		msleep(5);
		gpio_set_value(SMBA1002_WLAN_RESET, 1); /* Deassert Sys reset */
		msleep(5); /* Leave some time for stabilization */
	
	
		/* Bluetooth power on sequence */
		msleep(200);
		gpio_set_value(SMBA1002_BT_RESET, 1); /* Deassert reset */
		msleep(2);
		
	}
}
EXPORT_SYMBOL_GPL(smba1002_wlan_bt_poweron);

void smba1002_wlan_bt_poweroff(void)
{
	if (atomic_dec_return(&smba1002_wlan_bt_powered) == 0) {
		pr_info("Disabling WLAN/BT module\n");
		
		gpio_set_value(SMBA1002_BT_RESET, 0); /* Assert reset */
		
		gpio_set_value(SMBA1002_WLAN_POWER, 0); /* Powerdown wlan adapter */
		msleep(1);
		
		gpio_set_value(SMBA1002_WLAN_RESET, 0); /* Assert reset */
		msleep(1);

		clk_disable(wlan_bt_ctx.clk);
		regulator_disable(wlan_bt_ctx.regulator[2]);
		regulator_disable(wlan_bt_ctx.regulator[1]);
		regulator_disable(wlan_bt_ctx.regulator[0]);
		msleep(1);
	}
}
EXPORT_SYMBOL_GPL(smba1002_wlan_bt_poweroff);

static atomic_t smba1002_wlan_bt_inited = ATOMIC_INIT(0);
int smba1002_wlan_bt_init(void)
{
	if (atomic_inc_return(&smba1002_wlan_bt_inited) == 1) {
	
		wlan_bt_ctx.regulator[0] = regulator_get(NULL, "vddio_wlan"); /* 1.8v */
		if (IS_ERR(wlan_bt_ctx.regulator[0])) {
			pr_err("unable to get regulator 0 (1.8v)\n");
			atomic_dec(&smba1002_wlan_bt_inited);
			return -ENODEV;
		}

		wlan_bt_ctx.regulator[1] = regulator_get(NULL, "vcore_wifi"); /* 1.2v */
		if (IS_ERR(wlan_bt_ctx.regulator[1])) {
			pr_err("unable to get regulator 1 (1.2v)\n");
			regulator_put(wlan_bt_ctx.regulator[0]);
			atomic_dec(&smba1002_wlan_bt_inited);
			return -ENODEV;
		}
	
		/* Init io pins */
		gpio_request(SMBA1002_WLAN_POWER, "wlan_power");
		gpio_direction_output(SMBA1002_WLAN_POWER, 0);

		gpio_request(SMBA1002_WLAN_RESET, "wlan_reset");
		gpio_direction_output(SMBA1002_WLAN_RESET, 0);

		wlan_bt_ctx.regulator[2] = regulator_get(NULL, "vddhostif_bt");
		if (IS_ERR(wlan_bt_ctx.regulator[2])) {
			pr_err("Failed to get regulator\n");
			regulator_put(wlan_bt_ctx.regulator[1]);
			regulator_put(wlan_bt_ctx.regulator[0]);
			atomic_dec(&smba1002_wlan_bt_inited);
			return -ENODEV;
		}
	
		wlan_bt_ctx.clk = clk_get(NULL, "blink");
		if (IS_ERR(wlan_bt_ctx.clk)) {
			pr_err("Failed to get clock\n");
			regulator_put(wlan_bt_ctx.regulator[2]);
			regulator_put(wlan_bt_ctx.regulator[1]);
			regulator_put(wlan_bt_ctx.regulator[0]);
			atomic_dec(&smba1002_wlan_bt_inited);
			return -ENODEV;
		}

		/* Init io pins */
		gpio_request(SMBA1002_BT_RESET, "bluetooth_reset");
		gpio_direction_output(SMBA1002_BT_RESET, 0);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(smba1002_wlan_bt_init);

void smba1002_wlan_bt_deinit(void)
{
	if (atomic_dec_return(&smba1002_wlan_bt_inited) == 0) {
	
		regulator_put(wlan_bt_ctx.regulator[2]);
		clk_put(wlan_bt_ctx.clk);
		
		regulator_put(wlan_bt_ctx.regulator[1]);	
		regulator_put(wlan_bt_ctx.regulator[0]);
		
		memset(&wlan_bt_ctx,0,sizeof(wlan_bt_ctx));
	}
	
}
EXPORT_SYMBOL_GPL(smba1002_wlan_bt_deinit);

static void smba1002_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void smba1002_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data smba1002_suspend = {
	.cpu_timer = 5000,
	.cpu_off_timer = 5000,
	.core_timer = 0x7e7e,
	.core_off_timer = 0x7f,
    .corereq_high = false,
	.sysclkreq_high = true,
	.suspend_mode = TEGRA_SUSPEND_LP1,
	.cpu_lp2_min_residency = 2000,
    .board_suspend = smba1002_board_suspend,
    .board_resume = smba1002_board_resume,
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE                                        
static struct resource ram_console_resources[] = {                       
    {                                                                    
        .flags = IORESOURCE_MEM,                                         
    },                                                                   
};                                                                      

static struct platform_device ram_console_device = {                    
    .name           = "ram_console",                                     
    .id             = -1,                                                
    .num_resources  = ARRAY_SIZE(ram_console_resources),                 
    .resource       = ram_console_resources,                             
};                                        

static void __init tegra_ramconsole_reserve(unsigned long size)
{
    struct resource *res;
    long ret;

    res = platform_get_resource(&ram_console_device, IORESOURCE_MEM, 0); 
    if (!res) {                                                          
        pr_err("Failed to find memory resource for ram console\n");      
        return;                                                          
    }                                                                    
    res->start = memblock_end_of_DRAM() - size;                          
    res->end = res->start + size - 1;                                    
    ret = memblock_remove(res->start, size);                             
    if (ret) {                                                           
        ram_console_device.resource = NULL;                              
        ram_console_device.num_resources = 0;                            
        pr_err("Failed to reserve memory block for ram console\n");      
    }                                                                    
}                                                                        
#endif

static void __init tegra_smba1002_init(void)
{
	struct clk *clk;

	/* force consoles to stay enabled across suspend/resume */
	// console_suspend_enabled = 0;	

	/* Init the suspend information */
	tegra_init_suspend(&smba1002_suspend);

	/* Set the SDMMC1 (wifi) tap delay to 6.  This value is determined
	 * based on propagation delay on the PCB traces. */
	clk = clk_get_sys("sdhci-tegra.0", NULL);
	if (!IS_ERR(clk)) {
		tegra_sdmmc_tap_delay(clk, 6);
		clk_put(clk);
	} else {
		pr_err("Failed to set wifi sdmmc tap delay\n");
	}

	/* Initialize the pinmux */
	smba1002_pinmux_init();

	/* Initialize the clocks - clocks require the pinmux to be initialized first */
	smba1002_clks_init();

	/* Register i2c devices - required for Power management and MUST be done before the power register */
	smba1002_i2c_register_devices();

	/* Register the power subsystem - Including the poweroff handler - Required by all the others */
	smba1002_power_register_devices();
	
	/* Register the USB device */
	smba1002_usb_register_devices();

	/* Register UART devices */
	smba1002_uart_register_devices();
	
	/* Register SPI devices */
	smba1002_spi_register_devices();

	/* Register GPU devices */
	smba1002_gpu_register_devices();

	/* Register Audio devices */
	smba1002_audio_register_devices();

	/* Register Jack devices */
	//smba1002_jack_register_devices();

	/* Register AES encryption devices */
	smba1002_aes_register_devices();

	/* Register Watchdog devices */
	smba1002_wdt_register_devices();

	/* Register all the keyboard devices */
	smba1002_keyboard_register_devices();
	
	/* Register touchscreen devices */
	smba1002_touch_register_devices();
	
	/* Register SDHCI devices */
	smba1002_sdhci_register_devices();

	/* Register accelerometer device */
	smba1002_sensors_register_devices();
	
	/* Register wlan powermanagement devices */
//	smba1002_wlan_pm_register_devices();
	
	/* Register gps powermanagement devices */
	smba1002_gps_pm_register_devices();

	/* Register gsm powermanagement devices */
	smba1002_gsm_pm_register_devices();
	
	/* Register Bluetooth powermanagement devices */
	smba1002_bt_pm_register_devices();

	/* Register Camera powermanagement devices */
//	smba1002_camera_register_devices();

	/* Register NAND flash devices */
	smba1002_nand_register_devices();
	
	smba1002_gps_mag_init();
	smba1002_gps_mag_poweron();
#if 0
	/* Finally, init the external memory controller and memory frequency scaling
   	   NB: This is not working on SMBA1002. And seems there is no point in fixing it,
	   as the EMC clock is forced to the maximum speed as soon as the 2D/3D engine
	   starts.*/
	smba1002_init_emc();
#endif
	
#ifdef _DUMP_WBCODE                                                      
    dump_warmboot(tegra_lp0_vec_start,tegra_lp0_vec_size);
#endif

#ifdef _DUMP_BOOTCAUSE
    dump_bootflags();
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE                                        
    /* Register the RAM console device */                                
    platform_device_register(&ram_console_device);                       
#endif                                                                   
    /* Release the tegra bootloader framebuffer */                       
    tegra_release_bootloader_fb();
}

static void __init tegra_smba1002_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

#if defined(DYNAMIC_GPU_MEM)
	/* Reserve the graphics memory */
	tegra_reserve(SMBA1002_GPU_MEM_SIZE, SMBA1002_FB1_MEM_SIZE, SMBA1002_FB2_MEM_SIZE);
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE                                        
    /* Reserve 1M memory for the RAM console */                          
    tegra_ramconsole_reserve(SZ_1M);                                     
#endif
}

static void __init tegra_smba1002_fixup(struct machine_desc *desc,
	struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = SMBA1002_MEM_BANKS;
	mi->bank[0].start = PHYS_OFFSET;
#if defined(DYNAMIC_GPU_MEM)
	mi->bank[0].size  = SMBA1002_MEM_SIZE;
#else
	mi->bank[0].size  = SMBA1002_MEM_SIZE - SMBA1002_GPU_MEM_SIZE;
#endif
	// Adam has two 512MB banks. Easier to hardcode if we leave SMBA1002_MEM_SIZE at 512MB
	mi->bank[1].start = SMBA1002_MEM_SIZE;
	mi->bank[1].size = SMBA1002_MEM_SIZE;
} 

MACHINE_START(HARMONY, "harmony")
	.boot_params	= 0x00000100,
	.map_io         = tegra_map_common_io,
	.init_early     = tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer, 	
	.init_machine	= tegra_smba1002_init,
	.reserve		= tegra_smba1002_reserve,
	.fixup			= tegra_smba1002_fixup,
MACHINE_END

#if 0
#define PMC_WAKE_STATUS 0x14

static int smba1002_wakeup_key(void)
{
	unsigned long status = 
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
	return status & TEGRA_WAKE_GPIO_PV2 ? KEY_POWER : KEY_RESERVED;
}
#endif


