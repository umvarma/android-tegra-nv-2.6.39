/*
 * Camera low power control via GPIO
 *
 * Copyright (C) 2011 Eduardo Jos� Tagle <ejtagle@tutopia.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <linux/if.h>
#include <linux/skbuff.h>
#include <linux/random.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>

#include "board-smba1002.h"
#include "gpio-names.h"

struct smba1002_pm_camera_data {
	struct regulator *regulator;
#ifdef CONFIG_PM
	int pre_resume_state;
	int keep_on_in_suspend;
#endif
	int powered_up;
};


/* Power control */
static void __smba1002_pm_camera_power(struct device *dev, unsigned int on)
{
	struct smba1002_pm_camera_data *camera_data = dev_get_drvdata(dev);

	/* Avoid turning it on if already on */
	if (camera_data->powered_up == on)
		return;
	
	if (on) {
		dev_info(dev, "Enabling Camera\n");
	
		regulator_enable(camera_data->regulator);
	
		/* Camera power on sequence */
		gpio_set_value(SMBA1002_CAMERA_POWER, 0); /* Powerdown */
		msleep(2);
		gpio_set_value(SMBA1002_CAMERA_POWER, 1); /* Powerup */
		msleep(2);

	} else {
		dev_info(dev, "Disabling Camera\n");
		
		gpio_set_value(SMBA1002_CAMERA_POWER, 0); /* Powerdown */
		
		regulator_disable(camera_data->regulator);
	}
	
	/* store new state */
	camera_data->powered_up = on;
	
}

static ssize_t camera_read(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	int ret = 0;
	struct smba1002_pm_camera_data *camera_data = dev_get_drvdata(dev);
	
	if (!strcmp(attr->attr.name, "power_on")) {
		if (camera_data->powered_up)
			ret = 1;
	}
#ifdef CONFIG_PM
	else if (!strcmp(attr->attr.name, "keep_on_in_suspend")) {
		ret = camera_data->keep_on_in_suspend;
	}
#endif

	if (!ret) {
		return strlcpy(buf, "0\n", 3);
	} else {
		return strlcpy(buf, "1\n", 3);
	}
}

static ssize_t camera_write(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);
	struct smba1002_pm_camera_data *camera_data = dev_get_drvdata(dev);

	if (!strcmp(attr->attr.name, "power_on")) {
		__smba1002_pm_camera_power(dev, !!on);
	}
#ifdef CONFIG_PM
	else if (!strcmp(attr->attr.name, "keep_on_in_suspend")) {
		camera_data->keep_on_in_suspend = on;
	}
#endif

	return count;
}

static DEVICE_ATTR(power_on, 0666, camera_read, camera_write);
static DEVICE_ATTR(reset, 0666, camera_read, camera_write);
#ifdef CONFIG_PM
static DEVICE_ATTR(keep_on_in_suspend, 0666, camera_read, camera_write);
#endif

#ifdef CONFIG_PM
static int smba1002_camera_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct smba1002_pm_camera_data *camera_data = dev_get_drvdata(&pdev->dev);

	dev_dbg(&pdev->dev, "suspending\n");

	camera_data->pre_resume_state = camera_data->powered_up;
	
	if (!camera_data->keep_on_in_suspend)
		__smba1002_pm_camera_power(&pdev->dev, 0);
	else
		dev_warn(&pdev->dev, "keeping camera ON during suspend\n");
	return 0;
}

static int smba1002_camera_resume(struct platform_device *pdev)
{
	struct smba1002_pm_camera_data *camera_data = dev_get_drvdata(&pdev->dev);
	dev_dbg(&pdev->dev, "resuming\n");

	__smba1002_pm_camera_power(&pdev->dev, camera_data->pre_resume_state);
	return 0;
}
#else
#define smba1002_camera_suspend	NULL
#define smba1002_camera_resume	NULL
#endif

static struct attribute *smba1002_camera_sysfs_entries[] = {
	&dev_attr_power_on.attr,
	&dev_attr_reset.attr,
#ifdef CONFIG_PM
	&dev_attr_keep_on_in_suspend.attr,
#endif
	NULL
};

static struct attribute_group smba1002_camera_attr_group = {
	.name	= NULL,
	.attrs	= smba1002_camera_sysfs_entries,
};

/* ----- Initialization/removal -------------------------------------------- */
static int smba1002_camera_probe(struct platform_device *pdev)
{
	/* default-off */
	const int default_state = 0;

	struct regulator *regulator;
	struct smba1002_pm_camera_data *camera_data;

	camera_data = kzalloc(sizeof(*camera_data), GFP_KERNEL);
	if (!camera_data) {
		dev_err(&pdev->dev, "no memory for context\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, camera_data);

	regulator = regulator_get(&pdev->dev, "vdd_camera");
	if (IS_ERR(regulator)) {
		dev_err(&pdev->dev, "unable to get regulator 0\n");
		kfree(camera_data);
		dev_set_drvdata(&pdev->dev, NULL);
		return -ENODEV;
	}

	camera_data->regulator = regulator;

	/* Init io pins and disable camera */
	gpio_request(SMBA1002_CAMERA_POWER, "camera_power");
	gpio_direction_output(SMBA1002_CAMERA_POWER, 0);

	/* Set the default state */
	__smba1002_pm_camera_power(&pdev->dev, default_state);
	
	dev_info(&pdev->dev, "Camera power management driver loaded\n");
	
	return sysfs_create_group(&pdev->dev.kobj, &smba1002_camera_attr_group);
}

static int smba1002_camera_remove(struct platform_device *pdev)
{
	struct smba1002_pm_camera_data *camera_data = dev_get_drvdata(&pdev->dev);
	if (!camera_data)
		return 0;

	sysfs_remove_group(&pdev->dev.kobj, &smba1002_camera_attr_group);

	__smba1002_pm_camera_power(&pdev->dev, 0);
	
	regulator_put(camera_data->regulator);	

	kfree(camera_data);
	dev_set_drvdata(&pdev->dev, NULL);
	
	return 0;
}

static struct platform_driver smba1002_camera_driver = {
	.probe		= smba1002_camera_probe,
	.remove		= smba1002_camera_remove,
	.suspend	= smba1002_camera_suspend,
	.resume		= smba1002_camera_resume,
	.driver		= {
		.name		= "smba1002-pm-camera",
	},
};

static int __devinit smba1002_camera_init(void)
{
	return platform_driver_register(&smba1002_camera_driver);
}

static void smba1002_camera_exit(void)
{
	platform_driver_unregister(&smba1002_camera_driver);
}

module_init(smba1002_camera_init);
module_exit(smba1002_camera_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eduardo Jos� Tagle <ejtagle@tutopia.com>");
MODULE_DESCRIPTION("SMBA1002 CAMERA power management");
