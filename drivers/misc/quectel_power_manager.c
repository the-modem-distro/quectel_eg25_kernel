/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Quectel power_manager  driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/atomic.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/quectel_power_manager.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
/*
 * NOTE: I'm starting to hate these people. This driver is empty in all their releases
 * 
 * It should expose:
 *  dtr_in
 *  dtr_state
 *  sleep_polarity
 *  wakeup_enable
 *  wakeup_in
 *  wakeup_polarity
 *
 * But the released implementation is just an empty module which does nothing.
 * So, time to reconstruct everything from scratch.
 */
/* How I think this works:
 * 
 *
 * 
 *
 * Wakeup_In(dicator): 
 * Used by atfwd_daemon and quectel_daemon to keep the modem alive or let it sleep
 * Wakeup_in_polarity: My theory is that it can switch the direction to signal the
 * modem to sleep or switched, to let the modem notify the pinephone of the state
 * not sure if it's used though, need to disassemble both atfwd and quectel daemons
 * wakeup_enable: Don't know.
 * dtr_state: Not used by anything anywhere, apparently
 * dtr_in: Used to notify modem ready from hexagon to userspace. It also has
 * something to do with ring in I think, but not sure where it is going
 * If the GPIO setting aids with atfwd to echo a RING into the AT port or something
 * like that. It is read/set by atfwd_daemon
 * 
 */
/*
 *  States:
 *  wakup_flag: 1: SLEEP, 0: AWAKE
 *  dtr_flag: Signaled high when hexagon boots
 *  DTR_IN: High on suspend, low when active
 * 
 */
/*
Quectel_daemon sets the following params via sysfs:
sh: /sys/devices/soc:quec,quectel-power-manager/sleep_polarity: Permission denied
sh: /sys/devices/soc:quec,quectel-power-manager/wakeup_in_polarity: Permission denied
sh: /sys/devices/soc:quec,quectel-power-manager/wakeup_enable: Permission denied

need to watch them to see the values it is setting

*/
#define WAKEUP_FILE "/sys/devices/soc:quec,quectel-power-manager/wakeup_in"
static DEFINE_MUTEX(wakeup_lock);

/* SHOW functions: these report data to sysfs when requested */
static ssize_t show_sleep_polarity(struct device *dev, struct device_attribute *attr, 
                                    char *buf)
{
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    ssize_t ret;
    pr_info("%s: %i", __func__, data->sleep_polarity);
    if(data->sleep_polarity) {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "1");
    } else  {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "0");
    }

	return ret;
}
static ssize_t show_wakeup_in_polarity(struct device *dev, struct device_attribute *attr, 
                                        char *buf)
{
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    ssize_t ret;
    pr_info("%s: %i", __func__, data->wakeup_in_polarity);
    if(data->wakeup_in_polarity) {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "1");
    } else  {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "0");
    }
   	return ret;
}
static ssize_t show_wakeup_enable(struct device *dev, struct device_attribute *attr, 
                                    char *buf)
{
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    ssize_t ret;
    pr_info("%s: %i", __func__, data->wakeup_enable);
    if(data->wakeup_enable) {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "1");
    } else  {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "0");
    }
   	return ret;
}
static ssize_t show_dtr_status(struct device *dev, struct device_attribute *attr, 
                                char *buf)
{
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    ssize_t ret;
    pr_info("%s: %i", __func__, data->dtr_flag);
    if(data->dtr_flag) {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "1");
    } else  {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "0");
    }
   	return ret;
}

static ssize_t show_wakeup_status(struct device *dev, struct device_attribute *attr, 
                                    char *buf)
{
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    ssize_t ret;

	mutex_lock(&wakeup_lock);

    if(data->wakeup_flag) {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "Sleep");
    } else {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "Wakeup");
    }

	wake_unlock(&data->wlock_wakeup);
	mutex_unlock(&wakeup_lock);
	
	return ret;//must return none-zero value or will call poll continuously
}

extern char g_wakeup_src[]; // This comes from the gpio patch from quectel - see drivers/irqchip/irq-gic.c
static ssize_t show_wakeup_source(struct device *dev,
		                            struct device_attribute *attr, 
                                    char *buf)
{
    ssize_t ret;
	struct quectel_power_manager *data = dev_get_drvdata(dev);

	pr_info("[QTL_PM]: wakeup source: %s \n", g_wakeup_src);
	mutex_lock(&wakeup_lock);
    ret = snprintf(buf, PAGE_SIZE, "%s\n", g_wakeup_src);
	wake_unlock(&data->wlock_wakeup);

	mutex_unlock(&wakeup_lock);
	
	return ret;//must return none-zero value or will call poll continuously
}

// Store the wakeup source
static ssize_t store_wakeup_source(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
    ssize_t len = n;
	struct quectel_power_manager *data = dev_get_drvdata(dev);
	device_lock(dev);
	if(len > 32) {
	    len = 32;
	}
    pr_info("[QTL_PM]: Wakeup source: %s \n", buf);
	memcpy(data->wakeup_source, buf, len);
	device_unlock(dev);
	return n;
}

/*
 *  This is here only to be able to know if something in userspace
 *  tries to write to dtr_in
 *  Its only function is to complain if it is being called
 */
static ssize_t notify_attempted_dtr_in(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
//	struct quectel_power_manager *data = dev_get_drvdata(dev);
    pr_err("[QTL_PM]: %s, buffer: %s \n", __func__, buf);    
	return n;
}
static ssize_t store_sleep_polarity(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
//	struct quectel_power_manager *data = dev_get_drvdata(dev);
    pr_err("[QTL_PM]: %s, buffer: %s \n", __func__, buf);    
	return n;
}

static ssize_t store_wakeup_in_polarity(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
//	struct quectel_power_manager *data = dev_get_drvdata(dev);
    pr_err("[QTL_PM]: %s, buffer: %s \n", __func__, buf);    
	return n;
}

static ssize_t store_wakeup_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
//	struct quectel_power_manager *data = dev_get_drvdata(dev);
    pr_err("[QTL_PM]: %s, buffer: %s \n", __func__, buf);    
	return n;
}

// Notify SysFS for changes
static void quectel_sysfs_notify_work(struct work_struct *work) 
{
    struct quectel_power_manager *data = container_of(work, struct quectel_power_manager, sysfs_notify_work.work);
    int curgpio = gpio_get_value(25);
    printk("[QTL_PM] Curr gpio state: %i \n", curgpio);
    if (curgpio) {
        atomic_set(&data->notify_wakeup_in, 0);
        if (data->wakeup_flag)
        {
	        printk("[QTL_PM]: sysfs_notify wakeup_in from gpio\n");
            sysfs_notify(&data->pdev->dev.kobj, NULL, "wakeup_in");
        }
    }
    if (atomic_read(&data->notify_wakeup_in)) 
    {
        atomic_set(&data->notify_wakeup_in, 0);
        if (data->wakeup_flag)
        {
	        printk("[QTL_PM]: sysfs_notify wakeup_in\n");
            sysfs_notify(&data->pdev->dev.kobj, NULL, "wakeup_in");
        }
    }
}

static void quectel_sysfs_notify_dtrin_work(struct work_struct *work) 
{
    struct quectel_power_manager *data = container_of(work, struct quectel_power_manager, sysfs_notify_work.work);
    int curgpio = gpio_get_value(5);
    printk("[QTL_PM] DTR Curr gpio state: %i \n", curgpio);

    if (curgpio) {
        atomic_set(&data->notify_dtr_in, 0);
        if (data->dtr_flag)  {
	        printk("[QTL_PM]: sysfs_notify wakeup_in from gpio\n");
            sysfs_notify(&data->pdev->dev.kobj, NULL, "wakeup_in");
        }
    }
    printk("[QTL_PM]: sysfs_notify dtr_in\n");

    if (atomic_read(&data->notify_dtr_in)) {
        atomic_set(&data->notify_dtr_in, 0);
        if (data->wakeup_flag)
        {
	        printk("[QTL_PM]: sysfs_notify dtr_in\n");
            sysfs_notify(&data->pdev->dev.kobj, NULL, "dtrin");
        }
    } else {
        printk("[QTL_PM] DTRIN nothing to do \n");
    }
}

static irq_handler_t wakeup_irqhandler(unsigned int irq, void *dev_id, 
                                        struct pt_regs *regs) 
{
   int curgpio = gpio_get_value(25);
   pr_err("[QTL_PM] IRQ: WAKEUP_IN gpio changed %i \n", curgpio);
   return (irq_handler_t) IRQ_HANDLED;
}

static irq_handler_t dtr_irqhandler(unsigned int irq, void *dev_id, 
                                    struct pt_regs *regs) 
{
   int curgpio = gpio_get_value(5);
   pr_err("[QTL_PM] IRQ: DTR_IN gpio changed %i \n", curgpio);
   return (irq_handler_t) IRQ_HANDLED;
}

/* Exposed attributes in sysfs */
static DEVICE_ATTR(wakeup_source, 0644, show_wakeup_source, store_wakeup_source);
static DEVICE_ATTR(wakeup_in, 0444, show_wakeup_status, NULL);
static DEVICE_ATTR(dtr_in, 0644, show_dtr_status, notify_attempted_dtr_in);

static DEVICE_ATTR(sleep_polarity, 0644, show_sleep_polarity, store_sleep_polarity);
static DEVICE_ATTR(wakeup_in_polarity, 0644, show_wakeup_in_polarity, store_wakeup_in_polarity);
static DEVICE_ATTR(wakeup_enable, 0644, show_wakeup_enable, store_wakeup_enable);


/*
 * struct quectel_gpio_group: The GPIO itself (number, name, enabled flag)
 * quectel_power_manager_of_data: The array of GPIO structs
 */

static int quectel_power_manager_probe(struct platform_device *pdev) {
    struct quectel_power_manager *data = NULL;
    struct quectel_power_manager_pdata *pdata =NULL;
    struct device *dev = &(pdev->dev);
   	struct device_node *np = dev->of_node;
    struct quectel_power_manager_of_data *pin_data;
    int i = 0;
    int result = 0;
    int count = 0;
    pr_err("[QTL_PM]: %s: start \n", __func__);
    pdata = devm_kzalloc(dev, sizeof(struct quectel_power_manager_pdata), GFP_KERNEL);
    if (pdata == NULL) {
        dev_err(dev, "[QTL_PM] pdata is null, bailing out\n");
        return -ENOMEM;
    }

    pdata->of_data = NULL;

    data = devm_kzalloc(dev, sizeof(struct quectel_power_manager), GFP_KERNEL);
    if (data == NULL) {
        dev_err(dev, "[QTL_PM] Couldn't allocate data\n");
        return -ENOMEM;
    }

    count = of_gpio_count(np);
   	pin_data = devm_kzalloc(dev, sizeof(*pin_data), GFP_KERNEL);
	if (!pin_data) {
		dev_err(dev, "[QTL_PM] Not enough memory\n");
        return -ENOMEM;
	}
    if (count > 0) {
		pin_data->gpio = devm_kzalloc(dev, count * sizeof(struct quectel_gpio_group), GFP_KERNEL);
    }
    for (i = 0; i < count; i++) {
		const char *name = NULL;
        pin_data->gpio[i].no = of_get_gpio(np, i);
        of_property_read_string_index(np,"quec,gpio-names", i, &name);
        pin_data->gpio[i].name = name;
        pr_info("[QTL_PM] GPIO %i is %s \n", pin_data->gpio[i].no, name);
        gpio_request(pin_data->gpio[i].no, pin_data->gpio[i].name);
        gpio_direction_input(pin_data->gpio[i].no);
    }
    pdata->of_data = pin_data;
   
    INIT_DELAYED_WORK(&data->sysfs_notify_work, quectel_sysfs_notify_work);
    INIT_DELAYED_WORK(&data->sysfs_dtr_notify_work, quectel_sysfs_notify_dtrin_work);

    atomic_set(&data->notify_wakeup_in, 0);
    atomic_set(&data->notify_dtr_in, 0);

    wake_lock_init(&data->wlock_wakeup, WAKE_LOCK_SUSPEND, "quectel_wakeup");
    data->pdev = pdev;
    device_init_wakeup(&pdev->dev, 1);
    data->pdata = pdata;
    platform_set_drvdata(pdev, data);
    data->wakeup_flag = 0;
    data->dtr_flag = 0;

    /* Setup IRQ handlers */
    data->irq_wakeup = gpio_to_irq(pin_data->gpio[1].no); // get second gpio (wakeup_in)
    result = request_irq(data->irq_wakeup,
                       (irq_handler_t) wakeup_irqhandler,
                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 
                       "wakeup_irqhandler", dev);
    if (result) {
        pr_err("[QTL_PM] %s: Failed to setup %s irq \n", __func__, pin_data->gpio[1].name);
    }
    data->irq_dtr = gpio_to_irq(pin_data->gpio[0].no); // get first gpio (dtr_in from the dts)
    result = request_irq(data->irq_dtr,
                       (irq_handler_t) dtr_irqhandler,
                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                       "dtr_irqhandler", 
                       dev); 
    if (result) {
        pr_err("[QTL_PM] %s: Failed to setup %s irq \n", __func__, pin_data->gpio[0].name);
    }
    if (device_create_file(&pdev->dev, &dev_attr_wakeup_in) < 0) {
        pr_err("[QTL_PM] %s: Failed to create wakeup_in file\n", __func__);
    }

    if (device_create_file(&pdev->dev, &dev_attr_wakeup_source) < 0) {
        pr_err("[QTL_PM] %s: Failed to create wakeup_source file\n", __func__);
    }    
    
    if (device_create_file(&pdev->dev, &dev_attr_dtr_in) < 0) {
        pr_err("[QTL_PM] %s: Failed to create dtr_in file\n", __func__);
    }     
    if (device_create_file(&pdev->dev, &dev_attr_sleep_polarity) < 0) {
        pr_err("[QTL_PM] %s: Failed to create dtr_in file\n", __func__);
    }     
    if (device_create_file(&pdev->dev, &dev_attr_wakeup_in_polarity) < 0) {
        pr_err("[QTL_PM] %s: Failed to create dtr_in file\n", __func__);
    }     
    if (device_create_file(&pdev->dev, &dev_attr_wakeup_enable) < 0) {
        pr_err("[QTL_PM] %s: Failed to create dtr_in file\n", __func__);
    }    

    pr_err("[QTL_PM]: %s: finished normally \n", __func__);
    return 0;
}

static int quectel_power_manager_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	device_remove_file(&pdev->dev, &dev_attr_wakeup_in);
	device_remove_file(&pdev->dev, &dev_attr_dtr_in);
	pr_info("[QTL_PM]: %s \n", __func__);
	return 0;
}

static int quectel_power_manager_suspend(struct platform_device *pdev, 
                                            pm_message_t state)
{
	struct quectel_power_manager *data = platform_get_drvdata(pdev);
	data->wakeup_flag = 1;
    pr_info("[QTL_PM]: %s \n", __func__);
	return 0;
}

static int quectel_power_manager_resume(struct platform_device *pdev)
{
	struct quectel_power_manager *data = platform_get_drvdata(pdev);
	atomic_set(&data->notify_wakeup_in, 1);
	data->wakeup_flag = 0;    
    sysfs_notify(&data->pdev->dev.kobj, NULL, "wakeup_in");
    pr_info("[QTL_PM]: %s \n", __func__);
	return 0;
}

static const struct of_device_id quec_power_manager_match[] = 
{
	{
        .compatible = "quec,quectel-power-manager",
    },
	{},
};

static struct platform_driver quectel_power_manager_driver = 
{
	.probe		= quectel_power_manager_probe,
	.remove		= quectel_power_manager_remove,
	.suspend	= quectel_power_manager_suspend,
	.resume		= quectel_power_manager_resume,
	.driver		= {
		.name = MSM9615_QUECTEL_POWER_MANAGER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = quec_power_manager_match,
	},
};

module_platform_driver(quectel_power_manager_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("quectel sleep driver");
MODULE_VERSION("1.1");
