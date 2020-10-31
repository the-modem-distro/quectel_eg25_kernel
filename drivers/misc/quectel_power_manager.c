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

#define WAKEUP_FILE "/sys/devices/soc:quec,quectel-power-manager/wakeup_in"
static DEFINE_MUTEX(wakeup_lock);

/* SHOW functions: these report data to sysfs when requested */
static ssize_t quectel_dtr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    ssize_t ret;
    printk("%s: %i", __func__, data->dtr_flag);
	mutex_lock(&wakeup_lock);
    if(data->dtr_flag)   {
        ret =  snprintf(buf, PAGE_SIZE, "%s\n", "1");
    } else  {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "0");
    }
	mutex_unlock(&wakeup_lock);
	return ret;
}

static ssize_t quectel_dtr_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    ssize_t ret;
    printk("%s: %i", __func__, data->irq_dtr);
	mutex_lock(&wakeup_lock);
    if(data->irq_dtr)  {
        ret =  snprintf(buf, PAGE_SIZE, "%s\n", "1");
    } else  {
        ret =  snprintf(buf, PAGE_SIZE, "%s\n", "0");
    }
	mutex_unlock(&wakeup_lock);
	return ret;
}

static ssize_t quectel_wakeup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    ssize_t ret;

	mutex_lock(&wakeup_lock);

    if(data->wakeup_flag)
    {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "Sleep");
    }
    else
    {
        ret = snprintf(buf, PAGE_SIZE, "%s\n", "Wakeup");
    }
	wake_unlock(&data->wlock_wakeup);

	mutex_unlock(&wakeup_lock);
	
	return ret;//must return none-zero value or will call poll continuesly
}

// This comes from the gpio patch from quectel
extern char g_wakeup_src[];
static ssize_t quectel_wakeup_source_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    ssize_t ret;
	struct quectel_power_manager *data = dev_get_drvdata(dev);

	printk("[QTL_PM]: wakeup source: %s \n", g_wakeup_src);
	mutex_lock(&wakeup_lock);
    ret = snprintf(buf, PAGE_SIZE, "%s\n", g_wakeup_src);
	wake_unlock(&data->wlock_wakeup);

	mutex_unlock(&wakeup_lock);
	
	return ret;//must return none-zero value or will call poll continuously
}

// Store the wakeup source
static ssize_t quectel_wakeup_source_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
    ssize_t len = n;
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    printk("[QTL_PM]: %s \n", __func__);
	device_lock(dev);
	if(len > 32)
	{
	    len = 32;
	}
	memcpy(data->wakeup_source, buf, len);
	device_unlock(dev);
    
	return n;
}// Store the dtr source
static ssize_t quectel_dtr_in_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
    ssize_t len = n;
	struct quectel_power_manager *data = dev_get_drvdata(dev);
    printk("[QTL_PM]: %s \n", __func__);
/*	device_lock(dev);
	if(len > 32)
	{
	    len = 32;
	}
	memcpy(data->wakeup_source, buf, len);
	device_unlock(dev);*/
    
	return n;
}

// Notify SysFS for changes
static void quectel_sysfs_notify_work(struct work_struct *work) 
{
    struct quectel_power_manager *data = container_of(work, struct quectel_power_manager, sysfs_notify_work.work);
    int curgpio = gpio_get_value(25);
    printk("[QTL_PM] Curr gpio state: %i", curgpio);
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
    printk("[QTL_PM] DTR Curr gpio state: %i", curgpio);

    if (curgpio) {
        atomic_set(&data->notify_dtr_in, 0);
        if (data->dtr_flag)
        {
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

static irq_handler_t wakeup_irqhandler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
	struct device *data = &dev_id;
    printk("[QTL_PM] Wakeup_in IRQ handler called \n");
    int curgpio = gpio_get_value(25);
    printk("[QTL_PM] DTR Curr gpio state: %i", curgpio);

   printk("[QTL_PM] Wakeup_in IRQ handler called END\n");
   return (irq_handler_t) IRQ_HANDLED;
}

static irq_handler_t dtr_irqhandler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
   struct device *data = &dev_id;
   printk("[QTL_PM] DTR In IRQ handler called \n");
    int curgpio = gpio_get_value(5);
    printk("[QTL_PM] DTR Curr gpio state: %i", curgpio);
   printk("[QTL_PM] DTR In IRQ handler called end \n");
   return (irq_handler_t) IRQ_HANDLED;
}

/* Put all attributes toghether, why the fuck is everything spread everywhere? */
static DEVICE_ATTR(wakeup_source, 0644, quectel_wakeup_source_show, quectel_wakeup_source_store);
static DEVICE_ATTR(wakeup_in, 0444, quectel_wakeup_show, NULL);
static DEVICE_ATTR(dtr_in, 0644, quectel_dtr_show, quectel_dtr_in_store);
static DEVICE_ATTR(dtr_state, 0444, quectel_dtr_state_show, NULL);

static int quectel_power_manager_probe(struct platform_device *pdev)
{
    struct quectel_power_manager *data = NULL;
    struct quectel_power_manager_pdata *pdata =NULL;
    struct device *dev = &(pdev->dev);
   	struct device_node *np = dev->of_node;
    struct quectel_power_manager_of_data *pin_data;
    int i = 0;
    int ret = 0;
    int result = 0;
       //quectel_gpio_group <-- the gpio struct itself
       // quectel_power_manager_of_data <-- OF_DATA -> gpio array
    int count = 0;
    printk("[QTL_PM]: %s start \n", __func__);
    pdata = devm_kzalloc(dev, sizeof(struct quectel_power_manager_pdata), GFP_KERNEL);
    if (pdata == NULL) {
        return -ENOMEM;
    }

    pdata->of_data = NULL;

    data = devm_kzalloc(dev, sizeof(struct quectel_power_manager), GFP_KERNEL);
    if (data == NULL) {
        return -ENOMEM;
    }

    count = of_gpio_count(np);
    printk("[QTL_PM] %s: GPIO count is %i", __func__, count);
   	pin_data = devm_kzalloc(dev, sizeof(*pin_data), GFP_KERNEL);
	if (!pin_data) {
		dev_err(dev, "[QTL_PM] Not enough memory\n");
		ret = -ENOMEM;
		goto out;
	}
    if (count > 0) {
		pin_data->gpio = devm_kzalloc(dev, count * sizeof(struct quectel_gpio_group), GFP_KERNEL);
    }
    for (i = 0; i < count; i++) {
		const char *name = NULL;
			pin_data->gpio[i].no = of_get_gpio(np, i);
			of_property_read_string_index(np,"quec,gpio-names", i, &name);
            printk("[QTL_PM] GPIO name for %i is %s \n", pin_data->gpio[i].no, name);
			pin_data->gpio[i].name = name;
            gpio_request(pin_data->gpio[i].no, pin_data->gpio[i].name);
            gpio_direction_input(pin_data->gpio[i].no);           // Set GPIO as input
         //   gpio_set_debounce(pin_data->gpio[i].no, 50);          // Set a 50ms debounce, adjust to your needs
         //   gpio_export(pin_data->gpio[i].no, false);  
            printk("[QTL_PM] Set GPIO %s done \n", pin_data->gpio[i].name);
    }
    pdata->of_data = pin_data;
   

    INIT_DELAYED_WORK(&data->sysfs_notify_work, quectel_sysfs_notify_work);
    INIT_DELAYED_WORK(&data->sysfs_dtr_notify_work, quectel_sysfs_notify_dtrin_work);

    atomic_set(&data->notify_wakeup_in, 0);
    atomic_set(&data->notify_dtr_in, 0);
    atomic_set(&data->notify_dtr_state, 0);

    wake_lock_init(&data->wlock_wakeup, WAKE_LOCK_SUSPEND, "quectel_wakeup");

    data->pdev = pdev;
    device_init_wakeup(&pdev->dev, 1);
    data->pdata = pdata;
    platform_set_drvdata(pdev, data);
    data->wakeup_flag = 0;
    data->dtr_flag = 0;

 /* Setup IRQ handlers */
    data->irq_wakeup = gpio_to_irq(pin_data->gpio[1].no); // get second gpio (wakeup_in)
    result = request_irq(data->irq_wakeup,           // requested interrupt
                       (irq_handler_t) wakeup_irqhandler, // pointer to handler function
                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, // interrupt mode flag
                       "wakeup_irqhandler",        // used in /proc/interrupts
                       dev);               // the *dev_id shared interrupt lines, NULL is okay

    data->irq_dtr = gpio_to_irq(pin_data->gpio[0].no); // get second gpio (wakeup_in)
    result = request_irq(data->irq_dtr,           // requested interrupt
                       (irq_handler_t) dtr_irqhandler, // pointer to handler function
                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, // interrupt mode flag
                       "dtr_irqhandler",        // used in /proc/interrupts
                       dev);               // the *dev_id shared interrupt lines, NULL is okay

    if (device_create_file(&pdev->dev, &dev_attr_wakeup_in) < 0) {
        printk("[QTL_PM] %s: dev file creation for wakeup_in failed\n", __func__);
    }

    if (device_create_file(&pdev->dev, &dev_attr_wakeup_source) < 0) {
        printk("[QTL_PM] %s: dev file creation for wakeup_source failed\n", __func__);
    }    
    
    if (device_create_file(&pdev->dev, &dev_attr_dtr_in) < 0) {
        printk("[QTL_PM] %s: dev file creation for dtr_in failed\n", __func__);
    }    

    if (device_create_file(&pdev->dev, &dev_attr_dtr_state) < 0)  {
        printk("[QTL_PM] %s: dev file creation for dtr_state failed\n", __func__);
    }
    printk("[QTL_PM]: %s: ready \n", __func__);
    out:
    return 0;

/* Why is this even here if it is unreachable
err_wakeup_init:
    platform_set_drvdata(pdev, NULL);
    return -1;
    */
}

static int quectel_power_manager_remove(struct platform_device *pdev)
{
	//struct quectel_power_manager *data = platform_get_drvdata(pdev);
	printk("[QTL_PM]: %s \n", __func__);
	platform_set_drvdata(pdev, NULL);
	device_remove_file(&pdev->dev, &dev_attr_wakeup_in);
	device_remove_file(&pdev->dev, &dev_attr_dtr_in);
	device_remove_file(&pdev->dev, &dev_attr_dtr_state);
	
	return 0;
}

static int quectel_power_manager_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct quectel_power_manager *data = platform_get_drvdata(pdev);
	data->wakeup_flag = 1;
    printk("[QTL_PM]: %s \n", __func__);

	return 0;
}

static int quectel_power_manager_resume(struct platform_device *pdev)
{
	struct quectel_power_manager *data = platform_get_drvdata(pdev);
    printk("[QTL_PM]: %s  Sysfs_notify wakeup_in \n", __func__);
	atomic_set(&data->notify_wakeup_in, 1);
    
	data->wakeup_flag = 0;    
    sysfs_notify(&data->pdev->dev.kobj, NULL, "wakeup_in");
	
	return 0;
}

static const struct of_device_id quec_power_manager_match[] = 
{
	{.compatible = "quec,quectel-power-manager",},
	{},
};

static struct platform_driver quectel_power_manager_driver = {
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
MODULE_VERSION("1.0");
