/* 
 * Copyright (c) 2018-2019, The Quectel Wireless Solutions Co., Ltd.
 *
 * Authors
 *  running.qian    running.qian@quectel.com
 *
 * Fixs:
 *  2018-09-12      running     init
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/sysfs.h>


ssize_t about_show(struct class *class, struct class_attribute *attr, char *buf)
{
    pr_info("class_name:%s, classattr_name:%s.\n", class->name, attr->attr.name);
    
    sprintf(buf,"%s\n","This class is used to view quectel's security features");
    return strlen(buf) + 1;
}

CLASS_ATTR_RO(about);


struct class_attribute def_class_attrs[] = {
        __ATTR_RO(about),
        __ATTR_NULL,
};


static struct class *ql_security_class;


int ql_security_class_create_file(struct class_attribute *class_attr)
{
    return class_create_file(ql_security_class, class_attr);
}

EXPORT_SYMBOL_GPL(ql_security_class_create_file);


void ql_security_class_remove_file(struct class_attribute *class_attr)
{
     class_remove_file(ql_security_class, class_attr);
}

EXPORT_SYMBOL_GPL(ql_security_class_remove_file);


static int __init ql_security_init(void)
{
    int retval;

    ql_security_class = kzalloc(sizeof(*ql_security_class), GFP_KERNEL);
    if (!ql_security_class) {
        retval = -ENOMEM;
        goto error;
    }

    ql_security_class->name = "ql_securtiy";
    ql_security_class->owner = THIS_MODULE;
    ql_security_class->class_attrs = def_class_attrs;

    retval = class_register(ql_security_class);
    if (retval)
        goto error;

    pr_info("class_register ql_security_class ok!\n");
    return 0;

error:
    kfree(ql_security_class);
    return -EPERM;
}

static void __exit ql_security_exit(void)
{
    class_unregister(ql_security_class);
    pr_info("class_unregister ql_security_class ok!\n");
}


module_init(ql_security_init);
module_exit(ql_security_exit);

MODULE_DESCRIPTION("Quectel Security Features");
MODULE_LICENSE("GPL v2");

