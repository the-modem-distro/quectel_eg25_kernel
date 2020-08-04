/* Copyright (c) 2010-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and * only version 2 as published by the Free Software Foundation.  *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Quectel ql_lpm driver
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

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/netlink.h>
#include <linux/socket.h>

#include <linux/skbuff.h>
#include <net/sock.h>

//#define QL_LPM_NETLINK	26
#define QL_LPM_NETLINK 28 //changed by javen:2018/10/23:avoiding conflict and consistency
#define MAX_MSGSIZE	24

static int wakeup_in_irq = 0;
static struct sock *nl_sk = NULL;
static int user_pid = -1;
extern struct net init_net;

/*The below number is gpio number on Qcom BaseBand.*/
static int wakeup_in = 75;
module_param(wakeup_in, int, S_IRUGO);
MODULE_PARM_DESC(wakeup_in, "A pin(input) use to wakeup 4G module.");

/*The below number is gpio number on Qcom BaseBand.*/
static int wakeup_out = 24;
module_param(wakeup_out, int, S_IRUGO);
MODULE_PARM_DESC(wakeup_out, "A pin(output) use to wakeup MCU.");

/*The below number, 1 means the wakeupout pin will output high when wakeup module.*/
static int wakeup_out_edge = 1;
module_param(wakeup_out_edge, int, S_IRUGO);
MODULE_PARM_DESC(wakeup_out_edge, "The edge of wakeup_out pin(output) to wakeup mcu, rising by default");

#if 0	/*No need now, maybe useful in the future.*/
static int wakeup_in_edge = 1;
module_param(wakeup_in_edge, int, S_IRUGO);
MODULE_PARM_DESC(wakeup_in_edge, "The edge of wakeup_in pin(input) to wakeup 4G module, rising by default");
#endif

int lpm_send_state(char* msg, int len)
{
    struct sk_buff *skb;
    struct nlmsghdr *nlh;

    int ret;

    skb = nlmsg_new(len, GFP_ATOMIC);
    if(!skb)
    {
        pr_err("[ql_lpm]:netlink alloc failure\n");
        return -1;
    }

    nlh = nlmsg_put(skb, 0, 0, QL_LPM_NETLINK, len, 0);
    if(nlh == NULL)
    {
        pr_err("[ql_lpm]:nlmsg_put failaure \n");
        nlmsg_free(skb);
        return -1;
    }
 
    memcpy(nlmsg_data(nlh), msg, len);
    if(user_pid == -1)
	return -1;
    ret = netlink_unicast(nl_sk, skb, user_pid, MSG_DONTWAIT);
    pr_debug("[ql_lpm]: Send wakeupin state to user space, ret: %d\n", ret);

    return ret;
}

static irqreturn_t quectel_wakeup_irq_func(int irq, void *id)
{
	//TODO: debounce
	int value = gpio_get_value(wakeup_in);

	if(value == 0)
		lpm_send_state("falling", strlen("falling"));
	else if(value == 1)
		lpm_send_state("rising", strlen("rising"));

	return IRQ_HANDLED;
}

static int wakeup_in_init(void)
{
	int err = 0;

	if(wakeup_in == -1)
	{
		pr_err("[ql_lpm][%s]: forgot to assign wakeup_out pin when insmod this kmod\n", __FUNCTION__);
		return -1;
	}
	
	err = gpio_request(wakeup_in, "wakeup_in");
	if (err < 0)
	{
		pr_err("[ql_lpm][%s]: request wakeup_in: %d failed, error: %d\n", __FUNCTION__, wakeup_in, err);
		goto err_gpio_request;
	}
	
	err = gpio_direction_input(wakeup_in);
	if (err < 0)
	{
		pr_err("[ql_lpm][%s]: set wakeup_in:  direction input (%d) failed: %d\n", __FUNCTION__, wakeup_in, err);
		goto err_gpio_to_irq;
	}

	err = gpio_to_irq(wakeup_in);
	if (err < 0)
	{
		pr_err("[ql_lpm][%s]: wakeup_in: %d to irq failed, err: %d\n", __FUNCTION__, wakeup_in, err);
		goto err_gpio_to_irq;
	}
	
	wakeup_in_irq = err;
    
	err = request_any_context_irq(wakeup_in_irq, quectel_wakeup_irq_func,	\
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "wakeup_in_irq", NULL);
	if (err < 0)
	{
		pr_err("[ql_lpm][%s]: Can't request %d IRQ for wakeup_in: %d\n", __FUNCTION__, wakeup_in_irq, err);
		goto err_free_irq;
	}

	//wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "quectel_wakelock");

	return 0;

err_free_irq:
	free_irq(wakeup_in_irq, NULL);
err_gpio_to_irq:
	gpio_free(wakeup_in);
err_gpio_request:
	return err;
}

static void wakeup_in_exit(void)
{	
	free_irq(wakeup_in_irq, NULL);
	disable_irq(wakeup_in_irq);

	gpio_free(wakeup_in);
}

static void netlink_rcv_msg(struct sk_buff *__skb)
{

     struct sk_buff *skb;
     struct nlmsghdr *nlh;
     pr_debug("[ql_lpm]: user data come in\n");
     skb = skb_get (__skb);
     if(skb->len >= NLMSG_SPACE(0))
     {
         nlh = nlmsg_hdr(skb);
         user_pid = nlh->nlmsg_pid;
         pr_info("[ql_lpm]: user ID:%d, messages: %s\n",user_pid, (char *)NLMSG_DATA(nlh));
         kfree_skb(skb);
    }
}

static int quectel_low_consume_suspend(struct platform_device *pdev)
{
	pr_debug("[ql_lpm][%s]\n", __FUNCTION__);

	/* enable wakeup_in wakeup function */
	if (enable_irq_wake(wakeup_in_irq) < 0)
	{
		pr_err("[ql_lpm][%s]: enable wakeup_in wakeup function failed\n", __FUNCTION__);
		return -1;
	}
	
	/* Set wakeup_out to output level, 4G module enter sleep mode, and notify mcu */
	if(wakeup_out_edge == 1)
	{
		pr_info("[ql_lpm][%s]: output low\n", __FUNCTION__);
		gpio_direction_output(wakeup_out, 0);
	}
	else if(wakeup_out_edge == 0)
	{
		pr_info("[ql_lpm][%s]: output high\n", __FUNCTION__);
		gpio_direction_output(wakeup_out, 1);
	}
	
	return 0;
}

static int quectel_low_consume_resume(struct platform_device *pdev)
{
	pr_debug("[ql_lpm][%s]\n", __FUNCTION__);

	/* disable wakeup_in wakeup function */
	if (disable_irq_wake(wakeup_in_irq) < 0)
	{
		pr_err("[ql_lpm][%s]: disable wakeup_in wakeup function failed\n", __FUNCTION__);
		return -1;
	}
	
	/* Set wakeup_out to output high level, 4G module enter active mode, and notify mcu */
	//gpio_direction_output(wakeup_out, 1);	//Here, I think should be control in user application.

	return 0;
}

struct netlink_kernel_cfg cfg = { 
	.input  = netlink_rcv_msg, /* set recv callback */
};  

static int quectel_low_consume_probe(struct platform_device *pdev)
{
		int ret = 0;
		if((ret = wakeup_in_init()) < 0)
		{
			pr_err("[ql_lpm][%s]: wakeup_in init failed\n", __FUNCTION__);
			goto err_wakeup_in_exit;
		}

		nl_sk = netlink_kernel_create(&init_net, QL_LPM_NETLINK, &cfg);
                if(!nl_sk){
                    pr_err("[ql_lpm]netlink: create netlink socket error.\n");
                    return -1;
                }
                pr_debug("[ql_lpm]: create netlink socket ok.\n");
		pr_debug("[ql_lpm]: wakeup_in pin: %d, wakeup_out pin: %d\n", wakeup_in, wakeup_out);
	
		pr_info("[ql_lpm][%s]: module probe successfully\n", __FUNCTION__);
		return 0;

err_wakeup_in_exit:
	wakeup_in_exit();

	return ret;
}

static int quectel_low_consume_remove(struct platform_device *pdev)
{
	wakeup_in_exit();

        if(nl_sk != NULL){
            sock_release(nl_sk->sk_socket);
        }
	pr_info("[ql_lpm][%s]: module exit.\n", __FUNCTION__);

	return 0;
}

static const struct of_device_id quectel_low_consume_match[] =
{
	{.compatible = "quec,ql_lpm",},
	{},
};

static struct platform_driver quectel_low_consume_driver = {
	.probe		= quectel_low_consume_probe,
	.remove		= quectel_low_consume_remove,
	.suspend	= quectel_low_consume_suspend,
	.resume		= quectel_low_consume_resume,
	.driver		= {
		.name = "ql_lpm",
		.owner = THIS_MODULE,
		.of_match_table = quectel_low_consume_match,
	},
};

module_platform_driver(quectel_low_consume_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ql lpm");
MODULE_VERSION("1.0");
