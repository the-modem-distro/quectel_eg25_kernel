/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <linux/i2c.h>

#include "i2sraw-codec.h"

static int i2sraw_probe(struct snd_soc_codec *codec)
{
	printk("I2S raw dummy codec: probed\n");
	return 0;
}

static int i2sraw_remove(struct snd_soc_codec *codec)
{
	printk("I2S raw dummy codec: removed\n");
	return 0;
}

static struct snd_soc_dai_driver i2sraw_dais[] = {
	{
		.name = "i2sraw-codec-rx",
		.playback = { /* Support maximum range */
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = I2SRAW_RATES,
			.formats = I2SRAW_FORMATS,
		},
	},
	{
		.name = "i2sraw-codec-tx",
		.capture = { /* Support maximum range */
			.stream_name = "Record",
			.channels_min = 1,
			.channels_max = 4,
			.rates = I2SRAW_RATES,
			.formats = I2SRAW_FORMATS,
		},
	},
	//		.ops = &wm8960_dai_ops,
//	.symmetric_rates = 1,
};


static struct snd_soc_codec_driver soc_i2sraw = {
	.probe = i2sraw_probe,
	.remove = i2sraw_remove,
};

static const struct i2c_device_id i2sraw_i2c_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2sraw_i2c_id);

static int i2sraw_i2c_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	int ret;
	printk("I2S raw dummy codec: Registering codec... ");

	ret = snd_soc_register_codec(&i2c->dev, &soc_i2sraw,
			i2sraw_dais, ARRAY_SIZE(i2sraw_dais));
	
	if (ret == 0) {
		printk(" Success! \n");
	} else {
		printk(" Failed! (%i) \n", ret);

	}

	return ret;
}

static int i2sraw_i2c_remove(struct i2c_client *client)
{
	printk("[I2SR] Unregister driver \n");
	snd_soc_unregister_codec(&client->dev);
//	kfree(i2c_get_clientdata(client));
	return 0;
}

static void i2sraw_i2c_shutdown(struct i2c_client *client)
{
	printk("%s\n",__func__);
}

static const struct of_device_id i2sraw_id[] = {
	{ .compatible = "qcom,i2sraw-codec", },
	{ }
};
MODULE_DEVICE_TABLE(of, i2sraw_id);

struct i2c_driver i2sraw_i2c_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(i2sraw_id),
	},
	.probe = i2sraw_i2c_probe,
	.remove   = i2sraw_i2c_remove,
	.shutdown = i2sraw_i2c_shutdown,
	.id_table = i2sraw_i2c_id,
};

static int i2sraw_modinit(void)
{
	int ret = 0;
	printk("I2S raw dummy codec: Adding driver \n");
	ret = i2c_add_driver(&i2sraw_i2c_driver);
	if (ret != 0) {
		printk("[I2SR]: Error registering codec: %i", ret);
	}
	return ret;
}

module_init(i2sraw_modinit);

static void i2sraw_modexit(void)
{
	i2c_del_driver(&i2sraw_i2c_driver);
}
module_exit(i2sraw_modexit);

MODULE_DESCRIPTION("Dummy I2S raw codec");
MODULE_AUTHOR("biktorgj");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
