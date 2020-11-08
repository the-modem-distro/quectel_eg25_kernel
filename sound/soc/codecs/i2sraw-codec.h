/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __I2SRAW_H__
#define __I2SRAW_H__

#define DEVICE_NAME    "i2sraw-codec"
#define I2SRAW_RATES SNDRV_PCM_RATE_8000_48000

#define I2SRAW_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)
    
#endif /* __I2SRAW_H__ */

