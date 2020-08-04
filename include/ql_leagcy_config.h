/**
 * @file        quectel_leagcy_config.h
 * @brief       Quectel Leagcy MACROs Define.
 * @author      Running.qian
 * @copyright   Copyright (c) 2017-2020 @ Quectel Wireless Solutions Co., Ltd.
 */

#ifndef QUECTEL_LEAGCY_CONFIG_H
#define QUECTEL_LEAGCY_CONFIG_H


#if defined (__QUECTEL_PROJECT_EC20CE_CT__)
    #define QUECTEL_CTCC_SUPPORT
    #define QUECTEL_CTCC_PRIVATE
#endif


#ifdef QUECTEL_FEATURE_OPENLINUX
    #define QUECTEL_OPEN_LINUX_SUPPORT       1
#else
    #define QUECTEL_OPEN_LINUX_SUPPORT       0
#endif


/*** OMA-DM Features Macro Area ***/
//#define QUECTEL_OMA_DM 1
#ifdef QUECTEL_OMA_DM
#if defined(__QUECTEL_PROJECT_EC21V__) || defined(__QUECTEL_PROJECT_EC21NA__) || defined(__QUECTEL_PROJECT_EC25NA__)
#define QUECTEL_ODM_UPDATE_AUTO
#endif

#define QUECTEL_ODM_UPDATE_AUTOREPORT     0
#endif//QUECTEL_OMA_DM


#define QUECTEL_AT_Q_VERSION //2017.12.04 add command AT+QVERSION by wayne.wei

#define QUECTEL_AUTO_SCRIPT_FEATURE //maxcodeflag20170318 for exctuing AP's script automatically

#define QUECTEL_DBG_FUNC //maxcodeflag20170524 for enable quectel kmsg functions
    
#define QUECTEL_THERMAL //maxcodeflag20172106

#endif //ifdef QUECTEL_LEAGCY_CONFIG_H
