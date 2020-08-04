/**
 * @file        quectel_CUSTOM_config.h
 * @brief       Quectel CUSTOM MACROs Define.
 * @author      Running.qian
 * @copyright   Copyright (c) 2017-2020 @ Quectel Wireless Solutions Co., Ltd.
 */

#ifndef QL_CUSTOM_CONFIG_H
#define QL_CUSTOM_CONFIG_H

/******************  CUSTOME PRODUCT MACRO, Priority Level: High   ***********************************///
//#if QL_CUSTOM_INST
/******************* ReDefine FUNCS MACRO  ********************************/
//#undef    QL_FUNC_XXX
//#define   QL_FUNC_XXX     1

/******************* SPECIFIC FUNCS MACRO  ********************************/
//#define   QL_FUNC_YYY
//#endif

#if defined(QL_G_CUSTOM_STD)//Standard modulue 
/******************* ReDefine FUNCS MACRO  ********************************/

/******************* SPECIFIC FUNCS MACRO  ********************************/
#elif defined(QL_G_CUSTOM_MLABS)


#elif defined(QL_G_CUSTOM_INTEST) /* Intest Project INT */
    #ifdef FEATURE_QUECTEL_NAT
    #undef FEATURE_QUECTEL_NAT
    #endif
    #define FEATURE_QUECTEL_NAT 1
    
    #ifdef FEATURE_QUECTEL_IPTABLES
    #undef FEATURE_QUECTEL_IPTABLES
    #endif
    #define FEATURE_QUECTEL_IPTABLES 1
    
    #ifdef FEATURE_QUECTEL_ROUTE
    #undef FEATURE_QUECTEL_ROUTE
    #endif
    #define FEATURE_QUECTEL_ROUTE 1

    #ifdef FEATURE_QUECTEL_DNS
    #undef FEATURE_QUECTEL_DNS
    #endif
    #define FEATURE_QUECTEL_DNS 1
#else
    #error "v_v v_v v_v Undefine Cust Name v_v v_v v_v"
#endif

#endif //#ifndef QL_CUSTOM_CONFIG_H
