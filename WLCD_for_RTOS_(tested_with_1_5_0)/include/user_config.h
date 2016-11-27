#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

//#define DO_DEBUG						//uncomment this to output basic debug level msgs on TxD

#ifdef DO_DEBUG
#define DBG(...)			os_printf( __VA_ARGS__ )
#else
#define DBG
#endif

#endif
