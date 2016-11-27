/* Most common / basic includes + missing includes (Espressif intentionally hides some of provided functions).
 * Downloaded from the ESP8266 community, altered slightly by wdim, 11/2016
 */
#ifndef ESPMISSINGINCLUDES_H
#define ESPMISSINGINCLUDES_H

#include <c_types.h>

int strcasecmp(const char *a, const char *b);
int strncasecmp(const char *a, const char *b, size_t c);

#ifndef FREERTOS

#include <eagle_soc.h>
#include <ets_sys.h>
#include <os_type.h>
#include <osapi.h>
#include "mem.h"

//Missing function prototypes in include folders. Gcc will warn on these if we don't define 'em anywhere.
//MOST OF THESE ARE GUESSED! but they seem to swork and shut up the compiler.
typedef struct espconn espconn;

int atoi(const char *nptr);
void ets_install_putc1(void *routine);
void ets_isr_attach(int intr, void *handler, void *arg);
void ets_isr_mask(unsigned intr);
void ets_isr_unmask(unsigned intr);
int ets_memcmp(const void *s1, const void *s2, size_t n);
void *ets_memcpy(void *dest, const void *src, size_t n);
void *ets_memset(void *s, int c, size_t n);
void *ets_memmove(void *dest, const void *src, size_t n);
int ets_sprintf(char *str, const char *format, ...)  __attribute__ ((format (printf, 2, 3)));
int ets_snprintf(char *str, size_t size, const char *format, ...) __attribute__ ((format (printf, 3, 4)));
int ets_str2macaddr(void *, void *);
int ets_strcmp(const char *s1, const char *s2);
char *ets_strcpy(char *dest, const char *src);
size_t ets_strlen(const char *s);
int ets_strncmp(const char *s1, const char *s2, int len);
char *ets_strncpy(char *dest, const char *src, size_t n);
char *ets_strstr(const char *haystack, const char *needle);
void ets_timer_arm_new(ETSTimer *a, int b, int c, int isMstimer);
void ets_timer_disarm(ETSTimer *a);
void ets_timer_setfn(ETSTimer *t, ETSTimerFunc *fn, void *parg);
void ets_update_cpu_frequency(int freqmhz);
int os_printf(const char *format, ...)  __attribute__ ((format (printf, 1, 2)));
int os_printf_plus(const char *format, ...)  __attribute__ ((format (printf, 1, 2)));
void uart_div_modify(int no, unsigned int freq);
uint8 wifi_get_opmode(void);
uint32 system_get_time();
int rand(void);
void ets_bzero(void *s, size_t n);
void ets_delay_us(int ms);

void *pvPortMalloc(size_t xWantedSize, const char *file, int line);
void *pvPortZalloc(size_t, const char *file, int line);
void vPortFree(void *ptr, const char *file, int line);
void *vPortMalloc(size_t xWantedSize, const char *file, int line);
void pvPortFree(void *ptr, const char *file, int line);

#define os_snprintf ets_snprintf

#endif

#endif
