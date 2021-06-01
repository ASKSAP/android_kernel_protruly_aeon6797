/*
 * File:         byd_algorithm.h
 *
 * Created:	     2015-05-05
 * Used by:      byd_algorithm.c
 * Description:  the interface of BYD fingerprint algorithm encapsulation
 *
 * Copyright (C) 2015 BYD Company Limited
 *
 * Contact: qi.ximing@byd.com; lei.lei@icbyd.com;
 *
 */

#ifndef __BYD_FPS_ALG_H__
#define __BYD_FPS_ALG_H__


/*****************************************************************************/

/* Platform (SOC) select, current tested SOC are:
   PLATFORM_SAMSUNG:	Exynos Tiny4412
   PLATFORM_QUALCOMM:	MSM8909, MSM8976
   PLATFORM_MTK:		MT6735, MT6795
   Select only if platform is MTK:
//#define PLATFORM_MTK <--- defined in byd_fps_bf66xx.h
 */

/* Configure debug log, output by pr_debug(fmt, ...)
 * Note: 1. pr_debug() would not produce excutable code if DEBUG is not defined
 *       2. printk(KERN_DEBUG...) is controled by DEBUG and produce excutable
 *          code even if DEBUG is not specified
 *       3. printk() would alwayse produce log no matter DEBUG definition */

   //#define DEBUG
   #define BYD_FPS_TEST_TIME_LOG

/* The code you are compiling will run in: (please select target) */

 // 1. Linux kernel
       /* situation where compile drivr, driver-lib, alg-lib in kernel */
          #define BYD_FPS_CODE_RUN_IN_KERNEL // must be configured

       /* If you are compiling Algorithm with Driver) library in kernel: */
         // #define BYD_FPS_ALG_DRV_LIBRARY // for client version (make-lib-alg.sh)

       #if !defined(BYD_FPS_ALG_DRV_LIBRARY) // && defined(PLATFORM_QUALCOMM)
       /* Wether algorithm runs in TZ (currently, Qualcomm platform only): */
       // #define BYD_FPS_ALG_IN_TZ // for compile driver or driver-lib
       #endif

 // 2. Qualcomm QSEE, compile (by DS-5)
       // nothing configure here // #define BYD_FPS_ALG_IN_QSEE
 // 3. Qualcomm QSEE
       // #include "qsee_log.h"       // for QSEE only
 // 4. Android or Linux application
       // #define BYD_FPS_ALG_IN_ANDROID
       // #define DEFAULT_CONSOLE_LOGLEVEL  7 // 0 - 7
 // 5. Windows
       // #define BYD_FPS_ALG_IN_WINDOWS
       // #define DEFAULT_CONSOLE_LOGLEVEL  7 // 0 - 7
/*
    0. Linux kernel code, compile driver
          #include <linux/kernel.h>  // <linux/printk.h>
*/

/*****************************************************************************/
#if defined(BYD_FPS_CODE_RUN_IN_KERNEL) && !defined(BYD_FPS_ALG_IN_TZ)
    #define BYD_FPS_ALG_IN_KERNEL
#endif


#ifndef __KERNEL__ // __KERNEL_PRINTK__
  // kernel independent driver library (with/without algorithm) for kernel, or 
  // kernel independent driver library (without alg) for TZ, or 
  // standalone algorithm of Linux/Windows app, QSEE, kernel
/*===========================================================================*/

  #define KERN_EMERG "<0>"   /* system is unusable */
  #define KERN_ALERT "<1>"   /* immediate action must be taken */
  #define KERN_CRIT "<2>"    /* critical conditions, fatal error */
  #define KERN_ERR "<3>"     /* error conditions */
  #define KERN_WARNING "<4>" /* warning conditions */
  #define KERN_NOTICE "<5>"  /* normal but significant condition, worth notice */
  #define KERN_INFO "<6>"    /* informational */
  #define KERN_DEBUG "<7>"   /* debug-level messages */

  #ifndef pr_fmt
  #define pr_fmt(fmt) fmt
  #endif

  #define pr_emerg(fmt, ...) \
    printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
  #define pr_alert(fmt, ...) \
    printk(KERN_ALERT pr_fmt(fmt), ##__VA_ARGS__)
  #define pr_crit(fmt, ...) \
    printk(KERN_CRIT pr_fmt(fmt), ##__VA_ARGS__)
  #define pr_err(fmt, ...) \
    printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
  #define pr_warning(fmt, ...) \
    printk(KERN_WARNING pr_fmt(fmt), ##__VA_ARGS__)
  #define pr_warn pr_warning
  #define pr_notice(fmt, ...) \
    printk(KERN_NOTICE pr_fmt(fmt), ##__VA_ARGS__)
  #define pr_info(fmt, ...) \
    printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)

  #if defined(DEBUG)
    #define pr_debug(fmt, ...) \
      printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
  #else // do not produce debug code:
    #define pr_debug(fmt, ...)
  #endif

 #if defined(BYD_FPS_ALG_IN_KERNEL) /* compile kernel independent driver lib and/or algorithm for kernel */ \
   || defined(BYD_FPS_ALG_IN_TZ)    /* kernel independent driver lib for TZ */
 /*--------------------------------------------------------------------------*/

  /************************************************************
   * Linux Kernel independent driver and/or algorithm library *
   ************************************************************/

   // #include <linux/printk.h> :
   int printk(const char *fmt, ...);

   // #include <string.h> // <linux/string.h> :
   typedef long  __kernel_size_t; // <linux/types.h>
   extern char * strstr(const char *, const char *);
   #define __HAVE_ARCH_STRSTR
   extern char * strcpy(char *,const char *);
   #define __HAVE_ARCH_STRCPY
   extern char * strcat(char *, const char *);
   #define __HAVE_ARCH_STRCAT
   extern void * memset(void *, int, __kernel_size_t);
   #define __HAVE_ARCH_MEMSET
   extern void * memcpy(void *,const void *,__kernel_size_t);
   #define __HAVE_ARCH_MEMCPY

   // #include <stdio.h> // FILE, SEEK_XXX // <linux/fs.h> // SEEK_SET, ...
   #define SEEK_SET	0	/* seek relative to beginning of file */
   #define SEEK_CUR	1	/* seek relative to current file position */
   #define SEEK_END	2	/* seek relative to end of file */
   #define SEEK_MAX	SEEK_END

   typedef struct file FILE;

   /* for compiling byd_fps_libbf663x.c, BEGIN */

   // #include <stdlib.h> <linux/stddef.h> // NULL
   #define NULL ((void *)0)
   #define ssize_t long
   #define u32 unsigned long
   #define u16 unsigned short
   #define u8 unsigned char

   // #include </uapi/asm-generic/errno.h>
   #define	ETIMEDOUT	110	/* Connection timed out */
   // #include </uapi/asm-generic/errno-base.h>
   #define	EINTR		 4	/* Interrupted system call */

   // #include <linux/spi/spi.h>
   struct spi_device;

   // #include <linux/delay.h> // msleep()
   //void msleep(unsigned int msecs);

   /* for compiling byd_fps_libbf663x.c, END */

 #else //elif !defined(BYD_FPS_ALG_IN_WINDOWS) // Standalone algorithm for Android (or Linux app), QSEE
 /*--------------------------------------------------------------------------*/

   #include <stdio.h> // for definition of FILE
   /*
    typedef struct BYD_FILE {
      int fd;
    } FILE;
   */

 #endif

 #if defined(BYD_FPS_ALG_IN_WINDOWS) || defined(BYD_FPS_ALG_IN_ANDROID)
 /*--------------------------------------------------------------------------*/

  /*******************************
   * Windows or Linux App Log API
   *******************************/

  #ifdef BYD_FPS_ALG_IN_WINDOWS
    #include <windows.h>
    #include <direct.h>
  #elif defined(BYD_FPS_ALG_IN_ANDROID)
    // #include "android/log.h"
    // #define printk(...) __android_log_print(ANDROID_LOG_DEBUG, "byd_algorithm", __VA_ARGS__)
    #include <time.h>
  #endif

  #define printk(xx_fmt, ...) \
    do { \
      if ((strstr(xx_fmt, KERN_DEBUG) == xx_fmt) && DEBUG) {        \
          printf(xx_fmt, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_INFO) == xx_fmt) && (DEFAULT_CONSOLE_LOGLEVEL >= 6)) {    \
          printf(xx_fmt, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_NOTICE) == xx_fmt) && (DEFAULT_CONSOLE_LOGLEVEL >= 5)) {  \
          printf(xx_fmt, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_WARNING) == xx_fmt) && (DEFAULT_CONSOLE_LOGLEVEL >= 4)) { \
          printf(xx_fmt, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_ERR) == xx_fmt) && (DEFAULT_CONSOLE_LOGLEVEL >= 3)) {     \
          printf(xx_fmt, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_CRIT) == xx_fmt) && (DEFAULT_CONSOLE_LOGLEVEL >= 2)) {    \
          printf(xx_fmt, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_ALERT) == xx_fmt) && (DEFAULT_CONSOLE_LOGLEVEL >= 1)) {   \
          printf(xx_fmt, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_EMERG) == xx_fmt) && (DEFAULT_CONSOLE_LOGLEVEL >= 0)) {   \
          printf(xx_fmt, ##__VA_ARGS__); } \
      else if (xx_fmt[0] != '<' || xx_fmt[2] != '>') { \
          printf(xx_fmt, ##__VA_ARGS__); } \
    } while (0)

  /*
   * redecrect stdout stream to a log file
   */
  FILE *stdout_log_start(const char *logfile);
  int stdout_log_stop(FILE *fd);

 #elif !defined(BYD_FPS_ALG_IN_KERNEL) && !defined(BYD_FPS_ALG_IN_TZ) // code runs in Qualcomm's qsee
 /*--------------------------------------------------------------------------*/

  /*****************
   * qsee Log API
   *****************/

  #ifndef QSEE_LOG_H // code runs in Qualcomm's qsee but DS-5 independent compile

    #define QSEE_LOG_H

    #define ENABLE_QSEE_LOG_MSG_LOW      0
    #define ENABLE_QSEE_LOG_MSG_MED      0
    #define ENABLE_QSEE_LOG_MSG_HIGH     0
    #define ENABLE_QSEE_LOG_MSG_ERROR    1
    #define ENABLE_QSEE_LOG_MSG_FATAL    1
    #ifdef DEBUG
      #define ENABLE_QSEE_LOG_MSG_DEBUG    1
    #else
      #define ENABLE_QSEE_LOG_MSG_DEBUG    0
    #endif
    #define QSEE_LOG_MSG_LOW      0
    #define QSEE_LOG_MSG_MED      1
    #define QSEE_LOG_MSG_HIGH     2
    #define QSEE_LOG_MSG_ERROR    3
    #define QSEE_LOG_MSG_FATAL    4
    #define QSEE_LOG_MSG_DEBUG    5
	
  //#define uint32 unsigned long
  //#define uint16 unsigned short
    #define uint8 unsigned char
	
    void qsee_log(uint8 pri, const char *fmt, ...);
    void qsee_printf(const char *fmt, ...);

    unsigned long long qsee_get_uptime(void);

  #else

    // #include "string.h"  // strcpy(), memset(), ..., included by qsee_log.h
    #include "qsee_timer.h" // qsee_get_uptime()

  #endif // QSEE_LOG_H

  #define JIFFY "[%d]"    /* long long output %ld not supported in TZ */

  #define printk(xx_fmt, ...) \
    do { \
      if ((strstr(xx_fmt, KERN_INFO) == xx_fmt) && (ENABLE_QSEE_LOG_MSG_LOW)) {          \
          qsee_log(QSEE_LOG_MSG_LOW, xx_fmt+3, ##__VA_ARGS__); }   \
      else if ((strstr(xx_fmt, KERN_NOTICE) == xx_fmt) && (ENABLE_QSEE_LOG_MSG_MED)) {   \
          qsee_log(QSEE_LOG_MSG_MED, xx_fmt+3, ##__VA_ARGS__); }   \
      else if ((strstr(xx_fmt, KERN_WARNING) == xx_fmt) && (ENABLE_QSEE_LOG_MSG_HIGH)) { \
          qsee_log(QSEE_LOG_MSG_HIGH, xx_fmt+3, ##__VA_ARGS__); }  \
      else if ((strstr(xx_fmt, KERN_ERR) == xx_fmt) && (ENABLE_QSEE_LOG_MSG_ERROR)) {    \
          qsee_log(QSEE_LOG_MSG_ERROR, xx_fmt+3, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_CRIT) == xx_fmt                \
            || strstr(xx_fmt, KERN_ALERT) == xx_fmt                \
            || strstr(xx_fmt, KERN_EMERG) == xx_fmt) && (ENABLE_QSEE_LOG_MSG_FATAL)) {   \
          qsee_log(QSEE_LOG_MSG_FATAL, xx_fmt+3, ##__VA_ARGS__); } \
      else if ((strstr(xx_fmt, KERN_DEBUG) == xx_fmt) && (ENABLE_QSEE_LOG_MSG_DEBUG)) {  \
          qsee_log(QSEE_LOG_MSG_DEBUG, JIFFY xx_fmt, (unsigned long)qsee_get_uptime(), ##__VA_ARGS__); } \
      else if (xx_fmt[0] != '<' || xx_fmt[2] != '>') {             \
       /* long long n000 = 0, jiffy_time = qsee_get_uptime(); */   \
       /* while (jiffy_time > 0xffffffff) {jiffy_time /= 1000; ++n000;}; */  \
       /* if (n000 > 0) jiffy_time = qsee_get_uptime() % (1000 * n000);  */  \
       /* qsee_printf(JIFFY xx_fmt, (long)jiffy_time, ##__VA_ARGS__); }  */  \
       /* instead, truncate the time by casting from long long to long:  */  \
          qsee_printf(JIFFY xx_fmt, (unsigned long)qsee_get_uptime(), ##__VA_ARGS__); }  \
    } while (0)
  /*
  #define printf(...) \
    do { \
          qsee_log(QSEE_LOG_MSG_DEBUG, ##__VA_ARGS__); \
    } while (0)
  */

  #define BYD_FPS_ALG_IN_TZ
 /*--------------------------------------------------------------------------*/
 #endif // WINDOWS or ANDROID

#else //#elif !defined(BYD_FPS_ALG_IN_KERNEL) // kernel detected, compile driver using kernel head
/*===========================================================================*/

  /*******************************
   * Linux Kernel
   *******************************/
  #ifndef BYD_FPS_ALG_IN_TZ // for compile kernel driver
    #define BYD_FPS_ALG_IN_KERNEL
  #endif

  #include <linux/fs.h>     // O_RDONLY, O_RDWR, O_APPEND, SEEK_xxx
  #include <linux/string.h> // strcpy(), memset(), ...
  typedef struct file FILE;

#endif //__KERNEL__

/*===========================================================================*/

#if !defined(BYD_FPS_ALG_IN_KERNEL) && !defined(BYD_FPS_ALG_IN_TZ)
  #include <math.h>
  #include <stdio.h>   // FILE, SEEK_SET,...
  #include <stdlib.h>  // NULL
  #include <string.h>  // strxxx(), memxxx()
  #define bool int     // #include <stdbool.h>
#else
    #ifdef DEBUG
      #define FP_DBG(format, ...) \
		printk(format, ##__VA_ARGS__)
      #define PR_BUF(pr_msg, buf, len) \
		{ \
			int _i_; \
			FP_DBG("%s: %s", __FUNCTION__, pr_msg); \
			for (_i_ = 0; _i_ < len; _i_++) { \
				if(_i_%16 == 0) FP_DBG("\n"); \
					FP_DBG(" %02x", *(buf + _i_)); \
			} \
			FP_DBG("\n"); \
		}
    #else
      #define FP_DBG(format, ...)
      #define PR_BUF(pr_msg, buf, len)
    #endif
    #ifdef BYD_FPS_TEST_TIME_LOG
    	#define DBG_TIME(format, ...) \
    		printk(format, ##__VA_ARGS__)
    #else
    	#define DBG_TIME(format, ...)
    #endif
#endif


/*****************
 * Algorithm API
 *****************/

/* public interface of algorithm */

int byd_fps_alg_init(void);

void byd_fps_alg_remove(void);

int byd_fps_set_version(unsigned int version, unsigned char *out,unsigned char ChipType);//������Կ

void byd_fps_set_template_path(char *path);

void byd_fps_set_template(unsigned int max_num_ids,
						unsigned int num_templates,
						unsigned int max_num_templates,
						unsigned char same_fjudge);

void byd_fps_set_sensor_area(unsigned int valid_enroll, unsigned int invalid_enroll,
                             unsigned int valid_match,  unsigned int invalid_match, 
							 unsigned int invalid_var, unsigned char movexy, unsigned char cutmethod);

int byd_fps_safe_level_get_n_set(unsigned int safe_level);


/*===========================================================================*/

/* library internal interface between driver and algorithm */

#ifndef BYD_FPS_ALG_DRV_LIBRARY

void byd_fps_encoding22(unsigned char* d_imagedata, unsigned short *imagedata);

void byd_fps_encoding(unsigned char* d_imagedata,unsigned short *imagedata);
int byd_fps_image_normalizing(unsigned short *pBmpBuf1,unsigned char *pBmpBuf,unsigned char cutbit);
void  byd_fps_saveadjustround(unsigned short*imageadjudge);
int has_fingerprint_enrolled(int id);

int byd_fps_enroll(unsigned char* d_imagedata,unsigned int ui_nID,unsigned char *return_value);
int byd_fps_match(unsigned char* p_imagedata,unsigned int ui_nID,unsigned char *return_value);

int byd_fps_remove_template(unsigned int id);
int byd_fps_merge_template(unsigned int id, unsigned char *out);
int byd_fps_write_template(void);

int byd_fps_image_16bit8(unsigned char *pBmpBuf1,unsigned char *pBmpBuf,unsigned char cutbit);

int byd_fps_get_processed_image(unsigned char *binary_image, unsigned char *thin_image);

void byd_fps_matchqualityjudge(unsigned char *image16,unsigned char *re_val);
void byd_fps_enrolqualityjudge(unsigned char *image16,unsigned char *re_val);

void byd_fps_saveadjust(unsigned char* d_imagedata);
void byd_fps_saveadjust2(unsigned char *bmpadjudgst,unsigned short pointnum,unsigned short *pointcoor);

void byd_fps_adjustimage(unsigned char* d_imagedata,unsigned short *p_imagedata);
void byd_badpoint_judge(unsigned char *blankbmp,unsigned int *bpnum);

#endif

/*===========================================================================*/

/*****************
 * File IO API
 *****************/

FILE *byd_fopen(char *path, char *type);

int byd_fclose(FILE *fp);

int byd_fseek(FILE *fp, long offset, int fromwhere);

long byd_ftell(FILE *fp);

int byd_fread(void *buf, int size, int count, FILE *fp);

int byd_fwrite(void *buf, int size, int count,FILE *fp);

int byd_remove(char *path);

int byd_rename(char *old_name, char *new_name); // *** NOTICE: qsee not implement yet ***

int byd_feof(FILE *fp);


/*****************
 * Heap API
 *****************/

void *byd_malloc(unsigned int size);

void byd_free(void *buf);


#ifndef BYD_FPS_ALG_IN_WINDOWS

/************************
 * string-int Convertion
 ************************/
/*
 * str    : string to be converted
 * output : the integer of first data in the str
 */
int byd_atoi(const char *str);

/*
 * value : integer to be converted
 * buf   : converted string
 * radix : radix for integer
 */
char *itoa(int val, char *buf, unsigned int radix);

#endif // not BYD_FPS_ALG_IN_WINDOWS

#endif
