/*******************************************************************************
 *
 * Filename:
 * ---------
 *
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *
 * Author:
 * -------
 * ChiPeng
 *
 *------------------------------------------------------------------------------
 * $Revision:$ 1.0.0
 * $Modtime:$
 * $Log:$
 *
 * 06 26 2010 chipeng.chang
 * [ALPS00002705][Need Patch] [Volunteer Patch] ALPS.10X.W10.11 Volunteer patch for speech parameter 
 * modify speech parameters.
 *
 * Mar 15 2010 mtk02308
 * [ALPS] Init Custom parameter
 *
 *

 *
 *
 *******************************************************************************/
#ifndef SPEECH_COEFF_DEFAULT_H
#define SPEECH_COEFF_DEFAULT_H

#ifndef FALSE
#define FALSE 0
#endif

//speech parameter depen on BT_CHIP cersion
#if defined(MTK_MT6611)

#define BT_COMP_FILTER (1 << 15)
#define BT_SYNC_DELAY  86

#elif defined(MTK_MT6612)

#define BT_COMP_FILTER (1 << 15)
#define BT_SYNC_DELAY  86

#elif defined(MTK_MT6616) || defined(MTK_MT6620) || defined(MTK_MT6622) || defined(MTK_MT6626) || defined(MTK_MT6628)

#define BT_COMP_FILTER (1 << 15)
#define BT_SYNC_DELAY  86

#else // MTK_MT6620

#define BT_COMP_FILTER (0 << 15)
#define BT_SYNC_DELAY  86

#endif

#ifdef MTK_DUAL_MIC_SUPPORT

  #ifndef MTK_INTERNAL
  #define SPEECH_MODE_PARA13 (371)
  #define SPEECH_MODE_PARA14 (23)
  #define SPEECH_MODE_PARA03 (29)
  #define SPEECH_MODE_PARA08 (400)
  #else
  #define SPEECH_MODE_PARA13 (0)
  #define SPEECH_MODE_PARA14 (0)
  #define SPEECH_MODE_PARA03 (31)
  #define SPEECH_MODE_PARA08 (80)
  #endif

#else
#define SPEECH_MODE_PARA13 (0)
#define SPEECH_MODE_PARA14 (0)
#define SPEECH_MODE_PARA03 (31)
#define SPEECH_MODE_PARA08 (80)


#endif

#ifdef NXP_SMARTPA_SUPPORT
	#define MANUAL_CLIPPING (1 << 15)
	#define NXP_DELAY_REF   (1 << 6)
	#define PRE_CLIPPING_LEVEL 32767
#else
	#define MANUAL_CLIPPING (0 << 15)
	#define NXP_DELAY_REF   (0 << 6) 
	#define PRE_CLIPPING_LEVEL 10752
#endif



#define DEFAULT_SPEECH_NORMAL_MODE_PARA \
   96,   253,  16388,     SPEECH_MODE_PARA03,   57351,    27,   400,     0, \
   SPEECH_MODE_PARA08,  4325,      611,       0,   20488,      0|SPEECH_MODE_PARA13,     0|SPEECH_MODE_PARA14,  8192

#define DEFAULT_SPEECH_EARPHONE_MODE_PARA \
   256,    253, 4228,    31, 57351,    31,   400,     0,\
    80,  4325,    611,     0, 20488,     0,     0,     0  

#define DEFAULT_SPEECH_BT_EARPHONE_MODE_PARA \
     0,   253, 10756,    31, 53255,  31,   400,     148, \
    80,  4325,    611,     0, 20488 | BT_COMP_FILTER,  0,     0,     BT_SYNC_DELAY

#define DEFAULT_SPEECH_LOUDSPK_MODE_PARA \
    96|MANUAL_CLIPPING ,   224,  5256,    31, 57351, 24607,   400,   132, \
    84,  4325,    611,     0, 20488|NXP_DELAY_REF,     0,     0,     0

#define DEFAULT_SPEECH_CARKIT_MODE_PARA \
    96,   224,  5256,    31, 57351, 24607,   400,   132, \
    84,  4325,    611,     0, 20488,        0,     0,     0

#define DEFAULT_SPEECH_BT_CORDLESS_MODE_PARA \
    0,      0,      0,      0,      0,      0,      0,      0, \
    0,      0,      0,      0,      0,      0,      0,      0

#define DEFAULT_SPEECH_AUX1_MODE_PARA \
    0,      0,      0,      0,      0,      0,      0,      0, \
    0,      0,      0,      0,      0,      0,      0,      0

#define DEFAULT_SPEECH_AUX2_MODE_PARA \
    0,      0,      0,      0,      0,      0,      0,      0, \
    0,      0,      0,      0,      0,      0,      0,      0

#define DEFAULT_SPEECH_COMMON_PARA \
    0,  55997,  31000,    PRE_CLIPPING_LEVEL,      32769,      0,      0,      0, \
    0,      0,      0,      0

#define DEFAULT_SPEECH_VOL_PARA \
    0,      0,      0,      0

#define DEFAULT_AUDIO_DEBUG_INFO \
    0,      0,      0,      0,      0,      0,      0,      0, \
    0,      0,      0,      0,      0,      0,      0,      0

#define DEFAULT_VM_SUPPORT  FALSE

#define DEFAULT_AUTO_VM     FALSE

#define MICBAIS     1900

#define DEFAULT_WB_SPEECH_NORMAL_MODE_PARA \
    96,   253, 16388,    SPEECH_MODE_PARA03, 57607,   27,   400,     32, \
    SPEECH_MODE_PARA08,  4325,   611,     0,  16392,    0|SPEECH_MODE_PARA13,     0|SPEECH_MODE_PARA14,  8192  

#define DEFAULT_WB_SPEECH_EARPHONE_MODE_PARA \
     256,   253, 4228,    31, 57607,     31,  400,     0, \
    80,  4325,   611,     0,  16392,     0,     0,     0  

#define DEFAULT_WB_SPEECH_BT_EARPHONE_MODE_PARA \
     0,   253, 10756,    31, 53511,  31,   400,     0, \
    80,  4325,   611,     0,  16392 | BT_COMP_FILTER,  0,     0, BT_SYNC_DELAY  

#define DEFAULT_WB_SPEECH_LOUDSPK_MODE_PARA \
    96|MANUAL_CLIPPING,   224,  5256,    31, 57607, 24607,   400,   128, \
    84,  4325,   611,     0,  16392|NXP_DELAY_REF,     0,     0,     0  

#define DEFAULT_WB_SPEECH_CARKIT_MODE_PARA \
 65422, 65435, 65186, 65308, 65505, 65275, 65338,    13,\
 65234, 65363, 64715, 65304,   521, 65113, 65154, 64993

#define DEFAULT_WB_SPEECH_BT_CORDLESS_MODE_PARA \
 65479, 65531, 65467, 65414, 65375, 65364,    28, 65412,\
 65356, 65499, 65472, 65493, 65417, 65339, 65383, 65447

#define DEFAULT_WB_SPEECH_AUX1_MODE_PARA \
   343,   118, 64728, 64330,  1673, 65251, 64281, 63393,\
  2570, 63972,  3169, 57911, 16423, 16423, 57911,  3169

#define DEFAULT_WB_SPEECH_AUX2_MODE_PARA \
 63972,  2570, 63393, 64281, 65251,  1673, 64330, 64728,\
   118,   343, 64993, 65154, 65113,   521, 65304, 64715


/* The Bluetooth PCM digital volume */
/* default_bt_pcm_in_vol : uplink, only for enlarge volume,
                           0x100 : 0dB  gain
                           0x200 : 6dB  gain
                           0x300 : 9dB  gain
                           0x400 : 12dB gain
                           0x800 : 18dB gain
                           0xF00 : 24dB gain             */
#define DEFAULT_BT_PCM_IN_VOL        0x100
/* default_bt_pcm_out_vol : downlink gain,
                           0x1000 : 0dB; maximum 0x7FFF  */
#define DEFAULT_BT_PCM_OUT_VOL       0x1000


#endif
