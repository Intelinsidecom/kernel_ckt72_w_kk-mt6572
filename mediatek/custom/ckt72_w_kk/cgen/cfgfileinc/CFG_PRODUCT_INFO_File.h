/*******************************************************************************
 *
 * Filename:
 * ---------
 *   CFG_PRODUCT_INFO_File.h
 *
 * Project:
 * --------
 *   YuSu
 *
 * Description:
 * ------------
 *    header file of main function
 *
 * Author:
 * -------
 *   Yuchi Xu(MTK81073)
 *
 *------------------------------------------------------------------------------
 *
 *******************************************************************************/



#ifndef _CFG_PRODUCT_INFO_FILE_H
#define _CFG_PRODUCT_INFO_FILE_H


// the record structure define of PRODUCT_INFO nvram file
typedef struct
{
    unsigned char imei[8];
    unsigned char svn;
    unsigned char pad;
} nvram_ef_imei_imeisv_struct;

typedef struct{
		unsigned char barcode[64];
		nvram_ef_imei_imeisv_struct IMEI[4];
		unsigned char reserved[1024-40-64-20];//20140314 xiexijun add CKT advancereset flag to nvram ,which is a factory request.
		char ps_cali_nvram_data[20];  //add by jy  for ps_cali 2014.06.27
		//unsigned char reserved[1024-40-64-20];//[1024-40-64];//modify by jy subtract the length of ps_cali_nvram_data 2014.06.27
}PRODUCT_INFO;

//the record size and number of PRODUCT_INFO nvram file
#define CFG_FILE_PRODUCT_INFO_SIZE    sizeof(PRODUCT_INFO)
#define CFG_FILE_PRODUCT_INFO_TOTAL   1

#endif /* _CFG_PRODUCT_INFO_FILE_H */
