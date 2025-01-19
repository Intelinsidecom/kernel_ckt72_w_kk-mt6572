CKT_AUTO_ADD_GLOBAL_DEFINE_BY_NAME = CKT_INTERPOLATION LCD_UPDATE_TWO_TIMES CKT_ALSPS_TMD2772_CALI_SUPPORT TP_TYPE_NEW_SUPPORT
CKT_AUTO_ADD_GLOBAL_DEFINE_BY_NAME_VALUE =PROJ_NAME CUST_NAME SOFTCODE USB_MANUFACTURER_STRING USB_PRODUCT_STRING USB_STRING_SERIAL_IDX CUSTOM_EXIF_STRING_MAKE CUSTOM_EXIF_STRING_MODEL CUSTOM_EXIF_STRING_SOFTWARE CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME CUSTOM_BTMTK_ANDROID_ADAPTER_PREFIX CUSTOM_BTMTK_ANDROID_DEFAULT_LOCAL_NAME CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME CUSTOM_BTMTK_ANDROID_ADAPTER_PREFIX
CKT_AUTO_ADD_GLOBAL_DEFINE_BY_VALUE = PROJ_NAME CUST_NAME LCD_BACKLIGHT_MODE
#############################
#############################
#############################

#��Ŀ����ض���
PROJ_NAME = HOPE01A
CUST_NAME = DORO
SOFTCODE = S01A
BASEVERNO=401
#############################
#����������ro.product.model
#CKT_PRODUCT_MODEL="DoroPhoneEasy820"
#����������ȱʡʱ��persist.sys.timezone
TIMEZONE=Europe/Paris


############usb���#################
USB_MANUFACTURER_STRING="DORO"
USB_PRODUCT_STRING="DoroLiberto820Mini"
USB_STRING_SERIAL_IDX=$(strip $(USB_PRODUCT_STRING) )

############exif���#################
#CUSTOM_EXIF_STRING_MAKE="DORO"
#CUSTOM_EXIF_STRING_MODEL="DoroPhoneEasy820"
#CUSTOM_EXIF_STRING_SOFTWARE=""

############bt���#################
#CUSTOM_BTMTK_ANDROID_DEFAULT_LOCAL_NAME =$(strip $(PROJ_NAME) )_BT
#CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME=$(strip $(PROJ_NAME) )_DEVICE
#CUSTOM_BTMTK_ANDROID_ADAPTER_PREFIX=$(strip $(CUST_NAME) )BT

#$(error $(CUSTOM_BTMTK_ANDROID_ADAPTER_PREFIX))
#############################
#���ܵĿ���,�ᵼ�뵽mediatek/source/frameworks/featureoption/java/com/mediatek/featureoption/FeatureOption.java
#�޸ĵ�ʱ��ע��,�� mediatek/build/tools/javaoption.pm��������ģ��
#����ע�����enableֻ������yes,������������
TESTA = yes
TESTB = no
TESTC = testc_none

#����ģʽ13646633��ʾ������Ϣ,����Ӳ��������汾��
CKT_DISPLAY_OTHER_INFO_ENGINEERMODE = yes
#############################

#���Ҫ�̶��汾��,����������,����ע�͵���,����������!!!
#CKT_BUILD_VERNO = PANDORA-S0A_CKT_L2EN_111_111111
#CKT_BUILD_INTERNAL_VERNO =PANDORA-S0A_CKT_L2EN_111_111111111111

#############################
#����ͷ�����ֵ,���������800w����ͷ,��ֱ�Ӵ���,����ʹ��ov8825
CKT_INTERPOLATION = no

#lcd��������,����ʹ��USE_LED_PMIC_LCD_BOOST,USE_LED_MODE_PWM,Ŀǰ������˸������ʱʹ��USE_LED_PMIC_LCD_BOOST
LCD_BACKLIGHT_MODE=USE_LED_PMIC_LCD_BOOST

#LCD update screen two times controler,because MT6575M can only up to HVGA, but now need up to WVGA,so must default this.
LCD_UPDATE_TWO_TIMES = yes

#alsps is tmd2772,need to add the offset calibration
CKT_ALSPS_TMD2772_CALI_SUPPORT = yes














































###########����Ϊ�����Ķ���,һ�㲻��Ҫ���
_CKT_BUILD_VERNO  = $(strip $(PROJ_NAME) )-$(strip $(SOFTCODE) )_$(strip $(CUST_NAME) )_L$(words $(subst hdpi, ,$(strip $(MTK_PRODUCT_LOCALES))))$(word 1,$(subst _, ,$(subst zh_TW,TR,$(subst zh_CN,SM,$(strip $(MTK_PRODUCT_LOCALES))))))_$(strip $(BASEVERNO))

DATA_FOR_VERO=$(shell date +%y%m%d)
DATA_FOR_INTERNAL_VERO=$(shell date +%y%m%d%H%M%S)

DORO_BUILD_NO ?=  $(call uc , $(strip $(PROJ_NAME) )-$(strip $(SOFTCODE) )_$(strip $(BASEVERNO)))
DORO_SOFTCODE = $(call uc , $(SOFTCODE))

CKT_BUILD_VERNO  ?= $(call uc, $(_CKT_BUILD_VERNO)_$(strip $(DATA_FOR_VERO)))
CKT_BUILD_INTERNAL_VERNO  ?= $(call uc, $(_CKT_BUILD_VERNO)_$(strip $(DATA_FOR_INTERNAL_VERO)))

MTK_BUILD_VERNO  ?= $(call uc, $(_CKT_BUILD_VERNO)_$(strip $(DATA_FOR_VERO)))


#############################

