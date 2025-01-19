#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <linux/rtc.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>

#include <common.h>
#include <miniui.h>
#include <ftm.h>


#define TAG "[HALL] "
#define mod_to_hallsensor(p) (struct hallsensor*)((char*)(p) + sizeof(struct ftm_module))

#define SYS_SLIDE_STATUS "/sys/bus/platform/drivers/mtk-kpd/hallstatus"

enum {
	ITEM_HALL,
	ITEM_PASS,
	ITEM_FAIL
};

/* for removing compiler warnings */
static char str_hall[] = "Test Hall";
static char str_pass[] = "Test Pass";
static char str_fail[] = "Test Fail";

static item_t hallsensor_item[] = {
	{ ITEM_HALL,   uistr_sensor_test,   0 },
	{ ITEM_PASS,   uistr_pass,   0 },
	{ ITEM_FAIL,   uistr_fail,   0 },
	{ -1, NULL, 0 },
};

struct hallsensor
{
	struct ftm_module *mod;
	char  info[1024];
	bool  exit_thd;
	text_t title;
	text_t text;
	pthread_t update_thd;
	struct itemview *iv;
};

static int isSlid(void)
{
	char strbuff[10];
	FILE *fd = fopen(SYS_SLIDE_STATUS, "r");
	if (fd != 0)
		{
			if( fscanf(fd, "%s\n", strbuff) == 1)
				{
					fclose(fd);
					return strcmp(strbuff, "slid");
				}
			fclose(fd);
		}
	return 1;
}


static void *hallsensor_update_iv_thread(void *priv)
{
	struct hallsensor *phallsensor = (struct hallsensor *)priv;
	struct itemview *iv = phallsensor->iv; 
	int res = 0, len = 0;
	char strbuff[10];
	 while (1) {
	 	if (phallsensor->exit_thd)
            break;
		res = isSlid();
		len = 0;
		len += snprintf(phallsensor->info+len, sizeof(phallsensor->info)-len, "Hall Switch state is : %s \n", !res ? "open": "close");  
		iv->set_text(iv, &phallsensor->text);
        iv->redraw(iv);
	 }
	 
	 pthread_exit(NULL);
	 return NULL;
}



static int hallsensor_entry(struct ftm_param *param, void *priv)
{	
	int num;
	bool exit;
	struct hallsensor *phallsensor = (struct hallsensor *)priv;
	struct itemview *iv = phallsensor->iv;

	init_text(&phallsensor->title, param->name, COLOR_YELLOW);
	init_text(&phallsensor->text, &phallsensor->info[0], COLOR_YELLOW);
	LOGD(TAG"%s\n", __FUNCTION__);
	iv->set_title(iv, &phallsensor->title);
	iv->set_items(iv, hallsensor_item, ITEM_HALL);
    iv->set_text(iv, &phallsensor->text);
	snprintf(phallsensor->info, sizeof(phallsensor->info), "Initializing...\n");
	phallsensor->exit_thd = false;
	pthread_create(&phallsensor->update_thd, NULL, hallsensor_update_iv_thread, priv);
	while (1) {
		num = iv->run(iv, &exit);
		#if 1
			switch(num){
				case ITEM_HALL:
				break;

				case ITEM_PASS:
					phallsensor->mod->test_result = FTM_TEST_PASS;
					exit = true;
				break;

				case ITEM_FAIL:
					phallsensor->mod->test_result = FTM_TEST_FAIL;
					exit = true;
				break;

				default:
				break;
			}	
				
		#else
			if (num == ITEM_PASS) {
				phallsensor->mod->test_result = FTM_TEST_PASS;
				exit = true;
			} else if (num == ITEM_FAIL) {
				phallsensor->mod->test_result = FTM_TEST_FAIL;
				exit = true;
			}
		#endif
		if (exit) {
			phallsensor->exit_thd = true;
			break;
		}
	}
	pthread_join(phallsensor->update_thd, NULL);
	
	return 0;
}

int hallsensor_init(void)
{
	int r;
	struct ftm_module *mod;
	struct hallsensor *phallsensor;
       LOGD(TAG"%s\n", __FUNCTION__);
	mod = ftm_alloc(ITEM_HALLSENSOR, sizeof(struct hallsensor));
	if (!mod)
		return -ENOMEM;

	phallsensor = mod_to_hallsensor(mod);
	phallsensor->mod = mod;
	phallsensor->iv = ui_new_itemview();
	if (!phallsensor->iv)
		return -ENOMEM;

	r = ftm_register(mod, hallsensor_entry, (void*)phallsensor);
	if (r) {
		LOGD(TAG "register hallsensor failed (%d)\n", r);
		return r;
	}

	return 0;
}