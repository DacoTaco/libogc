#ifdef GEKKO

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "definitions.h"
#include "wiiuse_internal.h"
#include "events.h"
#include "io.h"
#include "lwp_wkspace.h"

#define MAX_COMMANDS					0x100
#define MAX_WIIMOTES					5

static vu32* const _ipcReg = (u32*)0xCD000000;
static u8 *__queue_buffer[MAX_WIIMOTES] = { 0, 0, 0, 0, 0 };

extern void parse_event(struct wiimote_t *wm);
extern void idle_cycle(struct wiimote_t* wm);
extern void hexdump(void *d, int len);

static __inline__ u32 ACR_ReadReg(u32 reg)
{
	return _ipcReg[reg>>2];
}

static __inline__ void ACR_WriteReg(u32 reg,u32 val)
{
	_ipcReg[reg>>2] = val;
}

static s32 __wiiuse_disconnected(void *arg,struct bte_pcb *pcb,u8 err)
{
	struct wiimote_listen_t *wml = (struct wiimote_listen_t*)arg;
	struct wiimote_t *wm = wml->wm;

	if(!wm) return ERR_OK;

	//printf("wiimote disconnected\n");
	WIIMOTE_DISABLE_STATE(wm, (WIIMOTE_STATE_IR|WIIMOTE_STATE_IR_INIT));
	WIIMOTE_DISABLE_STATE(wm, (WIIMOTE_STATE_SPEAKER|WIIMOTE_STATE_SPEAKER_INIT));
	WIIMOTE_DISABLE_STATE(wm, (WIIMOTE_STATE_EXP|WIIMOTE_STATE_EXP_HANDSHAKE|WIIMOTE_STATE_EXP_FAILED));
	WIIMOTE_DISABLE_STATE(wm,(WIIMOTE_STATE_CONNECTED|WIIMOTE_STATE_HANDSHAKE|WIIMOTE_STATE_HANDSHAKE_COMPLETE));

	while(wm->cmd_head) {
		__lwp_queue_append(&wm->cmdq,&wm->cmd_head->node);
		wm->cmd_head = wm->cmd_head->next;
	}
	wm->cmd_tail = NULL;
	
	if(wm->event_cb) wm->event_cb(wm,WIIUSE_DISCONNECT);

	wml->wm = NULL;
	return ERR_OK;
}

static s32 __wiiuse_receive(void *arg,void *buffer,u16 len)
{
	struct wiimote_listen_t *wml = (struct wiimote_listen_t*)arg;
	struct wiimote_t *wm = wml->wm;

	if(!wm || !buffer || len==0) return ERR_OK;

	//printf("__wiiuse_receive[%02x]\n",*(char*)buffer);
	wm->event = WIIUSE_NONE;

	memcpy(wm->event_buf,buffer,len);
	memset(&(wm->event_buf[len]),0,(MAX_PAYLOAD - len));
	parse_event(wm);

	if(wm->event!=WIIUSE_NONE) {
		if(wm->event_cb) wm->event_cb(wm,wm->event);
	}

	return ERR_OK;
}

static s32 __wiiuse_connected(void *arg,struct bte_pcb *pcb,u8 err)
{
	struct wiimote_listen_t *wml = (struct wiimote_listen_t*)arg;
	struct wiimote_t *wm;

	wm = wml->assign_cb(&wml->bdaddr);

	if(!wm) {
		bte_disconnect(wml->sock);
		return ERR_OK;
	}

	wml->wm = wm;

	wm->sock = wml->sock;
	wm->bdaddr = wml->bdaddr;

	//printf("__wiiuse_connected()\n");
	WIIMOTE_ENABLE_STATE(wm,(WIIMOTE_STATE_CONNECTED|WIIMOTE_STATE_HANDSHAKE));

	wm->handshake_state = 0;
	wiiuse_handshake(wm,NULL,0);

	return ERR_OK;
}

void __wiiuse_sensorbar_enable(int enable)
{
	u32 val;
	u32 level;

	level = IRQ_Disable();
	val = (ACR_ReadReg(0xc0)&~0x100);
	if(enable) val |= 0x100;
	ACR_WriteReg(0xc0,val);
	IRQ_Restore(level);
}

int wiiuse_register(struct wiimote_listen_t *wml, struct bd_addr *bdaddr, struct wiimote_t *(*assign_cb)(struct bd_addr *bdaddr))
{
	s32 err;

	if(!wml || !bdaddr || !assign_cb) return 0;

	wml->wm = NULL;
	wml->bdaddr = *bdaddr;
	wml->sock = bte_new();
	wml->assign_cb = assign_cb;
	if(wml->sock==NULL) return 0;

	bte_arg(wml->sock,wml);
	bte_received(wml->sock,__wiiuse_receive);
	bte_disconnected(wml->sock,__wiiuse_disconnected);
	
	err = bte_registerdeviceasync(wml->sock,bdaddr,__wiiuse_connected);
	if(err==ERR_OK) return 1;

	return 0;
}	

void wiiuse_disconnect(struct wiimote_t *wm)
{
	if(wm==NULL || wm->sock==NULL) return;

	WIIMOTE_DISABLE_STATE(wm,WIIMOTE_STATE_CONNECTED);
	bte_disconnect(wm->sock);
}

void wiiuse_sensorbar_enable(int enable)
{
	__wiiuse_sensorbar_enable(enable);
}


void wiiuse_init_cmd_queue(struct wiimote_t *wm)
{
	u32 size;

	if (!__queue_buffer[wm->unid]) {
		size = (MAX_COMMANDS*sizeof(struct cmd_blk_t));
		__queue_buffer[wm->unid] = __lwp_wkspace_allocate(size);
		if(!__queue_buffer[wm->unid]) return;
	}

	__lwp_queue_initialize(&wm->cmdq,__queue_buffer[wm->unid],MAX_COMMANDS,sizeof(struct cmd_blk_t));
}

int wiiuse_io_write(struct wiimote_t *wm,ubyte *buf,int len)
{
	if(wm->sock) {
		return bte_senddata(wm->sock,buf,len);
	}

	return ERR_CONN;
}

int wiiuse_find(struct wiimote_t** wm, int max_wiimotes, int timeout) {
	int found_devices;
	int found_wiimotes;

	/* reset all wiimote bluetooth device addresses */
	for (found_wiimotes = 0; found_wiimotes < max_wiimotes; ++found_wiimotes) {
		int i = 0;
		for(; i < BD_ADDR_LEN; i++)
			wm[found_wiimotes]->bdaddr.addr[i] = 0;
	}
	found_wiimotes = 0;

	struct inquiry_info scan_info_arr[128];
	struct inquiry_info* scan_info = scan_info_arr;
	memset(&scan_info_arr, 0, sizeof(scan_info_arr));

	/* scan for bluetooth devices */
	found_devices = bte_inquiry(scan_info, 128, 1);
	if (found_devices < 0) {
		return 0;
	}

	//printf("Found %i bluetooth device(s).", found_devices);

	int i = 0;

	/* display discovered devices */
	for (; (i < found_devices) && (found_wiimotes < max_wiimotes); ++i) {
		if ((scan_info[i].cod[0] == WM_DEV_CLASS_0) &&
			(scan_info[i].cod[1] == WM_DEV_CLASS_1) &&
			(scan_info[i].cod[2] == WM_DEV_CLASS_2))
		{
			/* found a device */
			sprintf(wm[i]->bdaddr_str, "%x:%x:%x:%x:%x:%x",
				scan_info[i].bdaddr.addr[0],
				scan_info[i].bdaddr.addr[1],
				scan_info[i].bdaddr.addr[2],
				scan_info[i].bdaddr.addr[3],
				scan_info[i].bdaddr.addr[4],
				scan_info[i].bdaddr.addr[5]);

			//printf("Found wiimote (%s) [id %i].", wm[found_wiimotes]->bdaddr_str, wm[found_wiimotes]->unid);

			wm[found_wiimotes]->bdaddr = scan_info[i].bdaddr;
			WIIMOTE_ENABLE_STATE(wm[found_wiimotes], WIIMOTE_STATE_DEV_FOUND);
			++found_wiimotes;
		}
	}

	return found_wiimotes;
}

#endif
