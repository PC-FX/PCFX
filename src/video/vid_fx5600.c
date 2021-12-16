/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Emulation of nVidia's FX 5600 graphics card.
 *		Special thanks to Marcelina Kościelnicka, without whom this
 *		would not have been possible.
 *
 * Version:	@(#)vid_fx5600.c	1.0.0	2019/09/13
 *
 * Authors:	Miran Grca, <mgrca8@gmail.com>
 *		Melissa Goad
 *
 *		Copyright 2020 Miran Grca.
 *		Copyright 2020 Melissa Goad.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <wchar.h>
#include <86box/86box.h>
#include "../cpu/cpu.h"
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/pci.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/video.h>
#include <86box/i2c.h>
#include <86box/vid_ddc.h>
#include <86box/vid_svga.h>
#include <86box/vid_svga_render.h>

#define BIOS_FX5600_PATH		"roms/video/nvidia/Inno3d_325-500mhz.rom"

#define FX5600_VENDOR_ID 0x10de
#define FX5600_DEVICE_ID 0x0312

typedef struct fx5600_t
{
	mem_mapping_t	mmio_mapping;
	mem_mapping_t 	linear_mapping;

	svga_t		svga;

	rom_t		bios_rom;

	uint32_t		vram_size, vram_mask,
			mmio_base, lfb_base;

	uint8_t		read_bank, write_bank;

	uint8_t		pci_regs[256];
	uint8_t		int_line;

	int			card;

	struct
	{
		uint8_t rma_access_reg[4];
		uint8_t rma_mode;
		uint32_t rma_dst_addr;
		uint32_t rma_data;
	} rma;

	struct 
	{
		uint32_t intr;
		uint32_t intr_en;
		uint32_t intr_line;
		uint32_t enable;
	} pmc;

	struct
	{
		uint32_t intr;
		uint32_t intr_en;
		uint32_t cache_error;

		struct
		{
			uint32_t push_enabled, pull_enabled;
			uint32_t status0, status1;
			uint32_t put, get;
		} caches[2];
	} pfifo;

	struct
	{
		uint32_t intr, intr_en;

		uint64_t time;
		uint32_t alarm;

		uint16_t clock_mul, clock_div;
	} ptimer;

	struct
	{
		uint16_t width;
		int bpp;

		uint32_t config_0;
	} pfb;
	
	struct
	{
		uint32_t intr, intr_en;
	} pcrtc;

	struct
	{
		uint32_t fifo_enable;
	} pgraph;
	

	struct
	{
		uint32_t nvpll, mpll, vpll;
	} pramdac;

	uint32_t ramin[0x100000/4];

	pc_timer_t nvtimer;
	pc_timer_t mtimer;

	double nvtime;
	double mtime;

	void *i2c, *ddc;
} fx5600_t;

static video_timings_t timing_fx5600		= {VIDEO_PCI, 2,  2,  1,  20, 20, 21};

static uint8_t fx5600_in(uint16_t addr, void *p);
static void fx5600_out(uint16_t addr, uint8_t val, void *p);

static uint8_t 
fx5600_pci_read(int func, int addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	// svga_t *svga = &fx5600->svga;

    //pclog("FX 5600 PCI read %02x\n", addr);

	switch (addr) {
	case 0x00: return 0xde; /*nVidia*/
	case 0x01: return 0x10;

	case 0x02: return 0x20;
	case 0x03: return 0x00;
	
	case 0x04: return fx5600->pci_regs[0x04] & 0x37; /*Respond to IO and memory accesses*/
	case 0x05: return fx5600->pci_regs[0x05] & 0x03;

	case 0x06: return 0x18;
	case 0x07: return 0x02;

	case 0x08: return 0x00; /*Revision ID*/
	case 0x09: return 0x00; /*Programming interface*/

	case 0x0a: return 0x00; /*Supports VGA interface*/
	case 0x0b: return 0x03;

	case 0x13: return fx5600->mmio_base >> 24;

	case 0x17: return fx5600->lfb_base >> 24;

	case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		return fx5600->pci_regs[addr];

	case 0x30: return (fx5600->pci_regs[0x30] & 0x01); /*BIOS ROM address*/
	case 0x31: return 0x00;
	case 0x32: return fx5600->pci_regs[0x32];
	case 0x33: return fx5600->pci_regs[0x33];

	case 0x3c: return fx5600->int_line;
	case 0x3d: return PCI_INTA;

	case 0x3e: return 0x03;
	case 0x3f: return 0x01;
	}

	return 0x00;
}


static void 
fx5600_recalc_mapping(fx5600_t *fx5600)
{
	svga_t *svga = &fx5600->svga;
		
	if (!(fx5600->pci_regs[PCI_REG_COMMAND] & PCI_COMMAND_MEM)) {
	//pclog("PCI mem off\n");
		mem_mapping_disable(&svga->mapping);
		mem_mapping_disable(&fx5600->mmio_mapping);
		mem_mapping_disable(&fx5600->linear_mapping);
	return;
	}

	//pclog("PCI mem on\n");
	//pclog("fx5600->mmio_base = %08X\n", fx5600->mmio_base);
	if (fx5600->mmio_base)
		mem_mapping_set_addr(&fx5600->mmio_mapping, fx5600->mmio_base, 0x1000000);
	else
		mem_mapping_disable(&fx5600->mmio_mapping);

	//pclog("fx5600->lfb_base = %08X\n", fx5600->lfb_base);
	if (fx5600->lfb_base) {
	mem_mapping_set_addr(&fx5600->linear_mapping, fx5600->lfb_base, 0x1000000);
	} else {
		mem_mapping_disable(&fx5600->linear_mapping);
	}

	switch (svga->gdcreg[6] & 0x0c) {
	case 0x0: /*128k at A0000*/
		mem_mapping_set_addr(&svga->mapping, 0xa0000, 0x20000);
		svga->banked_mask = 0xffff;
		break;
	case 0x4: /*64k at A0000*/
		mem_mapping_set_addr(&svga->mapping, 0xa0000, 0x10000);
		svga->banked_mask = 0xffff;
		break;
	case 0x8: /*32k at B0000*/
		mem_mapping_set_addr(&svga->mapping, 0xb0000, 0x08000);
		svga->banked_mask = 0x7fff;
		break;
	case 0xC: /*32k at B8000*/
		mem_mapping_set_addr(&svga->mapping, 0xb8000, 0x08000);
		svga->banked_mask = 0x7fff;
		break;
	}
}


static void 
fx5600_pci_write(int func, int addr, uint8_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

    //pclog("FX 5600 PCI write %02x %02x\n", addr, val);

	switch (addr) {
	case PCI_REG_COMMAND:
		fx5600->pci_regs[PCI_REG_COMMAND] = val & 0x37;
		io_removehandler(0x03c0, 0x0020, fx5600_in, NULL, NULL, fx5600_out, NULL, NULL, fx5600);
		if (val & PCI_COMMAND_IO)
			io_sethandler(0x03c0, 0x0020, fx5600_in, NULL, NULL, fx5600_out, NULL, NULL, fx5600);
		fx5600_recalc_mapping(fx5600);
		break;

	case 0x05:
		fx5600->pci_regs[0x05] = val & 0x01;
		break;

	case 0x13:
		fx5600->mmio_base = val << 24;
		fx5600_recalc_mapping(fx5600);
		break;

	case 0x17: 
		fx5600->lfb_base = val << 24;
		fx5600_recalc_mapping(fx5600);
		break;

	case 0x30: case 0x32: case 0x33:
		fx5600->pci_regs[addr] = val;
		if (fx5600->pci_regs[0x30] & 0x01) {
			uint32_t addr = (fx5600->pci_regs[0x32] << 16) | (fx5600->pci_regs[0x33] << 24);
			mem_mapping_set_addr(&fx5600->bios_rom.mapping, addr, 0x10000);
		} else
			mem_mapping_disable(&fx5600->bios_rom.mapping);
		break;

	case 0x3c:
		fx5600->int_line = val;
		break;

	case 0x40: case 0x41: case 0x42: case 0x43:
		/* 0x40-0x43 are ways to write to 0x2c-0x2f */
		fx5600->pci_regs[0x2c + (addr & 0x03)] = val;
		break;
	}
}

uint32_t
fx5600_pmc_recompute_intr(void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	uint32_t intr = 0;
	if(fx5600->pfifo.intr & fx5600->pfifo.intr_en) intr |= (1 << 8);
	if(fx5600->ptimer.intr & fx5600->ptimer.intr_en) intr |= (1 << 20);
	if(fx5600->pcrtc.intr & fx5600->pcrtc.intr_en) intr |= (1 << 24);
	if(fx5600->pmc.intr & (1u << 31)) intr |= (1u << 31);
	
	if((intr & 0x7fffffff) && (fx5600->pmc.intr_en & 1)) pci_set_irq(fx5600->card, PCI_INTA);
	else if((intr & (1 << 31)) && (fx5600->pmc.intr_en & 2)) pci_set_irq(fx5600->card, PCI_INTA);
	//else pci_clear_irq(fx5600->card, PCI_INTA);
	return intr;
}

uint32_t
fx5600_pmc_read(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	switch(addr)
	{
	case 0x000000:
		return 0x20104010; //ID register.
	case 0x000100:
		return fx5600_pmc_recompute_intr(fx5600);
	case 0x000140:
		return fx5600->pmc.intr_en;
	case 0x000200:
		return fx5600->pmc.enable;
	}
	return 0;
}

void
fx5600_pmc_write(uint32_t addr, uint32_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	switch(addr)
	{
	case 0x000100:
		fx5600->pmc.intr = val & (1u << 31);
		fx5600_pmc_recompute_intr(fx5600);
		break;
	case 0x000140:
		fx5600->pmc.intr_en = val & 3;
		fx5600_pmc_recompute_intr(fx5600);
		break;
	case 0x000200:
		fx5600->pmc.enable = val;
		break;
	}
}

uint32_t
fx5600_pfifo_read(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	switch(addr)
	{
	case 0x002080:
		return fx5600->pfifo.cache_error;
	case 0x002100:
		return fx5600->pfifo.intr;
	case 0x002140:
		return fx5600->pfifo.intr_en;
	}
	return 0;
}

void
fx5600_pfifo_write(uint32_t addr, uint32_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	switch(addr)
	{
	case 0x002100:
	{
		uint32_t tmp = fx5600->pfifo.intr & ~val;
		fx5600->pfifo.intr = tmp;
		pci_clear_irq(fx5600->card, PCI_INTA);
		if(!(fx5600->pfifo.intr & 1)) fx5600->pfifo.cache_error = 0;
		break;
	}
	case 0x002140:
		fx5600->pfifo.intr_en = val & 0x11111;
		fx5600_pmc_recompute_intr(fx5600);
		break;
	case 0x003000:
		fx5600->pfifo.caches[0].push_enabled = val & 1;
		break;
	case 0x003010:
		fx5600->pfifo.caches[0].put = val;
		break;
	case 0x003050:
		fx5600->pfifo.caches[0].pull_enabled = val & 1;
		break;
	case 0x003070:
		fx5600->pfifo.caches[0].get = val;
		break;
	case 0x003200:
		fx5600->pfifo.caches[1].push_enabled = val & 1;
		break;
	case 0x003210:
		fx5600->pfifo.caches[1].put = val;
		break;
	case 0x003250:
		fx5600->pfifo.caches[1].pull_enabled = val & 1;
		break;
	case 0x003270:
		fx5600->pfifo.caches[1].get = val;
		break;
	}
}

void
fx5600_ptimer_interrupt(int num, void *p)
{
	//fx_5600_log("FX 5600 PTIMER interrupt #%d fired!\n", num);
	fx5600_t *fx5600 = (fx5600_t *)p;

	fx5600->ptimer.intr |= (1 << num);

	fx5600_pmc_recompute_intr(fx5600);
}

uint32_t
fx5600_ptimer_read(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	switch(addr)
	{
	case 0x009100:
		return fx5600->ptimer.intr;
	case 0x009140:
		return fx5600->ptimer.intr_en;
	case 0x009200:
		return fx5600->ptimer.clock_div;
	case 0x009210:
		return fx5600->ptimer.clock_mul;
	case 0x009400:
		return fx5600->ptimer.time & 0xffffffffULL;
	case 0x009410:
		return fx5600->ptimer.time >> 32;
	case 0x009420:
		return fx5600->ptimer.alarm;
	}
	return 0;
}

void
fx5600_ptimer_write(uint32_t addr, uint32_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	switch(addr)
	{
	case 0x009100:
		fx5600->ptimer.intr &= ~val;
		fx5600_pmc_recompute_intr(fx5600);
		break;
	case 0x009140:
		fx5600->ptimer.intr_en = val & 1;
		fx5600_pmc_recompute_intr(fx5600);
		break;
	case 0x009200:
		if(!(uint16_t)val) val = 1;
		fx5600->ptimer.clock_div = (uint16_t)val;
		break;
	case 0x009210:
		fx5600->ptimer.clock_mul = (uint16_t)val;
		break;
	case 0x009400:
		fx5600->ptimer.time &= 0x0fffffff00000000ULL;
		fx5600->ptimer.time |= val & 0xffffffe0;
		break;
	case 0x009410:
		fx5600->ptimer.time &= 0xffffffe0;
		fx5600->ptimer.time |= (uint64_t)(val & 0x0fffffff) << 32;
		break;
	case 0x009420:
		fx5600->ptimer.alarm = val & 0xffffffe0;
		//HACK to make wfw3.11 not take forever to start
		if(val == 0xffffffff)
		{
			fx5600_ptimer_interrupt(0, fx5600);
		}
		break;
	}
}

uint32_t
fx5600_pfb_read(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	switch(addr)
	{
		case 0x100000:
			switch(fx5600->vram_size)
			{
				case 4 << 20: return 0x15;
				case 8 << 20: return 0x16;
				case 16 << 20: return 0x17;
			}
			break;
	}

	return 0;
}

uint32_t
fx5600_pextdev_read(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	switch(addr)
	{
		case 0x101000:
			return 0x0000019e;
	}

	return 0;
}

uint32_t
fx5600_pcrtc_read(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	switch(addr)
	{
		case 0x600100:
			return fx5600->pcrtc.intr;
		case 0x600140:
			return fx5600->pcrtc.intr_en;
	}
	return 0;
}

void
fx5600_pcrtc_write(uint32_t addr, uint32_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	switch(addr)
	{
		case 0x600100:
			fx5600->pcrtc.intr &= ~val;
			fx5600_pmc_recompute_intr(fx5600);
			break;
		case 0x600140:
			fx5600->pcrtc.intr_en = val & 1;
			fx5600_pmc_recompute_intr(fx5600);
			break;
	}
}

uint32_t
fx5600_pramdac_read(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	switch(addr)
	{
		case 0x680500:
			return fx5600->pramdac.nvpll;
		case 0x680504:
			return fx5600->pramdac.mpll;
		case 0x680508:
			return fx5600->pramdac.vpll;
	}
	return 0;
}

void
fx5600_pramdac_write(uint32_t addr, uint32_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	switch(addr)
	{
		case 0x680500:
			fx5600->pramdac.nvpll = val;
			break;
		case 0x680504:
			fx5600->pramdac.mpll = val;
			break;
		case 0x680508:
			fx5600->pramdac.vpll = val;
			break;
	}
	svga_recalctimings(&fx5600->svga);
}

void
fx5600_ptimer_tick(void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	//pclog("[FX 5600] PTIMER tick! mul %04x div %04x\n", fx5600->ptimer.clock_mul, fx5600->ptimer.clock_div);

	double time = ((double)fx5600->ptimer.clock_mul * 10.0) / (double)fx5600->ptimer.clock_div; //Multiply by 10 to avoid timer system limitations.
	uint32_t tmp;
	int alarm_check;

	//if(cs == 0x0008 && !fx5600->pgraph.beta) nv_riva_log("FX 5600 PTIMER time elapsed %f alarm %08x, time_low %08x\n", time, fx5600->ptimer.alarm, fx5600->ptimer.time & 0xffffffff);

	tmp = fx5600->ptimer.time;
	fx5600->ptimer.time += (uint64_t)time;

	alarm_check = (uint32_t)(fx5600->ptimer.time - fx5600->ptimer.alarm) & 0x80000000;

	//alarm_check = ((uint32_t)fx5600->ptimer.time >= (uint32_t)fx5600->ptimer.alarm);

	//pclog("[FX 5600] Timer %08x %016llx %08x %d\n", fx5600->ptimer.alarm, fx5600->ptimer.time, tmp, alarm_check);

	if(alarm_check)
	{
		pclog("[FX 5600] PTIMER ALARM interrupt fired!\n");
		fx5600_ptimer_interrupt(0, fx5600);
	}
}

void
fx5600_nvclk_poll(void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	fx5600_ptimer_tick(fx5600);
	timer_on_auto(&fx5600->nvtimer, fx5600->nvtime);
}

void
fx5600_mclk_poll(void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	timer_on_auto(&fx5600->mtimer, fx5600->mtime);
}

uint32_t
fx5600_mmio_read_l(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	addr &= 0xffffff;

	uint32_t ret = 0;

	switch(addr) {
	case 0x6013b4: case 0x6013b5:
	case 0x6013d4: case 0x6013d5:
	case 0x6013da:
	case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
	case 0x6813c6: case 0x6813c7: case 0x6813c8: case 0x6813c9: case 0x6813ca: case 0x6813cb:
		ret = (fx5600_in((addr+0) & 0x3ff,p) << 0) | (fx5600_in((addr+1) & 0x3ff,p) << 8) | (fx5600_in((addr+2) & 0x3ff,p) << 16) | (fx5600_in((addr+3) & 0x3ff,p) << 24);
		break;
	}

	addr &= 0xfffffc;

	if ((addr >= 0x000000) && (addr <= 0x000fff)) ret = fx5600_pmc_read(addr, fx5600);
	if ((addr >= 0x002000) && (addr <= 0x003fff)) ret = fx5600_pfifo_read(addr, fx5600);
	if ((addr >= 0x009000) && (addr <= 0x009fff)) ret = fx5600_ptimer_read(addr, fx5600);
	if ((addr >= 0x100000) && (addr <= 0x100fff)) ret = fx5600_pfb_read(addr, fx5600);
	if ((addr >= 0x101000) && (addr <= 0x101fff)) ret = fx5600_pextdev_read(addr, fx5600);
	if ((addr >= 0x600000) && (addr <= 0x600fff)) ret = fx5600_pcrtc_read(addr, fx5600);
	if ((addr >= 0x680000) && (addr <= 0x680fff)) ret = fx5600_pramdac_read(addr, fx5600);
	if ((addr >= 0x700000) && (addr <= 0x7fffff)) ret = fx5600->ramin[(addr & 0xfffff) >> 2];
	if ((addr >= 0x300000) && (addr <= 0x30ffff)) ret = ((uint32_t *) fx5600->bios_rom.rom)[(addr & fx5600->bios_rom.mask) >> 2];

	if ((addr >= 0x1800) && (addr <= 0x18ff))
		ret = (fx5600_pci_read(0,(addr+0) & 0xff,p) << 0) | (fx5600_pci_read(0,(addr+1) & 0xff,p) << 8) | (fx5600_pci_read(0,(addr+2) & 0xff,p) << 16) | (fx5600_pci_read(0,(addr+3) & 0xff,p) << 24);

	if(addr != 0x9400) pclog("[FX 5600] MMIO read %08x returns value %08x\n", addr, ret);

	return ret;
}


uint8_t
fx5600_mmio_read(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	addr &= 0xffffff;

	if ((addr >= 0x300000) && (addr <= 0x30ffff)) return fx5600->bios_rom.rom[addr & fx5600->bios_rom.mask];

	if ((addr >= 0x1800) && (addr <= 0x18ff))
	return fx5600_pci_read(0,addr & 0xff,p);

	switch(addr) {
	case 0x6013b4: case 0x6013b5:
	case 0x6013d4: case 0x6013d5:
	case 0x6013da:
	case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
	case 0x6813c6: case 0x6813c7: case 0x6813c8: case 0x6813c9: case 0x6813ca: case 0x6813cb:
		return fx5600_in(addr & 0x3ff,p);
		break;
	}

	return (fx5600_mmio_read_l(addr & 0xffffff, fx5600) >> ((addr & 3) << 3)) & 0xff;
}


uint16_t
fx5600_mmio_read_w(uint32_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	addr &= 0xffffff;

	if ((addr >= 0x300000) && (addr <= 0x30ffff)) return ((uint16_t *) fx5600->bios_rom.rom)[(addr & fx5600->bios_rom.mask) >> 1];

	if ((addr >= 0x1800) && (addr <= 0x18ff))
	return (fx5600_pci_read(0,(addr+0) & 0xff,p) << 0) | (fx5600_pci_read(0,(addr+1) & 0xff,p) << 8);

	switch(addr) {
	case 0x6013b4: case 0x6013b5:
	case 0x6013d4: case 0x6013d5:
	case 0x6013da:
	case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
	case 0x6813c6: case 0x6813c7: case 0x6813c8: case 0x6813c9: case 0x6813ca: case 0x6813cb:
		return (fx5600_in((addr+0) & 0x3ff,p) << 0) | (fx5600_in((addr+1) & 0x3ff,p) << 8);
		break;
	}

   return (fx5600_mmio_read_l(addr & 0xffffff, fx5600) >> ((addr & 3) << 3)) & 0xffff;
}


void
fx5600_mmio_write_l(uint32_t addr, uint32_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	addr &= 0xffffff;

	pclog("[FX 5600] MMIO write %08x %08x\n", addr, val);

	if ((addr >= 0x1800) && (addr <= 0x18ff)) {
	fx5600_pci_write(0, addr & 0xff, val & 0xff, p);
	fx5600_pci_write(0, (addr+1) & 0xff, (val>>8) & 0xff, p);
	fx5600_pci_write(0, (addr+2) & 0xff, (val>>16) & 0xff, p);
	fx5600_pci_write(0, (addr+3) & 0xff, (val>>24) & 0xff, p);
	return;
	}

	if((addr >= 0x000000) && (addr <= 0x000fff)) fx5600_pmc_write(addr, val, fx5600);
	if((addr >= 0x002000) && (addr <= 0x003fff)) fx5600_pfifo_write(addr, val, fx5600);
	if((addr >= 0x009000) && (addr <= 0x009fff)) fx5600_ptimer_write(addr, val, fx5600);
	if((addr >= 0x600000) && (addr <= 0x600fff)) fx5600_pcrtc_write(addr, val, fx5600);
	if((addr >= 0x680000) && (addr <= 0x680fff)) fx5600_pramdac_write(addr, val, fx5600);
	if((addr >= 0x700000) && (addr <= 0x7fffff)) fx5600->ramin[(addr & 0xfffff) >> 2] = val;

	switch(addr) {
	case 0x6013b4: case 0x6013b5:
	case 0x6013d4: case 0x6013d5:
	case 0x6013da:
	case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
	case 0x6813c6: case 0x6813c7: case 0x6813c8: case 0x6813c9: case 0x6813ca: case 0x6813cb:
		fx5600_out(addr & 0xfff, val & 0xff, p);
		fx5600_out((addr+1) & 0xfff, (val>>8) & 0xff, p);
		fx5600_out((addr+2) & 0xfff, (val>>16) & 0xff, p);
		fx5600_out((addr+3) & 0xfff, (val>>24) & 0xff, p);
		break;
	}
}


void
fx5600_mmio_write(uint32_t addr, uint8_t val, void *p)
{
	uint32_t tmp;

	addr &= 0xffffff;

	switch(addr) {
	case 0x6013b4: case 0x6013b5:
	case 0x6013d4: case 0x6013d5:
	case 0x6013da:
	case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
	case 0x6813c6: case 0x6813c7: case 0x6813c8: case 0x6813c9: case 0x6813ca: case 0x6813cb:
		fx5600_out(addr & 0xfff, val & 0xff, p);
		return;
	}

	tmp = fx5600_mmio_read_l(addr,p);
	tmp &= ~(0xff << ((addr & 3) << 3));
	tmp |= val << ((addr & 3) << 3);
	fx5600_mmio_write_l(addr, tmp, p);
	if ((addr >= 0x1800) && (addr <= 0x18ff)) fx5600_pci_write(0, addr & 0xff, val, p);
}


void
fx5600_mmio_write_w(uint32_t addr, uint16_t val, void *p)
{
	uint32_t tmp;

	if ((addr >= 0x1800) && (addr <= 0x18ff)) {
	fx5600_pci_write(0, addr & 0xff, val & 0xff, p);
	fx5600_pci_write(0, (addr+1) & 0xff, (val>>8) & 0xff, p);
	return;
	}

	addr &= 0xffffff;
	tmp = fx5600_mmio_read_l(addr,p);
	tmp &= ~(0xffff << ((addr & 3) << 3));
	tmp |= val << ((addr & 3) << 3);
	fx5600_mmio_write_l(addr, tmp, p);
}

uint8_t
fx5600_rma_in(uint16_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	svga_t *svga = &fx5600->svga;
	uint8_t ret = 0;

	addr &= 0xff;

	// nv_riva_log("FX 5600 RMA read %04X %04X:%08X\n", addr, CS, cpu_state.pc);

	switch(addr) {
	case 0x00:
		ret = 0x65;
		break;
	case 0x01:
		ret = 0xd0;
		break;
	case 0x02:
		ret = 0x16;
		break;
	case 0x03:
		ret = 0x2b;
		break;
	case 0x08:
	case 0x09:
	case 0x0a:
	case 0x0b:
		if (fx5600->rma.rma_dst_addr < 0x1000000)
			ret = fx5600_mmio_read((fx5600->rma.rma_dst_addr + (addr & 3)) & 0xffffff, fx5600);
		else
			ret = svga_read_linear((fx5600->rma.rma_dst_addr - 0x1000000) & 0xffffff, svga);
		break;
	}

	return ret;
}


void
fx5600_rma_out(uint16_t addr, uint8_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	svga_t* svga = &fx5600->svga;

	addr &= 0xff;

	// nv_riva_log("FX 5600 RMA write %04X %02X %04X:%08X\n", addr, val, CS, cpu_state.pc);

	switch(addr) {
	case 0x04:
		fx5600->rma.rma_dst_addr &= ~0xff;
		fx5600->rma.rma_dst_addr |= val;
		break;
	case 0x05:
		fx5600->rma.rma_dst_addr &= ~0xff00;
		fx5600->rma.rma_dst_addr |= (val << 8);
		break;
	case 0x06:
		fx5600->rma.rma_dst_addr &= ~0xff0000;
		fx5600->rma.rma_dst_addr |= (val << 16);
		break;
	case 0x07:
		fx5600->rma.rma_dst_addr &= ~0xff000000;
		fx5600->rma.rma_dst_addr |= (val << 24);
		break;
	case 0x08:
	case 0x0c:
	case 0x10:
	case 0x14:
		fx5600->rma.rma_data &= ~0xff;
		fx5600->rma.rma_data |= val;
		break;
	case 0x09:
	case 0x0d:
	case 0x11:
	case 0x15:
		fx5600->rma.rma_data &= ~0xff00;
		fx5600->rma.rma_data |= (val << 8);
		break;
	case 0x0a:
	case 0x0e:
	case 0x12:
	case 0x16:
		fx5600->rma.rma_data &= ~0xff0000;
		fx5600->rma.rma_data |= (val << 16);
		break;
	case 0x0b:
	case 0x0f:
	case 0x13:
	case 0x17:
		fx5600->rma.rma_data &= ~0xff000000;
		fx5600->rma.rma_data |= (val << 24);
		if (fx5600->rma.rma_dst_addr < 0x1000000)
			fx5600_mmio_write_l(fx5600->rma.rma_dst_addr & 0xffffff, fx5600->rma.rma_data, fx5600);
		else
			svga_writel_linear((fx5600->rma.rma_dst_addr - 0x1000000) & 0xffffff, fx5600->rma.rma_data, svga);
		break;
	}

	if (addr & 0x10)
	fx5600->rma.rma_dst_addr+=4;
}


static void
fx5600_out(uint16_t addr, uint8_t val, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	svga_t *svga = &fx5600->svga;
	uint8_t old;

	if ((addr >= 0x3d0) && (addr <= 0x3d3)) {
	fx5600->rma.rma_access_reg[addr & 3] = val;
	if(!(fx5600->rma.rma_mode & 1))
		return;
	fx5600_rma_out(((fx5600->rma.rma_mode & 0xe) << 1) + (addr & 3), fx5600->rma.rma_access_reg[addr & 3], fx5600);
	}

	if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0) && !(svga->miscout & 1))
	addr ^= 0x60;

	switch (addr) {
	case 0x3D4:
		svga->crtcreg = val;
		return;
	case 0x3D5:
		if ((svga->crtcreg < 7) && (svga->crtc[0x11] & 0x80))
			return;
		if ((svga->crtcreg == 7) && (svga->crtc[0x11] & 0x80))
			val = (svga->crtc[7] & ~0x10) | (val & 0x10);
		old = svga->crtc[svga->crtcreg];
		svga->crtc[svga->crtcreg] = val;
		if(svga->seqregs[0x06] == 0x57)
		{
			switch(svga->crtcreg) {
				case 0x1e:
					fx5600->read_bank = val;
					if (svga->chain4) svga->read_bank = fx5600->read_bank << 15;
					else              svga->read_bank = fx5600->read_bank << 13;
					break;
				case 0x1d:
					fx5600->write_bank = val;
					if (svga->chain4) svga->write_bank = fx5600->write_bank << 15;
					else              svga->write_bank = fx5600->write_bank << 13;
					break;
				case 0x19: case 0x1a: case 0x25: case 0x28:
				case 0x2d:
					svga_recalctimings(svga);
					break;
				case 0x38:
					fx5600->rma.rma_mode = val & 0xf;
					break;
				case 0x3f:
					i2c_gpio_set(fx5600->i2c, !!(val & 0x20), !!(val & 0x10));
					break;
			}
		}
		//if (svga->crtcreg > 0x18)
			// pclog("FX 5600 Extended CRTC write %02X %02x\n", svga->crtcreg, val);
		if (old != val) {
			if ((svga->crtcreg < 0xe) || (svga->crtcreg > 0x10)) {
				svga->fullchange = changeframecount;
				svga_recalctimings(svga);
			}
		}
		break;
	}

	svga_out(addr, val, svga);
}


static uint8_t
fx5600_in(uint16_t addr, void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	svga_t *svga = &fx5600->svga;
	uint8_t temp;

	if ((addr >= 0x3d0) && (addr <= 0x3d3)) {
	if (!(fx5600->rma.rma_mode & 1))
		return 0x00;
	return fx5600_rma_in(((fx5600->rma.rma_mode & 0xe) << 1) + (addr & 3), fx5600);
	}

	if (((addr&0xFFF0) == 0x3D0 || (addr&0xFFF0) == 0x3B0) && !(svga->miscout&1)) addr ^= 0x60;

	switch (addr) {
	case 0x3D4:
		temp = svga->crtcreg;
		break;
	case 0x3D5:
		switch(svga->crtcreg) {
			case 0x3e:
					/* DDC status register */
				temp = (i2c_gpio_get_sda(fx5600->i2c) << 3) | (i2c_gpio_get_scl(fx5600->i2c) << 2);
				break;
			default:
				temp = svga->crtc[svga->crtcreg];
				break;
		}
		break;
	default:
		temp = svga_in(addr, svga);
		break;
	}

	return temp;
}

static void
fx5600_recalctimings(svga_t *svga)
{
	fx5600_t *fx5600 = (fx5600_t *)svga->p;

	svga->ma_latch += (svga->crtc[0x19] & 0x1f) << 16;
	svga->rowoffset += (svga->crtc[0x19] & 0xe0) << 3;
	if (svga->crtc[0x25] & 0x01) svga->vtotal      += 0x400;
	if (svga->crtc[0x25] & 0x02) svga->dispend     += 0x400;
	if (svga->crtc[0x25] & 0x04) svga->vblankstart += 0x400;
	if (svga->crtc[0x25] & 0x08) svga->vsyncstart  += 0x400;
	if (svga->crtc[0x25] & 0x10) svga->htotal      += 0x100;
	if (svga->crtc[0x2d] & 0x01) svga->hdisp       += 0x100;	
	/* The effects of the large screen bit seem to just be doubling the row offset.
	   However, these large modes still don't work. Possibly core SVGA bug? It does report 640x2 res after all. */

	switch(svga->crtc[0x28] & 3) {
	case 1:
		svga->bpp = 8;
		svga->lowres = 0;
		svga->render = svga_render_8bpp_highres;
		break;
	case 2:
		svga->bpp = 16;
		svga->lowres = 0;
		svga->render = svga_render_16bpp_highres;
		break;
	case 3:
		svga->bpp = 32;
		svga->lowres = 0;
		svga->render = svga_render_32bpp_highres;
		break;
	}

	double freq = 13500000.0;
	int m_m = fx5600->pramdac.mpll & 0xff;
	int m_n = (fx5600->pramdac.mpll >> 8) & 0xff;
	int m_p = (fx5600->pramdac.mpll >> 16) & 7;

	if(m_n == 0) m_n = 1;
	if(m_m == 0) m_m = 1;

	freq = (freq * m_n) / (m_m << m_p);
	fx5600->mtime = 10000000.0 / freq; //Multiply period by 10 to work around timer system limitations.
	timer_on_auto(&fx5600->mtimer, fx5600->mtime);

	freq = 13500000;
	int nv_m = fx5600->pramdac.nvpll & 0xff;
	int nv_n = (fx5600->pramdac.nvpll >> 8) & 0xff;
	int nv_p = (fx5600->pramdac.nvpll >> 16) & 7;

	if(nv_n == 0) nv_n = 1;
	if(nv_m == 0) nv_m = 1;

	freq = (freq * nv_n) / (nv_m << nv_p);
	fx5600->nvtime = 10000000.0 / freq; //Multiply period by 10 to work around timer system limitations.
	timer_on_auto(&fx5600->nvtimer, fx5600->nvtime);

	freq = 13500000;
	int v_m = fx5600->pramdac.vpll & 0xff;
	int v_n = (fx5600->pramdac.vpll >> 8) & 0xff;
	int v_p = (fx5600->pramdac.vpll >> 16) & 7;

	if(v_n == 0) v_n = 1;
	if(v_m == 0) v_m = 1;

	freq = (freq * v_n) / (v_m << v_p);
	svga->clock = (cpuclock * (double)(1ull << 32)) / freq;
}

void
fx5600_vblank_start(svga_t *svga)
{
	fx5600_t *fx5600 = (fx5600_t *)svga->p;

	fx5600->pcrtc.intr |= 1;

	fx5600_pmc_recompute_intr(fx5600);
}

static void
*fx5600_init(const device_t *info)
{
	fx5600_t *fx5600 = malloc(sizeof(fx5600_t));
	svga_t *svga;
	char *romfn = BIOS_fx5600_PATH;
	memset(fx5600, 0, sizeof(fx5600_t));
	svga = &fx5600->svga;

	fx5600->vram_size = device_get_config_int("memory") << 20;
	fx5600->vram_mask = fx5600->vram_size - 1;

	svga_init(info, &fx5600->svga, fx5600, fx5600->vram_size,
		  fx5600_recalctimings, fx5600_in, fx5600_out,
		  NULL, NULL);

	svga->decode_mask = fx5600->vram_mask;

	rom_init(&fx5600->bios_rom, romfn, 0xc0000, 0x10000, 0xffff, 0, MEM_MAPPING_EXTERNAL);
	mem_mapping_disable(&fx5600->bios_rom.mapping);

	mem_mapping_add(&fx5600->mmio_mapping, 0, 0, fx5600_mmio_read, fx5600_mmio_read_w, fx5600_mmio_read_l, fx5600_mmio_write, fx5600_mmio_write_w, fx5600_mmio_write_l,  NULL, MEM_MAPPING_EXTERNAL, fx5600);
	mem_mapping_disable(&fx5600->mmio_mapping);
	mem_mapping_add(&fx5600->linear_mapping, 0, 0, svga_read_linear, svga_readw_linear, svga_readl_linear, svga_write_linear, svga_writew_linear, svga_writel_linear,  NULL, MEM_MAPPING_EXTERNAL, &fx5600->svga);
	mem_mapping_disable(&fx5600->linear_mapping);

	svga->vblank_start = fx5600_vblank_start;

	fx5600->card = pci_add_card(PCI_ADD_VIDEO, fx5600_pci_read, fx5600_pci_write, fx5600);

	fx5600->pci_regs[0x04] = 0x07;
	fx5600->pci_regs[0x05] = 0x00;
	fx5600->pci_regs[0x07] = 0x02;

	fx5600->pci_regs[0x30] = 0x00;
	fx5600->pci_regs[0x32] = 0x0c;
	fx5600->pci_regs[0x33] = 0x00;

	fx5600->pmc.intr_en = 1;

	//Default values for the RAMDAC PLLs
	fx5600->pramdac.mpll = 0x03c20d;
	fx5600->pramdac.nvpll = 0x03c20d;
	fx5600->pramdac.vpll = 0x03c20d;

	timer_add(&fx5600->nvtimer, fx5600_nvclk_poll, fx5600, 0);
	timer_add(&fx5600->mtimer, fx5600_mclk_poll, fx5600, 0);

	video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_fx5600);

	fx5600->i2c = i2c_gpio_init("ddc_fx5600");
	fx5600->ddc = ddc_init(i2c_gpio_get_bus(fx5600->i2c));

	return fx5600;
}


static int
fx5600_available(void)
{
	return rom_present(BIOS_fx5600_PATH);
}


void
fx5600_close(void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	
	svga_close(&fx5600->svga);

	free(fx5600->ramin);
	
	free(fx5600);
}


void
fx5600_speed_changed(void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;
	
	svga_recalctimings(&fx5600->svga);
}


void
fx5600_force_redraw(void *p)
{
	fx5600_t *fx5600 = (fx5600_t *)p;

	fx5600->svga.fullchange = changeframecount;
}


static const device_config_t fx5600_config[] =
{
		{
				.name = "memory",
				.description = "Memory size",
				.type = CONFIG_SELECTION,
				.selection =
				{
						{
								.description = "128 MB",
								.value = 128
						},
						{
								.description = "256 MB",
								.value = 256
						},
						{
								.description = ""
						}
				},
				.default_int = 256
		},
		{
				.type = -1
		}
};

const device_t fx5600_pci_device =
{
	"nVidia Geforce FX 5600 (PCI)",
	DEVICE_PCI,
	fx5600_DEVICE_ID,
	fx5600_init,
	fx5600_close, 
	NULL,
	{ fx5600_available },
	fx5600_speed_changed,
	fx5600_force_redraw,
	fx5600_config
};