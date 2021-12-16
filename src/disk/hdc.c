/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Common code to handle all sorts of disk controllers.
 *
 *
 *
 * Authors:	Miran Grca, <mgrca8@gmail.com>
 *		Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *		Copyright 2016-2018 Miran Grca.
 *		Copyright 2017,2018 Fred N. van Kempen.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/machine.h>
#include <86box/device.h>
#include <86box/hdc.h>
#include <86box/hdc_ide.h>
#include <86box/hdd.h>


int	hdc_current;


#ifdef ENABLE_HDC_LOG
int hdc_do_log = ENABLE_HDC_LOG;


static void
hdc_log(const char *fmt, ...)
{
    va_list ap;

    if (hdc_do_log) {
	va_start(ap, fmt);
	pclog_ex(fmt, ap);
	va_end(ap);
    }
}
#else
#define hdc_log(fmt, ...)
#endif


static void *
null_init(const device_t *info)
{
    return(NULL);
}


static void
null_close(void *priv)
{
}


static const device_t null_device = {
    "Null HDC", 0, 0,
    null_init, null_close, NULL,
    { NULL }, NULL, NULL, NULL
};


static void *
inthdc_init(const device_t *info)
{
    return(NULL);
}


static void
inthdc_close(void *priv)
{
}


static const device_t inthdc_device = {
    "Internal controller", 0, 0,
    inthdc_init, inthdc_close, NULL,
    { NULL }, NULL, NULL, NULL
};


static const struct {
    const char		*internal_name;
    const device_t	*device;
} controllers[] = {
    { "none",			&null_device			},
    { "internal",		&inthdc_device			},
    { "st506_xt",		&st506_xt_xebec_device		},
    { "st506_xt_dtc5150x",	&st506_xt_dtc5150x_device	},
    { "st506_xt_st11_m",	&st506_xt_st11_m_device		},
    { "st506_xt_wd1002a_wx1",	&st506_xt_wd1002a_wx1_device	},
    { "st506_at",		&st506_at_wd1003_device		},
    { "st506_xt_st11_r",	&st506_xt_st11_r_device		},
    { "st506_xt_wd1002a_27x",	&st506_xt_wd1002a_27x_device	},
    { "esdi_at",		&esdi_at_wd1007vse1_device	},
    { "ide_isa",		&ide_isa_device			},
    { "ide_isa_2ch",		&ide_isa_2ch_device		},
    { "xtide_at",		&xtide_at_device		},
    { "xtide_at_386",		&xtide_at_386_device		},
    { "xtide_at_ps2",		&xtide_at_ps2_device		},
    { "xta_wdxt150",		&xta_wdxt150_device		},
    { "xtide_acculogic",	&xtide_acculogic_device		},
    { "xtide",			&xtide_device			},
    { "esdi_mca",		&esdi_ps2_device		},
    { "ide_pci",		&ide_pci_device			},
    { "ide_pci_2ch",		&ide_pci_2ch_device		},
    { "ide_vlb",		&ide_vlb_device			},
    { "ide_vlb_2ch",		&ide_vlb_2ch_device		},
    { "",			NULL				}
};


/* Initialize the 'hdc_current' value based on configured HDC name. */
void
hdc_init(void)
{
    hdc_log("HDC: initializing..\n");

    /* Zero all the hard disk image arrays. */
    hdd_image_init();
}


/* Reset the HDC, whichever one that is. */
void
hdc_reset(void)
{
    hdc_log("HDC: reset(current=%d, internal=%d)\n",
	hdc_current, (machines[machine].flags & MACHINE_HDC) ? 1 : 0);

    /* If we have a valid controller, add its device. */
    if (hdc_current > 1)
	device_add(controllers[hdc_current].device);

    /* Now, add the tertiary and/or quaternary IDE controllers. */
    if (ide_ter_enabled)
	device_add(&ide_ter_device);
    if (ide_qua_enabled)
	device_add(&ide_qua_device);
}


char *
hdc_get_internal_name(int hdc)
{
    return((char *) controllers[hdc].internal_name);
}


int
hdc_get_from_internal_name(char *s)
{
	int c = 0;
	
	while (strlen((char *) controllers[c].internal_name))
	{
		if (!strcmp((char *) controllers[c].internal_name, s))
			return c;
		c++;
	}
	
	return 0;
}


const device_t *
hdc_get_device(int hdc)
{
    return(controllers[hdc].device);
}


int
hdc_has_config(int hdc)
{
    const device_t *dev = hdc_get_device(hdc);

    if (dev == NULL) return(0);

    if (dev->config == NULL) return(0);

    return(1);
}


int
hdc_get_flags(int hdc)
{
    return(controllers[hdc].device->flags);
}


int
hdc_available(int hdc)
{
    return(device_available(controllers[hdc].device));
}