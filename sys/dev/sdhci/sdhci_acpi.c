/*-
 * Copyright (c) 2016 Johannes Lundberg <yohanesu75@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/stdarg.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <cam/mmc/mmcreg.h>
#include <cam/mmc/mmc_bus.h>
#include <dev/sdhci/sdhci.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include "sdhci_if.h"
#include "mmcbr_if.h"



struct sdhci_acpi_slot {
	const char *description;
};

struct sdhci_acpi_uid_slot {
	const char *hid;
	const char *uid;
	const struct sdhci_acpi_slot *slot;
};



static char *sdhci_acpi_ids[] = {
		"PNP0D40",
		NULL
};

static const struct sdhci_acpi_slot sdhci_acpi_slot_int_emmc = {
		"Intel(R) eMMC Controller - 80862294"
};

static const struct sdhci_acpi_slot sdhci_acpi_slot_int_sdio = {
		"Intel(R) SDIO Controller - 80862295"
};

static const struct sdhci_acpi_slot sdhci_acpi_slot_int_sd = {
		"Intel(R) SD Card Controller - 80862296"
};

static const struct sdhci_acpi_uid_slot sdhci_acpi_uids[] = {
		// HID        SLOT  DEVICE                         DESCRIPTION
		{ "80860F14", "1", &sdhci_acpi_slot_int_emmc }, // Intel(R) eMMC Controller - 80862294
		{ "80860F14", "2", &sdhci_acpi_slot_int_sdio }, // Intel(R) SDIO Controller - 80862295
		{ "80860F14", "3", &sdhci_acpi_slot_int_sd }, 	// Intel(R) SD Card Controller - 80862296
		{ NULL, NULL, NULL }
};

#define MAX_SLOTS	6
struct sdhci_acpi_softc {
	device_t	dev;			/* Controller device */
	ACPI_HANDLE	handle;

	// Copied from sdhci_fdt.c
	u_int		quirks;			/* Chip specific quirks */
	u_int		caps;			/* If we override SDHCI_CAPABILITIES */
	uint32_t	max_clk;		/* Max possible freq */
	struct resource *irq_res;	/* IRQ resource */
	void 		*intrhand;		/* Interrupt handle */
	int		num_slots;			/* Number of slots on this controller*/
	struct sdhci_slot slots[MAX_SLOTS];
	struct resource	*mem_res[MAX_SLOTS];	/* Memory resource */
};



static const struct sdhci_acpi_slot *sdhci_acpi_get_slot(const char *hid, const char *uid) {
	printf("%s:\n",__func__);
	const struct sdhci_acpi_uid_slot *u;
	for(u = sdhci_acpi_uids; u->hid != NULL; u++) {
		if(strcmp(u->hid, hid) != 0)
			continue;
		if(u->uid == NULL)
			return u->slot;
		if(uid != NULL && strcmp(u->uid, uid) == 0)
			return u->slot;
	}
	return NULL;
};

static void sdhci_acpi_intr(void *arg) {
	printf("%s:\n",__func__);
	struct sdhci_acpi_softc *sc = (struct sdhci_acpi_softc *)arg;
	int i;

	for (i = 0; i < sc->num_slots; i++) {
		struct sdhci_slot *slot = &sc->slots[i];
		sdhci_generic_intr(slot);
	}
}

static int sdhci_acpi_probe(device_t dev) {
	device_printf(dev, "%s:\n",__func__);

	struct sdhci_acpi_softc *sc = device_get_softc(dev);

	if (ACPI_ID_PROBE(device_get_parent(dev), dev, sdhci_acpi_ids) != NULL) {
		device_set_desc(dev, "Intel BayTrail SDIO/MMC Host Controller");

		sc->quirks = 0;
		// What does slots here mean? MMC+SD+SDIO = 3 slots?
		sc->num_slots = 1;
		sc->max_clk = 0;

		return BUS_PROBE_DEFAULT;
	}
	return ENXIO;
}

static int sdhci_acpi_attach(device_t dev) {
	device_printf(dev, "%s:\n",__func__);

	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	sc->dev = dev;

	ACPI_HANDLE devh;
	devh = acpi_get_handle(dev);
	sc->handle = devh;
	device_printf(dev, "%s: Got ACPI device handle\n",__func__);

    ACPI_DEVICE_INFO *devinfo;
    const char *hid, *uid;
    if(ACPI_FAILURE(AcpiGetObjectInfo(devh, &devinfo))) {
    	device_printf(dev, "%s: Failed to get devinfo\n",__func__);
    }
    if((devinfo->Valid & ACPI_VALID_HID) != 0) {
    	hid = devinfo->HardwareId.String;
    	uid = devinfo->UniqueId.String;
    	device_printf(dev, "%s: Got HID %s, UID %s\n",__func__,hid,uid);
    } else {
    	device_printf(dev, "%s: Failed to get HID/UID (devinfo not valid)\n",__func__);
    }

    // Break if other than MMC device
    if(strcmp(uid, "1") != 0)
    	return (ENXIO);

//    const struct sdhci_acpi_slot *slot = sdhci_acpi_get_slot(hid, uid);
//	device_printf(dev, "%s: Got device slot \"%s\"\n",__func__,slot->description);


	/* Allocate IRQ. */
	int err, slots, rid, i;
	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Can't allocate IRQ\n");
		return (ENOMEM);
	}

	/* Scan all slots. */
	slots = sc->num_slots;	/* number of slots determined in probe(). */
	sc->num_slots = 0;
	for (i = 0; i < slots; i++) {
		struct sdhci_slot *slot = &sc->slots[sc->num_slots];

		/* Allocate memory. */
		rid = 0;
		sc->mem_res[i] = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
							&rid, RF_ACTIVE);
		if (sc->mem_res[i] == NULL) {
			device_printf(dev, "Can't allocate memory for slot %d\n", i);
			continue;
		}

//		slot->quirks = sc->quirks;
//		slot->caps = sc->caps;
		slot->quirks = SDHCI_QUIRK_FORCE_DMA | SDHCI_QUIRK_LOWER_FREQUENCY;
		slot->caps = MMC_CAP_8_BIT_DATA | MMC_CAP_HSPEED;
		slot->max_clk = sc->max_clk;

		if (sdhci_init_slot(dev, slot, i) != 0)
			continue;

		sc->num_slots++;
	}
	device_printf(dev, "%s: %d slot(s) allocated\n", __func__, sc->num_slots);

	/* Activate the interrupt */
	err = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, sdhci_acpi_intr, sc, &sc->intrhand);
	if (err) {
		device_printf(dev, "Cannot setup IRQ\n");
		return (err);
	}

	/* Process cards detection. */
	for (i = 0; i < sc->num_slots; i++) {
		struct sdhci_slot *slot = &sc->slots[i];
		sdhci_start_slot(slot);
	}


	return (0);
}

static int sdhci_acpi_detach(device_t dev) {
	device_printf(dev, "%s:\n",__func__);
	return 0;
}

static int sdhci_acpi_resume(device_t dev) {
	device_printf(dev, "%s:\n",__func__);
	return 0;
}




static uint8_t
sdhci_acpi_read_1(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	return (bus_read_1(sc->mem_res[slot->num], off));
}

static void
sdhci_acpi_write_1(device_t dev, struct sdhci_slot *slot, bus_size_t off,
		  uint8_t val)
{
	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	bus_write_1(sc->mem_res[slot->num], off, val);
}

static uint16_t
sdhci_acpi_read_2(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	return (bus_read_2(sc->mem_res[slot->num], off));
}

static void
sdhci_acpi_write_2(device_t dev, struct sdhci_slot *slot, bus_size_t off,
		  uint16_t val)
{
	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	bus_write_2(sc->mem_res[slot->num], off, val);
}

static uint32_t
sdhci_acpi_read_4(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	return (bus_read_4(sc->mem_res[slot->num], off));
}

static void
sdhci_acpi_write_4(device_t dev, struct sdhci_slot *slot, bus_size_t off,
		  uint32_t val)
{
	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	bus_write_4(sc->mem_res[slot->num], off, val);
}

static void
sdhci_acpi_read_multi_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	bus_read_multi_4(sc->mem_res[slot->num], off, data, count);
}

static void
sdhci_acpi_write_multi_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct sdhci_acpi_softc *sc = device_get_softc(dev);
	bus_write_multi_4(sc->mem_res[slot->num], off, data, count);
}



static device_method_t sdhci_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, 		sdhci_acpi_probe),
	DEVMETHOD(device_attach,		sdhci_acpi_attach),
	DEVMETHOD(device_detach,		sdhci_acpi_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,		sdhci_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,		sdhci_generic_write_ivar),

	/* mmcbr_if */
	DEVMETHOD(mmcbr_update_ios, 	sdhci_generic_update_ios),
	DEVMETHOD(mmcbr_request, 		sdhci_generic_request),
	DEVMETHOD(mmcbr_get_ro, 		sdhci_generic_get_ro),
	DEVMETHOD(mmcbr_acquire_host, 	sdhci_generic_acquire_host),
	DEVMETHOD(mmcbr_release_host, 	sdhci_generic_release_host),

	/* SDHCI registers accessors */
	DEVMETHOD(sdhci_read_1,			sdhci_acpi_read_1),
	DEVMETHOD(sdhci_read_2,			sdhci_acpi_read_2),
	DEVMETHOD(sdhci_read_4,			sdhci_acpi_read_4),
	DEVMETHOD(sdhci_read_multi_4,	sdhci_acpi_read_multi_4),
	DEVMETHOD(sdhci_write_1,		sdhci_acpi_write_1),
	DEVMETHOD(sdhci_write_2,		sdhci_acpi_write_2),
	DEVMETHOD(sdhci_write_4,		sdhci_acpi_write_4),
	DEVMETHOD(sdhci_write_multi_4,	sdhci_acpi_write_multi_4),

	DEVMETHOD_END
};


static driver_t sdhci_acpi_driver = {
		"sdhci_acpi",
		sdhci_acpi_methods,
		sizeof(struct sdhci_acpi_softc)
};
static devclass_t sdhci_acpi_devclass;

DRIVER_MODULE(sdhci_acpi, acpi, sdhci_acpi_driver, sdhci_acpi_devclass, 0, 0);
MODULE_DEPEND(sdhci_acpi, sdhci, 1, 1, 1);
MODULE_DEPEND(sdhci_acpi, mmc, 1, 1, 1);

