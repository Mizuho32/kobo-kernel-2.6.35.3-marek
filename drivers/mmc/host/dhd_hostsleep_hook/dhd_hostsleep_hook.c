/*
 * dhd_hostsleep_hook: attaches a dev_pm_ops to the already-loaded, stock
 * bcmsdh_sdmmc driver (registered by dhd.ko) at runtime, without ever
 * touching dhd.ko's own source or binary.
 *
 * Why: Phase 2 of KoboWM's WiFi host-sleep work (see
 * mds/wifi-hostsleep/phase2-findings.md) needs bcmsdh_sdmmc_driver to have
 * a non-NULL dev_pm_ops, so that mmc_sdio_suspend() (drivers/mmc/core/sdio.c)
 * doesn't see func->dev.driver->pm == NULL and force a full card detach via
 * its -ENOSYS fallback. The straightforward way to do that is to patch
 * bcmsdh_sdmmc_linux.c and rebuild dhd.ko -- but mds/wifi-hostsleep/
 * phase5-attempt7-final-picture.md documents that ANY rebuild of dhd.ko
 * from this source tree (regardless of what's changed) risks triggering a
 * still-unexplained hardware-level hang (an SDIO command that never
 * completes), that the known-good official dhd.ko binary doesn't hit. So
 * instead of rebuilding dhd.ko, this separate, independently-built module
 * reaches into the driver-core structure the *already-loaded, untouched*
 * dhd.ko set up, and adds the dev_pm_ops there directly.
 *
 * How: sdio_bus_type (drivers/mmc/core/sdio_bus.c) is a static, unexported
 * symbol -- we can't reference it by name. But sdio_register_driver()
 * (exported) sets drv->drv.bus = &sdio_bus_type as a side effect before
 * calling driver_register(). So we register a harmless dummy sdio_driver
 * (empty ID table, never matches/binds to anything) purely to read back
 * its now-populated .drv.bus field, then use driver_find() (also exported)
 * to look up the real "bcmsdh_sdmmc" driver dhd.ko already registered, and
 * set *its* .drv.pm directly.
 *
 * Load this AFTER dhd.ko + insmod-ing sdio_wifi_pwr.ko/dhd.ko has already
 * bound the WiFi function to bcmsdh_sdmmc -- driver_find() will fail
 * (module load fails, harmlessly) if dhd.ko isn't loaded yet.
 *
 * UNTESTED on real hardware as of this file. See mds/wifi-hostsleep/.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/pm.h>
#include <linux/notifier.h>
#include <linux/netdevice.h>
#include <linux/mmc/sdio_func.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define TARGET_DRIVER_NAME "bcmsdh_sdmmc"
#define TARGET_MODULE_NAME "dhd"
#define TARGET_NETDEV_NAME "eth0"

/*
 * Minimal local copy of the handful of Broadcom dhd/wl ioctl ABI bits we
 * need (from kobo-src/bcm-dhd/src/include/wlioctl.h in the scratchpad,
 * not part of this kernel tree) -- just enough to issue WLC_SET_PM
 * ourselves, without pulling in dhd.ko's full private headers.
 */
typedef struct wl_ioctl {
	unsigned int cmd;
	void *buf;
	unsigned int len;
	unsigned char set;
	unsigned int used;
	unsigned int needed;
} wl_ioctl_t;

#define WLC_SET_PM 86
#define PM_FAST 2
#define WLC_SET_VAR 263	/* set named variable ("iovar") to value */

/*
 * mds/wifi-hostsleep/phase5-attempt9-*.md: disabling Phase 3's userspace
 * `iwconfig eth0 power on/off` (WLC_SET_PM via SIOCSIWPOWER) did NOT fix
 * the post-resume hang, so this isn't about restoring the PM_MAX/PM_OFF
 * value specifically. The remaining theory: dhd.ko's own suspend/resume
 * path (dhd_set_suspend(), normally wired to Android's early-suspend
 * framework, which isn't active here) is what the firmware and dhd's
 * internal SDIO command state machine actually expect to run at resume
 * time -- and our previous no-op .resume() never called anything, while
 * userspace `iwconfig` calls only run *after* mmc_resume_host() has fully
 * returned and unrelated resume activity has already raced in.
 *
 * dev_wlc_ioctl() in dhd.ko's own wl_iw.c issues WLC_* commands from
 * kernel context by building a fake ifreq pointing at a stack-local
 * wl_ioctl_t, forcing set_fs(KERNEL_DS) so copy_from_user() in
 * dhd_ioctl_entry() (dhd_linux.c) accepts it, then calling
 * dev->netdev_ops->ndo_do_ioctl(dev, &ifr, SIOCDEVPRIVATE) directly --
 * this reaches dhd.ko's *own*, already-loaded, unmodified ioctl handler,
 * no rebuild needed. We replicate exactly that trick here, from inside
 * our .resume() callback instead of wl_iw.c, so it runs synchronously
 * during mmc_resume_host() while the host is already claimed by the
 * resuming thread. That's safe, not a self-deadlock: mmc_claim_host()
 * (drivers/mmc/core/core.c) tracks host->claimer and re-enters cleanly
 * when host->claimer == current.
 */
static int dhd_wlc_ioctl_from_kernel(int cmd, void *arg, int len)
{
	struct net_device *dev;
	struct ifreq ifr;
	wl_ioctl_t ioc;
	mm_segment_t fs;
	int ret;

	dev = dev_get_by_name(&init_net, TARGET_NETDEV_NAME);
	if (!dev) {
		pr_warn("dhd_hostsleep_hook: %s not found, skipping ioctl 0x%x\n",
			TARGET_NETDEV_NAME, cmd);
		return -ENODEV;
	}

	memset(&ioc, 0, sizeof(ioc));
	ioc.cmd = cmd;
	ioc.buf = arg;
	ioc.len = len;
	ioc.set = 1;

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, dev->name, IFNAMSIZ - 1);
	ifr.ifr_data = (void __user *)&ioc;

	fs = get_fs();
	set_fs(get_ds());
	if (dev->netdev_ops && dev->netdev_ops->ndo_do_ioctl)
		ret = dev->netdev_ops->ndo_do_ioctl(dev, &ifr, SIOCDEVPRIVATE);
	else
		ret = -EOPNOTSUPP;
	set_fs(fs);

	pr_info("dhd_hostsleep_hook: kernel-issued ioctl 0x%x -> %d\n", cmd, ret);

	dev_put(dev);
	return ret;
}

/*
 * mds/wifi-hostsleep/phase5-attempt25-bcmdhd-reference-comparison.md: mainline
 * brcmfmac's brcmf_configure_wowl() doesn't just flip WLC_SET_PM -- it also
 * sets the "wowl"/"wowl_activate" *named variables* (iovars), which is what
 * actually tells the firmware "the host is about to disappear for real, not
 * just a longer beacon-poll interval" and is what flips the SDIO layer's
 * MMC_PM_KEEP_POWER negotiation on. Named variables (unlike WLC_SET_PM, a
 * fixed integer ioctl cmd) are sent as a single buffer: the variable's name,
 * NUL-terminated, immediately followed by its raw value bytes -- exactly
 * what the reference bcm_mkiovar() does (bcmutils.c). Replicated here
 * locally so no dhd.ko header/source is needed.
 */
static int dhd_iovar_set_from_kernel(const char *name, void *data, int datalen)
{
	char buf[64];
	int namelen = strlen(name) + 1;

	if (namelen + datalen > (int)sizeof(buf)) {
		pr_err("dhd_hostsleep_hook: iovar '%s' too long for local buffer\n",
		       name);
		return -ENOMEM;
	}

	memset(buf, 0, sizeof(buf));
	strncpy(buf, name, sizeof(buf));
	memcpy(&buf[namelen], data, datalen);

	return dhd_wlc_ioctl_from_kernel(WLC_SET_VAR, buf, namelen + datalen);
}

/*
 * One-shot, isolated probe (mds/wifi-hostsleep/phase5-attempt25-*.md "次の
 * 一手"): does this specific dhd.ko/firmware build even recognize the
 * "wowl_activate" iovar at all, BEFORE attempting any MMC-suspend-combined
 * test? Deliberately NOT wired into .suspend()/.resume() -- triggered only
 * by `echo 1 > /proc/kobowm_wowl_probe`, with no MMC suspend/resume
 * involved, same risk class as the WLC_SET_PM probes already done safely
 * many times this project. A clean negative (nonzero/negative return) means
 * this firmware doesn't support wowl and the approach is a dead end; a
 * clean 0 means it's worth trying the fuller PM_MAX+wowl_activate round
 * trip around an actual MMC suspend/resume next.
 */
static int kobowm_wowl_probe_write(struct file *file, const char *ubuf,
				    unsigned long count, void *data)
{
	int val = 1;
	int ret;

	ret = dhd_iovar_set_from_kernel("wowl_activate", &val, sizeof(val));
	pr_info("dhd_hostsleep_hook: wowl_activate probe -> %d\n", ret);

	return count;
}

static int hostsleep_suspend(struct device *dev)
{
	dev_info(dev, "dhd_hostsleep_hook: suspend\n");
	return 0;
}

static int hostsleep_resume(struct device *dev)
{
	dev_info(dev, "dhd_hostsleep_hook: resume\n");

	/* TEMPORARILY DISABLED: the WLC_SET_PM(PM_FAST) kernel-context
	 * injection below (see dhd_wlc_ioctl_from_kernel()'s comment for the
	 * rationale) was found to consistently fail with -EINVAL
	 * (kernel-issued ioctl 0x56 -> -22) on every resume during the
	 * mds/wifi-hostsleep/ MMC-only isolation testing (echo 1 >
	 * /proc/kobowm_mmc_test), and connectivity broke even on runs where
	 * no "Timeout waiting for hardware interrupt" was logged -- raising
	 * the possibility that this injection is itself contributing to (or
	 * at least confounding the diagnosis of) the resume breakage it was
	 * meant to fix. Disabled to get a clean read on the MMC subsystem's
	 * own behavior in isolation. See
	 * mds/wifi-hostsleep/phase5-attempt9-signal-enable-and-oops.md.
	 *
	 * int pm = PM_FAST;
	 * dhd_wlc_ioctl_from_kernel(WLC_SET_PM, &pm, sizeof(pm));
	 */

	return 0;
}

static const struct dev_pm_ops hostsleep_pm_ops = {
	.suspend = hostsleep_suspend,
	.resume  = hostsleep_resume,
};

/* Empty ID table: this driver is never meant to actually probe/bind to
 * anything, it exists solely so sdio_register_driver() populates
 * .drv.bus for us. */
static const struct sdio_device_id dummy_ids[] = {
	{ /* all-zero terminator, no real entries */ },
};

static int dummy_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	return -ENODEV;
}

static struct sdio_driver dummy_driver = {
	.probe    = dummy_probe,
	.name     = "dhd_hostsleep_hook_dummy",
	.id_table = dummy_ids,
};

static struct sdio_driver *target;
static DEFINE_SPINLOCK(target_lock);

/*
 * dhd.ko's module refcount is never pinned by our driver_find()/put_driver()
 * pair above (that only bumps the target *driver*'s kobject refcount, not
 * dhd.ko's own module refcount) -- confirmed on real hardware: `rmmod dhd`
 * succeeds while this hook is still loaded and still holding `target`. Once
 * that happens, `target` points into memory that belonged to dhd.ko's now
 * -unloaded image; touching it (e.g. in hostsleep_hook_exit()) would write
 * into freed/unmapped memory. Watch for dhd.ko going away via the module
 * notifier chain and drop our reference before that can happen.
 */
static int hostsleep_hook_module_notify(struct notifier_block *nb,
					  unsigned long action, void *data)
{
	struct module *mod = data;

	if (action == MODULE_STATE_GOING &&
	    strcmp(mod->name, TARGET_MODULE_NAME) == 0) {
		unsigned long flags;
		spin_lock_irqsave(&target_lock, flags);
		if (target) {
			pr_warn("dhd_hostsleep_hook: %s module going away, "
				"dropping stale driver reference\n",
				TARGET_MODULE_NAME);
			target = NULL;
		}
		spin_unlock_irqrestore(&target_lock, flags);
	}
	return NOTIFY_OK;
}

static struct notifier_block hostsleep_hook_module_nb = {
	.notifier_call = hostsleep_hook_module_notify,
};

static struct proc_dir_entry *wowl_probe_pe;

static int __init hostsleep_hook_init(void)
{
	struct device_driver *drv;
	int err;

	err = sdio_register_driver(&dummy_driver);
	if (err) {
		pr_err("dhd_hostsleep_hook: dummy sdio_register_driver failed: %d\n", err);
		return err;
	}

	drv = driver_find(TARGET_DRIVER_NAME, dummy_driver.drv.bus);
	if (!drv) {
		pr_err("dhd_hostsleep_hook: %s driver not found -- is dhd.ko loaded?\n",
		       TARGET_DRIVER_NAME);
		sdio_unregister_driver(&dummy_driver);
		return -ENODEV;
	}

	target = container_of(drv, struct sdio_driver, drv);

	if (target->drv.pm) {
		pr_warn("dhd_hostsleep_hook: %s already has dev_pm_ops (%p), not overwriting\n",
			TARGET_DRIVER_NAME, target->drv.pm);
		put_driver(drv);
		target = NULL;
		sdio_unregister_driver(&dummy_driver);
		return -EEXIST;
	}

	target->drv.pm = &hostsleep_pm_ops;

	register_module_notifier(&hostsleep_hook_module_nb);

	wowl_probe_pe = create_proc_entry("kobowm_wowl_probe", 0200, NULL);
	if (wowl_probe_pe)
		wowl_probe_pe->write_proc = kobowm_wowl_probe_write;
	else
		pr_warn("dhd_hostsleep_hook: failed to create /proc/kobowm_wowl_probe\n");

	pr_info("dhd_hostsleep_hook: attached dev_pm_ops to %s\n", TARGET_DRIVER_NAME);
	return 0;
}

static void __exit hostsleep_hook_exit(void)
{
	unsigned long flags;
	struct sdio_driver *t;

	if (wowl_probe_pe)
		remove_proc_entry("kobowm_wowl_probe", NULL);

	unregister_module_notifier(&hostsleep_hook_module_nb);

	spin_lock_irqsave(&target_lock, flags);
	t = target;
	target = NULL;
	spin_unlock_irqrestore(&target_lock, flags);

	if (t) {
		t->drv.pm = NULL;
		put_driver(&t->drv);
	}
	sdio_unregister_driver(&dummy_driver);
}

module_init(hostsleep_hook_init);
module_exit(hostsleep_hook_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Attaches dev_pm_ops to the stock bcmsdh_sdmmc driver without rebuilding dhd.ko");
MODULE_AUTHOR("KoboWM");
