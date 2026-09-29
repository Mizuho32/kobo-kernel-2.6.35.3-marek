Original kobo kernel was copied from here:
https://github.com/kobolabs/Kobo-Reader/blob/master/hw/imx507/linux-2.6.35.3.tar.gz

It is kernel for Kobo Glo, Mini and newer Touch but changes were tested only with Kobo Touch N905C.
Compilation was tested with Sourcery-G++ 2010-q1.

Changes:

1)

eInk display driver (frame buffer)

After enabling auto_update (with eink_enable_autoupdate util) the changes of the framebuffer are automatically updated on the screen.

Every time period (Hz/10) changes to the frame buffer are detected with the help of MMU (deffered I/O mechanism). It detects changed pages (blocks of 4kB). The CRC16 of smaller blocks (16x8 pixels) is calculated and finally only changed regions are updated.

2)

Keyboard driver.

Original driver was sending EV_KEY event but without EV_SYN events. It is fixed here.

3)

In the config - framebuffer console is enabled (you need to pass "console=tty0" parameter to the uboot (insted of "console=ttymxc0,115200").

4)

Touch driver (zForce infra red) is enhanced to support multitouch (two fingers). For compatibility with original kobo software it sends both - multitouch and singletouch events now.

5)

WiFi (SDIO) host-sleep support.

Every resume from suspend re-runs the MMC host controller's full reset (SDHCI_RESET_ALL), which was corrupting the onboard WiFi chip's bus width and clock auto-gate settings, and a follow-up fix for missed SDIO card interrupts after resume caused the CPU to lock up servicing a continuous interrupt storm instead. Both are fixed now in mx_sdhci.c (sdhci_reset()/sdhci_resume()), so the WiFi chip can stay powered and associated to its access point across a real suspend/resume cycle instead of needing to be reloaded from scratch every time.

The stock WiFi driver (dhd.ko) never implements suspend/resume power management itself, so a small separate module (drivers/mmc/host/dhd_hostsleep_hook/) attaches it at runtime without touching dhd.ko's own source or binary. See drivers/mmc/host/dhd_hostsleep_hook/dhd_hostsleep_hook.c and build_and_install.sh there for building/installing it.
