// SPDX-License-Identifier: GPL-2.0
/*
 * Fullhan FH8626V100 DWC2 hardware integration.
 * The integrated PHY is powered and reset through PMU registers.
 * Copyright (C) 2026 Difan Zhang
 * Licensed under the GNU General Public License version 2.
 */
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/of.h>
#include <linux/usb/hcd.h>
#include <linux/soc/fullhan/fh8626-pmu.h>
#include "core.h"
#include "fh8626.h"

#ifdef CONFIG_DEBUG_FS
struct fh8626_usb_snapshot {
	u32 sts, mask, ahb, usb, pcg, port;
	unsigned long when;
	enum dwc2_fh8626_snapshot_reason reason;
	bool valid;
};
#endif

struct dwc2_fh8626 {
	struct clk *hclk;
	struct regulator *vbus;
	bool vbus_enabled;
#ifdef CONFIG_DEBUG_FS
	struct fh8626_usb_snapshot before_suspend;
	void __iomem *vbus_gpio_regs;
	u32 vbus_gpio_base;
	u32 vbus_gpio_bit;
#endif
};

#ifdef CONFIG_DEBUG_FS
#define FH8626_GPIO_DR		0x00
#define FH8626_GPIO_DDR		0x04
#define FH8626_GPIO_EXT		0x50
#define FH8626_GPIO_WINDOW	0x54

/*
 * GPIO data, direction and input remain accessible when USB is
 * clock-gated. Read them without claiming the regulator's GPIO or
 * touching interrupt registers.
 */
static void fh8626_init_gpio_observer(struct dwc2_hsotg *hsotg)
{
	struct dwc2_fh8626 *fh = hsotg->fh8626;
	struct device_node *supply, *controller;
	struct of_phandle_args args;
	struct resource res;
	u32 port;
	int ret;

	supply = of_parse_phandle(hsotg->dev->of_node, "vbus-supply", 0);
	if (!supply)
		return;
	ret = of_parse_phandle_with_args(supply, "gpio", "#gpio-cells", 0,
					 &args);
	of_node_put(supply);
	if (ret)
		return;
	controller = of_get_parent(args.np);
	if (!controller || args.args_count < 1 || args.args[0] >= 32 ||
	    of_property_read_u32(args.np, "reg", &port) || port ||
	    !of_device_is_compatible(controller, "fullhan,fh8626v100-gpio") ||
	    of_address_to_resource(controller, 0, &res) ||
	    resource_size(&res) < FH8626_GPIO_WINDOW ||
	    (res.start != 0xf0300000 && res.start != 0xf4000000))
		goto out;
	fh->vbus_gpio_regs = devm_ioremap(hsotg->dev, res.start,
					  FH8626_GPIO_WINDOW);
	if (fh->vbus_gpio_regs) {
		fh->vbus_gpio_base = res.start;
		fh->vbus_gpio_bit = args.args[0];
	}
out:
	of_node_put(controller);
	of_node_put(args.np);
}
#else
static inline void fh8626_init_gpio_observer(struct dwc2_hsotg *hsotg) { }
#endif

int dwc2_fh8626_init(struct dwc2_hsotg *hsotg)
{
	struct dwc2_fh8626 *fh;
	u32 val;
	int ret;

	if (!of_device_is_compatible(hsotg->dev->of_node,
				     "fullhan,fh8626v100-usb"))
		return 0;
	if (usb_get_dr_mode(hsotg->dev) != USB_DR_MODE_HOST)
		return -EINVAL;
	ret = fh8626_pmu_read(0x210, &val);
	if (ret)
		return ret;
	fh = devm_kzalloc(hsotg->dev, sizeof(*fh), GFP_KERNEL);
	if (!fh)
		return -ENOMEM;
	fh->hclk = devm_clk_get(hsotg->dev, "hclk");
	if (IS_ERR(fh->hclk))
		return PTR_ERR(fh->hclk);
	/* A dummy supply would hide the board's missing USB power switch. */
	if (!of_find_property(hsotg->dev->of_node, "vbus-supply", NULL)) {
		dev_err(hsotg->dev, "missing board vbus-supply\n");
		return -EINVAL;
	}
	fh->vbus = devm_regulator_get_optional(hsotg->dev, "vbus");
	if (IS_ERR(fh->vbus))
		return PTR_ERR(fh->vbus);
	hsotg->fh8626 = fh;
	fh8626_init_gpio_observer(hsotg);
	return 0;
}

int dwc2_fh8626_enable(struct dwc2_hsotg *hsotg)
{
	struct dwc2_fh8626 *fh = hsotg->fh8626;
	u32 value;
	int ret;

	if (!fh)
		return 0;
	ret = clk_prepare_enable(fh->hclk);
	if (ret)
		return ret;
	ret = fh8626_pmu_update_bits(0x210, BIT(31), 0);
	if (ret)
		goto err_clk;
	udelay(1000);
	ret = fh8626_pmu_update_bits(0x210, BIT(0) | BIT(4), BIT(0) | BIT(4));
	if (ret)
		goto err_clk;
	udelay(1000);
	ret = fh8626_pmu_update_bits(0x210, BIT(0) | BIT(4), 0);
	if (ret)
		goto err_clk;
	ret = fh8626_pmu_update_bits(0x218, 0x7, 0x7);
	if (ret)
		goto err_clk;
	ret = fh8626_pmu_update_bits(0x4c, BIT(19), 0);
	if (ret)
		goto err_clk;
	/* Read back to drain the reset write before starting the delay. */
	ret = fh8626_pmu_read(0x4c, &value);
	if (ret)
		goto err_clk;
	udelay(1000);
	ret = fh8626_pmu_update_bits(0x4c, BIT(19), BIT(19));
	if (ret)
		goto err_clk;
	ret = fh8626_pmu_read(0x4c, &value);
	if (ret)
		goto err_clk;
	msleep(20);
	ret = fh8626_pmu_update_bits(0x210, BIT(24), BIT(24));
	if (ret)
		goto err_clk;
	udelay(1000);
	/* Enable VBUS after the PHY reset sequence and before core reset. */
	if (!fh->vbus_enabled) {
		ret = regulator_enable(fh->vbus);
		if (ret) {
			dev_err(hsotg->dev,
				"failed to enable VBUS supply: %d\n", ret);
			goto err_clk;
		}
		fh->vbus_enabled = true;
	}
	return 0;
err_clk:
	clk_disable_unprepare(fh->hclk);
	return ret;
}

void dwc2_fh8626_disable(struct dwc2_hsotg *hsotg)
{
	struct dwc2_fh8626 *fh = hsotg->fh8626;
	int ret;

	if (!fh)
		return;
	if (fh->vbus_enabled) {
		ret = regulator_disable(fh->vbus);
		if (ret)
			dev_err(hsotg->dev,
				"failed to disable VBUS supply: %d\n", ret);
		else
			fh->vbus_enabled = false;
	}
	clk_disable_unprepare(fh->hclk);
}

#ifdef CONFIG_DEBUG_FS
/* Called under hsotg->lock in the disconnect IRQ with the core accessible. */
void dwc2_fh8626_report_disconnect(struct dwc2_hsotg *hsotg)
{
	struct dwc2_fh8626 *fh = hsotg->fh8626;
	u32 dr, ddr, ext, mask;

	if (!fh || !fh->vbus_gpio_regs)
		return;
	dr = readl(fh->vbus_gpio_regs + FH8626_GPIO_DR);
	ddr = readl(fh->vbus_gpio_regs + FH8626_GPIO_DDR);
	ext = readl(fh->vbus_gpio_regs + FH8626_GPIO_EXT);
	mask = BIT(fh->vbus_gpio_bit);
	dev_warn_ratelimited(hsotg->dev,
		"disconnect: HPRT0=%08x VBUS GPIO%u output=%u latch=%u input=%u DR=%08x DDR=%08x EXT=%08x\n",
		dwc2_readl(hsotg, HPRT0), fh->vbus_gpio_bit,
		!!(ddr & mask), !!(dr & mask), !!(ext & mask), dr, ddr, ext);
}

/* Caller holds hsotg->lock and has established an active core. */
static void fh8626_read_snapshot(struct dwc2_hsotg *hsotg,
				struct fh8626_usb_snapshot *snapshot,
				enum dwc2_fh8626_snapshot_reason reason)
{
	snapshot->sts = dwc2_readl(hsotg, GINTSTS);
	snapshot->mask = dwc2_readl(hsotg, GINTMSK);
	snapshot->ahb = dwc2_readl(hsotg, GAHBCFG);
	snapshot->usb = dwc2_readl(hsotg, GUSBCFG);
	snapshot->pcg = dwc2_readl(hsotg, PCGCTL);
	snapshot->port = (snapshot->sts & GINTSTS_CURMODE_HOST) ?
		dwc2_readl(hsotg, HPRT0) : 0;
	snapshot->when = jiffies;
	snapshot->reason = reason;
	snapshot->valid = true;
}

void dwc2_fh8626_before_suspend(struct dwc2_hsotg *hsotg,
			      enum dwc2_fh8626_snapshot_reason reason)
{
	if (!hsotg->fh8626 || !hsotg->ll_hw_enabled ||
	    hsotg->lx_state != DWC2_L0 || hsotg->bus_suspended)
		return;
	fh8626_read_snapshot(hsotg, &hsotg->fh8626->before_suspend, reason);
}

static int fh8626_status_show(struct seq_file *s, void *unused)
{
	struct dwc2_hsotg *hsotg = s->private;
	struct dwc2_fh8626 *fh = hsotg->fh8626;
	struct fh8626_usb_snapshot snapshot;
	struct usb_hcd *hcd = hsotg->priv;
	unsigned long flags;
	unsigned int state;
	bool active, suspended, accessible;

	spin_lock_irqsave(&hsotg->lock, flags);
	state = hsotg->lx_state;
	suspended = hsotg->bus_suspended;
	accessible = hcd && HCD_HW_ACCESSIBLE(hcd);
	/*
	 * With hibernation=0, an empty root hub enters L2 without suspending
	 * hardware. It still permits live reads; suspended hardware uses
	 * the cache.
	 */
	active = hsotg->ll_hw_enabled && hsotg->hcd_enabled &&
		 accessible && !suspended &&
		 (state == DWC2_L0 ||
		  (state == DWC2_L2 && !hsotg->hw_params.hibernation));
	if (active) {
		u32 pcg = dwc2_readl(hsotg, PCGCTL);

		active = !(pcg & (PCGCTL_STOPPCLK | PCGCTL_GATEHCLK |
				 PCGCTL_PWRCLMP | PCGCTL_RSTPDWNMODULE |
				 PCGCTL_ENBL_SLEEP_GATING |
				 PCGCTL_ENBL_EXTND_HIBER));
	}
	if (active)
		fh8626_read_snapshot(hsotg, &snapshot, FH8626_USB_LIVE);
	else
		snapshot = fh->before_suspend;
	spin_unlock_irqrestore(&hsotg->lock, flags);

	seq_printf(s, "irq=%d lx_state=%u bus_suspended=%u hw_accessible=%u vbus_enabled=%d otg_hz=%lu\n",
		   hsotg->irq, state, suspended, accessible,
		   regulator_is_enabled(fh->vbus), clk_get_rate(hsotg->clk));
	if (snapshot.valid) {
		const char *reason = "read";

		if (snapshot.reason == FH8626_USB_BEFORE_BUS_SUSPEND)
			reason = "before-bus-suspend";
		else if (snapshot.reason == FH8626_USB_BEFORE_PORT_SUSPEND)
			reason = "before-port-suspend";
		seq_printf(s, "core-source=%s reason=%s age_ms=%u\n",
			   active ? "live" : "cached", reason,
			   jiffies_to_msecs(jiffies - snapshot.when));
		seq_printf(s, "host=%u HPRT0=%08x GINTSTS=%08x GINTMSK=%08x GAHBCFG=%08x\n",
			   !!(snapshot.sts & GINTSTS_CURMODE_HOST),
			   snapshot.port,
			   snapshot.sts, snapshot.mask, snapshot.ahb);
		seq_printf(s, "GUSBCFG=%08x PCGCTL=%08x\n",
			   snapshot.usb, snapshot.pcg);
	} else {
		seq_puts(s, "core-registers=unavailable (inactive; no resume attempted)\n");
	}
	if (fh->vbus_gpio_regs) {
		u32 dr = readl(fh->vbus_gpio_regs + FH8626_GPIO_DR);
		u32 ddr = readl(fh->vbus_gpio_regs + FH8626_GPIO_DDR);
		u32 ext = readl(fh->vbus_gpio_regs + FH8626_GPIO_EXT);
		u32 mask = BIT(fh->vbus_gpio_bit);

		seq_printf(s, "vbus-gpio bit=%u output=%u latch=%u input=%u\n",
			   fh->vbus_gpio_bit,
			   !!(ddr & mask), !!(dr & mask), !!(ext & mask));
	} else {
		seq_puts(s, "vbus-gpio=unavailable\n");
	}
	return 0;
}

static int fh8626_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, fh8626_status_show, inode->i_private);
}

static const struct file_operations fh8626_status_fops = {
	.owner = THIS_MODULE,
	.open = fh8626_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void dwc2_fh8626_debugfs_init(struct dwc2_hsotg *hsotg, struct dentry *root)
{
	struct dentry *file;

	if (!hsotg->fh8626 || IS_ERR_OR_NULL(root))
		return;
	file = debugfs_create_file("fh8626_status", 0400, root, hsotg,
				   &fh8626_status_fops);
	if (IS_ERR_OR_NULL(file))
		dev_warn(hsotg->dev, "failed to create debugfs status file\n");
}
#endif
