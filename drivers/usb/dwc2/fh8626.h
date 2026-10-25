/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __DWC2_FH8626_H
#define __DWC2_FH8626_H
struct dwc2_hsotg;
struct dentry;
enum dwc2_fh8626_snapshot_reason {
	FH8626_USB_LIVE,
	FH8626_USB_BEFORE_BUS_SUSPEND,
	FH8626_USB_BEFORE_PORT_SUSPEND,
};
#ifdef CONFIG_USB_DWC2_FH8626
int dwc2_fh8626_init(struct dwc2_hsotg *hsotg);
int dwc2_fh8626_enable(struct dwc2_hsotg *hsotg);
void dwc2_fh8626_disable(struct dwc2_hsotg *hsotg);
#else
static inline int dwc2_fh8626_init(struct dwc2_hsotg *hsotg) { return 0; }
static inline int dwc2_fh8626_enable(struct dwc2_hsotg *hsotg) { return 0; }
static inline void dwc2_fh8626_disable(struct dwc2_hsotg *hsotg) {}
#endif
#if defined(CONFIG_USB_DWC2_FH8626) && defined(CONFIG_DEBUG_FS)
void dwc2_fh8626_debugfs_init(struct dwc2_hsotg *hsotg, struct dentry *root);
void dwc2_fh8626_report_disconnect(struct dwc2_hsotg *hsotg);
void dwc2_fh8626_before_suspend(struct dwc2_hsotg *hsotg,
			      enum dwc2_fh8626_snapshot_reason reason);
#else
static inline void dwc2_fh8626_debugfs_init(struct dwc2_hsotg *hsotg,
					 struct dentry *root) {}
static inline void dwc2_fh8626_report_disconnect(struct dwc2_hsotg *hsotg) {}
static inline void dwc2_fh8626_before_suspend(struct dwc2_hsotg *hsotg,
			      enum dwc2_fh8626_snapshot_reason reason) {}
#endif
#endif
