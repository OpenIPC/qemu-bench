#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kallsyms.h>
#include <linux/printk.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("OpenIPC qemu-hisilicon");

static unsigned long anchor;
module_param(anchor, ulong, 0644);
MODULE_PARM_DESC(anchor, "BSS anchor base (kernel-virtual addr of LANCHOR1)");

static void dump_words(unsigned long addr, const char *desc, int n)
{
    int i;
    pr_info("vpss-introspect: %s @0x%lx:\n", desc, addr);
    for (i = 0; i < n; i++) {
        unsigned long *slot = (unsigned long *)(addr + i * 4);
        pr_info("  [+%02x]  0x%lx -> 0x%lx\n",
                i * 4, (unsigned long)slot, *slot);
    }
}

static int __init mod_init(void)
{
    unsigned long s;

    pr_info("vpss-introspect: starting\n");
    s = kallsyms_lookup_name("VPSS_COMM_OfflineIrqProc");
    pr_info("  VPSS_COMM_OfflineIrqProc = 0x%lx\n", s);
    s = kallsyms_lookup_name("VPSS_HAL_GetIntStatus");
    pr_info("  VPSS_HAL_GetIntStatus    = 0x%lx\n", s);

    if (!anchor) {
        pr_info("vpss-introspect: pass anchor=<bss_base> module param to dump\n");
        return 0;
    }

    pr_info("vpss-introspect: bss anchor = 0x%lx\n", anchor);
    dump_words(anchor + 0x1740, "BSS+0x1740 (per-node arr)", 8);

    {
        unsigned long ptr0 = *(unsigned long *)(anchor + 0x1740);
        if (ptr0) {
            dump_words(ptr0, "*ARR[0]", 16);
            pr_info("vpss-introspect: *ARR[0] + 8 = 0x%lx\n",
                    *(unsigned long *)(ptr0 + 8));
        } else {
            pr_info("vpss-introspect: ARR[0] is NULL — vendor never populated\n");
        }
    }

    dump_words(anchor + 0xb60, "BSS+0xb60", 4);
    return 0;
}

static void __exit mod_exit(void) { pr_info("vpss-introspect: unloading\n"); }
module_init(mod_init);
module_exit(mod_exit);
