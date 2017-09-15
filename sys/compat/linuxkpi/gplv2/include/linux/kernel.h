#ifndef _KERNEL_GPLV2_H_
#define _KERNEL_GPLV2_H_

#include_next <linux/kernel.h>

/* XXX */
#define	irqs_disabled() (curthread->td_critnest != 0 || curthread->td_intr_nesting_level != 0)

#include <linux/irqflags.h>
#include <linux/kconfig.h>

#include <asm/cpufeature.h>
#include <asm/processor.h>

#endif /* _KERNEL_GPLV2_H_ */
