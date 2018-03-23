#ifndef _LINUX_MMU_CONTEXT_H
#define _LINUX_MMU_CONTEXT_H

struct mm_struct;

static inline void
use_mm(struct mm_struct *mm)
{
	UNIMPLEMENTED();
}

static inline void
unuse_mm(struct mm_struct *mm)
{
	UNIMPLEMENTED();	
}

#endif
