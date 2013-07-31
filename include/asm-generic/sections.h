/*
 * Copyright (c) 2011 The Chromium OS Authors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* Taken from Linux kernel, commit f56c3196 */

#ifndef _ASM_GENERIC_SECTIONS_H_
#define _ASM_GENERIC_SECTIONS_H_

/* References to section boundaries */

extern char _text[], _stext[], _etext[];
extern char _data[], _sdata[], _edata[];
extern char __bss_start[], __bss_stop[];
extern char __init_begin[], __init_end[];
extern char _sinittext[], _einittext[];
extern char _end[], _init[];
extern char __per_cpu_load[], __per_cpu_start[], __per_cpu_end[];
extern char __kprobes_text_start[], __kprobes_text_end[];
extern char __entry_text_start[], __entry_text_end[];
extern char __initdata_begin[], __initdata_end[];
extern char __start_rodata[], __end_rodata[];

/* Start and end of .ctors section - used for constructor calls. */
extern char __ctors_start[], __ctors_end[];

/* function descriptor handling (if any).  Override
 * in asm/sections.h */
#ifndef dereference_function_descriptor
#define dereference_function_descriptor(p) (p)
#endif

/* random extra sections (if any).  Override
 * in asm/sections.h */
#ifndef arch_is_kernel_text
static inline int arch_is_kernel_text(unsigned long addr)
{
	return 0;
}
#endif

#ifndef arch_is_kernel_data
static inline int arch_is_kernel_data(unsigned long addr)
{
	return 0;
}
#endif

/* U-Boot-specific things begin here */

/* Start of U-Boot text region */
extern char __text_start[];

/* This marks the end of the text region which must be relocated */
extern char __image_copy_start[];
extern char __image_copy_end[];

/*
 * This is the U-Boot entry point - prior to relocation it should be same
 * as __text_start
 */
extern void _start(void);

#ifdef CONFIG_ARM
/* ARM uses _end instead of __init_end in .lds files at present */
#define __init_end __image_copy_end
#endif

/* Exports from the Linker Script */
extern ulong __data_end;
extern ulong __rel_dyn_start;
extern ulong __rel_dyn_end;
extern ulong __bss_end;

extern ulong _TEXT_BASE;	/* code start */

#endif /* _ASM_GENERIC_SECTIONS_H_ */
