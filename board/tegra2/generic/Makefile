#
# Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#

include $(TOPDIR)/config.mk

$(shell mkdir -p $(obj)sdmmc)
$(shell mkdir -p $(obj)crypto)
$(shell mkdir -p $(obj)lcd)
$(shell mkdir -p $(obj)lcd/gpinit)
$(shell mkdir -p $(obj)nand)

LIB	= $(obj)lib$(BOARD).a

COBJS-y	:= $(BOARD).o
COBJS-y	+= board.o
COBJS-y	+= sdmmc/nvboot_sdmmc.o
COBJS-y	+= sdmmc/nvboot_reset.o
COBJS-y	+= sdmmc/nvboot_util.o
COBJS-y	+= sdmmc/nvboot_clocks.o
COBJS-y	+= sdmmc/nvboot_pads.o
COBJS-y	+= sdmmc/nvboot_pinmux.o
COBJS-y	+= sdmmc/nvboot_pinmux_tables.o
COBJS-$(CONFIG_TEGRA2_KEYBOARD)	+= $(BOARD)_kbc.o
COBJS-$(CONFIG_TEGRA2_LP0)	+= crypto/aes_ref.o
COBJS-$(CONFIG_TEGRA2_LP0)	+= crypto/crypto.o
COBJS-$(CONFIG_TEGRA2_LCD)	+= gpinit.o
COBJS-$(CONFIG_TEGRA2_LCD)	+= lcd/gpinit/gp-util.o
COBJS-$(CONFIG_TEGRA2_LCD)	+= lcd/gpinit/gp-dc.o
COBJS-$(CONFIG_TEGRA2_LCD)	+= lcd/gpinit/gp-clk.o
COBJS-$(CONFIG_TEGRA2_LCD)	+= lcd/gpinit/gp-pinmux.o
COBJS-$(CONFIG_TEGRA2_LCD)	+= lcd/gpinit/gp-pinmux-t2-tables.o
COBJS-$(CONFIG_TEGRA2_LCD)	+= lcd/nvboot_lcd.o
COBJS-$(CONFIG_TEGRA2_NAND)	+= nand/tegra2_nand.o

COBJS	:= $(sort $(COBJS-y))
SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))

$(LIB):	$(obj).depend $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

clean:
	rm -f $(OBJS)

distclean:	clean
	rm -f $(LIB) core *.bak $(obj).depend

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
