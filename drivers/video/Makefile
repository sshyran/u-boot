#
# (C) Copyright 2000-2007
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB	:= $(obj)libvideo.o

COBJS-$(CONFIG_ATI_RADEON_FB) += ati_radeon_fb.o videomodes.o
COBJS-$(CONFIG_ATMEL_LCD) += atmel_lcdfb.o
COBJS-$(CONFIG_CFB_CONSOLE) += cfb_console.o
COBJS-$(CONFIG_EXYNOS_LCD) += exynos-fb.o
COBJS-$(CONFIG_FSL_DIU_FB) += fsl_diu_fb.o videomodes.o
COBJS-$(CONFIG_S6E63D6) += s6e63d6.o
COBJS-$(CONFIG_SED156X) += sed156x.o
COBJS-$(CONFIG_VIDEO_AMBA) += amba.o
COBJS-$(CONFIG_VIDEO_CT69000) += ct69000.o videomodes.o
COBJS-$(CONFIG_VIDEO_DA8XX) += da8xx-fb.o videomodes.o
COBJS-$(CONFIG_VIDEO_MB862xx) += mb862xx.o videomodes.o
COBJS-$(CONFIG_VIDEO_MB86R0xGDC) += mb86r0xgdc.o videomodes.o
COBJS-$(CONFIG_VIDEO_MX3) += mx3fb.o videomodes.o
COBJS-$(CONFIG_VIDEO_MX5) += mxc_ipuv3_fb.o ipu_common.o ipu_disp.o
COBJS-$(CONFIG_VIDEO_OMAP3) += omap3_dss.o videomodes.o
COBJS-$(CONFIG_VIDEO_SED13806) += sed13806.o
COBJS-$(CONFIG_VIDEO_SM501) += sm501.o
COBJS-$(CONFIG_VIDEO_SMI_LYNXEM) += smiLynxEM.o videomodes.o
COBJS-$(CONFIG_VIDEO_VCXK) += bus_vcxk.o
COBJS-$(CONFIG_VIDEO_COREBOOT) += coreboot_fb.o
COBJS-$(CONFIG_VIDEO_TEST) += test_lcd.o

COBJS	:= $(sort $(COBJS-y))
SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))

all:	$(LIB)

$(LIB):	$(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
