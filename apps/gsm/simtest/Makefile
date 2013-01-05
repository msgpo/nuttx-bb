############################################################################
# apps/gsm/simtest/Makefile
#
#   Copyright (C) 2008, 2010-2012 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# simtest built-in application info

APPNAME		= simtest
PRIORITY	= SCHED_PRIORITY_DEFAULT
STACKSIZE	= 2048

# simtest Example

ASRCS		=
CSRCS		= simtest_main.c \
		../common/comm/timer.c \
		../common/sim/sim.c \
		../common/abb/twl3025.c \
		../common/calypso/dsp.c \
		../common/calypso/tpu.c \
		../common/calypso/tsp.c \
		../common/rffe/rffe_gta0x_triband.c \
		../common/rf/trf6151.c \
		../common/libosmocore/utils.c \
		../common/libosmocore/gsm_utils.c \
		../common/rf/gta0x/rf_power.c \
		../common/layer1/avg.c \
		../common/layer1/agc.c \
		../common/layer1/afc.c \
		../common/layer1/toa.c \
		../common/layer1/sync.c \
		../common/layer1/tdma_sched.c \
		../common/layer1/tpu_window.c \
		../common/layer1/init.c \
		../common/layer1/l23_api.c \
		../common/layer1/mframe_sched.c \
		../common/layer1/sched_gsmtime.c \
		../common/layer1/async.c \
		../common/layer1/rfch.c \
		../common/layer1/apc.c \
		../common/layer1/prim_pm.c \
		../common/layer1/prim_rach.c \
		../common/layer1/prim_tx_nb.c \
		../common/layer1/prim_rx_nb.c \
		../common/layer1/prim_fbsb.c \
		../common/layer1/prim_freq.c \
		../common/layer1/prim_utils.c \
		../common/layer1/prim_tch.c \

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

CFLAGS		+= -I../common/include/
CFLAGS		+= -I$(TOPDIR)/include/

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN		= ..\..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN		= ..\\..\\libapps$(LIBEXT)
else
  BIN		= ../../libapps$(LIBEXT)
endif
endif

ROOTDEPPATH	= --dep-path .

# Common build

VPATH		= 

all: .built
.PHONY: clean depend distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

ifeq ($(CONFIG_EXAMPLES_SIMTEST_BUILTIN),y)
$(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat: $(DEPCONFIG) Makefile
	$(call REGISTER,$(APPNAME),$(PRIORITY),$(STACKSIZE),$(APPNAME)_main)

context: $(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat
else
context:
endif

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
