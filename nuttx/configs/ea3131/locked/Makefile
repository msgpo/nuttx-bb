############################################################################
# configs/ea3131/locked/Makefile
#
#   Copyright (C) 2010 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

# Board-specific directory, board library, and application library

PASS1_SRCDIR	= arch/$(CONFIG_ARCH)/src
PASS1_BOARDDIR	= $(PASS1_SRCDIR)/board
PASS1_LIBBOARD	= $(PASS1_BOARDDIR)/libboard$(LIBEXT)

# Where is the application library?

ifneq ($(CONFIG_APPS_DIR),)
PASS1_LIBAPPS	= $(CONFIG_APPS_DIR)/libapps$(LIBEXT)
else
ifneq ($(APPDIR),)
PASS1_LIBAPPS	= $(APPDIR)/libapps$(LIBEXT)
endif
endif

# Remove the application library (libapps) from the list of libraries.  Add
# the boad library (liboard)

PASS1_LINKLIBS	= $(filter-out $(PASS1_LIBAPPS),$(LINKLIBS))
PASS1_LINKLIBS	+= $(PASS1_LIBBOARD)

# Get the paths to the libraries and the links script path in format that
# is appropriate for the host OS

ifeq ($(WINTOOL),y)
  # Windows-native toolchains
  PASS1_LIBPATHS	= ${shell for path in $(PASS1_LINKLIBS); do dir=`dirname $(TOPDIR)/$$path`;echo "-L\"`cygpath -w $$dir`\"";done}
  PASS1_LDSCRIPT	= -T "${shell cygpath -w $(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/locked/ld-locked.inc}"
else
  # Linux/Cygwin-native toolchain 
  PASS1_LIBPATHS	= $(addprefix -L$(TOPDIR)/,$(dir $(PASS1_LINKLIBS)))
  PASS1_LDSCRIPT	= -T$(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/locked/ld-locked.inc
endif

PASS1_LDFLAGS	= -r $(PASS1_LDSCRIPT)
PASS1_LDLIBS	= $(patsubst lib%,-l%,$(basename $(notdir $(PASS1_LINKLIBS))))
PASS1_LIBGCC	= "${shell $(CC) -print-libgcc-file-name}"

# Targets:

all: $(PASS1_SRCDIR)/locked.r

.PHONY:	depend clean distclean

# Create include-able linker script that specifies all of the symbols to be
# resolved in the locked.r file.

ld-locked.inc: mklocked.sh $(TOPDIR)/.config
	@echo "MK:  ld-locked.inc"
	@./mklocked.sh "$(TOPDIR)"

# Make the board library. This is normally done in arch/arm/src/Makefile.
# However, we need it earlier here when doing a two-pass build so that libboard.a
# is available to link against.

$(PASS1_LIBBOARD):
	@$(MAKE) -C $(TOPDIR)/configs/ea3131/src TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

# Create the locked.r file containing all of the code (except the start-up code)
# that needs to lie in the locked text region.

locked.r: ld-locked.inc $(PASS1_LIBBOARD)
	@echo "LD:  locked.r"
	@$(LD) -o $@ $(PASS1_LDFLAGS) $(PASS1_LIBPATHS) --start-group $(PASS1_LDLIBS) --end-group $(PASS1_LIBGCC) 
	@$(NM) $@ > locked.map
	@fgrep " U " locked.map | grep -v os_start
	@$(CROSSDEV)size $@

$(PASS1_SRCDIR)/locked.r: locked.r
	@cp -a locked.r $(TOPDIR)/$(PASS1_SRCDIR)/locked.r

.depend:

depend: .depend

clean:
	@rm -f locked.r locked.map *~ .*.swp

distclean: clean
	@rm -f ld-locked.inc

