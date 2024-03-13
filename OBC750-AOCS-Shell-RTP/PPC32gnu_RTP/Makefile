# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))
WIND_BASE := $(subst \,/,$(WIND_BASE))
WIND_USR := $(subst \,/,$(WIND_USR))
WRVX_COMPBASE := $(subst \,/,$(WRVX_COMPBASE))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

JOBS?=1
TARGET_JOBS?=$(JOBS)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC32gnu_RTP
DEBUG_MODE = 0
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/WindRiver/workspace
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/OBC750-AOCS-Shell-RTP



#Global Build Macros
PROJECT_TYPE = RTP
DEFINES = 
DO_STRIP = 1
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC32
TOOL_FAMILY = gnu
TOOL = gnu
TOOL_PATH = 
CC_ARCH_SPEC = -mhard-float -mstrict-align -mregnames 
LIBPATH = 
LIBS = -lstdc++
VSB_DIR = $(WIND_BASE)/target/lib
LD_LIBRARY_PATH = host:c:/vxsim/lib

IDE_INCLUDES = -I$(WIND_BASE)/target/usr/h -I$(WIND_BASE)/target/usr/h/wrn/coreip -I$(WS_ROOT_DIR)/include -I$(WS_ROOT_DIR)/AOCS\ include -I$(WS_ROOT_DIR)/OBC750-File-Number-Library/include -I$(WS_ROOT_DIR)/OBC750-AOCS-Shell-RTP/include -I$(WS_ROOT_DIR)/OBC750-CAN-Server-RTP/Include -I$(WS_ROOT_DIR)/OBC750-AOCS-Algs/include -I$(WS_ROOT_DIR)/OBC750-Endian-Transposition-Library/include -I$(WS_ROOT_DIR)/OBC750-CAN-Server-API-Lib/include -I$(WS_ROOT_DIR)/AOCS\ include -I$(WS_ROOT_DIR)/OBC750-PFH-Library/include -I$(WS_ROOT_DIR)/OBC750-eWOD-RTP/include -I$(WS_ROOT_DIR)/OBC750-Logging-Library/Include -I$(WS_ROOT_DIR)/OBC750-Downlink-Telemetry-Library/include -I$(WS_ROOT_DIR)/OBC750-SkedExec-Library/include 

IDE_LIBRARIES = 

IDE_DEFINES = -D_VX_CPU=_VX_$(CPU) -D_VX_TOOL_FAMILY=$(TOOL_FAMILY) -D_VX_TOOL=$(TOOL)   -DSET_THREAD_PRIORITY 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler =  -O2  -fstrength-reduce -fno-builtin
DEBUGFLAGS_C++-Compiler =  -O2  -fstrength-reduce -fno-builtin
DEBUGFLAGS_Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler =  -O2  -fstrength-reduce -fno-builtin
endif


#Project Targets
PROJECT_TARGETS = OBC750-AOCS-Shell-RTP/$(MODE_DIR)/OBC750-AOCS-Shell-RTP.vxe


#Rules

# OBC750-AOCS-Shell-RTP
ifeq ($(DEBUG_MODE),1)
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_Linker = 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler =  -O2  -fstrength-reduce -fno-builtin
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler =  -O2  -fstrength-reduce -fno-builtin
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_Linker = 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEBUGFLAGS_Assembler =  -O2  -fstrength-reduce -fno-builtin
endif
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/usr/h -I$(WIND_BASE)/target/usr/h/wrn/coreip -I$(WS_ROOT_DIR)/include -I$(WS_ROOT_DIR)/AOCS\ include -I$(WS_ROOT_DIR)/OBC750-File-Number-Library/include -I$(WS_ROOT_DIR)/OBC750-AOCS-Shell-RTP/include -I$(WS_ROOT_DIR)/OBC750-CAN-Server-RTP/Include -I$(WS_ROOT_DIR)/OBC750-AOCS-Algs/include -I$(WS_ROOT_DIR)/OBC750-Endian-Transposition-Library/include -I$(WS_ROOT_DIR)/OBC750-CAN-Server-API-Lib/include -I$(WS_ROOT_DIR)/AOCS\ include -I$(WS_ROOT_DIR)/OBC750-PFH-Library/include -I$(WS_ROOT_DIR)/OBC750-eWOD-RTP/include -I$(WS_ROOT_DIR)/OBC750-Logging-Library/Include -I$(WS_ROOT_DIR)/OBC750-Downlink-Telemetry-Library/include -I$(WS_ROOT_DIR)/OBC750-SkedExec-Library/include 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : IDE_LIBRARIES = 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : IDE_DEFINES = -D_VX_CPU=_VX_$(CPU) -D_VX_TOOL_FAMILY=$(TOOL_FAMILY) -D_VX_TOOL=$(TOOL)   -DSET_THREAD_PRIORITY 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : PROJECT_TYPE = RTP
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DEFINES = 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : DO_STRIP = 1
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : EXPAND_DBG = 0
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : CPU = PPC32
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : TOOL_FAMILY = gnu
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : TOOL = gnu
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : TOOL_PATH = 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : CC_ARCH_SPEC = -mhard-float -mstrict-align -mregnames 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : LIBPATH = 
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : LIBS = -lstdc++
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : VSB_DIR = $(WIND_BASE)/target/lib
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : LD_LIBRARY_PATH = host:c:/vxsim/lib
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/% : OBJ_DIR := OBC750-AOCS-Shell-RTP/$(MODE_DIR)

OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ACON_AocsConfig.o : $(PRJ_ROOT_DIR)/source/ACON_AocsConfig.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ACON_sked.o : $(PRJ_ROOT_DIR)/source/ACON_sked.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AFIH_Filehandler.o : $(PRJ_ROOT_DIR)/source/AFIH_Filehandler.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AINT_AdcsInterface.o : $(PRJ_ROOT_DIR)/source/AINT_AdcsInterface.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AMOH_AocsModeHandler.o : $(PRJ_ROOT_DIR)/source/AMOH_AocsModeHandler.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AOCS_AocsFSM.o : $(PRJ_ROOT_DIR)/source/AOCS_AocsFSM.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AOCS_AocsShell.o : $(PRJ_ROOT_DIR)/source/AOCS_AocsShell.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AROH_AroHandler.o : $(PRJ_ROOT_DIR)/source/AROH_AroHandler.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ATTC_AocsTTC.o : $(PRJ_ROOT_DIR)/source/ATTC_AocsTTC.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/EWOD_EwodHandler.o : $(PRJ_ROOT_DIR)/source/EWOD_EwodHandler.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_AIM.o : $(PRJ_ROOT_DIR)/source/HINT_AIM.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_Gps.o : $(PRJ_ROOT_DIR)/source/HINT_Gps.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_MWheel.o : $(PRJ_ROOT_DIR)/source/HINT_MWheel.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_Cmd.o : $(PRJ_ROOT_DIR)/source/HMGR_Cmd.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_HardwareManager.o : $(PRJ_ROOT_DIR)/source/HMGR_HardwareManager.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_Tlm.o : $(PRJ_ROOT_DIR)/source/HMGR_Tlm.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/PLAN_Planner.o : $(PRJ_ROOT_DIR)/source/PLAN_Planner.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/RCON_Reconfigure.o : $(PRJ_ROOT_DIR)/source/RCON_Reconfigure.c $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C-Compiler) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -fsigned-char $(IDE_DEFINES) $(DEFINES) $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -o "$@" -c "$<" -D_OBC750_


OBJECTS_OBC750-AOCS-Shell-RTP = OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ACON_AocsConfig.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ACON_sked.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AFIH_Filehandler.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AINT_AdcsInterface.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AMOH_AocsModeHandler.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AOCS_AocsFSM.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AOCS_AocsShell.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AROH_AroHandler.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ATTC_AocsTTC.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/EWOD_EwodHandler.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_AIM.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_Gps.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_MWheel.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_Cmd.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_HardwareManager.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_Tlm.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/PLAN_Planner.o \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/RCON_Reconfigure.o \
	 $(WS_ROOT_DIR)/OBC750-PFH-Library/PPC32gnu_RTP/lib-sstl-pfh/$(MODE_DIR)/lib-sstl-pfh.so \
	 $(WS_ROOT_DIR)/OBC750-Endian-Transposition-Library/PPC32gnu_RTP/lib-sstl-endian-transposition/$(MODE_DIR)/lib-sstl-endian-transposition.so \
	 $(WS_ROOT_DIR)/OBC750-CAN-Server-API-Lib/PPC32gnu_RTP/lib-sstl-can-server-api/$(MODE_DIR)/lib-sstl-can-server-api.so \
	 $(WS_ROOT_DIR)/OBC750-Logging-Library/PPC32gnu_RTP/lib-sstl-logging/$(MODE_DIR)/lib-sstl-logging.so \
	 $(WS_ROOT_DIR)/OBC750-File-Number-Library/PPC32gnu_RTP/lib-sstl-file-number/$(MODE_DIR)/lib-sstl-file-number.so \
	 $(WS_ROOT_DIR)/OBC750-AOCS-Algs/PPC32gnu_RTP/lib-sstl-aocs-algs.so/$(MODE_DIR)/lib-sstl-aocs-algs.so \
	 $(WS_ROOT_DIR)/OBC750-Downlink-Telemetry-Library/PPC32gnu_RTP/lib-sstl-downlink-telemetry/$(MODE_DIR)/lib-sstl-downlink-telemetry.so \
	 $(WS_ROOT_DIR)/OBC750-SkedExec-Library/PPC32gnu_RTP/lib-sstl-skedexec/$(MODE_DIR)/lib-sstl-skedexec.so

ifeq ($(TARGET_JOBS),1)
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/OBC750-AOCS-Shell-RTP.vxe : $(OBJECTS_OBC750-AOCS-Shell-RTP)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)c++ppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -mrtp -fno-strict-aliasing -D_C99 -D_HAS_C9X -std=c99 -fasm -Wall -Wsystem-headers   -MD -MP -mrtp  -non-static -o "$@" $(OBJECTS_OBC750-AOCS-Shell-RTP) $(ADDED_OBJECTS) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS) -L$(VSB_DIR)/usr/lib/ppc/PPC32/common $(ADDED_LIBPATH) $(ADDED_LIBS) -Wl,--start-group -Wl,--end-group && if [ "$(DO_STRIP)" = "1" ]; then mv -f "$@" "$@.unstripped" && stripppc -g -o "$@" "$@.unstripped";fi && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

else
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/OBC750-AOCS-Shell-RTP.vxe : OBC750-AOCS-Shell-RTP/$(MODE_DIR)/OBC750-AOCS-Shell-RTP.vxe_jobs

endif
OBC750-AOCS-Shell-RTP/$(MODE_DIR)/OBC750-AOCS-Shell-RTP_compile_file : $(FILE) ;

_clean :: OBC750-AOCS-Shell-RTP/$(MODE_DIR)/OBC750-AOCS-Shell-RTP_clean

OBC750-AOCS-Shell-RTP/$(MODE_DIR)/OBC750-AOCS-Shell-RTP_clean : 
	$(TRACE_FLAG)if [ -d "OBC750-AOCS-Shell-RTP" ]; then cd "OBC750-AOCS-Shell-RTP"; rm -rf $(MODE_DIR); fi

$(WS_ROOT_DIR)/OBC750-AOCS-Algs/PPC32gnu_RTP/lib-sstl-aocs-algs.so/$(MODE_DIR)/lib-sstl-aocs-algs.so : force_referenced_rebuilds_OBC750-AOCS-Algs ;

force_referenced_rebuilds_OBC750-AOCS-Algs : 
	$(TRACE_FLAG)+cd $(WS_ROOT_DIR)/OBC750-AOCS-Algs/PPC32gnu_RTP;\
	make --no-print-directory BUILD_SPEC=PPC32gnu_RTP DEBUG_MODE=0 lib-sstl-aocs-algs.so/$(MODE_DIR)/lib-sstl-aocs-algs.so

$(WS_ROOT_DIR)/OBC750-CAN-Server-API-Lib/PPC32gnu_RTP/lib-sstl-can-server-api/$(MODE_DIR)/lib-sstl-can-server-api.so : force_referenced_rebuilds_OBC750-CAN-Server-API-Lib ;

force_referenced_rebuilds_OBC750-CAN-Server-API-Lib : 
	$(TRACE_FLAG)+cd $(WS_ROOT_DIR)/OBC750-CAN-Server-API-Lib/PPC32gnu_RTP;\
	make --no-print-directory BUILD_SPEC=PPC32gnu_RTP DEBUG_MODE=0 lib-sstl-can-server-api/$(MODE_DIR)/lib-sstl-can-server-api.so

$(WS_ROOT_DIR)/OBC750-Downlink-Telemetry-Library/PPC32gnu_RTP/lib-sstl-downlink-telemetry/$(MODE_DIR)/lib-sstl-downlink-telemetry.so : force_referenced_rebuilds_OBC750-Downlink-Telemetry-Library ;

force_referenced_rebuilds_OBC750-Downlink-Telemetry-Library : 
	$(TRACE_FLAG)+cd $(WS_ROOT_DIR)/OBC750-Downlink-Telemetry-Library/PPC32gnu_RTP;\
	make --no-print-directory BUILD_SPEC=PPC32gnu_RTP DEBUG_MODE=0 lib-sstl-downlink-telemetry/$(MODE_DIR)/lib-sstl-downlink-telemetry.so

$(WS_ROOT_DIR)/OBC750-Endian-Transposition-Library/PPC32gnu_RTP/lib-sstl-endian-transposition/$(MODE_DIR)/lib-sstl-endian-transposition.so : force_referenced_rebuilds_OBC750-Endian-Transposition-Library ;

force_referenced_rebuilds_OBC750-Endian-Transposition-Library : 
	$(TRACE_FLAG)+cd $(WS_ROOT_DIR)/OBC750-Endian-Transposition-Library/PPC32gnu_RTP;\
	make --no-print-directory BUILD_SPEC=PPC32gnu_RTP DEBUG_MODE=0 lib-sstl-endian-transposition/$(MODE_DIR)/lib-sstl-endian-transposition.so

$(WS_ROOT_DIR)/OBC750-File-Number-Library/PPC32gnu_RTP/lib-sstl-file-number/$(MODE_DIR)/lib-sstl-file-number.so : force_referenced_rebuilds_OBC750-File-Number-Library ;

force_referenced_rebuilds_OBC750-File-Number-Library : 
	$(TRACE_FLAG)+cd $(WS_ROOT_DIR)/OBC750-File-Number-Library/PPC32gnu_RTP;\
	make --no-print-directory BUILD_SPEC=PPC32gnu_RTP DEBUG_MODE=0 lib-sstl-file-number/$(MODE_DIR)/lib-sstl-file-number.so

$(WS_ROOT_DIR)/OBC750-Logging-Library/PPC32gnu_RTP/lib-sstl-logging/$(MODE_DIR)/lib-sstl-logging.so : force_referenced_rebuilds_OBC750-Logging-Library ;

force_referenced_rebuilds_OBC750-Logging-Library : 
	$(TRACE_FLAG)+cd $(WS_ROOT_DIR)/OBC750-Logging-Library/PPC32gnu_RTP;\
	make --no-print-directory BUILD_SPEC=PPC32gnu_RTP DEBUG_MODE=0 lib-sstl-logging/$(MODE_DIR)/lib-sstl-logging.so

$(WS_ROOT_DIR)/OBC750-PFH-Library/PPC32gnu_RTP/lib-sstl-pfh/$(MODE_DIR)/lib-sstl-pfh.so : force_referenced_rebuilds_OBC750-PFH-Library ;

force_referenced_rebuilds_OBC750-PFH-Library : 
	$(TRACE_FLAG)+cd $(WS_ROOT_DIR)/OBC750-PFH-Library/PPC32gnu_RTP;\
	make --no-print-directory BUILD_SPEC=PPC32gnu_RTP DEBUG_MODE=0 lib-sstl-pfh/$(MODE_DIR)/lib-sstl-pfh.so

$(WS_ROOT_DIR)/OBC750-SkedExec-Library/PPC32gnu_RTP/lib-sstl-skedexec/$(MODE_DIR)/lib-sstl-skedexec.so : force_referenced_rebuilds_OBC750-SkedExec-Library ;

force_referenced_rebuilds_OBC750-SkedExec-Library : 
	$(TRACE_FLAG)+cd $(WS_ROOT_DIR)/OBC750-SkedExec-Library/PPC32gnu_RTP;\
	make --no-print-directory BUILD_SPEC=PPC32gnu_RTP DEBUG_MODE=0 lib-sstl-skedexec/$(MODE_DIR)/lib-sstl-skedexec.so

force : 

TARGET_JOBS_RULE?=echo "Update the makefile template via File > Import > Build Settings : Update makefile template";exit 1
%_jobs : 
	$(TRACE_FLAG)$(TARGET_JOBS_RULE)

DEP_FILES := OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ACON_AocsConfig.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ACON_sked.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AFIH_Filehandler.d \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AINT_AdcsInterface.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AMOH_AocsModeHandler.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AOCS_AocsFSM.d \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AOCS_AocsShell.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/AROH_AroHandler.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/ATTC_AocsTTC.d \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/EWOD_EwodHandler.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_AIM.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_Gps.d \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HINT_MWheel.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_Cmd.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_HardwareManager.d \
	 OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/HMGR_Tlm.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/PLAN_Planner.d OBC750-AOCS-Shell-RTP/$(MODE_DIR)/Objects/OBC750-AOCS-Shell-RTP/source/RCON_Reconfigure.d
-include $(DEP_FILES)


WIND_SCOPETOOLS_BASE := $(subst \,/,$(WIND_SCOPETOOLS_BASE))

clean_scopetools :
	$(TRACE_FLAG)rm -rf $(PRJ_ROOT_DIR)/.coveragescope/db

CLEAN_STEP := clean_scopetools


-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

TARGET_JOBS_RULE=$(MAKE) -f $(MAKEFILE) --jobs $(TARGET_JOBS) $(MFLAGS) $* TARGET_JOBS=1
ifeq ($(JOBS),1)
main_all : external_build  $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"
else
main_all : external_build 
	@$(MAKE) -f $(MAKEFILE) --jobs $(JOBS) $(MFLAGS) $(PROJECT_TARGETS) TARGET_JOBS=1;\
	echo "make: built targets of `pwd`"
endif 

# entry point for extending the build
external_build ::
	@echo ""

# main entry point for pre processing prior to the build
pre_build :: $(PRE_BUILD_STEP) generate_sources
	@echo ""

# entry point for generating sources prior to the build
generate_sources ::
	@echo ""

# main entry point for post processing after the build
post_build :: $(POST_BUILD_STEP) deploy_output
	@echo ""

# entry point for deploying output after the build
deploy_output ::
	@echo ""

clean :: external_clean $(CLEAN_STEP) _clean

# entry point for extending the build clean
external_clean ::
	@echo ""
