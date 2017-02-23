### Application-specific constants

APP_NAME := tiny_server
APP_TEST_NAME := aes_test

### Environment constants 

LGW_PATH ?= ../../lora_gateway/libloragw
ARCH ?=
CROSS_COMPILE ?=
#ARCH ?=arm
#CROSS_COMPILE ?=/opt/toolchains/arm-2011.03-wirgrid/bin/arm-none-linux-gnueabi-

OBJDIR = obj
INCLUDES = $(wildcard inc/*.h)

### External constant definitions
# must get library build option to know if mpsse must be linked or not

include $(LGW_PATH)/library.cfg
RELEASE_VERSION := `cat ../VERSION`

### Constant symbols

CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

CFLAGS := -O2 -g -Wall -Wextra -std=c99 -Iinc -I.
VFLAG := -D VERSION_STRING="\"$(RELEASE_VERSION)\""

ifdef ENABLE_HAL_UBX
CFLAGS += -DENABLE_HAL_UBX
endif

# remove if only need class-A:
CFLAGS += -DENABLE_CLASS_B

#CFLAGS += -DUSE_BAND_915_HYBRID
#CFLAGS += -DUSE_BAND_868
CFLAGS += -DUSE_BAND_ARIB_8CH

### Constants for Lora concentrator HAL library
# List the library sub-modules that are used by the application

LGW_INC =
ifneq ($(wildcard $(LGW_PATH)/inc/config.h),)
  # only for HAL version 1.3 and beyond
  LGW_INC += $(LGW_PATH)/inc/config.h
endif
LGW_INC += $(LGW_PATH)/inc/loragw_hal.h
LGW_INC += $(LGW_PATH)/inc/loragw_gps.h

### Linking options

LIBS := -lloragw -lrt -lpthread -lm
ifeq ($(CFG_SPI),ftdi) 
    LIBS += -lmpsse
endif

### General build targets

all: $(APP_NAME) $(APP_TEST_NAME)

clean:
	rm -f $(OBJDIR)/*.o
	rm -f $(APP_NAME)
	rm -f $(APP_TEST_NAME)

### Sub-modules compilation

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: src/%.c $(INCLUDES) | $(OBJDIR)
	$(CC) -c $(CFLAGS) -I$(LGW_PATH)/inc $< -o $@

### Main program compilation and assembly

$(OBJDIR)/$(APP_NAME).o: src/$(APP_NAME).c $(LGW_INC) $(INCLUDES) | $(OBJDIR)
	$(CC) -c $(CFLAGS) $(VFLAG) -I$(LGW_PATH)/inc $< -o $@

$(OBJDIR)/$(APP_TEST_NAME).o: src/$(APP_TEST_NAME).c $(LGW_INC) $(INCLUDES) | $(OBJDIR)
	$(CC) -c $(CFLAGS) $(VFLAG) -I$(LGW_PATH)/inc $< -o $@

OBJS := $(OBJDIR)/parson.o $(OBJDIR)/base64.o $(OBJDIR)/lorawan.o $(OBJDIR)/aes.o $(OBJDIR)/cmac.o $(OBJDIR)/lorawan_bands.o

$(APP_NAME): $(OBJDIR)/$(APP_NAME).o $(LGW_PATH)/libloragw.a $(OBJS)
	$(CC) -L$(LGW_PATH) $< $(OBJS) -o $@ $(LIBS)

TEST_OBJS := $(OBJDIR)/parson.o $(OBJDIR)/base64.o $(OBJDIR)/aes.o $(OBJDIR)/cmac.o

$(APP_TEST_NAME): $(OBJDIR)/$(APP_TEST_NAME).o $(LGW_PATH)/libloragw.a $(TEST_OBJS)
	$(CC) -L$(LGW_PATH) $< $(TEST_OBJS) -o $@ $(LIBS)

### EOF
