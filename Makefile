TARGET = ceu2rtp

OBJS = ceu2rtp.o

CROSS_COMPILE ?= arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc

CFLAGS = -Wall -Wextra -O3 -mfpu=neon -mcpu=cortex-a9 -march=armv7-a -pthread \
		 -I/usr/arm-linux-gnueabihf/include/gstreamer-1.0 \
		 -I/usr/arm-linux-gnueabihf/include/glib-2.0 \
		 -I/usr/arm-linux-gnueabihf/lib/glib-2.0/include

# If you set KERNEL_DIR, Compiler will use header files that are in KERNEL_DIR.
KERNEL_DIR:=
CFLAGS += -I$(KERNEL_DIR)

# Else, Compiler will use header files that are in the current directory.
CFLAGS += -I./include

LDFLAGS += -L/usr/arm-linux-gnueabihf/lib
LDLIBS  += -lgstapp-1.0 -lgstbase-1.0 -lgstreamer-1.0 -lgobject-2.0 -lgmodule-2.0 -lgthread-2.0 -lrt -lglib-2.0

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	$(RM) *~ *.o $(TARGET)

