CC      = gcc
CFLAGS  = -W -Wall -g
# CFLAGS  += -D__DEBUG__

INCLUDE = -I/usr/local/include
LDFLAGS = -L/usr/local/lib
# LDLIBS  = -lwiringPi -lwiringPiDev -lpthread -lm -lrt -lcrypt

TARGET  = adc-ltc2309 

SRC_DIRS = .
SRCS     = $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)/*.c))
OBJS     = $(SRCS:.c=.o)

all : $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

clean :
	rm -f $(OBJS)
	rm -f $(TARGET)
