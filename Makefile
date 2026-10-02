CC      ?= gcc
CFLAGS  ?= -Wall -Wextra -Werror -std=c11 -pedantic
INCLUDES = -Ihal -Isrc

SRCS = hal/mock_uart.c src/modbus_rtu.c src/modbus_master.c
OBJS = $(SRCS:.c=.o)

.PHONY: all demo test clean

all: demo test_runner

demo: $(OBJS) app/main.o
	$(CC) $(CFLAGS) -o $@ $^

test_runner: $(OBJS) tests/test_modbus.o
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

test: test_runner
	./test_runner

clean:
	rm -f $(OBJS) app/main.o tests/test_modbus.o demo test_runner
