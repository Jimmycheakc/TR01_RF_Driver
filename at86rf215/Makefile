DEPS = ../i2c/i2c.h at86rf215.h at86rf215_common.h at86rf215_radio.h at86rf215_regs.h ../io_utils/io_utils.h
SRC_FILES = ../i2c/i2c.c at86rf215.c at86rf215_events.c at86rf215_radio.c ../io_utils/io_utils.c test_at86rf215.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = at86rf215_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	$(CC) $(OBJ_FILES) -o $(EXECUTABLE) -lgpiod -lpthread

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)
