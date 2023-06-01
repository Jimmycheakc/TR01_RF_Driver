DEPS = ../spi/spi.h at86rf215_bb_regs.h at86rf215_bb.h ../io_utils/io_utils.h ../Json/cJSON.h at86rf215_bb_usb.h
SRC_FILES = ../spi/spi.c at86rf215_bb.c ../io_utils/io_utils.c ../Json/cJSON.c at86rf215_bb_events.c at86rf215_bb_usb.c at86rf215_bb_test.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = at86rf215_bb_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	$(CC) $(OBJ_FILES) -o $(EXECUTABLE) -lgpiod -lpthread -lft4222

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -DDEBUG -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)
