DEPS = ../spi/tr01_spi.h at86rf215.h at86rf215_common.h at86rf215_radio.h at86rf215_regs.h ../io_utils/io_utils.h
SRC_FILES = ../spi/tr01_spi.c at86rf215.c at86rf215_events.c at86rf215_radio.c ../io_utils/io_utils.c test_at86rf215.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = at86rf215_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	$(CC) $(OBJ_FILES) -o $(EXECUTABLE) -I/opt/fsl-imx-xwayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/include -L/opt/fsl-imx-xwayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/lib -lgpiod -lpthread

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)
