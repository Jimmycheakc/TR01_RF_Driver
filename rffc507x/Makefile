DEPS = ../io_utils/io_utils.h ../tr01_spi.h rffc507x.h rffc507x_regs.h
SRC_FILES = ../io_utils/io_utils.c ../tr01_spi.c rffc507x.c rffc507x_test.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = rffc507x_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	$(CC) $(OBJ_FILES) -o $(EXECUTABLE) -I/opt/fsl-imx-xwayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/include -L/opt/fsl-imx-xwayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/lib -lgpiod -lm

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)