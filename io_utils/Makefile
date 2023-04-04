DEPS = io_utils.h
SRC_FILES = io_utils.c gpio_test.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = gpio_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE) -I/opt/fsl-imx-xwayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/include -L/opt/fsl-imx-xwayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/lib -lgpiod

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)