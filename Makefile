DEPS = gpio_connector/gpio_connector.h io_utils/io_utils.h i2c_to_gpio/peripheral_gpio.h i2c/i2c.h
SRC_FILES = test_app_tr01.c gpio_connector/gpio_connector.c io_utils/io_utils.c i2c_to_gpio/peripheral_gpio.c i2c/i2c.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = test_app_tr01

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE) -lgpiod -lft4222

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)