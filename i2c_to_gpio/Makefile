DEPS = peripheral_gpio.h
SRC_FILES = peripheral_gpio_test.c peripheral_gpio.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = peripheral_gpio_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE)

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)