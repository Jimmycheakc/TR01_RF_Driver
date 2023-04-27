DEPS = ../io_utils/io_utils.h
SRC_FILES = gpio_connector_test.c gpio_connector.c ../io_utils/io_utils.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = gpio_connector_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE) -lgpiod

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)