DEPS = spi.h
SRC_FILES = spi.c spi_test.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = spi_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE)

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)