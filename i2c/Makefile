DEPS = i2c.h
SRC_FILES = i2c.c test_i2c.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = i2c_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE)

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)