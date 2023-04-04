DEPS = at30ts74.h ../i2c/i2c.h
SRC_FILES = at30ts74.c ../i2c/i2c.c at30ts74_test.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = at30ts74_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE)

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)