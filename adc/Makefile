DEPS = ads7041.h ../i2c/i2c.h
SRC_FILES = ads7041.c ../i2c/i2c.c ads7041_test.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = ads7041_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE)

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)