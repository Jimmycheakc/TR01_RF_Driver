DEPS = ../io_utils/io_utils.h ../i2c/i2c.h rffc507x.h rffc507x_regs.h
SRC_FILES = ../io_utils/io_utils.c ../i2c/i2c.c rffc507x.c rffc507x_test.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = rffc507x_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	$(CC) $(OBJ_FILES) -o $(EXECUTABLE) -lgpiod -lm

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)