DEPS = cJSON.h
SRC_FILES = cJSON.c jsontest.c
OBJ_FILES = $(SRC_FILES:.c=.o)
EXECUTABLE = json_test

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ_FILES)
	${CC} $(OBJ_FILES) -o $(EXECUTABLE)

%.o: %.c $(DEPS)
	${CC} $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ_FILES) $(EXECUTABLE)