# with help from https://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/ and some digging

CC = g++ # gcc creates error
CFLAGS = -std=c++11 -I./include

SRC_DEPS = $(shell find include -type f -name '*.h')
SRC_DEPS += $(shell find include -type f -name '*.hpp')
SRC_C = $(shell find src -type f -name '*.cpp')

OBJ = $(addprefix build/, $(addsuffix .o, $(basename $(SRC_C))) )

build/%.o: %.cpp ${SRC_DEPS}
	$(info CC $<)
	@mkdir -p $(dir $@)
	@$(CC) -c -o $@ $< $(CFLAGS)

all: $(OBJ)
	$(info LINK $@)
	@$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean
clean:
	$(info cleaning)
	@rm -rf build
