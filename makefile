CC = g++
SRCS = src/tracking_class.cpp
CFLAGS_COMMON = -g -Wall -std=c++11 -O0 -fPIC -shared `python3 -m pybind11 --includes` -I ./

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
    CFLAGS = $(CFLAGS_COMMON)
    PROG = src/lib/find_marker.so
endif

ifeq ($(UNAME_S),Darwin)
    CFLAGS = $(CFLAGS_COMMON) -undefined dynamic_lookup
    PROG = src/lib/find_marker`python3-config --extension-suffix`
endif

$(PROG):$(SRCS)
	mkdir -p src/lib
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS)

clean:
	rm -rf src/lib
