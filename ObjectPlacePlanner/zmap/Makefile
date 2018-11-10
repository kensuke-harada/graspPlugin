VVVINCDIR = /usr/local/VVV/include

INCLUDES = -I $(VVVINCDIR)

CC = gcc -pipe

SRCS = $(shell ls *.c)
OBJS = $(shell ls *.c|sed -e 's/.c$$/.o/')


BIN = make_zmap
OPT = -g -Wall
COMPILE = $(CC) -c $(INCLUDES) $(CFLAGS)
LDFLAGS =
LINK = $(CC) $(LDFLAGS) 

LIBS = -L /usr/local/VVV/lib \
	-lrange \
	-lrngedge \
	-lepbm \
	-lscm \
	-lrng2brep \
	-lbrep \
	-lrngutil \
	-lrngseg \
	-lstepone \
	-lvvvstd \
	-lmatutil \
	-lverbose \
	-lm

.SUFFIXES:
.SUFFIXES: .c .o

.c.o:
	$(COMPILE) $(OPT) $<

all: $(BIN)

$(BIN):$(OBJS)
	$(LINK) $(OPT) -o $(BIN) $(OBJS) $(LIBS)

clean:
	rm $(BIN) $(OBJS)
