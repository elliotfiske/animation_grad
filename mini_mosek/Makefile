#Gnu Make / MacOSX 64x86 / GCC

INCPATHS=-I/Users/elliotfiske/mosek/7/tools/platform/osx64x86/h -I.
LIBPATHS=-L/Users/elliotfiske/mosek/7/tools/platform/osx64x86/bin
MOSEKLIB=-lmosek64
CCOPT=
LDOPT=-Wl,-rpath,@loader_path~/mosek/7/tools/platform/osx64x86/bin -pthread -lSystem -lm -Xlinker -bind_at_load
CC=gcc -m64
LD=gcc -m64

lo1: lo1.c
	$(CC) -c $(INCPATHS)          $(CCOPT) -o lo1.o lo1.c
	$(LD) $(LIBPATHS) lo1.o $(MOSEKLIB) $(LDOPT) -o lo1
	install_name_tool -change @loader_path/libmosek64.7.1.dylib ../../platform/osx64x86/bin/libmosek64.7.1.dylib lo1


#
# These targets do not produce output of targetname:
#
.PHONY: all clean cleanall test

all: $(EXAMPLES)

clean:
	rm -f *.o $(EXAMPLES)
