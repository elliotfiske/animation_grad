CC=g++ -m64
LD=g++ -m64
SRC=sparseblock.cpp Camera.cpp Cloth.cpp GLSL.cpp HUD.cpp MatrixStack.cpp Particle.cpp Program.cpp Scene.cpp Shape.cpp Spring.cpp tiny_obj_loader.cc MosekSolver.cpp
OBJ=build/sparseblock.o build/Camera.o build/Cloth.o build/GLSL.o build/HUD.o build/main.o build/MatrixStack.o build/Particle.o build/Program.o build/Scene.o build/Shape.o build/Spring.o build/tiny_obj_loader.o build/MosekSolver.o

CFLAGS=-pedantic -Wno-deprecated -g -O3 -std=c++11
INC=-I$(EIGEN3_INCLUDE_DIR) -I$(MOSEK_DIR)/h -I.
LIBPATHS=-L$(MOSEK_DIR)/bin
LIB=-DGL_GLEXT_PROTOTYPES -lglut -framework OpenGL
LDOPT= -Wl,-rpath,@loader_path$(MOSEK_DIR)/bin -pthread -lSystem -lm -Xlinker -bind_at_load
LIB_OSX=-framework GLUT -framework OpenGL
MOSEKLIB=-lmosek64



osx: $(OBJ)
	$(LD) $(LIBPATHS) build/*.o $(MOSEKLIB) $(LDOPT) $(LIB_OSX) -o clothsim
	install_name_tool -change @loader_path/libmosek64.7.1.dylib $(MOSEK_DIR)/bin/libmosek64.7.1.dylib clothsim

build/tiny_obj_loader.o: tiny_obj_loader.cc | build
	$(CC) $(CFLAGS) $(INC) $< -c -o $@

build/%.o : %.cpp | build
	$(CC) $(CFLAGS) $(INC) $< -c -o $@

build:
	mkdir build
clean:
	rm -rf build clothsim