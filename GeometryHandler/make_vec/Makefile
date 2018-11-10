all: make_vec cluster2mesh.o jacobi.o make_cluster.o  mymath_float.o file_handle.o list_handle.o matutil.o mymath.o object_handle.o pm6.o tri_ver_math.o

INCPATH = $(INCALL) -I./

LIBGL = -lglut

DEFS =  -DMAKE_CLUSTER_HIERARCHY
LIBS = model_viewer.o cluster2mesh.o jacobi.o make_cluster.o  mymath_float.o file_handle.o list_handle.o mymath.o object_handle.o pm6.o tri_ver_math.o matutil.o

model_viewer.o cluster2mesh.o jacobi.o make_cluster.o  mymath_float.o file_handle.o list_handle.o mymath.o object_handle.o pm6.o tri_ver_math.o matutil.o:
	$(CXX) -c  $(DEFS)  $(*F).c


make_vec: cluster2mesh.o jacobi.o make_cluster.o  mymath_float.o file_handle.o list_handle.o mymath.o object_handle.o pm6.o tri_ver_math.o model_viewer.o matutil.o
	$(CXX)  -o make_vec $(DEFS) $(LIBS) $(LIBGL)  


clean:
	rm -f *.o *~ *.log make_vec


cluster2mesh.o: cluster2mesh.c set_define.h struct.h mymath.h \
  list_handle.h
file_handle.o: file_handle.c set_define.h struct.h file_handle.h \
  list_handle.h
jacobi.o: jacobi.c set_define.h struct.h mymath.h list_handle.h matutil.h \
  pm6.h
list_handle.o: list_handle.c struct.h set_define.h
make_cluster.o: make_cluster.c set_define.h struct.h mymath.h jacobi.h \
  list_handle.h cluster2mesh.h
matutil.o: matutil.c
model_viewer.o: model_viewer.c set_define.h struct.h list_handle.h \
  file_handle.h object_handle.h tri_ver_math.h make_cluster.h \
  cluster2mesh.h
mymath.o: mymath.c
mymath_float.o: mymath_float.c
object_handle.o: object_handle.c set_define.h struct.h list_handle.h \
  tri_ver_math.h mymath.h mymath_float.h
octree.o: octree.c set_define.h struct.h
pm6.o: pm6.c
tri_ver_math.o: tri_ver_math.c struct.h set_define.h mymath.h
