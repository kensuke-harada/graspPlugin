all: viewer_vec  

INCPATH = $(INCALL) -I./

LIBGL = -lglut -lGLU

DEFS =  #-DMAKE_CLUSTER_HIERARCHY
LIBS = viewer_vec.o  file_handle.o object_handle.o vec_handle.o mymath_float.o mymath.o list_handle.o

#viewer_vec.o mymath_float.o file_handle.o list_handle.o mymath.o object_handle.o vec_handle.o:
$(LIBS):
	$(CXX) -c  $(DEFS)  $(*F).c


viewer_vec: $(LIBS)
	$(CXX)  -o viewer_vec $(DEFS) $(LIBS) $(LIBGL)  


clean:
	rm -f *.o *~ *.log make_vec


file_handle.o: file_handle.c set_define.h struct.h file_handle.h \
  list_handle.h
list_handle.o: list_handle.c struct.h set_define.h
viewer_vec.o: viewer_vec.c set_define.h struct.h list_handle.h \
  file_handle.h object_handle.h  
mymath.o: mymath.c
mymath_float.o: mymath_float.c
object_handle.o: object_handle.c set_define.h struct.h list_handle.h \
  mymath.h mymath_float.h
vec_handle.o: vec_handle.c struct.h