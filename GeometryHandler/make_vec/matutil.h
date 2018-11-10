
#ifndef MATUTIL
#define MATUTIL

#include <stdio.h>
#include <stdlib.h>
#ifndef SCALAR
    //#define SCALAR float
    #define SCALAR double
#endif
typedef SCALAR *vector, **matrix;

void error(char *message);
vector newvec(int n);
matrix newmat(int nrow, int ncol);
vector new_vector(int n);
matrix new_matrix(int nrow, int ncol);
void free_vector(vector v);
void free_matrix(matrix a);
double innerproduct(int n, vector u, vector v);
void vecprint(vector v, int n, int perline, char *format);
void matprint(matrix a, int ncol, int perline, char *format);


#endif  /* ç≈èâÇÃ #ifndef ... Ç…ëŒâûÇ∑ÇÈ */
