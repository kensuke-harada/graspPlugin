#ifndef __DMATRIX_H__
#define __DMATRIX_H__

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>


namespace dmatrix {

    namespace ublas = boost::numeric::ublas;

    //typedef boost::numeric::ublas::matrix<double, ublas::column_major> dMatrix;
	typedef ublas::bounded_matrix<double, 3, 3, ublas::column_major> dMatrix33;
	typedef ublas::bounded_vector<double, 3> dVector3;
};

#define	COPY_DVECTOR3(dst, src)	{ for (int i = 0; i < 3; i++) { (dst)[i] = (src)[i]; } }
#define	COPY_DMATRIX33(dst, src)	{ for (int i = 0; i < 3; i++) {			\
											for (int j = 0; j < 3; j++) {		\
												(dst)(i, j) = (src)[i * 3 + j]; \
											} 									\
										  } }

#endif
