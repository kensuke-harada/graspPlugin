#ifndef _PCL_EIGENINTERNALMATH_H_
#define _PCL_EIGENINTERNALMATH_H_

#include <math.h>
#include <Eigen/Core>

namespace Eigen {
	namespace internal {
		template<typename Scalar> Scalar sqrt(Scalar x) { return std::sqrt(x);}
		template<typename Scalar> Scalar cos(Scalar x) { return std::cos(x);}
		template<typename Scalar> Scalar sin(Scalar x) { return std::sin(x);}
	}
}

#endif
