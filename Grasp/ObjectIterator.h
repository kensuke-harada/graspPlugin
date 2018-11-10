#ifndef _GRASP_OBJECTITERATOR_H_
#define _GRASP_OBJECTITERATOR_H_

#include <iterator>

#include <boost/function.hpp>

#include "ObjectManager.h"

namespace grasp {
	template <class Type>
	class ObjectIterator :
		public std::iterator<std::forward_iterator_tag, ObjectBase*> {
		friend class ObjectManager;

	public:
		ObjectIterator(const ObjectIterator<Type>& iterator);
		ObjectIterator& operator=(const ObjectIterator<Type>& iterator);

		ObjectIterator& operator++();
		ObjectIterator operator++(int);
		ObjectIterator operator+(size_t n);

		Type* operator*();

		bool operator==(const ObjectIterator<Type>& iterator);
		bool operator!=(const ObjectIterator<Type>& iterator);

	private:
		typedef ObjectManager::ObjectContainerIte ObjectContainerIte;
		ObjectIterator();
		ObjectIterator(ObjectManager* om, boost::function<bool(const ObjectBase*)> check_func, bool begin);

		ObjectManager* om_;
		ObjectContainerIte ite_;
		boost::function<bool(const ObjectBase*)> check_function_;
	};
}

#include "ObjectIterator.hpp"

#endif /* _GRASP_OBJECTITERATOR_H_ */
