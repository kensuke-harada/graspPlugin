using namespace grasp;

template <class Type>
ObjectIterator<Type>::ObjectIterator(const ObjectIterator<Type>& iterator) {
	om_ = iterator.om_;
	ite_ = iterator.ite_;
	check_function_ = iterator.check_function_;
}

template <class Type>
ObjectIterator<Type>& ObjectIterator<Type>::operator=(const ObjectIterator<Type>& iterator) {
	om_ = iterator.om_;
	ite_ = iterator.ite_;
	check_function_ = iterator.check_function_;
	return *this;
}

template <class Type>
ObjectIterator<Type>& ObjectIterator<Type>::operator++() {
	ite_++;
	while (ite_ != om_->objects_.end()) {
		if (check_function_((*ite_).second.get())) break;
		ite_++;
	}
	return *this;
}

template <class Type>
ObjectIterator<Type> ObjectIterator<Type>::operator++(int) {
	ObjectIterator<Type> ret = *this;
	ite_++;
	while (ite_ != om_->objects_.end()) {
		if (check_function_((*ite_).second.get())) break;
		ite_++;
	}
	return ret;
}

template <class Type>
ObjectIterator<Type> ObjectIterator<Type>::operator+(size_t n) {
	ObjectIterator<Type> ret = *this;
	for (size_t i = 0; i < n; i++) {
		if (ret.ite_ == om_->objects_.end()) break;
		ret++;
	}
	return ret;
}

template <class Type>
Type* ObjectIterator<Type>::operator*() {
	return (ite_ != om_->objects_.end()) ? static_cast<Type*>((*ite_).second.get()) : NULL;
}

template <class Type>
bool ObjectIterator<Type>::operator==(const ObjectIterator<Type>& iterator) {
	return !(*this != iterator);
}

template <class Type>
bool ObjectIterator<Type>::operator!=(const ObjectIterator<Type>& iterator) {
	return ((this->om_ != iterator.om_) || (this->ite_ != iterator.ite_));
}

template <class Type>
ObjectIterator<Type>::ObjectIterator() {
}

template <class Type>
ObjectIterator<Type>::ObjectIterator(ObjectManager* om, boost::function<bool(const ObjectBase*)> check_func, bool begin) {
	om_ = om;
	check_function_ = check_func;
	if (begin) {
		for (ObjectContainerIte it = om->objects_.begin(); it != om->objects_.end(); ++it) {
			if (check_function_((*it).second.get())) {
				ite_ = it;
				return;
			}
		}
	}
	ite_ = om->objects_.end();
}
