cmake_minimum_required(VERSION 2.7)

find_package(Boost COMPONENTS system filesystem REQUIRED)

add_executable(train
	train.cpp
	../AbstractClassifier.cpp
	../RandomForestClassifier.cpp
	../SVMClassifier.cpp
	../libsvm/svm.cpp
	)

target_link_libraries(train
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	)

add_executable(predict
	predict.cpp
	../AbstractClassifier.cpp
	../RandomForestClassifier.cpp
	../SVMClassifier.cpp
	../libsvm/svm.cpp
	)

target_link_libraries(predict
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	)
