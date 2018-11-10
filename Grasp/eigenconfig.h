/**
  * Weiwei Wan (wanweiwei07@gmail.com),
  * configure eigen for serialization, etc
  */

#ifndef EIGENCONFIG_H
#define EIGENCONFIG_H

#include <iostream>
#include <fstream>
#include <string>

#include <boost/serialization/array.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <Eigen/Core>
#include "eigenboostserialization.h"

// Boost Serialization Helper

template <typename T>
bool enSerialize(const T& data, const std::string& filename) {
    std::ofstream ofs(filename.c_str(), std::ios::out);
    if (!ofs.is_open()) {
        return false;
    }
    else {
        boost::archive::text_oarchive oa(ofs);
        oa << data;
    }
    ofs.close();
    return true;
}

template <typename T>
bool deSerialize(T& data, const std::string& filename) {
    std::ifstream ifs(filename.c_str(), std::ios::in);
    if (!ifs.is_open()) {
      return false;
    }
    else {
      boost::archive::text_iarchive ia(ifs);
      ia >> data;
    }
    ifs.close();
    return true;
}

#endif // EIGENCONFIG_H

