//
// Created by root on 10/7/21.
//

#ifndef LIBHUMAN_UTIL_IMPL_H
#define LIBHUMAN_UTIL_IMPL_H

namespace human {
    namespace util {
//==============================================================================
        template<class T>
        T getRosParam(const std::string &paramName, const ros::NodeHandle &nh) {
            T value;
            if (!nh.getParam(paramName, value)) {
                throw std::runtime_error("Failed to load ros parameter " + paramName);
            }
            return value;
        }
    } // namespace util
} // namespace ada


#endif //LIBHUMAN_UTIL_IMPL_H
