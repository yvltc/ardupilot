// util.h
#ifndef UTIL_H
#define UTIL_H
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma push_macro("_GLIBCXX_USE_C99_STDIO")
#undef _GLIBCXX_USE_C99_STDIO
#include <stdio.h>
#include <Eigen/Dense>
#pragma GCC diagnostic pop
#pragma pop_macro("_GLIBCXX_USE_C99_STDIO")
// Declaration of the softmax function
Eigen::ArrayXf softmax(const Eigen::ArrayXf& logits);

//std::string matrixToString(const Eigen::MatrixXf& matrix);

#endif 
