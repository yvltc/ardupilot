# pragma once

#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <cassert>
#include <cmath>

std::vector<float> getLastColumn(const std::vector<std::vector<float>>& matrix);

void clampToRange(std::vector<float>& vec, float min_val, float max_val);

std::vector<float> vecCat(const std::vector<float>& vec1, const std::vector<float>& vec2);
std::vector<float> vecAdd(const std::vector<float>& vec1, const std::vector<float>& vec2);
std::vector<std::vector<float>> vec2DAdd(const std::vector<std::vector<float>>& c1, const std::vector<std::vector<float>>& buf);

std::vector<float> relu(const std::vector<float>& x);
std::vector<std::vector<float>> relu2D(const std::vector<std::vector<float>>& input);
std::vector<float> softmax(const std::vector<float>& x);

std::vector<std::vector<float>> chomp1d(const std::vector<std::vector<float>>& input, int padding);

std::vector<std::vector<float>> applyPadding(const std::vector<std::vector<float>>& input, int padding);

std::vector<std::vector<float>> conv1d(
    const std::vector<std::vector<float>>& input,
    const std::vector<std::vector<std::vector<float>>>& weight,
    const std::vector<float>& bias,
    int stride, int padding, int dilation);

float vecDot(const std::vector<float>& vec1, const std::vector<float>& vec2);
std::vector<float> matVecMul(const std::vector<std::vector<float>>& mat, const std::vector<float>& vec);
std::vector<float> linear_layer(const std::vector<float>& bias, const std::vector<std::vector<float>>& weight, const std::vector<float>& input, bool activation);
std::vector<float> composition_layer(
    const std::vector<std::vector<std::vector<float>>>& weight, // [contextdim, outdim, indim]
    const std::vector<std::vector<float>>& bias,               // [contextdim, outdim]
    const std::vector<float>& comp_weight,                     // [contextdim]
    const std::vector<float>& input,                           // [indim]
    bool activation);
#endif 
