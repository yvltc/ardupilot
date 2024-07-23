#include "util.h"

// Function to get the last column of a 2D vector
std::vector<float> getLastColumn(const std::vector<std::vector<float>>& matrix) {
    std::vector<float> last_column;
    for (const auto& row : matrix) {
        if (!row.empty()) {
            last_column.push_back(row.back());  // Get the last element of the row
        }
    }
    return last_column;
}

// Function to clamp elements of a vector to the range [-1, 1]
void clampToRange(std::vector<float>& vec, float min_val = -1.0, float max_val = 1.0) {
    for (float& element : vec) {
        element = std::max(min_val, std::min(element, max_val));
    }
}

std::vector<float> vecCat(const std::vector<float>& vec1, const std::vector<float>& vec2) {
    std::vector<float> result = vec1; // Copy the first vector into the result
    result.insert(result.end(), vec2.begin(), vec2.end()); // Insert the second vector at the end of the result
    return result;
}
// Function to perform vector addition
std::vector<float> vecAdd(const std::vector<float>& vec1, const std::vector<float>& vec2) {
    assert(vec1.size() == vec2.size());
    std::vector<float> result(vec1.size(), 0.0);
    for (size_t i = 0; i < vec1.size(); ++i) {
        result[i] = vec1[i] + vec2[i];
    }
    return result;
}
std::vector<std::vector<float>> vec2DAdd(const std::vector<std::vector<float>>& c1, const std::vector<std::vector<float>>& buf) {
    // Ensure that the dimensions match
    assert(c1.size() == buf.size() && c1[0].size() == buf[0].size());

    // Create a result vector with the same dimensions
    std::vector<std::vector<float>> result(c1.size(), std::vector<float>(c1[0].size(), 0.0));

    for (size_t i = 0; i < c1.size(); ++i) {
        for (size_t j = 0; j < c1[i].size(); ++j) {
            result[i][j] = c1[i][j] + buf[i][j];
        }
    }

    return result;
}

// Function to apply the ReLU activation
std::vector<float> relu(const std::vector<float>& x) {
    std::vector<float> result(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        result[i] = std::max(float(0.0), x[i]);
    }
    return result;
}

// Function to apply ReLU to a 2D vector
std::vector<std::vector<float>> relu2D(const std::vector<std::vector<float>>& input) {
    std::vector<std::vector<float>> result = input;  // Create a copy of the input

    for (auto& row : result) {
        for (auto& element : row) {
            element = std::max(float(0.0), element);  // Apply ReLU
        }
    }

    return result;
}
// Function to apply the softmax activation
std::vector<float> softmax(const std::vector<float>& x) {
    std::vector<float> exp_x(x.size());
    float sum_exp = 0.0;
    for (size_t i = 0; i < x.size(); ++i) {
        exp_x[i] = std::exp(x[i]);
        sum_exp += exp_x[i];
    }
    for (size_t i = 0; i < x.size(); ++i) {
        exp_x[i] /= sum_exp;
    }
    return exp_x;
}

std::vector<std::vector<float>> chomp1d(const std::vector<std::vector<float>>& input, int padding) {
    std::vector<std::vector<float>> result;

    for (const auto& row : input) {
        // Create a new row with the required number of elements removed from the end
        std::vector<float> new_row(row.begin(), row.end() - padding);
        result.push_back(new_row);
    }

    return result;
}

// Helper function to apply padding
std::vector<std::vector<float>> applyPadding(const std::vector<std::vector<float>>& input, int padding) {
    int C_in = input.size();
    int L_in = input[0].size();
    int padded_L_in = L_in + 2 * padding;

    std::vector<std::vector<float>> padded_input(C_in, std::vector<float>(padded_L_in, 0.0));

    for (int c_in = 0; c_in < C_in; ++c_in) {
        for (int l_in = 0; l_in < L_in; ++l_in) {
            padded_input[c_in][l_in + padding] = input[c_in][l_in];
        }
    }

    return padded_input;
}

std::vector<std::vector<float>> conv1d(
    const std::vector<std::vector<float>>& input,
    const std::vector<std::vector<std::vector<float>>>& weight,
    const std::vector<float>& bias,
    int stride = 1,
    int padding = 0,
    int dilation = 1) {

    int C_in = input.size();
    int L_in = input[0].size();
    int C_out = weight.size();
    int K = weight[0][0].size();

    // Calculate the output length
    int L_out = (L_in + 2 * padding - dilation * (K - 1) - 1) / stride + 1;

    // Initialize the output tensor
    std::vector<std::vector<float>> output(C_out, std::vector<float>(L_out, 0.0));

    // Apply padding to the input
    std::vector<std::vector<float>> input_padded = applyPadding(input, padding);

    // Perform the convolution
    for (int c_out = 0; c_out < C_out; ++c_out) {
        for (int l_out = 0; l_out < L_out; ++l_out) {
            for (int c_in = 0; c_in < C_in; ++c_in) {
                for (int k = 0; k < K; ++k) {
                    int l_in = l_out * stride + k * dilation;
                    output[c_out][l_out] += input_padded[c_in][l_in] * weight[c_out][c_in][k];
                }
            }
            if (!bias.empty()) {
                output[c_out][l_out] += bias[c_out];
            }
        }
    }

    return output;
}

// Function to perform vector dot product
float vecDot(const std::vector<float>& vec1, const std::vector<float>& vec2) {
    assert(vec1.size() == vec2.size());
    float result = 0.0;
    for (size_t i = 0; i < vec1.size(); ++i) {
        result += vec1[i] * vec2[i];
    }
    return result;
}

// Function to perform matrix-vector multiplication
std::vector<float> matVecMul(const std::vector<std::vector<float>>& mat, const std::vector<float>& vec) {
    assert(mat[0].size() == vec.size());
    std::vector<float> result(mat.size(), 0.0);
    for (size_t i = 0; i < mat.size(); ++i) {
        for (size_t j = 0; j < vec.size(); ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }
    return result;
}


// Function to apply the linear layer with activation
std::vector<float> linear_layer(const std::vector<float>& bias, const std::vector<std::vector<float>>& weight, const std::vector<float>& input, bool activation) {
    // Compute the linear transformation
    std::vector<float> output = vecAdd(bias, matVecMul(weight, input));

    // Apply the activation function
    if (activation) {
        return relu(output);
    }
    return output;
}

// Function to apply the composition layer
std::vector<float> composition_layer(
    const std::vector<std::vector<std::vector<float>>>& weight, // [contextdim][outdim][indim]
    const std::vector<std::vector<float>>& bias,               // [contextdim][outdim]
    const std::vector<float>& comp_weight,                     // [contextdim]
    const std::vector<float>& input,                           // [indim]
    bool activation = true) {
    // Ensure dimensions are correct
    assert(weight.size() == bias.size());
    assert(weight[0].size() == bias[0].size());
    assert(weight[0][0].size() == input.size());
    assert(comp_weight.size() == weight.size());

    size_t contextdim = weight.size();
    size_t outdim = weight[0].size();
    
    // Compute x = bias + weight * input for each context dimension
    std::vector<std::vector<float>> x(contextdim, std::vector<float>(outdim));
    for (size_t i = 0; i < contextdim; ++i) {
        x[i] = vecAdd(bias[i], matVecMul(weight[i], input));
    }

    // Compute output = comp_weight * sum(x)
    std::vector<float> output(outdim, 0.0);
    for (size_t i = 0; i < contextdim; ++i) {
        for (size_t j = 0; j < outdim; ++j) {
            output[j] += comp_weight[i] * x[i][j];
        }
    }

    // Apply activation function
    if (activation) {
        return relu(output);
    } else {
        return output;
    }
}