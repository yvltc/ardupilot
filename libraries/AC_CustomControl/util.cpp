#include "util.h"

// Softmax function using Eigen
Eigen::ArrayXf softmax(const Eigen::ArrayXf& logits) {
    // Subtract the maximum value for numerical stability
    Eigen::ArrayXf exp_logits = (logits - logits.maxCoeff()).exp();

    // Calculate softmax probabilities
    return exp_logits / exp_logits.sum();
}

// Function to convert Eigen matrix to string
// std::string matrixToString(const Eigen::MatrixXf& matrix) {
//     std::ostringstream oss;
//     oss << matrix; // Use the stream insertion operator provided by Eigen
//     return oss.str();
//}
