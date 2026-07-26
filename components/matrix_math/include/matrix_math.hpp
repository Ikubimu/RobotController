#ifndef MATRIX_MATH_HPP
#define MATRIX_MATH_HPP

#include <cstdint>
#include <cmath>

#define MAT_MAX_DIM 8

struct Matrix {
    float data[MAT_MAX_DIM * MAT_MAX_DIM];
    uint8_t rows;
    uint8_t cols;
};

namespace MatMath {

Matrix create(uint8_t rows, uint8_t cols);
void   zeros(Matrix &m);
void   identity(Matrix &m);
Matrix multiply(const Matrix &a, const Matrix &b);
Matrix transpose(const Matrix &m);
Matrix inverse4(const Matrix &m);
Matrix pinv(const Matrix &m);
Matrix cross(const Matrix &a, const Matrix &b);
float  dot(const Matrix &a, const Matrix &b);
Matrix scale(const Matrix &m, float s);
Matrix add(const Matrix &a, const Matrix &b);
Matrix sub(const Matrix &a, const Matrix &b);

void print(const Matrix &m);

}

#endif
