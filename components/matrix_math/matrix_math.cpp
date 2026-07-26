#include "matrix_math.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>

namespace MatMath {

Matrix create(uint8_t rows, uint8_t cols) {
    Matrix m;
    m.rows = rows;
    m.cols = cols;
    zeros(m);
    return m;
}

void zeros(Matrix &m) {
    memset(m.data, 0, sizeof(m.data));
}

void identity(Matrix &m) {
    zeros(m);
    uint8_t n = m.rows < m.cols ? m.rows : m.cols;
    for (uint8_t i = 0; i < n; i++) {
        m.data[i * m.cols + i] = 1.0f;
    }
}

Matrix multiply(const Matrix &a, const Matrix &b) {
    Matrix r = create(a.rows, b.cols);
    uint8_t m = a.rows;
    uint8_t n = b.cols;
    uint8_t p = a.cols;

    for (uint8_t i = 0; i < m; i++) {
        uint8_t ri = i * n;
        uint8_t ai = i * p;
        for (uint8_t k = 0; k < p; k++) {
            float a_ik = a.data[ai + k];
            uint8_t bk = k * n;
            for (uint8_t j = 0; j < n; j++) {
                r.data[ri + j] += a_ik * b.data[bk + j];
            }
        }
    }
    return r;
}

Matrix transpose(const Matrix &m) {
    Matrix r = create(m.cols, m.rows);
    for (uint8_t i = 0; i < m.rows; i++) {
        for (uint8_t j = 0; j < m.cols; j++) {
            r.data[j * r.cols + i] = m.data[i * m.cols + j];
        }
    }
    return r;
}

Matrix inverse4(const Matrix &m) {
    Matrix r = create(4, 4);

    float R[3][3], R_inv[3][3];
    float p[3];

    for (uint8_t i = 0; i < 3; i++) {
        p[i] = m.data[i * 4 + 3];
        for (uint8_t j = 0; j < 3; j++) {
            R[i][j] = m.data[i * 4 + j];
        }
    }

    float det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
              - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
              + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);

    if (fabsf(det) < 1e-10f) {
        identity(r);
        return r;
    }

    float inv_det = 1.0f / det;

    R_inv[0][0] = (R[1][1] * R[2][2] - R[1][2] * R[2][1]) * inv_det;
    R_inv[0][1] = (R[0][2] * R[2][1] - R[0][1] * R[2][2]) * inv_det;
    R_inv[0][2] = (R[0][1] * R[1][2] - R[0][2] * R[1][1]) * inv_det;
    R_inv[1][0] = (R[1][2] * R[2][0] - R[1][0] * R[2][2]) * inv_det;
    R_inv[1][1] = (R[0][0] * R[2][2] - R[0][2] * R[2][0]) * inv_det;
    R_inv[1][2] = (R[0][2] * R[1][0] - R[0][0] * R[1][2]) * inv_det;
    R_inv[2][0] = (R[1][0] * R[2][1] - R[1][1] * R[2][0]) * inv_det;
    R_inv[2][1] = (R[0][1] * R[2][0] - R[0][0] * R[2][1]) * inv_det;
    R_inv[2][2] = (R[0][0] * R[1][1] - R[0][1] * R[1][0]) * inv_det;

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            r.data[i * 4 + j] = R_inv[i][j];
        }
        r.data[i * 4 + 3] = 0.0f;
        for (uint8_t j = 0; j < 3; j++) {
            r.data[i * 4 + 3] -= R_inv[i][j] * p[j];
        }
    }

    r.data[12] = 0.0f;
    r.data[13] = 0.0f;
    r.data[14] = 0.0f;
    r.data[15] = 1.0f;

    return r;
}

static void swap_rows(float *a, float *b, uint8_t cols) {
    for (uint8_t j = 0; j < cols; j++) {
        float tmp = a[j];
        a[j] = b[j];
        b[j] = tmp;
    }
}

Matrix pinv(const Matrix &m) {
    uint8_t r = m.rows;
    uint8_t c = m.cols;

    Matrix mt = transpose(m);
    Matrix mtm = multiply(mt, m);

    uint8_t n = mtm.rows;

    Matrix aug = create(n, n * 2);

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            aug.data[i * aug.cols + j] = mtm.data[i * n + j];
            aug.data[i * aug.cols + (n + j)] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (uint8_t i = 0; i < n; i++) {
        uint8_t pivot = i;
        float max_val = fabsf(aug.data[i * aug.cols + i]);
        for (uint8_t k = i + 1; k < n; k++) {
            float val = fabsf(aug.data[k * aug.cols + i]);
            if (val > max_val) {
                max_val = val;
                pivot = k;
            }
        }
        if (pivot != i) {
            swap_rows(&aug.data[i * aug.cols], &aug.data[pivot * aug.cols], aug.cols);
        }

        float diag = aug.data[i * aug.cols + i];
        if (fabsf(diag) < 1e-10f) continue;

        float inv_diag = 1.0f / diag;
        for (uint8_t j = 0; j < aug.cols; j++) {
            aug.data[i * aug.cols + j] *= inv_diag;
        }

        for (uint8_t k = 0; k < n; k++) {
            if (k == i) continue;
            float factor = aug.data[k * aug.cols + i];
            uint8_t ki = k * aug.cols;
            uint8_t ii = i * aug.cols;
            for (uint8_t j = 0; j < aug.cols; j++) {
                aug.data[ki + j] -= factor * aug.data[ii + j];
            }
        }
    }

    Matrix mtm_inv = create(n, n);
    for (uint8_t i = 0; i < n; i++) {
        uint8_t ai = i * aug.cols;
        uint8_t mi = i * n;
        for (uint8_t j = 0; j < n; j++) {
            mtm_inv.data[mi + j] = aug.data[ai + (n + j)];
        }
    }

    Matrix result = multiply(mtm_inv, mt);
    return result;
}

Matrix cross(const Matrix &a, const Matrix &b) {
    Matrix r = create(3, 1);
    r.data[0] = a.data[1] * b.data[2] - a.data[2] * b.data[1];
    r.data[1] = a.data[2] * b.data[0] - a.data[0] * b.data[2];
    r.data[2] = a.data[0] * b.data[1] - a.data[1] * b.data[0];
    return r;
}

float dot(const Matrix &a, const Matrix &b) {
    float sum = 0.0f;
    uint8_t n = a.rows < b.rows ? a.rows : b.rows;
    for (uint8_t i = 0; i < n; i++) {
        sum += a.data[i] * b.data[i];
    }
    return sum;
}

Matrix scale(const Matrix &m, float s) {
    Matrix r = create(m.rows, m.cols);
    for (uint8_t i = 0; i < m.rows * m.cols; i++) {
        r.data[i] = m.data[i] * s;
    }
    return r;
}

Matrix add(const Matrix &a, const Matrix &b) {
    Matrix r = create(a.rows, a.cols);
    for (uint8_t i = 0; i < a.rows * a.cols; i++) {
        r.data[i] = a.data[i] + b.data[i];
    }
    return r;
}

Matrix sub(const Matrix &a, const Matrix &b) {
    Matrix r = create(a.rows, a.cols);
    for (uint8_t i = 0; i < a.rows * a.cols; i++) {
        r.data[i] = a.data[i] - b.data[i];
    }
    return r;
}

void print(const Matrix &m) {
    for (uint8_t i = 0; i < m.rows; i++) {
        printf("[ ");
        for (uint8_t j = 0; j < m.cols; j++) {
            printf("%8.4f ", m.data[i * m.cols + j]);
        }
        printf("]\n");
    }
    printf("\n");
}

}
