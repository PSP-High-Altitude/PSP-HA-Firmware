/**
 * @file arm_math.c
 * @brief Fake arm math - just the functions needed to run the kalman filter.
 * Some math from chatgpt, still need to test
 * @date 2025-01-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "fake_arm_math.h"

#include <string.h>

#include "math.h"

void arm_copy_f32(const float32_t* pSrc, float32_t* pDst, uint32_t blockSize) {
    memcpy(pDst, pSrc, blockSize * sizeof(float32_t));
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32* pSrcA,
                            const arm_matrix_instance_f32* pSrcB,
                            arm_matrix_instance_f32* pDst) {
    /* From chatgpt */
    // Check for size mismatch
    if (pSrcA->numCols != pSrcB->numRows || pDst->numRows != pSrcA->numRows ||
        pDst->numCols != pSrcB->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    // Matrix dimensions
    uint16_t A_rows = pSrcA->numRows;
    uint16_t A_cols = pSrcA->numCols;
    uint16_t B_cols = pSrcB->numCols;

    // Pointer to data
    const float32_t* A = pSrcA->pData;
    const float32_t* B = pSrcB->pData;
    float32_t* C = pDst->pData;

    // Perform matrix multiplication
    for (uint16_t i = 0; i < A_rows; i++) {
        for (uint16_t j = 0; j < B_cols; j++) {
            float32_t sum = 0.0f;
            for (uint16_t k = 0; k < A_cols; k++) {
                sum += A[i * A_cols + k] * B[k * B_cols + j];
            }
            C[i * B_cols + j] = sum;
        }
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32* pSrcA,
                           const arm_matrix_instance_f32* pSrcB,
                           arm_matrix_instance_f32* pDst) {
    /* chatgpt */
    // Check for size mismatch
    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols ||
        pDst->numRows != pSrcA->numRows || pDst->numCols != pSrcA->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    // Matrix dimensions
    uint16_t numRows = pSrcA->numRows;
    uint16_t numCols = pSrcA->numCols;
    uint32_t totalElements = numRows * numCols;

    // Pointer to data
    const float32_t* A = pSrcA->pData;
    const float32_t* B = pSrcB->pData;
    float32_t* C = pDst->pData;

    // Perform element-wise addition
    for (uint32_t i = 0; i < totalElements; i++) {
        C[i] = A[i] + B[i];
    }

    return ARM_MATH_SUCCESS;
}

void arm_add_f32(const float32_t* pSrcA, const float32_t* pSrcB,
                 float32_t* pDst, uint32_t blockSize) {
    for (int i = 0; i < blockSize; i++) {
        pDst[i] = pSrcA[i] + pSrcB[i];
    }
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32* pSrc,
                             arm_matrix_instance_f32* pDst) {
    // VERIFIED this works
    // Check for size mismatch
    if (pSrc->numRows != pDst->numCols || pSrc->numCols != pDst->numRows) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    // Matrix dimensions
    uint16_t srcRows = pSrc->numRows;
    uint16_t srcCols = pSrc->numCols;

    // Pointer to data
    const float32_t* srcData = pSrc->pData;
    float32_t* dstData = pDst->pData;

    // Perform matrix transpose
    for (uint16_t i = 0; i < srcRows; i++) {
        for (uint16_t j = 0; j < srcCols; j++) {
            // Transpose element at (i, j) to (j, i)
            dstData[j * srcRows + i] = srcData[i * srcCols + j];
        }
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32* src,
                               arm_matrix_instance_f32* dst) {
    uint16_t n = src->numRows;

    // Check if the input is square
    if (src->numRows != src->numCols || dst->numRows != dst->numCols ||
        dst->numRows != n) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    // Allocate temporary storage for augmented matrix
    float32_t aug[n * 2 * n];  // [src | I]
    float32_t* pAug = aug;

    // Initialize augmented matrix [src | I]
    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            pAug[i * 2 * n + j] = src->pData[i * n + j];  // Copy src
            pAug[i * 2 * n + j + n] =
                (i == j) ? 1.0f : 0.0f;  // Identity matrix
        }
    }

    // Perform Gaussian elimination
    for (uint16_t i = 0; i < n; i++) {
        // Find the pivot row
        float32_t pivot = fabs(pAug[i * 2 * n + i]);
        uint16_t pivotRow = i;
        for (uint16_t k = i + 1; k < n; k++) {
            if (fabs(pAug[k * 2 * n + i]) > pivot) {
                pivot = fabs(pAug[k * 2 * n + i]);
                pivotRow = k;
            }
        }

        // If pivot is zero, the matrix is singular
        if (pivot == 0.0f) {
            return ARM_MATH_SINGULAR;
        }

        // Swap the pivot row with the current row
        if (pivotRow != i) {
            for (uint16_t j = 0; j < 2 * n; j++) {
                float32_t temp = pAug[i * 2 * n + j];
                pAug[i * 2 * n + j] = pAug[pivotRow * 2 * n + j];
                pAug[pivotRow * 2 * n + j] = temp;
            }
        }

        // Normalize the pivot row
        float32_t pivotValue = pAug[i * 2 * n + i];
        for (uint16_t j = 0; j < 2 * n; j++) {
            pAug[i * 2 * n + j] /= pivotValue;
        }

        // Eliminate other rows
        for (uint16_t k = 0; k < n; k++) {
            if (k != i) {
                float32_t factor = pAug[k * 2 * n + i];
                for (uint16_t j = 0; j < 2 * n; j++) {
                    pAug[k * 2 * n + j] -= factor * pAug[i * 2 * n + j];
                }
            }
        }
    }

    // Extract the inverse matrix from the augmented matrix
    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            dst->pData[i * n + j] = pAug[i * 2 * n + j + n];
        }
    }

    return ARM_MATH_SUCCESS;
}

void arm_fill_f32(float32_t value, float32_t* pDst, uint32_t blockSize) {
    for (int i = 0; i < blockSize; i++) {
        pDst[i] = value;
    }
}