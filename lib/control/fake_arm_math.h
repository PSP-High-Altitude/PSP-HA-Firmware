/* All these headers are directly copied from the real arm_math.h */
#ifndef _ARM_MATH_H
#define _ARM_MATH_H

#include <stdint.h>

// TYPEDEFS ///////////////////////////////////////////////////////////////////

/**
 * @brief Error status returned by some functions in the library.
 */

typedef enum {
    ARM_MATH_SUCCESS = 0,         /**< No error */
    ARM_MATH_ARGUMENT_ERROR = -1, /**< One or more arguments are incorrect */
    ARM_MATH_LENGTH_ERROR = -2,   /**< Length of data buffer is incorrect */
    ARM_MATH_SIZE_MISMATCH =
        -3, /**< Size of matrices is not compatible with the operation */
    ARM_MATH_NANINF = -4, /**< Not-a-number (NaN) or infinity is generated */
    ARM_MATH_SINGULAR =
        -5, /**< Input matrix is singular and cannot be inverted */
    ARM_MATH_TEST_FAILURE = -6 /**< Test Failed */
} arm_status;

/**
 * @brief 32-bit floating-point type definition.
 */
typedef float float32_t;

/**
 * @brief Instance structure for the floating-point matrix structure.
 */
typedef struct {
    uint16_t numRows; /**< number of rows of the matrix.     */
    uint16_t numCols; /**< number of columns of the matrix.  */
    float32_t* pData; /**< points to the data of the matrix. */
} arm_matrix_instance_f32;

// FUNCTIONS //////////////////////////////////////////////////////////////////

/**
 * @brief  Copies the elements of a floating-point vector.
 * @param[in]  pSrc       input pointer
 * @param[out] pDst       output pointer
 * @param[in]  blockSize  number of samples to process
 */
void arm_copy_f32(const float32_t* pSrc, float32_t* pDst, uint32_t blockSize);

/**
 * @brief Floating-point matrix multiplication
 * @param[in]  pSrcA  points to the first input matrix structure
 * @param[in]  pSrcB  points to the second input matrix structure
 * @param[out] pDst   points to output matrix structure
 * @return     The function returns either
 * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on
 * the outcome of size checking.
 */
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32* pSrcA,
                            const arm_matrix_instance_f32* pSrcB,
                            arm_matrix_instance_f32* pDst);

/**
 * @brief Floating-point matrix addition.
 * @param[in]  pSrcA  points to the first input matrix structure
 * @param[in]  pSrcB  points to the second input matrix structure
 * @param[out] pDst   points to output matrix structure
 * @return     The function returns either
 * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on
 * the outcome of size checking.
 */
arm_status arm_mat_add_f32(const arm_matrix_instance_f32* pSrcA,
                           const arm_matrix_instance_f32* pSrcB,
                           arm_matrix_instance_f32* pDst);

/**
 * @brief Floating-point vector addition.
 * @param[in]  pSrcA      points to the first input vector
 * @param[in]  pSrcB      points to the second input vector
 * @param[out] pDst       points to the output vector
 * @param[in]  blockSize  number of samples in each vector
 */
void arm_add_f32(const float32_t* pSrcA, const float32_t* pSrcB,
                 float32_t* pDst, uint32_t blockSize);

/**
 * @brief Floating-point matrix transpose.
 * @param[in]  pSrc  points to the input matrix
 * @param[out] pDst  points to the output matrix
 * @return    The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
 * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
 */
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32* pSrc,
                             arm_matrix_instance_f32* pDst);

/**
 * @brief Floating-point matrix inverse.
 * @param[in]  src   points to the instance of the input floating-point matrix
 * structure.
 * @param[out] dst   points to the instance of the output floating-point matrix
 * structure.
 * @return The function returns ARM_MATH_SIZE_MISMATCH, if the dimensions do not
 * match. If the input matrix is singular (does not have an inverse), then the
 * algorithm terminates and returns error status ARM_MATH_SINGULAR.
 */
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32* src,
                               arm_matrix_instance_f32* dst);

/**
 * @brief  Fills a constant value into a floating-point vector.
 * @param[in]  value      input value to be filled
 * @param[out] pDst       output pointer
 * @param[in]  blockSize  number of samples to process
 */
void arm_fill_f32(float32_t value, float32_t* pDst, uint32_t blockSize);

#endif  // ARM_MATH_H
