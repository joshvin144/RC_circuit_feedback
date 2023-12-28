

/*
 * @description: Calculates the derivative
 * @param: A pointer to the destination array
 * @param: The number of elements in the destination array
 * @param: A pointer to the source array
 * @param: The number of elements in the source array
 * @note: The number of elements in the desination array must be equal to (the number of elements in the source array) - 1
 */
void math_helpers_derivative(float* dest, uint32_t dest_size, float* src, uint32_t src_size);


/*
 * @description: Calculates the left-sided Riemman Sum to approximate the Integral
 * @param: A pointer to the source array
 * @param: The number of elements in the source array
 */
float math_helpers_riemman_sum_approximation(float* src, uint32_t src_size);


/*
 * @description: Calculates the Integral through the Trapezoid Approximation
 * @param: A pointer to the source array
 * @param: The number of elements in the source array
 */
float math_helpers_trapezoid_approximation(float* src, uint32_t src_size);


/*
 * @description: Calculates the Riemman Sum to approximate the Integral
 * @param: A pointer to an array; each element represents the output of a generic function evaluated at a time point
 * @param: The number of elements in the output array
 * @param: A pointer to an array: each element represents the output of a filter function evaluated at a time point
 * @param: The number of elements in the filter array
 * @param: The minimum time point to evaluate the integral at
 * @param: The maximum time point to evaluate the integral at
 * @param: A time point to evaluate the filter function at
 * @note: 
 */
float math_helpers_convolution_step(float* func, uint32_t func_size,
	                                float* filt, uint32_t filt_size,
	                                uint32_t a, uint32_t b,
	                                uint32_t x);

