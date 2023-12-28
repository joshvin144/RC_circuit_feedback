

#include "math_helpers.h"


float calculate_area_rectangle(float height, float width)
{
	return height * width;
}


float calculate_area_triangle(float height, float width)
{
	return (height * width) / 2.0f;
}


void math_helpers_derivative(float* dest, uint32_t dest_size, float* src, uint32_t src_size)
{
	assert( NULL != dest );
	assert( 0u != dest_size );
	assert( NULL != src );
	assert( 0u != src_size );
	assert( dest_size == ( src_size - 1u ) );

	for( uint32_t idx = 0u; idx < dest_size; idx++ )
	{
		dest[idx] = src[idx + 1u] - src[idx];
	}
}


float math_helpers_riemman_sum_approximation(float* src, uint32_t src_size)
{
	assert( NULL != src );
	assert( 0u != src_size );

	float sum = 0.0f;
	for( uint32_t idx = 0u; idx < src_size; idx++ )
	{
		sum += calculate_area_rectangle(src[idx], 1.0f);
	}
	return sum;
}


float math_helpers_trapezoid_approximation(float* src, uint32_t src_size)
{
	assert( NULL != src );
	assert( 0u != src_size );

    float rect_height = 0.0f;
    float tri_height = 0.0f;
    float rect_area = 0.0f;
    float tri_area = 0.0f;
    float trap_area = 0.0f;
	float sum = 0.0f;

	for( uint32_t idx = 0u; idx < ( src_size - 1u ); idx++ )
	{
		// Increasing
		if( src[idx] < src[idx + 1] )
		{
			rect_height = src[idx];
			tri_height = src[idx + 1] - src[idx];
		}
		// Decreasing
		else
		{
			rect_height = src[idx + 1];
			tri_height = src[idx] - src[idx + 1];
		}

		rect_area = calculate_area_rectangle(rect_height, 1.0f);
		tri_area = calculate_area_triangle(tri_height, 1.0f);
		trap_area = rect_area + tri_area;
		sum += trap_area;
	}

	return sum;
}


float math_helpers_convolution_step(float* func, uint32_t func_size,
	                                float* filt, uint32_t filt_size,
	                                uint32_t a, uint32_t b,
	                                uint32_t x)
{
	assert( NULL != func );
	assert( 0u != func_size );
	assert( NULL != filt );
	assert( 0u != filt_size );
	assert( a < b );

	float sum = 0.0f;
	for( uint32_t idx = a; idx < b; idx++ )
	{
		sum += ( func[idx] * filt[x - idx] );
	}
	return sum;
}

