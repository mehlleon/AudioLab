float dot_product_intrinsic(float * vec1, float *  vec2, int n)
{
	float32x4_t vec1_q, vec2_q; /* declare NEON vectors with 4 floats */
	float32x4_t sum_q = {0.0, 0.0, 0.0, 0.0}; /* vector collecting the sum */
	float32x2_t tmp[2]; /* temporary space for intermediate sums */
	float result;
	for( int i=0; i<( n & ~3); i+=4 )
	{
		vec1_q=vld1q_f32(&vec1[i]); /* loads 4 elements from vec1 into NEON vec1_q vector */
		vec2_q=vld1q_f32(&vec2[i]); /* loads 4 elements from vec2 into NEON vec2_q vector */
		sum_q = vmlaq_f32(sum_q, vec1_q, vec2_q ); /* multiply vec1_q and vec2_q element wise and accumulate result in sum_q */
	}

	tmp[0] = vget_high_f32(sum_q); /* get the high half of sum_q */
	tmp[1] = vget_low_f32 (sum_q); /* get the low half of sum_q */ // -> resulting in two 2-element vectors tmp[0] and tmp[1]
	tmp[0] = vpadd_f32(tmp[0], tmp[1]); /* pairwise add elements from tmp[0] and tmp[1] */ // -> resulting in a 2-element vector tmp[0] containing the sum
	tmp[0] = vpadd_f32(tmp[0], tmp[0]); /* pairwise add elements from tmp[0] */ // -> resulting in complete sum in tmp[0] element 1 as well as in tmp[0] element
	result = vget_lane_f32(tmp[0], 0); /* extract the result from the first lane in the 2-element vector */
	return result;
}