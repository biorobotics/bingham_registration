/*
 * File Header for conversions.h:
 * 		This file contains functions for converting common data types
 */

#ifndef CONVERSIONS
#define CONVERSIONS

#include "type_defs.h"

/* eul2rotm:
 *		Input: euler angle in array (for the use of cos, sin function), in ZYX order
 		Output: quaternion after conversion 
 */
Matrix4ld eul2rotm(const Array3ld& eul);

/* eul2quat:
 *		Input: euler angle in vector
 		Output: quaternion after conversion 
 */
Quaternionld eul2quat(const Vector3ld& eul);

/* reg_params_to_transformation_matrix:
 *		Input: registration parameters in array
 		Output: transformation matrix after conversion 
 */
Matrix4ld reg_params_to_transformation_matrix(const ArrayXld& params);



#endif