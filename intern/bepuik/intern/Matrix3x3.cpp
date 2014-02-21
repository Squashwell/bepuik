/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2013 by
 * Harrison Nordby and Ross Nordby
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include <cmath>
#include "Quaternion.hpp"
#include "Vector3.hpp"
#include "Matrix.hpp"
#include "Matrix3x3.hpp"

namespace BEPUmath
{
	/// <summary>
	/// Constructs a new 3 row, 3 column matrix.
	/// </summary>
	/// <param name="m11">Value at row 1, column 1 of the matrix.</param>
	/// <param name="m12">Value at row 1, column 2 of the matrix.</param>
	/// <param name="m13">Value at row 1, column 3 of the matrix.</param>
	/// <param name="m21">Value at row 2, column 1 of the matrix.</param>
	/// <param name="m22">Value at row 2, column 2 of the matrix.</param>
	/// <param name="m23">Value at row 2, column 3 of the matrix.</param>
	/// <param name="m31">Value at row 3, column 1 of the matrix.</param>
	/// <param name="m32">Value at row 3, column 2 of the matrix.</param>
	/// <param name="m33">Value at row 3, column 3 of the matrix.</param>
	Matrix3X3::Matrix3X3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
	{
		M11 = m11;
		M12 = m12;
		M13 = m13;
		M21 = m21;
		M22 = m22;
		M23 = m23;
		M31 = m31;
		M32 = m32;
		M33 = m33;
	}

	Matrix3X3::Matrix3X3() : M11(0), M12(0), M13(0), M21(0), M22(0), M23(0), M31(0), M32(0), M33(0)
	{
	}

	/// <summary>
	/// Adds the two matrices together on a per-element basis.
	/// </summary>
	/// <param name="a">First matrix to add.</param>
	/// <param name="b">Second matrix to add.</param>
	/// <param name="result">Sum of the two matrices.</param>
	void Matrix3X3::Add(Matrix3X3 &a, Matrix3X3 &b, Matrix3X3 &result)
	{
		float m11 = a.M11 + b.M11;
		float m12 = a.M12 + b.M12;
		float m13 = a.M13 + b.M13;

		float m21 = a.M21 + b.M21;
		float m22 = a.M22 + b.M22;
		float m23 = a.M23 + b.M23;

		float m31 = a.M31 + b.M31;
		float m32 = a.M32 + b.M32;
		float m33 = a.M33 + b.M33;

		result.M11 = m11;
		result.M12 = m12;
		result.M13 = m13;

		result.M21 = m21;
		result.M22 = m22;
		result.M23 = m23;

		result.M31 = m31;
		result.M32 = m32;
		result.M33 = m33;
	}

	/// <summary>
	/// Adds the two matrices together on a per-element basis.
	/// </summary>
	/// <param name="a">First matrix to add.</param>
	/// <param name="b">Second matrix to add.</param>
	/// <param name="result">Sum of the two matrices.</param>
	void Matrix3X3::Add(Matrix &a, Matrix3X3 &b, Matrix3X3 &result)
	{
		float m11 = a.M11 + b.M11;
		float m12 = a.M12 + b.M12;
		float m13 = a.M13 + b.M13;

		float m21 = a.M21 + b.M21;
		float m22 = a.M22 + b.M22;
		float m23 = a.M23 + b.M23;

		float m31 = a.M31 + b.M31;
		float m32 = a.M32 + b.M32;
		float m33 = a.M33 + b.M33;

		result.M11 = m11;
		result.M12 = m12;
		result.M13 = m13;

		result.M21 = m21;
		result.M22 = m22;
		result.M23 = m23;

		result.M31 = m31;
		result.M32 = m32;
		result.M33 = m33;
	}

	/// <summary>
	/// Adds the two matrices together on a per-element basis.
	/// </summary>
	/// <param name="a">First matrix to add.</param>
	/// <param name="b">Second matrix to add.</param>
	/// <param name="result">Sum of the two matrices.</param>
	void Matrix3X3::Add(Matrix3X3 &a, Matrix &b, Matrix3X3 &result)
	{
		float m11 = a.M11 + b.M11;
		float m12 = a.M12 + b.M12;
		float m13 = a.M13 + b.M13;

		float m21 = a.M21 + b.M21;
		float m22 = a.M22 + b.M22;
		float m23 = a.M23 + b.M23;

		float m31 = a.M31 + b.M31;
		float m32 = a.M32 + b.M32;
		float m33 = a.M33 + b.M33;

		result.M11 = m11;
		result.M12 = m12;
		result.M13 = m13;

		result.M21 = m21;
		result.M22 = m22;
		result.M23 = m23;

		result.M31 = m31;
		result.M32 = m32;
		result.M33 = m33;
	}

	/// <summary>
	/// Adds the two matrices together on a per-element basis.
	/// </summary>
	/// <param name="a">First matrix to add.</param>
	/// <param name="b">Second matrix to add.</param>
	/// <param name="result">Sum of the two matrices.</param>
	void Matrix3X3::Add(Matrix &a, Matrix &b, Matrix3X3 &result)
	{
		float m11 = a.M11 + b.M11;
		float m12 = a.M12 + b.M12;
		float m13 = a.M13 + b.M13;

		float m21 = a.M21 + b.M21;
		float m22 = a.M22 + b.M22;
		float m23 = a.M23 + b.M23;

		float m31 = a.M31 + b.M31;
		float m32 = a.M32 + b.M32;
		float m33 = a.M33 + b.M33;

		result.M11 = m11;
		result.M12 = m12;
		result.M13 = m13;

		result.M21 = m21;
		result.M22 = m22;
		result.M23 = m23;

		result.M31 = m31;
		result.M32 = m32;
		result.M33 = m33;
	}
	/// <summary>
	/// Creates a skew symmetric matrix M from vector A such that M * B for some other vector B is equivalent to the cross product of A and B.
	/// </summary>
	/// <param name="v">Vector to base the matrix on.</param>
	/// <param name="result">Skew-symmetric matrix result.</param>
	void Matrix3X3::CreateCrossProduct(Vector3 &v, Matrix3X3 &result)
	{
		result.M11 = 0;
		result.M12 = -v.Z;
		result.M13 = v.Y;
		result.M21 = v.Z;
		result.M22 = 0;
		result.M23 = -v.X;
		result.M31 = -v.Y;
		result.M32 = v.X;
		result.M33 = 0;
	}

	/// <summary>
	/// Creates a 3x3 matrix from an XNA 4x4 matrix.
	/// </summary>
	/// <param name="matrix4X4">Matrix to extract a 3x3 matrix from.</param>
	/// <param name="matrix3X3">Upper 3x3 matrix extracted from the XNA matrix.</param>
	void Matrix3X3::CreateFromMatrix(Matrix &matrix4X4, Matrix3X3 &matrix3X3)
	{
		matrix3X3.M11 = matrix4X4.M11;
		matrix3X3.M12 = matrix4X4.M12;
		matrix3X3.M13 = matrix4X4.M13;

		matrix3X3.M21 = matrix4X4.M21;
		matrix3X3.M22 = matrix4X4.M22;
		matrix3X3.M23 = matrix4X4.M23;

		matrix3X3.M31 = matrix4X4.M31;
		matrix3X3.M32 = matrix4X4.M32;
		matrix3X3.M33 = matrix4X4.M33;
	}

	/// <summary>
	/// Constructs a uniform scaling matrix.
	/// </summary>
	/// <param name="scale">Value to use in the diagonal.</param>
	/// <param name="matrix">Scaling matrix.</param>
	void Matrix3X3::CreateScale(float scale, Matrix3X3 &matrix)
	{

		matrix.M11 = scale;
		matrix.M12 = 0;
		matrix.M13 = 0;


		matrix.M21 = 0;
		matrix.M22 = scale;
		matrix.M23 = 0;


		matrix.M31 = 0;
		matrix.M32 = 0;
		matrix.M33 = scale;

	}

	/// <summary>
	/// Constructs a non-uniform scaling matrix.
	/// </summary>
	/// <param name="scale">Values defining the axis scales.</param>
	/// <param name="matrix">Scaling matrix.</param>
	void Matrix3X3::CreateScale(Vector3 &scale, Matrix3X3 &matrix)
	{
		matrix.M11 = scale.X;
		matrix.M12 = 0;
		matrix.M13 = 0;


		matrix.M21 = 0;
		matrix.M22 = scale.Y;
		matrix.M23 = 0;


		matrix.M31 = 0;
		matrix.M32 = 0;
		matrix.M33 = scale.Z;
	}

	/// <summary>
	/// Constructs a non-uniform scaling matrix.
	/// </summary>
	/// <param name="x">Scaling along the x axis.</param>
	/// <param name="y">Scaling along the y axis.</param>
	/// <param name="z">Scaling along the z axis.</param>
	/// <param name="matrix">Scaling matrix.</param>
	void Matrix3X3::CreateScale(float x, float y, float z, Matrix3X3 &matrix)
	{
		matrix.M11 = x;
		matrix.M12 = 0;
		matrix.M13 = 0;


		matrix.M21 = 0;
		matrix.M22 = y;
		matrix.M23 = 0;


		matrix.M31 = 0;
		matrix.M32 = 0;
		matrix.M33 = z;
	}

	/// <summary>
	/// Inverts the given matix.
	/// </summary>
	/// <param name="matrix">Matrix to be inverted.</param>
	/// <param name="result">Inverted matrix.</param>
	void Matrix3X3::Invert(Matrix3X3 &matrix, Matrix3X3 &result)
	{
		float determinantInverse = 1 / matrix.Determinant();
		float m11 = (matrix.M22 * matrix.M33 - matrix.M23 * matrix.M32) * determinantInverse;
		float m12 = (matrix.M13 * matrix.M32 - matrix.M33 * matrix.M12) * determinantInverse;
		float m13 = (matrix.M12 * matrix.M23 - matrix.M22 * matrix.M13) * determinantInverse;

		float m21 = (matrix.M23 * matrix.M31 - matrix.M21 * matrix.M33) * determinantInverse;
		float m22 = (matrix.M11 * matrix.M33 - matrix.M13 * matrix.M31) * determinantInverse;
		float m23 = (matrix.M13 * matrix.M21 - matrix.M11 * matrix.M23) * determinantInverse;

		float m31 = (matrix.M21 * matrix.M32 - matrix.M22 * matrix.M31) * determinantInverse;
		float m32 = (matrix.M12 * matrix.M31 - matrix.M11 * matrix.M32) * determinantInverse;
		float m33 = (matrix.M11 * matrix.M22 - matrix.M12 * matrix.M21) * determinantInverse;

		result.M11 = m11;
		result.M12 = m12;
		result.M13 = m13;

		result.M21 = m21;
		result.M22 = m22;
		result.M23 = m23;

		result.M31 = m31;
		result.M32 = m32;
		result.M33 = m33;
	}

	/// <summary>
	/// Inverts the largest nonsingular submatrix in the matrix, excluding 2x2's that involve M13 or M31, and excluding 1x1's that include nondiagonal elements.
	/// </summary>
	/// <param name="matrix">Matrix to be inverted.</param>
	/// <param name="result">Inverted matrix.</param>
	void Matrix3X3::AdaptiveInvert(Matrix3X3 &matrix, Matrix3X3 &result)
	{
		int submatrix;
		float determinantInverse = 1 / matrix.AdaptiveDeterminant(submatrix);
		float m11, m12, m13, m21, m22, m23, m31, m32, m33;
		switch (submatrix)
		{
		case 0: //Full matrix.
			m11 = (matrix.M22 * matrix.M33 - matrix.M23 * matrix.M32) * determinantInverse;
			m12 = (matrix.M13 * matrix.M32 - matrix.M33 * matrix.M12) * determinantInverse;
			m13 = (matrix.M12 * matrix.M23 - matrix.M22 * matrix.M13) * determinantInverse;

			m21 = (matrix.M23 * matrix.M31 - matrix.M21 * matrix.M33) * determinantInverse;
			m22 = (matrix.M11 * matrix.M33 - matrix.M13 * matrix.M31) * determinantInverse;
			m23 = (matrix.M13 * matrix.M21 - matrix.M11 * matrix.M23) * determinantInverse;

			m31 = (matrix.M21 * matrix.M32 - matrix.M22 * matrix.M31) * determinantInverse;
			m32 = (matrix.M12 * matrix.M31 - matrix.M11 * matrix.M32) * determinantInverse;
			m33 = (matrix.M11 * matrix.M22 - matrix.M12 * matrix.M21) * determinantInverse;
			break;
		case 1: //Upper left matrix, m11, m12, m21, m22.
			m11 = matrix.M22 * determinantInverse;
			m12 = -matrix.M12 * determinantInverse;
			m13 = 0;

			m21 = -matrix.M21 * determinantInverse;
			m22 = matrix.M11 * determinantInverse;
			m23 = 0;

			m31 = 0;
			m32 = 0;
			m33 = 0;
			break;
		case 2: //Lower right matrix, m22, m23, m32, m33.
			m11 = 0;
			m12 = 0;
			m13 = 0;

			m21 = 0;
			m22 = matrix.M33 * determinantInverse;
			m23 = -matrix.M23 * determinantInverse;

			m31 = 0;
			m32 = -matrix.M32 * determinantInverse;
			m33 = matrix.M22 * determinantInverse;
			break;
		case 3: //Corners, m11, m31, m13, m33.
			m11 = matrix.M33 * determinantInverse;
			m12 = 0;
			m13 = -matrix.M13 * determinantInverse;

			m21 = 0;
			m22 = 0;
			m23 = 0;

			m31 = -matrix.M31 * determinantInverse;
			m32 = 0;
			m33 = matrix.M11 * determinantInverse;
			break;
		case 4: //M11
			m11 = 1 / matrix.M11;
			m12 = 0;
			m13 = 0;

			m21 = 0;
			m22 = 0;
			m23 = 0;

			m31 = 0;
			m32 = 0;
			m33 = 0;
			break;
		case 5: //M22
			m11 = 0;
			m12 = 0;
			m13 = 0;

			m21 = 0;
			m22 = 1 / matrix.M22;
			m23 = 0;

			m31 = 0;
			m32 = 0;
			m33 = 0;
			break;
		case 6: //M33
			m11 = 0;
			m12 = 0;
			m13 = 0;

			m21 = 0;
			m22 = 0;
			m23 = 0;

			m31 = 0;
			m32 = 0;
			m33 = 1 / matrix.M33;
			break;
		default: //Completely singular.
			m11 = 0; m12 = 0; m13 = 0; m21 = 0; m22 = 0; m23 = 0; m31 = 0; m32 = 0; m33 = 0;
			break;
		}

		result.M11 = m11;
		result.M12 = m12;
		result.M13 = m13;

		result.M21 = m21;
		result.M22 = m22;
		result.M23 = m23;

		result.M31 = m31;
		result.M32 = m32;
		result.M33 = m33;
	}

	/// <summary>
	/// Multiplies the two matrices.
	/// </summary>
	/// <param name="a">First matrix to multiply.</param>
	/// <param name="b">Second matrix to multiply.</param>
	/// <param name="result">Product of the multiplication.</param>
	void Matrix3X3::Multiply(Matrix3X3 &a, Matrix3X3 &b, Matrix3X3 &result)
	{
		float resultM11 = a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31;
		float resultM12 = a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32;
		float resultM13 = a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33;

		float resultM21 = a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31;
		float resultM22 = a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32;
		float resultM23 = a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33;

		float resultM31 = a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31;
		float resultM32 = a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32;
		float resultM33 = a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33;

		result.M11 = resultM11;
		result.M12 = resultM12;
		result.M13 = resultM13;

		result.M21 = resultM21;
		result.M22 = resultM22;
		result.M23 = resultM23;

		result.M31 = resultM31;
		result.M32 = resultM32;
		result.M33 = resultM33;
	}

	/// <summary>
	/// Multiplies the two matrices.
	/// </summary>
	/// <param name="a">First matrix to multiply.</param>
	/// <param name="b">Second matrix to multiply.</param>
	/// <param name="result">Product of the multiplication.</param>
	void Matrix3X3::Multiply(Matrix3X3 &a, Matrix &b, Matrix3X3 &result)
	{
		float resultM11 = a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31;
		float resultM12 = a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32;
		float resultM13 = a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33;

		float resultM21 = a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31;
		float resultM22 = a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32;
		float resultM23 = a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33;

		float resultM31 = a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31;
		float resultM32 = a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32;
		float resultM33 = a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33;

		result.M11 = resultM11;
		result.M12 = resultM12;
		result.M13 = resultM13;

		result.M21 = resultM21;
		result.M22 = resultM22;
		result.M23 = resultM23;

		result.M31 = resultM31;
		result.M32 = resultM32;
		result.M33 = resultM33;
	}

	/// <summary>
	/// Multiplies the two matrices.
	/// </summary>
	/// <param name="a">First matrix to multiply.</param>
	/// <param name="b">Second matrix to multiply.</param>
	/// <param name="result">Product of the multiplication.</param>
	void Matrix3X3::Multiply(Matrix &a, Matrix3X3 &b, Matrix3X3 &result)
	{
		float resultM11 = a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31;
		float resultM12 = a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32;
		float resultM13 = a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33;

		float resultM21 = a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31;
		float resultM22 = a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32;
		float resultM23 = a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33;

		float resultM31 = a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31;
		float resultM32 = a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32;
		float resultM33 = a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33;

		result.M11 = resultM11;
		result.M12 = resultM12;
		result.M13 = resultM13;

		result.M21 = resultM21;
		result.M22 = resultM22;
		result.M23 = resultM23;

		result.M31 = resultM31;
		result.M32 = resultM32;
		result.M33 = resultM33;
	}


	/// <summary>
	/// Multiplies a transposed matrix with another matrix.
	/// </summary>
	/// <param name="matrix">Matrix to be multiplied.</param>
	/// <param name="transpose">Matrix to be transposed and multiplied.</param>
	/// <param name="result">Product of the multiplication.</param>
	void Matrix3X3::MultiplyTransposed(Matrix3X3 &transpose, Matrix3X3 &matrix, Matrix3X3 &result)
	{
		float resultM11 = transpose.M11 * matrix.M11 + transpose.M21 * matrix.M21 + transpose.M31 * matrix.M31;
		float resultM12 = transpose.M11 * matrix.M12 + transpose.M21 * matrix.M22 + transpose.M31 * matrix.M32;
		float resultM13 = transpose.M11 * matrix.M13 + transpose.M21 * matrix.M23 + transpose.M31 * matrix.M33;

		float resultM21 = transpose.M12 * matrix.M11 + transpose.M22 * matrix.M21 + transpose.M32 * matrix.M31;
		float resultM22 = transpose.M12 * matrix.M12 + transpose.M22 * matrix.M22 + transpose.M32 * matrix.M32;
		float resultM23 = transpose.M12 * matrix.M13 + transpose.M22 * matrix.M23 + transpose.M32 * matrix.M33;

		float resultM31 = transpose.M13 * matrix.M11 + transpose.M23 * matrix.M21 + transpose.M33 * matrix.M31;
		float resultM32 = transpose.M13 * matrix.M12 + transpose.M23 * matrix.M22 + transpose.M33 * matrix.M32;
		float resultM33 = transpose.M13 * matrix.M13 + transpose.M23 * matrix.M23 + transpose.M33 * matrix.M33;

		result.M11 = resultM11;
		result.M12 = resultM12;
		result.M13 = resultM13;

		result.M21 = resultM21;
		result.M22 = resultM22;
		result.M23 = resultM23;

		result.M31 = resultM31;
		result.M32 = resultM32;
		result.M33 = resultM33;
	}

	/// <summary>
	/// Multiplies a matrix with a transposed matrix.
	/// </summary>
	/// <param name="matrix">Matrix to be multiplied.</param>
	/// <param name="transpose">Matrix to be transposed and multiplied.</param>
	/// <param name="result">Product of the multiplication.</param>
	void Matrix3X3::MultiplyByTransposed(Matrix3X3 &matrix, Matrix3X3 &transpose, Matrix3X3 &result)
	{
		float resultM11 = matrix.M11 * transpose.M11 + matrix.M12 * transpose.M12 + matrix.M13 * transpose.M13;
		float resultM12 = matrix.M11 * transpose.M21 + matrix.M12 * transpose.M22 + matrix.M13 * transpose.M23;
		float resultM13 = matrix.M11 * transpose.M31 + matrix.M12 * transpose.M32 + matrix.M13 * transpose.M33;

		float resultM21 = matrix.M21 * transpose.M11 + matrix.M22 * transpose.M12 + matrix.M23 * transpose.M13;
		float resultM22 = matrix.M21 * transpose.M21 + matrix.M22 * transpose.M22 + matrix.M23 * transpose.M23;
		float resultM23 = matrix.M21 * transpose.M31 + matrix.M22 * transpose.M32 + matrix.M23 * transpose.M33;

		float resultM31 = matrix.M31 * transpose.M11 + matrix.M32 * transpose.M12 + matrix.M33 * transpose.M13;
		float resultM32 = matrix.M31 * transpose.M21 + matrix.M32 * transpose.M22 + matrix.M33 * transpose.M23;
		float resultM33 = matrix.M31 * transpose.M31 + matrix.M32 * transpose.M32 + matrix.M33 * transpose.M33;

		result.M11 = resultM11;
		result.M12 = resultM12;
		result.M13 = resultM13;

		result.M21 = resultM21;
		result.M22 = resultM22;
		result.M23 = resultM23;

		result.M31 = resultM31;
		result.M32 = resultM32;
		result.M33 = resultM33;
	}

	/// <summary>
	/// Scales the matrix.
	/// </summary>
	/// <param name="matrix">Matrix to scale.</param>
	/// <param name="scale">Amount to scale.</param>
	/// <param name="result">Scaled matrix.</param>
	void Matrix3X3::Multiply(Matrix3X3 &matrix, float scale, Matrix3X3 &result)
	{
		result.M11 = matrix.M11 * scale;
		result.M12 = matrix.M12 * scale;
		result.M13 = matrix.M13 * scale;

		result.M21 = matrix.M21 * scale;
		result.M22 = matrix.M22 * scale;
		result.M23 = matrix.M23 * scale;

		result.M31 = matrix.M31 * scale;
		result.M32 = matrix.M32 * scale;
		result.M33 = matrix.M33 * scale;
	}

	/// <summary>
	/// Negates every element in the matrix.
	/// </summary>
	/// <param name="matrix">Matrix to negate.</param>
	/// <param name="result">Negated matrix.</param>
	void Matrix3X3::Negate(Matrix3X3 &matrix, Matrix3X3 &result)
	{
		result.M11 = -matrix.M11;
		result.M12 = -matrix.M12;
		result.M13 = -matrix.M13;

		result.M21 = -matrix.M21;
		result.M22 = -matrix.M22;
		result.M23 = -matrix.M23;

		result.M31 = -matrix.M31;
		result.M32 = -matrix.M32;
		result.M33 = -matrix.M33;
	}

	/// <summary>
	/// Subtracts the two matrices from each other on a per-element basis.
	/// </summary>
	/// <param name="a">First matrix to subtract.</param>
	/// <param name="b">Second matrix to subtract.</param>
	/// <param name="result">Difference of the two matrices.</param>
	void Matrix3X3::Subtract(Matrix3X3 &a, Matrix3X3 &b, Matrix3X3 &result)
	{
		float m11 = a.M11 - b.M11;
		float m12 = a.M12 - b.M12;
		float m13 = a.M13 - b.M13;

		float m21 = a.M21 - b.M21;
		float m22 = a.M22 - b.M22;
		float m23 = a.M23 - b.M23;

		float m31 = a.M31 - b.M31;
		float m32 = a.M32 - b.M32;
		float m33 = a.M33 - b.M33;

		result.M11 = m11;
		result.M12 = m12;
		result.M13 = m13;

		result.M21 = m21;
		result.M22 = m22;
		result.M23 = m23;

		result.M31 = m31;
		result.M32 = m32;
		result.M33 = m33;
	}

	/// <summary>
	/// Creates a 4x4 matrix from a 3x3 matrix.
	/// </summary>
	/// <param name="a">3x3 matrix.</param>
	/// <param name="b">Created 4x4 matrix.</param>
	void Matrix3X3::ToMatrix4X4(Matrix3X3 &a, Matrix &b)
	{
		b.M11 = a.M11;
		b.M12 = a.M12;
		b.M13 = a.M13;

		b.M21 = a.M21;
		b.M22 = a.M22;
		b.M23 = a.M23;

		b.M31 = a.M31;
		b.M32 = a.M32;
		b.M33 = a.M33;

		b.M44 = 1;
		b.M14 = 0;
		b.M24 = 0;
		b.M34 = 0;
		b.M41 = 0;
		b.M42 = 0;
		b.M43 = 0;
	}

	/// <summary>
	/// Transforms the vector by the matrix.
	/// </summary>
	/// <param name="v">Vector3 to transform.</param>
	/// <param name="matrix">Matrix to use as the transformation.</param>
	/// <param name="result">Product of the transformation.</param>
	void Matrix3X3::Transform(Vector3 &v, Matrix3X3 &matrix, Vector3 &result)
	{
		float vX = v.X;
		float vY = v.Y;
		float vZ = v.Z;

		result.X = vX * matrix.M11 + vY * matrix.M21 + vZ * matrix.M31;
		result.Y = vX * matrix.M12 + vY * matrix.M22 + vZ * matrix.M32;
		result.Z = vX * matrix.M13 + vY * matrix.M23 + vZ * matrix.M33;
	}

	/// <summary>
	/// Transforms the vector by the matrix.
	/// </summary>
	/// <param name="v">Vector3 to transform.</param>
	/// <param name="matrix">Matrix to use as the transformation.</param>
	/// <param name="result">Product of the transformation.</param>
	void Matrix3X3::Transform(Vector3 &v, Matrix &matrix, Vector3 &result)
	{
		float vX = v.X;
		float vY = v.Y;
		float vZ = v.Z;

		result.X = vX * matrix.M11 + vY * matrix.M21 + vZ * matrix.M31;
		result.Y = vX * matrix.M12 + vY * matrix.M22 + vZ * matrix.M32;
		result.Z = vX * matrix.M13 + vY * matrix.M23 + vZ * matrix.M33;
	}

	/// <summary>
	/// Transforms the vector by the matrix's transpose.
	/// </summary>
	/// <param name="v">Vector3 to transform.</param>
	/// <param name="matrix">Matrix to use as the transformation transpose.</param>
	/// <param name="result">Product of the transformation.</param>
	void Matrix3X3::TransformTranspose(Vector3 &v, Matrix3X3 &matrix, Vector3 &result)
	{
		float vX = v.X;
		float vY = v.Y;
		float vZ = v.Z;

		result.X = vX * matrix.M11 + vY * matrix.M12 + vZ * matrix.M13;
		result.Y = vX * matrix.M21 + vY * matrix.M22 + vZ * matrix.M23;
		result.Z = vX * matrix.M31 + vY * matrix.M32 + vZ * matrix.M33;
	}

	/// <summary>
	/// Transforms the vector by the matrix's transpose.
	/// </summary>
	/// <param name="v">Vector3 to transform.</param>
	/// <param name="matrix">Matrix to use as the transformation transpose.</param>
	/// <param name="result">Product of the transformation.</param>
	void Matrix3X3::TransformTranspose(Vector3 &v, Matrix &matrix, Vector3 &result)
	{
		float vX = v.X;
		float vY = v.Y;
		float vZ = v.Z;

		result.X = vX * matrix.M11 + vY * matrix.M12 + vZ * matrix.M13;
		result.Y = vX * matrix.M21 + vY * matrix.M22 + vZ * matrix.M23;
		result.Z = vX * matrix.M31 + vY * matrix.M32 + vZ * matrix.M33;
	}

	/// <summary>
	/// Computes the transposed matrix of a matrix.
	/// </summary>
	/// <param name="matrix">Matrix to transpose.</param>
	/// <param name="result">Transposed matrix.</param>
	void Matrix3X3::Transpose(Matrix3X3 &matrix, Matrix3X3 &result)
	{
		float m21 = matrix.M12;
		float m31 = matrix.M13;
		float m12 = matrix.M21;
		float m32 = matrix.M23;
		float m13 = matrix.M31;
		float m23 = matrix.M32;

		result.M11 = matrix.M11;
		result.M12 = m12;
		result.M13 = m13;
		result.M21 = m21;
		result.M22 = matrix.M22;
		result.M23 = m23;
		result.M31 = m31;
		result.M32 = m32;
		result.M33 = matrix.M33;
	}

	/// <summary>
	/// Computes the transposed matrix of a matrix.
	/// </summary>
	/// <param name="matrix">Matrix to transpose.</param>
	/// <param name="result">Transposed matrix.</param>
	void Matrix3X3::Transpose(Matrix &matrix, Matrix3X3 &result)
	{
		float m21 = matrix.M12;
		float m31 = matrix.M13;
		float m12 = matrix.M21;
		float m32 = matrix.M23;
		float m13 = matrix.M31;
		float m23 = matrix.M32;

		result.M11 = matrix.M11;
		result.M12 = m12;
		result.M13 = m13;
		result.M21 = m21;
		result.M22 = matrix.M22;
		result.M23 = m23;
		result.M31 = m31;
		result.M32 = m32;
		result.M33 = matrix.M33;
	}

	/// <summary>
	/// Calculates the determinant of the matrix.
	/// </summary>
	/// <returns>The matrix's determinant.</returns>
	float Matrix3X3::Determinant()
	{
		return M11 * M22 * M33 + M12 * M23 * M31 + M13 * M21 * M32 -
			M31 * M22 * M13 - M32 * M23 * M11 - M33 * M21 * M12;
	}

	/// <summary>
	/// Calculates the determinant of largest nonsingular submatrix, excluding 2x2's that involve M13 or M31, and excluding all 1x1's that involve nondiagonal elements.
	/// </summary>
	/// <param name="subMatrixCode">Represents the submatrix that was used to compute the determinant.
	/// 0 is the full 3x3.  1 is the upper left 2x2.  2 is the lower right 2x2.  3 is the four corners.
	/// 4 is M11.  5 is M22.  6 is M33.</param>
	/// <returns>The matrix's determinant.</returns>
	float Matrix3X3::AdaptiveDeterminant(int &subMatrixCode)
	{
		//Try the full matrix first.
		float determinant = M11 * M22 * M33 + M12 * M23 * M31 + M13 * M21 * M32 -
			M31 * M22 * M13 - M32 * M23 * M11 - M33 * M21 * M12;
		if (determinant != 0) //This could be a little numerically flimsy.  Fortunately, the way this method is used, that doesn't matter!
		{
			subMatrixCode = 0;
			return determinant;
		}
		//Try m11, m12, m21, m22.
		determinant = M11 * M22 - M12 * M21;
		if (determinant != 0)
		{
			subMatrixCode = 1;
			return determinant;
		}
		//Try m22, m23, m32, m33.
		determinant = M22 * M33 - M23 * M32;
		if (determinant != 0)
		{
			subMatrixCode = 2;
			return determinant;
		}
		//Try m11, m13, m31, m33.
		determinant = M11 * M33 - M13 * M12;
		if (determinant != 0)
		{
			subMatrixCode = 3;
			return determinant;
		}
		//Try m11.
		if (M11 != 0)
		{
			subMatrixCode = 4;
			return M11;
		}
		//Try m22.
		if (M22 != 0)
		{
			subMatrixCode = 5;
			return M22;
		}
		//Try m33.
		if (M33 != 0)
		{
			subMatrixCode = 6;
			return M33;
		}
		//It's completely singular!
		subMatrixCode = -1;
		return 0;
	}

	/// <summary>
	/// Constructs a quaternion from a 3x3 rotation matrix.
	/// </summary>
	/// <param name="r">Rotation matrix to create the quaternion from.</param>
	/// <param name="q">Quaternion based on the rotation matrix.</param>
	void Matrix3X3::CreateQuaternion(Matrix3X3 &r, Quaternion &q)
	{
		float trace = r.M11 + r.M22 + r.M33;

		if (trace >= 0)
		{
			float S = (float)sqrt(trace + 1.0) * 2; // S=4*qw
			float inverseS = 1 / S;
			q.W = 0.25f * S;
			q.X = (r.M23 - r.M32) * inverseS;
			q.Y = (r.M31 - r.M13) * inverseS;
			q.Z = (r.M12 - r.M21) * inverseS;
		}
		else if ((r.M11 > r.M22) & (r.M11 > r.M33))
		{
			float S = (float)sqrt(1.0 + r.M11 - r.M22 - r.M33) * 2; // S=4*qx
			float inverseS = 1 / S;
			q.W = (r.M23 - r.M32) * inverseS;
			q.X = 0.25f * S;
			q.Y = (r.M21 + r.M12) * inverseS;
			q.Z = (r.M31 + r.M13) * inverseS;
		}
		else if (r.M22 > r.M33)
		{
			float S = (float)sqrt(1.0 + r.M22 - r.M11 - r.M33) * 2; // S=4*qy
			float inverseS = 1 / S;
			q.W = (r.M31 - r.M13) * inverseS;
			q.X = (r.M21 + r.M12) * inverseS;
			q.Y = 0.25f * S;
			q.Z = (r.M32 + r.M23) * inverseS;
		}
		else
		{
			float S = (float)sqrt(1.0 + r.M33 - r.M11 - r.M22) * 2; // S=4*qz
			float inverseS = 1 / S;
			q.W = (r.M12 - r.M21) * inverseS;
			q.X = (r.M31 + r.M13) * inverseS;
			q.Y = (r.M32 + r.M23) * inverseS;
			q.Z = 0.25f * S;
		}
	}

	/// <summary>
	/// Creates a 3x3 matrix representing the orientation stored in the quaternion.
	/// </summary>
	/// <param name="quaternion">Quaternion to use to create a matrix.</param>
	/// <param name="result">Matrix representing the quaternion's orientation.</param>
	void Matrix3X3::CreateFromQuaternion(Quaternion &quaternion, Matrix3X3 &result)
	{

		float XX = 2 * quaternion.X * quaternion.X;
		float YY = 2 * quaternion.Y * quaternion.Y;
		float ZZ = 2 * quaternion.Z * quaternion.Z;
		float XY = 2 * quaternion.X * quaternion.Y;
		float XZ = 2 * quaternion.X * quaternion.Z;
		float XW = 2 * quaternion.X * quaternion.W;
		float YZ = 2 * quaternion.Y * quaternion.Z;
		float YW = 2 * quaternion.Y * quaternion.W;
		float ZW = 2 * quaternion.Z * quaternion.W;

		result.M11 = 1 - YY - ZZ;
		result.M21 = XY - ZW;
		result.M31 = XZ + YW;

		result.M12 = XY + ZW;
		result.M22 = 1 - XX - ZZ;
		result.M32 = YZ - XW;

		result.M13 = XZ - YW;
		result.M23 = YZ + XW;
		result.M33 = 1 - XX - YY;
	}

	void Matrix3X3::CreateFromMatrix4X4(Matrix &m, Matrix3X3 &result)
	{
		result.M11 = m.M11;
		result.M12 = m.M12;
		result.M13 = m.M13;

		result.M21 = m.M21;
		result.M22 = m.M22;
		result.M23 = m.M23;

		result.M31 = m.M31;
		result.M32 = m.M32;
		result.M33 = m.M33;
	}

	/// <summary>
	/// Computes the outer product of the given vectors.
	/// </summary>
	/// <param name="a">First vector.</param>
	/// <param name="b">Second vector.</param>
	/// <param name="result">Outer product result.</param>
	void Matrix3X3::CreateOuterProduct(Vector3 &a, Vector3 &b, Matrix3X3 &result)
	{
		result.M11 = a.X * b.X;
		result.M12 = a.X * b.Y;
		result.M13 = a.X * b.Z;

		result.M21 = a.Y * b.X;
		result.M22 = a.Y * b.Y;
		result.M23 = a.Y * b.Z;

		result.M31 = a.Z * b.X;
		result.M32 = a.Z * b.Y;
		result.M33 = a.Z * b.Z;
	}

	/// <summary>
	/// Creates a matrix representing a rotation of a given angle around a given axis.
	/// </summary>
	/// <param name="axis">Axis around which to rotate.</param>
	/// <param name="angle">Amount to rotate.</param>
	/// <param name="result">Matrix representing the rotation.</param>
	void Matrix3X3::CreateFromAxisAngle(Vector3 &axis, float angle, Matrix3X3 &result)
	{
		float xx = axis.X * axis.X;
		float yy = axis.Y * axis.Y;
		float zz = axis.Z * axis.Z;
		float xy = axis.X * axis.Y;
		float xz = axis.X * axis.Z;
		float yz = axis.Y * axis.Z;

		float sinAngle = (float)sin(angle);
		float oneMinusCosAngle = 1 - (float)cos(angle);

		result.M11 = 1 + oneMinusCosAngle * (xx - 1);
		result.M21 = -axis.Z * sinAngle + oneMinusCosAngle * xy;
		result.M31 = axis.Y * sinAngle + oneMinusCosAngle * xz;

		result.M12 = axis.Z * sinAngle + oneMinusCosAngle * xy;
		result.M22 = 1 + oneMinusCosAngle * (yy - 1);
		result.M32 = -axis.X * sinAngle + oneMinusCosAngle * yz;

		result.M13 = -axis.Y * sinAngle + oneMinusCosAngle * xz;
		result.M23 = axis.X * sinAngle + oneMinusCosAngle * yz;
		result.M33 = 1 + oneMinusCosAngle * (zz - 1);
	}

	/// <summary>
	/// Creates an identity matrix.
	/// </summary>
	/// <param name="result">Reference to store the identity matrix in.</param>
	void Matrix3X3::GetIdentity(Matrix3X3 &result)
	{
		result.M11 = 1;
		result.M12 = 0;
		result.M13 = 0;
		result.M21 = 0;
		result.M22 = 1;
		result.M23 = 0;
		result.M31 = 0;
		result.M32 = 0;
		result.M33 = 1;
	}

}
