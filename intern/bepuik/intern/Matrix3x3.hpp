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

#pragma once
#include "Matrix.hpp"

namespace BEPUmath
{
	struct Matrix3X3
	{
		/// <summary>
		/// Value at row 1, column 1 of the matrix.
		/// </summary>
		float M11;

		/// <summary>
		/// Value at row 1, column 2 of the matrix.
		/// </summary>
		float M12;

		/// <summary>
		/// Value at row 1, column 3 of the matrix.
		/// </summary>
		float M13;

		/// <summary>
		/// Value at row 2, column 1 of the matrix.
		/// </summary>
		float M21;

		/// <summary>
		/// Value at row 2, column 2 of the matrix.
		/// </summary>
		float M22;

		/// <summary>
		/// Value at row 2, column 3 of the matrix.
		/// </summary>
		float M23;

		/// <summary>
		/// Value at row 3, column 1 of the matrix.
		/// </summary>
		float M31;

		/// <summary>
		/// Value at row 3, column 2 of the matrix.
		/// </summary>
		float M32;

		/// <summary>
		/// Value at row 3, column 3 of the matrix.
		/// </summary>
		float M33;

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
		Matrix3X3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33);


		Matrix3X3();

		/// <summary>
		/// Adds the two matrices together on a per-element basis.
		/// </summary>
		/// <param name="a">First matrix to add.</param>
		/// <param name="b">Second matrix to add.</param>
		/// <param name="result">Sum of the two matrices.</param>
		static void Add(Matrix3X3 &a, Matrix3X3 &b, Matrix3X3 &result);

		/// <summary>
		/// Adds the two matrices together on a per-element basis.
		/// </summary>
		/// <param name="a">First matrix to add.</param>
		/// <param name="b">Second matrix to add.</param>
		/// <param name="result">Sum of the two matrices.</param>
		static void Add(Matrix &a, Matrix3X3 &b, Matrix3X3 &result);

		/// <summary>
		/// Adds the two matrices together on a per-element basis.
		/// </summary>
		/// <param name="a">First matrix to add.</param>
		/// <param name="b">Second matrix to add.</param>
		/// <param name="result">Sum of the two matrices.</param>
		static void Add(Matrix3X3 &a, Matrix &b, Matrix3X3 &result);

		/// <summary>
		/// Adds the two matrices together on a per-element basis.
		/// </summary>
		/// <param name="a">First matrix to add.</param>
		/// <param name="b">Second matrix to add.</param>
		/// <param name="result">Sum of the two matrices.</param>
		static void Add(Matrix &a, Matrix &b, Matrix3X3 &result);

		/// <summary>
		/// Creates a skew symmetric matrix M from vector A such that M * B for some other vector B is equivalent to the cross product of A and B.
		/// </summary>
		/// <param name="v">Vector to base the matrix on.</param>
		/// <param name="result">Skew-symmetric matrix result.</param>
		static void CreateCrossProduct(Vector3 &v, Matrix3X3 &result);

		/// <summary>
		/// Creates a 3x3 matrix from an XNA 4x4 matrix.
		/// </summary>
		/// <param name="matrix4X4">Matrix to extract a 3x3 matrix from.</param>
		/// <param name="matrix3X3">Upper 3x3 matrix extracted from the XNA matrix.</param>
		static void CreateFromMatrix(Matrix &matrix4X4, Matrix3X3 &matrix3X3);

		/// <summary>
		/// Constructs a uniform scaling matrix.
		/// </summary>
		/// <param name="scale">Value to use in the diagonal.</param>
		/// <param name="matrix">Scaling matrix.</param>
		static void CreateScale(float scale, Matrix3X3 &matrix);

		/// <summary>
		/// Constructs a non-uniform scaling matrix.
		/// </summary>
		/// <param name="scale">Values defining the axis scales.</param>
		/// <param name="matrix">Scaling matrix.</param>
		static void CreateScale(Vector3 &scale, Matrix3X3 &matrix);

		/// <summary>
		/// Constructs a non-uniform scaling matrix.
		/// </summary>
		/// <param name="x">Scaling along the x axis.</param>
		/// <param name="y">Scaling along the y axis.</param>
		/// <param name="z">Scaling along the z axis.</param>
		/// <param name="matrix">Scaling matrix.</param>
		static void CreateScale(float x, float y, float z, Matrix3X3 &matrix);

		/// <summary>
		/// Inverts the given matix.
		/// </summary>
		/// <param name="matrix">Matrix to be inverted.</param>
		/// <param name="result">Inverted matrix.</param>
		static void Invert(Matrix3X3 &matrix, Matrix3X3 &result);

		/// <summary>
		/// Inverts the largest nonsingular submatrix in the matrix, excluding 2x2's that involve M13 or M31, and excluding 1x1's that include nondiagonal elements.
		/// </summary>
		/// <param name="matrix">Matrix to be inverted.</param>
		/// <param name="result">Inverted matrix.</param>
		static void AdaptiveInvert(Matrix3X3 &matrix, Matrix3X3 &result);

		/// <summary>
		/// Multiplies the two matrices.
		/// </summary>
		/// <param name="a">First matrix to multiply.</param>
		/// <param name="b">Second matrix to multiply.</param>
		/// <param name="result">Product of the multiplication.</param>
		static void Multiply(Matrix3X3 &a, Matrix3X3 &b, Matrix3X3 &result);

		/// <summary>
		/// Multiplies the two matrices.
		/// </summary>
		/// <param name="a">First matrix to multiply.</param>
		/// <param name="b">Second matrix to multiply.</param>
		/// <param name="result">Product of the multiplication.</param>
		static void Multiply(Matrix3X3 &a, Matrix &b, Matrix3X3 &result);

		/// <summary>
		/// Multiplies the two matrices.
		/// </summary>
		/// <param name="a">First matrix to multiply.</param>
		/// <param name="b">Second matrix to multiply.</param>
		/// <param name="result">Product of the multiplication.</param>
		static void Multiply(Matrix &a, Matrix3X3 &b, Matrix3X3 &result);


		/// <summary>
		/// Multiplies a transposed matrix with another matrix.
		/// </summary>
		/// <param name="matrix">Matrix to be multiplied.</param>
		/// <param name="transpose">Matrix to be transposed and multiplied.</param>
		/// <param name="result">Product of the multiplication.</param>
		static void MultiplyTransposed(Matrix3X3 &transpose, Matrix3X3 &matrix, Matrix3X3 &result);

		/// <summary>
		/// Multiplies a matrix with a transposed matrix.
		/// </summary>
		/// <param name="matrix">Matrix to be multiplied.</param>
		/// <param name="transpose">Matrix to be transposed and multiplied.</param>
		/// <param name="result">Product of the multiplication.</param>
		static void MultiplyByTransposed(Matrix3X3 &matrix, Matrix3X3 &transpose, Matrix3X3 &result);

		/// <summary>
		/// Scales the matrix.
		/// </summary>
		/// <param name="matrix">Matrix to scale.</param>
		/// <param name="scale">Amount to scale.</param>
		/// <param name="result">Scaled matrix.</param>
		static void Multiply(Matrix3X3 &matrix, float scale, Matrix3X3 &result);

		/// <summary>
		/// Negates every element in the matrix.
		/// </summary>
		/// <param name="matrix">Matrix to negate.</param>
		/// <param name="result">Negated matrix.</param>
		static void Negate(Matrix3X3 &matrix, Matrix3X3 &result);

		/// <summary>
		/// Subtracts the two matrices from each other on a per-element basis.
		/// </summary>
		/// <param name="a">First matrix to subtract.</param>
		/// <param name="b">Second matrix to subtract.</param>
		/// <param name="result">Difference of the two matrices.</param>
		static void Subtract(Matrix3X3 &a, Matrix3X3 &b, Matrix3X3 &result);

		/// <summary>
		/// Creates a 4x4 matrix from a 3x3 matrix.
		/// </summary>
		/// <param name="a">3x3 matrix.</param>
		/// <param name="b">Created 4x4 matrix.</param>
		static void ToMatrix4X4(Matrix3X3 &a, Matrix &b);

		/// <summary>
		/// Transforms the vector by the matrix.
		/// </summary>
		/// <param name="v">Vector3 to transform.</param>
		/// <param name="matrix">Matrix to use as the transformation.</param>
		/// <param name="result">Product of the transformation.</param>
		static void Transform(Vector3 &v, Matrix3X3 &matrix, Vector3 &result);

		/// <summary>
		/// Transforms the vector by the matrix.
		/// </summary>
		/// <param name="v">Vector3 to transform.</param>
		/// <param name="matrix">Matrix to use as the transformation.</param>
		/// <param name="result">Product of the transformation.</param>
		static void Transform(Vector3 &v, Matrix &matrix, Vector3 &result);

		/// <summary>
		/// Transforms the vector by the matrix's transpose.
		/// </summary>
		/// <param name="v">Vector3 to transform.</param>
		/// <param name="matrix">Matrix to use as the transformation transpose.</param>
		/// <param name="result">Product of the transformation.</param>
		static void TransformTranspose(Vector3 &v, Matrix3X3 &matrix, Vector3 &result);

		/// <summary>
		/// Transforms the vector by the matrix's transpose.
		/// </summary>
		/// <param name="v">Vector3 to transform.</param>
		/// <param name="matrix">Matrix to use as the transformation transpose.</param>
		/// <param name="result">Product of the transformation.</param>
		static void TransformTranspose(Vector3 &v, Matrix &matrix, Vector3 &result);

		/// <summary>
		/// Computes the transposed matrix of a matrix.
		/// </summary>
		/// <param name="matrix">Matrix to transpose.</param>
		/// <param name="result">Transposed matrix.</param>
		static void Transpose(Matrix3X3 &matrix, Matrix3X3 &result);

		/// <summary>
		/// Computes the transposed matrix of a matrix.
		/// </summary>
		/// <param name="matrix">Matrix to transpose.</param>
		/// <param name="result">Transposed matrix.</param>
		static void Transpose(Matrix &matrix, Matrix3X3 &result);

		/// <summary>
		/// Calculates the determinant of the matrix.
		/// </summary>
		/// <returns>The matrix's determinant.</returns>
		float Determinant();

		/// <summary>
		/// Calculates the determinant of largest nonsingular submatrix, excluding 2x2's that involve M13 or M31, and excluding all 1x1's that involve nondiagonal elements.
		/// </summary>
		/// <param name="subMatrixCode">Represents the submatrix that was used to compute the determinant.
		/// 0 is the full 3x3.  1 is the upper left 2x2.  2 is the lower right 2x2.  3 is the four corners.
		/// 4 is M11.  5 is M22.  6 is M33.</param>
		/// <returns>The matrix's determinant.</returns>
		float AdaptiveDeterminant(int &subMatrixCode);

		/// <summary>
		/// Constructs a quaternion from a 3x3 rotation matrix.
		/// </summary>
		/// <param name="r">Rotation matrix to create the quaternion from.</param>
		/// <param name="q">Quaternion based on the rotation matrix.</param>
		static void CreateQuaternion(Matrix3X3 &r, Quaternion &q);

		/// <summary>
		/// Creates a 3x3 matrix representing the orientation stored in the quaternion.
		/// </summary>
		/// <param name="quaternion">Quaternion to use to create a matrix.</param>
		/// <param name="result">Matrix representing the quaternion's orientation.</param>
		static void CreateFromQuaternion(Quaternion &quaternion, Matrix3X3 &result);

		/// <summary>
		/// Computes the outer product of the given vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="result">Outer product result.</param>
		static void CreateOuterProduct(Vector3 &a, Vector3 &b, Matrix3X3 &result);

		/// <summary>
		/// Creates a matrix representing a rotation of a given angle around a given axis.
		/// </summary>
		/// <param name="axis">Axis around which to rotate.</param>
		/// <param name="angle">Amount to rotate.</param>
		/// <param name="result">Matrix representing the rotation.</param>
		static void CreateFromAxisAngle(Vector3 &axis, float angle, Matrix3X3 &result);

		static void CreateFromMatrix4X4(Matrix &matrix, Matrix3X3 &result);
		/// <summary>
		/// Creates an identity matrix.
		/// </summary>
		/// <param name="result">Reference to store the identity matrix in.</param>
		static void GetIdentity(Matrix3X3 &result);
	};
}
