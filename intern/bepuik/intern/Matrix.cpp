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

namespace BEPUmath
{

	/// <summary>
	/// Constructs a new 4 row, 4 column matrix.
	/// </summary>
	/// <param name="m11">Value at row 1, column 1 of the matrix.</param>
	/// <param name="m12">Value at row 1, column 2 of the matrix.</param>
	/// <param name="m13">Value at row 1, column 3 of the matrix.</param>
	/// <param name="m14">Value at row 1, column 4 of the matrix.</param>
	/// <param name="m21">Value at row 2, column 1 of the matrix.</param>
	/// <param name="m22">Value at row 2, column 2 of the matrix.</param>
	/// <param name="m23">Value at row 2, column 3 of the matrix.</param>
	/// <param name="m24">Value at row 2, column 4 of the matrix.</param>
	/// <param name="m31">Value at row 3, column 1 of the matrix.</param>
	/// <param name="m32">Value at row 3, column 2 of the matrix.</param>
	/// <param name="m33">Value at row 3, column 3 of the matrix.</param>
	/// <param name="m34">Value at row 3, column 4 of the matrix.</param>
	/// <param name="m41">Value at row 4, column 1 of the matrix.</param>
	/// <param name="m42">Value at row 4, column 2 of the matrix.</param>
	/// <param name="m43">Value at row 4, column 3 of the matrix.</param>
	/// <param name="m44">Value at row 4, column 4 of the matrix.</param>
	Matrix::Matrix(float m11, float m12, float m13, float m14,
		float m21, float m22, float m23, float m24,
		float m31, float m32, float m33, float m34,
		float m41, float m42, float m43, float m44)
	{
		M11 = m11;
		M12 = m12;
		M13 = m13;
		M14 = m14;

		M21 = m21;
		M22 = m22;
		M23 = m23;
		M24 = m24;

		M31 = m31;
		M32 = m32;
		M33 = m33;
		M34 = m34;

		M41 = m41;
		M42 = m42;
		M43 = m43;
		M44 = m44;
	}

	Matrix::Matrix() : M11(0), M12(0), M13(0), M14(0),
					   M21(0), M22(0), M23(0), M24(0),
					   M31(0), M32(0), M33(0), M34(0),
					   M41(0), M42(0), M43(0), M44(0)
	{
	}


	/// <summary>
	/// Computes the determinant of the matrix.
	/// </summary>
	/// <returns></returns>
	float Matrix::Determinant()
	{
		//Compute the re-used 2x2 determinants.
		float det1 = M33 * M44 - M34 * M43;
		float det2 = M32 * M44 - M34 * M42;
		float det3 = M32 * M43 - M33 * M42;
		float det4 = M31 * M44 - M34 * M41;
		float det5 = M31 * M43 - M33 * M41;
		float det6 = M31 * M42 - M32 * M41;
		return
			(M11 * ((M22 * det1 - M23 * det2) + M24 * det3)) -
			(M12 * ((M21 * det1 - M23 * det4) + M24 * det5)) +
			(M13 * ((M21 * det2 - M22 * det4) + M24 * det6)) -
			(M14 * ((M21 * det3 - M22 * det5) + M23 * det6));
	}


	/// <summary>
	/// Creates a matrix representing the given axis and angle rotation.
	/// </summary>
	/// <param name="axis">Axis around which to rotate.</param>
	/// <param name="angle">Angle to rotate around the axis.</param>
	/// <param name="result">Matrix created from the axis and angle.</param>
	void Matrix::CreateFromAxisAngle(Vector3 &axis, float angle, Matrix &result)
	{
		float xx = axis.X * axis.X;
		float yy = axis.Y * axis.Y;
		float zz = axis.Z * axis.Z;
		float xy = axis.X * axis.Y;
		float xz = axis.X * axis.Z;
		float yz = axis.Y * axis.Z;

		float sinAngle = sin(angle);
		float oneMinusCosAngle = 1 - cos(angle);

		result.M11 = 1 + oneMinusCosAngle * (xx - 1);
		result.M21 = -axis.Z * sinAngle + oneMinusCosAngle * xy;
		result.M31 = axis.Y * sinAngle + oneMinusCosAngle * xz;
		result.M41 = 0;

		result.M12 = axis.Z * sinAngle + oneMinusCosAngle * xy;
		result.M22 = 1 + oneMinusCosAngle * (yy - 1);
		result.M32 = -axis.X * sinAngle + oneMinusCosAngle * yz;
		result.M42 = 0;

		result.M13 = -axis.Y * sinAngle + oneMinusCosAngle * xz;
		result.M23 = axis.X * sinAngle + oneMinusCosAngle * yz;
		result.M33 = 1 + oneMinusCosAngle * (zz - 1);
		result.M43 = 0;

		result.M14 = 0;
		result.M24 = 0;
		result.M34 = 0;
		result.M44 = 1;
	}

	/// <summary>
	/// Creates a rotation matrix from a quaternion.
	/// </summary>
	/// <param name="quaternion">Quaternion to convert.</param>
	/// <param name="result">Rotation matrix created from the quaternion.</param>
	void Matrix::CreateFromQuaternion(Quaternion &quaternion, Matrix &result)
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
		result.M41 = 0;

		result.M12 = XY + ZW;
		result.M22 = 1 - XX - ZZ;
		result.M32 = YZ - XW;
		result.M42 = 0;

		result.M13 = XZ - YW;
		result.M23 = YZ + XW;
		result.M33 = 1 - XX - YY;
		result.M43 = 0;

		result.M14 = 0;
		result.M24 = 0;
		result.M34 = 0;
		result.M44 = 1;
	}

	/// <summary>
	/// Multiplies two matrices together.
	/// </summary>
	/// <param name="a">First matrix to multiply.</param>
	/// <param name="b">Second matrix to multiply.</param>
	/// <param name="result">Combined transformation.</param>
	void Matrix::Multiply(Matrix &a, Matrix &b, Matrix &result)
	{
		float resultM11 = a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31 + a.M14 * b.M41;
		float resultM12 = a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32 + a.M14 * b.M42;
		float resultM13 = a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33 + a.M14 * b.M43;
		float resultM14 = a.M11 * b.M14 + a.M12 * b.M24 + a.M13 * b.M34 + a.M14 * b.M44;

		float resultM21 = a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31 + a.M24 * b.M41;
		float resultM22 = a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32 + a.M24 * b.M42;
		float resultM23 = a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33 + a.M24 * b.M43;
		float resultM24 = a.M21 * b.M14 + a.M22 * b.M24 + a.M23 * b.M34 + a.M24 * b.M44;

		float resultM31 = a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31 + a.M34 * b.M41;
		float resultM32 = a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32 + a.M34 * b.M42;
		float resultM33 = a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33 + a.M34 * b.M43;
		float resultM34 = a.M31 * b.M14 + a.M32 * b.M24 + a.M33 * b.M34 + a.M34 * b.M44;

		float resultM41 = a.M41 * b.M11 + a.M42 * b.M21 + a.M43 * b.M31 + a.M44 * b.M41;
		float resultM42 = a.M41 * b.M12 + a.M42 * b.M22 + a.M43 * b.M32 + a.M44 * b.M42;
		float resultM43 = a.M41 * b.M13 + a.M42 * b.M23 + a.M43 * b.M33 + a.M44 * b.M43;
		float resultM44 = a.M41 * b.M14 + a.M42 * b.M24 + a.M43 * b.M34 + a.M44 * b.M44;

		result.M11 = resultM11;
		result.M12 = resultM12;
		result.M13 = resultM13;
		result.M14 = resultM14;

		result.M21 = resultM21;
		result.M22 = resultM22;
		result.M23 = resultM23;
		result.M24 = resultM24;

		result.M31 = resultM31;
		result.M32 = resultM32;
		result.M33 = resultM33;
		result.M34 = resultM34;

		result.M41 = resultM41;
		result.M42 = resultM42;
		result.M43 = resultM43;
		result.M44 = resultM44;
	}

	/// <summary>
	/// Creates a quaternion from a rotation matrix.
	/// </summary>
	/// <param name="r">Rotation matrix used to create a new quaternion.</param>
	/// <param name="q">Resulting quaternion representing the same rotation as the matrix.</param>
	void Matrix::CreateQuaternion(Matrix &r, Quaternion &q)
	{
		float tr = r.M11 + r.M22 + r.M33;
		if (tr > 0)
		{
			float S = sqrt(tr + 1.0) * 2; // S=4*qw
			float inverseS = 1 / S;
			q.W = 0.25f * S;
			q.X = (r.M32 - r.M23) * inverseS;
			q.Y = (r.M13 - r.M31) * inverseS;
			q.Z = (r.M21 - r.M12) * inverseS;
		}
		else if ((r.M11 > r.M22) & (r.M11 > r.M33))
		{
			float S = sqrt(1.0 + r.M11 - r.M22 - r.M33) * 2; // S=4*qx
			float inverseS = 1 / S;
			q.W = (r.M32 - r.M23) * inverseS;
			q.X = 0.25f * S;
			q.Y = (r.M12 + r.M21) * inverseS;
			q.Z = (r.M13 + r.M31) * inverseS;
		}
		else if (r.M22 > r.M33)
		{
			float S = sqrt(1.0 + r.M22 - r.M11 - r.M33) * 2; // S=4*qy
			float inverseS = 1 / S;
			q.W = (r.M13 - r.M31) * inverseS;
			q.X = (r.M12 + r.M21) * inverseS;
			q.Y = 0.25f * S;
			q.Z = (r.M23 + r.M32) * inverseS;
		}
		else
		{
			float S = sqrt(1.0 + r.M33 - r.M11 - r.M22) * 2; // S=4*qz
			float inverseS = 1 / S;
			q.W = (r.M21 - r.M12) * inverseS;
			q.X = (r.M13 + r.M31) * inverseS;
			q.Y = (r.M23 + r.M32) * inverseS;
			q.Z = 0.25f * S;
		}
	}

	/// <summary>
	/// Transforms a vector using the 3x3 upper left component of a matrix.
	/// </summary>
	/// <param name="v">Vector to transform.</param>
	/// <param name="matrix">Transform to apply to the vector.</param>
	/// <param name="result">Transformed </param>
	void Matrix::TransformNormal(Vector3 &v, Matrix &matrix, Vector3 &result)
	{
		float vX = v.X;
		float vY = v.Y;
		float vZ = v.Z;
		result.X = vX * matrix.M11 + vY * matrix.M21 + vZ * matrix.M31;
		result.Y = vX * matrix.M12 + vY * matrix.M22 + vZ * matrix.M32;
		result.Z = vX * matrix.M13 + vY * matrix.M23 + vZ * matrix.M33;
	}

	/// <summary>
	/// Transforms a vector using a matrix.
	/// </summary>
	/// <param name="v">Vector to transform.</param>
	/// <param name="matrix">Transform to apply to the vector.</param>
	/// <param name="result">Transformed vector.</param>
	void Matrix::Transform(Vector3 &v, Matrix &matrix, Vector3 &result)
	{
		float vX = v.X;
		float vY = v.Y;
		float vZ = v.Z;
		result.X = vX * matrix.M11 + vY * matrix.M21 + vZ * matrix.M31 + matrix.M41;
		result.Y = vX * matrix.M12 + vY * matrix.M22 + vZ * matrix.M32 + matrix.M42;
		result.Z = vX * matrix.M13 + vY * matrix.M23 + vZ * matrix.M33 + matrix.M43;
	}
}
