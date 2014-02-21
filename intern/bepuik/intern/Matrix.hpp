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

#include "Quaternion.hpp"
#include "Vector3.hpp"

namespace BEPUmath
{
	struct Matrix
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
		/// Value at row 1, column 4 of the matrix.
		/// </summary>
		float M14;

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
		/// Value at row 2, column 4 of the matrix.
		/// </summary>
		float M24;

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
		/// Value at row 3, column 4 of the matrix.
		/// </summary>
		float M34;

		/// <summary>
		/// Value at row 4, column 1 of the matrix.
		/// </summary>
		float M41;

		/// <summary>
		/// Value at row 4, column 2 of the matrix.
		/// </summary>
		float M42;

		/// <summary>
		/// Value at row 4, column 3 of the matrix.
		/// </summary>
		float M43;

		/// <summary>
		/// Value at row 4, column 4 of the matrix.
		/// </summary>
		float M44;

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
		Matrix(float m11, float m12, float m13, float m14,
			float m21, float m22, float m23, float m24,
			float m31, float m32, float m33, float m34,
			float m41, float m42, float m43, float m44);

		Matrix();


		/// <summary>
		/// Computes the determinant of the matrix.
		/// </summary>
		/// <returns></returns>
		float Determinant();


		/// <summary>
		/// Creates a matrix representing the given axis and angle rotation.
		/// </summary>
		/// <param name="axis">Axis around which to rotate.</param>
		/// <param name="angle">Angle to rotate around the axis.</param>
		/// <param name="result">Matrix created from the axis and angle.</param>
		static void CreateFromAxisAngle(Vector3 &axis, float angle, Matrix &result);

		/// <summary>
		/// Creates a rotation matrix from a quaternion.
		/// </summary>
		/// <param name="quaternion">Quaternion to convert.</param>
		/// <param name="result">Rotation matrix created from the quaternion.</param>
		static void CreateFromQuaternion(Quaternion &quaternion, Matrix &result);

		/// <summary>
		/// Multiplies two matrices together.
		/// </summary>
		/// <param name="a">First matrix to multiply.</param>
		/// <param name="b">Second matrix to multiply.</param>
		/// <param name="result">Combined transformation.</param>
		static void Multiply(Matrix &a, Matrix &b, Matrix &result);

		/// <summary>
		/// Creates a quaternion from a rotation matrix.
		/// </summary>
		/// <param name="r">Rotation matrix used to create a new quaternion.</param>
		/// <param name="q">Quaternion representing the same rotation as the matrix.</param>
		static void CreateQuaternion(Matrix &r, Quaternion &q);

		/// <summary>
		/// Transforms a vector using the 3x3 upper left component of a matrix.
		/// </summary>
		/// <param name="v">Vector to transform.</param>
		/// <param name="matrix">Transform to apply to the vector.</param>
		/// <param name="result">Transformed </param>
		static void TransformNormal(Vector3 &v, Matrix &matrix, Vector3 &result);

		/// <summary>
		/// Transforms a vector using a matrix.
		/// </summary>
		/// <param name="v">Vector to transform.</param>
		/// <param name="matrix">Transform to apply to the vector.</param>
		/// <param name="result">Transformed vector.</param>
		static void Transform(Vector3 &v, Matrix &matrix, Vector3 &result);
	};
}


