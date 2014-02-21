#pragma once
namespace BEPUmath
{
	/// <summary>
	/// Contains helper math methods used by the engine.
	/// </summary>
	class MathHelper
	{
	public:
		/// <summary>
		/// Approximate value of Pi.
		/// </summary>
		static const float Pi;// = 3.141592653589793239f;

		/// <summary>
		/// Approximate value of Pi multiplied by two.
		/// </summary>
		static const float TwoPi;// = 6.283185307179586477f;

		/// <summary>
		/// Approximate value of Pi divided by two.
		/// </summary>
		static const float PiOver2;// = 1.570796326794896619f;

		/// <summary>
		/// Approximate value of Pi divided by four.
		/// </summary>
		static const float PiOver4;// = 0.785398163397448310f;

		/// <summary>
		/// Clamps a value between a minimum and maximum value.
		/// </summary>
		/// <param name="value">Value to clamp.</param>
		/// <param name="min">Minimum value.  If the value is less than this, the minimum is returned instead.</param>
		/// <param name="max">Maximum value.  If the value is more than this, the maximum is returned instead.</param>
		/// <returns>Clamped value.</returns>
		static float Clamp(float value, float min, float max);

		/// <summary>
		/// Returns the higher value of the two parameters.
		/// </summary>
		/// <param name="a">First value.</param>
		/// <param name="b">Second value.</param>
		/// <returns>Higher value of the two parameters.</returns>
		static float Max(float a, float b);

		/// <summary>
		/// Returns the lower value of the two parameters.
		/// </summary>
		/// <param name="a">First value.</param>
		/// <param name="b">Second value.</param>
		/// <returns>Lower value of the two parameters.</returns>
		static float Min(float a, float b);

		/// <summary>
		/// Converts degrees to radians.
		/// </summary>
		/// <param name="degrees">Degrees to convert.</param>
		/// <returns>Radians equivalent to the input degrees.</returns>
		static float ToRadians(float degrees);

		/// <summary>
		/// Converts radians to degrees.
		/// </summary>
		/// <param name="radians">Radians to convert.</param>
		/// <returns>Degrees equivalent to the input radians.</returns>
		static float ToDegrees(float radians);
	};
}
