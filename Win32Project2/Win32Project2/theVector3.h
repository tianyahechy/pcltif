#pragma once
class Vector3
{
public:
	Vector3(float fx, float fy, float fz)
		:x(fx), y(fy), z(fz)
	{
	}

	// Subtract
	Vector3 operator - (const Vector3& v) const
	{
		return Vector3(x - v.x, y - v.y, z - v.z);
	}

	// Dot product
	float Dot(const Vector3& v) const
	{
		return x * v.x + y * v.y + z * v.z;
	}

	// Cross product
	Vector3 Cross(const Vector3& v) const
	{
		return Vector3(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x);
	}

public:
	float x, y, z;
};