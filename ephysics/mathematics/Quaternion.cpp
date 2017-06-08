/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/mathematics/Quaternion.h>
#include <ephysics/mathematics/Vector3.h>
#include <cassert>

// Namespace
using namespace reactphysics3d;

// Constructor of the class
Quaternion::Quaternion() : x(0.0), y(0.0), z(0.0), w(0.0) {

}

// Constructor with arguments
Quaternion::Quaternion(float newX, float newY, float newZ, float newW)
		   :x(newX), y(newY), z(newZ), w(newW) {

}

// Constructor with the component w and the vector v=(x y z)
Quaternion::Quaternion(float newW, const Vector3& v) : x(v.x), y(v.y), z(v.z), w(newW) {

}

// Constructor which convert Euler angles (in radians) to a quaternion
Quaternion::Quaternion(float angleX, float angleY, float angleZ) {
	initWithEulerAngles(angleX, angleY, angleZ);
}

// Constructor which convert Euler angles (in radians) to a quaternion
Quaternion::Quaternion(const Vector3& eulerAngles) {
	initWithEulerAngles(eulerAngles.x, eulerAngles.y, eulerAngles.z);
}

// Copy-constructor
Quaternion::Quaternion(const Quaternion& quaternion)
		   :x(quaternion.x), y(quaternion.y), z(quaternion.z), w(quaternion.w) {

}

// Create a unit quaternion from a rotation matrix
Quaternion::Quaternion(const Matrix3x3& matrix) {

	// Get the trace of the matrix
	float trace = matrix.getTrace();

	float r;
	float s;

	if (trace < 0.0) {
		if (matrix[1][1] > matrix[0][0]) {
			if(matrix[2][2] > matrix[1][1]) {
				r = sqrt(matrix[2][2] - matrix[0][0] - matrix[1][1] + float(1.0));
				s = float(0.5) / r;
				
				// Compute the quaternion
				x = (matrix[2][0] + matrix[0][2]) * s;
				y = (matrix[1][2] + matrix[2][1]) * s;
				z = float(0.5) * r;
				w = (matrix[1][0] - matrix[0][1]) * s;
			}
			else {
				r = sqrt(matrix[1][1] - matrix[2][2] - matrix[0][0] + float(1.0));
				s = float(0.5) / r;

				// Compute the quaternion
				x = (matrix[0][1] + matrix[1][0]) * s;
				y = float(0.5) * r;
				z = (matrix[1][2] + matrix[2][1]) * s;
				w = (matrix[0][2] - matrix[2][0]) * s;
			}
		}
		else if (matrix[2][2] > matrix[0][0]) {
			r = sqrt(matrix[2][2] - matrix[0][0] - matrix[1][1] + float(1.0));
			s = float(0.5) / r;

			// Compute the quaternion
			x = (matrix[2][0] + matrix[0][2]) * s;
			y = (matrix[1][2] + matrix[2][1]) * s;
			z = float(0.5) * r;
			w = (matrix[1][0] - matrix[0][1]) * s;
		}
		else {
			r = sqrt(matrix[0][0] - matrix[1][1] - matrix[2][2] + float(1.0));
			s = float(0.5) / r;

			// Compute the quaternion
			x = float(0.5) * r;
			y = (matrix[0][1] + matrix[1][0]) * s;
			z = (matrix[2][0] - matrix[0][2]) * s;
			w = (matrix[2][1] - matrix[1][2]) * s;
		}
	}
	else {
		r = sqrt(trace + float(1.0));
		s = float(0.5) / r;

		// Compute the quaternion
		x = (matrix[2][1] - matrix[1][2]) * s;
		y = (matrix[0][2] - matrix[2][0]) * s;
		z = (matrix[1][0] - matrix[0][1]) * s;
		w = float(0.5) * r;
	}
}

// Destructor
Quaternion::~Quaternion() {

}

// Compute the rotation angle (in radians) and the rotation axis
/// This method is used to get the rotation angle (in radian) and the unit
/// rotation axis of an orientation quaternion.
void Quaternion::getRotationAngleAxis(float& angle, Vector3& axis) const {
	Quaternion quaternion;

	// If the quaternion is unit
	if (length() == 1.0) {
		quaternion = *this;
	}
	else {
		// We compute the unit quaternion
		quaternion = getUnit();
	}

	// Compute the roation angle
	angle = acos(quaternion.w) * float(2.0);

	// Compute the 3D rotation axis
	Vector3 rotationAxis(quaternion.x, quaternion.y, quaternion.z);

	// Normalize the rotation axis
	rotationAxis = rotationAxis.getUnit();

	// Set the rotation axis values
	axis.setAllValues(rotationAxis.x, rotationAxis.y, rotationAxis.z);
}

// Return the orientation matrix corresponding to this quaternion
Matrix3x3 Quaternion::getMatrix() const {

	float nQ = x*x + y*y + z*z + w*w;
	float s = 0.0;

	if (nQ > 0.0) {
		s = float(2.0) / nQ;
	}

	// Computations used for optimization (less multiplications)
	float xs = x*s;
	float ys = y*s;
	float zs = z*s;
	float wxs = w*xs;
	float wys = w*ys;
	float wzs = w*zs;
	float xxs = x*xs;
	float xys = x*ys;
	float xzs = x*zs;
	float yys = y*ys;
	float yzs = y*zs;
	float zzs = z*zs;

	// Create the matrix corresponding to the quaternion
	return Matrix3x3(float(1.0) - yys - zzs, xys-wzs, xzs + wys,
					 xys + wzs, float(1.0) - xxs - zzs, yzs-wxs,
					 xzs-wys, yzs + wxs, float(1.0) - xxs - yys);
}

// Compute the spherical linear int32_terpolation between two quaternions.
/// The t argument has to be such that 0 <= t <= 1. This method is static.
Quaternion Quaternion::slerp(const Quaternion& quaternion1,
							 const Quaternion& quaternion2, float t) {
	assert(t >= 0.0 && t <= 1.0);

	float invert = 1.0;

	// Compute cos(theta) using the quaternion scalar product
	float cosineTheta = quaternion1.dot(quaternion2);

	// Take care of the sign of cosineTheta
	if (cosineTheta < 0.0) {
			cosineTheta = -cosineTheta;
			invert = -1.0;
	}

	// Because of precision, if cos(theta) is nearly 1,
	// therefore theta is nearly 0 and we can write
	// sin((1-t)*theta) as (1-t) and sin(t*theta) as t
	const float epsilon = float(0.00001);
	if(1-cosineTheta < epsilon) {
		return quaternion1 * (float(1.0)-t) + quaternion2 * (t * invert);
	}

	// Compute the theta angle
	float theta = acos(cosineTheta);

	// Compute sin(theta)
	float sineTheta = sin(theta);

	// Compute the two coefficients that are in the spherical linear int32_terpolation formula
	float coeff1 = sin((float(1.0)-t)*theta) / sineTheta;
	float coeff2 = sin(t*theta) / sineTheta * invert;

	// Compute and return the int32_terpolated quaternion
	return quaternion1 * coeff1 + quaternion2 * coeff2;
}

// Initialize the quaternion using Euler angles
void Quaternion::initWithEulerAngles(float angleX, float angleY, float angleZ) {

	float angle = angleX * float(0.5);
	const float sinX = std::sin(angle);
	const float cosX = std::cos(angle);

	angle = angleY * float(0.5);
	const float sinY = std::sin(angle);
	const float cosY = std::cos(angle);

	angle = angleZ * float(0.5);
	const float sinZ = std::sin(angle);
	const float cosZ = std::cos(angle);

	const float cosYcosZ = cosY * cosZ;
	const float sinYcosZ = sinY * cosZ;
	const float cosYsinZ = cosY * sinZ;
	const float sinYsinZ = sinY * sinZ;

	x = sinX * cosYcosZ - cosX * sinYsinZ;
	y = cosX * sinYcosZ + sinX * cosYsinZ;
	z = cosX * cosYsinZ - sinX * sinYcosZ;
	w = cosX * cosYcosZ + sinX * sinYsinZ;

	// Normalize the quaternion
	normalize();
}
