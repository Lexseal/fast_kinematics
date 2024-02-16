#pragma once

#include <cmath>

class Quaternion {
public:
  __host__ __device__ Quaternion(float w = 1.0f, float x = 0.0f, float y = 0.0f, float z = 0.0f)
    : w(w), x(x), y(y), z(z) {}

  __host__ __device__ float norm() const {
    return std::sqrt(w * w + x * x + y * y + z * z);
  }

  __host__ __device__ Quaternion conjugate() const {
    return Quaternion(w, -x, -y, -z);
  }

  __host__ __device__ Quaternion operator+(const Quaternion& other) const {
    return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
  }

  __host__ __device__ Quaternion operator*(const Quaternion& other) const {
    float nw = w * other.w - x * other.x - y * other.y - z * other.z;
    float nx = w * other.x + x * other.w + y * other.z - z * other.y;
    float ny = w * other.y - x * other.z + y * other.w + z * other.x;
    float nz = w * other.z + x * other.y - y * other.x + z * other.w;
    return Quaternion(nw, nx, ny, nz);
  }

  __host__ __device__ Quaternion operator*(float scalar) const {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
  }

  __host__ __device__ Quaternion operator/(float scalar) const {
    return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
  }

  __host__ __device__ Quaternion& operator+=(const Quaternion& other) {
    w += other.w;
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  __host__ __device__ Quaternion& operator*=(const Quaternion& other) {
    float nw = w * other.w - x * other.x - y * other.y - z * other.z;
    float nx = w * other.x + x * other.w + y * other.z - z * other.y;
    float ny = w * other.y - x * other.z + y * other.w + z * other.x;
    float nz = w * other.z + x * other.y - y * other.x + z * other.w;
    w = nw;
    x = nx;
    y = ny;
    z = nz;
    return *this;
  }

  __host__ __device__ Quaternion& operator*=(float scalar) {
    w *= scalar;
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  __host__ __device__ Quaternion& operator/=(float scalar) {
    w /= scalar;
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }

  __host__ __device__ void normalize() {
    float n = norm();
    w /= n;
    x /= n;
    y /= n;
    z /= n;
  }

  __host__ __device__ Quaternion normalized() const {
    float n = norm();
    return Quaternion(w / n, x / n, y / n, z / n);
  }

  __host__ __device__ Quaternion inverse() const {
    return conjugate() / (w * w + x * x + y * y + z * z);
  }

  __host__ __device__ static Quaternion Identity() {
    return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
  }

  __host__ __device__ static Quaternion FromAngleAxis(float angle, float x, float y, float z) {
    float halfAngle = angle / 2;
    float s = std::sin(halfAngle);
    return Quaternion(std::cos(halfAngle), x * s, y * s, z * s);
  }

public:
  float w, x, y, z;
};
