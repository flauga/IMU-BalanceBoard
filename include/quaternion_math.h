#pragma once

#include "types.h"
#include <cstdint>

namespace quat {

Quaternion identity();
Quaternion multiply(const Quaternion& a, const Quaternion& b);
Quaternion conjugate(const Quaternion& q);
Quaternion normalize(const Quaternion& q);
EulerAngles toEuler(const Quaternion& q);

}  // namespace quat
