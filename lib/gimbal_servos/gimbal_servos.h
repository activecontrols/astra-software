#pragma once

namespace GimbalServos {
void centerGimbal();
void setGimbalAngle(float bottom, float top, float* out_bottom, float* out_top);
void setGimbalAngle(float bottom, float top);
void init();

void setServoAngle(float bottom, float top);

} // namespace GimbalServos