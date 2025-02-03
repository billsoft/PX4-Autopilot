uint64 timestamp                # 时间戳，单位为微秒
float32[3] euler_angle          # 欧拉角（roll, pitch, yaw），单位为 rad
float32[4] quaternion           # 姿态四元数 [w, x, y, z]
float32 confidence              # 置信度（范围 0~1）
