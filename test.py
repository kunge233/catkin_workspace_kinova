import tf.transformations
import math

# 给定的四元数
quaternion = [-0.6537199093950201, 0.7438234097415034, -0.08853944881187369, 0.10741406418705332]

# 将四元数转换为欧拉角（弧度）
euler_rad = tf.transformations.euler_from_quaternion(quaternion)

# 将欧拉角从弧度转换为度
euler_deg = [math.degrees(angle) for angle in euler_rad]

print("欧拉角 (度):", euler_deg)
