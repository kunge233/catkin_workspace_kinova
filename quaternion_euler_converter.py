from scipy.spatial.transform import Rotation as R
import numpy as np

def quaternion_to_fixed_euler(quaternion):
    # 将四元数转换为固定轴旋转的欧拉角
    r = R.from_quat(quaternion)
    euler_angles_deg = r.as_euler('XYZ', degrees=True)  # 大写 'XYZ' 表示固定轴旋转
    return euler_angles_deg

def fixed_euler_to_quaternion(euler_angles_deg):
    # 将固定轴旋转的欧拉角转换为四元数
    r = R.from_euler('XYZ', np.radians(euler_angles_deg))  # 大写 'XYZ' 表示固定轴旋转
    quaternion = r.as_quat()
    return quaternion

def main():
    while True:
        print("\n选择输入类型:")
        print("1. 输入四元数")
        print("2. 输入欧拉角")
        print("3. 退出")
        choice = input("请输入选项 (1/2/3): ")

        if choice == '1':
            qx = float(input("请输入四元数 x: "))
            qy = float(input("请输入四元数 y: "))
            qz = float(input("请输入四元数 z: "))
            qw = float(input("请输入四元数 w: "))
            quaternion = [qx, qy, qz, qw]
            euler_angles_deg = quaternion_to_fixed_euler(quaternion)
            print(f"对应的固定轴旋转欧拉角 (度): {euler_angles_deg}")

        elif choice == '2':
            ex = float(input("请输入欧拉角 theta_x (度): "))
            ey = float(input("请输入欧拉角 theta_y (度): "))
            ez = float(input("请输入欧拉角 theta_z (度): "))
            euler_angles_deg = [ex, ey, ez]
            quaternion = fixed_euler_to_quaternion(euler_angles_deg)
            print(f"对应的四元数: {quaternion}")

        elif choice == '3':
            print("退出程序")
            break

        else:
            print("无效的选项，请重新选择")

if __name__ == "__main__":
    main()
