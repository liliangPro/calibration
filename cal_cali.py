import numpy as np
import glob, cv2, os
from scipy.spatial.transform import Rotation

np.set_printoptions(suppress=True)  # 不显示科学计数法

class calibration_cy:
    def __init__(self):
        # 参数设置
        self.folder_adress = f"data/calib_20250703"
        # self.folder_adress = f"data/eye_in_hand"
        # self.calib_type, self.chess_x, self.chess_y, self.chess_len = "eye_on_hand", 8, 11, 10
        self.calib_type, self.chess_x, self.chess_y, self.chess_len = "eye_on_hand", 8, 11, 10
        # self.calib_type, self.chess_x, self.chess_y, self.chess_len = "eye_in_hand", 7, 9, 20

    def RT_b_to_c(self):
        # 参数设置
        calib_para = self.get_file_data(f"{self.folder_adress}/calib_para.txt")
        R_b_to_c, T_b_to_c, right_index = [], [], []
        chess_x, chess_y, chess_len = self.chess_x, self.chess_y, self.chess_len
        fx, fy, ppx, ppy = map(float, (calib_para['fx'], calib_para['fy'], calib_para['ppx'], calib_para['ppy']))
        png_files = glob.glob(os.path.join(self.folder_adress, "*.png"))  # 获取所有图片
        mtx = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]], dtype=np.float64)
        # 角点计算
        for i in range(len(png_files)):
            img = cv2.imread(f"{self.folder_adress}/{i}.png")
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # 角点检测
            ret, corners = cv2.findChessboardCorners(gray, (chess_x, chess_y), None)
            if not ret:
                continue
            right_index.append(i)
            corners_det = corners[:].reshape(-1, 2)
            point_1, point_2 = corners_det[chess_x - 1], corners_det[chess_x * 2 - 2]
            point_3, point_4 = corners_det[chess_x - 2], corners_det[chess_x * 2 - 3]
            (cx1, cy1), (cx2, cy2) = (point_1 + point_2) / 2, (point_3 + point_4) / 2
            diff = abs(int(gray[int(cy1), int(cx1)]) - int(gray[int(cy2), int(cx2)]))
            print(f"{diff=}")
            if  diff > 60:
                corners = corners[::-1].astype(np.float32)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)  # 迭代次数、精度
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            # 计算转换矩阵
            points_img = corners[:, 0, :].T
            points_obj = np.zeros((chess_x * chess_y, 3), np.float32)
            points_obj[:, :2] = (np.mgrid[0:chess_x, 0:chess_y] * chess_len).T.reshape(-1, 2)
            ret, R, T = cv2.solvePnP(points_obj, points_img.T, mtx, distCoeffs=np.zeros((4, 1)))
            print(ret)
            R_b_to_c.append(cv2.Rodrigues(R)[0])
            T_b_to_c.append(T)
            # vis
            cv2.drawChessboardCorners(img, (chess_x, chess_y), corners, ret)
            cv2.imshow('findCorners', img)
            cv2.waitKey(0)

        return R_b_to_c, T_b_to_c, right_index

    def RT_e_to_b(self, right_index):
        xarm_data = self.get_file_data(f"{self.folder_adress}/xarm_data.txt")
        R_e_to_b, T_e_to_b = [], []
        for i, (key, value) in enumerate(xarm_data.items()):
            if not i in right_index:
                continue
            pos_R, pos_T, RT_e_to_b = eval(value)[3:], eval(value)[:3], np.eye(4)
            RT_e_to_b[:3, :3], RT_e_to_b[:3, 3] = Rotation.from_euler("xyz", pos_R).as_matrix(), pos_T
            if self.calib_type == "eye_on_hand":  # 眼在手外，整体求逆
                RT_e_to_b = np.linalg.inv(RT_e_to_b)
            R_e_to_b.append(RT_e_to_b[:3, :3])
            T_e_to_b.append(RT_e_to_b[:3, 3].reshape((3, 1)))
        return R_e_to_b, T_e_to_b

    def get_file_data(self, file_path):
        data = {}
        with open(file_path, "r") as file:
            for line in file:
                key, value = line.split(":")
                data[key] = value.strip()
        return data

    def calib_calculation(self):
        # 计算RT_board_to_cam
        R_b_to_c, T_b_to_c, right_index = self.RT_b_to_c()
        # 计算RT_end_to_base
        R_e_to_b, T_e_to_b = self.RT_e_to_b(right_index)
        # 标定结果计算
        R_hand_eye, T_hand_eye = cv2.calibrateHandEye(R_e_to_b, T_e_to_b, R_b_to_c, T_b_to_c, cv2.CALIB_HAND_EYE_TSAI)
        RT = np.vstack([np.hstack([R_hand_eye, T_hand_eye.reshape(-1, 1)]), np.array([0, 0, 0, 1])])
        print(f"{self.calib_type}:\n {RT=}")
        # 保存标定结果
        np.savetxt(f"{self.folder_adress}/calib_result.txt", RT, delimiter=",", fmt="%.3f")

    def run(self):
        self.calib_calculation()
        pass


if __name__ == '__main__':
    c = calibration_cy()
    c.run()
