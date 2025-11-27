import numpy as np

def solve_AX_equals_B(A, B):
    # 检查A是否可逆
    if np.linalg.det(A) == 0:
        raise ValueError("矩阵 A 是奇异矩阵，无法求逆")
    
    A_inv = np.linalg.inv(A)
    X = np.dot(A_inv, B)
    return X

# 示例输入
A = np.array([[1, 0, 0],
              [0, -1, 0],
              [0, 0, -1]])

B = np.array([[0, -1, 0],
              [-1, 0, 0],
              [0, 0, -1]])

# 求解 X
X = solve_AX_equals_B(A, B)

print("解出的 X 矩阵为：")
print(X)
