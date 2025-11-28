from PySide6 import QtCore, QtGui,QtUiTools
from PySide6.QtCore import QEventLoop, QTimer, Signal, SIGNAL, QObject
from PySide6.QtGui import QImage, QPixmap, QIcon
from PySide6.QtWidgets import QApplication, QMainWindow  # 窗口
import threading as th
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
import time,os,cv2,sys,math,requests,json

np.set_printoptions(precision=4, suppress=True)  # 不显示科学计数法

class qt_cy(QMainWindow):
    textWritten = QtCore.Signal(str)
    def __init__(self):
        super().__init__()
        # realsense API
        self.realsense_init()
        # xarm API
        self.xarm_init()
        # qt_init
        self.qt_init()
        # cali_init
        self.cali_init()
    
    def qt_init(self):
        self.ui = QtUiTools.QUiLoader().load(f"{os.path.dirname(__file__)}/ui/calibration.ui")  # 加载文件
        self.ui.pushButton.clicked.connect(lambda: self.button_multi(msg="open_cam"))
        self.ui.pushButton_2.clicked.connect(lambda: self.button_multi(msg="save_data"))
        # 终端显示
        sys.stdout = self
        sys.stderr = self
        self.ui.textEdit.connect(sys.stdout, QtCore.SIGNAL("textWritten(QString)"), self.outputWritten)
        print("终端信息显示中")
    
    def cali_init(self):
        # 创建文件夹
        t_time = time.strftime("%Y%m%d", time.localtime())
        self.folder_adress = f"data/calib_{t_time}"  # 默认是当前日期
        if os.path.exists(self.folder_adress):
            print("文件夹已存在")
        else:
            os.makedirs(self.folder_adress)
            print(f"创建成功{self.folder_adress}")
        fx, fy,ppx, ppy = self.intr2.fx, self.intr2.fy,self.intr2.ppx, self.intr2.ppy
        self.calib_intr = {"fx": fx, "fy": fy, "ppx": ppx, "ppy": ppy}
        with open(f"{self.folder_adress}/calib_para.txt", "w") as file:
            for key, value in self.calib_intr.items():
                file.write(f"{key}:{value}\n")
        print(self.calib_intr)
        # 加载UI文件
        self.img_num = 0

    def xarm_init(self):
        self.url = "http://127.0.0.1:7000/"
            
    def realsense_init(self):
        self.pipeline2, rs_config2 = rs.pipeline(), rs.config()
        rs_config2.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        rs_config2.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        # rs_config2.enable_device("021422060263")
        self.align2, cfg2 = rs.align(rs.stream.color), self.pipeline2.start(rs_config2)
        self.intr2 = cfg2.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.depth_scale2 = cfg2.get_device().first_depth_sensor().get_depth_scale()
        print(self.depth_scale2)
        self.mtx2 = np.array([[self.intr2.fx, 0, self.intr2.ppx],
                              [0, self.intr2.fy, self.intr2.ppy],
                              [0, 0, 1]], dtype=np.float64)  # 相机内参

    def get_image2(self):
        frames = self.pipeline2.wait_for_frames()
        aligned_frames = self.align2.process(frames)
        color_frame, depth_frame = frames.get_color_frame(), aligned_frames.get_depth_frame()
        self.color_image2 = np.asanyarray(color_frame.get_data())
        self.depth_image2 = np.asanyarray(depth_frame.get_data())

    def button_multi(self, msg):
        th.Thread(target=self.multi_th, args=(msg,), daemon=True).start()

    def multi_th(self, msg):
        if msg == "open_cam":
            current_text = self.ui.pushButton.text()
            print(current_text)
            if current_text == "开启相机":
                self.start_cam = True
                self.ui.pushButton.setText("关闭相机")
            elif current_text == "关闭相机":
                self.start_cam = False
                self.ui.pushButton.setText("开启相机")
            while self.start_cam:
                self.get_image2()
                img, (h, w, ch) = self.color_image2, self.color_image2.shape
                self.ui.label.setPixmap(QtGui.QPixmap.fromImage(QImage(img, w, h, ch * w, QImage.Format_BGR888)))
            else:
                self.ui.label.clear()
        elif msg == "save_data":
            pos = self.get_pos()
            ret = cv2.imwrite(f"{self.folder_adress}/{self.img_num}.png", self.color_image2)
            print(f"图片{self.img_num}保存:{ret}")
            print(f"保存成功{pos}")
            with open(f"{self.folder_adress}/xarm_data.txt", "a") as file:
                file.write(f"{self.img_num}:{pos}\n")
            self.img_num += 1

    def get_pos(self):
        response = requests.get(self.url+"get_pos")
        pos=  np.array(response.json()["pos"])
        # quat to euler
        quat = np.roll(pos[3:],-1) # 整体向左移动,wxyz → xyzw
        rpy = Rotation.from_quat(quat).as_euler('xyz')
        end_pos = np.concatenate((pos[:3],rpy))
        end_pos[:3] = [x * 1000 for x in end_pos[:3]]
        return end_pos

    def outputWritten(self, text):
        cursor = self.ui.textEdit.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        cursor.insertText(text)
        self.ui.textEdit.setTextCursor(cursor)
        self.ui.textEdit.ensureCursorVisible()

    def flush(self):
        pass

    def write(self, text):
        self.textWritten.emit(str(text))
        loop = QEventLoop()
        QTimer.singleShot(10, loop.quit)
        loop.exec()

    def run(self):
        pass

if __name__ == "__main__":
    app = QApplication(sys.argv) 
    q = qt_cy()
    q.run()  #
    q.ui.show()  
    sys.exit(app.exec())  
