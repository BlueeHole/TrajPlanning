import cv2 as cv
import threading


class Map:
    def __init__(self, path):
        self.map = cv.imread(path)
        self.origin_map = cv.imread(path)

    def getMap(self):
        return self.map

    def refresh(self):
        self.map = self.origin_map.copy()

    def setMap(self, map):
        self.map = map

    def getSize(self):
        return self.map.shape

    def inMap(self, point):
        return self.map.shape[0] > point[0] > 0 and self.map.shape[1] > point[1] > 0

    def showMap(self):
        # img[0:10, 0:10] = [0, 0, 255]
        # 创建窗口并显示图像
        cv.namedWindow('Map')
        cv.imshow('Map', self.map)
        cv.waitKey(20)
        return 'Map'

    def __del__(self):
        cv.destroyAllWindows()
