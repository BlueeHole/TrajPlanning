import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from map_manager import Map

plt.ion() 	# 开启interactive mode 成功的关键函数
plt.figure(figsize=(12, 6))
t_now = 0
x = 0
y = 0
x_v = 0
y_v = 5

img = mpimg.imread('map.jpg')
plt.imshow(img)
map = Map('map.jpg')

while True:
	# plt.clf() # 清空画布上的所有内容。此处不能调用此函数，不然之前画出的点，将会被清空。
	x = x + x_v
	y = y + y_v
	x_v = x_v + 5
	if not map.inMap((x/2, y/2)):
		plt.pause(0.001)
		continue

	# plt.plot(t_now, sin(t_now), '.') # 每次对画布添加一个点，覆盖式的
	plt.scatter(x, y, c='b', marker='o', linewidths=0.01)
	plt.pause(0.001)