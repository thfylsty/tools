


imgPath = "youimgPath"  # 读取图片路径
videoPath = "youvideoPath"  # 保存视频路径

images = os.listdir(imgPath)
fps = 25  # 每秒25帧数

# VideoWriter_fourcc为视频编解码器 ('I', '4', '2', '0') —>(.avi) 、('P', 'I', 'M', 'I')—>(.avi)、('X', 'V', 'I', 'D')—>(.avi)、('T', 'H', 'E', 'O')—>.ogv、('F', 'L', 'V', '1')—>.flv、('m', 'p', '4', 'v')—>.mp4
fourcc = VideoWriter_fourcc(*"MJPG")

image = Image.open(imgPath + images[0])
videoWriter = cv2.VideoWriter(videoPath, fourcc, fps, image.size)
for im_name in range(len(images)):
    frame = cv2.imread(imgPath + images[im_name])  
    print(im_name)
    videoWriter.write(frame)
videoWriter.release()
cv2.destroyAllWindows()

