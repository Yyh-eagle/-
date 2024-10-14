import cv2
print(cv2.getBuildInformation())
def test_camera_stream():
    # 打开默认摄像头（通常是0）
    cap = cv2.VideoCapture(0)
    cap2 = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    if not cap2.isOpened():
        print("无法打开摄像头")
        return
    while True:
        # 捕获视频流中的一帧
        ret, frame = cap.read()
        ret2, frame2 = cap2.read() 
        # 检查是否成功读取帧
        if not ret:
            print("无法接收帧，退出...")
            break
        if not ret2:
            print("无法接收帧，退出...")
            break
        # 在窗口中显示视频流
        cv2.imshow('Camera Stream', frame)
        cv2.imshow('Camera2 Stream', frame2)
        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头和关闭窗口
    cap.release()
    cap2.release()
    cv2.destroyAllWindows()

# 调用测试函数
test_camera_stream()
