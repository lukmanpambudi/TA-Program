import cv2
import numpy as np
import utilsV4
from ultralytics import YOLO
import time

last_mask_time = time.time()

def getLane(results, display=2):
    global last_mask_time, frame, tampilResults
    frameCopy = frame.copy()

    mask_np = utilsV4.masking(results)
    if mask_np is not None:
        last_mask_time = time.time()  
        points = utilsV4.valTrackbars()
        hT, wT, c = frame.shape
        
        frameWarp = utilsV4.warping(mask_np, points, wT, hT)
        if frameWarp is None or frameWarp.size == 0:
            print("Error: frameWarp is invalid or has zero size.")
            return
        else: 
            print("ada nilai framewarp")

        frameWarpPoints = utilsV4.drawPoints(frameCopy, points)
        processedImage = utilsV4.isEndOfLane(results, model, frameWarp, display=True)
        if processedImage is None or processedImage.size == 0:
            print("processedImage tidak ada nilainya ")
            return
        else: 
            print("ada nilai processed image")

        if display == 1:
            stack_img = utilsV4.stackImages(0.7, ([tampilResults, frameWarpPoints], [mask_np, frameWarp]))
            cv2.imshow('Image Processing', stack_img)
            cv2.imshow('Processed Image', processedImage)
        elif display == 2:
            stack_img = utilsV4.stackImages(0.7, ([frameWarpPoints, mask_np], [frameWarp, processedImage]))
            cv2.imshow('Image Processing', stack_img)
    else:
        current_time = time.time()
        if current_time - last_mask_time > 15:
            if cv2.getWindowProperty("Masking", cv2.WND_PROP_VISIBLE) > 0:
                cv2.destroyWindow("Masking")
            if cv2.getWindowProperty("Warp", cv2.WND_PROP_VISIBLE) > 0:
                cv2.destroyWindow("Warp")
            if cv2.getWindowProperty("Warp Points", cv2.WND_PROP_VISIBLE) > 0:
                cv2.destroyWindow("Warp Points")
            if cv2.getWindowProperty("Processed Image", cv2.WND_PROP_VISIBLE) > 0:
                cv2.destroyWindow("Processed Image")
            if cv2.getWindowProperty("Image Processing", cv2.WND_PROP_VISIBLE) > 0:
                cv2.destroyWindow("Image Processing")
            if cv2.getWindowProperty("YOLOv8 Instance Segmentation", cv2.WND_PROP_VISIBLE) > 0:
                cv2.destroyWindow("YOLOv8 Instance Segmentation")

if __name__ == '__main__': 
    # model = YOLO("/home/pambudi/Yolov8/model/YOLO8n/YOLO8n_13Juli.pt")
    model = YOLO("/home/pambudi/Yolov8/model/YOLO8n/Best_3Agustus.pt")
    # model = YOLO("/home/pambudi/Yolov8/model/YOLO8n/best23Juli_v2.pt")
    video_path = ("/home/pambudi/Yolov8/model/WIN_20240530_16_26_19_Pro.mp4")

    cap = cv2.VideoCapture(video_path)  
    # cap = cv2.VideoCapture(2)    
    if not cap.isOpened():
        print("Gagal membuka video.")
        exit()
    initializeTrackBarVals = [8, 128, 0, 179]
    utilsV4.initializeTrackbars(initializeTrackBarVals)
    frameCounter = 0
    
    # Initialize VideoWriter object for MP4 format
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter('/home/pambudi/Yolov8/Data/output1.mp4', fourcc, 20.0, (512, 256))
    
    while True:
        frameCounter += 1
        if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frameCounter = 0

        ret, video = cap.read()
        if not ret:
            print("Gagal membaca frame dari video.")
            break
        
        # frame = utilsV4.tambahkan_bingkai(video)
        frame = cv2.resize(video, (512, 256))
        results = model(frame, imgsz=416, conf=0.5)
        tampilResults = results[0].plot()
        getLane(results, display=2)
        
        # Write the frame to the output video file
        # out.write(tampilResults)  
        
        cv2.imshow("YOLOv8 Instance Segmentation", tampilResults)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    cap.release()
    # out.release()
    cv2.destroyAllWindows()
