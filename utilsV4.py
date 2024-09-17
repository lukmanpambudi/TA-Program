import cv2
import numpy as np
import torch
import time 
import serial 
import csv
import time
import os
# import Jetson.GPIO as GPIO

esp_ser = serial.Serial(
    port = '/dev/ttyUSB0',
    # port = 'COM4',
    baudrate = 115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1,
    # timeout=1,
)

delta_error = None
prev_error = None

start_detection_time = None
ujung_jalur_detected = False

robot_berbelok = None  
robot_lurus = None

last_detected_movement = None

detected_jalur = False
detected_ujung_jalur = False

# Setup GPIO
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Fungsi untuk mendapatkan nama file CSV berdasarkan waktu saat ini
# Generate CSV untuk data General
def get_csv_filename(directory, prefix):
    current_time = time.strftime('%H:%M_%d%b.csv')
    return f'{directory}/{prefix}_{current_time}'

def write_csv_header(filename, fieldnames):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

def write_to_csv(filename, fieldnames, clss, confidence, fps, error, delta_error):
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        timestamp = time.strftime('%S')
        writer.writerow({'Timestamp': timestamp, 'Class': clss, 'Confidence Score': confidence, 'FPS': f"{fps:.2f}", 'Error': error, 'Delta Error': delta_error})

# Mendapatkan nama file CSV berdasarkan waktu saat ini
csv_filename_general = get_csv_filename('/home/pambudi/Yolov8/Data/FLC/General', 'data')
csv_filename_model = get_csv_filename('/home/pambudi/Yolov8/Data/FLC/model', 'data')

# Header untuk kedua file CSV
fieldnames = ['Timestamp', 'Class', 'Confidence Score', 'FPS', 'Error', 'Delta Error']

# Menulis header ke kedua file CSV
write_csv_header(csv_filename_general, fieldnames)
write_csv_header(csv_filename_model, fieldnames)     

def tambahkan_bingkai(video, lebar_bingkai_vertikal=128, tinggi_bingkai_horizontal=64):
    h, w, _ = video.shape

    # Mengatur ulang ukuran video menjadi lebih kecil
    new_h = h - 2 * tinggi_bingkai_horizontal
    new_w = w - 2 * lebar_bingkai_vertikal
    video_kecil = cv2.resize(video, (new_w, new_h))

    # Membuat bingkai vertikal (sisi kiri dan kanan)
    bingkai_vertikal = np.zeros((new_h, lebar_bingkai_vertikal, 3), dtype=np.uint8)
    bingkai_vertikal[:] = (0, 0, 0)  # Set warna bingkai ke hitam
    
    # Membuat bingkai horizontal (sisi atas dan bawah)
    bingkai_horizontal = np.zeros((tinggi_bingkai_horizontal, new_w + 2 * lebar_bingkai_vertikal, 3), dtype=np.uint8)
    bingkai_horizontal[:] = (0, 0, 0)  # Set warna bingkai ke hitam

    # Menambahkan bingkai di sisi kiri dan kanan
    video_dengan_bingkai_vertikal = np.concatenate((bingkai_vertikal, video_kecil, bingkai_vertikal), axis=1)

    # Menambahkan bingkai di sisi atas dan bawah
    frame_dengan_bingkai = np.concatenate((bingkai_horizontal, video_dengan_bingkai_vertikal, bingkai_horizontal), axis=0)
    
    return frame_dengan_bingkai

def masking(results):
    all_masks = None

    for result in results:
        if len(result) < 1:
            continue

        masks = result.masks.data
        boxes = result.boxes.data
        clss = boxes[:, 5]

        for class_id in range(int(torch.max(clss).item()) + 1):
            class_indices = torch.where(clss == class_id)
            class_masks = masks[class_indices]
            class_mask = torch.any(class_masks, dim=0).int()

            if all_masks is None:
                all_masks = torch.zeros_like(class_mask)

            all_masks += class_mask

    if all_masks is None:
        return None
    all_masks *= 255
    all_masks = all_masks.to('cpu')  # Memindahkan tensor ke CPU
    mask_np = all_masks.numpy().astype('uint8')
    mask_np = cv2.resize(mask_np, (512, 256))
    # mask_np = cv2.resize(mask_np, (480, 240))

    return mask_np

def warping(frame, points, w, h, inv=False):
    points1 = np.float32(points)
    points2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(points2, points1)
    else:
        matrix = cv2.getPerspectiveTransform(points1, points2)

    frameWarp = cv2.warpPerspective(frame, matrix, (w, h))
    return frameWarp

def nothing(a):
    pass

def initializeTrackbars(intialTracbarVals, wT=512, hT=256):
# def initializeTrackbars(intialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 512, 256)
    # cv2.resizeWindow("Trackbars", 480, 240)
    cv2.createTrackbar("Width Top", "Trackbars",
                       intialTracbarVals[0], wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars",
                       intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars",
                       intialTracbarVals[2], wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars",
                       intialTracbarVals[3], hT, nothing)

def valTrackbars(wT=512, hT=256):
# def valTrackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                         (widthBottom, heightBottom), (wT-widthBottom, heightBottom)])
    return points

def drawPoints(frame, points):
    for x in range(4):
        cv2.circle(frame, (int(points[x][0]), int(
            points[x][1])), 8, (0, 0, 255), cv2.FILLED)
    return frame
    
def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(
                        imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(
                        imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(
                        imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(
                    imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(
                    imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver

def isEndOfLane(results, model, frameWarp, display=True):
    global prev_error, start_detection_time, ujung_jalur_detected, robot_berbelok, robot_lurus, last_detected_movement, detected_jalur, detected_ujung_jalur, last_detection_time

    names = model.names
    detected_jalur = False
    detected_ujung_jalur = False

    current_time = time.time()
    fps = 1 / (current_time - isEndOfLane.last_time) if hasattr(isEndOfLane, 'last_time') else 0
    isEndOfLane.last_time = current_time
    
    org_fps = (300, 30)
    text_fps = f"FPS: {fps:.2f}"

    for r in results:
        for box in r.boxes:
            clss = names[int(box.cls)]
            confidence = box.conf
            if clss == 'track':
                detected_jalur = True
                write_to_csv(csv_filename_model, fieldnames, clss, confidence, fps, 0, 0)
            elif clss == 'end-track':
                detected_ujung_jalur = True
                write_to_csv(csv_filename_model, fieldnames, clss, confidence, fps, 0, 0)
    current_time = time.time()

    if detected_jalur and not detected_ujung_jalur:
        frameWarpCopy = frameWarp.copy() # Salin gambar yang sudah diproses (warp)
        frameWarpCopy = cv2.GaussianBlur(frameWarpCopy, (5, 5), 0) # Blur untuk mengurangi noise

        # Binerisasi gambar untuk mendeteksi piksel putih (jalur)
        _, thresholded = cv2.threshold(frameWarpCopy, 200, 255, cv2.THRESH_BINARY)

        # Area yang akan dianalisis (¼ bagian bawah gambar)
        bottom_area_start = 3 * frameWarpCopy.shape[0] // 4
        bottom_area_end = frameWarpCopy.shape[0]
        bottom_area = thresholded[bottom_area_start:bottom_area_end, :]

        # Hitung total piksel putih (jalur) di area bawah gambar
        white_pixels = np.sum(bottom_area == 255)
        # white_pixels = np.where(thresholded[bottom_area_start:bottom_area_end, :] == 255)[1]

        # Buat titik referensi pada tengah jalur di area bawah
        M = cv2.moments(bottom_area)
        if M["m00"] != 0:
            cx = M["m10"] / M["m00"]  # Koordinat X dari tengah jalur pada area bawah
        else:
            # Default ke tengah jika tidak ada piksel putih
            cx = frameWarpCopy.shape[1] / 2  # Pastikan ini menggunakan pembagian floating point

        # Titik tengah gambar di bagian bawah, namun sejajar horizontal (Y sama dengan Y jalur)
        height, width = frameWarpCopy.shape[:2]
        x_center = width / 2  # Pastikan titik X dari tengah gambar juga menggunakan pembagian floating point
        y_center = bottom_area_start + (bottom_area_end - bottom_area_start) / 2  # Titik Y dari tengah jalur

        # Pastikan bahwa titik referensi pada jalur juga sejajar horizontal di Y yang sama
        cy = y_center  # Jadikan Y dari titik jalur sama dengan Y dari titik tengah gambar

        # Ukur jarak horizontal (error) antara titik tengah gambar dengan titik tengah jalur
        error = round(cx - x_center, 2)  # Hitung error dalam floating point dan bulatkan hingga 2 desimal
        print("Posisi Error = ", error)

        # Hitung delta_error
        if prev_error is not None:
            delta_error = round(error - prev_error, 2)
        else:
            delta_error = 0.0

        print("Delta Error: ", delta_error)

        # Simpan error saat ini untuk frame berikutnya
        prev_error = error

        # Kirim data jarak ke ESP32, periksa koneksi esp_ser agar tidak error
        if esp_ser and esp_ser.is_open:
            kirimData(esp_ser, error, None, None)

        # Tulis hasil ke CSV, pastikan data yang dikirim tidak menyebabkan error
        write_to_csv(csv_filename_general, fieldnames, clss, confidence, fps, error, delta_error)

        if robot_berbelok or robot_lurus:
            last_detected_movement = (robot_berbelok, robot_lurus)

        # Visualisasi hasil jika display diaktifkan
        if display:
            frameWarpCopy_bgr = cv2.cvtColor(frameWarpCopy, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(frameWarpCopy_bgr, (0, bottom_area_start), (width, bottom_area_end), (50, 50, 50), 2)
            cv2.circle(frameWarpCopy_bgr, (int(cx), int(cy)), 10, (0, 255, 0), -1)  # Titik referensi pada jalur (hijau)
            cv2.circle(frameWarpCopy_bgr, (int(x_center), int(y_center)), 10, (255, 0, 0), -1)  # Titik tengah gambar (biru)
            cv2.line(frameWarpCopy_bgr, (int(cx), int(cy)), (int(x_center), int(y_center)), (0, 0, 255), 2)  # Garis penghubung (merah)

            if -15 <= error <= 15:
                text = "Maju Lurus"
                robot_lurus = "Lurus"
            elif -30 <= error < -15:
                text = "Belok Kiri"
            elif 15 < error <= 30:
                text = "Belok Kanan"
            elif error < -30:
                text = "Kiri Tajam"
                robot_berbelok = "Belok Kiri"
            elif error > 30:
                text = "Kanan Tajam"
                robot_berbelok = "Belok Kanan"

            # Tambahkan teks visualisasi ke frame
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frameWarpCopy_bgr, text, (10, 30), font, 1, (255, 0, 255), 2)
            cv2.putText(frameWarpCopy_bgr, f"Error: {error:.2f} px", (10, 60), font, 1, (255, 0, 255), 2)
            cv2.putText(frameWarpCopy_bgr, f"FPS: {fps:.2f}", (10, 90), font, 1, (255, 0, 255), 2)

            if robot_berbelok and robot_lurus:
                    print("HISTORY: ['{}', '{}']".format(robot_berbelok, robot_lurus))
        return frameWarpCopy_bgr

    elif detected_jalur and detected_ujung_jalur:
        print('track dan end-tarck')
        Ujung_Jalur = "MAJU\n"
        kirimData(esp_ser, None, None, Ujung_Jalur)

        start_detection_time = None
        ujung_jalur_detected = False
        last_detection_time = None

        write_to_csv(csv_filename_general, fieldnames, clss, confidence, fps, 0, 0)

        return

    elif detected_ujung_jalur and not detected_jalur:
        write_to_csv(csv_filename_general, fieldnames, clss, confidence, fps, 0, 0)
        
        if not ujung_jalur_detected:
            start_detection_time = current_time
            ujung_jalur_detected = True
            last_detection_time = current_time
            print("Deteksi awal ujung jalur, start_detection_time diatur ke:", start_detection_time)
        else:
            # Jika deteksi ujung jalur telah hilang sebelumnya, reset waktunya
            if last_detection_time and (current_time - last_detection_time > 0.5):  # Anggap 0.5 detik sebagai durasi untuk reset deteksi
                start_detection_time = current_time
                print("Deteksi ujung jalur di-reset, start_detection_time diatur ke:", start_detection_time)

            last_detection_time = current_time

            Ujung_Jalur = "MAJU\n"
            kirimData(esp_ser, None, None, Ujung_Jalur)
            selisih_waktu = current_time - start_detection_time
            print("Selisih waktu: ", selisih_waktu)
            if selisih_waktu > 4:
                if last_detected_movement:
                    print("Info 2 last movement: ['{}', '{}']".format(last_detected_movement[0], last_detected_movement[1]))
                    kirimData(esp_ser, None, last_detected_movement, None)

                    robot_berbelok = None
                    robot_lurus = None
                    last_detected_movement = None
                    ujung_jalur_detected = False
                    start_detection_time = None
                    last_detection_time = None
            else:
                print("Selisih waktu belum mencapai 4 detik. Tidak mengirim data last detected movement.")

def kirimData(esp_ser, error, last_detected_movement, Ujung_Jalur):
    global detected_jalur, detected_ujung_jalur
    
    # Prioritas pertama: kirim Ujung_Jalur jika tersedia
    if Ujung_Jalur:
        data_to_send = Ujung_Jalur
        print("Mengirim data ke ESP32: {}".format(data_to_send))
        if esp_ser.is_open:
            esp_ser.write(data_to_send.encode())
            print("Data berhasil dikirim ke ESP32.")
        else:
            print("Gagal mengirim data. Port serial tidak terbuka.")
        return

    # Prioritas kedua: kirim last_detected_movement jika tersedia
    if last_detected_movement:
        if last_detected_movement[0] == "Belok Kanan" and last_detected_movement[1] == "Lurus":
            data_to_send = "KIRI\n"
        elif last_detected_movement[0] == "Belok Kiri" and last_detected_movement[1] == "Lurus":
            data_to_send = "KANAN\n"
        else:
            data_to_send = None

        if data_to_send:
            print("Mengirim data ke ESP32: {}".format(data_to_send))
            if esp_ser.is_open:
                esp_ser.write(data_to_send.encode())
                print("Data berhasil dikirim ke ESP32.")
            else:
                print("Gagal mengirim data. Port serial tidak terbuka.")
            return

    # Prioritas terakhir: kirim error jika tidak ada Ujung_Jalur dan last_detected_movement yang valid
    if not Ujung_Jalur and not last_detected_movement and detected_jalur and not detected_ujung_jalur:
        data_to_send = "error_{}\n".format(error)
        print("Mengirim data ke ESP32: {}".format(data_to_send))
        if esp_ser.is_open:
            esp_ser.write(data_to_send.encode())
            print("Data berhasil dikirim ke ESP32.")
        else:
            print("Gagal mengirim data. Port serial tidak terbuka.")