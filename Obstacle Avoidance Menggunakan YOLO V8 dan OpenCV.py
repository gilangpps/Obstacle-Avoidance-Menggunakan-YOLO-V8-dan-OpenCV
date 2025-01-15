import cv2
import serial
import time
import torch
from ultralytics import YOLO
from cvzone.HandTrackingModule import HandDetector
import cvzone

# Inisialisasi komunikasi serial dengan Arduino
try:
    arduino = serial.Serial('COM15', 9600, timeout=1)  # Ganti 'COM15' sesuai dengan port Arduino Anda
    time.sleep(2)  # Tunggu Arduino siap
except Exception as e:
    print(f"Error initializing serial connection: {e}")
    exit()

# Load model YOLOv8
model = YOLO('yolov8n.pt')

# Perangkat untuk menjalankan model (GPU atau CPU)
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)

# Daftar target objek yang ingin dideteksi
TARGET_CLASSES = ["bottle", "cup", "book"]

# Setup kamera
camera = cv2.VideoCapture(0)  # Ganti '0' dengan ID kamera Anda jika lebih dari satu kamera
camera.set(3, 640)  # Lebar resolusi kamera
camera.set(4, 480)  # Tinggi resolusi kamera

# Inisialisasi HandDetector
hand_detector = HandDetector(detectionCon=0.8, maxHands=1)  # Deteksi 1 tangan dengan confidence 0.8

# Pusat layar untuk resolusi 640x480
screen_center_x = 320
screen_center_y = 240

# Variabel untuk melacak frame yang dilewati (pengurangan beban)
frame_skip = 6  # Proses setiap 6 frame
frame_count = 0

# Fungsi untuk mengirim perintah ke Arduino
def send_command(command):
    try:
        arduino.write(command.encode())  # Kirim perintah dalam bentuk byte
    except Exception as e:
        print(f"Error sending command to Arduino: {e}")

# Fungsi kontrol motor
def move_forward():
    send_command('F')

def stop():
    send_command('S')

def turn_left():
    send_command('L')

def turn_right():
    send_command('R')

# Loop utama
try:
    while True:
        success, img = camera.read()
        if not success:
            print("Error: Unable to read from camera.")
            break

        frame_count += 1
        if frame_count % frame_skip != 0:
            continue  # Lewati frame untuk mengurangi beban pemrosesan

        # Deteksi tangan menggunakan HandDetector
        hands, img = hand_detector.findHands(img, flipType=False)  # Deteksi tangan dalam frame

        # Jika tangan terdeteksi, kendalikan robot berdasarkan posisi tangan
        if hands:
            hand = hands[0]  # Ambil tangan pertama
            lmList = hand['lmList']  # Landmark tangan
            center_x, center_y = hand['center']  # Pusat tangan

            # Tentukan gerakan berdasarkan posisi tangan relatif ke pusat layar
            if center_x < screen_center_x - 100:  # Tangan di kiri layar
                turn_left()
            elif center_x > screen_center_x + 100:  # Tangan di kanan layar
                turn_right()
            else:  # Tangan di tengah layar
                move_forward()

        else:
            # Jika tidak ada tangan, lanjutkan dengan deteksi objek YOLO
            results = model(img, show=False)
            object_detected = False  # Flag jika objek terdeteksi

            for result in results[0].boxes:
                box = result.xyxy[0].numpy()  # Bounding box [x1, y1, x2, y2]
                conf = result.conf[0].item()  # Confidence
                cls_id = int(result.cls[0].item())  # Class ID
                class_name = model.names[cls_id]  # Nama kelas

                # Filter hanya target kelas
                if class_name in TARGET_CLASSES and conf > 0.5:
                    object_detected = True  # Objek terdeteksi

                    x1, y1, x2, y2 = map(int, box)  # Koordinat bounding box
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    # Hitung jarak Z (dummy calculation, sesuaikan dengan kebutuhan)
                    distance_virtual = x2 - x1
                    if distance_virtual > 150:
                        Z_real = (-0.125 * distance_virtual) + 44.125
                    elif 100 < distance_virtual <= 150:
                        Z_real = (-0.217391 * distance_virtual) + 58.260869
                    else:
                        Z_real = 100  # Default jika di luar range

                    # Hitung X relatif ke pusat layar
                    X_virtual = center_x - screen_center_x

                    # Tampilkan bounding box dan info
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cvzone.putTextRect(
                        img,
                        f'{class_name} {int(Z_real)}cm',
                        (x1, y1 - 10),
                        scale=1,
                        colorR=(0, 255, 0)
                    )

                    # Kontrol motor berdasarkan posisi dan jarak
                    if Z_real < 30:  # Jika objek dekat
                        if X_virtual > 0:  # Objek di kanan
                            turn_right()
                        else:  # Objek di kiri
                            turn_left()
                    else:  # Jika objek jauh
                        move_forward()

            # Jika tidak ada objek, lanjut maju
            if not object_detected:
                move_forward()

        # Tampilkan frame
        cv2.imshow("YOLOv8 Object & Hand Tracking", img)

        # Keluar jika tombol 'q' ditekan
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Bersihkan sumber daya
    camera.release()
    cv2.destroyAllWindows()
    arduino.close()
