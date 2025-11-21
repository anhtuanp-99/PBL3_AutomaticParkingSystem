# Tên file: server.py
from flask import Flask, request
import time
import os
import sys
from pathlib import Path
import cv2
import numpy as np
import torch
import easyocr
import firebase_admin
from firebase_admin import credentials, db
import logging

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
# === 1. CẤU HÌNH ĐƯỜNG DẪN (QUAN TRỌNG) ===
YOLO_ROOT = r"D:\PBL3\CameraWebServer1\yolov5" # Đường dẫn gốc đến thư mục yolov5
if YOLO_ROOT not in sys.path:
    sys.path.insert(0, YOLO_ROOT) # Chèn vào đầu để ưu tiên

# === 2. "SIÊU FIX" LỖI UNPICKLE (AttributeError / ModuleNotFoundError) ===
import types # Thêm import này

# Tạo một package 'yolov5' giả
if 'yolov5' not in sys.modules:
    sys.modules['yolov5'] = types.ModuleType('yolov5')

# Import các module cục bộ thật sự
import models
import models.yolo
import models.common
import utils

# Gán ghép thủ công các module vào package giả
sys.modules['yolov5.models'] = models
sys.modules['yolov5.models.yolo'] = models.yolo
sys.modules['yolov5.models.common'] = models.common
sys.modules['yolov5.utils'] = utils

# Gán ghép class bị thiếu (nếu có)
try:
    from models.yolo import DetectionModel
    sys.modules['models.yolo'].DetectionModel = DetectionModel
    sys.modules['yolov5.models.yolo'].DetectionModel = DetectionModel
except ImportError:
    pass # Bỏ qua nếu không tìm thấy
# ====================================================================

# Import các module của YOLOv5 (SAU KHI ĐÃ FIX)
from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes # Đổi tên từ scale_coords
from utils.augmentations import letterbox
from utils.torch_utils import select_device

# Cấu hình Firebase & Model
SERVICE_ACCOUNT_KEY = 'serviceAccountKey.json'
DATABASE_URL = 'https://licenseplate-65834-default-rtdb.asia-southeast1.firebasedatabase.app/' # Sửa lại nếu cần
WEIGHTS_PATH = os.path.join(YOLO_ROOT, 'best.pt')
CONF_THRES = 0.4
IOU_THRES = 0.45
IMG_SIZE = 640

# Cấu hình thư mục lưu ảnh
IMAGE_SAVE_DIR = os.path.join(YOLO_ROOT, 'captured_images')
if not os.path.exists(IMAGE_SAVE_DIR):
    os.makedirs(IMAGE_SAVE_DIR)
# ==========================================

# --- Khởi tạo Firebase ---
try:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY)
    firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
    db_ref = db.reference('detections')
    print("✅ Kết nối Firebase thành công.")
except Exception as e:
    print(f"❌ Lỗi Firebase: {e}")
    db_ref = None

# --- KHỞI TẠO MODEL (CHỈ CHẠY 1 LẦN KHI START SERVER) ---
print("⏳ Đang tải model YOLOv5... (Vui lòng đợi)")
device = select_device('') # Tự động chọn CPU hoặc GPU
model = DetectMultiBackend(WEIGHTS_PATH, device=device, dnn=False, data=None, fp16=False)
stride, names, pt = model.stride, model.names, model.pt
print("✅ Model đã tải xong! Sẵn sàng xử lý.")

# --- Khởi tạo EasyOCR (Chỉ chạy 1 lần) ---
print("⏳ Đang tải EasyOCR...")
reader = easyocr.Reader(['en'], gpu=torch.cuda.is_available())
print("✅ EasyOCR sẵn sàng.")
# -------------------------------------------------------

app = Flask(__name__)

def process_image_in_memory(image_bytes):
    nparr = np.frombuffer(image_bytes, np.uint8)
    img0 = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    img = letterbox(img0, IMG_SIZE, stride=stride, auto=pt)[0]
    img = img.transpose((2, 0, 1))[::-1]
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.float() / 255.0
    if len(img.shape) == 3: img = img[None]

    pred = model(img, augment=False, visualize=False)
    pred = non_max_suppression(pred, CONF_THRES, IOU_THRES, classes=None, agnostic=False)

    detected_plate_text = None
    for i, det in enumerate(pred):
        if len(det):
            det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round() # Sửa tên hàm
            for *xyxy, conf, cls in reversed(det):
                # 🎯 THAY TÊN CLASS CỦA BẠN VÀO ĐÂY
                if names[int(cls)] == 'license_plate': 
                    x1, y1, x2, y2 = map(int, xyxy)
                    crop = img0[y1:y2, x1:x2]
                    
                    ocr_result = reader.readtext(crop)
                    text = "".join([res[1] for res in ocr_result])
                    text = "".join(filter(str.isalnum, text)).upper()
                    
                    if text:
                        print(f"🔍 Tìm thấy biển số: {text} (Độ tin cậy: {conf:.2f})")
                        detected_plate_text = text
                        return detected_plate_text

    return None

@app.route('/upload', methods=['POST'])
def upload_image():
    try:
        start_time = time.time()
        print("📸 Camera đang hoạt động...")
        image_data = request.data
        
        # 1. Xử lý trực tiếp trên RAM trước
        license_plate = process_image_in_memory(image_data)
        
        # 2. Chỉ khi tìm thấy biển số thì mới thực hiện các hành động tiếp theo
        if license_plate:
            print(f"🙂‍↔️ Tìm thấy biển số: {license_plate}")

            # === 💾 LƯU ẢNH (Chỉ chạy khi có biển số) ===
            # Mẹo: Đặt tên file kèm biển số luôn cho dễ tìm!
            filename = f"{license_plate}_{int(time.time())}.jpg" 
            filepath = os.path.join(IMAGE_SAVE_DIR, filename)
            
            with open(filepath, 'wb') as f:
                f.write(image_data)
            print(f"💾 Đã lưu thông tin biển số: {filepath}")
            # ===========================================

            # Gửi lên Firebase
            if db_ref:
                db_ref.push().set({
                    'plate': license_plate,
                    'timestamp': int(time.time() * 1000)
                })
                print(f"🥵 Đã gửi '{license_plate}' lên Firebase.")
        
        else:
            print("😭 Không tìm thấy biển số xe")
            pass 

        # print(f"⏱️ Xử lý xong: {time.time() - start_time:.2f}s")
        return "OK", 200

    except Exception as e:
        print(f"⛔️ Lỗi server: {e}")
        return "Error", 500
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)