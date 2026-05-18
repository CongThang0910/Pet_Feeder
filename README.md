# Pet Feeder - ESP32 Smart Feeding System

Hệ thống cho thú cưng ăn tự động sử dụng ESP32, hỗ trợ điều khiển và giám sát qua Web Server.  
Dự án tích hợp Loadcell để định lượng thức ăn chính xác, RTC DS3231 để hẹn giờ và EEPROM để lưu dữ liệu ngay cả khi mất điện.

---

# Tính năng chính

- Cho ăn tự động theo lịch hẹn
- Cho ăn thủ công bằng nút bấm
- Cho ăn tự động khi cảm biến IR phát hiện thú cưng trong 5 giây
- Điều khiển và giám sát qua giao diện Web
- Định lượng thức ăn bằng Loadcell + HX711
- Đồng bộ thời gian thực bằng RTC DS3231
- Lưu dữ liệu bằng EEPROM
- Logic chặn cho ăn trùng lịch
- Hệ thống hoạt động non-blocking, không dùng delay() trong loop()

---

# Phần cứng sử dụng

| Thiết bị | Chức năng |
|---|---|
| ESP32 | Vi điều khiển trung tâm |
| Servo MG90S | Đóng/mở cơ cấu cấp thức ăn |
| Loadcell + HX711 | Cân thức ăn |
| RTC DS3231 | Thời gian thực |
| LCD I2C 16x2 | Hiển thị trạng thái |
| IR Sensor | Phát hiện thú cưng |
| Push Button | Cho ăn thủ công |

---

# Sơ đồ chân kết nối

| Thiết bị | GPIO ESP32 |
|---|---|
| Servo | GPIO 13 |
| HX711 DT | GPIO 4 |
| HX711 SCK | GPIO 5 |
| IR Sensor | GPIO 15 |
| Button | GPIO 16 |
| LCD SDA | GPIO 21 |
| LCD SCL | GPIO 22 |

---

# Cơ chế hoạt động

## 1. Cho ăn theo lịch

Người dùng cài đặt:
- giờ ăn
- số lần ăn
- khối lượng từng bữa

Khi đến giờ:
- Servo mở
- Thức ăn được đổ xuống khay
- Loadcell đo khối lượng liên tục
- Đủ gram -> Servo đóng

---

## 2. Logic chặn thông minh

Trong khoảng:
- 20 phút trước giờ hẹn

Nếu:
- cảm biến IR kích hoạt
- hoặc nhấn nút cho ăn

thì hệ thống sẽ:
- tính đó là lần ăn của lịch hẹn
- không cho ăn lại khi đến đúng giờ

Mục đích:
- tránh thú cưng ăn trùng bữa



```cpp
delay()
