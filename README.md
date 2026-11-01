# Delta2D_V009-1 — Kết nối, giải mã UART và hiển thị radar 360°

> **Tài liệu thử nghiệm thực tế (01/11/2026)** · Ngôn ngữ: Tiếng Việt  
> **Phần cứng:** module LiDAR 2D có PCB in `Delta2D_V009-1` · **Vi điều khiển thử nghiệm:** ESP8266 D1 Mini  
> **Kết quả:** đọc UART 115200, giải mã dữ liệu đo bằng `kaiaai/LDS`, hiển thị radar 360° và nhận thấy vật cản thay đổi vị trí khi chuyển từ khoảng 20 cm sang 40 cm.

## 1. Giới thiệu

Tài liệu này ghi lại quá trình đưa một module LiDAR tháo từ thiết bị khác, trên PCB có ký hiệu **`Delta2D_V009-1`**, hoạt động với ESP8266. Module có đầu nối **5 chân**: hai chân motor, hai chân cấp nguồn cho mạch cảm biến và một chân UART TX.

**Phát hiện quan trọng:** mặc dù tên PCB có chữ `Delta2D`, dữ liệu UART thu từ **module được thử nghiệm** sử dụng **protocol version `0x11`** và được thư viện [kaiaai/LDS](https://github.com/kaiaai/LDS) giải mã thành công bằng **`LDS_DELTA_2C_115200`**. Driver `LDS_DELTA_2D` thông thường không đọc được dữ liệu đo của module này.

Đây là nhận dạng **theo giao thức quan sát được**, **không phải** xác nhận rằng mọi module có chữ `Delta2D` đều tương đương Delta-2C PRO, hoặc thông số điện của chúng hoàn toàn giống nhau.


![Alt text]([image_url](https://github.com/pmkhue/LiDAR-Delta-2D-Delta2D_V009-1/blob/main/Test%20distance%20.png))

![Alt text]([image_url](https://github.com/pmkhue/LiDAR-Delta-2D-Delta2D_V009-1/blob/main/Test%20distance%202.png))


### Trạng thái đã kiểm chứng

| Hạng mục | Kết quả trên module thử nghiệm |
|---|---|
| UART | **115200 baud, 8N1** |
| Chân UART TX | Đã xác định và đọc được dữ liệu trên D7/GPIO13 của ESP8266 |
| Protocol version | **`0x11`** |
| Driver tương thích | **`LDS_DELTA_2C_115200`** trong `kaiaai/LDS` |
| Điện áp mạch đo đã chạy thành công | **5 V** trên chân VCC của module thử nghiệm |
| Điện áp motor đã thử | **3 V DC**, nguồn riêng |
| Tốc độ quét đo được | **~7,45–7,60 vòng/giây (Hz)** |
| Tốc độ dữ liệu UART | **~11,1–11,3 kB/s** |
| Tốc độ lấy mẫu thô | **~3.000 mẫu/giây** |
| Điểm đo hợp lệ | Khoảng **1.600–1.700 điểm/giây** trong các lần đo tốt |
| Phạm vi khoảng cách dương đã thấy | **~140–4.000 mm** (không phải thông số tầm đo được nhà sản xuất bảo đảm) |
| Radar web | Đã hoạt động trên điện thoại qua Wi-Fi AP |

> **Giới hạn:** chưa có datasheet đúng chính xác mã PCB `Delta2D_V009-1`; sơ đồ chân và điện áp dưới đây là kết quả thử trên **một module**, không được xem là pinout chung cho mọi biến thể.

---

## 2. Pinout 5 chân — đã dò trên module thử nghiệm

Các số chân dưới đây là **số thứ tự được đánh khi dò dây**, không phải vị trí vật lý tính từ một chiều đầu cắm chuẩn do nhà sản xuất công bố. **Đừng dựa vào số chân để cắm đầu nối khác nếu chưa xác định hướng nhìn đầu cắm và đo lại bằng đồng hồ.**

| Chân theo quá trình dò | Tín hiệu | Kết nối trong bài thử | Ghi chú |
|---|---|---|---|
| 1 | `M+` | Dương nguồn motor 3 V | Không nối GPIO trực tiếp |
| 2 | `M−` | Âm nguồn motor | Nguồn motor riêng |
| 3 | `VCC` | Nguồn DC **5 V** ổn áp | Đã giúp khối đo trả về khoảng cách |
| 4 | `GND` | GND chung | Chung mốc điện thế giữa các nguồn và ESP8266 |
| 5 | `TX` | **D7 / GPIO13** của D1 Mini | **Chỉ nối khi mức TX an toàn cho 3,3 V** |

### Sơ đồ đấu dây nguyên lý

```text
         LiDAR Delta2D_V009-1                  ESP8266 D1 Mini
         (module đã thử nghiệm)
        +---------------------+
   1 M+ |                     | <--- +3 V nguồn motor riêng
   2 M- |                     | <--- 0 V nguồn motor
   3 VCC|                     | <--- +5 V nguồn cảm biến ổn áp
   4 GND|                     | ----- GND chung -------- GND
   5 TX |                     | ----- [đổi mức nếu cần] --> D7 / GPIO13 (RX0)
        +---------------------+

  Lưu ý: GND nguồn motor, nguồn cảm biến và ESP8266 phải có mốc chung phù hợp.
           Không đưa 5 V trực tiếp vào GPIO13 của ESP8266.
```

**Cảnh báo điện áp:** cấp **5 V cho VCC của đúng module đã thử nghiệm** đã cho kết quả tốt, nhưng chưa xác nhận chân TX xuất mức logic 3,3 V hay 5 V. Hãy đo TX, hoặc sử dụng chuyển mức phù hợp, **trước khi kết nối với ESP8266**. Cắt nguồn trước khi đổi dây. Không nối nhầm nguồn motor 3 V vào VCC cảm biến và ngược lại.

### Vì sao dùng hai nguồn?

Trong quá trình thử, phần motor quay được khi cấp khoảng **3 V** (quan sát ban đầu khoảng **40 mA**); phần điện tử đo khoảng cách cần nguồn cao hơn trên chính mẫu thử. Nên sử dụng nguồn ổn áp và có giới hạn dòng khi khảo sát phần cứng không rõ nguồn gốc.

---

## 3. Kết nối UART trên ESP8266 D1 Mini

LiDAR phát dữ liệu liên tục với tốc độ gần chạm khả năng truyền thực tế của UART 115200 bps. Dùng `SoftwareSerial` cùng Wi-Fi trên ESP8266 từng gây mất byte, gói không hợp lệ và **Software WDT reset**.

Cấu hình hoạt động ổn định là **UART0 phần cứng** với `Serial.swap()`:

```cpp
// ESP8266 Arduino Core
Serial.begin(115200);
Serial.setDebugOutput(false);
Serial.setRxBufferSize(2048);
Serial.swap();

// Sau swap():
// UART0 RX = GPIO13 / D7  <-- TX của LiDAR (mức logic <= 3,3 V)
// UART0 TX = GPIO15 / D8  -- ĐỂ TRỐNG, không nối LiDAR
```

Lưu ý:

- `Serial.swap()` chuyển RX0 sang **D7/GPIO13** và TX0 sang **D8/GPIO15**. Không dùng D8 làm chân điều khiển tuỳ tiện: GPIO15 còn là chân liên quan đến chế độ khởi động của ESP8266.
- UART0 sau khi `swap()` không còn hoạt động như Serial Monitor USB ở các chân mặc định; muốn debug nên dùng log qua mạng hoặc một đường debug riêng.
- Phần mềm đọc UART phải xử lý nhanh và nhường thời gian cho Wi-Fi/watchdog (`delay(1)` hoặc cơ chế tương đương).
- Theo cấu hình này, chỉ cần **TX của LiDAR → RX của ESP8266**; không cần gửi lệnh tới chân RX của module vì đầu nối được dò không có chân RX.

---

## 4. Giao thức: vì sao không dùng driver `LDS_DELTA_2D`?

### 4.1. Dữ liệu thô đã bắt được

Khi motor quay ở vùng tốc độ thích hợp, module phát gói tương tự:

```text
AA 00 5D 11 61 AD 00 55 97 FF E4 4F 63 57 E4 ...
```

Giải thích phần đầu theo định dạng Delta-2C PRO tương thích `0x11`:

| Byte/vùng | Ý nghĩa | Giá trị ví dụ |
|---|---|---|
| `AA` | Byte đồng bộ | `0xAA` |
| `00 5D` | Độ dài phần gói **không kể 2 byte checksum** | `0x005D = 93` |
| `11` | Phiên bản giao thức | **`0x11`** |
| `61` | Loại dữ liệu chính | `0x61` |
| `AD` | Gói dữ liệu đo | `0xAD` |
| `00 55` | Độ dài trường dữ liệu | 85 byte |
| `97` | Tần số quay (giá trị / 20) | `0x97 / 20 = 7,55 Hz` |
| `FF E4` | Offset góc có dấu | Theo định dạng SDK |
| `4F 63` | Góc bắt đầu, đơn vị 0,01° | `203,23°` |
| `57 E4` | Góc kết thúc, đơn vị 0,01° | `225,00°` |

Ở ví dụ này, một gói có **95 byte trên đường truyền**: 93 byte nội dung được tính trong trường length và 2 byte checksum. Phần dữ liệu mẫu đo có **26 mẫu** vì `(85 - 7) / 3 = 26`.

Mỗi mẫu có cấu trúc:

```text
[quality: 1 byte] [distance_hi: 1 byte] [distance_lo: 1 byte]
```

Theo tài liệu giao thức Delta-2C PRO tham chiếu:

```text
distance_mm = ((distance_hi << 8) | distance_lo) * 0.25
scan_frequency_hz = frequency_byte / 20
```

Góc của các mẫu được nội suy giữa `start_angle` và `end_angle` bởi driver. Để ứng dụng thực tế, **ưu tiên dữ liệu góc/khoảng cách đã giải mã từ callback SDK**, thay vì tái hiện parser thủ công từ bảng này.

### 4.2. Gói tốc độ `0xAE`

Khi thử quay thủ công, cũng đã quan sát gói ngắn:

```text
AA 00 09 11 61 AE 00 01 00 01 D4
```

`0xAE` là loại gói báo trạng thái/tốc độ, **không chứa danh sách khoảng cách**. Gói đo dùng `0xAD`. Cả hai loại đều thuộc luồng giao thức phiên bản `0x11` quan sát được.

### 4.3. Chọn đúng thư viện

| Driver | Kết quả thử trên module này |
|---|---|
| `LDS_DELTA_2D` | UART có dữ liệu nhưng **0 gói đo, 0 điểm**; bộ đếm lỗi tăng |
| **`LDS_DELTA_2C_115200`** | **Giải mã thành công**, nhận góc, khoảng cách, chất lượng và vòng quét |

Tài liệu chính thức của thư viện [kaiaai/LDS](https://github.com/kaiaai/LDS) mô tả driver `LDS_DELTA_2C_115200` hỗ trợ **`0x11`/`0x13`**, có trường `end_angle` bổ sung, và quy đổi khoảng cách **0,25 mm/đơn vị**. Đây là kết quả cộng đồng công bố cho họ Delta-2C PRO; việc áp dụng lên `Delta2D_V009-1` là kết quả kiểm chứng trên module đang mô tả.

---

## 5. Thử nghiệm nguồn: 3,3 V so với 5 V

**Đây là bài học quan trọng nhất của quá trình debug.**

| Điều kiện thử | Tốc độ quét | Kết quả |
|---|---:|---|
| VCC cảm biến **3,3 V**; motor **3 V** | ~7,60 Hz | SDK nhận gói không lỗi, nhưng **48.434/48.434 mẫu có khoảng cách bằng 0** (`zero_pct = 100%`) |
| VCC cảm biến **5 V**; motor **3 V** | ~7,50 Hz | Xuất hiện **13.884 điểm có khoảng cách khác 0** trong lần thử đầu; các lần sau cho kết quả tốt hơn |
| VCC cảm biến **5 V**; motor **3 V**, thử ổn định | ~7,45–7,55 Hz | Khoảng **1.600–1.700 điểm hợp lệ/s** trong môi trường thử |

Như vậy, **UART và motor quay bình thường không có nghĩa là khối đo khoảng cách đã được cấp nguồn đúng**. Trên **mẫu LiDAR này**, đổi VCC sang 5 V làm kết quả thay đổi rõ rệt. Đây không phải bằng chứng rằng mọi module nhãn Delta2D đều chịu được 5 V: cần xác minh trước khi áp dụng cho bản khác.

---

## 6. Log đo thực tế

Ba ảnh chụp log sau cùng (giữ LiDAR trong cùng môi trường, tấm bìa được di chuyển) cho các giá trị:

| Thông số | Mốc 38 giây | Mốc 68 giây | Mốc 88 giây |
|---|---:|---:|---:|
| Góc/quét, `scan_hz` | 7,45 Hz | 7,50 Hz | 7,55 Hz |
| Tổng gói, `packets` | 4.404 | 7.980 | 10.413 |
| Tổng mẫu, `raw_points` | 111.035 | 201.014 | 262.434 |
| Điểm có khoảng cách, `points` | 60.918 | 107.030 | 138.822 |
| Điểm bằng 0, `zero_distances` | 50.117 | 93.984 | 123.612 |
| Tỷ lệ 0 cộng dồn, `zero_pct` | 45,14% | 46,75% | 47,10% |
| Tốc độ điểm thô, `raw_pps` | 3.027/s | 3.038/s | 3.040/s |
| Tốc độ điểm hợp lệ, `valid_pps` | 1.612/s | 1.713/s | 1.642/s |
| Lỗi checksum | 0 | 0 | 0 |

Các mốc trên là **số liệu thực nghiệm**, không phải giá trị hiệu năng cam kết của nhà sản xuất. Riêng lần đo 88 giây có 1 lỗi `invalid_value_errors` (`last_error = -12`) và **không có lỗi checksum**.

### Vì sao `zero_distances` liên tục tăng dù LiDAR đứng yên?

`zero_distances` trong firmware chẩn đoán là **bộ đếm tích lũy từ lúc khởi động**. LiDAR vẫn quét hàng nghìn điểm mỗi giây ngay cả khi đứng yên, nên số đếm tiếp tục tăng. Một số góc không có phản hồi hợp lệ do bề mặt, hướng phản xạ, vật xa hoặc điều kiện đo.

Tỷ lệ **theo từng khoảng thời gian** đáng xem hơn tổng số cộng dồn:

| Khoảng thời gian | Mẫu mới | Điểm bằng 0 mới | Tỷ lệ zero trong khoảng |
|---|---:|---:|---:|
| 38 → 68 giây | 89.979 | 43.867 | ~48,75% |
| 68 → 88 giây | 61.420 | 29.628 | ~48,24% |

Chênh lệch chỉ khoảng **0,51 điểm phần trăm**, cho thấy tỷ lệ phản hồi tương đối ổn định trong hai khoảng quan sát.

### Lưu ý về `last_distance`

`last_distance` chỉ là **khoảng cách của điểm cuối cùng** mà SDK vừa xử lý, ở một góc bất kỳ. Nó **không** tự động là khoảng cách tới tấm bìa đang thử. Để đo tấm bìa, cần chọn **một vùng góc cố định** có vật và lấy trung vị các điểm hợp lệ trong vùng đó.

---

## 7. Radar 360° trên ESP8266 D1 Mini

Firmware thử nghiệm **Delta2D Radar V7.1** (do người thực hiện thử nghiệm xây dựng; **không thuộc thư viện kaiaai/LDS**) sử dụng:

- UART0 phần cứng trên **D7/GPIO13** (`Serial.swap()`).
- Driver **`LDS_DELTA_2C_115200`**.
- Bộ đệm **360 ô góc**, mỗi ô tương ứng 1°; dữ liệu hiển thị lấy từ **vòng quét đã hoàn tất gần nhất**.
- Radar web hiển thị điểm xanh theo góc/khoảng cách.
- Vùng chọn màu cam để đọc **trung vị khoảng cách**, min/max và chất lượng trung bình.
- Thống kê `zero %` **theo vòng quét**, không nhầm với bộ đếm cộng dồn.
- Xuất CSV một vòng quét từ trình duyệt.

**Wi-Fi điểm truy cập** dùng trong bài thử:

```text
SSID:      Delta2D-Radar
Password:  12345678
URL:       http://192.168.4.1
```

Điện thoại có thể báo **“No internet connection”**; đây là hiện tượng bình thường vì ESP8266 tạo mạng nội bộ, không cung cấp Internet. Mật khẩu trên là mật khẩu demo, **nên đổi trước khi triển khai bên ngoài môi trường thử nghiệm**.

### Các API của firmware V7.1

| Endpoint | Chức năng |
|---|---|
| `/api/status` | UART B/s, quét Hz, số mẫu hợp lệ, tỷ lệ zero theo vòng, RAM, lỗi... |
| `/api/scan` | `{ "seq": ..., "points": [360 giá trị khoảng cách mm] }` |
| `/api/sector?angle=90&half=8` | Trung vị/min/max khoảng cách trong vùng 90° ± 8°, số ô có tín hiệu, chất lượng... |

Ví dụ lấy dữ liệu (sau khi máy tính đã kết nối Wi-Fi của ESP8266):

```bash
curl http://192.168.4.1/api/status
curl http://192.168.4.1/api/scan
curl 'http://192.168.4.1/api/sector?angle=90&half=8'
```

> **Lưu ý khi dùng source V7.1:** dự án Arduino gồm **hai file cần đặt cùng thư mục**, `Delta2D_Radar_V7_1.ino` và `SectorResult.h`. Nếu chỉ copy `.ino` vào sketch mới, Arduino IDE có thể báo `SectorResult does not name a type`.
>
> **Tài liệu Markdown này không đóng gói sẵn firmware.** Nếu công bố code, hãy upload hai file đó riêng và tôn trọng giấy phép của thư viện LDS.

### Kiểm tra khoảng cách bằng radar

1. Đặt LiDAR cố định; xác định hướng `0°` trong phần mềm (**chưa phải hướng trước của robot đã hiệu chuẩn**).
2. Chọn thang radar **0,5 m** khi thử gần, hoặc **2–4 m** khi quan sát cả phòng.
3. Đưa tấm bìa có bề mặt phản xạ tốt vào một hướng cố định ở **20 cm**, rồi **40 cm**.
4. Chọn **góc tâm** sao cho vùng góc bao trọn cụm điểm của tấm bìa.
5. Đọc **trung vị vùng góc**, giữ nguyên cấu hình góc khi dịch chuyển vật.
6. Nếu cần kiểm tra độ chính xác, đo bằng thước và so sánh nhiều lần; hai ảnh radar hiện có mới xác nhận **xu hướng vị trí điểm thay đổi**, chưa đủ để tuyên bố sai số tuyệt đối.

Trong hai ảnh thử gần, radar báo khoảng **221–229 điểm hợp lệ mỗi vòng**, `zero` theo vòng khoảng **44–45%**, và tần số **7,55 Hz**. Đây là các giá trị hiển thị tại thời điểm chụp.



---

## 8. Các lỗi đã gặp và cách xử lý

| Hiện tượng | Nguyên nhân phát hiện / hướng xử lý |
|---|---|
| Không nhận byte UART | Đọc nhầm chân `RX0/GPIO3` trong khi TX LiDAR đang nối D5. Kiểm tra lại chân nhận thực tế. |
| `Software WDT reset`, dữ liệu lỗi | `SoftwareSerial` 115200 cùng Wi-Fi quá tải trên ESP8266. Đổi sang **UART0 phần cứng + `Serial.swap()` + D7**. |
| UART ~11 kB/s nhưng `packets = 0`, `points = 0` | Chọn sai driver `LDS_DELTA_2D`. Với **luồng `0x11` đã quan sát**, dùng `LDS_DELTA_2C_115200`. |
| `packets` tăng, checksum tốt nhưng 100% khoảng cách bằng 0 | Trên mẫu thử, VCC **3,3 V** không đủ cho phần đo. Kiểm tra nguồn theo đúng phần cứng; mẫu đã chạy được khi dùng **5 V VCC**, motor 3 V. |
| `zero_distances` tăng liên tục | Đó là bộ đếm cộng dồn. Xem `zero %` **theo vòng** hoặc trong một khoảng thời gian. |
| Khoảng cách cuối cùng không bằng khoảng cách tấm bìa | `last_distance` thuộc góc đo cuối cùng. Dùng trung vị theo vùng góc có tấm bìa. |
| Arduino: `SectorResult does not name a type` | Thiếu file `SectorResult.h` hoặc lỗi tự sinh prototype của Arduino. Đặt `.ino` và `.h` cùng thư mục. |
| Điện thoại báo không có Internet | Bình thường: Wi-Fi ESP8266 là mạng AP cục bộ, chỉ phục vụ radar. |




## 10. Tài liệu và nguồn tham khảo

1. **kaiaai/LDS — Arduino LiDAR library:** <https://github.com/kaiaai/LDS>  
   Tài liệu driver `LDS_DELTA_2C_115200` cho protocol `0x11`/`0x13`, cùng ví dụ sử dụng Arduino.
2. **DELTA-2C PRO reverse-engineering toolkit:** <https://github.com/hapynfdj/delta2c-lidar>  
   Nguồn tham khảo phần cứng, khung dữ liệu và công cụ chẩn đoán của biến thể tương thích.
3. **Phân tích giao thức chi tiết:** <https://github.com/hapynfdj/delta2c-lidar/blob/master/PROTOCOL.md>  
   Cấu trúc khung `0xAD`/`0xAE`, trường góc, quality, checksum và quy đổi khoảng cách.
4. **Log và ảnh radar thực tế:** dữ liệu thu trực tiếp trên **một module PCB `Delta2D_V009-1`**, ESP8266 D1 Mini, trong quá trình thử nghiệm ngày **01/11/2026**.

### Ghi nhận / Attribution

Bộ giải mã LiDAR dựa trên công sức của các tác giả và cộng tác viên dự án **[kaiaai/LDS](https://github.com/kaiaai/LDS)** và nhóm nghiên cứu giao thức **[delta2c-lidar](https://github.com/hapynfdj/delta2c-lidar)**. Tài liệu này tổng hợp **kinh nghiệm thử nghiệm một phần cứng cụ thể**, không tuyên bố sở hữu SDK, không phải datasheet chính thức và không thay thế hướng dẫn an toàn điện.

---

**Kết luận:** với module có PCB `Delta2D_V009-1` đã thử nghiệm, tổ hợp **VCC cảm biến 5 V + motor 3 V + UART phần cứng 115200 + driver `LDS_DELTA_2C_115200`** đã giải mã được dữ liệu radar 360°. Phát hiện quan trọng nhất để tái sử dụng là **nhìn vào `protocol version` thực tế (`0x11`) thay vì chỉ lựa chọn SDK theo tên in trên PCB**.
