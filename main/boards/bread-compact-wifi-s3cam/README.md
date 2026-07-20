硬件基于基于ESP32S3CAM开发板，代码基于bread-compact-wifi-lcd修改
使用的摄像头是OV2640
注意因为摄像头占用IO较多，所以占用了ESP32S3的USB 19 20两个引脚
连线方式参考config.h文件中对引脚的定义

 
# 编译配置命令

**配置编译目标为 ESP32S3：**

```bash
idf.py set-target esp32s3
```

**打开 menuconfig：**

```bash
idf.py menuconfig
```

**选择板子：**

```bash
Xiaozhi Assistant -> Board Type ->面包板新版接线（WiFi）+ LCD + Camera
```

**编译烧入：**

```bash
idf.py build flash
```

---

# Hướng dẫn cấu hình phát tín hiệu hồng ngoại (IR TX)

## 1. Quy ước mã hóa bit

Mỗi bit dữ liệu được mã hóa thành **1 cặp xung sáng/tắt** (carrier ON / carrier OFF) với thời gian cụ thể:

### Header (Xung mở đầu frame — cố định)

| Trạng thái | Thời gian | Ý nghĩa |
| :--- | :--- | :--- |
| LED sáng (`level = 1`) | `TX_HDR_L = 6245 µs` | Báo hiệu bắt đầu frame mới |
| LED tắt (`level = 0`) | `TX_HDR_H = 460 µs` | Khoảng nghỉ phân tách |

### Data Bits (8 bit mã lệnh)

| Giá trị bit | LED sáng (µs) | LED tắt (µs) | Đặc điểm nhận dạng |
| :--- | :--- | :--- | :--- |
| **Bit `0`** | `TX_BIT0_L = 1690` | `TX_BIT0_H = 570` | Sáng **lâu**, tắt **nhanh** |
| **Bit `1`** | `TX_BIT1_L = 625` | `TX_BIT1_H = 1640` | Sáng **nhanh**, tắt **lâu** |

> **Quy tắc phân biệt:** Bit `0` và Bit `1` có tổng thời gian gần bằng nhau (~2260 µs), nhưng **tỷ lệ sáng/tắt ngược nhau**. Đây là cách bộ giải mã trên Robot phân biệt giá trị của từng bit.

---

## 2. Hàm cốt lõi: `build_tx_frame`

Hàm này có nhiệm vụ **chuyển đổi 1 mã lệnh 8-bit thành mảng xung vật lý** sẵn sàng phát ra GPIO.

### Mã nguồn

```c
static void build_tx_frame(rmt_symbol_word_t* symbols, dog_cmd_t cmd, bool is_first_frame)
{
    // [0] Header
    symbols[0].duration0 = TX_HDR_L;   // 6245µs — LED sáng
    symbols[0].level0    = 1;
    symbols[0].duration1 = TX_HDR_H;   // 460µs  — LED tắt
    symbols[0].level1    = 0;

    // [1]-[8] Data bits (8 bit, MSB first)
    for (int i = 0; i < 8; i++) {
        bool bit_val = is_first_frame && ((cmd >> (7 - i)) & 1);
        if (bit_val) {
            symbols[1 + i].duration0 = TX_BIT1_L;  // 625µs  — sáng nhanh
            symbols[1 + i].level0    = 1;
            symbols[1 + i].duration1 = TX_BIT1_H;  // 1640µs — tắt lâu
            symbols[1 + i].level1    = 0;
        } else {
            symbols[1 + i].duration0 = TX_BIT0_L;  // 1690µs — sáng lâu
            symbols[1 + i].level0    = 1;
            symbols[1 + i].duration1 = TX_BIT0_H;  // 570µs  — tắt nhanh
            symbols[1 + i].level1    = 0;
        }
    }
}
```

### 2.1. Đầu vào của hàm

| Tham số | Kiểu | Ý nghĩa |
| :--- | :--- | :--- |
| `symbols` | `rmt_symbol_word_t*` | Con trỏ tới mảng 9 phần tử, mỗi phần tử mô tả 1 cặp xung sáng/tắt. Hàm ghi trực tiếp vào mảng này (không cần `return`). |
| `cmd` | `dog_cmd_t` | Mã lệnh 8-bit cần phát. Ví dụ: `CMD_FORWARD = 0x10` (Tiến), `CMD_LEFT = 0x0D` (Trái). |
| `is_first_frame` | `bool` | `true` = frame lệnh thật (mã hóa đúng giá trị `cmd`). `false` = frame lặp (toàn bộ 8 bit đều thành `0`). |

### 2.2. Mảng `symbols` — Cấu trúc và cách lưu giá trị

Mảng `symbols` gồm **9 phần tử**, kiểu `rmt_symbol_word_t` — là kiểu dữ liệu riêng của driver RMT (ESP-IDF), mỗi phần tử lưu trữ **1 cặp xung**:

```
symbols[0]              → Header (xung mở đầu frame)
symbols[1] → symbols[8] → 8 bit dữ liệu của mã lệnh (MSB → LSB)
```

Mỗi phần tử `rmt_symbol_word_t` chứa 4 trường:

| Trường | Ý nghĩa |
| :--- | :--- |
| `duration0` | Thời gian (µs) của **nửa đầu** — giai đoạn LED **sáng** |
| `level0` | Mức logic nửa đầu — luôn bằng `1` (sáng) |
| `duration1` | Thời gian (µs) của **nửa sau** — giai đoạn LED **tắt** |
| `level1` | Mức logic nửa sau — luôn bằng `0` (tắt) |

### 2.3. Cách trích xuất từng bit từ mã lệnh

Biểu thức `(cmd >> (7 - i)) & 1` dùng phép dịch bit để lấy ra từng bit của `cmd` theo thứ tự **MSB trước** (bit cao nhất phát trước):

**Ví dụ:** `cmd = CMD_FORWARD = 0x10 = 0b00010000`

| Vòng lặp `i` | Bit lấy ra (vị trí `7-i`) | Giá trị | Timing được ghi |
| :--- | :--- | :--- | :--- |
| `i = 0` | Bit 7 | `0` | Sáng 1690 µs, Tắt 570 µs |
| `i = 1` | Bit 6 | `0` | Sáng 1690 µs, Tắt 570 µs |
| `i = 2` | Bit 5 | `0` | Sáng 1690 µs, Tắt 570 µs |
| `i = 3` | Bit 4 | **`1`** | **Sáng 625 µs, Tắt 1640 µs** |
| `i = 4` | Bit 3 | `0` | Sáng 1690 µs, Tắt 570 µs |
| `i = 5` | Bit 2 | `0` | Sáng 1690 µs, Tắt 570 µs |
| `i = 6` | Bit 1 | `0` | Sáng 1690 µs, Tắt 570 µs |
| `i = 7` | Bit 0 | `0` | Sáng 1690 µs, Tắt 570 µs |

---

## 3. Sau khi xây dựng xong mảng xung — Phát ra GPIO bằng RMT

Mảng `symbols` sau khi được `build_tx_frame` ghi đầy đủ 9 cặp xung sẽ được đưa thẳng cho ngoại vi phần cứng **RMT (Remote Control Transceiver)** của ESP32 để phát ra chân GPIO:

```c
rmt_symbol_word_t frame[9];                  // Tạo mảng 9 ô trống
build_tx_frame(frame, CMD_FORWARD, true);    // Ghi timing vào mảng

// Đưa mảng cho phần cứng RMT để tự động phát ra GPIO
rmt_transmit(s_tx_channel, s_copy_encoder, frame, sizeof(frame), &tx_cfg);

// Đợi phần cứng phát xong toàn bộ 9 cặp xung
rmt_tx_wait_all_done(s_tx_channel, portMAX_DELAY);
```

**Luồng xử lý:**

```
build_tx_frame()  →  rmt_transmit()  →  Phần cứng RMT  →  GPIO46  →  LED hồng ngoại  →  Robot Dog
   (phần mềm)         (giao cho HW)      (tự động phát)    (chân phát)    (phát tia IR)     (nhận lệnh)
```

> **Điểm mấu chốt:** Sau khi gọi `rmt_transmit()`, CPU hoàn toàn rảnh tay. Ngoại vi phần cứng RMT sẽ tự động đọc từng phần tử trong mảng `symbols` và bật/tắt chân GPIO46 đúng theo thời gian đã ghi — tạo ra tín hiệu hồng ngoại thực sự phát ra ngoài không gian mà **không cần CPU can thiệp**.

---

## 4. Bảng mã lệnh đầy đủ

| Lệnh | Mã nhị phân | Mã HEX | Mô tả |
| :--- | :--- | :--- | :--- |
| `CMD_FORWARD` | `0b00010000` | `0x10` | Tiến (bánh xe) |
| `CMD_BACKWARD` | `0b00001010` | `0x0A` | Lùi (bánh xe) |
| `CMD_LEFT` | `0b00001101` | `0x0D` | Quay trái (bánh xe) |
| `CMD_RIGHT` | `0b00001001` | `0x09` | Quay phải (bánh xe) |
| `CMD_MUSIC` | `0b00000110` | `0x06` | Mở nhạc |
| `CMD_STEP_FORWARD` | `0b00000111` | `0x07` | Tiến bước (chân + bánh) |
| `CMD_STEP_LEFT` | `0b00010001` | `0x11` | Trái từng bước |
| `CMD_STEP_BACKWARD` | `0b00010010` | `0x12` | Lùi từng bước |
| `CMD_STEP_RIGHT` | `0b00001000` | `0x08` | Phải từng bước |
| `CMD_TOGGLE` | `0b00001011` | `0x0B` | Ngồi ↔ Đứng |
| `CMD_STRETCH` | `0b00010011` | `0x13` | Duỗi chân |
| `CMD_HALT` | `0b00001111` | `0x0F` | Dừng lại |