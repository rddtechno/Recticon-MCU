?# Pin Assignment — Rectifier Controller (Recticon)

**MCU**    : STM32F412RETx — LQFP64
**Clock**  : HSE 16 MHz → PLL (M=8, N=100) → SYSCLK 100 MHz
**Bus**    : AHB 100 MHz · APB1 50 MHz (Timer 100 MHz) · APB2 100 MHz
**Sumber** : Analisa `V0.ioc` (MxCube 6.4.0, FW_F4 V1.26.2) + requirement `prompt.txt`

---

## 1. Ringkasan Peripheral

| Peripheral | Fungsi pada project ini | Status di `.ioc` |
|---|---|---|
| I2C1 | EEPROM 24LC08 + ADS1115 + MCP4725 | Aktif (PB6/PB7) |
| SPI1 | ADS131M04 (ADC 4-kanal) | Aktif + DMA TX/RX |
| SPI3 | 2× MCP23S17 (DI & DO expander) | Aktif |
| USART1 | Modbus Slave — TTL ke Orange Pi Zero3 | Aktif + DMA TX |
| USART2 | Modbus Master — RS485 ke Power Meter & SinePower ST36 | Aktif + DMA TX |
| USART6 | Modbus Slave — RS485 ke HMI | Aktif + DMA TX |
| ADC1 | NTC temp, arus BTS, arus VSAT, temp internal | Aktif + DMA (scan, circular) |
| TIM6 | Time base 1 ms (Presc 99, Period 1000) | Aktif, IRQ prio 15 |
| TIM7 | Time base tambahan (Presc 99) | Aktif |
| IWDG | Watchdog, prescaler 256 | Aktif |
| SYS | SWD debug (PA13/PA14) | Serial Wire |

---

## 2. Pin Assignment per Fungsi

### 2.1 EEPROM — 24LC08 (I2C1)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PB6 | `I2C1_SCL` | `I2C1_SCL` | AF4, Open-Drain, Pull-up | Bus bersama ADS1115 & MCP4725 |
| PB7 | `I2C1_SDA` | `I2C1_SDA` | AF4, Open-Drain, Pull-up | Bus bersama ADS1115 & MCP4725 |
| PB8 | `GPIO_Output` | `DO_EEPROM_WP` | Push-Pull, default **HIGH** | Write Protect aktif-HIGH → LOW saat menulis |

- Kapasitas 24LC08 = 8 kbit (1 kByte), dipetakan sebagai **4 blok × 256 byte**.
- Alamat I2C: `0xA0, 0xA2, 0xA4, 0xA6` (7-bit: `0x50`–`0x53`), 2 bit blok masuk ke byte alamat.
- Urutan tulis: `WP = LOW` → page write (≤16 byte/page) → tunggu ACK-polling ≤5 ms → `WP = HIGH`.

**Cakupan proteksi WP — sudah diuji di hardware (2026-08-18).** Pada 24LC08B Microchip standar, WP hanya memproteksi separuh atas array (0x200–0x3FF). Part yang terpasang di board ini **berbeda: WP memproteksi SELURUH array**. Diverifikasi dengan mencoba menulis satu byte di `0x010` (separuh bawah) dan `0x210` (separuh atas) sementara WP ditahan HIGH — keduanya terblokir, sementara tulis normal dengan WP LOW berhasil. Konsekuensinya: **layout config mulai 0x000 terlindungi penuh**, tidak perlu dipindah ke separuh atas. Uji ini bisa diulang kapan saja lewat `g_ee_cmd = 5` (lihat `eeprom_24lc08.c`) — jalankan lagi bila suatu saat vendor/part EEPROM diganti.

### Peta memori EEPROM

| Alamat | Blok | Isi |
|---|---|---|
| `0x000`–`0x0FF` | 0 | Config utama *(menyusul)* |
| `0x100`–`0x1FF` | 1 | Config backup *(menyusul)* |
| `0x200`–`0x23F` | 2 | Area uji manual (`g_ee_test_addr`) |
| `0x240`–`0x24F` | 2 | Area self-test |
| `0x300`–`0x3FF` | 3 | Cadangan |

### 2.2 Digital I/O Expander — 2× MCP23S17 (SPI3)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PC10 | `SPI3_SCK` | `SPI3_SCK` | AF6 | Clock bersama 2 chip |
| PC11 | `SPI3_MISO` | `SPI3_MISO` | AF6 | Data in bersama 2 chip |
| PC12 | `SPI3_MOSI` | `SPI3_MOSI` | AF6 | Data out bersama 2 chip |
| PD2 | `GPIO_Output` | `DO_IOEXP_DI_CS` | Push-Pull, default **HIGH** | **CS chip Digital Input** |
| PB3 | `GPIO_Output` | `DO_IOEXP_DI_RST` | Push-Pull, default **HIGH** | **Reset chip Digital Input** (aktif LOW) |
| PA15 | `GPIO_Output` | `DO_IOEXP_DO_CS` | Push-Pull, default **HIGH** | **CS chip Digital Output** |
| PA12 | `GPIO_Output` | `DO_IOEXP_DO_RST` | Push-Pull, default **HIGH** | **Reset chip Digital Output** (aktif LOW) |

- SPI Mode **0** (CPOL=0, CPHA=0) — sesuai `SPI3.CLKPhase = 1EDGE`, polarity LOW.
- Setiap chip punya CS terpisah → opcode alamat boleh sama (`0x40`/`0x41` dengan A2:A0 = 000), hardware addressing tidak wajib.
- Chip **DI**: `IODIRA/B = 0xFF` (semua input), aktifkan `GPPUA/B` bila perlu pull-up.
- Chip **DO**: `IODIRA/B = 0x00` (semua output), inisialisasi `OLATA/B = 0x00`.
- PB3 & PA15 adalah pin JTDO/JTDI. Aman dipakai GPIO karena debug memakai **Serial Wire** saja (PA13/PA14).

### 2.3 RS485 #1 — Modbus **Master** (Power Meter & SinePower ST36)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PA2 | `USART2_TX` | `USART2_TX` | AF7 | Ke pin DI transceiver |
| PA3 | `USART2_RX` | `USART2_RX` | AF7 | Dari pin RO transceiver |
| PA4 | `GPIO_Output` | `DO_RS485_1_DE` | Push-Pull, default **LOW** | **DE/RE transceiver** — HIGH = TX, LOW = RX |

- Baudrate saat ini di `.ioc`: **9600**, 8-N-1. DMA TX = `DMA1_Stream6`, IRQ prio 14.
- Toggle DE: set HIGH sebelum `HAL_UART_Transmit_DMA()`, turunkan di callback `TxCplt` setelah flag `TC` (bukan `TXE`).

### 2.4 RS485 #2 — Modbus **Slave** (HMI)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PC6 | `USART6_TX` | `USART6_TX` | AF8 | Ke pin DI transceiver |
| PC7 | `USART6_RX` | `USART6_RX` | AF8 | Dari pin RO transceiver |
| PC8 | `GPIO_Output` | `DO_RS485_2_DE` | Push-Pull, default **LOW** | **DE/RE transceiver** — HIGH = TX, LOW = RX |

- Baudrate saat ini di `.ioc`: **38400**, 8-N-1. DMA TX = `DMA2_Stream6`, IRQ prio 14.
- Sebagai slave, DE harus LOW (mode listen) selama idle agar tidak menabrak bus master HMI.

### 2.5 TTL — Modbus **Slave** (Orange Pi Zero3)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PA9 | `USART1_TX` | `USART1_TX` | AF7 | TTL 3V3 langsung, tanpa transceiver |
| PA10 | `USART1_RX` | `USART1_RX` | AF7 | TTL 3V3 langsung, tanpa transceiver |
| PA11 | `GPIO_Output` | `DO_OPI_PWR` | Push-Pull | Kontrol power Orange Pi (pin eksisting, dipertahankan) |

- Baudrate saat ini di `.ioc`: **38400**, 8-N-1. DMA TX = `DMA2_Stream7`, IRQ prio 14.
- Tidak butuh pin DE karena point-to-point TTL (half-duplex arbitration tidak diperlukan).

### 2.6 ADS1115 — Tegangan & Arus LOAD (I2C1)

Tidak memakai pin GPIO tambahan — sepenuhnya di bus I2C1 (PB6/PB7).

| Measurement | MUX | FSR | Data Rate | Keterangan |
|---|---|---|---|---|
| **LOAD Voltage** | **MODE 0** (`MUX = 000`, AIN0–AIN1 differential) | **±2.048 V** (`PGA = 010`) | **860 SPS** (`DR = 111`) | LSB = 62.5 µV |
| **LOAD Current** | **MODE 3** (`MUX = 011`, AIN2–AIN3 differential) | **±2.048 V** (`PGA = 010`) | **860 SPS** (`DR = 111`) | LSB = 62.5 µV |

- Alamat I2C: **`0x4A`** 7-bit → pin `ADDR` disambung ke **SDA**. Write = `0x94`, Read = `0x95`. Tidak bentrok dengan 0x50–0x53 (EEPROM) maupun 0x61 (DAC).
- Pin `ALERT/RDY` **tidak dikabelkan** ke MCU → gunakan **single-shot + polling** bit `OS` di Config Register, atau continuous mode dengan pembacaan periodik dari TIM6 (1 ms tick).
- Karena dua measurement memakai MUX berbeda pada satu ADC, akses harus **bergantian (time-multiplexed)**: tulis Config → tunggu ≥1.2 ms (1/860 SPS + margin) → baca Conversion Register.

### 2.7 ADS131M04 — Tegangan & Arus Rectifier / Baterai (SPI1)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PA5 | `SPI1_SCK` | `SPI1_SCK` | AF5, Pull-Down | CPOL=0 |
| PA6 | `SPI1_MISO` | `SPI1_MISO` | AF5, Pull-Down | |
| PA7 | `SPI1_MOSI` | `SPI1_MOSI` | AF5, Pull-Down | |
| PA8 | `GPIO_EXTI8` | `DI_ADC_DRDY` | Input, **EXTI Falling**, `GPIO_NOPULL` | **DRDY** — aktif LOW, IRQ `EXTI9_5_IRQn` prio 10. Pull-up **eksternal di board** |
| PB9 | `GPIO_Output` | `DO_ADC_RST` | Push-Pull, default **HIGH** | **Reset** — aktif LOW, pulse ≥2 µs lalu tunggu ≥5 µs |
| PC9 | `GPIO_Output` | `DO_ADC_CS` | Push-Pull, Very High Speed, default **HIGH** | **Chip Select** — aktif LOW |

Konfigurasi kanal:

| Kanal | Measurement | FSR | Gain (PGA) | Keterangan |
|---|---|---|---|---|
| **CH0** | Rectifier Voltage | **±1.2 V** | **1** | Gain default, full-scale = VREF/Gain = 1.2 V |
| **CH1** | Rectifier Current | **±300 mV** | **4** | 1.2 V / 4 = 300 mV |
| **CH2** | Battery Voltage | **±1.2 V** | **1** | |
| **CH3** | Battery Current | **±300 mV** | **4** | 1.2 V / 4 = 300 mV |

- Sample rate: **64 kSPS** → `OSR = 64` (`MODE.OSR = 000`) dengan fMOD = 4.096 MHz (CLKIN 8.192 MHz internal/eksternal).
- SPI Mode **1** (CPOL=0, CPHA=1) — sesuai `SPI1.CLKPolarity = LOW`, `SPI1.CLKPhase = 2EDGE`. **Sudah benar.**
- Word size 24-bit; satu frame = STATUS + 4× data + CRC = **6 word × 24 bit = 144 bit**.
- DMA: TX = `DMA2_Stream3`, RX = `DMA2_Stream2`, IRQ `SPI1_IRQn` prio 10.
- Alur baca: EXTI PA8 falling → CS LOW → `HAL_SPI_TransmitReceive_DMA()` 18 byte → callback → CS HIGH → parse.

### 2.8 MCP4725 — DAC (I2C1)

Tidak memakai pin GPIO tambahan — di bus I2C1 (PB6/PB7).

| Parameter | Nilai |
|---|---|
| Alamat I2C | **`0x61`** 7-bit — terverifikasi di hardware (2026-08-18) |
| Resolusi | 12-bit (0–4095) |
| Vref | VDD (3.3 V) → LSB ≈ 0.806 mV |
| Command | `0x40` = Write DAC Register (volatile, cepat) |
| EEPROM write | `0x60` = Write DAC + EEPROM — **hindari di runtime**, endurance terbatas |

Gunakan **Fast Mode Write** (2 byte) untuk update setpoint yang sering; simpan ke EEPROM internal MCP4725 hanya saat commissioning.

> **Catatan alamat.** Alamat dasar MCP4725 ditentukan **sufiks part number** (diprogram pabrik), bukan hanya pin `A0`: A0 → 0x60/0x61, A1 → 0x62/0x63, A2 → 0x64/0x65, A3 → 0x66/0x67. Part di board ini menjawab di **0x61**. Kalau suatu saat chip diganti dan `g_dac_present` jadi 0, kemungkinan besar sufiks part-nya berbeda — alamat bisa dikoreksi saat runtime lewat `g_dac_i2c_addr` + `g_dac_cmd = 4` tanpa rebuild.

---

## 3. Tabel Master — Seluruh Pin

Legenda kolom **Ubah**: ● = label/fungsi berubah dari `.ioc` saat ini · ○ = sudah sesuai · – = di luar scope prompt (dipertahankan)

| Pin | Signal / Mode | Label Baru | Fungsi | Ubah |
|---|---|---|---|---|
| PC13 | GPIO_Output | `DO_RLY_VIN` | Relay input AC | – |
| PC14 | GPIO_Output | `DO_SYS_RUN` | LED sistem RUN | – |
| PC15 | GPIO_Output | `DO_SYS_ALM` | LED sistem ALARM | – |
| PH0 | GPIO_Output | `DO_BMS_EN` | Enable BMS | – |
| PH1 | GPIO_Output | `DO_BMS1` | Kontrol BMS 1 | – |
| PC0 | ADC1_IN10 | `AI_NTC_TEMP` | Sensor suhu NTC | – |
| PC1 | ADC1_IN11 | `AI_BTS_CUR` | Arus BTS | – |
| PC2 | ADC1_IN12 | `AI_VSAT_CUR` | Arus VSAT | – |
| PC3 | GPIO_Output | `DO_BMS2` | Kontrol BMS 2 | – |
| PA0 | GPIO_Output | `DO_BMS3` | Kontrol BMS 3 | – |
| PA1 | GPIO_Output | `DO_USART2_MUX` | Mux jalur USART2 | – |
| **PA2** | **USART2_TX** (AF7) | `USART2_TX` | **RS485 Master TX** | ○ |
| **PA3** | **USART2_RX** (AF7) | `USART2_RX` | **RS485 Master RX** | ○ |
| **PA4** | **GPIO_Output** | `DO_RS485_1_DE` | **DE/RE RS485 Master** | ● |
| **PA5** | **SPI1_SCK** (AF5) | `SPI1_SCK` | **ADS131M04 clock** | ○ |
| **PA6** | **SPI1_MISO** (AF5) | `SPI1_MISO` | **ADS131M04 data in** | ○ |
| **PA7** | **SPI1_MOSI** (AF5) | `SPI1_MOSI` | **ADS131M04 data out** | ○ |
| PC4 | GPIO_Output | `DO_LCD_A0` | LCD command/data | – |
| PC5 | GPIO_Output | `DO_LCD_RST` | LCD reset | – |
| PB0 | TIM3_CH3 | `DI_BTN_OK_PM1_ZX` | Tombol OK / zero-cross PM1 | – |
| PB1 | GPIO_Input | `DI_BTN_BCK` | Tombol BACK | – |
| PB10 | TIM2_CH3 | `DI_BTN_UP_PM2_ZX` | Tombol UP / zero-cross PM2 | – |
| PB12 | GPIO_Input | `DI_BTN_DN` | Tombol DOWN | – |
| PB13 | GPIO_Output | `DO_LCD_CS` | LCD chip select | – |
| PB14 | GPIO_Output | `DO_RS232_MUXA` | Mux RS232 A | – |
| PB15 | GPIO_Output | `DO_RS232_MUXB` | Mux RS232 B | – |
| **PC6** | **USART6_TX** (AF8) | `USART6_TX` | **RS485 Slave HMI TX** | ○ |
| **PC7** | **USART6_RX** (AF8) | `USART6_RX` | **RS485 Slave HMI RX** | ○ |
| **PC8** | **GPIO_Output** | `DO_RS485_2_DE` | **DE/RE RS485 Slave HMI** | ● |
| **PC9** | **GPIO_Output** VeryHigh, init HIGH | `DO_ADC_CS` | **ADS131M04 CS** | ● |
| **PA8** | **GPIO_EXTI8** falling, NoPull | `DI_ADC_DRDY` | **ADS131M04 DRDY** (pull-up eksternal) | ● |
| **PA9** | **USART1_TX** (AF7) | `USART1_TX` | **TTL Modbus Slave TX (OPi)** | ○ |
| **PA10** | **USART1_RX** (AF7) | `USART1_RX` | **TTL Modbus Slave RX (OPi)** | ○ |
| PA11 | GPIO_Output | `DO_OPI_PWR` | Power enable Orange Pi | – |
| **PA12** | **GPIO_Output**, init HIGH | `DO_IOEXP_DO_RST` | **Reset MCP23S17 (Output)** | ● |
| PA13 | SYS_JTMS-SWDIO | `SWDIO` | Debug | ○ |
| PA14 | SYS_JTCK-SWCLK | `SWCLK` | Debug | ○ |
| **PA15** | **GPIO_Output**, init HIGH | `DO_IOEXP_DO_CS` | **CS MCP23S17 (Output)** | ● |
| **PC10** | **SPI3_SCK** (AF6) | `SPI3_SCK` | **MCP23S17 clock** | ○ |
| **PC11** | **SPI3_MISO** (AF6) | `SPI3_MISO` | **MCP23S17 data in** | ○ |
| **PC12** | **SPI3_MOSI** (AF6) | `SPI3_MOSI` | **MCP23S17 data out** | ○ |
| **PD2** | **GPIO_Output**, init HIGH | `DO_IOEXP_DI_CS` | **CS MCP23S17 (Input)** | ● |
| **PB3** | **GPIO_Output**, init HIGH | `DO_IOEXP_DI_RST` | **Reset MCP23S17 (Input)** | ● |
| PB4 | GPIO_Input | `DI_MCB_BTS` | Status MCB BTS | – |
| PB5 | GPIO_Input | `DI_MCB_VSAT` | Status MCB VSAT | – |
| **PB6** | **I2C1_SCL** (AF4, OD) | `I2C1_SCL` | **EEPROM + ADS1115 + DAC** | ○ |
| **PB7** | **I2C1_SDA** (AF4, OD) | `I2C1_SDA` | **EEPROM + ADS1115 + DAC** | ○ |
| **PB8** | **GPIO_Output**, init HIGH | `DO_EEPROM_WP` | **Write Protect 24LC08** | ○ |
| **PB9** | **GPIO_Output**, init HIGH | `DO_ADC_RST` | **ADS131M04 Reset** | ● |
| PB2 | — | — | **BEBAS** (BOOT1) | |
| PB11 | — | — | **BEBAS** | |

**Total**: 52 pin terpakai dari 64 (LQFP64), sisa 2 GPIO bebas (PB2, PB11) + pin power/ground.

---

## 4. Peta Alamat Bus

### I2C1 (PB6/PB7) — 3 device

| Device | Alamat 7-bit | Alamat 8-bit W/R | Catatan |
|---|---|---|---|
| 24LC08 EEPROM | `0x50`–`0x53` | `0xA0`/`0xA1` … `0xA6`/`0xA7` | 4 blok × 256 byte |
| ADS1115 | `0x4A` | `0x94` / `0x95` | ADDR → SDA |
| MCP4725 | `0x61` | `0xC2` / `0xC3` | terverifikasi di hardware |

Tidak ada bentrok alamat. Bus dijalankan di **100 kHz (Standard Mode)** — pull-up **4.7 kΩ**.

### SPI3 (PC10/11/12) — 2 device, CS terpisah

| Device | CS | Reset | Opcode Write / Read |
|---|---|---|---|
| MCP23S17 Digital **Input** | PD2 | PB3 | `0x40` / `0x41` |
| MCP23S17 Digital **Output** | PA15 | PA12 | `0x40` / `0x41` |

### SPI1 (PA5/6/7) — 1 device

| Device | CS | Reset | DRDY |
|---|---|---|---|
| ADS131M04 | PC9 | PB9 | PA8 (EXTI falling) |

---

## 5. Perubahan yang Perlu Dilakukan di CubeMX

> **STATUS: SELESAI SEMUA ✔** — seluruh label dan atribut GPIO di bawah sudah diterapkan ke `V0.ioc`. Backup versi sebelumnya ada di `V0.ioc.bak`.

Semua pin sudah teralokasi di `.ioc`, yang berubah hanya **label** dan beberapa **atribut GPIO**:

| Pin | Label lama | Label baru | Perubahan atribut |
|---|---|---|---|
| PA4 | `DO_FUEL_EN` | `DO_RS485_1_DE` | — |
| PA8 | `DIO_AUX1` | `DI_ADC_DRDY` | ✔ `GPIO_NOPULL` — pull-up sudah ada di hardware |
| PA12 | `DI_FAN_IOEXP_RST` | `DO_IOEXP_DO_RST` | ✔ `PinState = GPIO_PIN_SET` |
| PA15 | `DI_DOOR_SPI3_CS0` | `DO_IOEXP_DO_CS` | ✔ `PinState = GPIO_PIN_SET` |
| PB3 | `DO_LVD_VSAT` | `DO_IOEXP_DI_RST` | ✔ `PinState = GPIO_PIN_SET` |
| PB9 | `DIO_AUX2` | `DO_ADC_RST` | ✔ `PinState = GPIO_PIN_SET` |
| PC8 | `DO_RS232_EN_CLK_EN_HUM` | `DO_RS485_2_DE` | ⬜ `PinState = GPIO_PIN_RESET` (default, sudah benar) |
| PC9 | `DIO_AUX0` | `DO_ADC_CS` | ✔ Sudah HIGH + Very High Speed |
| PD2 | `DO_LVD_BTS/SPI3_CS1` | `DO_IOEXP_DI_CS` | ✔ `PinState = GPIO_PIN_SET` |
| PB8 | `DO_EEPROM_WP` | *(tetap)* | ✔ `PinState = GPIO_PIN_SET` |

> **Status `PinState`: seluruhnya selesai** — PA12, PA15, PB3, PB8, PB9, PC9, PD2. Semua chip (2× MCP23S17, ADS131M04) start dalam kondisi CS idle & tidak ter-reset, dan write protect EEPROM aktif sejak boot. Turunkan `DO_EEPROM_WP` ke LOW hanya di dalam rutin write, lalu kembalikan HIGH.

**Konfigurasi peripheral yang perlu ditinjau:**

1. ~~**SPI1 baudrate — WAJIB DIUBAH.**~~ **✔ SELESAI.** `BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8` → **12.5 Mbit/s**. Verifikasi timing: 1 frame = 144 bit → **11.5 µs**, periode sampel @64 kSPS = **15.6 µs** → margin **≈4.1 µs (26%)** untuk CS setup/hold + latensi ISR EXTI + setup DMA. Masih jauh di bawah batas 25 MHz SCLK ADS131M04. Lihat catatan margin di bagian 6.
2. ~~**SPI3 baudrate.**~~ **✔ SELESAI.** `BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16` → **3.125 Mbit/s** (SPI3 di APB1 50 MHz). Baca 2 register MCP23S17 (4 byte) kini ~10 µs — scan digital input bisa dilakukan tiap 1 ms tick TIM6 tanpa membebani bus. Masih di bawah batas 10 MHz MCP23S17.
3. **I2C1 — dipertahankan di 100 kHz (Standard Mode)** sesuai keputusan. Tidak ada parameter timing di `.ioc`, dan default CubeMX untuk I2C1 memang `ClockSpeed = 100000` — jadi tidak ada yang perlu diubah. Implikasi throughput ada di bagian 6.
4. **Prioritas NVIC.** `EXTI9_5` (DRDY) dan `SPI1` sama-sama prio 10, di atas USART (14) dan TIM6 (15). Sudah tepat karena akuisisi 64 kSPS paling kritis waktunya.

---

## 6. Catatan & Peringatan

- **PA11/PA12 adalah USB_DM/USB_DP.** Dipakai sebagai GPIO biasa; pastikan USB OTG tidak diaktifkan di CubeMX.
- **PB3 & PA15 adalah JTDO/JTDI.** Aman karena debug memakai Serial Wire (PA13/PA14). Namun `MX_GPIO_Init()` harus dipanggil setelah `HAL_Init()` — jangan tarik LOW saat boot bila board pernah dipakai JTAG penuh.
- **PC14/PC15 adalah OSC32_IN/OUT.** Dipakai GPIO → **LSE tidak tersedia**, RTC (jika nanti dibutuhkan) harus pakai LSI dengan akurasi rendah.
- **PB2 = BOOT1.** Jika nanti dipakai, pastikan tidak mengganggu boot mode.
- **Konflik shadow label lama.** Beberapa label `.ioc` menyiratkan dual-function (`DO_LVD_BTS/SPI3_CS1`, `DI_DOOR_SPI3_CS0`, `DI_GST_RUN_SPI3_SCK`). Setelah SPI3 dipakai penuh untuk MCP23S17, fungsi LVD/DOOR/GST **harus dipindah ke kanal MCP23S17**, bukan ke pin MCU.
- **I2C1 @100 kHz — anggaran waktu bus.** Estimasi per transaksi (1 bit ≈ 10 µs):

  | Transaksi | Byte di bus | Waktu |
  |---|---|---|
  | ADS1115 — tulis Config (ganti MUX) | 4 | ≈ 400 µs |
  | ADS1115 — set pointer + baca Conversion | 5 | ≈ 500 µs |
  | MCP4725 — Fast Mode Write | 3 | ≈ 300 µs |
  | 24LC08 — page write 16 byte | 19 | ≈ 1.7 ms + 5 ms siklus internal |

  Konsekuensi untuk ADS1115: satu siklus ganti MUX = tulis Config (0.4 ms) + tunggu konversi (1.16 ms) + baca (0.5 ms) ≈ **2.1 ms**. Bergantian antara LOAD Voltage dan LOAD Current → tiap kanal ter-refresh **≈ 4.2 ms (≈ 240 Hz)**. Lebih dari cukup untuk monitoring beban.
- **Setting 860 SPS pada ADS1115 tidak menaikkan laju sampel efektif.** Karena dua measurement berbagi satu ADC lewat MUX dan bus 100 kHz, laju nyata dibatasi I2C (≈240 Hz/kanal), bukan ADC. 860 SPS tetap berguna: waktu konversi jadi pendek (1.16 ms) sehingga siklus MUX cepat. Konsekuensinya noise per sampel lebih tinggi dibanding data rate rendah — lakukan **rata-rata/filter di firmware** (mis. moving average 8–16 sampel) untuk pembacaan yang stabil.
- **Jangan campur transaksi I2C dengan write EEPROM di jalur kritis.** Page write 24LC08 memblokir device 5 ms (ACK-polling). Selama itu ADS1115 dan MCP4725 tetap bisa diakses, tapi jika driver memakai satu mutex bus, akuisisi LOAD akan tertunda. Simpan konfigurasi ke EEPROM hanya saat idle/commissioning, bukan periodik.
- **ADS1115 tanpa ALERT/RDY** → tidak ada notifikasi konversi selesai. Jika latensi jadi masalah, pertimbangkan menyambungkan ALERT/RDY ke PB2 atau PB11 yang masih bebas.
- **ADS1115 `ADDR` → SDA (alamat 0x4A).** Konfigurasi ini sah, tapi datasheet mensyaratkan **SDA tidak boleh ditahan LOW lebih dari 100 µs setelah kondisi START**. Konsekuensinya: jangan pakai clock stretching di master, dan hindari menahan bus di tengah transaksi (mis. breakpoint debugger saat transfer I2C berjalan bisa membuat ADS1115 salah mendeteksi alamatnya). Jika ini mengganggu saat debugging, pindahkan `ADDR` ke GND (0x48) atau VDD (0x49) yang tidak punya batasan timing.
- **Margin timing ADS131M04 tipis (26%).** Dengan SPI1 @12.5 Mbit/s, transfer 144 bit memakan 11.5 µs dari 15.6 µs periode sampel. Agar tidak overrun: (a) gunakan **DMA** (sudah dikonfigurasi: `DMA2_Stream2` RX / `DMA2_Stream3` TX), jangan `HAL_SPI_TransmitReceive()` blocking; (b) jaga ISR `EXTI9_5` sesingkat mungkin — cukup turunkan CS dan start DMA, parsing dilakukan di main loop; (c) jika margin terbukti kurang saat diuji, turunkan sample rate ke 32 kSPS (`OSR = 128`) — untuk monitoring tegangan/arus rectifier, 32 kSPS masih sangat memadai.
- **Referensi ADS131M04.** FSR ±1.2 V mengasumsikan VREF internal 1.2 V dengan gain 1. Pastikan pembagi tegangan eksternal sudah menskalakan tegangan rectifier/baterai ke rentang ini, dan shunt amplifier menghasilkan ≤300 mV pada arus maksimum.

---

## 7. Rangkuman Cepat

```
I2C1  (PB6/PB7) ──┬── 24LC08 EEPROM   0x50-0x53   WP = PB8
                  ├── ADS1115         0x4A        LOAD V (MUX0) / LOAD I (MUX3), ±2.048V, 860SPS
                  └── MCP4725 DAC     0x61        12-bit

SPI1  (PA5/6/7) ──── ADS131M04  CS=PC9  RST=PB9  DRDY=PA8(EXTI)  64kSPS
                     CH0 Rect V ±1.2V | CH1 Rect I ±300mV
                     CH2 Batt V ±1.2V | CH3 Batt I ±300mV

SPI3 (PC10/11/12) ─┬─ MCP23S17 #1 (DI)  CS=PD2   RST=PB3
                   └─ MCP23S17 #2 (DO)  CS=PA15  RST=PA12

USART2 (PA2/PA3)  ─── RS485 DE=PA4  → Modbus MASTER → Power Meter + SinePower ST36
USART6 (PC6/PC7)  ─── RS485 DE=PC8  → Modbus SLAVE  → HMI
USART1 (PA9/PA10) ─── TTL 3V3       → Modbus SLAVE  → Orange Pi Zero3
```
