?# Pin Assignment — Rectifier Controller (Recticon)

**MCU**    : STM32F412RETx — LQFP64
**Clock**  : **HSI 16 MHz (internal RC)** → PLL (M=8, N=100, P=2) → SYSCLK 100 MHz · LSI ON (untuk IWDG)
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
| PB4 | `GPIO_EXTI4` | `DI_IOEXP_INTA` | Input, **EXTI Falling**, `GPIO_NOPULL` | **INTA chip DI** — port A, aktif LOW, IRQ `EXTI4_IRQn` prio 10 |
| PB5 | `GPIO_EXTI5` | `DI_IOEXP_INTB` | Input, **EXTI Falling**, `GPIO_NOPULL` | **INTB chip DI** — port B, aktif LOW, IRQ `EXTI9_5_IRQn` prio 10 |
| PA15 | `GPIO_Output` | `DO_IOEXP_DO_CS` | Push-Pull, default **HIGH** | **CS chip Digital Output** |
| PA12 | `GPIO_Output` | `DO_IOEXP_DO_RST` | Push-Pull, default **HIGH** | **Reset chip Digital Output** (aktif LOW) |

- SPI Mode **0** (CPOL=0, CPHA=0) — sesuai `SPI3.CLKPhase = 1EDGE`, polarity LOW.
- Setiap chip punya CS terpisah → opcode alamat boleh sama (`0x40`/`0x41` dengan A2:A0 = 000), hardware addressing tidak wajib.
- Chip **DI**: `IODIRA/B = 0xFF` (semua input), aktifkan `GPPUA/B` bila perlu pull-up.
- Chip **DO**: `IODIRA/B = 0x00` (semua output), inisialisasi `OLATA/B = 0x00`.
- PB3 & PA15 adalah pin JTDO/JTDI. Aman dipakai GPIO karena debug memakai **Serial Wire** saja (PA13/PA14).

**Chip Digital Output — sudah diuji di hardware (2026-08-19).** Komunikasi SPI3 ke chip DO (CS=PA15, RST=PA12) berjalan; self-test tulis-baca register lulus (`g_dout_present = 1`, `IOCON` terbaca `0x00`) dan kedelapan kanal bisa dikontrol satu per satu. Driver: `Core/Src/mcp23s17.c`.

| Item | Keputusan | Alasan |
|---|---|---|
| Jumlah kanal DO | **8 kanal, PORT A saja** (GPA0–GPA7) | Sesuai kebutuhan board |
| Port B chip DO | Output, ditahan **LOW** | Cadangan; dijadikan output agar pin tidak mengambang |
| `HAEN` | **0** (hardware addressing mati) | CS terpisah sudah cukup; opcode `0x40`/`0x41` jadi valid berapa pun strapping A2:A0 yang belum pernah diverifikasi |
| Deteksi keberadaan chip | Tulis-baca 2 pola komplemen (`0xAA55`, `0x55AA`) ke `DEFVALA/B` | **SPI tidak punya ACK** — `HAL_SPI_Transmit()` selalu balik `HAL_OK` walau chip dicabut, jadi presence HANYA bisa dibuktikan lewat read-back. `DEFVAL` dipilih sebagai scratch karena isinya tak berarti selama interrupt-on-change mati → uji tidak menggerakkan pin |
| Urutan init | `OLAT` = 0 dulu, baru `IODIR` = 0 | Saat power-up MCP23S17 di mode input (Hi-Z); latch diisi 0 lebih dulu supaya pin tidak menggerakkan relay dengan nilai acak saat arah dibalik |

Kontrol runtime memakai array `g_do[0..7]` (indeks = nomor GPA), bukan bitmask. Setiap perubahan output otomatis diikuti pembacaan `GPIOA` ke `g_dout_gpio_rb` — nilainya harus selalu sama dengan `g_dout_olat`; kalau beda berarti ada pin yang ditahan beban.

> **Menyusul saat relay disambung.** Uji dilakukan dengan seluruh relay **belum terhubung ke beban apa pun**, sehingga perintah `DOUT_CMD_ALL_ON` (5) dan `DOUT_CMD_WALK_TEST` (6) sengaja dibiarkan tanpa pengaman. Begitu relay tersambung, dua perintah itu harus digate ulang (mis. mask kanal yang boleh digerakkan) sebelum dipakai lagi.

**Chip Digital Input — sudah diuji di hardware (2026-08-19).** Chip DI (CS=PD2, RST=PB3) lolos self-test dan seluruh 16 kanal terbaca benar. **16 kanal dipakai penuh: PORT A + PORT B**, diakses lewat array `g_di_a[0..7]` dan `g_di_b[0..7]`. **Interrupt INTA (PB4) / INTB (PB5) juga sudah terverifikasi jalan** pada tanggal yang sama — kedua jalur memicu EXTI dan pembacaan mengikuti.

| Item | Keputusan | Alasan |
|---|---|---|
| Pull-up internal | **Aktif semua** (`GPPU = 0xFFFF`), bisa diubah runtime lewat `g_di_pullup` | Kontak kering dan keluaran opto-coupler sama-sama open-collector — tanpa pull-up pinnya mengambang |
| Inversi logika | Di **firmware** (`g_di_invert`, default `0x0000`), bukan register `IPOL` | `g_di_word` tetap memperlihatkan nilai register mentah untuk diagnosa |
| Interrupt | `INTCON = 0x0000`, `GPINTEN = 0xFFFF` | INTCON=0 → pembanding adalah nilai pin sebelumnya, jadi **setiap** perubahan level memicu INT (bukan dibandingkan ke DEFVAL) |
| `IOCON.MIRROR` | **0** (INTA & INTB terpisah) | INTA melayani port A, INTB port B — sesuai 2 pin MCU yang disediakan |
| Polaritas INT | Aktif **LOW** push-pull (`INTPOL = 0`, `ODR = 0`) | Tidak butuh pull-up eksternal di jalur INT; EXTI di-set **falling** |
| Isi ISR | Hanya menaikkan penghitung + mengangkat flag | Transaksi SPI blocking di dalam ISR akan menahan interrupt lain, termasuk DRDY ADS131M04 yang jauh lebih kritis. Pembacaan dilakukan `DIN_Task()` di main loop |

**Tiga lapis pemicu pembacaan — dan kenapa polling tetap ada.** MCP23S17 menahan INT tetap aktif sampai `GPIO` atau `INTCAP` dibaca. Kalau sebuah perubahan terjadi tepat saat pembacaan berlangsung, INT bisa tinggal LOW terus; karena EXTI kita edge-triggered, tidak akan ada tepi turun baru dan interrupt seolah mati. Karena itu `DIN_Task()` menyampel bila **salah satu** terpenuhi:

1. flag dari ISR terangkat — jalur normal, respons cepat;
2. level pin INTA/INTB terbaca masih LOW — menangkap kasus "INT nyangkut";
3. jatuh tempo `g_di_poll_ms` (default **200 ms**) — jaring pengaman terakhir.

Pembacaan `GPIO` sekaligus melepas INT di chip, jadi tidak perlu baca `INTCAP` terpisah. `g_di_int_level` (bit0=INTA, bit1=INTB) bisa dipantau untuk melihat apakah INT benar-benar lepas.

> **PB5 berbagi vektor dengan ADS131M04.** `EXTI9_5_IRQn` melayani PB5 (INTB) **dan** PA8 (DRDY ADS131M04 @64 kSPS). Konsekuensinya: (a) prioritas INTB terkunci ikut DRDY di 10 — tidak bisa dipisah; (b) tiap DRDY masuk ke vektor yang sama dan handler memeriksa dua jalur, tambahan beberapa siklus per interrupt — masih jauh dari signifikan pada anggaran 15.6 µs/sampel; (c) `HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn)` **tidak boleh** dipanggil saat akuisisi jalan karena ikut membuang event DRDY — di `DIN_Init()` hanya `__HAL_GPIO_EXTI_CLEAR_IT()` per jalur yang dipakai untuk PB5.

### 2.3 RS485 #1 — Modbus **Master** (Power Meter & SinePower ST36)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PA2 | `USART2_TX` | `USART2_TX` | AF7 | Ke pin DI transceiver |
| PA3 | `USART2_RX` | `USART2_RX` | AF7 | Dari pin RO transceiver |
| PA4 | `GPIO_Output` | `DO_RS485_1_DE` | Push-Pull, default **LOW** | **DE/RE transceiver** — HIGH = TX, LOW = RX |

- Baudrate saat ini di `.ioc`: **9600**, 8-N-1. DMA TX = `DMA1_Stream6`, IRQ prio 14. Cocok dengan default pabrik Power Meter (9600 8N1).
- Toggle DE: set HIGH sebelum `HAL_UART_Transmit_DMA()`, turunkan di callback `TxCplt` setelah flag `TC` (bukan `TXE`).

**Implementasi Modbus RTU Master.** Driver: `Core/Src/modbus_master.c`. Menjalankan daftar job bergilir: kirim → tunggu → urai → jeda → job berikutnya. Seluruhnya non-blocking lewat state machine di `MBM_Task()`; tidak ada penantian yang menahan CPU.

**Anggaran waktu @9600 — bus paling lambat di board.** Ini perlu dihitung, bukan ditebak:

| | |
|---|---|
| Jawaban 68 register | 3 + 136 + 2 = **141 byte** |
| Waktu di kabel | 141 × 10 bit / 9600 = **≈147 ms** |
| Timeout default (`g_mbm_timeout_ms`) | **500 ms** |
| Interval polling (`g_mbm_poll_ms`) | **1000 ms** |
| Jeda antar-frame (`g_mbm_gap_ms`) | **200 ms** — syarat Modbus 3.5 karakter @9600 hanya ≈3,7 ms; dipilih jauh lebih longgar agar bus benar-benar bersih antar device |

Meter mendukung sampai 115200 lewat holding register `0x0000` miliknya. **Kalau baud dinaikkan, urutannya kritis:** meter meng-ACK di kecepatan *lama*, menyelesaikan pengiriman ACK itu, baru berpindah — jadi master harus ganti kecepatan **setelah** menerima ACK, bukan sebelumnya. Ditunda dulu sampai polling 9600 terbukti stabil.

### Power Meter ADE7868A — ringkasan integrasi

**Alamat slave = 9 — terverifikasi di hardware (2026-08-19), meter terbaca.** Ditentukan DIP switch 4-posisi di meter (alamat = nilai DIP + 1, jadi 1–16 dan tidak pernah bentrok dengan broadcast). Tersimpan di `PM_DEFAULT_ADDR`, bisa diubah runtime lewat `g_pm_addr` tanpa rebuild. Pemeriksaan silang: input register `0x0000` berisi alamat yang dipakai meter sendiri, jadi **`g_pm_raw[0]` harus = 9**.

Dibaca dengan **satu** permintaan FC `0x04` untuk seluruh blok input register `0x0000`–`0x0043` (68 register, muat dalam batas 125).

> Satu permintaan utuh dipilih dengan sengaja: seluruh pengukuran tiba dari **saat yang sama**. Memecahnya jadi beberapa permintaan akan mencampur nilai dari waktu berbeda dalam satu tampilan.

**Jebakan penyandian — penyebab kesalahan integrasi paling umum di peta ini.** Hampir semua besaran **32-bit menempati dua register, word tinggi di alamat rendah**, dan satuannya **milli-unit** (mA, mV, mW, mvar, mVA). Membaca satu register saja dari sebuah pasangan itu **sah secara protokol tapi gagal diam-diam** — yang keluar bilangan wajar tapi salah, bukan error. Contoh dari dokumen meter: tegangan 224,884 V = `0x00036E74`; membaca register bawahnya saja memberi 28276, yang dikali faktor "masuk akal" 100 terbaca 282,8 V dan tidak ada yang tampak rusak.

Satu-satunya nilai 16-bit di seluruh peta: frekuensi jala-jala, tiga power factor, dua penunjuk fase puncak, dan holding register.

| Blok | Alamat | Isi |
|---|---|---|
| Identitas | `0x0000`–`0x0001` | Alamat slave, versi peta register |
| Fase A/B/C — ukur | `0x0002`–`0x001F` | I (mA), V (mV), P (mW), Q (mvar), S (mVA) — 10 register per fase |
| Fase A/B/C — energi | `0x0020`–`0x0031` | Wh, varh, VAh kumulatif — 6 register per fase |
| Frekuensi & PF | `0x0032`–`0x0035` | Frekuensi `uint16` **centi-hertz** (`5002` = 50,02 Hz), PF `int16` ×1000 |
| Deteksi puncak | `0x0036`–`0x003B` | Arus/tegangan puncak + fase sumbernya |
| Pulsa CF | `0x003C`–`0x0043` | Cacah CF1/2/3 + **konstanta meter efektif** |

Hasil terurai ada di `g_pm_phase[0..2]` (Volt, Amper, Watt, var, VA, PF), `g_pm_frequency` (Hz), dan `g_pm_energy_wh[]` / `g_pm_energy_varh[]`.

**Dua perilaku meter yang perlu diingat:**

1. **Exception `0x04` mungkin muncul dan itu normal.** Pengukuran live (`0x0002`–`0x001F`), frekuensi, PF, dan puncak diambil dari IC metering lewat SPI saat jawaban disusun. Kalau pembacaan SPI itu gagal, meter mengembalikan exception `0x04` untuk seluruh permintaan — **bukan** data basi atau nol. Jadi `g_pm_last_exception = 4` sesekali bukan tanda kerusakan bus; yang perlu dicurigai adalah kalau terus-menerus.
2. **Membaca register energi tidak menghapusnya.** Meter menguras akumulator IC-nya sendiri tiap detik dan menyimpan total berjalan, jadi berapa pun master yang polling akan melihat angka yang sama. Aman untuk multi-master.

**Konstanta meter: jangan pakai angka nameplate.** Kalau nanti cacah pulsa CF dipakai, konversinya `kWh = pulsa × 1000 / reg(0x0042)` — pakai `g_pm_meter_const`, bukan konstanta yang dikonfigurasi. Pembagi CF di meter adalah bilangan bulat, jadi laju yang diminta hanya terpenuhi persis bila pembagiannya kebetulan bulat; selain itu laju nyatanya bergeser.

### SinePower ST36

Sumber: `ST series boards Communication protocol.pdf` bagian **III.3 halaman 5** — *"Query the running mode information (read only): 0X03"*. (Halaman 4 berisi tabel **parameter setelan** `0x1006`–`0x1020` — konfigurasi, bukan data ukur.)

**Terverifikasi di hardware (2026-08-19): `g_st36_online = 1`.** Kedua device di bus USART2 terbaca bersamaan — `g_pm_online = 1` dan `g_st36_online = 1`.

**Alamat slave = 5**, diset lewat register `0x102F` ST36 (default pabrik 1). Tidak bentrok dengan Power Meter di alamat 9 pada bus yang sama.

Dibaca dengan **FC `0x03`, bukan `0x04`** — protokol ST36 hanya mengenal `0x03` dan `0x06`; `0x04` tidak ada sama sekali di dokumennya. Enam register berurutan, semuanya **16-bit satu register** (berbeda dari Power Meter yang berpasangan 32-bit).

| Register | Variabel | Isi |
|---|---|---|
| `0x1029`–`0x102B` | `g_st36_cur_u/v/w` | Arus fase U / V / W (nilai trafo arus) |
| `0x102C` | `g_st36_idc` | Umpan balik arus — nilai DC |
| `0x102D` | `g_st36_vdc` | Umpan balik tegangan — nilai DC |
| `0x102E` | `g_st36_fault` | Kode gangguan 0–13, lihat `ST36_Fault_t` |

Kode gangguan: 0 tidak ada · 1 phase loss · 2 IF overload · 3 IF overcurrent · 4 CT overload · 5 CT overcurrent · 6 over voltage · 7 under voltage · 8 arus tiga fase tak seimbang · 9 urutan fase salah · 10 over heat · 11 error komunikasi · 12 feedback fault · 13 error frekuensi.

> **WAJIB — format serial ST36 harus 8-N-1.** Default pabrik register parity `0x1031` adalah **`0` = "no check, 2 stop bytes"** (8-N-2), sementara Power Meter 8-N-1 dan **tidak bisa diubah**. Dua format berbeda tidak bisa hidup di satu bus RS485. Set `0x1031 = 3` ("no check, 1 stop byte") dan baud `0x1030 = 2` (9600) di ST36 saat commissioning. Gejala kalau terlewat: ST36 tidak pernah menjawab sama sekali — bukan data salah — karena permintaan kita ditolaknya sebagai framing error.

> **SKALA BELUM DIKALIBRASI.** Dokumen protokol hanya menulis *"Transformer current value"*, *"DC current value"*, *"DC voltage value"* — **tanpa satuan dan tanpa faktor skala**. Karena itu nilai mentah disimpan apa adanya di `g_st36_*_raw`, dan `g_st36_i_scale` / `g_st36_v_scale` sengaja dibiarkan `1.0`. **Nilai terskala belum boleh dipakai untuk kontrol.** Kalibrasi: jalankan rectifier, bandingkan `g_st36_vdc_raw` dengan tegangan di panel ST36, isi rasionya ke `g_st36_v_scale`; sama untuk arus.

**Register kendali — sudah didefinisikan, belum dipakai.** `0x1027` run/stop (0=stop, 1=run) dan `0x1028` setpoint 0–1000 (persen). Memakainya butuh job bertipe **tulis FC `0x06`** yang belum ada di master.

### 2.4 RS485 #2 — Modbus **Slave** (HMI)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PC6 | `USART6_TX` | `USART6_TX` | AF8 | Ke pin DI transceiver |
| PC7 | `USART6_RX` | `USART6_RX` | AF8 | Dari pin RO transceiver |
| PC8 | `GPIO_Output` | `DO_RS485_2_DE` | Push-Pull, default **LOW** | **DE/RE transceiver** — HIGH = TX, LOW = RX |

- Baudrate saat ini di `.ioc`: **38400**, 8-N-1. DMA TX = `DMA2_Stream6`, IRQ prio 14.
- Sebagai slave, DE harus LOW (mode listen) selama idle agar tidak menabrak bus master HMI.

**Implementasi Modbus RTU Slave.** Driver: `Core/Src/modbus_slave.c`. Alamat slave default **1**, bisa diubah runtime lewat `g_mb_hmi.addr` atau holding register 2. Function code yang didukung: **01, 02, 03, 04, 05, 06, 0F, 10**.

| Keputusan | Isi | Alasan |
|---|---|---|
| Batas frame | **IDLE line interrupt UART** | Modbus RTU memisahkan frame dengan diam 3.5 karakter; IDLE memicu setelah diam 1 karakter (~286 µs @38400). Tidak butuh timer maupun DMA RX tambahan |
| Kendali DE | Naik sebelum transmit, turun di `HAL_UART_TxCpltCallback()` | HAL memanggil callback itu setelah flag **TC**, bukan TXE — byte terakhir dijamin sudah keluar dari shift register. Menurunkannya di TXE akan memotong byte terakhir di kabel |
| Isi ISR | Hanya catat panjang + naikkan bendera | CRC dan penguraian dikerjakan di main loop, supaya tidak menahan DRDY ADS131M04 (16 kSPS) |
| Urutan buffer | Salin ke buffer kerja **lalu** arm ulang RX | Waktu tuli sependek mungkin tanpa isi buffer tertimpa frame berikutnya |
| Error UART | `HAL_UART_ErrorCallback` meng-arm ulang RX | Tanpa ini, satu derau di jalur RS485 membuat slave **tuli permanen** |
| FC 0x10 | Periksa seluruh nilai dulu, baru terapkan | Modbus mensyaratkan penulisan banyak register bersifat semua-atau-tidak — jangan separuh setelan berubah lalu sisanya ditolak |
| Broadcast (alamat 0) | Dieksekusi, **tidak dijawab** | Kalau semua slave menjawab, bus bertabrakan |

**Kapasitas register: 300 Input + 300 Holding.** Isinya disimpan di dua array — `g_mb_input[300]` dan `g_mb_hold[300]` — dan **array itulah sumber data** saat permintaan baca dilayani. Alamat di awal "hidup": nilainya disegarkan dari variabel driver **tepat sebelum permintaan baca diproses**, bukan periodik, sehingga selalu segar tanpa membebani main loop saat bus sepi. Alamat sisanya penyimpanan biasa yang bebas dipakai kode aplikasi maupun ditulis HMI.

Kedua array sengaja global, bukan di dalam `MB_Slave_t` — slave HMI dan slave Orange Pi nanti harus melihat data yang sama persis. Biaya RAM keseluruhan modul: **2012 byte**.

> **Batas protokol:** satu permintaan baca maksimum **125 register** (FC 03/04) dan satu penulisan maksimum **123 register** (FC 10). Membaca seluruh 300 register perlu **3 permintaan** — itu batas Modbus, bukan batasan implementasi ini.

> **PETA REGISTER MASIH PROVISIONAL — harus dicocokkan dengan project HMI.** Angka di bawah usulan awal yang masuk akal, bukan standar. Semua alamat berbasis 0. Definisinya ada di `enum` pada `modbus_slave.h` sehingga mudah digeser.

**Discrete Input (FC 02)** — 16 kanal: `0–7` = GPA0–7, `8–15` = GPB0–7 chip DI.
**Coil (FC 01/05/0F)** — 8 kanal: `0–7` = GPA0–7 chip DO (relay).

| Input Register (FC 04) | Isi | Skala |
|---|---|---|
| 0 / 1 | Rectifier Voltage / Current | ×100, int16 |
| 2 / 3 | Battery Voltage / Current | ×100, int16 |
| 4 / 5 | Load Voltage / Current (ADS1115) | ×100, int16 |
| 6 / 7 | Bitmap DI (16 bit) / DO (8 bit) | — |
| 8 | Status device (bit: EEPROM, ADS1115, DAC, IOEXP DO, IOEXP DI, ADS131M04, ADC run) | — |
| 9 / 10 | Laju sampel ADC terukur / cacah overrun | — |
| 11 | Uptime detik (berputar di 65535) | — |
| **12–299** | **Cadangan** — penyimpanan `g_mb_input[]`, diisi kode aplikasi | — |

| Holding Register (FC 03/06/10) | Isi | Rentang |
|---|---|---|
| 0 | Setpoint DAC MCP4725 | 0–4095 |
| 1 | Bitmap 8 relay sekaligus | 0–255 |
| 2 | Alamat slave | 1–247 |
| **3–299** | **Cadangan** — HMI bebas menulis/membaca `0–65535`, belum berefek ke hardware | 0–65535 |

Skala ×100 pada int16 memberi rentang ±327,67 — cukup untuk tegangan sampai 327 V dan arus 327 A. Nilai di luar rentang **dijepit, bukan dilipat**, supaya HMI tidak menampilkan angka negatif palsu saat terjadi lonjakan.

**Kalau `g_mb_hmi.crc_err_count` merangkak naik padahal kabel bagus**, tersangka pertama adalah IDLE line yang memicu lebih cepat dari 3.5 karakter: master yang menyisipkan jeda di tengah frame membuat frame terbelah. Datanya tidak salah — potongan itu gagal CRC dan tidak dijawab, master mengulang. Jalan keluarnya memakai **TIM7** (masih bebas) sebagai pewaktu 3.5 karakter.

### 2.5 TTL — Modbus **Slave** (Orange Pi Zero3)

| Pin | Signal | Label | Mode | Keterangan |
|---|---|---|---|---|
| PA9 | `USART1_TX` | `USART1_TX` | AF7 | TTL 3V3 langsung, tanpa transceiver |
| PA10 | `USART1_RX` | `USART1_RX` | AF7 | TTL 3V3 langsung, tanpa transceiver |
| PA11 | `GPIO_Output` | `DO_OPI_PWR` | Push-Pull | Kontrol power Orange Pi (pin eksisting, dipertahankan) |

- Baudrate saat ini di `.ioc`: **38400**, 8-N-1. DMA TX = `DMA2_Stream7`, IRQ prio 14.
- Tidak butuh pin DE karena point-to-point TTL (half-duplex arbitration tidak diperlukan).

**Implementasi Modbus RTU Slave — instance kedua dari inti yang sama.** Driver: `Core/Src/modbus_slave.c`, instance `g_mb_opi`. Dibuat lewat `MB_SlaveInit()` dengan **`de_port = NULL`**; seluruh sisa inti slave berjalan identik dengan HMI — deteksi frame lewat IDLE line, arm-ulang saat error UART, FC `01/02/03/04/05/06/0F/10`, semuanya sama.

Alamat slave **10** (`MB_OPI_DEFAULT_ADDR`). Secara teknis boleh saja sama dengan HMI karena jalurnya terpisah dan USART1 point-to-point hanya punya satu slave — alamat di sini penanda protokol, bukan pembeda di bus bersama. Tetap dibuat berbeda agar setiap port punya alamat unik.

**Peta alamat Modbus seluruh board:**

| Port | Peran | Alamat |
|---|---|---|
| USART6 (RS485) | Slave → HMI | **1** |
| USART1 (TTL) | Slave → Orange Pi Zero3 | **10** |
| USART2 (RS485) | Master → SinePower ST36 | **5** |
| USART2 (RS485) | Master → Power Meter ADE7868A | **9** |

Semuanya unik, jadi tidak ada kerancuan saat menelusuri masalah — nomor alamat pada sebuah frame langsung menunjuk satu perangkat.

> **Kedua slave berbagi `g_mb_input[]` dan `g_mb_hold[]` yang sama — disengaja.** HMI dan Orange Pi harus melihat angka yang sama persis, bukan salinan yang bisa menyimpang. Konsekuensinya penulisan dari satu sisi langsung terlihat di sisi lain; penulis terakhir yang menang.
>
> Pengecualian `MB_HR_SLAVE_ADDR`: register itu selalu memantulkan alamat instance yang **sedang menjawab**, dan penulisan ke sana hanya mengubah alamat instance itu saja.

Biaya RAM modul Modbus slave dengan dua instance: **2824 byte** (600 register × 2 byte = 1200, plus 3 buffer × 256 byte per instance).

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

- **KOREKSI sample rate (2026-08-19).** Versi awal dokumen ini menulis *"64 kSPS → OSR = 64 (`MODE.OSR = 000`)"*. Dua hal keliru di situ:
  1. **OSR ada di register `CLOCK` bit [4:2], bukan `MODE`.**
  2. **OSR = 64 bukan pilihan yang tersedia.** Opsinya hanya 128 / 256 / 512 / 1024 / 2048 / 4096 / 8192 / 16256 (kode `000`–`111`), dengan 1024 sebagai default pabrik.

  Karena `fDATA = fMOD / OSR` dan `fMOD = fCLKIN / 2`, laju maksimum sepenuhnya ditentukan kristal CLKIN di board:

  **CLKIN board = 8.192 MHz — terkonfirmasi lewat pengukuran (2026-08-19).** Maka fMOD = 4.096 MHz:

  | OSR (kode) | fDATA teoretis | Terukur |
  |---|---|---|
  | 128 (`000`) | 32000 | — (lihat peringatan di bawah) |
  | **256 (`001`)** | **16000** | **15791 (−1.3%) ← titik operasi** |
  | 512 (`010`) | 8000 | |
  | 1024 (`011`, default pabrik) | 4000 | |

  Selisih −1.3% berasal dari `HAL_GetTick()` yang jamnya dari **HSI** (lihat catatan HSI di bagian 6). Memperpanjang jendela ukur tidak menambah akurasi — galat HSI sistematis, bukan acak.

  > **PERINGATAN — `g_adc_sps_measured` tidak sahih selama `g_adc_overrun_count` naik.** Pada OSR 128 pernah terbaca **46656**, dan angka itu sempat menghasilkan kesimpulan keliru bahwa CLKIN ≈ 12 MHz. Itu artefak: dalam kondisi overrun 31%, DRDY tidak pernah dilayani tepat waktu sehingga tepinya terhitung ganda. Laju sebenarnya di OSR 128 adalah 32000. **Baca laju hanya saat overrun berhenti bertambah.**

  **Target 64 kSPS tidak tercapai** — itu butuh CLKIN 16.384 MHz. Laju maksimum board ini 32 kSPS (OSR 128), tapi titik operasi yang dipakai adalah **16 kSPS (OSR 256)** karena alasan di bawah.
- SPI Mode **1** (CPOL=0, CPHA=1) — sesuai `SPI1.CLKPolarity = LOW`, `SPI1.CLKPhase = 2EDGE`. **Sudah benar.**
- Word size 24-bit; satu frame = STATUS + 4× data + CRC = **6 word × 24 bit = 144 bit**.
- DMA: TX = `DMA2_Stream3`, RX = `DMA2_Stream2`, IRQ `SPI1_IRQn` prio 10.
- Alur baca: EXTI PA8 falling → CS LOW → `HAL_SPI_TransmitReceive_DMA()` 18 byte → callback → CS HIGH → parse.

**Konfigurasi terverifikasi di hardware (2026-08-19).** `g_adc_present = 1`, `g_adc_init_stage = 10`, tulis-baca `GAIN1` cocok di `0x2020` (gain 1/4/1/4 sesuai FSR ±1.2 V / ±300 mV). Driver: `Core/Src/ads131m04.c`.

| Item | Nilai terverifikasi |
|---|---|
| Frame | 6 word × 24-bit = **18 byte** (1 response + 4 data + 1 CRC) |
| `WLENGTH` | 24-bit — dibaca dari STATUS, bukan diasumsikan |
| Opcode RREG / WREG | `0xA000` / `0x6000`, alamat di bit [12:7], count−1 di bit [6:0] |
| Ack WREG | `010a aaaa annn nnnn` — teramati `0x4180` untuk register CLOCK (`0x03`) |
| Kunci register | Device **tidak terkunci** setelah reset (STATUS bit15 = 0); UNLOCK tetap dikirim sebagai pengaman |

> **CATATAN HARDWARE — board ADS131M04 pertama cacat.** Pada board versi lama, penulisan register **intermiten**: perintah yang sama kadang mendarat kadang tidak (`GAIN1` terbaca `0x0000` atau `0x2020` bergantian), sementara pembacaan selalu normal. Semua register terbaca benar dan chip menjawab, jadi gejalanya mudah disangka bug protokol. **Diselesaikan dengan mengganti board ke versi terbaru**, bukan dengan perubahan firmware.
>
> Pelajarannya untuk perbaikan berikutnya: pola "baca selalu benar, tulis kadang-kadang" pada bus tanpa ACK adalah tanda hardware, bukan tanda protokol salah — protokol yang keliru gagal secara konsisten. Firmware tetap menyimpan tiga pengaman yang dibuat saat menelusuri masalah ini: jeda ~200 ns di tepi CS, kecepatan SPI terpisah untuk konfigurasi (3.125 Mbit/s) dan streaming (12.5 Mbit/s), serta retry ber-verifikasi pada penulisan register. **`g_adc_wreg_retry_count` adalah alat ukurnya** — kalau angka itu merangkak naik di board mana pun, jalurnya marginal dan retry hanya menutupi.
>
> **Board baru terukur bersih: `g_adc_wreg_retry_count = 0`** — setiap penulisan register mendarat pada percobaan pertama. Ketiga pengaman di atas jadi cadangan, bukan penopang.

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
| **PB4** | **GPIO_EXTI4** falling, NoPull | `DI_IOEXP_INTA` | **INTA MCP23S17 (Input) — port A** | ● |
| **PB5** | **GPIO_EXTI5** falling, NoPull | `DI_IOEXP_INTB` | **INTB MCP23S17 (Input) — port B** | ● |
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

- **Sumber clock = HSI (RC internal), bukan kristal — ini keputusan desain, board tidak memakai HSE.** Konsekuensinya semua baudrate USART diturunkan dari RC internal yang akurasinya bergantung suhu (STM32F412: ±1% @25 °C, melebar ke beberapa persen di rentang suhu penuh), sedangkan UART 8-N-1 asinkron hanya toleran ±2–3% total error dua sisi. Implikasi untuk tahap Modbus nanti:
  - Pertahankan **oversampling 16×** (default) di ketiga USART — jangan pindah ke 8× yang toleransinya lebih sempit.
  - Baudrate tinggi memperbesar error relatif. 38400 di USART1/USART6 masih aman; hindari menaikkan ke 115200 tanpa uji suhu.
  - Kalau muncul CRC/framing error Modbus yang muncul-hilang mengikuti suhu board, kalibrasi lewat `RCC->CR.HSITRIM` (±0.4%/step) sebelum menyalahkan sisi lawan.
  - Timing SPI1/SPI3 dan tick TIM6 ikut menyimpang persen yang sama — tidak masalah untuk ADS131M04 (SPI sinkron, clock ikut master), tapi jam sistem/jam energi jangan diandalkan dari TIM6 tanpa koreksi.
- **PA11/PA12 adalah USB_DM/USB_DP.** Dipakai sebagai GPIO biasa; pastikan USB OTG tidak diaktifkan di CubeMX.
- **PB3 & PA15 adalah JTDO/JTDI.** Aman karena debug memakai Serial Wire (PA13/PA14). Namun `MX_GPIO_Init()` harus dipanggil setelah `HAL_Init()` — jangan tarik LOW saat boot bila board pernah dipakai JTAG penuh.
- **PC14/PC15 adalah OSC32_IN/OUT.** Dipakai GPIO → **LSE tidak tersedia**, RTC (jika nanti dibutuhkan) harus pakai LSI dengan akurasi rendah.
- **PB2 = BOOT1.** Jika nanti dipakai, pastikan tidak mengganggu boot mode.
- **Konflik shadow label lama.** Beberapa label `.ioc` menyiratkan dual-function (`DO_LVD_BTS/SPI3_CS1`, `DI_DOOR_SPI3_CS0`, `DI_GST_RUN_SPI3_SCK`). Setelah SPI3 dipakai penuh untuk MCP23S17, fungsi LVD/DOOR/GST **harus dipindah ke kanal MCP23S17**, bukan ke pin MCU.
- **`DI_MCB_BTS` (PB4) dan `DI_MCB_VSAT` (PB5) kehilangan pin MCU.** Kedua pin itu kini dipakai INTA/INTB chip DI. Status MCB BTS & MCB VSAT **harus dipindah ke kanal MCP23S17 Digital Input**, sama seperti LVD/DOOR/GST di atas. Belum ada kode yang memakai kedua label lama itu saat pemindahan dilakukan, jadi tidak ada yang rusak — tapi fungsinya wajib dialokasikan ulang sebelum daftar kanal DI difinalkan.
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
- **Anggaran waktu ADS131M04 — biaya perangkat lunak jauh lebih besar dari waktu di kabel.** Ini pelajaran terpenting dari bring-up ADC ini.

  | Komponen | Waktu |
  |---|---|
  | Transfer 144 bit @12.5 Mbit/s (di kabel) | 11.5 µs |
  | Overhead: masuk/keluar ISR + `HAL_SPI_TransmitReceive_DMA()` + jalur penyelesaian DMA | **≈10 µs** |
  | **Total waktu layan per frame** | **≈21 µs** |

  `HAL_SPI_TransmitReceive_DMA()` memeriksa parameter, mengambil lock, dan mengonfigurasi dua stream DMA — masing-masing menonaktifkan stream lalu menunggu bit EN benar-benar turun; di sisi penyelesaian `SPI_EndRxTxTransaction()` menunggu flag BSY sebelum callback dipanggil. Perhitungan margin yang hanya menghitung bit di kabel akan meleset hampir 2×.

  **Terbukti di hardware:** pada 32 kSPS (periode 31.25 µs) waktu layan ~21 µs ternyata terlalu dekat ke periodenya — hasilnya **31% konversi terlewat** (`g_adc_overrun_count` naik terus). Bukan 50%, karena waktunya berfluktuasi di sekitar batas: kadang muat, kadang tidak. Pada **16 kSPS (periode 62.5 µs) overrun berhenti sama sekali** → margin ~66%. Itulah kenapa titik operasi dipilih OSR 256, bukan OSR 128 yang lebih cepat.

  Bonusnya: OSR lebih tinggi berarti modulator merata-ratakan lebih banyak, jadi **noise per sampel lebih rendah**. Untuk pengukuran V/I DC, 16 kSPS dengan noise lebih kecil lebih berguna daripada 32 kSPS yang sepertiganya hilang tak beraturan — pelewatan yang tidak seragam juga membuat isi jendela rata-rata jadi tidak konsisten.

  **Jangan turunkan `g_adc_presc_run`.** Di 6.25 Mbit/s transfer jadi 23 µs; masih muat di periode 62.5 µs, tapi tidak ada gunanya — 12.5 Mbit/s sudah terbukti andal di board ini dan margin lebih longgar selalu lebih baik. Agar tidak overrun: (a) gunakan **DMA** (sudah dikonfigurasi: `DMA2_Stream2` RX / `DMA2_Stream3` TX), jangan `HAL_SPI_TransmitReceive()` blocking; (b) jaga ISR `EXTI9_5` sesingkat mungkin — cukup turunkan CS dan start DMA, parsing dilakukan di main loop; (c) jika margin terbukti kurang saat diuji, turunkan sample rate ke 32 kSPS (`OSR = 128`) — untuk monitoring tegangan/arus rectifier, 32 kSPS masih sangat memadai.
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

SPI3 (PC10/11/12) ─┬─ MCP23S17 #1 (DI)  CS=PD2   RST=PB3    16 kanal @ PORT A+B
                   │                     INTA=PB4 (EXTI4)  INTB=PB5 (EXTI9_5)
                   └─ MCP23S17 #2 (DO)  CS=PA15  RST=PA12   8 kanal @ PORT A (GPA0-7)

USART2 (PA2/PA3)  ─── RS485 DE=PA4  → Modbus MASTER → Power Meter + SinePower ST36
USART6 (PC6/PC7)  ─── RS485 DE=PC8  → Modbus SLAVE  → HMI
USART1 (PA9/PA10) ─── TTL 3V3       → Modbus SLAVE  → Orange Pi Zero3
```
