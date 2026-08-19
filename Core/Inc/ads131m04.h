/**
  ******************************************************************************
  * @file    ads131m04.h
  * @brief   Driver ADS131M04 - ADC 24-bit 4 kanal simultan (SPI) - Recticon
  *
  * Hardware:
  *   - Bus   : SPI1 (PA5 SCK / PA6 MISO / PA7 MOSI), Mode 1, 12.5 Mbit/s
  *   - CS    : PC9  (aktif LOW)
  *   - RST   : PB9  (aktif LOW)
  *   - DRDY  : PA8  (aktif LOW, EXTI falling, vektor EXTI9_5 prio 10)
  *   - DMA   : TX = DMA2_Stream3, RX = DMA2_Stream2
  *
  * Pemetaan kanal (sesuai prompt.txt):
  *   CH0 = Rectifier Voltage   FSR +-1.2 V     gain 1
  *   CH1 = Rectifier Current   FSR +-300 mV    gain 4
  *   CH2 = Battery Voltage     FSR +-1.2 V     gain 1
  *   CH3 = Battery Current     FSR +-300 mV    gain 4
  *
  * FRAME. Semua komunikasi berbentuk frame utuh, bukan byte lepas. Untuk
  * 4 kanal aktif dengan word 24-bit:
  *     word0 = command / response
  *     word1..4 = data CH0..CH3
  *     word5 = CRC keluaran device
  *   -> 6 word x 3 byte = 18 byte per frame.
  * Jawaban sebuah command SELALU muncul di word0 frame BERIKUTNYA, jadi baca
  * register butuh dua frame.
  *
  * LAJU SAMPEL. fDATA = fMOD / OSR, dengan fMOD = fCLKIN / 2. Pilihan OSR
  * hanya 128/256/.../16256 - TIDAK ADA OSR 64.
  *
  * CLKIN board ini = 8.192 MHz -> fMOD = 4.096 MHz. Terkonfirmasi lewat
  * pengukuran BERSIH di OSR 256: teoretis 16000 SPS, terukur 15791 (-1.3%,
  * sesuai galat HSI yang jadi acuan HAL_GetTick).
  *
  * Jangan percaya g_adc_sps_measured saat g_adc_overrun_count sedang naik.
  * Pembacaan 46656 di OSR 128 pernah membuat CLKIN disangka ~12 MHz, padahal
  * itu artefak: dalam kondisi overrun 31%, DRDY tidak pernah dilayani tepat
  * waktu dan tepinya terhitung ganda. Laju sebenarnya di OSR 128 = 32000.
  *
  * Titik operasi = OSR 256 -> 16 kSPS. Bukan yang tercepat, tapi 32 kSPS
  * melampaui kemampuan layan (lihat catatan di g_adc_osr_sel). Target
  * 64 kSPS tetap tidak mungkin; itu butuh CLKIN 16.384 MHz.
  *
  * Anggaran waktu di 16 kSPS: periode DRDY 62.5 us, satu frame 18 byte
  * @12.5 Mbit/s = 11.5 us di kabel + ~10 us overhead HAL DMA = ~21 us.
  * Margin ~66%.
  ******************************************************************************
  */

#ifndef __ADS131M04_H
#define __ADS131M04_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Bentuk frame ------------------------------------------------------------- */
#define ADC_CH_COUNT        4U
#define ADC_WORD_BYTES      3U    /* WLENGTH = 24-bit                        */
#define ADC_FRAME_WORDS     6U    /* 1 cmd/resp + 4 data + 1 CRC             */
#define ADC_FRAME_BYTES     (ADC_FRAME_WORDS * ADC_WORD_BYTES)   /* = 18 */

#define ADC_SPI_TIMEOUT_MS  100U

/* Command ------------------------------------------------------------------ */
#define ADC_CMD_NULL        0x0000U
#define ADC_CMD_RESET       0x0011U
#define ADC_CMD_STANDBY     0x0022U
#define ADC_CMD_WAKEUP      0x0033U
#define ADC_CMD_LOCK        0x0555U
#define ADC_CMD_UNLOCK      0x0655U

/* RREG/WREG: alamat di bit 12..7, jumlah register - 1 di bit 6..0 */
#define ADC_CMD_RREG(a, n)  ((uint16_t)(0xA000U | (((uint16_t)(a) & 0x3FU) << 7) | (((uint16_t)(n) - 1U) & 0x7FU)))
#define ADC_CMD_WREG(a, n)  ((uint16_t)(0x6000U | (((uint16_t)(a) & 0x3FU) << 7) | (((uint16_t)(n) - 1U) & 0x7FU)))

/* Register ----------------------------------------------------------------- */
#define ADC_REG_ID          0x00U
#define ADC_REG_STATUS      0x01U
#define ADC_REG_MODE        0x02U
#define ADC_REG_CLOCK       0x03U
#define ADC_REG_GAIN1       0x04U
#define ADC_REG_CFG         0x06U
#define ADC_REG_CH0_CFG     0x09U
#define ADC_REG_CH1_CFG     0x0EU
#define ADC_REG_CH2_CFG     0x13U
#define ADC_REG_CH3_CFG     0x18U

/* Field MODE */
#define ADC_MODE_RX_CRC_EN  0x1000U
#define ADC_MODE_WLENGTH_Msk 0x0300U
#define ADC_MODE_WLENGTH_24  0x0100U   /* 01 = 24-bit */

/* Field CLOCK */
#define ADC_CLOCK_CHEN_Msk  0x0F00U    /* CH3..CH0 enable, bit 11..8 */
#define ADC_CLOCK_OSR_Msk   0x001CU    /* OSR[2:0], bit 4..2         */
#define ADC_CLOCK_OSR_Pos   2U
#define ADC_CLOCK_PWR_Msk   0x0003U    /* PWR[1:0], bit 1..0         */
#define ADC_CLOCK_PWR_HR    0x0002U    /* 10 = High Resolution       */

/** Kode OSR untuk g_adc_osr_sel. Nilai OSR-nya, bukan laju sampel - laju
    tergantung CLKIN board (lihat catatan di kepala file). */
typedef enum
{
  ADC_OSR_128   = 0,   /* paling cepat */
  ADC_OSR_256   = 1,
  ADC_OSR_512   = 2,
  ADC_OSR_1024  = 3,   /* default chip */
  ADC_OSR_2048  = 4,
  ADC_OSR_4096  = 5,
  ADC_OSR_8192  = 6,
  ADC_OSR_16256 = 7
} ADC_Osr_t;

/** Kode gain PGA untuk g_adc_gain_code[]. Nilai = 2^kode. */
typedef enum
{
  ADC_GAIN_1   = 0,
  ADC_GAIN_2   = 1,
  ADC_GAIN_4   = 2,
  ADC_GAIN_8   = 3,
  ADC_GAIN_16  = 4,
  ADC_GAIN_32  = 5,
  ADC_GAIN_64  = 6,
  ADC_GAIN_128 = 7
} ADC_Gain_t;

/** Tegangan referensi internal. FSR tiap kanal = +-VREF / gain. */
#define ADC_VREF_V          1.2f

/** Skala kode 24-bit two's complement: +-2^23. */
#define ADC_FULLSCALE_CODE  8388608.0f

/* Status ------------------------------------------------------------------- */
typedef enum
{
  ADC_OK           =  0,
  ADC_ERR_PARAM    = -1,
  ADC_ERR_NOT_INIT = -2,
  ADC_ERR_SPI      = -3,
  ADC_ERR_VERIFY   = -4,   /* register tidak terbaca sesuai yang ditulis */
  ADC_ERR_BUSY     = -5
} ADC_Status_t;

/* Perintah untuk g_adc_cmd ------------------------------------------------- */
typedef enum
{
  ADC_CMD_IDLE      = 0,
  ADC_CMD_DUMP_REGS = 1,  /* refresh seluruh g_adc_*_reg                     */
  ADC_CMD_REINIT    = 2,  /* reset hardware + konfigurasi ulang + start      */
  ADC_CMD_APPLY_CFG = 3,  /* terapkan g_adc_osr_sel & g_adc_gain_code[]      */
  ADC_CMD_START     = 4,  /* mulai akuisisi (DRDY dilayani)                  */
  ADC_CMD_STOP      = 5,  /* hentikan akuisisi, DRDY diabaikan               */
  ADC_CMD_PROBE_WREG = 6  /* sapu kandidat opcode WREG - lihat g_adc_probe_* */
} ADC_UserCmd_t;

/* Probe opcode WREG -------------------------------------------------------- */
#define ADC_PROBE_COUNT   4U

/** Berapa kali penulisan register diulang sebelum menyerah. */
#define ADC_WREG_MAX_TRIES  5U

/* ==========================================================================
 * Variabel LIVE EXPRESSIONS
 * ========================================================================== */

/** Sejauh mana ADC_Init() sempat berjalan - dibaca saat g_adc_present = 0. */
typedef enum
{
  ADC_STAGE_NONE       = 0,   /* ADC_Init() belum dipanggil               */
  ADC_STAGE_RESET      = 1,   /* gagal di frame pertama setelah reset     */
  ADC_STAGE_DUMP       = 2,   /* gagal saat membaca register              */
  ADC_STAGE_MODE       = 3,   /* gagal set/verifikasi MODE                */
  ADC_STAGE_GAIN       = 4,   /* gagal set/verifikasi GAIN1               */
  ADC_STAGE_CLOCK      = 5,   /* gagal set/verifikasi CLOCK               */
  ADC_STAGE_DONE       = 10   /* lulus semua                              */
} ADC_Stage_t;

/* --- DIBACA: status --- */
extern volatile uint8_t  g_adc_present;        /* 1 = konfigurasi terverifikasi */
extern volatile int8_t   g_adc_last_result;    /* ADC_Status_t terakhir         */
extern volatile uint8_t  g_adc_running;        /* 1 = akuisisi jalan            */
extern volatile uint8_t  g_adc_init_stage;     /* ADC_Stage_t - lihat di atas   */

/* --- DIBACA: bahan diagnosa saat gagal --- */

/** Salinan 18 byte frame terakhir yang diterima. Kalau seluruhnya 0x00,
    MISO tidak pernah didorong device (chip absen / CS salah / MISO putus);
    kalau seluruhnya 0xFF, MISO ketarik HIGH terus. */
extern volatile uint8_t  g_adc_rx_dump[ADC_FRAME_BYTES];

/** Nilai yang DIKIRIM ke GAIN1/CLOCK, untuk dibandingkan dengan yang terbaca
    di g_adc_gain1_reg / g_adc_clock_reg saat verifikasi gagal. */
extern volatile uint16_t g_adc_gain1_want;
extern volatile uint16_t g_adc_clock_want;

/** Jawaban device atas perintah UNLOCK (semestinya 0x0655 dipantulkan). */
extern volatile uint16_t g_adc_unlock_resp;

/**
  * Acknowledge WREG terakhir. Device menjawab 0b010a aaaa annn nnnn, jadi
  * untuk GAIN1 (alamat 0x04, 1 register) semestinya 0x4200.
  *   0x4200  -> perintah tulis DITERIMA, masalahnya di nilai yang nyangkut
  *   0x0000  -> perintah tulis TIDAK diterima sama sekali
  *   lainnya -> penomoran alamat atau penjajaran frame meleset
  */
extern volatile uint16_t g_adc_wreg_ack;

/**
  * Basis opcode WREG (3 bit teratas command word). Dibuat runtime supaya
  * hasil probe bisa langsung dipakai tanpa rebuild.
  *   RREG sudah terbukti bekerja di 0xA000 (101), jadi kandidat WREG adalah
  *   prefix lain: 0x6000 (011), 0x4000 (010), 0x2000 (001), 0x8000 (100).
  */
extern volatile uint16_t g_adc_wreg_base;

/**
  * Hasil ADC_CMD_PROBE_WREG. Untuk tiap kandidat basis opcode, driver
  * mencoba menulis 0x2020 ke GAIN1 lalu membacanya kembali:
  *   g_adc_probe_base[i] = basis yang dicoba
  *   g_adc_probe_ack[i]  = word jawaban device
  *   g_adc_probe_rb[i]   = isi GAIN1 setelah percobaan  <-- YANG MENENTUKAN
  * Kandidat yang benar adalah yang membuat g_adc_probe_rb[i] = 0x2020.
  */
extern volatile uint16_t g_adc_probe_base[ADC_PROBE_COUNT];
extern volatile uint16_t g_adc_probe_ack[ADC_PROBE_COUNT];
extern volatile uint16_t g_adc_probe_rb[ADC_PROBE_COUNT];

/**
  * Berapa kali penulisan register harus diulang sejak boot. Ini UKURAN
  * KUALITAS JALUR, bukan sekadar penghitung: 0 = jalur bersih; angka yang
  * terus naik berarti masalah intermiten masih ada dan cuma tertutupi oleh
  * retry - jangan dianggap selesai kalau nilainya merangkak.
  */
extern volatile uint32_t g_adc_wreg_retry_count;

/**
  * Prescaler SPI1 terpisah untuk dua fase kerja yang kebutuhannya berlawanan.
  *   g_adc_presc_cfg - akses register: jarang, tapi harus andal.
  *                     Default SPI_BAUDRATEPRESCALER_32 = 3.125 Mbit/s,
  *                     kecepatan yang sama dengan SPI3 yang terbukti stabil.
  *   g_adc_presc_run - streaming: harus selesai sebelum DRDY berikutnya.
  *                     Default SPI_BAUDRATEPRESCALER_8 = 12.5 Mbit/s
  *                     (18 byte = 11.5 us dari periode 31.25 us @32 kSPS).
  * Keduanya diterapkan saat ADC_CMD_REINIT / ADC_CMD_START.
  */
extern volatile uint32_t g_adc_presc_cfg;
extern volatile uint32_t g_adc_presc_run;

/* --- DIBACA: isi register mentah, untuk bring-up --- */
extern volatile uint16_t g_adc_id_reg;
extern volatile uint16_t g_adc_status_reg;
extern volatile uint16_t g_adc_mode_reg;
extern volatile uint16_t g_adc_clock_reg;
extern volatile uint16_t g_adc_gain1_reg;
extern volatile uint16_t g_adc_cfg_reg;
extern volatile uint16_t g_adc_reset_resp;     /* word0 frame pertama setelah reset */

/* --- DIBACA: hasil akuisisi --- */
extern volatile int32_t  g_adc_raw[ADC_CH_COUNT];   /* kode 24-bit sampel terakhir */
extern volatile int32_t  g_adc_avg[ADC_CH_COUNT];   /* rata-rata jendela terakhir  */
extern volatile float    g_adc_volt[ADC_CH_COUNT];  /* tegangan di pin ADC (Volt)  */

/* Besaran fisik setelah skala/offset dipakai */
extern volatile float    g_rect_voltage;   /* Volt  */
extern volatile float    g_rect_current;   /* Amper */
extern volatile float    g_batt_voltage;   /* Volt  */
extern volatile float    g_batt_current;   /* Amper */

/* --- DIBACA: penghitung & diagnosa --- */
extern volatile uint32_t g_adc_drdy_count;     /* total tepi DRDY dilayani      */
extern volatile uint32_t g_adc_sample_count;   /* total frame selesai di-DMA    */
extern volatile uint32_t g_adc_overrun_count;  /* DRDY datang saat DMA sibuk    */
extern volatile uint32_t g_adc_error_count;
extern volatile uint32_t g_adc_last_hal_error; /* HAL_SPI_GetError()            */

/**
  * Laju sampel NYATA hasil pengukuran, di-refresh tiap detik dari cacah DRDY.
  * Inilah jawaban atas pertanyaan CLKIN board: kalau OSR = 128 dan angka ini
  * ~32000, berarti CLKIN 8.192 MHz; kalau ~64000, berarti 16.384 MHz.
  */
extern volatile uint32_t g_adc_sps_measured;

/* --- DIUBAH dari Live Expressions --- */
extern volatile uint8_t  g_adc_cmd;            /* isi dengan ADC_UserCmd_t      */

/** OSR, isi dengan ADC_Osr_t lalu g_adc_cmd = 3 untuk menerapkan. */
extern volatile uint8_t  g_adc_osr_sel;

/** Gain PGA per kanal (ADC_Gain_t). Default {1, 4, 1, 4} sesuai FSR yang
    diminta: +-1.2 V untuk tegangan, +-300 mV untuk arus. */
extern volatile uint8_t  g_adc_gain_code[ADC_CH_COUNT];

/** Panjang jendela rata-rata dalam ms. Rata-rata dihitung di main loop dari
    akumulator yang diisi ISR, jadi tiap sampel ikut terhitung - bukan
    pencuplikan. Default 10 ms. */
extern volatile uint16_t g_adc_avg_ms;

/** Skala & offset ke besaran fisik: fisik = volt_adc * scale + offset.
    Isi sesuai pembagi tegangan dan shunt di board saat kalibrasi. */
extern volatile float    g_adc_rect_v_scale;
extern volatile float    g_adc_rect_v_offset;
extern volatile float    g_adc_rect_i_scale;
extern volatile float    g_adc_rect_i_offset;
extern volatile float    g_adc_batt_v_scale;
extern volatile float    g_adc_batt_v_offset;
extern volatile float    g_adc_batt_i_scale;
extern volatile float    g_adc_batt_i_offset;

/* API ---------------------------------------------------------------------- */

/**
  * @brief  Reset hardware, konfigurasi, verifikasi, lalu mulai akuisisi.
  *         Panggil setelah MX_SPI1_Init() dan MX_GPIO_Init().
  */
ADC_Status_t ADC_Init(SPI_HandleTypeDef *hspi);

/** @brief Baca satu register (2 frame: kirim RREG, lalu ambil jawabannya). */
ADC_Status_t ADC_ReadReg(uint8_t addr, uint16_t *val);

/** @brief Tulis satu register. Verifikasi lakukan dengan ADC_ReadReg(). */
ADC_Status_t ADC_WriteReg(uint8_t addr, uint16_t val);

/** @brief Terapkan g_adc_osr_sel dan g_adc_gain_code[] ke chip. */
ADC_Status_t ADC_ApplyConfig(void);

/** @brief Mulai / hentikan pelayanan DRDY. */
void ADC_Start(void);
void ADC_Stop(void);

/**
  * @brief  Dipanggil dari HAL_GPIO_EXTI_Callback() saat DRDY turun.
  *         Aman di ISR: hanya menurunkan CS dan menjalankan DMA.
  */
void ADC_OnDrdy(void);

/**
  * @brief  Dipanggil dari HAL_SPI_TxRxCpltCallback() saat frame selesai.
  *         Menaikkan CS, mem-parse 4 kanal, dan mengakumulasi rata-rata.
  */
void ADC_OnFrameComplete(void);

/** @brief Dipanggil dari HAL_SPI_ErrorCallback() untuk SPI1. */
void ADC_OnSpiError(void);

/**
  * @brief  Hitung rata-rata + konversi ke Volt/Amper, ukur laju sampel, dan
  *         jalankan g_adc_cmd. Panggil dari while(1).
  */
void ADC_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADS131M04_H */
