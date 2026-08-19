/**
  ******************************************************************************
  * @file    modbus_master.h
  * @brief   Modbus RTU Master - Recticon Rectifier Controller
  *
  * Hardware : USART2, RS485, DE = PA4, 9600 8-N-1, DMA TX = DMA1_Stream6
  * Device   : 1) Power Meter ADE7868A
  *            2) SinePower ST36   <-- peta registernya BELUM ada, lihat catatan
  *
  * CARA KERJA. Master menjalankan daftar job secara bergilir (round-robin):
  * kirim permintaan -> tunggu jawaban -> urai -> jeda antar-frame -> job
  * berikutnya. Semuanya non-blocking, digerakkan MBM_Task() dari main loop;
  * tidak ada satu pun penantian yang menahan CPU.
  *
  * ANGGARAN WAKTU @9600 BAUD. Ini bus paling lambat di board, dan responsnya
  * paling panjang - perlu diperhitungkan, bukan ditebak:
  *   - Jawaban 68 register = 3 + 136 + 2 = 141 byte
  *   - 141 byte x 10 bit / 9600 = ~147 ms
  * Karena itu timeout default 500 ms dan interval polling 1000 ms. Menaikkan
  * baud meter (lewat holding register 0x0000 miliknya) akan memperpendek ini
  * drastis, tapi lakukan belakangan - lihat peringatan di modbus_master.c.
  ******************************************************************************
  */

#ifndef __MODBUS_MASTER_H
#define __MODBUS_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define MBM_ADU_MAX          256U

/* ==========================================================================
 * Power Meter ADE7868A
 * ========================================================================== */

/** Blok input register meter: 0x0000..0x0043 = 68 register, muat 1 permintaan. */
#define PM_IR_BASE           0x0000U
#define PM_IR_COUNT          68U

/* Alamat penting di dalam blok (indeks = alamat, karena base = 0) */
#define PM_IR_SLAVE_ADDR     0x0000U  /* uint16, 1..16 dari DIP switch      */
#define PM_IR_FW_VERSION     0x0001U  /* uint16, saat ini 0x0001            */
#define PM_IR_PHASE_BASE     0x0002U  /* fase A; tiap fase 10 register      */
#define PM_IR_ENERGY_BASE    0x0020U  /* fase A; tiap fase 6 register       */
#define PM_IR_FREQUENCY      0x0032U  /* uint16, centi-hertz                */
#define PM_IR_PF_BASE        0x0033U  /* int16 x1000, per fase              */
#define PM_IR_METER_CONST    0x0042U  /* uint32, milli-imp/kWh              */

#define PM_PHASE_COUNT       3U

/**
  * Alamat slave meter = 9, terverifikasi di hardware.
  *
  * Ditentukan DIP switch 4-posisi di meter dan dibaca sekali saat meter boot;
  * alamat = nilai DIP + 1, jadi DIP 0..15 memetakan ke 1..16 dan tidak pernah
  * bentrok dengan alamat broadcast. Ubah di sini bila DIP-nya digeser -
  * atau saat runtime lewat g_pm_addr tanpa rebuild.
  *
  * Pemeriksaan silang: input register 0x0000 meter berisi alamat yang
  * dipakainya sendiri, jadi g_pm_raw[0] harus sama dengan nilai ini.
  */
#define PM_DEFAULT_ADDR      9U

/** Satu fase, sudah dikonversi dari milli-unit ke satuan teknik. */
typedef struct
{
  float voltage;      /* Volt   */
  float current;      /* Amper  */
  float active_w;     /* Watt   - negatif = daya mengalir balik ke sumber */
  float reactive_var; /* var    */
  float apparent_va;  /* VA     */
  float pf;           /* -1..+1 */
} PM_Phase_t;

/* ==========================================================================
 * Variabel LIVE EXPRESSIONS - Power Meter
 * ========================================================================== */

extern volatile uint16_t g_pm_raw[PM_IR_COUNT];   /* isi register mentah    */
extern volatile PM_Phase_t g_pm_phase[PM_PHASE_COUNT];
extern volatile float    g_pm_frequency;          /* Hz                     */
extern volatile int32_t  g_pm_energy_wh[PM_PHASE_COUNT];   /* Wh   netto    */
extern volatile int32_t  g_pm_energy_varh[PM_PHASE_COUNT]; /* varh netto    */
extern volatile uint32_t g_pm_meter_const;        /* milli-imp/kWh          */

extern volatile uint8_t  g_pm_online;             /* 1 = jawaban terakhir sah */
extern volatile uint8_t  g_pm_addr;               /* alamat slave, bisa diubah */
extern volatile uint8_t  g_pm_last_exception;     /* 0 = tidak ada          */
extern volatile uint32_t g_pm_ok_count;
extern volatile uint32_t g_pm_timeout_count;
extern volatile uint32_t g_pm_crc_err_count;
extern volatile uint32_t g_pm_exception_count;

/* ==========================================================================
 * SinePower ST36
 *
 * Sumber: "ST series boards Communication protocol.pdf" bagian III.3
 * (halaman 5) - "Query the running mode information (read only): 0X03".
 *
 * Berbeda dari Power Meter: SEMUA nilai di sini 16-bit satu register, tidak
 * ada pasangan 32-bit. Enam register berurutan, muat satu permintaan.
 * ========================================================================== */

#define ST36_IR_BASE         0x1029U
#define ST36_IR_COUNT        6U

#define ST36_IR_CUR_U        0x1029U  /* arus fase U (nilai trafo arus) */
#define ST36_IR_CUR_V        0x102AU  /* arus fase V                    */
#define ST36_IR_CUR_W        0x102BU  /* arus fase W                    */
#define ST36_IR_IF_CURRENT   0x102CU  /* umpan balik arus  - nilai DC    */
#define ST36_IR_VF_VOLTAGE   0x102DU  /* umpan balik tegangan - nilai DC */
#define ST36_IR_FAULT        0x102EU  /* kode gangguan, lihat ST36_Fault_t */

/**
  * Alamat slave ST36 = 5 (bukan default pabrik 1).
  *
  * Diset di ST36 lewat register 0x102F miliknya. Tidak bentrok dengan Power
  * Meter di alamat 9 yang berbagi bus USART2 yang sama.
  *
  * Format serial ST36 diset ke 9600 8-N-1 agar cocok dengan Power Meter yang
  * 8-N-1 mati (tidak bisa diubah). Ini WAJIB: default pabrik ST36 untuk
  * register parity 0x1031 adalah 0 = "no check, 2 stop bytes" alias 8-N-2,
  * dan dua format berbeda tidak bisa hidup di satu bus. Nilai yang benar
  * untuk 8-N-1 adalah 0x1031 = 3 ("no check, 1 stop byte").
  */
#define ST36_DEFAULT_ADDR    5U

/** Kode gangguan di register 0x102E. */
typedef enum
{
  ST36_FAULT_NONE            = 0,
  ST36_FAULT_PHASE_LOSS      = 1,
  ST36_FAULT_IF_OVERLOAD     = 2,
  ST36_FAULT_IF_OVERCURRENT  = 3,
  ST36_FAULT_CT_OVERLOAD     = 4,
  ST36_FAULT_CT_OVERCURRENT  = 5,
  ST36_FAULT_OVERVOLTAGE     = 6,
  ST36_FAULT_UNDERVOLTAGE    = 7,
  ST36_FAULT_CUR_UNBALANCE   = 8,
  ST36_FAULT_PHASE_SEQUENCE  = 9,
  ST36_FAULT_OVERHEAT        = 10,
  ST36_FAULT_COMM_ERROR      = 11,
  ST36_FAULT_FEEDBACK        = 12,
  ST36_FAULT_FREQUENCY       = 13
} ST36_Fault_t;

/* Register kendali - BELUM dipakai, untuk tahap kontrol nanti (FC 0x06) */
#define ST36_HR_RUN_STOP     0x1027U  /* 0 = stop, 1 = run              */
#define ST36_HR_SETPOINT     0x1028U  /* sinyal given, 0..1000 (persen) */
#define ST36_HR_COMM_ADDR    0x102FU  /* 1..247, default 1              */
#define ST36_HR_BAUD         0x1030U  /* 2 = 9600 (default pabrik)      */
#define ST36_HR_PARITY       0x1031U  /* lihat PERINGATAN di .c         */

/* --- Live Expressions ST36 --- */
extern volatile uint16_t g_st36_raw[ST36_IR_COUNT];  /* isi register mentah */

/** Nilai mentah per besaran - inilah yang sahih sampai skala dikalibrasi. */
extern volatile uint16_t g_st36_cur_u_raw;
extern volatile uint16_t g_st36_cur_v_raw;
extern volatile uint16_t g_st36_cur_w_raw;
extern volatile uint16_t g_st36_idc_raw;
extern volatile uint16_t g_st36_vdc_raw;
extern volatile uint16_t g_st36_fault;      /* ST36_Fault_t */

/**
  * Nilai terskala. PERINGATAN: dokumen protokol TIDAK menyebutkan satuan
  * maupun faktor skala untuk register-register ini - hanya menulis
  * "Transformer current value" / "DC current value" / "DC voltage value".
  * Karena itu skala di bawah default 1.0 dan angka ini SAMA dengan nilai
  * mentah sampai dikalibrasi terhadap tampilan panel ST36 yang sebenarnya.
  * Jangan dipakai untuk kontrol sebelum itu.
  */
extern volatile float    g_st36_cur_u;
extern volatile float    g_st36_cur_v;
extern volatile float    g_st36_cur_w;
extern volatile float    g_st36_idc;
extern volatile float    g_st36_vdc;

extern volatile float    g_st36_i_scale;   /* default 1.0 - HARUS dikalibrasi */
extern volatile float    g_st36_v_scale;   /* default 1.0 - HARUS dikalibrasi */

extern volatile uint8_t  g_st36_online;
extern volatile uint8_t  g_st36_addr;
extern volatile uint8_t  g_st36_last_exception;
extern volatile uint32_t g_st36_ok_count;
extern volatile uint32_t g_st36_timeout_count;
extern volatile uint32_t g_st36_crc_err_count;
extern volatile uint32_t g_st36_exception_count;

/* ==========================================================================
 * Diagnostik master (seluruh bus, bukan per device)
 * ========================================================================== */

extern volatile uint32_t g_mbm_tx_count;
extern volatile uint32_t g_mbm_rx_count;
extern volatile uint32_t g_mbm_uart_err_count;

/**
  * Jawaban yang DATANG tapi ditolak - alamat tidak cocok, function code
  * salah, panjang tidak sesuai, atau CRC gagal.
  *
  * Ini yang membedakan dua kegagalan yang gejalanya mirip:
  *   timeout naik, reject tetap 0  -> bus sunyi, tidak ada yang menjawab
  *   reject naik                   -> perangkat menjawab tapi jawabannya
  *                                    tidak sesuai yang kita minta
  * Tanpa pemisahan ini, jawaban yang ditolak hilang tanpa jejak.
  */
extern volatile uint32_t g_pm_reject_count;
extern volatile uint32_t g_st36_reject_count;

/**
  * Salinan mentah frame terakhir yang diterima master, apa pun hasilnya.
  * g_mbm_rx_dump_job menandai job mana yang sedang menunggu saat frame itu
  * tiba (0 = Power Meter, 1 = ST36).
  */
#define MBM_DUMP_LEN         24U
extern volatile uint8_t  g_mbm_rx_dump[MBM_DUMP_LEN];
extern volatile uint8_t  g_mbm_rx_dump_len;
extern volatile uint8_t  g_mbm_rx_dump_job;
extern volatile uint8_t  g_mbm_state;        /* MBM_State_t, untuk diintip  */
extern volatile uint8_t  g_mbm_job_index;    /* job yang sedang dikerjakan  */

/** Timeout menunggu jawaban (ms). Default 500 - lihat anggaran waktu di atas. */
extern volatile uint16_t g_mbm_timeout_ms;

/**
  * Jeda setelah satu transaksi selesai sebelum job berikutnya (ms).
  * Default 200 ms.
  *
  * Syarat Modbus RTU sendiri hanya diam 3.5 karakter - di 9600 itu ~3.7 ms.
  * 200 ms jauh di atas itu; dipilih supaya tiap slave punya waktu longgar
  * melepas transceiver-nya dan bus benar-benar bersih sebelum permintaan
  * berikutnya, ketimbang mengejar laju polling maksimum.
  */
extern volatile uint16_t g_mbm_gap_ms;

/** Jarak antar putaran polling penuh (ms). Default 1000. */
extern volatile uint16_t g_mbm_poll_ms;

typedef enum
{
  MBM_ST_IDLE   = 0,   /* menunggu jadwal polling berikutnya */
  MBM_ST_WAIT   = 1,   /* permintaan terkirim, menunggu jawaban */
  MBM_ST_GAP    = 2    /* jeda antar-frame                    */
} MBM_State_t;

/* API ---------------------------------------------------------------------- */

/** @brief Siapkan master dan mulai polling. Panggil setelah MX_USART2_UART_Init(). */
void MBM_Init(UART_HandleTypeDef *huart);

/** @brief Jalankan state machine. Panggil dari while(1). */
void MBM_Task(void);

/* Callback dari HAL - konteks ISR ------------------------------------------ */
void MBM_OnRxEvent(uint16_t size);
void MBM_OnTxComplete(void);
void MBM_OnUartError(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_MASTER_H */
