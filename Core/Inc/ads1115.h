/**
  ******************************************************************************
  * @file    ads1115.h
  * @brief   Driver ADS1115 - pembacaan Tegangan & Arus LOAD
  *          Recticon Rectifier Controller
  *
  * Hardware:
  *   - Bus     : I2C1 (PB6/PB7), 100 kHz
  *   - Alamat  : 0x4A 7-bit (pin ADDR -> SDA)
  *   - ALERT/RDY TIDAK dikabelkan -> memakai single-shot + tunggu berbasis tick
  *
  * Konfigurasi sesuai spesifikasi project:
  *   LOAD Voltage : MUX 0 (AIN0-AIN1 diff), FSR +/-2.048 V, 860 SPS
  *   LOAD Current : MUX 3 (AIN2-AIN3 diff), FSR +/-2.048 V, 860 SPS
  *
  * CATATAN pin ADDR -> SDA:
  *   Datasheet melarang SDA ditahan LOW lebih dari 100 us setelah START.
  *   Jangan aktifkan clock stretching, dan hindari halt debugger di tengah
  *   transaksi I2C - chip bisa salah mengenali alamatnya.
  ******************************************************************************
  */

#ifndef __ADS1115_H
#define __ADS1115_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Alamat & register -------------------------------------------------------- */
#define ADS_I2C_ADDR_7B      0x4AU   /* ADDR -> SDA */
#define ADS_REG_CONVERSION   0x00U
#define ADS_REG_CONFIG       0x01U
#define ADS_REG_LO_THRESH    0x02U
#define ADS_REG_HI_THRESH    0x03U

/* Bit field Config register (16-bit, MSB first) ---------------------------- */
#define ADS_OS_SINGLE        (1U << 15)  /* tulis 1 = mulai konversi          */
#define ADS_OS_NOT_BUSY      (1U << 15)  /* baca  1 = konversi selesai        */
#define ADS_MUX_Pos          12
#define ADS_PGA_Pos          9
#define ADS_MODE_SINGLE      (1U << 8)   /* 1 = single-shot                   */
#define ADS_DR_Pos           5
#define ADS_COMP_QUE_DISABLE (3U << 0)   /* comparator mati (ALERT tak dipakai)*/

/* Nilai MUX. Angka = nilai field, sesuai penomoran "MUX MODEx" di spesifikasi.
   Ubah di sini bila ternyata pengukuran dimaksudkan single-ended:
     AIN0-GND = 4, AIN1-GND = 5, AIN2-GND = 6, AIN3-GND = 7 */
#define ADS_MUX_LOAD_V       0U   /* MUX MODE0 = AIN0-AIN1 differential */
#define ADS_MUX_LOAD_I       3U   /* MUX MODE3 = AIN2-AIN3 differential */

#define ADS_PGA_2048MV       2U   /* FSR +/-2.048 V                     */
#define ADS_DR_860SPS        7U   /* 860 sample/detik                   */

/* Skala: FSR 2.048 V / 32768 = 62.5 uV per LSB */
#define ADS_LSB_UV           62.5f

/* Waktu konversi 860 SPS = 1.163 ms. Tick HAL beresolusi 1 ms dan bisa baru
   saja bertambah, jadi menunggu N tick hanya menjamin (N-1) ms nyata.
   Pakai 3 -> dijamin >= 2 ms, aman di atas 1.163 ms. */
#define ADS_CONV_WAIT_TICKS  3U

#define ADS_I2C_TIMEOUT_MS   100U
#define ADS_FILTER_LEN       8U    /* moving average, harus pangkat 2 */

/* Status ------------------------------------------------------------------- */
typedef enum
{
  ADS_OK           =  0,
  ADS_ERR_PARAM    = -1,
  ADS_ERR_NOT_INIT = -2,
  ADS_ERR_I2C      = -3,
  ADS_ERR_TIMEOUT  = -4
} ADS_Status_t;

typedef enum
{
  ADS_CH_LOAD_V = 0,
  ADS_CH_LOAD_I = 1,
  ADS_CH_COUNT  = 2
} ADS_Channel_t;

/* ==========================================================================
 * Variabel LIVE EXPRESSIONS
 * ========================================================================== */

/* --- status --- */
extern volatile uint8_t  g_ads_present;        /* 1 = chip menjawab di bus    */
extern volatile int8_t   g_ads_last_result;    /* ADS_Status_t terakhir       */
extern volatile uint32_t g_ads_sample_count;   /* total sampel sukses         */
extern volatile uint32_t g_ads_error_count;
extern volatile uint32_t g_ads_last_hal_error;

/* --- hasil pembacaan mentah --- */
extern volatile int16_t  g_ads_raw_load_v;     /* kode ADC sampel terakhir    */
extern volatile int16_t  g_ads_raw_load_i;
extern volatile int16_t  g_ads_avg_load_v;     /* rata-rata ADS_FILTER_LEN    */
extern volatile int16_t  g_ads_avg_load_i;

/* --- tegangan di pin ADC (milivolt), sebelum penskalaan --- */
extern volatile float    g_ads_mv_load_v;
extern volatile float    g_ads_mv_load_i;

/* --- hasil akhir setelah kalibrasi --- */
extern volatile float    g_load_voltage;       /* Volt  */
extern volatile float    g_load_current;       /* Amper */

/* --- kalibrasi: UBAH dari Live Expressions, hasil langsung terlihat ---
   nilai_akhir = (mV_di_pin * scale) + offset
   Contoh: pembagi 1:100 -> scale = 0.1 (1 mV di pin = 0.1 V di beban) */
extern volatile float    g_ads_load_v_scale;
extern volatile float    g_ads_load_v_offset;
extern volatile float    g_ads_load_i_scale;
extern volatile float    g_ads_load_i_offset;

/* API ---------------------------------------------------------------------- */

/** @brief Inisialisasi driver. Panggil setelah MX_I2C1_Init(). */
void ADS_Init(I2C_HandleTypeDef *hi2c);

/** @brief Cek apakah chip menjawab di bus. */
bool ADS_IsReady(void);

/**
  * @brief  Baca satu kanal secara blocking (single-shot). Berguna untuk uji
  *         cepat; jalur normal sebaiknya memakai ADS_Task().
  * @param  raw [out] kode ADC signed 16-bit
  */
ADS_Status_t ADS_ReadBlocking(ADS_Channel_t ch, int16_t *raw);

/**
  * @brief  State machine non-blocking: bergantian LOAD V dan LOAD I,
  *         memperbarui seluruh variabel global di atas.
  *         Panggil sesering mungkin dari while(1) - tidak pernah menunda.
  */
void ADS_Task(void);

/** @brief Konversi kode ADC -> milivolt di pin (FSR 2.048 V). */
float ADS_RawToMillivolt(int16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* __ADS1115_H */
