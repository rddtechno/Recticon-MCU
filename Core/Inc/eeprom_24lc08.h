/**
  ******************************************************************************
  * @file    eeprom_24lc08.h
  * @brief   Driver EEPROM 24LC08 (1 kByte, I2C) - Recticon Rectifier Controller
  *
  * Hardware:
  *   - Bus     : I2C1 (PB6 = SCL, PB7 = SDA), 100 kHz standard mode
  *   - WP pin  : PB8 (DO_EEPROM_WP), HIGH = write protected
  *   - Kapasitas: 1024 byte = 4 blok x 256 byte
  *
  * Catatan penting:
  *   24LC08 menempati EMPAT alamat I2C (0x50..0x53). Dua bit blok (A9:A8)
  *   masuk ke device address byte, BUKAN ke word address. Driver ini
  *   menangani pemetaan tersebut secara otomatis - pemanggil cukup memakai
  *   alamat linier 0x000..0x3FF.
  ******************************************************************************
  */

#ifndef __EEPROM_24LC08_H
#define __EEPROM_24LC08_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Parameter chip ----------------------------------------------------------- */
#define EE_I2C_BASE_ADDR_7B   0x50U   /* 1010 B1 B0 -> 0x50..0x53             */
#define EE_TOTAL_SIZE         1024U   /* 8 kbit = 1 kByte                     */
#define EE_BLOCK_SIZE         256U    /* batas blok = batas device address    */
#define EE_PAGE_SIZE          16U     /* batas page write (wrap-around!)      */
#define EE_WRITE_CYCLE_MS     5U      /* siklus tulis internal maksimum       */
#define EE_I2C_TIMEOUT_MS     100U
#define EE_ACK_POLL_TRIALS    100U

/* Peta memori -------------------------------------------------------------- */
#define EE_ADDR_CONFIG        0x000U  /* blok 0 - config utama   (menyusul)   */
#define EE_ADDR_CONFIG_BAK    0x100U  /* blok 1 - config backup  (menyusul)   */
#define EE_ADDR_TEST          0x200U  /* blok 2 - area uji manual             */
#define EE_ADDR_SELFTEST      0x240U  /* blok 2 - area self-test (terpisah)   */
#define EE_ADDR_SPARE         0x300U  /* blok 3 - cadangan                    */

/* Alamat probe diagnosa WP (satu di tiap separuh array) */
#define EE_ADDR_WP_PROBE_LO   0x010U  /* separuh bawah */
#define EE_ADDR_WP_PROBE_HI   0x210U  /* separuh atas  */

#define EE_TEST_LEN           16U     /* panjang buffer uji Live Expressions  */

/* Kode status -------------------------------------------------------------- */
typedef enum
{
  EE_OK           =  0,
  EE_ERR_PARAM    = -1,   /* alamat/panjang/pointer tidak valid               */
  EE_ERR_NOT_INIT = -2,   /* EE_Init() belum dipanggil                        */
  EE_ERR_I2C      = -3,   /* transfer I2C gagal (NACK / bus error)            */
  EE_ERR_TIMEOUT  = -4,   /* ACK polling tidak selesai (siklus tulis macet)   */
  EE_ERR_VERIFY   = -5    /* data terbaca tidak sama dengan yang ditulis      */
} EE_Status_t;

/* Perintah untuk g_ee_cmd -------------------------------------------------- */
typedef enum
{
  EE_CMD_IDLE     = 0,
  EE_CMD_WRITE    = 1,    /* tulis g_ee_test_wr -> g_ee_test_addr             */
  EE_CMD_READ     = 2,    /* baca g_ee_test_addr -> g_ee_test_rd              */
  EE_CMD_SELFTEST = 3,    /* tulis pola -> baca balik -> verifikasi           */
  EE_CMD_ERASE    = 4,    /* isi 0xFF di g_ee_test_addr                       */
  EE_CMD_WP_PROBE = 5     /* diagnosa cakupan proteksi pin WP                 */
} EE_Cmd_t;

/* ==========================================================================
 * Variabel global untuk LIVE EXPRESSIONS (STM32CubeIDE)
 *
 * Semua volatile + global agar tidak dioptimasi hilang dan terlihat oleh
 * debugger saat target BERJALAN (tidak perlu halt).
 * ========================================================================== */

/* --- DIUBAH dari Live Expressions --- */
extern volatile uint8_t  g_ee_cmd;              /* isi dengan EE_Cmd_t        */
extern volatile uint16_t g_ee_test_addr;        /* alamat target cmd 1/2/4    */
extern volatile uint8_t  g_ee_test_wr[EE_TEST_LEN];  /* data yang akan ditulis */

/* --- DIBACA dari Live Expressions --- */
extern volatile uint8_t  g_ee_test_rd[EE_TEST_LEN];  /* hasil pembacaan      */
extern volatile uint8_t  g_ee_present;          /* 1 = chip menjawab di bus   */
extern volatile int8_t   g_ee_last_result;      /* EE_Status_t operasi akhir  */
extern volatile uint32_t g_ee_cmd_done;         /* naik tiap perintah selesai */
extern volatile uint32_t g_ee_write_count;      /* total page write sukses    */
extern volatile uint32_t g_ee_error_count;      /* total operasi gagal        */
extern volatile uint32_t g_ee_last_hal_error;   /* HAL_I2C_GetError()         */
extern volatile uint8_t  g_ee_wp_lower_locked;  /* hasil cmd 5: separuh bawah */
extern volatile uint8_t  g_ee_wp_upper_locked;  /* hasil cmd 5: separuh atas  */

/* API ---------------------------------------------------------------------- */

/**
  * @brief  Inisialisasi driver. Panggil setelah MX_I2C1_Init().
  * @param  hi2c handle I2C yang dipakai (mis. &hi2c1)
  */
void EE_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief  Cek apakah chip menjawab di bus (ACK polling ke blok 0).
  */
bool EE_IsReady(void);

EE_Status_t EE_ReadByte  (uint16_t addr, uint8_t *data);
EE_Status_t EE_WriteByte (uint16_t addr, uint8_t  data);

/**
  * @brief  Baca sejumlah byte. Transfer dipecah otomatis di batas blok 256 B.
  */
EE_Status_t EE_ReadBuffer (uint16_t addr, uint8_t *buf, uint16_t len);

/**
  * @brief  Tulis sejumlah byte. Transfer dipecah otomatis di batas page 16 B
  *         (wajib - 24LC08 wrap-around di dalam page, bukan lanjut ke page
  *         berikutnya). WP diturunkan selama menulis lalu dinaikkan kembali,
  *         termasuk bila terjadi error.
  */
EE_Status_t EE_WriteBuffer(uint16_t addr, const uint8_t *buf, uint16_t len);

/**
  * @brief  Isi area dengan pola tertentu (mis. 0xFF untuk "erase").
  */
EE_Status_t EE_Erase(uint16_t addr, uint16_t len, uint8_t pattern);

/**
  * @brief  Tulis pola -> baca balik -> verifikasi di EE_ADDR_SELFTEST.
  *         Pola berubah tiap pemanggilan agar data basi ikut terdeteksi.
  */
EE_Status_t EE_SelfTest(void);

/**
  * @brief  Eksekusi g_ee_cmd bila tidak nol, lalu kembalikan g_ee_cmd ke 0.
  *         Panggil dari while(1) di main().
  */
void EE_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_24LC08_H */
