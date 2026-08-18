/**
  ******************************************************************************
  * @file    mcp4725.h
  * @brief   Driver MCP4725 DAC 12-bit - Recticon Rectifier Controller
  *
  * Hardware:
  *   - Bus     : I2C1 (PB6/PB7), 100 kHz
  *   - Alamat  : 0x61 7-bit (terverifikasi di hardware)
  *               Alamat dasar MCP4725 ditentukan sufiks part, bukan cuma pin A0:
  *               A0 -> 0x60/0x61, A1 -> 0x62/0x63, A2 -> 0x64/0x65, A3 -> 0x66/0x67
  *   - Vref    : VDD (3.3 V) -> 1 LSB = 3300 mV / 4096 = 0.8057 mV
  *
  * PERINGATAN keausan:
  *   MCP4725 punya EEPROM internal untuk menyimpan nilai saat power-up, tapi
  *   endurance-nya terbatas (~1 juta siklus). Update rutin WAJIB memakai
  *   MCP_SetRaw() (Fast Mode Write, hanya register volatile). Simpan ke EEPROM
  *   internal (MCP_SaveToEeprom) HANYA saat commissioning - jangan pernah
  *   dipanggil periodik dari loop kontrol.
  ******************************************************************************
  */

#ifndef __MCP4725_H
#define __MCP4725_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Parameter chip ----------------------------------------------------------- */
#define MCP_I2C_ADDR_7B      0x61U   /* terpasang di board (varian A0, pin A0 -> VDD) */
#define MCP_MAX_CODE         4095U   /* 12-bit    */
#define MCP_VREF_MV          3300.0f /* VDD, sesuaikan bila suplai berbeda */
#define MCP_I2C_TIMEOUT_MS   100U

/* Command byte untuk mode Write DAC Register (3-byte) */
#define MCP_CMD_WRITE_DAC    0x40U   /* volatile saja - aman untuk runtime  */
#define MCP_CMD_WRITE_EEPROM 0x60U   /* DAC + EEPROM - endurance terbatas!  */

/* Status ------------------------------------------------------------------- */
typedef enum
{
  MCP_OK           =  0,
  MCP_ERR_PARAM    = -1,
  MCP_ERR_NOT_INIT = -2,
  MCP_ERR_I2C      = -3,
  MCP_ERR_TIMEOUT  = -4,
  MCP_ERR_VERIFY   = -5
} MCP_Status_t;

/* Perintah untuk g_dac_cmd ------------------------------------------------- */
typedef enum
{
  MCP_CMD_IDLE      = 0,
  MCP_CMD_READBACK  = 1,   /* baca isi register DAC -> g_dac_readback       */
  MCP_CMD_SAVE_EE   = 2,   /* simpan setpoint ke EEPROM internal MCP4725    */
  MCP_CMD_RAMP_TEST = 3,   /* sapu 0 -> 4095 -> 0, untuk uji dengan multimeter */
  MCP_CMD_PROBE     = 4    /* uji ulang keberadaan chip di g_dac_i2c_addr   */
} MCP_Cmd_t;

/* ==========================================================================
 * Variabel LIVE EXPRESSIONS
 * ========================================================================== */

/* --- DIUBAH dari Live Expressions --- */
extern volatile uint16_t g_dac_setpoint;      /* 0..4095, output ikut otomatis */
extern volatile uint8_t  g_dac_cmd;           /* isi dengan MCP_Cmd_t          */

/** Alamat 7-bit chip. Default MCP_I2C_ADDR_7B, tapi bisa diubah saat runtime
    dari Live Expressions bila part-nya varian lain (A1/A2/A3) - tidak perlu
    rebuild. Setelah diubah, set g_dac_cmd = MCP_CMD_PROBE untuk uji ulang. */
extern volatile uint8_t  g_dac_i2c_addr;

/* --- DIBACA dari Live Expressions --- */
extern volatile uint8_t  g_dac_present;       /* 1 = chip menjawab di bus      */
extern volatile int8_t   g_dac_last_result;   /* MCP_Status_t terakhir         */
extern volatile uint16_t g_dac_readback;      /* hasil MCP_CMD_READBACK        */
extern volatile float    g_dac_mv;            /* tegangan output teoritis (mV) */
extern volatile uint32_t g_dac_write_count;   /* total penulisan volatile      */
extern volatile uint32_t g_dac_ee_write_count;/* total penulisan EEPROM chip   */
extern volatile uint32_t g_dac_error_count;
extern volatile uint32_t g_dac_last_hal_error;

/* API ---------------------------------------------------------------------- */

/** @brief Inisialisasi driver. Panggil setelah MX_I2C1_Init(). */
void MCP_Init(I2C_HandleTypeDef *hi2c);

/** @brief Cek apakah chip menjawab di bus. */
bool MCP_IsReady(void);

/**
  * @brief  Set output DAC memakai Fast Mode Write (2 byte, hanya register
  *         volatile). Ini jalur normal untuk update setpoint.
  * @param  code 0..4095 (nilai di luar rentang akan ditolak)
  */
MCP_Status_t MCP_SetRaw(uint16_t code);

/** @brief Set output dalam milivolt (dibulatkan ke kode 12-bit terdekat). */
MCP_Status_t MCP_SetMillivolt(float mv);

/** @brief Baca kembali isi register DAC (3 byte: status + 12-bit data). */
MCP_Status_t MCP_ReadBack(uint16_t *code);

/**
  * @brief  Simpan nilai ke EEPROM internal MCP4725 agar dipulihkan saat
  *         power-up. HANYA untuk commissioning - endurance ~1 juta siklus.
  */
MCP_Status_t MCP_SaveToEeprom(uint16_t code);

/**
  * @brief  Pantau perubahan g_dac_setpoint dan jalankan g_dac_cmd.
  *         Panggil dari while(1). Menulis ke DAC hanya bila nilainya berubah,
  *         jadi aman dipanggil sesering mungkin.
  */
void MCP_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __MCP4725_H */
