/**
  ******************************************************************************
  * @file    i2c_scan.h
  * @brief   Pemindai bus I2C - alat diagnosa via Live Expressions
  *          Recticon Rectifier Controller
  *
  * Memindai alamat 7-bit 0x08..0x77 dan mencatat siapa saja yang menjawab.
  * Hasilnya dibaca langsung dari Live Expressions - tidak perlu UART/printf.
  ******************************************************************************
  */

#ifndef __I2C_SCAN_H
#define __I2C_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define I2C_SCAN_FIRST_ADDR   0x08U   /* 0x00-0x07 dicadangkan spesifikasi */
#define I2C_SCAN_LAST_ADDR    0x77U   /* 0x78-0x7F dicadangkan             */
#define I2C_SCAN_MAX_FOUND    16U

/* ==========================================================================
 * Variabel LIVE EXPRESSIONS
 * ========================================================================== */

/** Jumlah device yang menjawab */
extern volatile uint8_t g_i2c_found_count;

/** Daftar alamat 7-bit yang menjawab (expand array ini di Live Expressions) */
extern volatile uint8_t g_i2c_found[I2C_SCAN_MAX_FOUND];

/** Hasil pencocokan dengan device yang diharapkan (1 = ditemukan) */
extern volatile uint8_t g_i2c_seen_eeprom;   /* 0x50..0x53 */
extern volatile uint8_t g_i2c_seen_ads1115;  /* 0x4A       */
extern volatile uint8_t g_i2c_seen_mcp4725;  /* 0x60..0x67 */

/** Alamat MCP4725 yang benar-benar ditemukan (0 = tidak ada) */
extern volatile uint8_t g_i2c_mcp4725_addr;

/* API ---------------------------------------------------------------------- */

void I2C_ScanInit(I2C_HandleTypeDef *hi2c);

/** @brief Jalankan pemindaian sekarang (blocking, ~100 ms). Hanya dipanggil
  *        saat boot sebagai diagnosa - tidak ada task yang berjalan di loop. */
void I2C_ScanRun(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SCAN_H */
