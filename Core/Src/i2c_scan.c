/**
  ******************************************************************************
  * @file    i2c_scan.c
  * @brief   Pemindai bus I2C - Recticon Rectifier Controller
  ******************************************************************************
  */

#include "i2c_scan.h"

/* Variabel Live Expressions ------------------------------------------------ */
volatile uint8_t g_i2c_found_count  = 0U;
volatile uint8_t g_i2c_found[I2C_SCAN_MAX_FOUND];

volatile uint8_t g_i2c_seen_eeprom   = 0U;
volatile uint8_t g_i2c_seen_ads1115  = 0U;
volatile uint8_t g_i2c_seen_mcp4725  = 0U;
volatile uint8_t g_i2c_mcp4725_addr  = 0U;

/* State privat ------------------------------------------------------------- */
static I2C_HandleTypeDef *s_hi2c = NULL;

/* API ---------------------------------------------------------------------- */

void I2C_ScanInit(I2C_HandleTypeDef *hi2c)
{
  uint8_t i;

  s_hi2c = hi2c;

  for (i = 0U; i < I2C_SCAN_MAX_FOUND; i++)
  {
    g_i2c_found[i] = 0U;
  }
}

void I2C_ScanRun(void)
{
  uint8_t addr;
  uint8_t n = 0U;
  uint8_t i;

  if (s_hi2c == NULL)
  {
    return;
  }

  /* Reset hasil sebelumnya */
  for (i = 0U; i < I2C_SCAN_MAX_FOUND; i++)
  {
    g_i2c_found[i] = 0U;
  }

  g_i2c_seen_eeprom  = 0U;
  g_i2c_seen_ads1115 = 0U;
  g_i2c_seen_mcp4725 = 0U;
  g_i2c_mcp4725_addr = 0U;

  for (addr = I2C_SCAN_FIRST_ADDR; addr <= I2C_SCAN_LAST_ADDR; addr++)
  {
    /* 2 percobaan sudah cukup; timeout pendek supaya scan penuh tetap cepat */
    if (HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(addr << 1), 2U, 5U) == HAL_OK)
    {
      if (n < I2C_SCAN_MAX_FOUND)
      {
        g_i2c_found[n] = addr;
        n++;
      }

      if ((addr >= 0x50U) && (addr <= 0x53U))
      {
        g_i2c_seen_eeprom = 1U;
      }

      if (addr == 0x4AU)
      {
        g_i2c_seen_ads1115 = 1U;
      }

      /* MCP4725: alamat dasar ditentukan sufiks part (A0..A3), bukan cuma
         pin A0. A0->0x60/61, A1->0x62/63, A2->0x64/65, A3->0x66/67 */
      if ((addr >= 0x60U) && (addr <= 0x67U))
      {
        g_i2c_seen_mcp4725 = 1U;
        g_i2c_mcp4725_addr = addr;
      }
    }
  }

  g_i2c_found_count = n;
}
