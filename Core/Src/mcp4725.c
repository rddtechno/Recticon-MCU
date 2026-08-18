/**
  ******************************************************************************
  * @file    mcp4725.c
  * @brief   Driver MCP4725 DAC 12-bit - Recticon Rectifier Controller
  ******************************************************************************
  */

#include "mcp4725.h"

/* Variabel Live Expressions ------------------------------------------------ */
volatile uint16_t g_dac_setpoint       = 0U;
volatile uint8_t  g_dac_cmd            = MCP_CMD_IDLE;
volatile uint8_t  g_dac_i2c_addr       = MCP_I2C_ADDR_7B;

volatile uint8_t  g_dac_present        = 0U;
volatile int8_t   g_dac_last_result    = MCP_OK;
volatile uint16_t g_dac_readback       = 0U;
volatile float    g_dac_mv             = 0.0f;
volatile uint32_t g_dac_write_count    = 0U;
volatile uint32_t g_dac_ee_write_count = 0U;
volatile uint32_t g_dac_error_count    = 0U;
volatile uint32_t g_dac_last_hal_error = 0U;

/* State privat ------------------------------------------------------------- */
static I2C_HandleTypeDef *s_hi2c        = NULL;
static uint16_t           s_last_written = 0xFFFFU;  /* paksa tulis pertama */

/* Helper privat ------------------------------------------------------------ */

static void MCP_NoteError(MCP_Status_t st)
{
  g_dac_last_result = (int8_t)st;
  g_dac_error_count++;

  if (s_hi2c != NULL)
  {
    g_dac_last_hal_error = HAL_I2C_GetError(s_hi2c);
  }
}

/* API ---------------------------------------------------------------------- */

void MCP_Init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c         = hi2c;
  s_last_written = 0xFFFFU;   /* nilai mustahil -> tulis pertama selalu jalan */
  g_dac_setpoint = 0U;
  g_dac_mv       = 0.0f;
}

bool MCP_IsReady(void)
{
  if (s_hi2c == NULL)
  {
    return false;
  }

  return (HAL_I2C_IsDeviceReady(s_hi2c,
                                (uint16_t)(g_dac_i2c_addr << 1),
                                3U,
                                MCP_I2C_TIMEOUT_MS) == HAL_OK);
}

MCP_Status_t MCP_SetRaw(uint16_t code)
{
  uint8_t buf[2];

  if (s_hi2c == NULL)     { return MCP_ERR_NOT_INIT; }
  if (code > MCP_MAX_CODE) { return MCP_ERR_PARAM;   }

  /* Fast Mode Write: 2 byte, tanpa command byte.
       byte0 = 0 0 PD1 PD0 D11 D10 D9 D8   (PD = 00 -> normal mode)
       byte1 = D7..D0                                                      */
  buf[0] = (uint8_t)((code >> 8) & 0x0FU);
  buf[1] = (uint8_t)(code & 0xFFU);

  if (HAL_I2C_Master_Transmit(s_hi2c,
                              (uint16_t)(g_dac_i2c_addr << 1),
                              buf,
                              2U,
                              MCP_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return MCP_ERR_I2C;
  }

  g_dac_write_count++;
  g_dac_mv = ((float)code * MCP_VREF_MV) / (float)(MCP_MAX_CODE + 1U);

  return MCP_OK;
}

MCP_Status_t MCP_SetMillivolt(float mv)
{
  float code_f;

  if (mv < 0.0f)
  {
    return MCP_ERR_PARAM;
  }

  code_f = (mv * (float)(MCP_MAX_CODE + 1U)) / MCP_VREF_MV;

  if (code_f > (float)MCP_MAX_CODE)
  {
    return MCP_ERR_PARAM;
  }

  return MCP_SetRaw((uint16_t)(code_f + 0.5f));
}

MCP_Status_t MCP_ReadBack(uint16_t *code)
{
  uint8_t buf[3];

  if (s_hi2c == NULL) { return MCP_ERR_NOT_INIT; }
  if (code == NULL)   { return MCP_ERR_PARAM;    }

  /* Format baca: byte0 = status/config, byte1 = D11..D4, byte2 = D3..D0 <<4 */
  if (HAL_I2C_Master_Receive(s_hi2c,
                             (uint16_t)(g_dac_i2c_addr << 1),
                             buf,
                             3U,
                             MCP_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return MCP_ERR_I2C;
  }

  *code = (uint16_t)(((uint16_t)buf[1] << 4) | ((uint16_t)buf[2] >> 4));

  return MCP_OK;
}

MCP_Status_t MCP_SaveToEeprom(uint16_t code)
{
  uint8_t  buf[3];
  uint32_t start;

  if (s_hi2c == NULL)      { return MCP_ERR_NOT_INIT; }
  if (code > MCP_MAX_CODE) { return MCP_ERR_PARAM;    }

  /* Write DAC + EEPROM: 3 byte
       byte0 = 0 1 1 0 0 PD1 PD0 0
       byte1 = D11..D4
       byte2 = D3..D0 0 0 0 0                                              */
  buf[0] = MCP_CMD_WRITE_EEPROM;
  buf[1] = (uint8_t)(code >> 4);
  buf[2] = (uint8_t)((code & 0x0FU) << 4);

  if (HAL_I2C_Master_Transmit(s_hi2c,
                              (uint16_t)(g_dac_i2c_addr << 1),
                              buf,
                              3U,
                              MCP_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return MCP_ERR_I2C;
  }

  /* Siklus tulis EEPROM internal ~25-50 ms; chip tidak meng-ACK selama itu */
  start = HAL_GetTick();
  while (HAL_I2C_IsDeviceReady(s_hi2c,
                               (uint16_t)(g_dac_i2c_addr << 1),
                               1U,
                               MCP_I2C_TIMEOUT_MS) != HAL_OK)
  {
    if ((HAL_GetTick() - start) > 100U)
    {
      return MCP_ERR_TIMEOUT;
    }
  }

  g_dac_ee_write_count++;
  return MCP_OK;
}

void MCP_Task(void)
{
  MCP_Status_t st;
  uint16_t     setpoint;
  uint8_t      cmd;

  if (s_hi2c == NULL)
  {
    return;
  }

  /* --- 1. Ikuti perubahan setpoint (ditulis hanya bila berubah) --- */
  setpoint = g_dac_setpoint;

  if (setpoint != s_last_written)
  {
    if (setpoint > MCP_MAX_CODE)
    {
      /* Nilai di luar rentang dari Live Expressions: jepit dan pantulkan
         balik supaya pemakai langsung melihat koreksinya. */
      setpoint       = MCP_MAX_CODE;
      g_dac_setpoint = MCP_MAX_CODE;
    }

    st = MCP_SetRaw(setpoint);

    if (st == MCP_OK)
    {
      s_last_written    = setpoint;
      g_dac_last_result = (int8_t)MCP_OK;
    }
    else
    {
      MCP_NoteError(st);
    }
  }

  /* --- 2. Perintah manual --- */
  cmd = g_dac_cmd;

  if (cmd == (uint8_t)MCP_CMD_IDLE)
  {
    return;
  }

  switch ((MCP_Cmd_t)cmd)
  {
    case MCP_CMD_READBACK:
    {
      uint16_t rb = 0U;

      st = MCP_ReadBack(&rb);
      if (st == MCP_OK)
      {
        g_dac_readback    = rb;
        g_dac_last_result = (int8_t)MCP_OK;
      }
      else
      {
        MCP_NoteError(st);
      }
      break;
    }

    case MCP_CMD_SAVE_EE:
      st = MCP_SaveToEeprom(g_dac_setpoint);
      if (st == MCP_OK)
      {
        g_dac_last_result = (int8_t)MCP_OK;
      }
      else
      {
        MCP_NoteError(st);
      }
      break;

    case MCP_CMD_RAMP_TEST:
    {
      uint16_t v;

      /* Sapu naik lalu turun - amati dengan multimeter di pin OUT.
         Blocking ~1.6 detik, hanya untuk uji manual. */
      for (v = 0U; v <= MCP_MAX_CODE; v = (uint16_t)(v + 64U))
      {
        (void)MCP_SetRaw(v);
        HAL_Delay(10U);
      }
      for (v = MCP_MAX_CODE; v > 64U; v = (uint16_t)(v - 64U))
      {
        (void)MCP_SetRaw(v);
        HAL_Delay(10U);
      }

      (void)MCP_SetRaw(g_dac_setpoint);
      s_last_written    = g_dac_setpoint;
      g_dac_last_result = (int8_t)MCP_OK;
      break;
    }

    case MCP_CMD_PROBE:
      /* Uji ulang setelah g_dac_i2c_addr diubah dari Live Expressions */
      g_dac_present = (uint8_t)(MCP_IsReady() ? 1U : 0U);

      if (g_dac_present != 0U)
      {
        /* Paksa penulisan ulang setpoint ke alamat yang baru */
        s_last_written    = 0xFFFFU;
        g_dac_last_result = (int8_t)MCP_OK;
      }
      else
      {
        MCP_NoteError(MCP_ERR_I2C);
      }
      break;

    default:
      g_dac_last_result = (int8_t)MCP_ERR_PARAM;
      break;
  }

  g_dac_cmd = (uint8_t)MCP_CMD_IDLE;
}
