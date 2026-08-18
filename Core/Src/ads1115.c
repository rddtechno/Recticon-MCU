/**
  ******************************************************************************
  * @file    ads1115.c
  * @brief   Driver ADS1115 - Recticon Rectifier Controller
  ******************************************************************************
  */

#include "ads1115.h"

/* Variabel Live Expressions ------------------------------------------------ */
volatile uint8_t  g_ads_present        = 0U;
volatile int8_t   g_ads_last_result    = ADS_OK;
volatile uint32_t g_ads_sample_count   = 0U;
volatile uint32_t g_ads_error_count    = 0U;
volatile uint32_t g_ads_last_hal_error = 0U;

volatile int16_t  g_ads_raw_load_v     = 0;
volatile int16_t  g_ads_raw_load_i     = 0;
volatile int16_t  g_ads_avg_load_v     = 0;
volatile int16_t  g_ads_avg_load_i     = 0;

volatile float    g_ads_mv_load_v      = 0.0f;
volatile float    g_ads_mv_load_i      = 0.0f;

volatile float    g_load_voltage       = 0.0f;
volatile float    g_load_current       = 0.0f;

/* Default 1.0 / 0.0 = tampilkan apa adanya (milivolt di pin).
   Isi dengan faktor pembagi & shunt yang sebenarnya saat kalibrasi. */
volatile float    g_ads_load_v_scale   = 1.0f;
volatile float    g_ads_load_v_offset  = 0.0f;
volatile float    g_ads_load_i_scale   = 1.0f;
volatile float    g_ads_load_i_offset  = 0.0f;

/* State privat ------------------------------------------------------------- */
static I2C_HandleTypeDef *s_hi2c = NULL;

typedef enum
{
  ADS_ST_START = 0,   /* kirim perintah mulai konversi */
  ADS_ST_WAIT         /* tunggu konversi selesai       */
} ADS_State_t;

static ADS_State_t   s_state   = ADS_ST_START;
static ADS_Channel_t s_channel = ADS_CH_LOAD_V;
static uint32_t      s_tick    = 0U;

/* Buffer moving average per kanal */
static int16_t  s_filt_buf[ADS_CH_COUNT][ADS_FILTER_LEN];
static uint8_t  s_filt_idx[ADS_CH_COUNT];
static uint8_t  s_filt_fill[ADS_CH_COUNT];
static int32_t  s_filt_sum[ADS_CH_COUNT];

/* Helper privat ------------------------------------------------------------ */

/** @brief Susun isi Config register untuk kanal tertentu. */
static uint16_t ADS_BuildConfig(ADS_Channel_t ch, bool start)
{
  uint16_t mux = (ch == ADS_CH_LOAD_V) ? ADS_MUX_LOAD_V : ADS_MUX_LOAD_I;
  uint16_t cfg = 0U;

  if (start)
  {
    cfg |= ADS_OS_SINGLE;
  }

  cfg |= (uint16_t)(mux             << ADS_MUX_Pos);
  cfg |= (uint16_t)(ADS_PGA_2048MV  << ADS_PGA_Pos);
  cfg |= ADS_MODE_SINGLE;
  cfg |= (uint16_t)(ADS_DR_860SPS   << ADS_DR_Pos);
  cfg |= ADS_COMP_QUE_DISABLE;

  return cfg;
}

static ADS_Status_t ADS_WriteReg16(uint8_t reg, uint16_t val)
{
  uint8_t buf[2];

  buf[0] = (uint8_t)(val >> 8);
  buf[1] = (uint8_t)(val & 0xFFU);

  if (HAL_I2C_Mem_Write(s_hi2c,
                        (uint16_t)(ADS_I2C_ADDR_7B << 1),
                        reg,
                        I2C_MEMADD_SIZE_8BIT,
                        buf,
                        2U,
                        ADS_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return ADS_ERR_I2C;
  }

  return ADS_OK;
}

static ADS_Status_t ADS_ReadReg16(uint8_t reg, uint16_t *val)
{
  uint8_t buf[2];

  if (HAL_I2C_Mem_Read(s_hi2c,
                       (uint16_t)(ADS_I2C_ADDR_7B << 1),
                       reg,
                       I2C_MEMADD_SIZE_8BIT,
                       buf,
                       2U,
                       ADS_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return ADS_ERR_I2C;
  }

  *val = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
  return ADS_OK;
}

/** @brief Masukkan sampel ke moving average, kembalikan rata-rata terkini. */
static int16_t ADS_Filter(ADS_Channel_t ch, int16_t sample)
{
  uint8_t idx = s_filt_idx[ch];

  s_filt_sum[ch] -= s_filt_buf[ch][idx];
  s_filt_buf[ch][idx] = sample;
  s_filt_sum[ch] += sample;

  s_filt_idx[ch] = (uint8_t)((idx + 1U) % ADS_FILTER_LEN);

  if (s_filt_fill[ch] < ADS_FILTER_LEN)
  {
    s_filt_fill[ch]++;
  }

  /* Selama buffer belum penuh, bagi dengan jumlah sampel yang sudah masuk
     supaya nilai awal tidak tampak seperempatnya. */
  return (int16_t)(s_filt_sum[ch] / (int32_t)s_filt_fill[ch]);
}

/* API ---------------------------------------------------------------------- */

void ADS_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t ch;
  uint8_t i;

  s_hi2c    = hi2c;
  s_state   = ADS_ST_START;
  s_channel = ADS_CH_LOAD_V;
  s_tick    = HAL_GetTick();

  for (ch = 0U; ch < (uint8_t)ADS_CH_COUNT; ch++)
  {
    s_filt_idx[ch]  = 0U;
    s_filt_fill[ch] = 0U;
    s_filt_sum[ch]  = 0;

    for (i = 0U; i < ADS_FILTER_LEN; i++)
    {
      s_filt_buf[ch][i] = 0;
    }
  }
}

bool ADS_IsReady(void)
{
  if (s_hi2c == NULL)
  {
    return false;
  }

  return (HAL_I2C_IsDeviceReady(s_hi2c,
                                (uint16_t)(ADS_I2C_ADDR_7B << 1),
                                3U,
                                ADS_I2C_TIMEOUT_MS) == HAL_OK);
}

float ADS_RawToMillivolt(int16_t raw)
{
  return ((float)raw * ADS_LSB_UV) / 1000.0f;
}

ADS_Status_t ADS_ReadBlocking(ADS_Channel_t ch, int16_t *raw)
{
  ADS_Status_t st;
  uint16_t     cfg;
  uint32_t     start;

  if (s_hi2c == NULL) { return ADS_ERR_NOT_INIT; }
  if (raw == NULL)    { return ADS_ERR_PARAM;    }
  if (ch >= ADS_CH_COUNT) { return ADS_ERR_PARAM; }

  st = ADS_WriteReg16(ADS_REG_CONFIG, ADS_BuildConfig(ch, true));
  if (st != ADS_OK)
  {
    return st;
  }

  /* Polling bit OS: 1 = konversi selesai */
  start = HAL_GetTick();
  do
  {
    if ((HAL_GetTick() - start) > 10U)
    {
      return ADS_ERR_TIMEOUT;
    }

    st = ADS_ReadReg16(ADS_REG_CONFIG, &cfg);
    if (st != ADS_OK)
    {
      return st;
    }
  } while ((cfg & ADS_OS_NOT_BUSY) == 0U);

  st = ADS_ReadReg16(ADS_REG_CONVERSION, &cfg);
  if (st != ADS_OK)
  {
    return st;
  }

  *raw = (int16_t)cfg;
  return ADS_OK;
}

void ADS_Task(void)
{
  ADS_Status_t st;

  if (s_hi2c == NULL)
  {
    return;
  }

  switch (s_state)
  {
    case ADS_ST_START:
      st = ADS_WriteReg16(ADS_REG_CONFIG, ADS_BuildConfig(s_channel, true));

      if (st != ADS_OK)
      {
        g_ads_last_result   = (int8_t)st;
        g_ads_error_count++;
        g_ads_last_hal_error = HAL_I2C_GetError(s_hi2c);
        /* jangan macet di kanal yang bermasalah - lanjut ke kanal berikutnya */
        s_channel = (ADS_Channel_t)((s_channel + 1) % ADS_CH_COUNT);
        break;
      }

      s_tick  = HAL_GetTick();
      s_state = ADS_ST_WAIT;
      break;

    case ADS_ST_WAIT:
    {
      uint16_t val;
      int16_t  raw;
      int16_t  avg;
      float    mv;

      if ((HAL_GetTick() - s_tick) < ADS_CONV_WAIT_TICKS)
      {
        break;   /* konversi belum tentu selesai - keluar tanpa menunda */
      }

      st = ADS_ReadReg16(ADS_REG_CONVERSION, &val);
      if (st != ADS_OK)
      {
        g_ads_last_result    = (int8_t)st;
        g_ads_error_count++;
        g_ads_last_hal_error = HAL_I2C_GetError(s_hi2c);
      }
      else
      {
        raw = (int16_t)val;
        avg = ADS_Filter(s_channel, raw);
        mv  = ADS_RawToMillivolt(avg);

        if (s_channel == ADS_CH_LOAD_V)
        {
          g_ads_raw_load_v = raw;
          g_ads_avg_load_v = avg;
          g_ads_mv_load_v  = mv;
          g_load_voltage   = (mv * g_ads_load_v_scale) + g_ads_load_v_offset;
        }
        else
        {
          g_ads_raw_load_i = raw;
          g_ads_avg_load_i = avg;
          g_ads_mv_load_i  = mv;
          g_load_current   = (mv * g_ads_load_i_scale) + g_ads_load_i_offset;
        }

        g_ads_sample_count++;
        g_ads_last_result = (int8_t)ADS_OK;
      }

      s_channel = (ADS_Channel_t)((s_channel + 1) % ADS_CH_COUNT);
      s_state   = ADS_ST_START;
      break;
    }

    default:
      s_state = ADS_ST_START;
      break;
  }
}
