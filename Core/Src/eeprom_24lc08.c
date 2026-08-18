/**
  ******************************************************************************
  * @file    eeprom_24lc08.c
  * @brief   Driver EEPROM 24LC08 (1 kByte, I2C) - Recticon Rectifier Controller
  ******************************************************************************
  */

#include "eeprom_24lc08.h"
#include <string.h>

/* Variabel Live Expressions ------------------------------------------------ */
volatile uint8_t  g_ee_cmd             = EE_CMD_IDLE;
volatile uint16_t g_ee_test_addr       = EE_ADDR_TEST;
volatile uint8_t  g_ee_test_wr[EE_TEST_LEN];
volatile uint8_t  g_ee_test_rd[EE_TEST_LEN];

volatile uint8_t  g_ee_present         = 0U;
volatile int8_t   g_ee_last_result     = EE_OK;
volatile uint32_t g_ee_cmd_done        = 0U;
volatile uint32_t g_ee_write_count     = 0U;
volatile uint32_t g_ee_error_count     = 0U;
volatile uint32_t g_ee_last_hal_error  = 0U;
volatile uint8_t  g_ee_wp_lower_locked = 0U;
volatile uint8_t  g_ee_wp_upper_locked = 0U;

/* State privat ------------------------------------------------------------- */
static I2C_HandleTypeDef *s_hi2c          = NULL;
static uint8_t            s_selftest_seed = 0U;

/* Helper privat ------------------------------------------------------------ */

/**
  * @brief  Hitung device address 8-bit dari alamat linier.
  *         Bit A9:A8 alamat masuk ke bit blok device address (B1:B0).
  */
static uint16_t EE_DevAddr8(uint16_t addr)
{
  return (uint16_t)((EE_I2C_BASE_ADDR_7B | ((addr >> 8) & 0x03U)) << 1);
}

/** @brief WP HIGH - proteksi aktif (kondisi normal/idle). */
static void EE_WpEnable(void)
{
  HAL_GPIO_WritePin(DO_EEPROM_WP_GPIO_Port, DO_EEPROM_WP_Pin, GPIO_PIN_SET);
}

/** @brief WP LOW - proteksi dilepas, hanya selama menulis. */
static void EE_WpDisable(void)
{
  HAL_GPIO_WritePin(DO_EEPROM_WP_GPIO_Port, DO_EEPROM_WP_Pin, GPIO_PIN_RESET);
}

/* API ---------------------------------------------------------------------- */

void EE_Init(I2C_HandleTypeDef *hi2c)
{
  uint16_t i;

  s_hi2c = hi2c;

  /* Pastikan proteksi aktif saat idle */
  EE_WpEnable();

  g_ee_test_addr = EE_ADDR_TEST;

  /* Isi pola awal agar buffer langsung terlihat "hidup" di Live Expressions */
  for (i = 0U; i < EE_TEST_LEN; i++)
  {
    g_ee_test_wr[i] = (uint8_t)(0x10U + i);
    g_ee_test_rd[i] = 0x00U;
  }
}

bool EE_IsReady(void)
{
  if (s_hi2c == NULL)
  {
    return false;
  }

  return (HAL_I2C_IsDeviceReady(s_hi2c,
                                (uint16_t)(EE_I2C_BASE_ADDR_7B << 1),
                                EE_ACK_POLL_TRIALS,
                                EE_I2C_TIMEOUT_MS) == HAL_OK);
}

EE_Status_t EE_ReadBuffer(uint16_t addr, uint8_t *buf, uint16_t len)
{
  uint16_t done = 0U;

  if (s_hi2c == NULL)                       { return EE_ERR_NOT_INIT; }
  if ((buf == NULL) || (len == 0U))         { return EE_ERR_PARAM;    }
  if (((uint32_t)addr + len) > EE_TOTAL_SIZE) { return EE_ERR_PARAM;  }

  /* Pecah di batas blok 256 B: bit blok ada di device address, sequential
     read tidak menyeberang blok dengan andal. */
  while (done < len)
  {
    uint16_t cur   = (uint16_t)(addr + done);
    uint16_t space = (uint16_t)(EE_BLOCK_SIZE - (cur % EE_BLOCK_SIZE));
    uint16_t chunk = (uint16_t)(len - done);

    if (chunk > space)
    {
      chunk = space;
    }

    if (HAL_I2C_Mem_Read(s_hi2c,
                         EE_DevAddr8(cur),
                         (uint16_t)(cur & 0xFFU),
                         I2C_MEMADD_SIZE_8BIT,
                         &buf[done],
                         chunk,
                         EE_I2C_TIMEOUT_MS) != HAL_OK)
    {
      return EE_ERR_I2C;
    }

    done = (uint16_t)(done + chunk);
  }

  return EE_OK;
}

EE_Status_t EE_WriteBuffer(uint16_t addr, const uint8_t *buf, uint16_t len)
{
  EE_Status_t st      = EE_OK;
  uint16_t    written = 0U;

  if (s_hi2c == NULL)                       { return EE_ERR_NOT_INIT; }
  if ((buf == NULL) || (len == 0U))         { return EE_ERR_PARAM;    }
  if (((uint32_t)addr + len) > EE_TOTAL_SIZE) { return EE_ERR_PARAM;  }

  EE_WpDisable();

  /* Pecah di batas page 16 B. Menulis melewati batas page akan wrap-around
     ke awal page yang sama dan menimpa data - bukan lanjut ke page berikut. */
  while (written < len)
  {
    uint16_t cur   = (uint16_t)(addr + written);
    uint16_t space = (uint16_t)(EE_PAGE_SIZE - (cur % EE_PAGE_SIZE));
    uint16_t chunk = (uint16_t)(len - written);

    if (chunk > space)
    {
      chunk = space;
    }

    if (HAL_I2C_Mem_Write(s_hi2c,
                          EE_DevAddr8(cur),
                          (uint16_t)(cur & 0xFFU),
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)&buf[written],
                          chunk,
                          EE_I2C_TIMEOUT_MS) != HAL_OK)
    {
      st = EE_ERR_I2C;
      break;
    }

    /* Siklus tulis internal 5 ms: chip tidak meng-ACK selama proses.
       ACK polling lebih cepat dan lebih andal daripada HAL_Delay() buta. */
    if (HAL_I2C_IsDeviceReady(s_hi2c,
                              EE_DevAddr8(cur),
                              EE_ACK_POLL_TRIALS,
                              EE_I2C_TIMEOUT_MS) != HAL_OK)
    {
      st = EE_ERR_TIMEOUT;
      break;
    }

    g_ee_write_count++;
    written = (uint16_t)(written + chunk);
  }

  /* WP dikembalikan HIGH juga di jalur error - jangan pernah tertinggal LOW */
  EE_WpEnable();

  return st;
}

EE_Status_t EE_ReadByte(uint16_t addr, uint8_t *data)
{
  return EE_ReadBuffer(addr, data, 1U);
}

EE_Status_t EE_WriteByte(uint16_t addr, uint8_t data)
{
  return EE_WriteBuffer(addr, &data, 1U);
}

EE_Status_t EE_Erase(uint16_t addr, uint16_t len, uint8_t pattern)
{
  uint8_t     page[EE_PAGE_SIZE];
  uint16_t    done = 0U;
  EE_Status_t st   = EE_OK;

  if (len == 0U)                            { return EE_ERR_PARAM; }
  if (((uint32_t)addr + len) > EE_TOTAL_SIZE) { return EE_ERR_PARAM; }

  memset(page, pattern, sizeof(page));

  while ((done < len) && (st == EE_OK))
  {
    uint16_t chunk = (uint16_t)(len - done);

    if (chunk > EE_PAGE_SIZE)
    {
      chunk = EE_PAGE_SIZE;
    }

    st   = EE_WriteBuffer((uint16_t)(addr + done), page, chunk);
    done = (uint16_t)(done + chunk);
  }

  return st;
}

EE_Status_t EE_SelfTest(void)
{
  uint8_t     wr[EE_PAGE_SIZE];
  uint8_t     rd[EE_PAGE_SIZE];
  EE_Status_t st;
  uint16_t    i;

  /* Pola berubah tiap pemanggilan supaya data basi ikut terdeteksi -
     kalau tulis diam-diam gagal, pembacaan akan mengembalikan pola lama. */
  s_selftest_seed++;
  for (i = 0U; i < EE_PAGE_SIZE; i++)
  {
    wr[i] = (uint8_t)(0xA5U ^ (uint8_t)(s_selftest_seed + (uint8_t)(i * 7U)));
  }

  st = EE_WriteBuffer(EE_ADDR_SELFTEST, wr, EE_PAGE_SIZE);
  if (st != EE_OK)
  {
    return st;
  }

  memset(rd, 0, sizeof(rd));
  st = EE_ReadBuffer(EE_ADDR_SELFTEST, rd, EE_PAGE_SIZE);
  if (st != EE_OK)
  {
    return st;
  }

  if (memcmp(wr, rd, EE_PAGE_SIZE) != 0)
  {
    return EE_ERR_VERIFY;
  }

  return EE_OK;
}

/**
  * @brief  Coba tulis satu byte dengan WP tetap HIGH, lalu lihat apakah
  *         nilainya berubah. Dipakai untuk memastikan cakupan proteksi WP
  *         pada part yang terpasang (sebagian varian 24LC08 hanya
  *         memproteksi separuh atas array).
  * @param  locked [out] 1 = tulis terblokir, 0 = tulis lolos
  */
static EE_Status_t EE_WpProbe(uint16_t addr, uint8_t *locked)
{
  uint8_t     orig;
  uint8_t     probe;
  uint8_t     readback;
  EE_Status_t st;

  st = EE_ReadByte(addr, &orig);
  if (st != EE_OK)
  {
    return st;
  }

  probe = (uint8_t)(orig ^ 0xFFU);

  /* Sengaja TIDAK memakai EE_WriteByte() - WP harus tetap HIGH di sini */
  EE_WpEnable();
  (void)HAL_I2C_Mem_Write(s_hi2c,
                          EE_DevAddr8(addr),
                          (uint16_t)(addr & 0xFFU),
                          I2C_MEMADD_SIZE_8BIT,
                          &probe,
                          1U,
                          EE_I2C_TIMEOUT_MS);
  (void)HAL_I2C_IsDeviceReady(s_hi2c,
                              EE_DevAddr8(addr),
                              EE_ACK_POLL_TRIALS,
                              EE_I2C_TIMEOUT_MS);

  st = EE_ReadByte(addr, &readback);
  if (st != EE_OK)
  {
    return st;
  }

  *locked = (uint8_t)((readback == orig) ? 1U : 0U);

  /* Pulihkan nilai semula bila tulis ternyata lolos */
  if (readback != orig)
  {
    st = EE_WriteByte(addr, orig);
  }

  return st;
}

void EE_Task(void)
{
  EE_Status_t st  = EE_OK;
  uint8_t     cmd = g_ee_cmd;

  if (cmd == (uint8_t)EE_CMD_IDLE)
  {
    return;
  }

  switch ((EE_Cmd_t)cmd)
  {
    case EE_CMD_WRITE:
      st = EE_WriteBuffer(g_ee_test_addr, (const uint8_t *)g_ee_test_wr, EE_TEST_LEN);
      break;

    case EE_CMD_READ:
      st = EE_ReadBuffer(g_ee_test_addr, (uint8_t *)g_ee_test_rd, EE_TEST_LEN);
      break;

    case EE_CMD_SELFTEST:
      st = EE_SelfTest();
      break;

    case EE_CMD_ERASE:
      st = EE_Erase(g_ee_test_addr, EE_TEST_LEN, 0xFFU);
      break;

    case EE_CMD_WP_PROBE:
    {
      uint8_t lo = 0U;
      uint8_t hi = 0U;

      st = EE_WpProbe(EE_ADDR_WP_PROBE_LO, &lo);
      if (st == EE_OK)
      {
        st = EE_WpProbe(EE_ADDR_WP_PROBE_HI, &hi);
      }

      g_ee_wp_lower_locked = lo;
      g_ee_wp_upper_locked = hi;
      break;
    }

    default:
      st = EE_ERR_PARAM;
      break;
  }

  g_ee_last_result = (int8_t)st;

  if (st != EE_OK)
  {
    g_ee_error_count++;
    if (s_hi2c != NULL)
    {
      g_ee_last_hal_error = HAL_I2C_GetError(s_hi2c);
    }
  }

  g_ee_cmd_done++;
  g_ee_cmd = (uint8_t)EE_CMD_IDLE;
}
