/**
  ******************************************************************************
  * @file    modbus_slave.c
  * @brief   Modbus RTU Slave - Recticon Rectifier Controller
  ******************************************************************************
  */

#include "modbus_slave.h"
#include "mcp23s17.h"
#include "ads131m04.h"
#include "ads1115.h"
#include "mcp4725.h"
#include "eeprom_24lc08.h"
#include <string.h>

/* Instance ----------------------------------------------------------------- */
MB_Slave_t g_mb_hmi;   /* USART6, RS485 + DE      */
MB_Slave_t g_mb_opi;   /* USART1, TTL tanpa DE    */

/* Penampung register. Sengaja global, bukan di dalam MB_Slave_t: kedua slave
   (HMI dan Orange Pi nanti) harus melihat data yang sama persis. */
volatile uint16_t g_mb_input[MB_IR_COUNT] = {0};
volatile uint16_t g_mb_hold[MB_HR_COUNT]  = {0};

/* Helper: CRC -------------------------------------------------------------- */

uint16_t MB_Crc16(const uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t  b;

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)buf[i];

    for (b = 0U; b < 8U; b++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/* Helper: akses data ------------------------------------------------------- */

/** Konversi float ke int16 berskala x100, dijepit agar tidak melipat. */
static int16_t MB_ScaleX100(float v)
{
  float f = v * 100.0f;

  if (f > 32767.0f)  { return (int16_t)32767;  }
  if (f < -32768.0f) { return (int16_t)-32768; }

  return (int16_t)(f + ((f >= 0.0f) ? 0.5f : -0.5f));
}

static uint16_t MB_DiWord(void)
{
  uint16_t w = 0U;
  uint8_t  i;

  for (i = 0U; i < DIN_CH_PER_PORT; i++)
  {
    if (g_di_a[i] != 0U) { w |= (uint16_t)(1U << i);                     }
    if (g_di_b[i] != 0U) { w |= (uint16_t)(1U << (i + DIN_CH_PER_PORT)); }
  }

  return w;
}

static uint16_t MB_DoWord(void)
{
  uint16_t w = 0U;
  uint8_t  i;

  for (i = 0U; i < DOUT_CH_COUNT; i++)
  {
    if (g_do[i] != 0U)
    {
      w |= (uint16_t)(1U << i);
    }
  }

  return w;
}

static uint16_t MB_DevStatus(void)
{
  uint16_t s = 0U;

  if (g_ee_present   != 0U) { s |= MB_DEVBIT_EEPROM;    }
  if (g_ads_present  != 0U) { s |= MB_DEVBIT_ADS1115;   }
  if (g_dac_present  != 0U) { s |= MB_DEVBIT_MCP4725;   }
  if (g_dout_present != 0U) { s |= MB_DEVBIT_IOEXP_DO;  }
  if (g_di_present   != 0U) { s |= MB_DEVBIT_IOEXP_DI;  }
  if (g_adc_present  != 0U) { s |= MB_DEVBIT_ADS131M04; }
  if (g_adc_running  != 0U) { s |= MB_DEVBIT_ADC_RUN;   }

  return s;
}

static bool MB_GetDiscrete(uint16_t idx)
{
  if (idx < DIN_CH_PER_PORT)
  {
    return (g_di_a[idx] != 0U);
  }

  if (idx < MB_DI_COUNT)
  {
    return (g_di_b[idx - DIN_CH_PER_PORT] != 0U);
  }

  return false;
}

/**
  * Segarkan alamat "hidup" di g_mb_input[] dari variabel driver.
  * Dipanggil tepat sebelum permintaan baca dilayani - bukan periodik - jadi
  * datanya selalu segar tanpa membebani main loop saat bus sepi.
  * Alamat >= MB_IR_MAPPED_END tidak disentuh: itu milik kode aplikasi.
  */
static void MB_RefreshInputRegs(void)
{
  g_mb_input[MB_IR_RECT_VOLTAGE] = (uint16_t)MB_ScaleX100(g_rect_voltage);
  g_mb_input[MB_IR_RECT_CURRENT] = (uint16_t)MB_ScaleX100(g_rect_current);
  g_mb_input[MB_IR_BATT_VOLTAGE] = (uint16_t)MB_ScaleX100(g_batt_voltage);
  g_mb_input[MB_IR_BATT_CURRENT] = (uint16_t)MB_ScaleX100(g_batt_current);
  g_mb_input[MB_IR_LOAD_VOLTAGE] = (uint16_t)MB_ScaleX100(g_load_voltage);
  g_mb_input[MB_IR_LOAD_CURRENT] = (uint16_t)MB_ScaleX100(g_load_current);
  g_mb_input[MB_IR_DI_WORD]      = MB_DiWord();
  g_mb_input[MB_IR_DO_WORD]      = MB_DoWord();
  g_mb_input[MB_IR_DEV_STATUS]   = MB_DevStatus();
  g_mb_input[MB_IR_ADC_SPS]      = (uint16_t)g_adc_sps_measured;
  g_mb_input[MB_IR_ADC_OVERRUN]  = (uint16_t)(g_adc_overrun_count & 0xFFFFU);
  g_mb_input[MB_IR_UPTIME_S]     = (uint16_t)((HAL_GetTick() / 1000U) & 0xFFFFU);
}

/** Sama untuk holding register: cerminkan nilai driver yang sebenarnya. */
static void MB_RefreshHoldingRegs(const MB_Slave_t *mb)
{
  g_mb_hold[MB_HR_DAC_SETPOINT] = g_dac_setpoint;
  g_mb_hold[MB_HR_DO_WORD]      = MB_DoWord();
  g_mb_hold[MB_HR_SLAVE_ADDR]   = (uint16_t)mb->addr;
}

/**
  * Periksa keabsahan satu penulisan holding register TANPA menerapkannya.
  * Dipisah dari penerapannya karena FC 0x10 wajib memeriksa seluruh nilai
  * lebih dulu - kalau tidak, separuh setelan bisa berubah lalu sisanya
  * ditolak. Satu fungsi untuk kedua jalur supaya aturannya tidak pernah
  * berbeda antara FC 0x06 dan FC 0x10.
  * @return exception code, 0 bila sah.
  */
static uint8_t MB_ValidateHoldingReg(uint16_t idx, uint16_t val)
{
  if (idx >= MB_HR_COUNT)
  {
    return MB_EX_ILLEGAL_ADDRESS;
  }

  switch (idx)
  {
    case MB_HR_DAC_SETPOINT:
      return (val > MCP_MAX_CODE) ? (uint8_t)MB_EX_ILLEGAL_VALUE : 0U;

    case MB_HR_DO_WORD:
      return (val > 0x00FFU) ? (uint8_t)MB_EX_ILLEGAL_VALUE : 0U;

    case MB_HR_SLAVE_ADDR:
      return ((val < 1U) || (val > 247U)) ? (uint8_t)MB_EX_ILLEGAL_VALUE : 0U;

    default:
      return 0U;   /* alamat cadangan menerima nilai apa pun */
  }
}

/** Terapkan penulisan yang SUDAH lolos MB_ValidateHoldingReg(). */
static void MB_ApplyHoldingReg(uint16_t idx, uint16_t val, MB_Slave_t *mb)
{
  uint8_t i;

  g_mb_hold[idx] = val;

  switch (idx)
  {
    case MB_HR_DAC_SETPOINT:
      g_dac_setpoint = val;      /* MCP_Task() yang menuliskannya ke chip */
      break;

    case MB_HR_DO_WORD:
      for (i = 0U; i < DOUT_CH_COUNT; i++)
      {
        g_do[i] = (uint8_t)(((val >> i) & 0x01U) != 0U ? 1U : 0U);
      }
      break;                     /* DOUT_Task() yang menuliskannya ke chip */

    case MB_HR_SLAVE_ADDR:
      mb->addr = (uint8_t)val;
      break;

    default:
      break;                     /* alamat cadangan: cukup tersimpan di array */
  }
}

/* Helper: transmit --------------------------------------------------------- */

static void MB_DeAssert(MB_Slave_t *mb, bool tx_mode)
{
  if (mb->de_port == NULL)
  {
    return;   /* jalur TTL, tidak ada transceiver yang perlu diarahkan */
  }

  HAL_GPIO_WritePin(mb->de_port, mb->de_pin,
                    tx_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void MB_StartRx(MB_Slave_t *mb)
{
  mb->frame_ready = 0U;
  mb->rx_len      = 0U;

  (void)HAL_UARTEx_ReceiveToIdle_IT(mb->huart, mb->rx, (uint16_t)MB_ADU_MAX);
}

static void MB_SendResponse(MB_Slave_t *mb, uint16_t len)
{
  uint16_t crc;

  crc = MB_Crc16(mb->tx, len);
  mb->tx[len]      = (uint8_t)(crc & 0xFFU);        /* CRC low dulu */
  mb->tx[len + 1U] = (uint8_t)((crc >> 8) & 0xFFU);
  len += 2U;

  mb->tx_busy = 1U;

  /* DE dinaikkan SEBELUM transfer dimulai, dan baru diturunkan di callback
     TxCplt. HAL memanggil callback itu setelah flag TC - bukan TXE - jadi
     byte terakhir dijamin sudah keluar sepenuhnya dari shift register
     sebelum transceiver dikembalikan ke mode terima. Menurunkannya di TXE
     akan memotong byte terakhir di kabel. */
  MB_DeAssert(mb, true);

  if (HAL_UART_Transmit_DMA(mb->huart, mb->tx, len) != HAL_OK)
  {
    MB_DeAssert(mb, false);
    mb->tx_busy = 0U;
    MB_StartRx(mb);
    return;
  }

  mb->tx_frame_count++;
}

static uint16_t MB_BuildException(MB_Slave_t *mb, uint8_t fc, uint8_t ex)
{
  mb->tx[0] = mb->addr;
  mb->tx[1] = (uint8_t)(fc | 0x80U);
  mb->tx[2] = ex;

  mb->exception_count++;
  mb->last_exception = ex;

  return 3U;
}

/* Pengurai ----------------------------------------------------------------- */

/**
  * Proses satu ADU yang CRC-nya sudah valid dan alamatnya sudah cocok.
  * @return panjang jawaban di mb->tx TANPA CRC, 0 bila tidak perlu menjawab.
  */
static uint16_t MB_Process(MB_Slave_t *mb, const uint8_t *pdu, uint16_t len)
{
  uint8_t  fc = pdu[1];
  uint16_t start;
  uint16_t qty;
  uint16_t i;

  mb->last_fc        = fc;
  mb->last_exception = 0U;

  switch (fc)
  {
    /* ---- Baca bit: coil (DO) dan discrete input (DI) ---- */
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE:
    {
      uint16_t max_bits = (fc == MB_FC_READ_COILS) ? MB_COIL_COUNT : MB_DI_COUNT;
      uint8_t  nbytes;

      if (len != 8U) { return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE); }

      start = (uint16_t)(((uint16_t)pdu[2] << 8) | pdu[3]);
      qty   = (uint16_t)(((uint16_t)pdu[4] << 8) | pdu[5]);

      if ((qty < 1U) || (qty > 2000U))
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE);
      }
      if (((uint32_t)start + qty) > max_bits)
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_ADDRESS);
      }

      nbytes = (uint8_t)((qty + 7U) / 8U);

      mb->tx[0] = mb->addr;
      mb->tx[1] = fc;
      mb->tx[2] = nbytes;
      (void)memset(&mb->tx[3], 0, nbytes);

      for (i = 0U; i < qty; i++)
      {
        bool on = (fc == MB_FC_READ_COILS)
                    ? (g_do[start + i] != 0U)
                    : MB_GetDiscrete((uint16_t)(start + i));

        if (on)
        {
          mb->tx[3U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
        }
      }

      return (uint16_t)(3U + nbytes);
    }

    /* ---- Baca register: holding dan input ---- */
    case MB_FC_READ_HOLDING:
    case MB_FC_READ_INPUT:
    {
      uint16_t max_regs = (fc == MB_FC_READ_HOLDING) ? (uint16_t)MB_HR_COUNT
                                                      : (uint16_t)MB_IR_COUNT;

      if (len != 8U) { return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE); }

      start = (uint16_t)(((uint16_t)pdu[2] << 8) | pdu[3]);
      qty   = (uint16_t)(((uint16_t)pdu[4] << 8) | pdu[5]);

      if ((qty < 1U) || (qty > 125U))
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE);
      }
      if (((uint32_t)start + qty) > max_regs)
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_ADDRESS);
      }

      /* Segarkan alamat "hidup" dulu supaya HMI selalu dapat nilai terkini */
      if (fc == MB_FC_READ_HOLDING) { MB_RefreshHoldingRegs(mb); }
      else                          { MB_RefreshInputRegs();     }

      mb->tx[0] = mb->addr;
      mb->tx[1] = fc;
      mb->tx[2] = (uint8_t)(qty * 2U);

      for (i = 0U; i < qty; i++)
      {
        uint16_t v = (fc == MB_FC_READ_HOLDING)
                       ? g_mb_hold[start + i]
                       : g_mb_input[start + i];

        mb->tx[3U + (i * 2U)]      = (uint8_t)(v >> 8);
        mb->tx[3U + (i * 2U) + 1U] = (uint8_t)(v & 0xFFU);
      }

      return (uint16_t)(3U + (qty * 2U));
    }

    /* ---- Tulis satu coil ---- */
    case MB_FC_WRITE_COIL:
    {
      uint16_t val;

      if (len != 8U) { return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE); }

      start = (uint16_t)(((uint16_t)pdu[2] << 8) | pdu[3]);
      val   = (uint16_t)(((uint16_t)pdu[4] << 8) | pdu[5]);

      if (start >= MB_COIL_COUNT)
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_ADDRESS);
      }
      if ((val != 0xFF00U) && (val != 0x0000U))
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE);
      }

      g_do[start] = (uint8_t)((val == 0xFF00U) ? 1U : 0U);

      /* Jawaban FC 05 adalah gema permintaannya, tanpa CRC lama */
      (void)memcpy(mb->tx, pdu, 6U);
      mb->tx[0] = mb->addr;
      return 6U;
    }

    /* ---- Tulis satu holding register ---- */
    case MB_FC_WRITE_REG:
    {
      uint16_t val;
      uint8_t  ex;

      if (len != 8U) { return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE); }

      start = (uint16_t)(((uint16_t)pdu[2] << 8) | pdu[3]);
      val   = (uint16_t)(((uint16_t)pdu[4] << 8) | pdu[5]);

      if (start >= MB_HR_COUNT)
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_ADDRESS);
      }

      ex = MB_ValidateHoldingReg(start, val);
      if (ex != 0U)
      {
        return MB_BuildException(mb, fc, ex);
      }

      MB_ApplyHoldingReg(start, val, mb);

      (void)memcpy(mb->tx, pdu, 6U);
      mb->tx[0] = mb->addr;
      return 6U;
    }

    /* ---- Tulis banyak coil ---- */
    case MB_FC_WRITE_COILS:
    {
      uint8_t nbytes;

      if (len < 10U) { return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE); }

      start  = (uint16_t)(((uint16_t)pdu[2] << 8) | pdu[3]);
      qty    = (uint16_t)(((uint16_t)pdu[4] << 8) | pdu[5]);
      nbytes = pdu[6];

      if ((qty < 1U) || (qty > 1968U) || (nbytes != (uint8_t)((qty + 7U) / 8U)) ||
          (len != (uint16_t)(9U + nbytes)))
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE);
      }
      if (((uint32_t)start + qty) > MB_COIL_COUNT)
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_ADDRESS);
      }

      for (i = 0U; i < qty; i++)
      {
        uint8_t byte = pdu[7U + (i / 8U)];

        g_do[start + i] = (uint8_t)(((byte >> (i % 8U)) & 0x01U) != 0U ? 1U : 0U);
      }

      mb->tx[0] = mb->addr;
      mb->tx[1] = fc;
      mb->tx[2] = pdu[2];
      mb->tx[3] = pdu[3];
      mb->tx[4] = pdu[4];
      mb->tx[5] = pdu[5];
      return 6U;
    }

    /* ---- Tulis banyak holding register ---- */
    case MB_FC_WRITE_REGS:
    {
      uint8_t nbytes;

      if (len < 11U) { return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE); }

      start  = (uint16_t)(((uint16_t)pdu[2] << 8) | pdu[3]);
      qty    = (uint16_t)(((uint16_t)pdu[4] << 8) | pdu[5]);
      nbytes = pdu[6];

      if ((qty < 1U) || (qty > 123U) || (nbytes != (uint8_t)(qty * 2U)) ||
          (len != (uint16_t)(9U + nbytes)))
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_VALUE);
      }
      if (((uint32_t)start + qty) > MB_HR_COUNT)
      {
        return MB_BuildException(mb, fc, MB_EX_ILLEGAL_ADDRESS);
      }

      /* Periksa SELURUH nilai dulu, baru terapkan. Modbus mensyaratkan
         penulisan banyak register bersifat semua-atau-tidak sama sekali -
         jangan sampai separuh setelan berubah lalu sisanya ditolak. */
      for (i = 0U; i < qty; i++)
      {
        uint16_t v = (uint16_t)(((uint16_t)pdu[7U + (i * 2U)] << 8) |
                                 pdu[7U + (i * 2U) + 1U]);
        uint8_t  ex = MB_ValidateHoldingReg((uint16_t)(start + i), v);

        if (ex != 0U)
        {
          return MB_BuildException(mb, fc, ex);
        }
      }

      for (i = 0U; i < qty; i++)
      {
        uint16_t v = (uint16_t)(((uint16_t)pdu[7U + (i * 2U)] << 8) |
                                 pdu[7U + (i * 2U) + 1U]);

        MB_ApplyHoldingReg((uint16_t)(start + i), v, mb);
      }

      mb->tx[0] = mb->addr;
      mb->tx[1] = fc;
      mb->tx[2] = pdu[2];
      mb->tx[3] = pdu[3];
      mb->tx[4] = pdu[4];
      mb->tx[5] = pdu[5];
      return 6U;
    }

    default:
      return MB_BuildException(mb, fc, MB_EX_ILLEGAL_FUNCTION);
  }
}

/* API ---------------------------------------------------------------------- */

void MB_SlaveInit(MB_Slave_t *mb,
                  UART_HandleTypeDef *huart,
                  GPIO_TypeDef *de_port, uint16_t de_pin,
                  uint8_t addr)
{
  if (mb == NULL)
  {
    return;
  }

  (void)memset(mb, 0, sizeof(*mb));

  mb->huart   = huart;
  mb->de_port = de_port;
  mb->de_pin  = de_pin;
  mb->addr    = addr;

  /* Sebagai slave, transceiver HARUS diam di mode terima selama menganggur -
     kalau tidak, bus master HMI akan ditabrak. */
  MB_DeAssert(mb, false);

  MB_StartRx(mb);
}

void MB_SlaveOnRxEvent(MB_Slave_t *mb, uint16_t size)
{
  /* Konteks ISR: hanya catat dan tandai. HAL sudah menghentikan penerimaan;
     penyalinan buffer dan pengarmingan ulang dilakukan MB_SlaveTask() supaya
     data tidak tertimpa frame berikutnya di tengah pengolahan. */
  mb->rx_len      = size;
  mb->frame_ready = 1U;
}

void MB_SlaveOnTxComplete(MB_Slave_t *mb)
{
  /* Dipanggil setelah flag TC, jadi byte terakhir sudah benar-benar keluar. */
  MB_DeAssert(mb, false);
  mb->tx_busy = 0U;

  MB_StartRx(mb);
}

void MB_SlaveOnUartError(MB_Slave_t *mb)
{
  mb->uart_err_count++;

  /* Framing/overrun/noise di jalur RS485 itu wajar - yang penting penerimaan
     di-arm ulang, kalau tidak slave jadi tuli permanen setelah satu derau. */
  MB_DeAssert(mb, false);
  mb->tx_busy = 0U;

  MB_StartRx(mb);
}

void MB_SlaveTask(MB_Slave_t *mb)
{
  uint16_t len;
  uint16_t crc_calc;
  uint16_t crc_rx;
  uint16_t resp_len;

  if ((mb->huart == NULL) || (mb->frame_ready == 0U) || (mb->tx_busy != 0U))
  {
    return;
  }

  len = mb->rx_len;

  /* Salin dulu, baru arm ulang penerimaan - selisih waktu tuli jadi sependek
     mungkin tanpa mengorbankan isi buffer yang sedang diolah. */
  if (len > MB_ADU_MAX)
  {
    len = MB_ADU_MAX;
  }

  (void)memcpy(mb->work, mb->rx, len);
  MB_StartRx(mb);

  /* ADU terpendek yang sah = alamat + FC + 2 byte CRC */
  if (len < 4U)
  {
    return;
  }

  mb->rx_frame_count++;

  crc_calc = MB_Crc16(mb->work, (uint16_t)(len - 2U));
  crc_rx   = (uint16_t)(((uint16_t)mb->work[len - 1U] << 8) | mb->work[len - 2U]);

  if (crc_calc != crc_rx)
  {
    mb->crc_err_count++;
    return;   /* frame rusak - diam saja, master akan mengulang */
  }

  if ((mb->work[0] != mb->addr) && (mb->work[0] != (uint8_t)MB_ADDR_BROADCAST))
  {
    mb->not_for_us_count++;
    return;
  }

  resp_len = MB_Process(mb, mb->work, len);

  /* Broadcast dieksekusi tapi TIDAK dijawab - kalau semua slave menjawab,
     bus akan bertabrakan. */
  if ((resp_len == 0U) || (mb->work[0] == (uint8_t)MB_ADDR_BROADCAST))
  {
    return;
  }

  MB_SendResponse(mb, resp_len);
}

/* Pintasan instance HMI ---------------------------------------------------- */

void MB_HmiInit(UART_HandleTypeDef *huart)
{
  MB_SlaveInit(&g_mb_hmi, huart,
               DO_RS485_2_DE_GPIO_Port, DO_RS485_2_DE_Pin,
               MB_HMI_DEFAULT_ADDR);
}

void MB_HmiTask(void)
{
  MB_SlaveTask(&g_mb_hmi);
}

/* Pintasan instance Orange Pi ---------------------------------------------- */

void MB_OpiInit(UART_HandleTypeDef *huart)
{
  /* de_port = NULL: jalur TTL 3V3 point-to-point ke Orange Pi, tidak ada
     transceiver RS485 yang perlu diarahkan. MB_DeAssert() akan langsung
     keluar, jadi seluruh sisa inti slave berjalan sama persis. */
  MB_SlaveInit(&g_mb_opi, huart,
               NULL, 0U,
               MB_OPI_DEFAULT_ADDR);
}

void MB_OpiTask(void)
{
  MB_SlaveTask(&g_mb_opi);
}
