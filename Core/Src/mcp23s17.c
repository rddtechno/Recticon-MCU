/**
  ******************************************************************************
  * @file    mcp23s17.c
  * @brief   Driver MCP23S17 I/O Expander 16-bit (SPI) - Recticon
  ******************************************************************************
  */

#include "mcp23s17.h"

/* Variabel Live Expressions - instance DIGITAL OUTPUT ---------------------- */
volatile uint8_t  g_do[DOUT_CH_COUNT]   = {0U};      /* g_do[n] -> GPAn */
volatile uint8_t  g_dout_cmd            = DOUT_CMD_IDLE;

volatile uint8_t  g_dout_present        = 0U;
volatile int8_t   g_dout_last_result    = IOX_OK;
volatile uint8_t  g_dout_olat           = 0U;
volatile uint8_t  g_dout_olat_rb        = 0U;
volatile uint8_t  g_dout_gpio_rb        = 0U;
volatile uint8_t  g_dout_iocon_rb       = 0U;
volatile uint32_t g_dout_write_count    = 0U;
volatile uint32_t g_dout_error_count    = 0U;
volatile uint32_t g_dout_last_hal_error = 0U;

/* Variabel Live Expressions - instance DIGITAL INPUT ----------------------- */
volatile uint8_t  g_di_a[DIN_CH_PER_PORT] = {0U};   /* g_di_a[n] -> GPAn */
volatile uint8_t  g_di_b[DIN_CH_PER_PORT] = {0U};   /* g_di_b[n] -> GPBn */
volatile uint16_t g_di_word             = 0U;

volatile uint8_t  g_di_present          = 0U;
volatile int8_t   g_di_last_result      = IOX_OK;
volatile uint8_t  g_di_iocon_rb         = 0U;
volatile uint32_t g_di_read_count       = 0U;
volatile uint32_t g_di_change_count     = 0U;
volatile uint32_t g_di_error_count      = 0U;
volatile uint32_t g_di_last_hal_error   = 0U;

volatile uint8_t  g_di_cmd              = DIN_CMD_IDLE;
volatile uint16_t g_di_poll_ms          = DIN_POLL_MS_DEFAULT;
volatile uint16_t g_di_pullup           = 0xFFFFU;  /* lihat catatan di header */
volatile uint16_t g_di_invert           = 0x0000U;

volatile uint32_t g_di_irq_count        = 0U;
volatile uint32_t g_di_irqa_count       = 0U;
volatile uint32_t g_di_irqb_count       = 0U;
volatile uint8_t  g_di_int_level        = 0x03U;    /* 11 = kedua INT idle HIGH */
volatile uint16_t g_di_irq_enable       = 0xFFFFU;

/* State privat ------------------------------------------------------------- */
static IOX_Dev_t s_dout;
static uint16_t  s_dout_last_written = 0xFFFFU;  /* >0xFF = belum pernah ditulis */

static IOX_Dev_t s_din;
static uint32_t  s_din_tick        = 0U;
static uint16_t  s_din_last_pullup = 0xFFFFU;
static uint16_t  s_din_last_logic  = 0U;
static uint8_t   s_din_has_sample  = 0U;   /* 0 = s_din_last_logic belum sah */
static uint16_t  s_din_last_irqen  = 0xFFFFU;

/** Diangkat ISR, dikonsumsi DIN_Task(). volatile: ditulis di luar alur main. */
static volatile uint8_t s_din_irq_flag = 0U;

/* Nilai IOCON yang kita pakai:
     BANK   = 0 -> peta register sekuensial (A/B berpasangan)
     MIRROR = 0 -> INTA/INTB terpisah (tak dipakai di chip output)
     SEQOP  = 0 -> alamat auto-increment, jadi 1 transaksi bisa isi A lalu B
     DISSLW = 0, HAEN = 0, ODR = 0, INTPOL = 0                               */
#define IOX_IOCON_VALUE   0x00U

/* Helper privat ------------------------------------------------------------ */

static inline void IOX_CsLow(IOX_Dev_t *dev)
{
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void IOX_CsHigh(IOX_Dev_t *dev)
{
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void DOUT_NoteError(IOX_Status_t st)
{
  g_dout_last_result = (int8_t)st;
  g_dout_error_count++;

  if (s_dout.hspi != NULL)
  {
    g_dout_last_hal_error = HAL_SPI_GetError(s_dout.hspi);
  }
}

static void DIN_NoteError(IOX_Status_t st)
{
  g_di_last_result = (int8_t)st;
  g_di_error_count++;

  if (s_din.hspi != NULL)
  {
    g_di_last_hal_error = HAL_SPI_GetError(s_din.hspi);
  }
}

/* API generik -------------------------------------------------------------- */

void IOX_Bind(IOX_Dev_t *dev,
              SPI_HandleTypeDef *hspi,
              GPIO_TypeDef *cs_port,  uint16_t cs_pin,
              GPIO_TypeDef *rst_port, uint16_t rst_pin,
              uint8_t hw_addr)
{
  if (dev == NULL)
  {
    return;
  }

  dev->hspi     = hspi;
  dev->cs_port  = cs_port;
  dev->cs_pin   = cs_pin;
  dev->rst_port = rst_port;
  dev->rst_pin  = rst_pin;
  dev->hw_addr  = hw_addr;
  dev->ready    = 0U;
}

IOX_Status_t IOX_WriteReg(IOX_Dev_t *dev, uint8_t reg, uint8_t val)
{
  uint8_t      buf[3];
  HAL_StatusTypeDef hal;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }

  buf[0] = IOX_OP_WRITE(dev->hw_addr);
  buf[1] = reg;
  buf[2] = val;

  IOX_CsLow(dev);
  hal = HAL_SPI_Transmit(dev->hspi, buf, 3U, IOX_SPI_TIMEOUT_MS);
  IOX_CsHigh(dev);

  return (hal == HAL_OK) ? IOX_OK : IOX_ERR_SPI;
}

IOX_Status_t IOX_ReadReg(IOX_Dev_t *dev, uint8_t reg, uint8_t *val)
{
  uint8_t      tx[3];
  uint8_t      rx[3] = {0U, 0U, 0U};
  HAL_StatusTypeDef hal;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }
  if (val == NULL)                          { return IOX_ERR_PARAM;    }

  /* Satu transfer full-duplex untuk seluruh frame. Sengaja tidak memakai
     HAL_SPI_Receive(): di master 2-line HAL meneruskannya ke
     TransmitReceive dengan buffer yang sama, jadi isi buffer penerima ikut
     terkirim ke MOSI. Dengan cara ini byte dummy-nya eksplisit. */
  tx[0] = IOX_OP_READ(dev->hw_addr);
  tx[1] = reg;
  tx[2] = 0xFFU;   /* dummy - MOSI diabaikan chip selama fase data baca */

  IOX_CsLow(dev);
  hal = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 3U, IOX_SPI_TIMEOUT_MS);
  IOX_CsHigh(dev);

  if (hal != HAL_OK)
  {
    return IOX_ERR_SPI;
  }

  *val = rx[2];
  return IOX_OK;
}

IOX_Status_t IOX_WriteReg16(IOX_Dev_t *dev, uint8_t reg_a, uint16_t val)
{
  uint8_t      buf[4];
  HAL_StatusTypeDef hal;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }

  /* SEQOP = 0 -> pointer alamat naik sendiri: reg_a lalu reg_a+1 (port B) */
  buf[0] = IOX_OP_WRITE(dev->hw_addr);
  buf[1] = reg_a;
  buf[2] = (uint8_t)(val & 0xFFU);          /* port A = bit 0..7  */
  buf[3] = (uint8_t)((val >> 8) & 0xFFU);   /* port B = bit 8..15 */

  IOX_CsLow(dev);
  hal = HAL_SPI_Transmit(dev->hspi, buf, 4U, IOX_SPI_TIMEOUT_MS);
  IOX_CsHigh(dev);

  return (hal == HAL_OK) ? IOX_OK : IOX_ERR_SPI;
}

IOX_Status_t IOX_ReadReg16(IOX_Dev_t *dev, uint8_t reg_a, uint16_t *val)
{
  uint8_t      tx[4];
  uint8_t      rx[4] = {0U, 0U, 0U, 0U};
  HAL_StatusTypeDef hal;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }
  if (val == NULL)                          { return IOX_ERR_PARAM;    }

  tx[0] = IOX_OP_READ(dev->hw_addr);
  tx[1] = reg_a;
  tx[2] = 0xFFU;   /* dummy */
  tx[3] = 0xFFU;   /* dummy */

  IOX_CsLow(dev);
  hal = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 4U, IOX_SPI_TIMEOUT_MS);
  IOX_CsHigh(dev);

  if (hal != HAL_OK)
  {
    return IOX_ERR_SPI;
  }

  /* rx[2] = port A (reg_a), rx[3] = port B (reg_a + 1, auto-increment) */
  *val = (uint16_t)(((uint16_t)rx[3] << 8) | (uint16_t)rx[2]);
  return IOX_OK;
}

IOX_Status_t IOX_HardReset(IOX_Dev_t *dev)
{
  IOX_Status_t st;
  uint8_t      iocon = 0xFFU;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }

  /* CS harus HIGH selama reset supaya chip tidak menangkap frame sampah */
  IOX_CsHigh(dev);

  /* RST aktif LOW. Datasheet minta lebar pulsa minimal ~1 us; 1 ms jauh di
     atas itu dan tidak mengganggu apa pun karena hanya dijalankan saat init. */
  HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);
  HAL_Delay(1U);

  /* Setelah reset hardware, BANK dijamin 0 - jadi alamat IOCON = 0x0A valid. */
  st = IOX_WriteReg(dev, IOX_REG_IOCON, IOX_IOCON_VALUE);
  if (st != IOX_OK)
  {
    return st;
  }

  st = IOX_ReadReg(dev, IOX_REG_IOCON, &iocon);
  if (st != IOX_OK)
  {
    return st;
  }

  /* Bit 0 IOCON tidak terpakai dan selalu terbaca 0; sisanya harus persis
     seperti yang ditulis. Kalau tidak cocok, kemungkinan CS/MISO belum benar
     atau chip tidak terpasang (MISO mengambang -> terbaca 0x00 atau 0xFF). */
  if (iocon != IOX_IOCON_VALUE)
  {
    return IOX_ERR_VERIFY;
  }

  return IOX_OK;
}

IOX_Status_t IOX_SelfTest(IOX_Dev_t *dev)
{
  static const uint16_t patterns[2] = {0xAA55U, 0x55AAU};
  IOX_Status_t st;
  uint16_t     rb = 0U;
  uint8_t      i;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }

  /* DEFVALA/DEFVALB dipakai sebagai scratch: isinya hanya berarti bila
     interrupt-on-change diaktifkan (INTCON/GPINTEN), dan itu tidak pernah
     kita nyalakan di chip output. Jadi menulisnya sama sekali tidak
     menggerakkan pin. */
  for (i = 0U; i < 2U; i++)
  {
    st = IOX_WriteReg16(dev, IOX_REG_DEFVALA, patterns[i]);
    if (st != IOX_OK)
    {
      return st;
    }

    st = IOX_ReadReg16(dev, IOX_REG_DEFVALA, &rb);
    if (st != IOX_OK)
    {
      return st;
    }

    if (rb != patterns[i])
    {
      /* Dua pola saling komplemen: 0x0000 atau 0xFFFF konstan = MISO mati /
         chip absen; nilai tergeser = polaritas atau fase SPI salah. */
      return IOX_ERR_VERIFY;
    }
  }

  /* Kembalikan scratch ke nilai default reset */
  (void)IOX_WriteReg16(dev, IOX_REG_DEFVALA, 0x0000U);

  dev->ready = 1U;
  return IOX_OK;
}

IOX_Status_t IOX_ConfigAllOutput(IOX_Dev_t *dev)
{
  IOX_Status_t st;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }

  /* 1. Latch dulu ke 0 selagi pin masih Hi-Z (mode input setelah reset) */
  st = IOX_WriteReg16(dev, IOX_REG_OLATA, 0x0000U);
  if (st != IOX_OK) { return st; }

  /* 2. Matikan fitur yang tidak dipakai di chip output */
  st = IOX_WriteReg16(dev, IOX_REG_GPINTENA, 0x0000U);   /* interrupt-on-change off */
  if (st != IOX_OK) { return st; }

  st = IOX_WriteReg16(dev, IOX_REG_IPOLA, 0x0000U);      /* polaritas normal        */
  if (st != IOX_OK) { return st; }

  st = IOX_WriteReg16(dev, IOX_REG_GPPUA, 0x0000U);      /* pull-up off             */
  if (st != IOX_OK) { return st; }

  /* 3. Baru balik arah pin jadi output - saat ini latch sudah pasti 0 */
  st = IOX_WriteReg16(dev, IOX_REG_IODIRA, 0x0000U);
  if (st != IOX_OK) { return st; }

  return IOX_OK;
}

IOX_Status_t IOX_ConfigAllInput(IOX_Dev_t *dev, uint16_t pullup_mask)
{
  IOX_Status_t st;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }

  /* Setelah reset chip memang sudah mode input; tetap ditulis eksplisit agar
     tidak bergantung pada kondisi sisa bila fungsi ini dipanggil ulang. */
  st = IOX_WriteReg16(dev, IOX_REG_IODIRA, 0xFFFFU);
  if (st != IOX_OK) { return st; }

  /* IPOL dibiarkan 0 - inversi logika ditangani di firmware supaya nilai
     mentah register tetap bisa diamati saat bring-up. */
  st = IOX_WriteReg16(dev, IOX_REG_IPOLA, 0x0000U);
  if (st != IOX_OK) { return st; }

  /* Interrupt dimatikan dulu di sini; diaktifkan terpisah lewat
     IOX_ConfigIntOnChange() setelah konfigurasi dasar selesai, supaya tidak
     ada INT yang memicu sebelum pull-up sempat di-set. */
  st = IOX_WriteReg16(dev, IOX_REG_GPINTENA, 0x0000U);
  if (st != IOX_OK) { return st; }

  st = IOX_WriteReg16(dev, IOX_REG_GPPUA, pullup_mask);
  if (st != IOX_OK) { return st; }

  return IOX_OK;
}

IOX_Status_t IOX_ConfigIntOnChange(IOX_Dev_t *dev, uint16_t enable_mask)
{
  IOX_Status_t st;

  if ((dev == NULL) || (dev->hspi == NULL)) { return IOX_ERR_NOT_INIT; }

  /* INTCON = 0 -> pembanding adalah nilai pin sebelumnya, bukan DEFVAL.
     Artinya setiap perubahan level memicu INT, naik maupun turun. */
  st = IOX_WriteReg16(dev, IOX_REG_INTCONA, 0x0000U);
  if (st != IOX_OK) { return st; }

  st = IOX_WriteReg16(dev, IOX_REG_GPINTENA, enable_mask);
  if (st != IOX_OK) { return st; }

  return IOX_OK;
}

IOX_Status_t IOX_SetOutputs(IOX_Dev_t *dev, uint16_t val)
{
  return IOX_WriteReg16(dev, IOX_REG_OLATA, val);
}

IOX_Status_t IOX_GetOutputLatch(IOX_Dev_t *dev, uint16_t *val)
{
  return IOX_ReadReg16(dev, IOX_REG_OLATA, val);
}

IOX_Status_t IOX_ReadPins(IOX_Dev_t *dev, uint16_t *val)
{
  return IOX_ReadReg16(dev, IOX_REG_GPIOA, val);
}

/* ==========================================================================
 * Instance DIGITAL OUTPUT
 * ========================================================================== */


/* Helper privat ------------------------------------------------------------ */

/**
  * Susun g_do[0..7] menjadi satu byte OLATA (bit n = GPAn).
  * Nilai selain 0/1 dianggap ON dan dinormalkan jadi 1, lalu dipantulkan
  * balik ke array supaya koreksinya kelihatan di Live Expressions.
  */
static uint8_t DOUT_ComposeByte(void)
{
  uint8_t val = 0U;
  uint8_t i;

  for (i = 0U; i < DOUT_CH_COUNT; i++)
  {
    uint8_t v = g_do[i];   /* volatile - baca sekali saja per kanal */

    if (v != 0U)
    {
      if (v != 1U)
      {
        g_do[i] = 1U;
      }

      val |= (uint8_t)(1U << i);
    }
  }

  return val;
}

/** Sebarkan satu byte ke g_do[] supaya array tetap jadi cermin isi OLATA. */
static void DOUT_SyncArray(uint8_t val)
{
  uint8_t i;

  for (i = 0U; i < DOUT_CH_COUNT; i++)
  {
    g_do[i] = (uint8_t)(((val >> i) & 0x01U) != 0U ? 1U : 0U);
  }
}

/**
  * Tulis byte ke OLATA (port A saja), perbarui state bayangan, lalu baca
  * balik GPIOA.
  *
  * Readback dilakukan di sini - bukan di DOUT_Task() - supaya SEMUA jalur
  * yang mengubah output ikut ter-cover: perubahan g_do[], DOUT_SetChannel(),
  * DOUT_WriteByte(), ALL_ON/ALL_OFF, dan tiap langkah walk test. Biayanya 1
  * transaksi 3 byte (~10 us @3.125 Mbit/s) dan hanya jalan saat ada
  * perubahan, jadi tidak membebani loop.
  *
  * GPIOA membaca kondisi pin yang sebenarnya, jadi nilainya sekaligus
  * verifikasi: kalau g_dout_gpio_rb tidak sama dengan g_dout_olat, berarti
  * penulisan tidak mendarat atau ada beban yang menahan pin.
  */
static IOX_Status_t DOUT_PushByte(uint8_t val)
{
  IOX_Status_t st;
  uint8_t      pins = 0U;

  st = IOX_WriteReg(&s_dout, IOX_REG_OLATA, val);

  if (st != IOX_OK)
  {
    DOUT_NoteError(st);
    return st;
  }

  s_dout_last_written = (uint16_t)val;
  g_dout_olat         = val;
  g_dout_write_count++;

  st = IOX_ReadReg(&s_dout, IOX_REG_GPIOA, &pins);

  if (st != IOX_OK)
  {
    /* Penulisannya sendiri sudah sukses, jadi state bayangan di atas tetap
       sah - yang gagal cuma konfirmasinya. Tetap dicatat supaya tidak lolos
       diam-diam, dan g_dout_gpio_rb sengaja tidak diperbarui agar nilai
       basinya tidak disangka hasil baru. */
    DOUT_NoteError(st);
    return IOX_OK;
  }

  g_dout_gpio_rb     = pins;
  g_dout_last_result = (int8_t)IOX_OK;

  return IOX_OK;
}

/* API ---------------------------------------------------------------------- */

IOX_Status_t DOUT_Init(SPI_HandleTypeDef *hspi)
{
  IOX_Status_t st;
  uint8_t      iocon = 0U;

  IOX_Bind(&s_dout, hspi,
           DO_IOEXP_DO_CS_GPIO_Port,  DO_IOEXP_DO_CS_Pin,
           DO_IOEXP_DO_RST_GPIO_Port, DO_IOEXP_DO_RST_Pin,
           0U);   /* HAEN = 0 -> bit alamat diabaikan chip */

  g_dout_present      = 0U;
  s_dout_last_written = 0xFFFFU;

  st = IOX_HardReset(&s_dout);

  /* Baca IOCON apa pun hasilnya - saat gagal, nilai mentahnya justru yang
     paling membantu: 0x00 = MISO diam/chip absen, 0xFF = MISO ketarik high,
     nilai tergeser = fase/polaritas SPI salah. */
  if (IOX_ReadReg(&s_dout, IOX_REG_IOCON, &iocon) == IOX_OK)
  {
    g_dout_iocon_rb = iocon;
  }

  if (st != IOX_OK)
  {
    DOUT_NoteError(st);
    return st;
  }

  st = IOX_SelfTest(&s_dout);
  if (st != IOX_OK)
  {
    DOUT_NoteError(st);
    return st;
  }

  g_dout_present = 1U;

  /* Port A = 8 kanal relay, Port B = cadangan; keduanya dijadikan output
     bernilai 0 supaya tidak ada pin yang mengambang. */
  st = IOX_ConfigAllOutput(&s_dout);
  if (st != IOX_OK)
  {
    DOUT_NoteError(st);
    return st;
  }

  DOUT_SyncArray(0x00U);
  g_dout_olat         = 0U;
  s_dout_last_written = 0U;
  g_dout_last_result  = (int8_t)IOX_OK;

  return IOX_OK;
}

IOX_Status_t DOUT_SetChannel(uint8_t ch, bool on)
{
  if (ch >= DOUT_CH_COUNT)
  {
    g_dout_last_result = (int8_t)IOX_ERR_PARAM;
    return IOX_ERR_PARAM;
  }

  g_do[ch] = (uint8_t)(on ? 1U : 0U);

  return DOUT_PushByte(DOUT_ComposeByte());
}

bool DOUT_GetChannel(uint8_t ch)
{
  if (ch >= DOUT_CH_COUNT)
  {
    return false;
  }

  return (g_do[ch] != 0U);
}

IOX_Status_t DOUT_WriteByte(uint8_t val)
{
  DOUT_SyncArray(val);

  return DOUT_PushByte(val);
}

void DOUT_Task(void)
{
  IOX_Status_t st;
  uint8_t      value;
  uint8_t      cmd;

  if (s_dout.hspi == NULL)
  {
    return;
  }

  /* --- 1. Ikuti perubahan g_do[] --- */
  value = DOUT_ComposeByte();

  if ((uint16_t)value != s_dout_last_written)
  {
    (void)DOUT_PushByte(value);
  }

  /* --- 2. Perintah manual --- */
  cmd = g_dout_cmd;

  if (cmd == (uint8_t)DOUT_CMD_IDLE)
  {
    return;
  }

  switch ((DOUT_Cmd_t)cmd)
  {
    case DOUT_CMD_SELFTEST:
      st = IOX_SelfTest(&s_dout);
      g_dout_present = (uint8_t)((st == IOX_OK) ? 1U : 0U);

      if (st == IOX_OK) { g_dout_last_result = (int8_t)IOX_OK; }
      else              { DOUT_NoteError(st);                  }
      break;

    case DOUT_CMD_READBACK:
    {
      uint8_t olat  = 0U;
      uint8_t pins  = 0U;
      uint8_t iocon = 0U;

      st = IOX_ReadReg(&s_dout, IOX_REG_OLATA, &olat);
      if (st == IOX_OK) { st = IOX_ReadReg(&s_dout, IOX_REG_GPIOA, &pins);  }
      if (st == IOX_OK) { st = IOX_ReadReg(&s_dout, IOX_REG_IOCON, &iocon); }

      if (st == IOX_OK)
      {
        g_dout_olat_rb     = olat;
        g_dout_gpio_rb     = pins;
        g_dout_iocon_rb    = iocon;
        g_dout_last_result = (int8_t)IOX_OK;
      }
      else
      {
        DOUT_NoteError(st);
      }
      break;
    }

    case DOUT_CMD_REINIT:
      (void)DOUT_Init(s_dout.hspi);
      break;

    case DOUT_CMD_ALL_OFF:
      (void)DOUT_WriteByte(0x00U);
      break;

    case DOUT_CMD_ALL_ON:
      (void)DOUT_WriteByte(0xFFU);
      break;

    case DOUT_CMD_WALK_TEST:
    {
      uint8_t i;

      /* Blocking 8 x 300 ms = 2.4 s. Masih jauh di bawah timeout IWDG
         (~32.8 s) yang di-refresh di main loop setelah task ini selesai. */
      st = IOX_OK;

      for (i = 0U; (i < DOUT_CH_COUNT) && (st == IOX_OK); i++)
      {
        st = DOUT_PushByte((uint8_t)(1U << i));
        HAL_Delay(300U);
      }

      /* Kembalikan ke kondisi yang diminta operator lewat g_do[], termasuk
         bila gagal di tengah jalan - jangan tinggalkan kanal menyala. */
      (void)DOUT_PushByte(DOUT_ComposeByte());

      if (st != IOX_OK)
      {
        DOUT_NoteError(st);
      }
      break;
    }

    default:
      g_dout_last_result = (int8_t)IOX_ERR_PARAM;
      break;
  }

  g_dout_cmd = (uint8_t)DOUT_CMD_IDLE;
}

/* ==========================================================================
 * Instance DIGITAL INPUT - 16 kanal (PORT A + PORT B)
 * ========================================================================== */

/** Sebarkan hasil pembacaan ke g_di_a[]/g_di_b[] setelah dibalik g_di_invert. */
static void DIN_ScatterToArrays(uint16_t logic)
{
  uint8_t i;

  for (i = 0U; i < DIN_CH_PER_PORT; i++)
  {
    g_di_a[i] = (uint8_t)(((logic >> i) & 0x01U) != 0U ? 1U : 0U);
    g_di_b[i] = (uint8_t)(((logic >> (i + DIN_CH_PER_PORT)) & 0x01U) != 0U ? 1U : 0U);
  }
}

void DIN_OnInterrupt(uint16_t gpio_pin)
{
  /* Dipanggil dari konteks ISR. Sengaja tidak ada akses SPI di sini:
     transaksi blocking di dalam ISR akan menahan interrupt lain, termasuk
     DRDY ADS131M04 yang jauh lebih kritis waktunya. Cukup catat & tandai. */
  if (gpio_pin == DI_IOEXP_INTA_Pin)
  {
    g_di_irqa_count++;
  }
  else if (gpio_pin == DI_IOEXP_INTB_Pin)
  {
    g_di_irqb_count++;
  }
  else
  {
    return;   /* bukan punya kita (mis. DRDY ADS131M04) */
  }

  g_di_irq_count++;
  s_din_irq_flag = 1U;
}

/** Rekam level pin INTA/INTB - bit0 = INTA, bit1 = INTB, 1 = HIGH (idle). */
static uint8_t DIN_ReadIntLevel(void)
{
  uint8_t lvl = 0U;

  if (HAL_GPIO_ReadPin(DI_IOEXP_INTA_GPIO_Port, DI_IOEXP_INTA_Pin) != GPIO_PIN_RESET)
  {
    lvl |= 0x01U;
  }

  if (HAL_GPIO_ReadPin(DI_IOEXP_INTB_GPIO_Port, DI_IOEXP_INTB_Pin) != GPIO_PIN_RESET)
  {
    lvl |= 0x02U;
  }

  g_di_int_level = lvl;
  return lvl;
}

/** Satu siklus baca GPIOA+GPIOB lalu perbarui seluruh variabel tampilan.
    Pembacaan GPIO sekaligus MELEPAS INT di chip - itu memang mekanisme
    clear-nya menurut datasheet, tidak perlu baca INTCAP terpisah. */
static IOX_Status_t DIN_Sample(void)
{
  IOX_Status_t st;
  uint16_t     raw   = 0U;
  uint16_t     logic;

  st = IOX_ReadPins(&s_din, &raw);

  if (st != IOX_OK)
  {
    DIN_NoteError(st);
    return st;
  }

  logic = (uint16_t)(raw ^ g_di_invert);

  g_di_word = raw;
  DIN_ScatterToArrays(logic);

  g_di_read_count++;

  /* Hitung perubahan hanya setelah ada sampel pembanding yang sah, supaya
     pembacaan pertama tidak dihitung sebagai "berubah". Penghitung ini
     berguna saat bring-up: kalau melonjak terus padahal tidak ada yang
     ditekan, berarti pin mengambang atau kontaknya memantul. */
  if (s_din_has_sample == 0U)
  {
    s_din_has_sample = 1U;
  }
  else if (logic != s_din_last_logic)
  {
    g_di_change_count++;
  }
  else
  {
    /* nilai stabil - tidak ada yang dicatat */
  }

  s_din_last_logic = logic;
  g_di_last_result = (int8_t)IOX_OK;

  /* Setelah GPIO dibaca, INT semestinya sudah lepas kembali ke HIGH.
     Kalau di sini masih LOW berarti ada perubahan baru yang menyusul -
     DIN_Task() akan menyampelnya lagi di putaran berikutnya. */
  (void)DIN_ReadIntLevel();

  return IOX_OK;
}

IOX_Status_t DIN_Init(SPI_HandleTypeDef *hspi)
{
  IOX_Status_t st;
  uint8_t      iocon = 0U;

  IOX_Bind(&s_din, hspi,
           DO_IOEXP_DI_CS_GPIO_Port,  DO_IOEXP_DI_CS_Pin,
           DO_IOEXP_DI_RST_GPIO_Port, DO_IOEXP_DI_RST_Pin,
           0U);   /* HAEN = 0 -> bit alamat diabaikan chip */

  g_di_present     = 0U;
  s_din_has_sample = 0U;
  s_din_tick       = HAL_GetTick();

  st = IOX_HardReset(&s_din);

  if (IOX_ReadReg(&s_din, IOX_REG_IOCON, &iocon) == IOX_OK)
  {
    g_di_iocon_rb = iocon;
  }

  if (st != IOX_OK)
  {
    DIN_NoteError(st);
    return st;
  }

  st = IOX_SelfTest(&s_din);
  if (st != IOX_OK)
  {
    DIN_NoteError(st);
    return st;
  }

  g_di_present = 1U;

  st = IOX_ConfigAllInput(&s_din, g_di_pullup);
  if (st != IOX_OK)
  {
    DIN_NoteError(st);
    return st;
  }

  s_din_last_pullup = g_di_pullup;

  /* Ambil satu sampel awal supaya array langsung terisi kondisi nyata,
     tidak menampilkan nol semu sampai tick polling pertama tiba. Sekaligus
     melepas INT yang mungkin sudah aktif sejak sebelum reset. */
  (void)DIN_Sample();

  st = IOX_ConfigIntOnChange(&s_din, g_di_irq_enable);
  if (st != IOX_OK)
  {
    DIN_NoteError(st);
    return st;
  }

  s_din_last_irqen = g_di_irq_enable;

  /* Buang tepi turun yang sempat terlatch di EXTI selama urutan reset chip -
     kalau tidak, satu interrupt palsu langsung menyala begitu init selesai.
     __HAL_GPIO_EXTI_CLEAR_IT() bekerja per jalur EXTI, jadi aman.

     Pending NVIC hanya dibersihkan untuk EXTI4 yang vektornya milik INTA
     sendiri. Vektor EXTI9_5 SENGAJA tidak disentuh karena dipakai bersama
     DRDY ADS131M04 - membersihkannya di sini (DIN_CMD_REINIT bisa dipanggil
     saat akuisisi sedang jalan) berarti ikut membuang satu event DRDY.
     Kalaupun pending-nya tersisa, ISR cuma masuk sekali lalu tidak menemukan
     PR bit milik kita dan langsung keluar. */
  __HAL_GPIO_EXTI_CLEAR_IT(DI_IOEXP_INTA_Pin);
  __HAL_GPIO_EXTI_CLEAR_IT(DI_IOEXP_INTB_Pin);
  HAL_NVIC_ClearPendingIRQ(DI_IOEXP_INTA_EXTI_IRQn);
  s_din_irq_flag = 0U;

  g_di_last_result = (int8_t)IOX_OK;

  return IOX_OK;
}

bool DIN_GetA(uint8_t ch)
{
  if (ch >= DIN_CH_PER_PORT)
  {
    return false;
  }

  return (g_di_a[ch] != 0U);
}

bool DIN_GetB(uint8_t ch)
{
  if (ch >= DIN_CH_PER_PORT)
  {
    return false;
  }

  return (g_di_b[ch] != 0U);
}

IOX_Status_t DIN_ReadNow(void)
{
  if (s_din.hspi == NULL)
  {
    return IOX_ERR_NOT_INIT;
  }

  s_din_tick = HAL_GetTick();

  return DIN_Sample();
}

void DIN_Task(void)
{
  IOX_Status_t st;
  uint16_t     pullup;
  uint16_t     irqen;
  uint8_t      irq_hit;
  uint8_t      int_low;
  uint8_t      due;
  uint8_t      cmd;

  if (s_din.hspi == NULL)
  {
    return;
  }

  /* --- 1. Terapkan perubahan mask pull-up --- */
  pullup = g_di_pullup;

  if (pullup != s_din_last_pullup)
  {
    st = IOX_WriteReg16(&s_din, IOX_REG_GPPUA, pullup);

    if (st == IOX_OK)
    {
      s_din_last_pullup = pullup;
      g_di_last_result  = (int8_t)IOX_OK;
    }
    else
    {
      DIN_NoteError(st);
    }
  }

  /* --- 2. Terapkan perubahan mask interrupt-on-change --- */
  irqen = g_di_irq_enable;

  if (irqen != s_din_last_irqen)
  {
    st = IOX_ConfigIntOnChange(&s_din, irqen);

    if (st == IOX_OK)
    {
      s_din_last_irqen = irqen;
      g_di_last_result = (int8_t)IOX_OK;
    }
    else
    {
      DIN_NoteError(st);
    }
  }

  /* --- 3. Tentukan apakah perlu menyampel ---
     Tiga pemicu, sengaja berlapis:
       a. flag dari ISR       -> jalur normal, respons cepat
       b. level INT masih LOW -> menangkap kasus "INT nyangkut": chip menahan
          INT sampai GPIO dibaca, dan kalau tepi turunnya terlewat (mis.
          berubah persis saat pembacaan sebelumnya) tidak akan ada tepi baru
       c. jatuh tempo polling -> jaring pengaman terakhir bila keduanya gagal
     Flag dibaca-lalu-dinolkan SEBELUM sampling supaya event yang datang di
     tengah pembacaan tidak hilang - paling buruk hanya menyampel sekali lagi. */
  irq_hit = s_din_irq_flag;
  s_din_irq_flag = 0U;

  int_low = (uint8_t)((DIN_ReadIntLevel() & 0x03U) != 0x03U ? 1U : 0U);
  due     = (uint8_t)(((uint32_t)(HAL_GetTick() - s_din_tick) >= (uint32_t)g_di_poll_ms) ? 1U : 0U);

  if ((irq_hit != 0U) || (int_low != 0U) || (due != 0U))
  {
    s_din_tick = HAL_GetTick();
    (void)DIN_Sample();
  }

  /* --- 4. Perintah manual --- */
  cmd = g_di_cmd;

  if (cmd == (uint8_t)DIN_CMD_IDLE)
  {
    return;
  }

  switch ((DIN_Cmd_t)cmd)
  {
    case DIN_CMD_SELFTEST:
      st = IOX_SelfTest(&s_din);
      g_di_present = (uint8_t)((st == IOX_OK) ? 1U : 0U);

      if (st == IOX_OK) { g_di_last_result = (int8_t)IOX_OK; }
      else              { DIN_NoteError(st);                 }
      break;

    case DIN_CMD_READ_NOW:
      (void)DIN_ReadNow();
      break;

    case DIN_CMD_REINIT:
      (void)DIN_Init(s_din.hspi);
      break;

    default:
      g_di_last_result = (int8_t)IOX_ERR_PARAM;
      break;
  }

  g_di_cmd = (uint8_t)DIN_CMD_IDLE;
}
