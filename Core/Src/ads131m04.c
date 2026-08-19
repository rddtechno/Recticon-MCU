/**
  ******************************************************************************
  * @file    ads131m04.c
  * @brief   Driver ADS131M04 - ADC 24-bit 4 kanal simultan (SPI) - Recticon
  ******************************************************************************
  */

#include "ads131m04.h"
#include <string.h>

/* Variabel Live Expressions ------------------------------------------------ */
volatile uint8_t  g_adc_present        = 0U;
volatile int8_t   g_adc_last_result    = ADC_OK;
volatile uint8_t  g_adc_running        = 0U;
volatile uint8_t  g_adc_init_stage     = (uint8_t)ADC_STAGE_NONE;

volatile uint8_t  g_adc_rx_dump[ADC_FRAME_BYTES] = {0};
volatile uint16_t g_adc_gain1_want     = 0U;
volatile uint16_t g_adc_clock_want     = 0U;
volatile uint16_t g_adc_unlock_resp    = 0U;
volatile uint16_t g_adc_wreg_ack       = 0U;
volatile uint16_t g_adc_wreg_base      = 0x6000U;
volatile uint32_t g_adc_wreg_retry_count = 0U;
volatile uint32_t g_adc_presc_cfg      = SPI_BAUDRATEPRESCALER_32;  /* 3.125 Mbit/s */
volatile uint32_t g_adc_presc_run      = SPI_BAUDRATEPRESCALER_8;   /* 12.5  Mbit/s */

volatile uint16_t g_adc_probe_base[ADC_PROBE_COUNT] = {0};
volatile uint16_t g_adc_probe_ack[ADC_PROBE_COUNT]  = {0};
volatile uint16_t g_adc_probe_rb[ADC_PROBE_COUNT]   = {0};

volatile uint16_t g_adc_id_reg         = 0U;
volatile uint16_t g_adc_status_reg     = 0U;
volatile uint16_t g_adc_mode_reg       = 0U;
volatile uint16_t g_adc_clock_reg      = 0U;
volatile uint16_t g_adc_gain1_reg      = 0U;
volatile uint16_t g_adc_cfg_reg        = 0U;
volatile uint16_t g_adc_reset_resp     = 0U;

volatile int32_t  g_adc_raw[ADC_CH_COUNT]  = {0};
volatile int32_t  g_adc_avg[ADC_CH_COUNT]  = {0};
volatile float    g_adc_volt[ADC_CH_COUNT] = {0.0f};

volatile float    g_rect_voltage       = 0.0f;
volatile float    g_rect_current       = 0.0f;
volatile float    g_batt_voltage       = 0.0f;
volatile float    g_batt_current       = 0.0f;

volatile uint32_t g_adc_drdy_count     = 0U;
volatile uint32_t g_adc_sample_count   = 0U;
volatile uint32_t g_adc_overrun_count  = 0U;
volatile uint32_t g_adc_error_count    = 0U;
volatile uint32_t g_adc_last_hal_error = 0U;
volatile uint32_t g_adc_sps_measured   = 0U;

volatile uint8_t  g_adc_cmd            = ADC_CMD_IDLE;
/* OSR 256 -> 16 kSPS. BUKAN laju tercepat yang mungkin (OSR 128 = 32 kSPS),
   tapi 32 kSPS terbukti melampaui kemampuan layan: satu frame memakan ~21 us
   termasuk overhead HAL DMA, sementara periodenya cuma 31.25 us - hasilnya
   31% konversi terlewat. Di 16 kSPS periode jadi 62.5 us dan overrun hilang
   sama sekali. Bonusnya OSR lebih tinggi = modulator merata-ratakan lebih
   banyak = noise per sampel lebih rendah. */
volatile uint8_t  g_adc_osr_sel        = (uint8_t)ADC_OSR_256;
volatile uint8_t  g_adc_gain_code[ADC_CH_COUNT] =
{
  (uint8_t)ADC_GAIN_1,   /* CH0 Rectifier Voltage -> FSR +-1.2 V  */
  (uint8_t)ADC_GAIN_4,   /* CH1 Rectifier Current -> FSR +-300 mV */
  (uint8_t)ADC_GAIN_1,   /* CH2 Battery Voltage   -> FSR +-1.2 V  */
  (uint8_t)ADC_GAIN_4    /* CH3 Battery Current   -> FSR +-300 mV */
};

volatile uint16_t g_adc_avg_ms         = 10U;

volatile float    g_adc_rect_v_scale   = 1.0f;
volatile float    g_adc_rect_v_offset  = 0.0f;
volatile float    g_adc_rect_i_scale   = 1.0f;
volatile float    g_adc_rect_i_offset  = 0.0f;
volatile float    g_adc_batt_v_scale   = 1.0f;
volatile float    g_adc_batt_v_offset  = 0.0f;
volatile float    g_adc_batt_i_scale   = 1.0f;
volatile float    g_adc_batt_i_offset  = 0.0f;

/* State privat ------------------------------------------------------------- */
static SPI_HandleTypeDef *s_hspi = NULL;

/* Buffer DMA. Tidak boleh di stack - DMA menulis ke sini di luar alur main. */
static uint8_t  s_tx_frame[ADC_FRAME_BYTES];
static uint8_t  s_rx_frame[ADC_FRAME_BYTES];

/** 1 = DMA sedang berjalan; dipakai untuk mendeteksi overrun DRDY. */
static volatile uint8_t s_dma_busy = 0U;

/* Akumulator rata-rata, diisi ISR dan dikosongkan ADC_Task().
   int64: 8.4e6 (skala penuh) x jumlah sampel per jendela bisa lewat int32
   pada 64 kSPS dengan jendela 10 ms (640 sampel -> 5.4e9). */
static volatile int64_t  s_acc[ADC_CH_COUNT] = {0};
static volatile uint32_t s_acc_count = 0U;

static uint32_t s_avg_tick  = 0U;
static uint32_t s_sps_tick  = 0U;
static uint32_t s_sps_last  = 0U;

/* FSR per kanal (Volt), diturunkan dari gain saat konfigurasi diterapkan. */
static float s_fsr_v[ADC_CH_COUNT] = {ADC_VREF_V, ADC_VREF_V, ADC_VREF_V, ADC_VREF_V};

/* Helper privat ------------------------------------------------------------ */

static void ADC_NoteError(ADC_Status_t st)
{
  g_adc_last_result = (int8_t)st;
  g_adc_error_count++;

  if (s_hspi != NULL)
  {
    g_adc_last_hal_error = HAL_SPI_GetError(s_hspi);
  }
}

static inline void ADC_CsLow(void)
{
  HAL_GPIO_WritePin(DO_ADC_CS_GPIO_Port, DO_ADC_CS_Pin, GPIO_PIN_RESET);
}

static inline void ADC_CsHigh(void)
{
  HAL_GPIO_WritePin(DO_ADC_CS_GPIO_Port, DO_ADC_CS_Pin, GPIO_PIN_SET);
}

/**
  * Jeda pendek di sekitar tepi CS (~200 ns @100 MHz).
  *
  * Frame dipisahkan oleh CS, dan device butuh CS benar-benar sempat naik
  * sebelum frame berikutnya dimulai. Tanpa jeda, dua panggilan transfer yang
  * berurutan hanya terpisah beberapa siklus overhead pemanggilan fungsi -
  * cukup rapat untuk membuat device menganggapnya satu frame panjang, dan
  * command word frame kedua tidak pernah terbaca sebagai command.
  */
static inline void ADC_CsDelay(void)
{
  volatile uint32_t i;

  for (i = 0U; i < 20U; i++)
  {
    __NOP();
  }
}

/**
  * Ganti kecepatan SPI1 saat berjalan.
  *
  * Akses register dan akuisisi punya kebutuhan yang berlawanan: konfigurasi
  * jarang dilakukan tapi harus andal, sedangkan streaming butuh cepat supaya
  * frame selesai sebelum DRDY berikutnya. Jadi keduanya dipisah - lambat
  * saat mengatur register, cepat saat mengalirkan data.
  */
static void ADC_SetSpiSpeed(uint32_t presc)
{
  if ((s_hspi == NULL) || (s_hspi->Init.BaudRatePrescaler == presc))
  {
    return;
  }

  s_hspi->Init.BaudRatePrescaler = presc;

  /* State sudah READY sejak MX_SPI1_Init(), jadi HAL_SPI_Init() di sini
     hanya mengkonfigurasi ulang register - MspInit tidak dipanggil lagi dan
     tautan DMA tetap utuh. */
  (void)HAL_SPI_Init(s_hspi);
}

/** Sisipkan word 16-bit ke posisi word ke-n frame (MSB dulu, byte ke-3 = 0). */
static void ADC_FrameSetWord(uint8_t *frame, uint8_t idx, uint16_t val)
{
  uint32_t off = (uint32_t)idx * ADC_WORD_BYTES;

  frame[off]     = (uint8_t)(val >> 8);
  frame[off + 1U] = (uint8_t)(val & 0xFFU);
  frame[off + 2U] = 0x00U;   /* 8 bit terbawah word 24-bit tidak dipakai */
}

/** Ambil 16 bit teratas word ke-n (register selalu 16 bit). */
static uint16_t ADC_FrameGetWord(const uint8_t *frame, uint8_t idx)
{
  uint32_t off = (uint32_t)idx * ADC_WORD_BYTES;

  return (uint16_t)(((uint16_t)frame[off] << 8) | (uint16_t)frame[off + 1U]);
}

/** Ambil satu kanal sebagai kode 24-bit two's complement -> int32. */
static int32_t ADC_FrameGetSample(const uint8_t *frame, uint8_t ch)
{
  uint32_t off = ((uint32_t)ch + 1U) * ADC_WORD_BYTES;   /* word0 = response */
  int32_t  v;

  v = (int32_t)(((uint32_t)frame[off] << 16) |
                ((uint32_t)frame[off + 1U] << 8) |
                 (uint32_t)frame[off + 2U]);

  if ((v & 0x00800000) != 0)
  {
    v -= 0x01000000;   /* perluasan tanda dari 24 ke 32 bit */
  }

  return v;
}

/**
  * Satu frame blocking. Hanya dipakai saat konfigurasi - jalur akuisisi
  * memakai DMA. Jangan dipanggil saat g_adc_running = 1.
  */
static ADC_Status_t ADC_Transfer(uint16_t cmd, uint16_t arg, bool has_arg)
{
  HAL_StatusTypeDef hal;

  if (s_hspi == NULL) { return ADC_ERR_NOT_INIT; }
  if (s_dma_busy != 0U) { return ADC_ERR_BUSY; }

  (void)memset(s_tx_frame, 0, sizeof(s_tx_frame));
  ADC_FrameSetWord(s_tx_frame, 0U, cmd);

  if (has_arg)
  {
    ADC_FrameSetWord(s_tx_frame, 1U, arg);
  }

  ADC_CsLow();
  ADC_CsDelay();
  hal = HAL_SPI_TransmitReceive(s_hspi, s_tx_frame, s_rx_frame,
                                (uint16_t)ADC_FRAME_BYTES, ADC_SPI_TIMEOUT_MS);
  ADC_CsHigh();
  ADC_CsDelay();

  /* Simpan mentahnya untuk diperiksa di Live Expressions. Hanya di jalur
     konfigurasi yang blocking - jalur akuisisi tidak lewat sini supaya tidak
     ada penyalinan tambahan pada laju puluhan ribu frame per detik. */
  {
    uint8_t i;

    for (i = 0U; i < (uint8_t)ADC_FRAME_BYTES; i++)
    {
      g_adc_rx_dump[i] = s_rx_frame[i];
    }
  }

  return (hal == HAL_OK) ? ADC_OK : ADC_ERR_SPI;
}

/* API ---------------------------------------------------------------------- */

ADC_Status_t ADC_ReadReg(uint8_t addr, uint16_t *val)
{
  ADC_Status_t st;

  if (val == NULL) { return ADC_ERR_PARAM; }

  /* Frame 1: kirim RREG. Jawabannya belum ada di frame ini. */
  st = ADC_Transfer(ADC_CMD_RREG(addr, 1U), 0U, false);
  if (st != ADC_OK) { return st; }

  /* Frame 2: NULL, isi register muncul di word0. */
  st = ADC_Transfer(ADC_CMD_NULL, 0U, false);
  if (st != ADC_OK) { return st; }

  *val = ADC_FrameGetWord(s_rx_frame, 0U);
  return ADC_OK;
}

/** Satu kali percobaan tulis, tanpa verifikasi. */
static ADC_Status_t ADC_WriteRegOnce(uint8_t addr, uint16_t val)
{
  ADC_Status_t st;
  uint16_t     cmd;

  /* Basis opcode diambil dari g_adc_wreg_base, bukan konstanta, supaya hasil
     ADC_CMD_PROBE_WREG bisa langsung dipakai tanpa rebuild. */
  cmd = (uint16_t)(g_adc_wreg_base | (((uint16_t)addr & 0x3FU) << 7));

  /* Frame 1: WREG di word0, data di word1. */
  st = ADC_Transfer(cmd, val, true);
  if (st != ADC_OK) { return st; }

  /* Frame 2: NULL untuk menarik keluar acknowledge. Verifikasi sesungguhnya
     tetap dilakukan dengan membaca ulang register - jauh lebih meyakinkan
     daripada mempercayai satu word ack. Tapi ack-nya direkam karena sangat
     menentukan saat mendiagnosa: device menjawab WREG dengan
     0b010a aaaa annn nnnn, jadi untuk GAIN1 (0x04, 1 register) semestinya
     0x4200. Kalau yang muncul 0x0000, perintah tulisnya tidak pernah
     diterima - bukan sekadar nilainya tidak nyangkut. */
  st = ADC_Transfer(ADC_CMD_NULL, 0U, false);

  g_adc_wreg_ack = ADC_FrameGetWord(s_rx_frame, 0U);

  return st;
}

ADC_Status_t ADC_WriteReg(uint8_t addr, uint16_t val)
{
  ADC_Status_t st;
  uint8_t      attempt;

  /* Tulis lalu baca-banding, ulangi bila belum nyangkut.
     Penulisan register di chip ini terbukti kadang tidak diterima (lihat
     catatan intermiten saat bring-up), dan SPI tidak punya mekanisme ACK di
     lapisan bus - jadi satu-satunya cara memastikan adalah membaca ulang.
     Yang dihitung bukan cuma berhasil/gagal tapi berapa kali harus diulang,
     supaya kualitas jalurnya terukur, bukan sekadar tertutupi. */
  for (attempt = 0U; attempt < ADC_WREG_MAX_TRIES; attempt++)
  {
    uint16_t rb = 0U;

    st = ADC_WriteRegOnce(addr, val);
    if (st != ADC_OK)
    {
      return st;
    }

    st = ADC_ReadReg(addr, &rb);
    if (st != ADC_OK)
    {
      return st;
    }

    if (rb == val)
    {
      return ADC_OK;
    }

    g_adc_wreg_retry_count++;
  }

  /* Tidak pernah sama persis. Tetap dikembalikan sebagai OK - beberapa
     register memang punya bit reserved yang terbaca berbeda dari yang
     ditulis, dan penilaian akhirnya milik pemanggil yang tahu mask-nya. */
  return ADC_OK;
}

ADC_Status_t ADC_ApplyConfig(void)
{
  ADC_Status_t st;
  uint16_t     mode  = 0U;
  uint16_t     clk   = 0U;
  uint16_t     gain1 = 0U;
  uint16_t     rb    = 0U;
  uint8_t      i;

  if (s_hspi == NULL) { return ADC_ERR_NOT_INIT; }

  /* UNLOCK sebelum menulis apa pun. Saat terkunci, device hanya menerima
     NULL, RREG, dan UNLOCK - jadi RREG tetap sukses sementara setiap WREG
     diabaikan diam-diam. Persis pola "baca bisa, tulis tidak". Default
     setelah reset semestinya sudah terbuka, tapi perintah ini murah dan
     tidak berefek samping kalau memang sudah terbuka. */
  st = ADC_Transfer(ADC_CMD_UNLOCK, 0U, false);
  if (st != ADC_OK) { return st; }

  st = ADC_Transfer(ADC_CMD_NULL, 0U, false);   /* tarik keluar acknowledge */
  if (st != ADC_OK) { return st; }

  g_adc_unlock_resp = ADC_FrameGetWord(s_rx_frame, 0U);

  g_adc_init_stage = (uint8_t)ADC_STAGE_MODE;

  /* --- MODE: hanya pastikan word 24-bit dan CRC masukan mati ---
     Sengaja read-modify-write, bukan menulis nilai absolut: bit lain di MODE
     (TIMEOUT, DRDY_SEL, DRDY_FMT) dibiarkan pada default pabrik supaya tidak
     ada perilaku yang berubah diam-diam. */
  st = ADC_ReadReg(ADC_REG_MODE, &mode);
  if (st != ADC_OK) { return st; }

  g_adc_mode_reg = mode;

  {
    uint16_t want = (uint16_t)((mode & (uint16_t)(~(ADC_MODE_WLENGTH_Msk | ADC_MODE_RX_CRC_EN)))
                               | ADC_MODE_WLENGTH_24);

    /* RX_CRC_EN dimatikan supaya host tidak wajib menghitung CRC untuk tiap
       frame yang dikirim. CRC keluaran device tetap ada di word terakhir dan
       bisa diperiksa nanti bila dibutuhkan. */
    if (want != mode)
    {
      st = ADC_WriteReg(ADC_REG_MODE, want);
      if (st != ADC_OK) { return st; }

      st = ADC_ReadReg(ADC_REG_MODE, &rb);
      if (st != ADC_OK) { return st; }

      g_adc_mode_reg = rb;

      if ((rb & (ADC_MODE_WLENGTH_Msk | ADC_MODE_RX_CRC_EN)) != ADC_MODE_WLENGTH_24)
      {
        return ADC_ERR_VERIFY;
      }
    }
  }

  /* --- GAIN1: gain PGA keempat kanal, 3 bit per kanal --- */
  g_adc_init_stage = (uint8_t)ADC_STAGE_GAIN;

  for (i = 0U; i < ADC_CH_COUNT; i++)
  {
    uint8_t code = (uint8_t)(g_adc_gain_code[i] & 0x07U);

    gain1 |= (uint16_t)((uint16_t)code << (i * 4U));
    s_fsr_v[i] = ADC_VREF_V / (float)(1U << code);
  }

  g_adc_gain1_want = gain1;

  st = ADC_WriteReg(ADC_REG_GAIN1, gain1);
  if (st != ADC_OK) { return st; }

  st = ADC_ReadReg(ADC_REG_GAIN1, &rb);
  if (st != ADC_OK) { return st; }

  g_adc_gain1_reg = rb;

  /* Bandingkan hanya keempat field gain (bit 14:12, 10:8, 6:4, 2:0). Bit
     sisanya di tiap nibble adalah reserved; kalau device memaksanya ke nilai
     tertentu, perbandingan bulat-bulat akan menjatuhkan verifikasi padahal
     gain-nya sudah masuk. Alasan yang sama seperti pada CLOCK. */
  if ((rb & 0x7777U) != (gain1 & 0x7777U))
  {
    return ADC_ERR_VERIFY;
  }

  /* --- CLOCK: aktifkan 4 kanal, set OSR & mode daya ---
     Read-modify-write lagi: bit selain CHxEN/OSR/PWR di register ini tidak
     didokumentasikan seragam antar varian ADS131M0x, jadi lebih aman
     mempertahankan nilai bawaan chip daripada menulis konstanta tebakan. */
  g_adc_init_stage = (uint8_t)ADC_STAGE_CLOCK;

  st = ADC_ReadReg(ADC_REG_CLOCK, &clk);
  if (st != ADC_OK) { return st; }

  clk = (uint16_t)(clk & (uint16_t)(~(ADC_CLOCK_CHEN_Msk | ADC_CLOCK_OSR_Msk | ADC_CLOCK_PWR_Msk)));
  clk = (uint16_t)(clk | ADC_CLOCK_CHEN_Msk);   /* CH0..CH3 aktif semua */
  clk = (uint16_t)(clk | (uint16_t)(((uint16_t)g_adc_osr_sel << ADC_CLOCK_OSR_Pos) & ADC_CLOCK_OSR_Msk));
  clk = (uint16_t)(clk | ADC_CLOCK_PWR_HR);

  g_adc_clock_want = clk;

  st = ADC_WriteReg(ADC_REG_CLOCK, clk);
  if (st != ADC_OK) { return st; }

  st = ADC_ReadReg(ADC_REG_CLOCK, &rb);
  if (st != ADC_OK) { return st; }

  g_adc_clock_reg = rb;

  /* Hanya field yang memang kita kuasai yang dibandingkan. Bit lain di CLOCK
     tidak seragam antar varian ADS131M0x - ada yang read-only atau reserved,
     dan membandingkannya bulat-bulat akan menjatuhkan verifikasi padahal
     konfigurasinya sudah benar. */
  {
    const uint16_t mask = (uint16_t)(ADC_CLOCK_CHEN_Msk | ADC_CLOCK_OSR_Msk | ADC_CLOCK_PWR_Msk);

    if ((rb & mask) != (clk & mask))
    {
      return ADC_ERR_VERIFY;
    }
  }

  g_adc_init_stage = (uint8_t)ADC_STAGE_DONE;

  return ADC_OK;
}

/**
  * Sapu kandidat basis opcode WREG.
  *
  * RREG sudah terbukti jalan di 0xA000, artinya posisi field alamat (bit
  * 12..7) dan penjajaran frame sudah benar - yang tersisa hanya 3 bit
  * penanda perintah. Jadi cukup mencoba prefix lain sambil melihat apakah
  * GAIN1 benar-benar berubah. Isi register yang berubah adalah bukti yang
  * jauh lebih kuat daripada word acknowledgment.
  *
  * GAIN1 dipilih sebagai sasaran karena hanya mengubah penguatan PGA - tidak
  * mengganggu laju sampel maupun kanal yang aktif, dan nilainya dikembalikan
  * ke 0 setelah tiap percobaan.
  */
static ADC_Status_t ADC_ProbeWregOpcode(void)
{
  static const uint16_t cand[ADC_PROBE_COUNT] = {0x6000U, 0x4000U, 0x2000U, 0x8000U};
  uint16_t saved_base = g_adc_wreg_base;
  uint8_t  i;

  for (i = 0U; i < (uint8_t)ADC_PROBE_COUNT; i++)
  {
    uint16_t rb = 0xFFFFU;

    g_adc_wreg_base    = cand[i];
    g_adc_probe_base[i] = cand[i];
    g_adc_probe_ack[i]  = 0U;
    g_adc_probe_rb[i]   = 0xFFFFU;

    if (ADC_WriteReg(ADC_REG_GAIN1, 0x2020U) != ADC_OK)
    {
      continue;
    }

    g_adc_probe_ack[i] = g_adc_wreg_ack;

    if (ADC_ReadReg(ADC_REG_GAIN1, &rb) != ADC_OK)
    {
      continue;
    }

    g_adc_probe_rb[i] = rb;

    /* Kembalikan ke nilai reset supaya percobaan berikutnya berangkat dari
       titik yang sama - kalau tidak, kandidat yang gagal bisa terlihat
       "berhasil" karena membaca sisa tulisan kandidat sebelumnya. */
    (void)ADC_WriteReg(ADC_REG_GAIN1, 0x0000U);
  }

  g_adc_wreg_base = saved_base;

  return ADC_OK;
}

/** Refresh seluruh salinan register untuk diamati di Live Expressions. */
static ADC_Status_t ADC_DumpRegs(void)
{
  ADC_Status_t st;
  uint16_t     v = 0U;

  st = ADC_ReadReg(ADC_REG_ID, &v);
  if (st != ADC_OK) { return st; }
  g_adc_id_reg = v;

  st = ADC_ReadReg(ADC_REG_STATUS, &v);
  if (st != ADC_OK) { return st; }
  g_adc_status_reg = v;

  st = ADC_ReadReg(ADC_REG_MODE, &v);
  if (st != ADC_OK) { return st; }
  g_adc_mode_reg = v;

  st = ADC_ReadReg(ADC_REG_CLOCK, &v);
  if (st != ADC_OK) { return st; }
  g_adc_clock_reg = v;

  st = ADC_ReadReg(ADC_REG_GAIN1, &v);
  if (st != ADC_OK) { return st; }
  g_adc_gain1_reg = v;

  st = ADC_ReadReg(ADC_REG_CFG, &v);
  if (st != ADC_OK) { return st; }
  g_adc_cfg_reg = v;

  return ADC_OK;
}

void ADC_Start(void)
{
  uint32_t i;

  /* Kosongkan akumulator supaya jendela rata-rata pertama tidak tercampur
     sisa sesi sebelumnya. */
  __disable_irq();
  for (i = 0U; i < ADC_CH_COUNT; i++)
  {
    s_acc[i] = 0;
  }
  s_acc_count = 0U;
  __enable_irq();

  /* Frame keluar untuk akuisisi = command NULL (semua nol). Diisi eksplisit,
     tidak mengandalkan sisa isi dari transaksi konfigurasi terakhir. */
  (void)memset(s_tx_frame, 0, sizeof(s_tx_frame));

  /* Naikkan ke kecepatan streaming: frame 18 byte harus selesai sebelum DRDY
     berikutnya (31.25 us @32 kSPS). */
  ADC_SetSpiSpeed(g_adc_presc_run);

  s_avg_tick = HAL_GetTick();
  s_sps_tick = HAL_GetTick();
  s_sps_last = g_adc_drdy_count;

  /* Buang DRDY yang sempat terlatch selama konfigurasi. Hanya jalur EXTI
     milik DRDY yang dibersihkan - vektor EXTI9_5 dipakai bersama INTB
     MCP23S17, jadi pending NVIC-nya tidak boleh disapu di sini. */
  __HAL_GPIO_EXTI_CLEAR_IT(DI_ADC_DRDY_Pin);

  g_adc_running = 1U;
}

void ADC_Stop(void)
{
  g_adc_running = 0U;
}

ADC_Status_t ADC_Init(SPI_HandleTypeDef *hspi)
{
  ADC_Status_t st;

  s_hspi           = hspi;
  g_adc_running    = 0U;
  g_adc_present    = 0U;
  s_dma_busy       = 0U;
  g_adc_init_stage = (uint8_t)ADC_STAGE_RESET;

  ADC_CsHigh();

  /* Seluruh urutan init hanya akses register - pakai kecepatan konfigurasi
     yang lebih lambat. Kecepatan streaming dipasang belakangan di
     ADC_Start(). */
  ADC_SetSpiSpeed(g_adc_presc_cfg);

  /* Reset hardware: RST aktif LOW, datasheet minta pulsa >=2 us lalu jeda
     >=5 us sebelum komunikasi. 1 ms jauh di atas keduanya dan hanya
     dijalankan saat init. */
  HAL_GPIO_WritePin(DO_ADC_RST_GPIO_Port, DO_ADC_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(DO_ADC_RST_GPIO_Port, DO_ADC_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(5U);

  /* Frame pertama setelah reset membawa word pengenal dari device. Nilainya
     tidak dijadikan syarat lulus - hanya direkam untuk diperiksa saat
     bring-up, karena word ini berbeda antar varian dan revisi. */
  st = ADC_Transfer(ADC_CMD_NULL, 0U, false);
  if (st != ADC_OK)
  {
    ADC_NoteError(st);
    return st;
  }

  g_adc_reset_resp = ADC_FrameGetWord(s_rx_frame, 0U);

  g_adc_init_stage = (uint8_t)ADC_STAGE_DUMP;

  st = ADC_DumpRegs();
  if (st != ADC_OK)
  {
    ADC_NoteError(st);
    return st;
  }

  /* Keberadaan chip dibuktikan lewat tulis-baca-banding register yang memang
     harus dikonfigurasi (GAIN1 & CLOCK), bukan lewat pencocokan nilai ID -
     SPI tidak punya ACK, jadi hanya read-back yang membuktikan ada lawan
     bicara di ujung sana. */
  st = ADC_ApplyConfig();
  if (st != ADC_OK)
  {
    ADC_NoteError(st);
    return st;
  }

  g_adc_present      = 1U;
  g_adc_last_result  = (int8_t)ADC_OK;

  ADC_Start();

  return ADC_OK;
}

/* Jalur akuisisi ----------------------------------------------------------- */

void ADC_OnDrdy(void)
{
  if ((g_adc_running == 0U) || (s_hspi == NULL))
  {
    return;
  }

  g_adc_drdy_count++;

  if (s_dma_busy != 0U)
  {
    /* Frame sebelumnya belum selesai. Sampel ini dilewat - dicatat, bukan
       dipaksakan, karena memulai transfer di atas transfer yang berjalan
       justru merusak sinkronisasi frame. */
    g_adc_overrun_count++;
    return;
  }

  s_dma_busy = 1U;
  ADC_CsLow();

  if (HAL_SPI_TransmitReceive_DMA(s_hspi, s_tx_frame, s_rx_frame,
                                  (uint16_t)ADC_FRAME_BYTES) != HAL_OK)
  {
    ADC_CsHigh();
    s_dma_busy = 0U;
    g_adc_error_count++;
  }
}

void ADC_OnFrameComplete(void)
{
  uint8_t i;

  ADC_CsHigh();
  s_dma_busy = 0U;

  for (i = 0U; i < ADC_CH_COUNT; i++)
  {
    int32_t v = ADC_FrameGetSample(s_rx_frame, i);

    g_adc_raw[i] = v;
    s_acc[i]    += (int64_t)v;
  }

  s_acc_count++;
  g_adc_sample_count++;
}

void ADC_OnSpiError(void)
{
  ADC_CsHigh();
  s_dma_busy = 0U;
  g_adc_error_count++;

  if (s_hspi != NULL)
  {
    g_adc_last_hal_error = HAL_SPI_GetError(s_hspi);
  }
}

/* Task --------------------------------------------------------------------- */

/** Ambil isi akumulator lalu kosongkan, dengan interrupt dimatikan sesaat. */
static uint32_t ADC_TakeAverage(int32_t *out)
{
  int64_t  acc[ADC_CH_COUNT];
  uint32_t n;
  uint8_t  i;

  __disable_irq();
  n = s_acc_count;
  for (i = 0U; i < ADC_CH_COUNT; i++)
  {
    acc[i]   = s_acc[i];
    s_acc[i] = 0;
  }
  s_acc_count = 0U;
  __enable_irq();

  if (n == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < ADC_CH_COUNT; i++)
  {
    out[i] = (int32_t)(acc[i] / (int64_t)n);
  }

  return n;
}

void ADC_Task(void)
{
  uint32_t now;
  uint8_t  cmd;

  if (s_hspi == NULL)
  {
    return;
  }

  now = HAL_GetTick();

  /* --- 1. Jendela rata-rata + konversi ke besaran fisik --- */
  if ((uint32_t)(now - s_avg_tick) >= (uint32_t)g_adc_avg_ms)
  {
    int32_t avg[ADC_CH_COUNT];

    s_avg_tick = now;

    if (ADC_TakeAverage(avg) != 0U)
    {
      uint8_t i;

      for (i = 0U; i < ADC_CH_COUNT; i++)
      {
        g_adc_avg[i]  = avg[i];
        g_adc_volt[i] = ((float)avg[i] / ADC_FULLSCALE_CODE) * s_fsr_v[i];
      }

      g_rect_voltage = (g_adc_volt[0] * g_adc_rect_v_scale) + g_adc_rect_v_offset;
      g_rect_current = (g_adc_volt[1] * g_adc_rect_i_scale) + g_adc_rect_i_offset;
      g_batt_voltage = (g_adc_volt[2] * g_adc_batt_v_scale) + g_adc_batt_v_offset;
      g_batt_current = (g_adc_volt[3] * g_adc_batt_i_scale) + g_adc_batt_i_offset;
    }
  }

  /* --- 2. Ukur laju sampel nyata tiap detik ---
     Inilah yang menjawab berapa CLKIN board sebenarnya, tanpa perlu
     membongkar hardware: fDATA = fCLKIN / 2 / OSR. */
  if ((uint32_t)(now - s_sps_tick) >= 1000U)
  {
    uint32_t count = g_adc_drdy_count;
    uint32_t dt    = (uint32_t)(now - s_sps_tick);

    g_adc_sps_measured = (uint32_t)(((uint64_t)(count - s_sps_last) * 1000U) / dt);

    s_sps_last = count;
    s_sps_tick = now;
  }

  /* --- 3. Perintah manual --- */
  cmd = g_adc_cmd;

  if (cmd == (uint8_t)ADC_CMD_IDLE)
  {
    return;
  }

  switch ((ADC_UserCmd_t)cmd)
  {
    case ADC_CMD_DUMP_REGS:
    case ADC_CMD_APPLY_CFG:
    case ADC_CMD_PROBE_WREG:
    {
      ADC_Status_t st;
      uint8_t      was_running = g_adc_running;

      /* Akuisisi dihentikan dulu: ADC_Transfer() blocking dan akan bentrok
         dengan DMA yang dipicu DRDY kalau dibiarkan jalan. */
      ADC_Stop();

      /* Tunggu frame yang sedang berjalan selesai - normalnya ~12 us. Diberi
         batas waktu supaya perintah dari Live Expressions tidak pernah bisa
         menggantung main loop kalau DMA tersangkut. */
      {
        uint32_t wait_start = HAL_GetTick();

        while ((s_dma_busy != 0U) && ((uint32_t)(HAL_GetTick() - wait_start) < 5U))
        {
          /* tunggu */
        }

        if (s_dma_busy != 0U)
        {
          ADC_OnSpiError();   /* paksa lepas: CS naik, tanda sibuk dibuang */
        }
      }

      /* Ketiga perintah di cabang ini hanya akses register */
      ADC_SetSpiSpeed(g_adc_presc_cfg);

      if (cmd == (uint8_t)ADC_CMD_DUMP_REGS)
      {
        st = ADC_DumpRegs();
      }
      else if (cmd == (uint8_t)ADC_CMD_PROBE_WREG)
      {
        st = ADC_ProbeWregOpcode();
      }
      else
      {
        st = ADC_ApplyConfig();
      }

      if (st == ADC_OK) { g_adc_last_result = (int8_t)ADC_OK; }
      else              { ADC_NoteError(st);                  }

      if (was_running != 0U)
      {
        ADC_Start();
      }
      break;
    }

    case ADC_CMD_REINIT:
      (void)ADC_Init(s_hspi);
      break;

    case ADC_CMD_START:
      ADC_Start();
      g_adc_last_result = (int8_t)ADC_OK;
      break;

    case ADC_CMD_STOP:
      ADC_Stop();
      g_adc_last_result = (int8_t)ADC_OK;
      break;

    default:
      g_adc_last_result = (int8_t)ADC_ERR_PARAM;
      break;
  }

  g_adc_cmd = (uint8_t)ADC_CMD_IDLE;
}
