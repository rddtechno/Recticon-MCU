/**
  ******************************************************************************
  * @file    modbus_master.c
  * @brief   Modbus RTU Master - Recticon Rectifier Controller
  ******************************************************************************
  */

#include "modbus_master.h"
#include "modbus_slave.h"   /* pakai ulang MB_Crc16() - CRC-nya sama persis */
#include <string.h>

/* Variabel Live Expressions - Power Meter ---------------------------------- */
volatile uint16_t   g_pm_raw[PM_IR_COUNT]      = {0};
volatile PM_Phase_t g_pm_phase[PM_PHASE_COUNT] = {{0}};
volatile float      g_pm_frequency             = 0.0f;
volatile int32_t    g_pm_energy_wh[PM_PHASE_COUNT]   = {0};
volatile int32_t    g_pm_energy_varh[PM_PHASE_COUNT] = {0};
volatile uint32_t   g_pm_meter_const           = 0U;

volatile uint8_t    g_pm_online                = 0U;
volatile uint8_t    g_pm_addr                  = PM_DEFAULT_ADDR;
volatile uint8_t    g_pm_last_exception        = 0U;
volatile uint32_t   g_pm_ok_count              = 0U;
volatile uint32_t   g_pm_timeout_count         = 0U;
volatile uint32_t   g_pm_crc_err_count         = 0U;
volatile uint32_t   g_pm_exception_count       = 0U;

/* Variabel Live Expressions - SinePower ST36 ------------------------------- */
volatile uint16_t   g_st36_raw[ST36_IR_COUNT] = {0};

volatile uint16_t   g_st36_cur_u_raw           = 0U;
volatile uint16_t   g_st36_cur_v_raw           = 0U;
volatile uint16_t   g_st36_cur_w_raw           = 0U;
volatile uint16_t   g_st36_idc_raw             = 0U;
volatile uint16_t   g_st36_vdc_raw             = 0U;
volatile uint16_t   g_st36_fault               = 0U;

volatile float      g_st36_cur_u               = 0.0f;
volatile float      g_st36_cur_v               = 0.0f;
volatile float      g_st36_cur_w               = 0.0f;
volatile float      g_st36_idc                 = 0.0f;
volatile float      g_st36_vdc                 = 0.0f;

/* 1.0 = belum dikalibrasi; dokumen protokol tidak menyebutkan satuan.
   Lihat peringatan di modbus_master.h. */
volatile float      g_st36_i_scale             = 1.0f;
volatile float      g_st36_v_scale             = 1.0f;

volatile uint8_t    g_st36_online              = 0U;
volatile uint8_t    g_st36_addr                = ST36_DEFAULT_ADDR;
volatile uint8_t    g_st36_last_exception      = 0U;
volatile uint32_t   g_st36_ok_count            = 0U;
volatile uint32_t   g_st36_timeout_count       = 0U;
volatile uint32_t   g_st36_crc_err_count       = 0U;
volatile uint32_t   g_st36_exception_count     = 0U;

/* Diagnostik bus ----------------------------------------------------------- */
volatile uint32_t   g_mbm_tx_count             = 0U;
volatile uint32_t   g_mbm_rx_count             = 0U;
volatile uint32_t   g_mbm_uart_err_count       = 0U;
volatile uint8_t    g_mbm_state                = (uint8_t)MBM_ST_IDLE;
volatile uint8_t    g_mbm_job_index            = 0U;

volatile uint32_t   g_pm_reject_count          = 0U;
volatile uint32_t   g_st36_reject_count        = 0U;

volatile uint8_t    g_mbm_rx_dump[MBM_DUMP_LEN] = {0};
volatile uint8_t    g_mbm_rx_dump_len          = 0U;
volatile uint8_t    g_mbm_rx_dump_job          = 0U;

volatile uint16_t   g_mbm_timeout_ms           = 500U;
volatile uint16_t   g_mbm_gap_ms               = 200U;
volatile uint16_t   g_mbm_poll_ms              = 1000U;

/* State privat ------------------------------------------------------------- */
static UART_HandleTypeDef *s_huart = NULL;

static uint8_t  s_tx[MBM_ADU_MAX];
static uint8_t  s_rx[MBM_ADU_MAX];
static uint8_t  s_work[MBM_ADU_MAX];

static volatile uint16_t s_rx_len      = 0U;
static volatile uint8_t  s_frame_ready = 0U;
static volatile uint8_t  s_tx_busy     = 0U;

static uint32_t s_tick       = 0U;   /* penanda waktu state saat ini */
static uint32_t s_cycle_tick = 0U;   /* awal putaran polling         */

/* Daftar job --------------------------------------------------------------- */

typedef void (*MBM_Decoder_t)(void);

typedef struct
{
  const volatile uint8_t *addr;    /* alamat slave, ditunjuk agar bisa runtime */
  uint8_t                 fc;
  uint16_t                start;
  uint16_t                qty;
  volatile uint16_t      *dest;    /* penampung hasil, sepanjang qty word */
  MBM_Decoder_t           decode;  /* dipanggil setelah dest terisi       */

  /* Penghitung per device. Ditunjuk, bukan disalin, supaya nama variabel
     yang sudah dipantau di Live Expressions tidak perlu berubah saat
     device baru ditambahkan. */
  volatile uint8_t       *online;
  volatile uint8_t       *last_exception;
  volatile uint32_t      *ok_count;
  volatile uint32_t      *timeout_count;
  volatile uint32_t      *crc_err_count;
  volatile uint32_t      *exception_count;
  volatile uint32_t      *reject_count;
} MBM_Job_t;

static void PM_Decode(void);
static void ST36_Decode(void);

/**
  * Dua job, dijalankan bergilir.
  *
  * Power Meter : seluruh 68 input register dalam SATU permintaan (batas
  *   Modbus 125), jadi semua pengukuran tiba dari saat yang sama - lebih
  *   baik daripada memecahnya jadi beberapa permintaan yang isinya berasal
  *   dari waktu berbeda.
  * ST36 : 6 register berurutan 0x1029..0x102E, dibaca dengan FC 0x03 karena
  *   protokol ST36 memang hanya mengenal 0x03/0x06 - tidak ada FC 0x04 di
  *   dokumennya, jadi "input register" di sana pun diakses lewat 0x03.
  */
static const MBM_Job_t s_jobs[] =
{
  { &g_pm_addr,   0x04U, PM_IR_BASE,   PM_IR_COUNT,   g_pm_raw,   PM_Decode,
    &g_pm_online,   &g_pm_last_exception,   &g_pm_ok_count,
    &g_pm_timeout_count,   &g_pm_crc_err_count,   &g_pm_exception_count,
    &g_pm_reject_count },

  { &g_st36_addr, 0x03U, ST36_IR_BASE, ST36_IR_COUNT, g_st36_raw, ST36_Decode,
    &g_st36_online, &g_st36_last_exception, &g_st36_ok_count,
    &g_st36_timeout_count, &g_st36_crc_err_count, &g_st36_exception_count,
    &g_st36_reject_count }
};

#define MBM_JOB_COUNT   (sizeof(s_jobs) / sizeof(s_jobs[0]))

/* Helper ------------------------------------------------------------------- */

static inline void MBM_De(bool tx_mode)
{
  HAL_GPIO_WritePin(DO_RS485_1_DE_GPIO_Port, DO_RS485_1_DE_Pin,
                    tx_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void MBM_StartRx(void)
{
  s_frame_ready = 0U;
  s_rx_len      = 0U;

  (void)HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rx, (uint16_t)MBM_ADU_MAX);
}

/** Ambil int32 dari dua register berurutan, word tinggi di alamat rendah. */
static int32_t MBM_I32(const volatile uint16_t *r, uint16_t idx)
{
  return (int32_t)(((uint32_t)r[idx] << 16) | (uint32_t)r[idx + 1U]);
}

static uint32_t MBM_U32(const volatile uint16_t *r, uint16_t idx)
{
  return (((uint32_t)r[idx] << 16) | (uint32_t)r[idx + 1U]);
}

/* Dekoder Power Meter ------------------------------------------------------ */

/**
  * Seluruh besaran fisik meter dikirim sebagai int32 dalam MILLI-unit
  * (mA, mV, mW, mvar, mVA), word tinggi di alamat rendah. Tidak ada faktor
  * skala dan tidak ada float di kabel - pembagian 1000 dilakukan di sini.
  *
  * Perhatikan: membaca hanya SATU register dari sebuah pasangan itu sah
  * secara protokol tapi menghasilkan setengah angka, dan gagalnya diam-diam -
  * yang keluar bilangan wajar tapi salah. Karena itu blok dibaca utuh dan
  * penguraiannya selalu berpasangan.
  */
static void PM_Decode(void)
{
  uint8_t i;

  for (i = 0U; i < PM_PHASE_COUNT; i++)
  {
    uint16_t base = (uint16_t)(PM_IR_PHASE_BASE + ((uint16_t)i * 10U));
    uint16_t ebas = (uint16_t)(PM_IR_ENERGY_BASE + ((uint16_t)i * 6U));

    g_pm_phase[i].current      = (float)MBM_I32(g_pm_raw, base)             / 1000.0f;
    g_pm_phase[i].voltage      = (float)MBM_I32(g_pm_raw, (uint16_t)(base + 2U)) / 1000.0f;
    g_pm_phase[i].active_w     = (float)MBM_I32(g_pm_raw, (uint16_t)(base + 4U)) / 1000.0f;
    g_pm_phase[i].reactive_var = (float)MBM_I32(g_pm_raw, (uint16_t)(base + 6U)) / 1000.0f;
    g_pm_phase[i].apparent_va  = (float)MBM_I32(g_pm_raw, (uint16_t)(base + 8U)) / 1000.0f;

    /* Power factor int16 bertanda, skala x1000 */
    g_pm_phase[i].pf = (float)((int16_t)g_pm_raw[PM_IR_PF_BASE + i]) / 1000.0f;

    g_pm_energy_wh[i]   = MBM_I32(g_pm_raw, ebas);
    g_pm_energy_varh[i] = MBM_I32(g_pm_raw, (uint16_t)(ebas + 2U));
  }

  /* Frekuensi uint16 dalam centi-hertz: 5002 = 50.02 Hz. Satu register
     bersama untuk ketiga fase - sistem yang sah memang hanya punya satu
     frekuensi jala-jala. */
  g_pm_frequency = (float)g_pm_raw[PM_IR_FREQUENCY] / 100.0f;

  /* Konstanta meter efektif - yang BENAR-BENAR dipancarkan pin CF, bukan
     angka nameplate. Dipakai kalau nanti cacah pulsa CF diolah. */
  g_pm_meter_const = MBM_U32(g_pm_raw, PM_IR_METER_CONST);
}

/* Dekoder ST36 ------------------------------------------------------------- */

/**
  * Seluruh besaran ST36 adalah 16-bit satu register - tidak ada pasangan
  * 32-bit seperti di Power Meter. Indeks dihitung relatif terhadap
  * ST36_IR_BASE karena blok dibaca mulai dari 0x1029.
  *
  * Skala masih 1.0: dokumen protokol tidak menyebutkan satuan maupun faktor
  * untuk register-register ini. Nilai mentah yang dipakai sebagai acuan
  * sampai dikalibrasi terhadap tampilan panel ST36.
  */
static void ST36_Decode(void)
{
  g_st36_cur_u_raw = g_st36_raw[ST36_IR_CUR_U      - ST36_IR_BASE];
  g_st36_cur_v_raw = g_st36_raw[ST36_IR_CUR_V      - ST36_IR_BASE];
  g_st36_cur_w_raw = g_st36_raw[ST36_IR_CUR_W      - ST36_IR_BASE];
  g_st36_idc_raw   = g_st36_raw[ST36_IR_IF_CURRENT - ST36_IR_BASE];
  g_st36_vdc_raw   = g_st36_raw[ST36_IR_VF_VOLTAGE - ST36_IR_BASE];
  g_st36_fault     = g_st36_raw[ST36_IR_FAULT      - ST36_IR_BASE];

  g_st36_cur_u = (float)g_st36_cur_u_raw * g_st36_i_scale;
  g_st36_cur_v = (float)g_st36_cur_v_raw * g_st36_i_scale;
  g_st36_cur_w = (float)g_st36_cur_w_raw * g_st36_i_scale;
  g_st36_idc   = (float)g_st36_idc_raw   * g_st36_i_scale;
  g_st36_vdc   = (float)g_st36_vdc_raw   * g_st36_v_scale;
}

/* Transaksi ---------------------------------------------------------------- */

static void MBM_SendRequest(const MBM_Job_t *job)
{
  uint16_t crc;

  s_tx[0] = *job->addr;
  s_tx[1] = job->fc;
  s_tx[2] = (uint8_t)(job->start >> 8);
  s_tx[3] = (uint8_t)(job->start & 0xFFU);
  s_tx[4] = (uint8_t)(job->qty >> 8);
  s_tx[5] = (uint8_t)(job->qty & 0xFFU);

  crc     = MB_Crc16(s_tx, 6U);
  s_tx[6] = (uint8_t)(crc & 0xFFU);        /* CRC low dulu */
  s_tx[7] = (uint8_t)((crc >> 8) & 0xFFU);

  MBM_StartRx();   /* siap menerima SEBELUM kirim - jawaban bisa datang cepat */

  s_tx_busy = 1U;
  MBM_De(true);

  if (HAL_UART_Transmit_DMA(s_huart, s_tx, 8U) != HAL_OK)
  {
    MBM_De(false);
    s_tx_busy = 0U;
    return;
  }

  g_mbm_tx_count++;
}

/**
  * Periksa dan urai jawaban.
  * @return true bila jawaban sah dan sudah disalin ke job->dest.
  */
static bool MBM_HandleResponse(const MBM_Job_t *job, const uint8_t *buf, uint16_t len)
{
  uint16_t crc_calc;
  uint16_t crc_rx;
  uint8_t  i;

  if (len < 5U)
  {
    return false;   /* terlalu pendek untuk apa pun yang sah */
  }

  crc_calc = MB_Crc16(buf, (uint16_t)(len - 2U));
  crc_rx   = (uint16_t)(((uint16_t)buf[len - 1U] << 8) | buf[len - 2U]);

  if (crc_calc != crc_rx)
  {
    (*job->crc_err_count)++;
    return false;
  }

  if (buf[0] != *job->addr)
  {
    return false;   /* jawaban milik slave lain */
  }

  /* Jawaban exception: FC dikembalikan dengan bit 7 diset */
  if (buf[1] == (uint8_t)(job->fc | 0x80U))
  {
    *job->last_exception = buf[2];
    (*job->exception_count)++;
    return false;
  }

  if (buf[1] != job->fc)
  {
    return false;
  }

  /* FC 0x03/0x04: [addr][fc][byteCount][data...][crc] */
  if (buf[2] != (uint8_t)(job->qty * 2U))
  {
    return false;
  }

  if (len != (uint16_t)(5U + (job->qty * 2U)))
  {
    return false;
  }

  for (i = 0U; i < job->qty; i++)
  {
    job->dest[i] = (uint16_t)(((uint16_t)buf[3U + (i * 2U)] << 8) |
                               buf[3U + (i * 2U) + 1U]);
  }

  *job->last_exception = 0U;
  return true;
}

/* API ---------------------------------------------------------------------- */

void MBM_Init(UART_HandleTypeDef *huart)
{
  s_huart       = huart;
  s_frame_ready = 0U;
  s_tx_busy     = 0U;
  g_mbm_state   = (uint8_t)MBM_ST_IDLE;
  g_mbm_job_index = 0U;
  s_tick        = HAL_GetTick();
  s_cycle_tick  = HAL_GetTick();

  /* Sebagai master pun DE harus diam di mode terima selama tidak mengirim -
     kalau tidak, slave tidak akan pernah bisa menjawab. */
  MBM_De(false);

  MBM_StartRx();
}

void MBM_OnRxEvent(uint16_t size)
{
  s_rx_len      = size;
  s_frame_ready = 1U;
  g_mbm_rx_count++;
}

void MBM_OnTxComplete(void)
{
  /* Dipanggil setelah flag TC, jadi byte terakhir sudah keluar sepenuhnya.
     Baru sekarang aman mengembalikan transceiver ke mode terima - kalau
     diturunkan lebih awal, byte terakhir permintaan terpotong di kabel dan
     slave tidak akan pernah menjawab. */
  MBM_De(false);
  s_tx_busy = 0U;
}

void MBM_OnUartError(void)
{
  g_mbm_uart_err_count++;

  MBM_De(false);
  s_tx_busy = 0U;

  MBM_StartRx();
}

void MBM_Task(void)
{
  const MBM_Job_t *job;
  uint32_t         now;

  if (s_huart == NULL)
  {
    return;
  }

  now = HAL_GetTick();
  job = &s_jobs[g_mbm_job_index];

  switch ((MBM_State_t)g_mbm_state)
  {
    case MBM_ST_IDLE:
      /* Job pertama menunggu jadwal putaran; job berikutnya jalan langsung
         supaya satu putaran polling tidak terseret sepanjang jumlah device. */
      if (g_mbm_job_index != 0U)
      {
        MBM_SendRequest(job);
        s_tick      = now;
        g_mbm_state = (uint8_t)MBM_ST_WAIT;
      }
      else if ((uint32_t)(now - s_cycle_tick) >= (uint32_t)g_mbm_poll_ms)
      {
        s_cycle_tick = now;
        MBM_SendRequest(job);
        s_tick       = now;
        g_mbm_state  = (uint8_t)MBM_ST_WAIT;
      }
      else
      {
        /* belum waktunya */
      }
      break;

    case MBM_ST_WAIT:
      if (s_frame_ready != 0U)
      {
        uint16_t len = s_rx_len;

        if (len > MBM_ADU_MAX)
        {
          len = MBM_ADU_MAX;
        }

        (void)memcpy(s_work, s_rx, len);
        s_frame_ready = 0U;

        /* Rekam mentahnya sebelum divalidasi - justru frame yang DITOLAK
           yang paling perlu dilihat isinya saat mendiagnosa. */
        {
          uint8_t n = (len > (uint16_t)MBM_DUMP_LEN) ? (uint8_t)MBM_DUMP_LEN : (uint8_t)len;
          uint8_t k;

          for (k = 0U; k < n; k++)
          {
            g_mbm_rx_dump[k] = s_work[k];
          }

          g_mbm_rx_dump_len = n;
          g_mbm_rx_dump_job = g_mbm_job_index;
        }

        if (MBM_HandleResponse(job, s_work, len))
        {
          if (job->decode != NULL) { job->decode(); }
          *job->online = 1U;
          (*job->ok_count)++;
        }
        else
        {
          /* Ada yang menjawab, tapi jawabannya tidak sesuai. Dicatat
             terpisah dari timeout supaya "bus sunyi" dan "jawaban salah"
             tidak lagi terlihat sama dari luar. */
          (*job->reject_count)++;
          *job->online = 0U;
        }

        s_tick      = now;
        g_mbm_state = (uint8_t)MBM_ST_GAP;
      }
      else if ((uint32_t)(now - s_tick) >= (uint32_t)g_mbm_timeout_ms)
      {
        (*job->timeout_count)++;
        *job->online = 0U;

        /* Arm ulang penerimaan: kalau jawaban datang terlambat, sisa byte
           di jalur harus punya tempat mendarat - kalau tidak, frame
           berikutnya akan tercampur dengan sampah frame ini. */
        MBM_StartRx();

        s_tick      = now;
        g_mbm_state = (uint8_t)MBM_ST_GAP;
      }
      else
      {
        /* masih menunggu */
      }
      break;

    case MBM_ST_GAP:
      if ((uint32_t)(now - s_tick) >= (uint32_t)g_mbm_gap_ms)
      {
        g_mbm_job_index = (uint8_t)((g_mbm_job_index + 1U) % MBM_JOB_COUNT);
        g_mbm_state     = (uint8_t)MBM_ST_IDLE;
      }
      break;

    default:
      g_mbm_state = (uint8_t)MBM_ST_IDLE;
      break;
  }
}
