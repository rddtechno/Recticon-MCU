/**
  ******************************************************************************
  * @file    modbus_slave.h
  * @brief   Modbus RTU Slave - Recticon Rectifier Controller
  *
  * Instance yang dipakai project ini:
  *   HMI       : USART6, RS485, DE = PC8, 38400 8-N-1   <-- dibuat di sini
  *   Orange Pi : USART1, TTL, tanpa DE, 38400 8-N-1     <-- menyusul
  * Keduanya memakai inti yang sama; yang membedakan hanya handle UART, pin DE,
  * dan alamat slave-nya. Karena itu semua state disimpan di MB_Slave_t, bukan
  * di variabel global - instance kedua nanti cukup satu struct lagi.
  *
  * BATAS FRAME. Modbus RTU memisahkan frame dengan diam 3.5 karakter. Di sini
  * dipakai interrupt IDLE line UART yang memicu setelah diam 1 karakter
  * (~286 us @38400). Lebih cepat dari 3.5 karakter, artinya master yang
  * menyisipkan jeda di TENGAH frame bisa membuat frame terbelah. Konsekuensinya
  * bukan salah data: potongan itu akan gagal CRC dan tidak dijawab, master
  * mengulang. Kalau g_mb_hmi.crc_err_count merangkak naik padahal kabel bagus,
  * itu tanda pertama yang harus dicurigai - jalan keluarnya memakai TIM7
  * (masih bebas) sebagai pewaktu 3.5 karakter.
  *
  * ISR SENGAJA PENDEK. Callback RX hanya mencatat panjang frame dan menaikkan
  * bendera; CRC, penguraian, dan penyusunan jawaban dikerjakan MB_SlaveTask()
  * di main loop. Alasannya sama seperti driver lain di project ini - jangan
  * menahan interrupt yang lebih kritis (DRDY ADS131M04 pada 16 kSPS).
  ******************************************************************************
  */

#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/** ADU Modbus RTU maksimum = 256 byte (alamat + PDU 253 + CRC 2). */
#define MB_ADU_MAX          256U

/** Alamat broadcast - dieksekusi tapi TIDAK dijawab. */
#define MB_ADDR_BROADCAST   0U

/* Function code ------------------------------------------------------------ */
#define MB_FC_READ_COILS        0x01U
#define MB_FC_READ_DISCRETE     0x02U
#define MB_FC_READ_HOLDING      0x03U
#define MB_FC_READ_INPUT        0x04U
#define MB_FC_WRITE_COIL        0x05U
#define MB_FC_WRITE_REG         0x06U
#define MB_FC_WRITE_COILS       0x0FU
#define MB_FC_WRITE_REGS        0x10U

/* Exception code ----------------------------------------------------------- */
#define MB_EX_ILLEGAL_FUNCTION  0x01U
#define MB_EX_ILLEGAL_ADDRESS   0x02U
#define MB_EX_ILLEGAL_VALUE     0x03U
#define MB_EX_SLAVE_FAILURE     0x04U

/* ==========================================================================
 * PETA REGISTER
 *
 * PENTING: peta ini harus dicocokkan dengan project HMI-nya. Angka di bawah
 * adalah usulan awal yang masuk akal untuk rectifier controller, BUKAN
 * standar - ubah bebas selama HMI mengikuti. Semua alamat berbasis 0
 * (register 0 = alamat protokol 0x0000).
 * ========================================================================== */

/* --- Discrete Input (FC 02) - 16 kanal MCP23S17 DI, read only --- */
#define MB_DI_COUNT         16U
/*   0..7  -> GPA0..GPA7  (g_di_a[])
 *   8..15 -> GPB0..GPB7  (g_di_b[])                                        */

/* --- Coil (FC 01/05/0F) - 8 relay MCP23S17 DO, read/write --- */
#define MB_COIL_COUNT       8U
/*   0..7  -> GPA0..GPA7  (g_do[])                                          */

/* --- Jumlah register yang disediakan --- */
#define MB_IR_COUNT         300U   /* Input Register   (FC 04)        */
#define MB_HR_COUNT         300U   /* Holding Register (FC 03/06/10)  */

/**
  * Isi register disimpan di dua array di bawah, dan ITULAH sumber datanya
  * saat permintaan baca dilayani. Sebagian alamat di awal "hidup": nilainya
  * disegarkan dari variabel driver tepat sebelum permintaan baca diproses
  * (lihat MB_IR_MAPPED_END / MB_HR_MAPPED_END), sisanya penyimpanan biasa
  * yang bebas dipakai kode aplikasi maupun ditulis HMI.
  *
  * Penyegaran dilakukan SAAT ADA PERMINTAAN, bukan periodik - datanya selalu
  * segar tanpa membebani main loop saat bus sedang sepi.
  */
extern volatile uint16_t g_mb_input[MB_IR_COUNT];
extern volatile uint16_t g_mb_hold[MB_HR_COUNT];

/* --- Input Register (FC 04) - read only --- */
typedef enum
{
  MB_IR_RECT_VOLTAGE = 0,  /* Volt  x100, int16 */
  MB_IR_RECT_CURRENT = 1,  /* Amper x100, int16 */
  MB_IR_BATT_VOLTAGE = 2,  /* Volt  x100, int16 */
  MB_IR_BATT_CURRENT = 3,  /* Amper x100, int16 */
  MB_IR_LOAD_VOLTAGE = 4,  /* Volt  x100, int16 - dari ADS1115 */
  MB_IR_LOAD_CURRENT = 5,  /* Amper x100, int16 - dari ADS1115 */
  MB_IR_DI_WORD      = 6,  /* 16 kanal DI sebagai bitmap        */
  MB_IR_DO_WORD      = 7,  /* 8 kanal DO sebagai bitmap         */
  MB_IR_DEV_STATUS   = 8,  /* bitmap keberadaan device, lihat MB_DEVBIT_* */
  MB_IR_ADC_SPS      = 9,  /* laju sampel ADS131M04 terukur     */
  MB_IR_ADC_OVERRUN  = 10, /* 16 bit bawah g_adc_overrun_count  */
  MB_IR_UPTIME_S     = 11, /* detik sejak boot, berputar di 65535 */

  /** Alamat >= ini murni penyimpanan di g_mb_input[], tidak terhubung ke
      driver mana pun. Isi dari kode aplikasi sesuai kebutuhan. */
  MB_IR_MAPPED_END   = 12
} MB_InputReg_t;

/* Bit untuk MB_IR_DEV_STATUS */
#define MB_DEVBIT_EEPROM    0x0001U
#define MB_DEVBIT_ADS1115   0x0002U
#define MB_DEVBIT_MCP4725   0x0004U
#define MB_DEVBIT_IOEXP_DO  0x0008U
#define MB_DEVBIT_IOEXP_DI  0x0010U
#define MB_DEVBIT_ADS131M04 0x0020U
#define MB_DEVBIT_ADC_RUN   0x0040U

/* --- Holding Register (FC 03/06/10) - setelan, read/write --- */
typedef enum
{
  MB_HR_DAC_SETPOINT = 0,  /* 0..4095 -> g_dac_setpoint          */
  MB_HR_DO_WORD      = 1,  /* tulis 8 relay sekaligus sebagai bitmap */
  MB_HR_SLAVE_ADDR   = 2,  /* 1..247 - berlaku seketika          */

  /** Alamat >= ini murni penyimpanan di g_mb_hold[]: HMI boleh menulis
      nilai apa pun (0..65535) dan membacanya kembali, tanpa efek ke
      hardware. Siap dipakai saat parameter kontrol rectifier ditentukan. */
  MB_HR_MAPPED_END   = 3
} MB_HoldingReg_t;

/* ==========================================================================
 * Instance
 * ========================================================================== */

typedef struct
{
  UART_HandleTypeDef *huart;
  GPIO_TypeDef       *de_port;   /* NULL = tanpa transceiver (jalur TTL) */
  uint16_t            de_pin;

  volatile uint8_t    addr;      /* alamat slave, 1..247 */

  uint8_t             rx[MB_ADU_MAX];   /* diisi HAL      */
  uint8_t             work[MB_ADU_MAX]; /* salinan kerja  */
  uint8_t             tx[MB_ADU_MAX];

  volatile uint16_t   rx_len;
  volatile uint8_t    frame_ready;
  volatile uint8_t    tx_busy;

  /* --- penghitung untuk Live Expressions --- */
  volatile uint32_t   rx_frame_count;    /* frame utuh diterima            */
  volatile uint32_t   tx_frame_count;    /* jawaban terkirim               */
  volatile uint32_t   crc_err_count;     /* CRC tidak cocok                */
  volatile uint32_t   not_for_us_count;  /* alamat slave lain              */
  volatile uint32_t   exception_count;   /* jawaban exception dikirim      */
  volatile uint32_t   uart_err_count;    /* framing/overrun/noise          */
  volatile uint8_t    last_fc;           /* function code terakhir         */
  volatile uint8_t    last_exception;    /* exception terakhir, 0 = tidak ada */
} MB_Slave_t;

/** Instance HMI - USART6, RS485, DE = PC8. */
extern MB_Slave_t g_mb_hmi;

/** Instance Orange Pi Zero3 - USART1, TTL 3V3, TANPA transceiver. */
extern MB_Slave_t g_mb_opi;

/** Alamat slave HMI saat boot. Bisa diubah runtime lewat g_mb_hmi.addr
    atau holding register MB_HR_SLAVE_ADDR. */
#define MB_HMI_DEFAULT_ADDR   1U

/**
  * Alamat slave Orange Pi saat boot = 10.
  *
  * Secara teknis boleh sama dengan HMI karena jalurnya terpisah - USART1
  * point-to-point TTL hanya punya satu slave, jadi alamat di sini penanda
  * protokol, bukan pembeda di bus bersama. Tetap dibuat berbeda supaya
  * setiap port di board punya alamat unik dan tidak ada kerancuan saat
  * menelusuri masalah:
  *     HMI (USART6)        = 1
  *     Orange Pi (USART1)  = 10
  *     ST36 (USART2)       = 5    <- device yang kita panggil
  *     Power Meter (USART2)= 9    <- device yang kita panggil
  */
#define MB_OPI_DEFAULT_ADDR   10U

/**
  * CATATAN - kedua slave berbagi g_mb_input[] dan g_mb_hold[] yang sama.
  * Ini disengaja: HMI dan Orange Pi harus melihat angka yang sama persis,
  * bukan salinan yang bisa menyimpang. Konsekuensinya penulisan dari satu
  * sisi langsung terlihat di sisi lain - penulis terakhir yang menang.
  *
  * Pengecualian MB_HR_SLAVE_ADDR: register itu selalu memantulkan alamat
  * instance yang SEDANG menjawab, dan penulisan ke sana hanya mengubah
  * alamat instance itu saja - bukan keduanya.
  */

/* API ---------------------------------------------------------------------- */

/**
  * @brief  Siapkan satu instance slave dan mulai menerima.
  * @param  de_port NULL bila jalur TTL tanpa transceiver.
  */
void MB_SlaveInit(MB_Slave_t *mb,
                  UART_HandleTypeDef *huart,
                  GPIO_TypeDef *de_port, uint16_t de_pin,
                  uint8_t addr);

/** @brief Proses frame yang sudah lengkap + kirim jawaban. Panggil dari while(1). */
void MB_SlaveTask(MB_Slave_t *mb);

/** @brief Dipanggil dari HAL_UARTEx_RxEventCallback(). Konteks ISR. */
void MB_SlaveOnRxEvent(MB_Slave_t *mb, uint16_t size);

/** @brief Dipanggil dari HAL_UART_TxCpltCallback(). Konteks ISR. */
void MB_SlaveOnTxComplete(MB_Slave_t *mb);

/** @brief Dipanggil dari HAL_UART_ErrorCallback(). Konteks ISR. */
void MB_SlaveOnUartError(MB_Slave_t *mb);

/** @brief Hitung CRC16 Modbus (polinomial 0xA001, awal 0xFFFF). */
uint16_t MB_Crc16(const uint8_t *buf, uint16_t len);

/* Pintasan per instance ---------------------------------------------------- */
void MB_HmiInit(UART_HandleTypeDef *huart);
void MB_HmiTask(void);

void MB_OpiInit(UART_HandleTypeDef *huart);
void MB_OpiTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_SLAVE_H */
