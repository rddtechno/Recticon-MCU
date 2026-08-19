/**
  ******************************************************************************
  * @file    mcp23s17.h
  * @brief   Driver MCP23S17 I/O Expander 16-bit (SPI) - Recticon
  *
  * Hardware:
  *   - Bus   : SPI3 (PC10 SCK / PC11 MISO / PC12 MOSI), Mode 0, 3.125 Mbit/s
  *   - Chip DIGITAL OUTPUT : CS = PA15, RST = PA12
  *   - Chip DIGITAL INPUT  : CS = PD2,  RST = PB3   (belum dipakai, menyusul)
  *
  * Pengalamatan:
  *   Tiap chip punya CS sendiri, jadi HAEN (hardware address enable) dibiarkan
  *   0. Dengan HAEN = 0 chip MENGABAIKAN bit alamat di opcode, sehingga opcode
  *   0x40 (write) / 0x41 (read) selalu valid berapa pun kondisi pin A2:A0 di
  *   board. Ini sengaja dipilih supaya driver tidak bergantung pada strapping
  *   pin alamat yang belum diverifikasi.
  *
  * Pemetaan bit untuk helper 16-bit generik (IOX_*Reg16):
  *   bit 0..7  -> GPA0..GPA7
  *   bit 8..15 -> GPB0..GPB7
  *
  * DIGITAL OUTPUT project ini hanya 8 kanal dan seluruhnya di PORT A
  * (GPA0..GPA7). Port B chip yang sama dipakai sebagai cadangan: tetap
  * dikonfigurasi output dan ditahan LOW supaya pinnya punya kondisi pasti,
  * tidak mengambang.
  ******************************************************************************
  */

#ifndef __MCP23S17_H
#define __MCP23S17_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Opcode SPI --------------------------------------------------------------- */
#define IOX_OP_WRITE(a)   ((uint8_t)(0x40U | (((uint8_t)(a) & 0x07U) << 1)))
#define IOX_OP_READ(a)    ((uint8_t)(0x41U | (((uint8_t)(a) & 0x07U) << 1)))

/* Register map - berlaku saat IOCON.BANK = 0 (default setelah reset) -------- */
#define IOX_REG_IODIRA    0x00U
#define IOX_REG_IODIRB    0x01U
#define IOX_REG_IPOLA     0x02U
#define IOX_REG_IPOLB     0x03U
#define IOX_REG_GPINTENA  0x04U
#define IOX_REG_GPINTENB  0x05U
#define IOX_REG_DEFVALA   0x06U
#define IOX_REG_DEFVALB   0x07U
#define IOX_REG_INTCONA   0x08U
#define IOX_REG_INTCONB   0x09U
#define IOX_REG_IOCON     0x0AU
#define IOX_REG_IOCON2    0x0BU   /* register yang sama, alamat bayangan */
#define IOX_REG_GPPUA     0x0CU
#define IOX_REG_GPPUB     0x0DU
#define IOX_REG_INTFA     0x0EU
#define IOX_REG_INTFB     0x0FU
#define IOX_REG_INTCAPA   0x10U
#define IOX_REG_INTCAPB   0x11U
#define IOX_REG_GPIOA     0x12U
#define IOX_REG_GPIOB     0x13U
#define IOX_REG_OLATA     0x14U
#define IOX_REG_OLATB     0x15U

/* Bit IOCON ---------------------------------------------------------------- */
#define IOX_IOCON_BANK    0x80U
#define IOX_IOCON_MIRROR  0x40U
#define IOX_IOCON_SEQOP   0x20U   /* 0 = alamat auto-increment (yang kita pakai) */
#define IOX_IOCON_DISSLW  0x10U
#define IOX_IOCON_HAEN    0x08U
#define IOX_IOCON_ODR     0x04U
#define IOX_IOCON_INTPOL  0x02U

#define IOX_SPI_TIMEOUT_MS 100U

/* Status ------------------------------------------------------------------- */
typedef enum
{
  IOX_OK           =  0,
  IOX_ERR_PARAM    = -1,
  IOX_ERR_NOT_INIT = -2,
  IOX_ERR_SPI      = -3,
  IOX_ERR_VERIFY   = -4,   /* tulis-baca-banding tidak cocok -> wiring/CS/mode */
  IOX_ERR_ABSENT   = -5    /* chip tidak menjawab sama sekali                  */
} IOX_Status_t;

/* Handle satu chip --------------------------------------------------------- */
typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef      *cs_port;
  uint16_t           cs_pin;
  GPIO_TypeDef      *rst_port;
  uint16_t           rst_pin;
  uint8_t            hw_addr;      /* A2:A0 - tak dipakai selama HAEN = 0 */
  uint8_t            ready;
} IOX_Dev_t;

/* ==========================================================================
 * API generik (dipakai chip DI maupun DO)
 * ========================================================================== */

/** @brief Isi handle. Belum menyentuh bus - panggil IOX_HardReset() setelahnya. */
void         IOX_Bind(IOX_Dev_t *dev,
                      SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port,  uint16_t cs_pin,
                      GPIO_TypeDef *rst_port, uint16_t rst_pin,
                      uint8_t hw_addr);

/**
  * @brief  Pulse pin RST (aktif LOW) lalu tulis IOCON ke kondisi dikenal.
  *         Reset hardware menjamin IOCON.BANK = 0, sehingga peta register di
  *         atas pasti valid walau chip sebelumnya sempat di-set BANK = 1.
  */
IOX_Status_t IOX_HardReset(IOX_Dev_t *dev);

IOX_Status_t IOX_WriteReg (IOX_Dev_t *dev, uint8_t reg, uint8_t val);
IOX_Status_t IOX_ReadReg  (IOX_Dev_t *dev, uint8_t reg, uint8_t *val);

/** @brief Tulis 2 byte berurutan (memanfaatkan auto-increment SEQOP = 0). */
IOX_Status_t IOX_WriteReg16(IOX_Dev_t *dev, uint8_t reg_a, uint16_t val);
IOX_Status_t IOX_ReadReg16 (IOX_Dev_t *dev, uint8_t reg_a, uint16_t *val);

/**
  * @brief  Uji komunikasi TANPA menyentuh pin output: tulis pola ke DEFVALA/B
  *         (register yang tak berpengaruh selama interrupt-on-change mati),
  *         baca balik, bandingkan. Pola diuji dua kali (0xAA55 lalu 0x55AA)
  *         supaya jalur MOSI yang stuck-high/low ikut ketahuan.
  */
IOX_Status_t IOX_SelfTest(IOX_Dev_t *dev);

/**
  * @brief  Set seluruh 16 pin sebagai OUTPUT dengan aman.
  *         Urutan sengaja OLAT dulu baru IODIR: saat power-up MCP23S17 ada di
  *         mode input (Hi-Z), jadi latch diisi 0 lebih dulu supaya pin tidak
  *         sempat menggerakkan beban dengan nilai acak saat arah pin dibalik.
  */
IOX_Status_t IOX_ConfigAllOutput(IOX_Dev_t *dev);

/**
  * @brief  Set seluruh 16 pin sebagai INPUT.
  * @param  pullup_mask  bit = 1 -> pull-up internal 100k aktif di pin itu
  *                      (bit0..7 = GPA0..7, bit8..15 = GPB0..7)
  */
IOX_Status_t IOX_ConfigAllInput(IOX_Dev_t *dev, uint16_t pullup_mask);

/**
  * @brief  Aktifkan interrupt-on-change pada pin yang dipilih.
  * @param  enable_mask  bit = 1 -> pin itu memicu INT saat nilainya berubah
  *
  * INTCON di-set 0 sehingga pembandingnya adalah nilai pin SEBELUMNYA
  * (bukan DEFVAL), jadi setiap perubahan level memicu interrupt.
  * INTA melayani port A, INTB melayani port B (IOCON.MIRROR = 0).
  */
IOX_Status_t IOX_ConfigIntOnChange(IOX_Dev_t *dev, uint16_t enable_mask);

/** @brief Tulis 16 bit ke OLATA/OLATB (bit0..7 = GPA, bit8..15 = GPB). */
IOX_Status_t IOX_SetOutputs(IOX_Dev_t *dev, uint16_t val);

/** @brief Baca kembali latch output (OLATA/OLATB) - nilai yang diperintahkan. */
IOX_Status_t IOX_GetOutputLatch(IOX_Dev_t *dev, uint16_t *val);

/** @brief Baca kondisi pin sebenarnya (GPIOA/GPIOB). */
IOX_Status_t IOX_ReadPins(IOX_Dev_t *dev, uint16_t *val);

/* ==========================================================================
 * Instance DIGITAL OUTPUT - 8 kanal di PORT A (CS = PA15, RST = PA12)
 * ========================================================================== */

#define DOUT_CH_COUNT   8U      /* GPA0..GPA7 */

/* Perintah untuk g_dout_cmd */
typedef enum
{
  DOUT_CMD_IDLE      = 0,
  DOUT_CMD_SELFTEST  = 1,  /* uji SPI lewat register scratch - output tidak bergerak */
  DOUT_CMD_READBACK  = 2,  /* refresh g_dout_olat_rb / g_dout_gpio_rb / g_dout_iocon_rb */
  DOUT_CMD_REINIT    = 3,  /* hard reset + konfigurasi ulang, semua kanal -> 0       */
  DOUT_CMD_ALL_OFF   = 4,  /* semua 8 kanal OFF                                     */
  DOUT_CMD_ALL_ON    = 5,  /* semua 8 kanal ON                                      */
  DOUT_CMD_WALK_TEST = 6   /* nyalakan GPA0..GPA7 bergantian, 300 ms per kanal       */
} DOUT_Cmd_t;

/* --- DIUBAH dari Live Expressions --------------------------------------- */

/**
  * KONTROL PER KANAL. Indeks = nomor pin di port A:
  *   g_do[0] -> GPA0 ... g_do[7] -> GPA7
  *
  * Isi 1 untuk ON, 0 untuk OFF. Menyalakan GPA1 cukup dengan
  * `g_do[1] = 1` - tidak perlu menghitung bitmask.
  *
  * DOUT_Task() menyusun kedelapan elemen jadi satu byte lalu menulisnya ke
  * OLATA, dan hanya menulis bila hasilnya berubah. Nilai selain 0/1
  * dinormalkan jadi 1 dan dipantulkan balik ke array supaya koreksinya
  * langsung terlihat di Live Expressions.
  */
extern volatile uint8_t g_do[DOUT_CH_COUNT];

extern volatile uint8_t g_dout_cmd;         /* isi dengan DOUT_Cmd_t */

/* --- DIBACA dari Live Expressions --------------------------------------- */
extern volatile uint8_t  g_dout_present;        /* 1 = self-test lulus              */
extern volatile int8_t   g_dout_last_result;    /* IOX_Status_t terakhir            */
extern volatile uint8_t  g_dout_olat;           /* byte terakhir yang DIKIRIM ke OLATA */
extern volatile uint8_t  g_dout_olat_rb;        /* OLATA hasil baca balik (perintah 2) */

/**
  * GPIOA - kondisi pin yang sebenarnya. Di-refresh OTOMATIS setiap kali
  * output berubah (lewat g_do[], DOUT_SetChannel/WriteByte, ALL_ON/OFF,
  * maupun tiap langkah walk test), jadi tidak perlu DOUT_CMD_READBACK untuk
  * memantaunya. Nilainya harus selalu sama dengan g_dout_olat; kalau beda,
  * ada pin yang ditahan beban atau penulisan tidak mendarat.
  */
extern volatile uint8_t  g_dout_gpio_rb;
extern volatile uint8_t  g_dout_iocon_rb;       /* IOCON hasil readback             */
extern volatile uint32_t g_dout_write_count;
extern volatile uint32_t g_dout_error_count;
extern volatile uint32_t g_dout_last_hal_error; /* HAL_SPI_GetError()               */

/**
  * @brief  Inisialisasi chip digital output: hard reset -> self-test ->
  *         seluruh pin jadi output dengan nilai 0.
  *         Panggil setelah MX_SPI3_Init() dan MX_GPIO_Init().
  */
IOX_Status_t DOUT_Init(SPI_HandleTypeDef *hspi);

/**
  * @brief  Set satu kanal (0..7) dan langsung tulis ke chip.
  *         Ikut memperbarui g_do[ch].
  */
IOX_Status_t DOUT_SetChannel(uint8_t ch, bool on);

/** @brief Baca status kanal terakhir yang diperintahkan (dari g_do[]). */
bool DOUT_GetChannel(uint8_t ch);

/**
  * @brief  Tulis kedelapan kanal sekaligus sebagai satu byte (bit0 = GPA0).
  *         Isi g_do[] disinkronkan agar tetap konsisten.
  */
IOX_Status_t DOUT_WriteByte(uint8_t val);

/**
  * @brief  Pantau perubahan g_do[] dan jalankan g_dout_cmd.
  *         Panggil dari while(1). Menulis ke chip hanya bila nilainya berubah.
  */
void DOUT_Task(void);

/* ==========================================================================
 * Instance DIGITAL INPUT - 16 kanal, PORT A + PORT B (CS = PD2, RST = PB3)
 * ========================================================================== */

#define DIN_CH_PER_PORT   8U      /* GPA0..GPA7 dan GPB0..GPB7 */

/**
  * Interval polling default (ms). Sejak INTA/INTB dipakai, polling ini
  * berfungsi sebagai JARING PENGAMAN, bukan mekanisme utama - lihat catatan
  * "INT nyangkut" di bawah. Bisa diubah runtime lewat g_di_poll_ms.
  */
#define DIN_POLL_MS_DEFAULT   200U

/* Perintah untuk g_di_cmd */
typedef enum
{
  DIN_CMD_IDLE     = 0,
  DIN_CMD_SELFTEST = 1,  /* uji SPI lewat register scratch                  */
  DIN_CMD_READ_NOW = 2,  /* paksa satu pembacaan seketika, tak menunggu tick */
  DIN_CMD_REINIT   = 3   /* hard reset + konfigurasi ulang sebagai input     */
} DIN_Cmd_t;

/* --- DIBACA dari Live Expressions --------------------------------------- */

/**
  * STATUS PER KANAL. Indeks = nomor pin di portnya masing-masing:
  *   g_di_a[0] -> GPA0 ... g_di_a[7] -> GPA7
  *   g_di_b[0] -> GPB0 ... g_di_b[7] -> GPB7
  *
  * Isi 0 atau 1, di-refresh tiap g_di_poll_ms. Nilainya sudah melewati
  * g_di_invert (lihat di bawah); untuk nilai register apa adanya pakai
  * g_di_word.
  */
extern volatile uint8_t  g_di_a[DIN_CH_PER_PORT];
extern volatile uint8_t  g_di_b[DIN_CH_PER_PORT];

/** Isi register GPIOA/GPIOB apa adanya - bit0..7 = GPA, bit8..15 = GPB. */
extern volatile uint16_t g_di_word;

extern volatile uint8_t  g_di_present;        /* 1 = self-test lulus             */
extern volatile int8_t   g_di_last_result;    /* IOX_Status_t terakhir           */
extern volatile uint8_t  g_di_iocon_rb;       /* IOCON hasil readback            */
extern volatile uint32_t g_di_read_count;     /* total pembacaan sukses          */
extern volatile uint32_t g_di_change_count;   /* berapa kali nilainya berubah    */
extern volatile uint32_t g_di_error_count;
extern volatile uint32_t g_di_last_hal_error; /* HAL_SPI_GetError()              */

/* --- DIUBAH dari Live Expressions --------------------------------------- */
extern volatile uint8_t  g_di_cmd;            /* isi dengan DIN_Cmd_t            */

/** Interval polling dalam ms. 0 = baca tiap putaran main loop. */
extern volatile uint16_t g_di_poll_ms;

/**
  * Mask pull-up internal 100k (bit0..7 = GPA, bit8..15 = GPB).
  * Default 0xFFFF = semua aktif, karena input kontak kering maupun keluaran
  * opto-coupler open-collector sama-sama BUTUH pull-up agar punya level
  * definitif saat tidak aktif. Perubahan nilai langsung diterapkan oleh
  * DIN_Task() tanpa rebuild - berguna untuk mengetes apakah board sudah
  * punya pull-up eksternal (matikan ke 0x0000; kalau pembacaan tetap wajar
  * berarti sudah ada pull-up di luar).
  */
extern volatile uint16_t g_di_pullup;

/**
  * Mask inversi logika, di-XOR ke g_di_word sebelum disebar ke array.
  * Default 0x0000 = array menampilkan level pin apa adanya (1 = HIGH).
  * Rangkaian DI umumnya aktif-LOW (kontak menarik pin ke GND), jadi bila
  * ternyata begitu, set 0xFFFF supaya g_di_a[]/g_di_b[] bernilai 1 saat
  * input benar-benar aktif. Inversi dilakukan di firmware, bukan lewat
  * register IPOL, supaya g_di_word tetap memperlihatkan nilai mentah.
  */
extern volatile uint16_t g_di_invert;

/* --- Interrupt INTA/INTB ------------------------------------------------- */

/**
  * Chip DI memicu interrupt lewat 2 pin terpisah (IOCON.MIRROR = 0):
  *   INTA -> PB4 (EXTI4)      melayani PORT A
  *   INTB -> PB5 (EXTI9_5)    melayani PORT B
  * Keduanya aktif LOW (IOCON.INTPOL = 0, push-pull) -> EXTI falling edge.
  *
  * ISR TIDAK menyentuh SPI - transaksi SPI blocking di dalam ISR akan
  * menahan interrupt lain, termasuk DRDY ADS131M04 yang jauh lebih kritis.
  * ISR hanya menaikkan penghitung dan mengangkat flag; pembacaan sebenarnya
  * dilakukan DIN_Task() di main loop.
  *
  * CATATAN PENTING - "INT nyangkut". MCP23S17 menahan INT tetap aktif sampai
  * GPIO atau INTCAP dibaca. Kalau sebuah perubahan terjadi tepat saat
  * pembacaan berlangsung, INT bisa tinggal LOW terus sementara EXTI kita
  * edge-triggered -> tidak ada tepi turun baru, interrupt seolah mati.
  * Karena itu DIN_Task() juga: (a) memeriksa LEVEL pin INTA/INTB tiap
  * putaran, dan (b) tetap menjalankan polling tiap g_di_poll_ms sebagai
  * jaring pengaman. Dua lapis ini membuat sistem selalu pulih sendiri.
  */
extern volatile uint32_t g_di_irq_count;    /* total ISR INTA + INTB          */
extern volatile uint32_t g_di_irqa_count;   /* ISR dari INTA (port A)         */
extern volatile uint32_t g_di_irqb_count;   /* ISR dari INTB (port B)         */

/** Level pin INT saat pembacaan terakhir: bit0 = INTA, bit1 = INTB.
    1 = HIGH (idle). Kalau bertahan 0 padahal input diam, INT sedang nyangkut. */
extern volatile uint8_t  g_di_int_level;

/** Mask GPINTEN - pin mana yang boleh memicu INT. Default 0xFFFF (semua).
    Perubahan langsung diterapkan DIN_Task() tanpa rebuild; set 0x0000 untuk
    mematikan interrupt dan kembali ke polling murni. */
extern volatile uint16_t g_di_irq_enable;

/**
  * @brief  Dipanggil dari HAL_GPIO_EXTI_Callback() saat INTA/INTB memicu.
  *         Aman dipanggil dari ISR - tidak melakukan I/O.
  * @param  gpio_pin  pin yang memicu (DI_IOEXP_INTA_Pin / DI_IOEXP_INTB_Pin);
  *                   pin lain diabaikan.
  */
void DIN_OnInterrupt(uint16_t gpio_pin);

/**
  * @brief  Inisialisasi chip digital input: hard reset -> self-test ->
  *         16 pin jadi input dengan pull-up sesuai g_di_pullup, lalu
  *         interrupt-on-change diaktifkan sesuai g_di_irq_enable.
  *         Panggil setelah MX_SPI3_Init() dan MX_GPIO_Init().
  */
IOX_Status_t DIN_Init(SPI_HandleTypeDef *hspi);

/** @brief Baca satu kanal port A (0..7) dari hasil polling terakhir. */
bool DIN_GetA(uint8_t ch);

/** @brief Baca satu kanal port B (0..7) dari hasil polling terakhir. */
bool DIN_GetB(uint8_t ch);

/** @brief Paksa satu pembacaan seketika (di luar jadwal polling). */
IOX_Status_t DIN_ReadNow(void);

/**
  * @brief  Polling input tiap g_di_poll_ms + eksekusi g_di_cmd.
  *         Panggil dari while(1).
  */
void DIN_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __MCP23S17_H */
