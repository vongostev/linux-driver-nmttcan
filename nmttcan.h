#ifndef __NMTTCAN_H
#define __NMTTCAN_H

#define NM_TTCAN_VENDOR_ID          0xA203
#define NM_TTCAN_DEVICE_ID          0x9471
#define NM_TTCAN_REVISION_ID_FW_01  0x02
#define NM_TTCAN_REVISION_ID_FW_02  0x12
#define NM_TTCAN_REVISION_ID_FW_03  0x22
#define NM_TTCAN_REVISION_ID        NM_TTCAN_REVISION_ID_FW_02

#define TTCAN_DMA_SLOT_SIZE         64
#define TTCAN_DMA_SIZE              (16 * 1024 * TTCAN_DMA_SLOT_SIZE)
#define TTCAN_CH_DMA_SIZE           (TTCAN_DMA_SIZE / 2)

#define MAJ_TTCAN 'c' // for dynamic IOCTL

//Standart functions
#define IOCTL_WR_MAINREG_TTCAN                  _IOWR(MAJ_TTCAN, 0, SADDR_DATA_MAIN_TTCAN) // write to reg
#define IOCTL_RD_MAINREG_TTCAN                  _IOWR(MAJ_TTCAN, 1, SADDR_DATA_MAIN_TTCAN) // read from reg
#define IOCTL_WR_CANREG_TTCAN                   _IOWR(MAJ_TTCAN, 2, SADDR_DATA_TTCAN) // write to reg
#define IOCTL_RD_CANREG_TTCAN                   _IOWR(MAJ_TTCAN, 3, SADDR_DATA_TTCAN) // read from reg
#define IOCTL_MODIFY_CANREG_TTCAN               _IOW(MAJ_TTCAN, 43, SADDR_DATA_MODIFY_TTCAN) // modify reg
#define IOCTL_WR_CANREGS_TTCAN                  _IOW(MAJ_TTCAN,  33, SADDR_DATAS_TTCAN) // write to regs
#define IOCTL_RD_CANREGS_TTCAN                  _IOWR(MAJ_TTCAN, 34, SADDR_DATAS_TTCAN) // read from regs
#define IOCTL_VERSION_TTCAN                     _IOWR(MAJ_TTCAN, 4, VERSION_TTCAN) // get info about device
#define IOCTL_VERSION_DRIVER_TTCAN              _IOWR(MAJ_TTCAN, 32, uint32_t) // getting information about driver

//Configuration functions
#define IOCTL_ENABLE_DMA_TTCAN                  _IO(MAJ_TTCAN, 5) // enable dma
#define IOCTL_DISABLE_DMA_TTCAN                 _IO(MAJ_TTCAN, 6) // disable dma
#define IOCTL_RESET_TG_TTCAN                    _IOW(MAJ_TTCAN, 10, CH_BUF_TTCAN)
#define IOCTL_SET_MODE_TTCAN                    _IOW(MAJ_TTCAN, 11, CONF_TTCAN) // set work mode
#define IOCTL_GET_MODE_TTCAN                    _IOWR(MAJ_TTCAN, 12, CONF_TTCAN) // get work mode
#define IOCTL_SET_ONESHOT_MODE_TTCAN            _IOW(MAJ_TTCAN, 49, CONF_TTCAN) // set/reset One Shot Mode
#define IOCTL_SET_SPEED_TTCAN                   _IOW(MAJ_TTCAN, 13, SPEED_TTCAN) // set speed
#define IOCTL_SET_SPEED_PARAMS_TTCAN            _IOW(MAJ_TTCAN, 55, SPEED_PARAMS_TTCAN) // set speed by parameters
#define IOCTL_GET_ERRORS_TTCAN                  _IOWR(MAJ_TTCAN, 56, ERRORS_TTCAN) // get errors on CANn

#define IOCTL_SET_MASKS_TTCAN                   _IOW(MAJ_TTCAN, 14, MASKS_TTCAN) // set masks for receive messages
#define IOCTL_SET_CANn_TIMER_TRSH_TTCAN         _IOW(MAJ_TTCAN, 15, TIMER_TRSH_TTCAN) // Set CANn_TIMER_TRSH register
#define IOCTL_SET_CANn_TIMER_CEED_TTCAN         _IOW(MAJ_TTCAN, 37, TIMER_TRSH_TTCAN) // Set CANn_TIMER_CEED register
#define IOCTL_SET_CANn_TIMER_FREE_TTCAN         _IOW(MAJ_TTCAN, 38, TIMER_TRSH_TTCAN) // Set free CANn_TIMER
#define IOCTL_SET_CANn_TIMER_RST_ON_RXB_TTCAN   _IOW(MAJ_TTCAN, 39, TIMER_TRSH_TTCAN) // Set CANn_TIMER RST_ON_RXBn
#define IOCTL_STOP_CANn_TIMER_TTCAN             _IOW(MAJ_TTCAN, 40, uint32_t) // Stop CANn_TIMER
#define IOCTL_GET_CANn_TIMER_TTCAN              _IOWR(MAJ_TTCAN, 45, TIMER_TRSH_TTCAN) // Read CANn current timer & epoch registers
#define IOCTL_START_CANn_TIMER_INT_TTCAN        _IOW(MAJ_TTCAN, 50, TIMER_TRSH_TTCAN) // Start CANn timer interupt
#define IOCTL_STOP_CANn_TIMER_INT_TTCAN         _IOW(MAJ_TTCAN, 51, uint32_t) // Stop CANn timer interupt
#define IOCTL_WAIT_CANn_TIMER_INT_TTCAN         _IOW(MAJ_TTCAN, 52, TIMER_TRSH_TTCAN) // Wait CANn timer interupt
#define IOCTL_SET_CANn_TIMEOUTS_TTCAN           _IOW(MAJ_TTCAN, 53, TIMEOUTS_TTCAN) // set CANn absolute&interval timeouts
#define IOCTL_RESET_TTCAN                       _IO(MAJ_TTCAN, 41) // Software reset device
#define IOCTL_RESET_CANn_TTCAN                  _IOW(MAJ_TTCAN, 42, uint32_t) // Hardware reset CANn

//Receive data functions
#define IOCTL_RD_CH_RAW_DMA_TTCAN               _IOWR(MAJ_TTCAN, 18, DMA_STR_TTCAN) // read DMA for channel
#define IOCTL_READ_DMA_BLOCKS_TTCAN             _IOWR(MAJ_TTCAN, 66, DMA_READ_BLOCK)

//Transmit data functions
#define IOCTL_WRITE_DATA_TO_FIFO_TTCAN1_V2      _IOW(MAJ_TTCAN, 69, CAN_WRT_MSG_V2)
#define IOCTL_WRITE_DATA_TO_FIFO_TTCAN2_V2      _IOW(MAJ_TTCAN, 70, CAN_WRT_MSG_V2)
#define IOCTL_WRITE_DATA_TO_FIFO_TTCAN1         _IOW(MAJ_TTCAN, 67, CAN_WRT_MSG)
#define IOCTL_WRITE_DATA_TO_FIFO_TTCAN2         _IOW(MAJ_TTCAN, 68, CAN_WRT_MSG)
#define IOCTL_WRITE_DATA_TO_TR_BUF_TTCAN        _IOW(MAJ_TTCAN, 19, SEND_DATA) // write data to transmit buffer
#define IOCTL_SEND_DATA_TTCAN                   _IOWR(MAJ_TTCAN, 35, SEND_DATA) // send data by priority
#define IOCTL_SEND_DATA_NOW_TTCAN               _IOW(MAJ_TTCAN, 20, SEND_DATA_NOW) // send data by priority
#define IOCTL_CHECK_TRANSMIT_TTCAN              _IOWR(MAJ_TTCAN, 21, SEND_DATA_NOW) // check data transmit
#define IOCTL_WAIT_TRANSMIT_TTCAN               _IOWR(MAJ_TTCAN, 44, SEND_DATA_NOW) // wait data transmit
#define IOCTL_END_TRANSMIT_TTCAN                _IOW(MAJ_TTCAN, 36, SEND_DATA_NOW) // abort transmit request
#define IOCTL_SEND_DATA_TG_TTCAN                _IOW(MAJ_TTCAN, 22, SEND_DATA_TG) // send data by trigger
#define IOCTL_SEND_DATA_LOOP_TTCAN              _IOW(MAJ_TTCAN, 23, SEND_DATA_TG) // auto send data by trigger
#define IOCTL_CHECK_TG_TTCAN                    _IOW(MAJ_TTCAN, 24, CH_BUF_TTCAN) // check data transmit
#define IOCTL_ABAT_TTCAN                        _IOW(MAJ_TTCAN, 48, CONF_TTCAN) // request/terminate request to abort all pending transmissions

#define IOCTL_TEST_WR_MAINREG_TTCAN             _IOW(MAJ_TTCAN, 46, SADDR_DATA_MAIN_TTCAN) // write 10 mln times to register
#define IOCTL_TEST_RD_MAINREG_TTCAN             _IOWR(MAJ_TTCAN, 47, SADDR_DATA_MAIN_TTCAN) // read 10 mln times from register

//OFFSETS
#define OFFSET_DMA_DATA_BASE_LO_REG_mPCIe_CAN   0x1000 // low part phys address DMA
#define OFFSET_DMA_DATA_BASE_HI_REG_mPCIe_CAN   0x1004 // hi part phys address DMA
#define OFFSET_CAN1_CTRL_REG_PCI_mPCIe_CAN      0x1040 // control reg for channel 0
#define OFFSET_CAN2_CTRL_REG_PCI_mPCIe_CAN      0x1050 // control reg for channel 1
#define OFFSET_CAN1_ACS_REG_mPCIe_CAN           0x1044 // command reg for channel 0
#define OFFSET_CAN2_ACS_REG_mPCIe_CAN           0x1054 // command reg for channel 1
#define OFFSET_INTERRUPT_REG_TTCAN              0x100C // interrupt reg
#define OFFSET_INTERRUPT_MASK_REG_TTCAN         0x1010 // mask reg for interrupt
#define OFFSET_DMA_INDEX_REG_TTCAN              0x1008 // write pointer DMA
#define OFFSET_DMA1_INDEX_REG_TTCAN             OFFSET_DMA_INDEX_REG_TTCAN // write pointer DMA1
#define OFFSET_DMA2_INDEX_REG_TTCAN             0x1030 // write pointer DMA2
#define OFFSET_DMA_RD_INDEX_REG_TTCAN           0x10A8 // read pointer DMA
#define OFFSET_DMA1_RD_INDEX_REG_TTCAN          OFFSET_DMA_RD_INDEX_REG_TTCAN // read pointer DMA1
#define OFFSET_DMA2_RD_INDEX_REG_TTCAN          0x10AC // read pointer DMA2
#define OFFSET_CAN1_TIMER                       0x1060
#define OFFSET_CAN2_TIMER                       0x1080
#define OFFSET_CAN1_TIMER_EPOCH                 0x10B0
#define OFFSET_CAN2_TIMER_EPOCH                 0x10B8
#define OFFSET_CAN1_TIMER_CEED                  0x1068
#define OFFSET_CAN2_TIMER_CEED                  0x1088
#define OFFSET_CAN1_EPOCH_CEED                  0x10B4
#define OFFSET_CAN2_EPOCH_CEED                  0x10BC
#define OFFSET_CAN1_TIMER_TRSH                  0x1064
#define OFFSET_CAN2_TIMER_TRSH                  0x1084
#define OFFSET_CAN1_TIMER_CTRL                  0x106C
#define OFFSET_CAN2_TIMER_CTRL                  0x108C
#define OFFSET_CAN1_TX1_TRIG                    0x1070
#define OFFSET_CAN1_TX2_TRIG                    0x1074
#define OFFSET_CAN1_TX3_TRIG                    0x1078
#define OFFSET_CAN1_TX1_TRIG_EPOCH              0x10C0
#define OFFSET_CAN1_TX2_TRIG_EPOCH              0x10C4
#define OFFSET_CAN1_TX3_TRIG_EPOCH              0x10C8
#define OFFSET_CAN2_TX1_TRIG                    0x1090
#define OFFSET_CAN2_TX2_TRIG                    0x1094
#define OFFSET_CAN2_TX3_TRIG                    0x1098
#define OFFSET_CAN2_TX1_TRIG_EPOCH              0x10D0
#define OFFSET_CAN2_TX2_TRIG_EPOCH              0x10D4
#define OFFSET_CAN2_TX3_TRIG_EPOCH              0x10D8
#define OFFSET_CAN1_INT_TRIG                    0x10A0
#define OFFSET_CAN2_INT_TRIG                    0x10A4
#define OFFSET_CAN1_INT_TRIG_EPOCH              0x10CC
#define OFFSET_CAN2_INT_TRIG_EPOCH              0x10DC
#define OFFSET_CAN1_TRIG_CTRL                   0x107C
#define OFFSET_CAN2_TRIG_CTRL                   0x109C
#define OFFSET_CAN1_INT_TIMEMARK                0x2080
#define OFFSET_CAN2_INT_TIMEMARK                0x20C0
#define OFFSET_CAN1_DATA_BUF_mPCIe_CAN          0x2000 // buf channel 0
#define OFFSET_CAN2_DATA_BUF_mPCIe_CAN          0x2010 // buf channel 1
#define OFFSET_CAN1_TIMEOUT_ABSOLUTE            0x1028
#define OFFSET_CAN2_TIMEOUT_ABSOLUTE            0x102C
#define OFFSET_CAN1_TIMEOUT_INTERVAL            0x1038
#define OFFSET_CAN2_TIMEOUT_INTERVAL            0x103C
#define OFFSET_CAN1_MIRR_ERR                    0x1048
#define OFFSET_CAN2_MIRR_ERR                    0x1058
#define OFFSET_CAN1_MIRR_TXB                    0x104C
#define OFFSET_CAN2_MIRR_TXB                    0x105C

// bits identify IRQ
#define INT_HDAT_TTCAN                          0x00000001 // 1/2 buffer DMA
#define INT_QDAT_TTCAN                          0x00000002 // 1/8 buffer DMA
#define INT_CAN1_TTCAN                          0x00000010 // channel 1 interrupt
#define INT_CAN2_TTCAN                          0x00000020 // channel 2 interrupt
#define INT_TIM_CAN1_TTCAN                      0x00000040 // CAN 1 timer interrupt
#define INT_TIM_CAN2_TTCAN                      0x00000080 // CAN 2 timer interrupt
#define INT_CAN_ACS1_TTCAN                      0x00000100 // CAN ACS 1 interrupt
#define INT_CAN_ACS2_TTCAN                      0x00000200 // CAN ACS 2 interrupt
#define INT_MSG_RCVD1_TTCAN                     0x00000400 // CAN 1 message received interrupt
#define INT_MSG_RCVD2_TTCAN                     0x00000800 // CAN 2 message received interrupt
#define INT_TIM_ABS1_TTCAN                      0x00001000 // CAN 1 absolute timer interrupt
#define INT_TIM_ABS2_TTCAN                      0x00002000 // CAN 2 absolute timer interrupt
#define INT_TIM_ITV1_TTCAN                      0x00004000 // CAN 1 interval timer interrupt
#define INT_TIM_ITV2_TTCAN                      0x00008000 // CAN 2 interval timer interrupt
#define INT_MSG_XMTD10_TTCAN                    0x00010000 // CAN 1 transmit buffer 0 empty interrupt
#define INT_MSG_XMTD11_TTCAN                    0x00020000 // CAN 1 transmit buffer 1 empty interrupt
#define INT_MSG_XMTD12_TTCAN                    0x00040000 // CAN 1 transmit buffer 2 empty interrupt
#define INT_ERR_CAN1_TTCAN                      0x00080000 // CAN 1 bus error interrupt
#define INT_WAKE_CAN1_TTCAN                     0x00100000 // CAN 1 wakeup interrupt
#define INT_MERR_CAN1_TTCAN                     0x00200000 // CAN 1 message error interrupt
#define INT_MSG_XMTD20_TTCAN                    0x00400000 // CAN 2 transmit buffer 0 empty interrupt
#define INT_MSG_XMTD21_TTCAN                    0x00800000 // CAN 2 transmit buffer 1 empty interrupt
#define INT_MSG_XMTD22_TTCAN                    0x01000000 // CAN 2 transmit buffer 2 empty interrupt
#define INT_ERR_CAN2_TTCAN                      0x02000000 // CAN 2 bus error interrupt
#define INT_WAKE_CAN2_TTCAN                     0x04000000 // CAN 2 wakeup interrupt
#define INT_MERR_CAN2_TTCAN                     0x08000000 // CAN 2 message error interrupt

///
/// \remark ЧТЕНИЕ ДАННЫХ из DMA БУФЕРА
/// IOCTL_READ_DMA_BLOCKS_TTCAN
///

/// \brief максимальное кол-во блоков дма в контейнере
#define DMA_COUNT_BLOCKS 64
/// \brief размер одного блока дма в байтах
#define DMA_RAW_BLOCK_SIZE_64 64
/// \brief мегабайт в байтах (размер кольцевого буфера)
#define MEGABYTE                            1024*1024
/// \brief размер кольцевого буфера канала в байтах
#define SIZE_DMA_CHANNEL_BUFFER             MEGABYTE / 2
/// \brief кол-во сообщений дма в кольцевого буфера канала
#define NUMBER_MSG_IN_DMA_CHANNEL_BUFFER    8192

#pragma pack(push, 1)
typedef struct {
    uint64_t prevWrt;
    uint64_t wrt;
    uint64_t prevRd;
    uint64_t rd;
    uint64_t dmaIndex;
} STAT_INFO;

/// \brief один дма блок 64 байта
typedef struct {
    unsigned char data[DMA_RAW_BLOCK_SIZE_64];  ///< содержимое блока дма
} DMA_RAW_BLOCK_64;

/// \brief контейнер считываемых блоков дма
typedef struct {
    int numChannel;                             ///< номер канала (1 или 2)
    unsigned int countBlocks;                   ///< фактическое количество 64 байтных блоков дма
    DMA_RAW_BLOCK_64 blocks[DMA_COUNT_BLOCKS];  ///< блоки дма
    STAT_INFO info;
} DMA_READ_BLOCK;
#pragma pack(pop)

/// /// /// /// ///

///
/// \remark ОТПРАВКА ДАННЫХ в РЕЖИМЕ FIFO
/// IOCTL_WRITE_DATA_TO_FIFO_TTCAN

#define REG_TTCAN1_BUF                0x2000                        ///< регистр для отправки сообщений в FIFO TTCAN1
#define REG_TTCAN2_BUF                0x2010                        ///< регистр для отправки сообщений в FIFO TTCAN2
#define REG_TTCAN1_FIFO_CONSTAT       0x10F0                        ///< регистр управления и статуса FIFO TTCAN1
#define REG_TTCAN2_FIFO_CONSTAT       0x10F4                        ///< регистр управления и статуса FIFO TTCAN2

#define REG_TTCAN1_HP_FIFO            0x2020                        ///< регистр для отправки сообщений в HP FIFO TTCAN1
#define REG_TTCAN2_HP_FIFO            0x2030                        ///< регистр для отправки сообщений в HP FIFO TTCAN2
#define REG_TTCAN1_HP_FIFO_CONSTAT    0x10F8                        ///< регистр управления и статуса HP FIFO TTCAN1
#define REG_TTCAN2_HP_FIFO_CONSTAT    0x10FC                        ///< регистр управления и статуса HP FIFO TTCAN2

#define CAN_WRT_MSG_EXT_MSG_NO      0                               ///< mIsExtMsg - не расширенное
#define CAN_WRT_MSG_EXT_MSG         ~CAN_WRT_MSG_EXT_MSG_NO         ///< mIsExtMsg - расширенное

#define CAN_WRT_MSG_DATA_LENGTH     8                               ///< максимальный размер буфера данных

#define CAN_WRT_MSG_TYPE_FIFO       0                               ///< mTypeFifo - обычное FIFO
#define CAN_WRT_MSG_TYPE_FIFO_HP    1                               ///< mTypeFifo - высокоприоритетное FIFO

#pragma pack(push, 1)
/// \brief блок сообщения для записи в FIFO
typedef struct {
    unsigned char   mNumChannel;                        ///< номер канала (1 или 2) - DEPRECATED - NOT USE
    unsigned char   mTypeFifo;                          ///< тип FIFO
    unsigned char   mIsExtMsg;                          ///< признак расширенного сообщения
    unsigned int    mSID;                               ///< SID
    unsigned int    mEID;                               ///< EID
    unsigned char   mDataLength;                        ///< фактический размер данных
    unsigned char   mData[CAN_WRT_MSG_DATA_LENGTH];     ///< данные пакета
    unsigned char   mMsgId;                             ///< уникальный идентификатор сообщения
} CAN_WRT_MSG;
/// \brief блок сообщения для записи в FIFO (обновлённое с возможностью упрвления RTR)
typedef struct {
    unsigned char   mRtrBit;                            ///< наличие RTR бита
    unsigned char   mTypeFifo;                          ///< тип FIFO
    unsigned char   mIsExtMsg;                          ///< признак расширенного сообщения
    unsigned int    mSID;                               ///< SID
    unsigned int    mEID;                               ///< EID
    unsigned char   mDataLength;                        ///< фактический размер данных
    unsigned char   mData[CAN_WRT_MSG_DATA_LENGTH];     ///< данные пакета
    unsigned char   mMsgId;                             ///< уникальный идентификатор сообщения
} CAN_WRT_MSG_V2;
#pragma pack(pop)

/// /// /// /// ///

// speeds
typedef enum _WORK_SPEED
{
  WORK_SPEED_125 = 125,
  WORK_SPEED_250 = 250,
  WORK_SPEED_500 = 500,
  WORK_SPEED_1000 = 1000
} WORD_SPEED;

#pragma pack(push, 1)
typedef struct _MSG_HDR_TTCAN
{
  unsigned RXBn       : 1;
  unsigned CANn       : 1;
  unsigned tmr_div    : 14;
  unsigned tmr_ntu    : 16;
} MSG_HDR_TTCAN;

typedef struct _DMA_SLOT_TTCAN
{
  MSG_HDR_TTCAN hdr;
  uint8_t RXBnCTRL;
  uint8_t RXBnSIDH;
  uint8_t RXBnSIDL;
  uint8_t RXBnEID8;
  uint8_t RXBnEID0;
  uint8_t RXBnDLC;
  uint8_t RXBnD[8];
  unsigned : 8;
  uint8_t epoch;
} DMA_SLOT_TTCAN;

typedef union _TXBCTRL
{
  struct
  {
    unsigned TXP    : 2;
    unsigned        : 1;
    unsigned TXREQ  : 1;
    unsigned TXERR  : 1;
    unsigned MLOA   : 1;
    unsigned ABTF   : 1;
    unsigned        : 1;
  } bits;
  uint8_t byte;
} TXBCTRL;
#pragma pack(pop)

typedef struct _SADDR_DATA_TTCAN
{
  uint8_t  daddr;    // u8 0x...
  uint8_t  data;     // 8bit data
  uint32_t channel;  // channel
} SADDR_DATA_TTCAN;

typedef struct _SADDR_DATA_MODIFY_TTCAN
{
  uint8_t  daddr;    // u8 0x...
  uint16_t data;    // low byte: mask; high byte: new value to set
  uint32_t channel;  // channel
} SADDR_DATA_MODIFY_TTCAN;

typedef struct _SADDR_DATAS_TTCAN
{
  uint8_t channel;
  uint8_t addr;
  uint8_t nsize;
  unsigned : 8;
  uint8_t data[16];
} SADDR_DATAS_TTCAN;

typedef struct _SADDR_DATA_MAIN_TTCAN
{
  uint32_t daddr;   // u32 0x...
  uint32_t data;    // u32 data
  uint32_t channel; // channel
} SADDR_DATA_MAIN_TTCAN;

typedef struct _TIMER_TRSH_TTCAN
{
  uint8_t  nCh;
  uint8_t  nEpochBits;
  uint8_t  bEpoch;
  uint32_t nEpoch;
  uint32_t nValue;
} TIMER_TRSH_TTCAN;

typedef struct _CH_BUF_TTCAN
{
  uint8_t nCh;
  uint8_t nBuf;
} CH_BUF_TTCAN;

typedef struct _DMA_STR_TTCAN
{
  uint32_t       number_block;   // number blocks
  uint32_t       number_channel; // number channel
  uint32_t       timeout;        // max time to wait for data
  DMA_SLOT_TTCAN buf[400];       // data buffer
} DMA_STR_TTCAN;

typedef struct _SINTHIOSA_TTCAN
{
  uint8_t intr;     // TX01E...
  int32_t proc_pid; // proc pid
  int32_t sig_no;   // signal
} SINTHIOSA_TTCAN;

typedef struct _CONF_TTCAN
{
  uint32_t channel;  // channel
  uint8_t  mode;     // work mode
} CONF_TTCAN;

typedef struct _SPEED_TTCAN
{
  uint32_t channel; // channel
  uint32_t speed;   // speed
} SPEED_TTCAN;

typedef struct _SPEED_PARAMS_TTCAN
{
  uint32_t channel; // channel
  uint8_t brp, sjw, sam, btlmode, phseg1, phseg2, prseg, wakfil;
} SPEED_PARAMS_TTCAN;

typedef struct _TIMEOUTS_TTCAN
{
  uint32_t channel;
  uint32_t interval;  // usec
  uint32_t absolute;  // usec
} TIMEOUTS_TTCAN;

typedef struct _ERRORS_TTCAN
{
  uint8_t channel;
  uint8_t nEFLG;
  uint8_t nTEC;
  uint8_t nREC;
} ERRORS_TTCAN;

typedef struct _SEND_DATA
{
  uint8_t  nChannel;
  uint8_t  nBufNumber;
  uint8_t  nPriority;
  TXBCTRL  txb_ctrl;
  uint32_t timeout;
  uint32_t SID;
  uint32_t EID;
  uint8_t  nData[8];
  uint32_t nSize;
} SEND_DATA;

typedef struct _SEND_DATA_NOW
{
  uint8_t  nChannel;
  uint8_t  nBuf;
  TXBCTRL  txb_ctrl;
  uint32_t timeout;
} SEND_DATA_NOW;

typedef struct _SEND_DATA_TG
{
  uint8_t  nChannel;
  uint8_t  nBuf;
  uint8_t  bEpoch;
  uint32_t nEpoch;
  uint32_t nTrigger;
} SEND_DATA_TG;

typedef struct _MASKS_TTCAN
{
  uint8_t  channel;     // channel
  uint8_t  rxb_mode;    // RXB_MODE, 0-3
  uint8_t  filter;      // number of filter and mask (0-5) (first 2 - RXB0; second 4 - RXB1)
  uint8_t  ident;       // 0 - standart; 1 - extended
  uint32_t identific_m; // mask ident; first 0-10 bits - standart; 11-28 - extended
  uint32_t identific_f; // filter ident; first 0-10 bits - standart; 11-28 - extended
} MASKS_TTCAN;

typedef struct _VERSION_TTCAN
{
  uint32_t device_id;
  uint32_t vendor_id;
  uint8_t  revision;
  char     dev_name[30];
  uint32_t minor;
  uint32_t irq;
  uint64_t size_dma;
  uint64_t addr_dma_virt;
  uint64_t pci_bars;
} VERSION_TTCAN;

//CAN controller interrupts
#define RX01E 0x01 // receive buffer 0 (only when DMA turned off)
#define RX11E 0x02 // receive buffer 1 (only when DMA turned off)
#define TX01E 0x04 // transmit buffer 0
#define TX11E 0x08 // transmit buffer 1
#define TX21E 0x10 // transmit buffer 2
#define ERRIE 0x20 // bus error
#define WAKIE 0x40 // wake up interrupt
#define MERRE 0x80 // message error

typedef enum _CAN_BUF
{
  BUF1      = 0,
  BUF2      = 1,
  BUF3      = 2,
  CAN_BUFS  = 3
} CAN_BUF;

// work modes
typedef enum _CAN_MODE
{
  CAN_WORK  = 0,
  CAN_SLEEP = 1,
  CAN_LOOP  = 2,
  CAN_MON   = 3,
  CAN_CONF  = 4
} CAN_MODE;

// receive buffer operating mode
typedef enum _RXB_MODE
{
  RXB_MODE_ALL = 0,
  RXB_MODE_SID = 1,
  RXB_MODE_EID = 2,
  RXB_MODE_OFF = 3,
  RXB_MODE_NUM
} RXB_MODE;

// can controller regs

#define CAN_REGS    128

//interrupt
#define CANINTE     0x2B
#define CANINTF     0x2C

//errors
#define EFLG        0x2D
#define TEC         0x1C
#define REC         0x1D

//control
#define BFPCTRL     0x0C
#define TXRTSCTRL   0x0D

#define CAN_CTRL    0x0F
#define CAN_STAT    0x0E
#define CAN_CNF1    0x2A
#define CAN_CNF2    0x29
#define CAN_CNF3    0x28

//transmit ctrl
#define TXB0CTRL    0x30
#define TXB1CTRL    0x40
#define TXB2CTRL    0x50
#define TXB0SIDL    0x32
#define TXB1SIDL    0x42
#define TXB2SIDL    0x52
#define TXB0SIDH    0x31
#define TXB1SIDH    0x41
#define TXB2SIDH    0x51
#define TXB0EID8    0x33
#define TXB1EID8    0x43
#define TXB2EID8    0x53
#define TXB0EID0    0x34
#define TXB1EID0    0x44
#define TXB2EID0    0x54
#define TXB0DLC     0x35
#define TXB1DLC     0x45
#define TXB2DLC     0x55

//transmit buffers
#define TXB0D0      0x36
#define TXB1D0      0x46
#define TXB2D0      0x56
#define TXB0D1      0x37
#define TXB1D1      0x47
#define TXB2D1      0x57
#define TXB0D2      0x38
#define TXB1D2      0x48
#define TXB2D2      0x58
#define TXB0D3      0x39
#define TXB1D3      0x49
#define TXB2D3      0x59
#define TXB0D4      0x3A
#define TXB1D4      0x4A
#define TXB2D4      0x5A
#define TXB0D5      0x3B
#define TXB1D5      0x4B
#define TXB2D5      0x5B
#define TXB0D6      0x3C
#define TXB1D6      0x4C
#define TXB2D6      0x5C
#define TXB0D7      0x3D
#define TXB1D7      0x4D
#define TXB2D7      0x5D

//receive filters
#define RXF0SIDH    0x00
#define RXF1SIDH    0x04
#define RXF2SIDH    0x08
#define RXF3SIDH    0x10
#define RXF4SIDH    0x14
#define RXF5SIDH    0x18
#define RXF0SIDL    0x01
#define RXF1SIDL    0x05
#define RXF2SIDL    0x09
#define RXF3SIDL    0x11
#define RXF4SIDL    0x15
#define RXF5SIDL    0x19
#define RXF0EID8    0x02
#define RXF1EID8    0x06
#define RXF2EID8    0x0A
#define RXF3EID8    0x12
#define RXF4EID8    0x16
#define RXF5EID8    0x1A
#define RXF0EID0    0x03
#define RXF1EID0    0x07
#define RXF2EID0    0x0B
#define RXF3EID0    0x13
#define RXF4EID0    0x17
#define RXF5EID0    0x1B

//receive masks
#define RXM0SIDH    0x20
#define RXM1SIDH    0x24
#define RXM0SIDL    0x21
#define RXM1SIDL    0x25
#define RXM0EID8    0x22
#define RXM1EID8    0x26
#define RXM0EID0    0x23
#define RXM1EID0    0x27

//receive ctrl
#define RXB0CTRL    0x60
#define RXB1CTRL    0x70
#define RXB0SIDH    0x61
#define RXB1SIDH    0x71
#define RXB0SIDL    0x62
#define RXB1SIDL    0x72
#define RXB0EID8    0x63
#define RXB1EID8    0x73
#define RXB0EID0    0x64
#define RXB1EID0    0x74
#define RXB0DLC     0x65
#define RXB1DLC     0x75

//receive buffers
#define RXB0D0      0x66
#define RXB1D0      0x76
#define RXB0D1      0x67
#define RXB1D1      0x77
#define RXB0D2      0x68
#define RXB1D2      0x78
#define RXB0D3      0x69
#define RXB1D3      0x79
#define RXB0D4      0x6A
#define RXB1D4      0x7A
#define RXB0D5      0x6B
#define RXB1D5      0x7B
#define RXB0D6      0x6C
#define RXB1D6      0x7C
#define RXB0D7      0x6D
#define RXB1D7      0x7D

#define BFPCTRL     0x0C

#endif /* __NMTTCAN_H */
