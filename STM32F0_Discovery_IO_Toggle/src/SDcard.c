/*
 SDcard.c - functions to read content of an SD card
 utilizes FATfs (include in project, all but diskio.c, functions are implemented here, just add diskio.h)
 get it here: http://elm-chan.org/fsw/ff/00index_e.html

 Created by Damian Schneider, March 29th, 2013.

 code taken from "ronan" and adapted for CooCox STMf0, original posted here
 https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2FSTM32Discovery%2FDriver%20to%20use%20MicroSD%20card%20with%20SPI&FolderCTID=0x01200200770978C69A1141439FE559EB459D75800084C20D8867EAD444A5987D47BE638E0F&currentviews=2201

 also mixed in with some ST-library parts and stuff of my own "until it works".
 */

#include "stm32f0xx_spi.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx.h"
#include "ffconf.h"
#include "ff.h"
#include "xprintf.h"
#include "string.h"
#include "diskio.h"



/* SPIx Communication boards Interface */
#define GPIO_AF_SPI1      GPIO_AF_0
#define SPIx_SD                         SPI1
#define SPIx_SD_CLK                     RCC_APB2Periph_SPI1
#define SPIx_SD_CLK_INIT                RCC_APB2PeriphClockCmd
#define SPIx_SD_IRQn                    SPI1_IRQn
#define SPIx_SD_IRQHANDLER              SPI1_IRQHandler

#define SPIx_SD_SCK_PIN                 GPIO_Pin_5
#define SPIx_SD_SCK_GPIO_PORT           GPIOA
#define SPIx_SD_SCK_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define SPIx_SD_SCK_SOURCE              GPIO_PinSource5
#define SPIx_SD_SCK_AF                  GPIO_AF_SPI1

#define SPIx_SD_MISO_PIN                GPIO_Pin_7
#define SPIx_SD_MISO_GPIO_PORT          GPIOA
#define SPIx_SD_MISO_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define SPIx_SD_MISO_SOURCE             GPIO_PinSource7
#define SPIx_SD_MISO_AF                 GPIO_AF_SPI1

#define SPIx_SD_MOSI_PIN                GPIO_Pin_6
#define SPIx_SD_MOSI_GPIO_PORT          GPIOA
#define SPIx_SD_MOSI_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define SPIx_SD_MOSI_SOURCE             GPIO_PinSource6
#define SPIx_SD_MOSI_AF                 GPIO_AF_SPI1

#define SPIx_SD_NSS_PIN                 GPIO_Pin_15
#define SPIx_SD_NSS_GPIO_PORT           GPIOA
#define SPIx_SD_NSS_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define SPIx_SD_NSS_SOURCE              GPIO_PinSource15
#define SPIx_SD_NSS_AF                  GPIO_AF_SPI1

#define SPIx_SD_BAUDRATE_SLOW           SPI_BaudRatePrescaler_128
#define SPIx_SD_BAUDRATE_FAST           SPI_BaudRatePrescaler_2

/* Definitions for SDC command --------------------------------------------*/
#define CMD0    (0x40+0)    /* GO_IDLE_STATE */
#define CMD1    (0x40+1)    /* SEND_OP_COND (MMC) */
#define ACMD41  (0xC0+41)   /* SEND_OP_COND (SDC) */
#define CMD8    (0x40+8)    /* SEND_IF_COND */
#define CMD9    (0x40+9)    /* SEND_CSD */
#define CMD10   (0x40+10)   /* SEND_CID */
#define CMD12   (0x40+12)   /* STOP_TRANSMISSION */
#define ACMD13  (0xC0+13)   /* SD_STATUS (SDC) */
#define CMD16   (0x40+16)   /* SET_BLOCKLEN */
#define CMD17   (0x40+17)   /* READ_SINGLE_BLOCK */
#define CMD18   (0x40+18)   /* READ_MULTIPLE_BLOCK */
#define CMD23   (0x40+23)   /* SET_BLOCK_COUNT (MMC) */
#define ACMD23  (0xC0+23)   /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24   (0x40+24)   /* WRITE_BLOCK */
#define CMD25   (0x40+25)   /* WRITE_MULTIPLE_BLOCK */
#define CMD55   (0x40+55)   /* APP_CMD */
#define CMD58   (0x40+58)   /* READ_OCR */

/* Definitions for card type --------------------------------------------*/
#define SD2_BYTE        0x04        /* SD Ver.2 (Byte address) */
#define SD2_BLOCK       0x0C        /* SD Ver.2 (Block address) */

//pin select
#define SDSELECT()        GPIOA->BRR = (1<<15) //pin low, MMC CS = L
#define SDDESELECT()      GPIOA->BSRR = (1<<15) //pin high,MMC CS = H
/*--------------------------------------------------------------------------

 Module Private Functions and Variables

 ---------------------------------------------------------------------------*/

static const uint32_t socket_state_mask_cp = (1 << 0);
static const uint32_t socket_state_mask_wp = (1 << 1);

static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */
static volatile uint32_t Timer1, Timer2; /* 100Hz decrement timers */
static uint8_t CardType; /* Card type flags */

enum speed_setting {
  INTERFACE_SLOW, INTERFACE_FAST
};

/*-----------------------------------------------------------------------*/
/* @descr  INTERFACE SPEED                                               */
/* @param  speed    INTERFACE_SLOW OR INTERFACE_FAST                     */
/* @retval none                                                          */
/*-----------------------------------------------------------------------*/
static void interface_speed(enum speed_setting speed) {
  uint32_t tmp;

  /* Lecture du registre CR1 ------------------------------- */
  tmp = SPIx_SD->CR1;

  /* Test la configuration --- ----------------------------- */
  if (speed == INTERFACE_SLOW) {
    /* Set slow clock (100k-400k) ------------------------ */
    tmp = (tmp | SPIx_SD_BAUDRATE_SLOW);
  } else {
    /* Set fast clock (depends on the CSD) --------------- */
    tmp = (tmp & ~SPIx_SD_BAUDRATE_SLOW) | SPIx_SD_BAUDRATE_FAST;
  }

  /* Ecriture de la nouvelle config. sur le registre CR1 --- */
  SPIx_SD->CR1 = tmp;
}

/*-----------------------------------------------------------------------*/
/* @descr  TRANSMIT & RECEIVE BYTE via SPI                               */
/* @param  out              octet  transemttre                          */
/* @retval stm32_spi_rw     octet recu                                   */
/*-----------------------------------------------------------------------*/
uint8_t stm32_spi_rw(uint8_t out) {

  /*!< Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_TXE) == RESET) {
  }

  /*!< Send the byte */
  SPI_SendData8(SPIx_SD, out);

  /*!< Wait to receive a byte*/
  while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_RXNE) == RESET) {
  }

  /*!< Return the byte read from the SPI bus */
  return SPI_ReceiveData8(SPIx_SD);

}

/*-----------------------------------------------------------------------*/
/* @descr  RECEIVE A BYTE via SPI                                        */
/* @param  None                                                          */
/* @retval rcvr_spi     octet recu                                       */
/*-----------------------------------------------------------------------*/
uint8_t rcvr_spi(void) {
  uint8_t Data = 0;

  /*!< Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_TXE) == RESET) {
  }
  /*!< Send the byte */
  SPI_SendData8(SPIx_SD, 0xFF);

  /*!< Wait until a data is received */
  while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_RXNE) == RESET) {
  }
  /*!< Get the received data */
  Data = SPI_ReceiveData8(SPIx_SD);

  /*!< Return the shifted data */
  return Data;
}

/* Alternative macro to receive data fast */
#define rcvr_spi_m(dst)  *(dst)=stm32_spi_rw(0xff)

/*-----------------------------------------------------------------------*/
/* @descr  Wait for card ready                                           */
/* @param  None                                                          */
/* @retval wait_ready   octet recu                                       */
/*-----------------------------------------------------------------------*/
static uint8_t wait_ready(void) {
  uint8_t res;
  uint16_t timeout = 100; //try 100 times max.
  //Timer2 = 50;    /* Wait for ready in timeout of 500ms */
  rcvr_spi();

  do {
    res = rcvr_spi();
  } while ((res != 0xFF && timeout--)); // && Timer2);

  return res;
}

/*-----------------------------------------------------------------------*/
/* @descr  Deselect the card and release SPI bus                         */
/* @param  None                                                          */
/* @retval None                                                          */
/*-----------------------------------------------------------------------*/
static void release_spi(void) {

  SDDESELECT();
  rcvr_spi();
}

/*-----------------------------------------------------------------------*/
/* @descr  Power Control and interface-initialization                    */
/* @param  None                                                          */
/* @retval None                                                          */
/*-----------------------------------------------------------------------*/
static void power_on(void) {

  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
  RCC_AHBPeriphClockCmd(
      SPIx_SD_SCK_GPIO_CLK | SPIx_SD_MISO_GPIO_CLK | SPIx_SD_MOSI_GPIO_CLK
          | SPIx_SD_NSS_GPIO_CLK, ENABLE);

  /* Enable the SPI clock */SPIx_SD_CLK_INIT(SPIx_SD_CLK, ENABLE);

  /* SPI GPIO Configuration --------------------------------------------------*/

  /* Configure I/O for Flash Chip select */
  GPIO_InitStructure.GPIO_Pin = SPIx_SD_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPIx_SD_NSS_GPIO_PORT, &GPIO_InitStructure);

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_SD_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPIx_SD_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_SD_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPIx_SD_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_SD_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPIx_SD_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* Connect SPI pins to AF0 */
  GPIO_PinAFConfig(SPIx_SD_SCK_GPIO_PORT, SPIx_SD_SCK_SOURCE, SPIx_SD_SCK_AF);
  GPIO_PinAFConfig(SPIx_SD_MOSI_GPIO_PORT, SPIx_SD_MOSI_SOURCE,
      SPIx_SD_MOSI_AF);
  GPIO_PinAFConfig(SPIx_SD_MISO_GPIO_PORT, SPIx_SD_MISO_SOURCE,
      SPIx_SD_MOSI_AF);

  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_SD_BAUDRATE_SLOW; // 42000kHz/128=328kHz < 400kHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;

  SPI_Init(SPIx_SD, &SPI_InitStructure);
  SPI_RxFIFOThresholdConfig(SPIx_SD, SPI_RxFIFOThreshold_QF);

  SPI_CalculateCRC(SPIx_SD, DISABLE);
  SPI_Cmd(SPIx_SD, ENABLE);

  /* drain SPI TX buffer,just in case*/
  while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_TXE) == RESET) {
    ;
  }
  SPI_ReceiveData8(SPIx_SD);

  /* De-select the Card: Chip Select high */
  SDDESELECT();

  /*
   //DMA setup, in case dma is ever needed, code snipped, not working
   DMA_StructInit(&DMA_InitStructure);
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
   // reset DMA configuration

   //configure DMA
   DMA_InitStructure.DMA_MemoryBaseAddr = 0;
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI2->DR);
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_BufferSize = 1;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(SPI_PORT_TX_DMA_STREAM, &DMA_InitStructure);

   // Enable the DMA transfer complete interrupt
   DMA_ITConfig(SPI_PORT_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

   // Enable SPI dma tx request.
   SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

   // Enable the interrupt in the NVIC
   NVIC_InitStructure.NVIC_IRQChannel = SPI_PORT_DMA_TX_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   */

}

/*-----------------------------------------------------------------------*/
/* @descr  Power Off                                                     */
/* @param  None                                                          */
/* @retval None                                                          */
/*-----------------------------------------------------------------------*/
static void power_off(void) {

  GPIO_InitTypeDef GPIO_InitStructure;

  if (!(Stat & STA_NOINIT)) {
    SDSELECT();
    wait_ready();
    release_spi();
  }

  SPI_I2S_DeInit(SPIx_SD);
  SPI_Cmd(SPIx_SD, DISABLE);
  SPIx_SD_CLK_INIT(SPIx_SD_CLK, DISABLE);

  /* All SPI-Pins to input with weak internal pull-downs */
  GPIO_InitStructure.GPIO_Pin = SPIx_SD_SCK_PIN | SPIx_SD_MISO_PIN
      | SPIx_SD_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(SPIx_SD_SCK_GPIO_PORT, &GPIO_InitStructure);

  Stat |= STA_NOINIT; /* Set STA_NOINIT */

}

/*-----------------------------------------------------------------------*/
/* @descr  Receive a data packet from MMC                                */
/* @param  *buff    Data buffer to store received data                   */
/* @param  btr      Byte count (must be multiple of 4)                   */
/* @retval None                                                          */
/*-----------------------------------------------------------------------*/
static uint8_t rcvr_datablock(uint8_t *buff, uint16_t btr) {
  uint8_t token;

  Timer1 = 10;
  do { /* Wait for data packet in timeout of 100ms */
    token = rcvr_spi();
  } while ((token == 0xFF) && Timer1);

  if (token != 0xFE)
    return 0; /* If not valid data token, return with error */

  do { /* Receive the data block into buffer */
    rcvr_spi_m(buff++);
    rcvr_spi_m(buff++);
    rcvr_spi_m(buff++);
    rcvr_spi_m(buff++);
//	  rcvr_spi_m(buff+3);
//	  rcvr_spi_m(buff+2);
//	  rcvr_spi_m(buff+1);
//	  rcvr_spi_m(buff+0);
//	  buff+=4;
  } while (btr -= 4);

  rcvr_spi(); /* Discard CRC */
  rcvr_spi();

  return 1; /* Return with success */
}

/*-----------------------------------------------------------------------*/
/* @descr  end a data packet to MMC                                      */
/* @param  *buff    512 byte data block to be transmitted                */
/* @param  token    Data/Stop token                                      */
/* @retval None                                                          */
/*-----------------------------------------------------------------------*/
static uint8_t xmit_datablock(const uint8_t *buff, uint8_t token) {
  uint8_t resp;
  uint8_t wc;

  if (wait_ready() != 0xFF)
    return 0;
  stm32_spi_rw(token); /* transmit data token */
  if (token != 0xFD) { /* Is data token */

    wc = 0;
    do { /* transmit the 512 byte data block to MMC */
      stm32_spi_rw(*buff++);
      stm32_spi_rw(*buff++);
    } while (--wc);

    stm32_spi_rw(0xFF); /* CRC (Dummy) */
    stm32_spi_rw(0xFF);
    resp = rcvr_spi(); /* Receive data response */
    if ((resp & 0x1F) != 0x05) /* If not accepted, return with error */
      return 0;
  }

  return 1;
}

/*-----------------------------------------------------------------------*/
/* @descr  SEND A COMMAND PACKET (6 BYTES)                               */
/* @param  cmd      Command byte                                         */
/* @param  arg      Argument                                             */
/* @retval None                                                          */
/*-----------------------------------------------------------------------*/
static uint8_t send_cmd(uint8_t cmd, uint32_t arg) {

  /* Declaration des variables ----------------------------- */
  uint8_t n, res;

  /* Test si c'est cmd ACDM -------------------------------- */
  if (cmd & 0x80) {
    cmd &= 0x7F;
    res = send_cmd(CMD55, 0); // envoie de la CMD55
    if (res > 1)
      return res;
  }

  /* Select the card --------------------------------------- */
  //  SDDESELECT();
  SDSELECT();

  /* wait for ready ---------------------------------------- */
  if (wait_ready() != 0xFF) {
    return 0xFF;
  }

  /* Send command packet ----------------------------------- */
  stm32_spi_rw(cmd); // Send command index

  stm32_spi_rw((uint8_t)(arg >> 24)); // Send argument[31..24]
  stm32_spi_rw((uint8_t)(arg >> 16)); // Send argument[23..16]
  stm32_spi_rw((uint8_t)(arg >> 8)); // Send argument[15..8]
  stm32_spi_rw((uint8_t) arg); // Send argument[7..0]

  n = 0x01; // Stop : Dummy CRC
  if (cmd == CMD0)
    n = 0x95; // Valid CRC for CMD0(0)
  if (cmd == CMD8)
    n = 0x87; // Valid CRC for CMD8(0x1AA)
  stm32_spi_rw(n); // Send CRC

  /* Receive command response ------------------------------ */
  if (cmd == CMD12)
    rcvr_spi(); // Skip a stuff byte when stop reading

  /* Wait for a valid response in timeout of 10 attempts --- */
  n = 10;
  do
    res = rcvr_spi();
  while ((res & 0x80) && --n);

  /* Return with the response value ------------------------ */
  return res;
}

/*-----------------------------------------------------------------------*/
/* @descr  INITIALIZE DISK DRIVE                                         */
/* @param  drv      Physical drive numbre (0)                            */
/* @retval DSTATUS  Status of Disk Functions (BYTE)                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize(uint8_t drv) {

  /* Declaration des variables ----------------------------- */
  uint8_t i, ocr[4], SDType;

  /* Supports only single drive ---------------------------- */
  if (drv)
    return STA_NOINIT;

  /* No card in the socket --------------------------------- */
  if (Stat & STA_NODISK)
    return Stat;

  /* Configure les broches & alimente la carte si necessaire */
  power_on();

  SDDESELECT();

  /* 80 dummy clocks --------------------------------------- */
  for (i = 10; i; i--) {
    stm32_spi_rw(0xff);
  }

  /* Initialise SDType ------------------------------------- */
  SDType = 0;
// DEBUG
//  SDSELECT();
//  while (1) stm32_spi_rw(0xaa);
// END DEBUG


  /* Initialise la SDCard en mode SPI ---------------------- */
  if (send_cmd(CMD0, 0) == 1) {

    /* Verifie les conditions (tension,...) -------------- */
    if (send_cmd(CMD8, 0x1AA) == 1) {
      /* lecture des trames de reponses ---------------- */
      for (i = 0; i < 4; i++)
        ocr[i] = rcvr_spi();

      /* Verifie les tensions autorisees --------------- */
      if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
        /* Attend de quitter l'etat repos ------------ */
        while (send_cmd(ACMD41, 0x40000000))
          ;

        /* Envoie de la commande OCR ----------------- */
        if (send_cmd(CMD58, 0) == 0) {
          for (i = 0; i < 4; i++)
            ocr[i] = rcvr_spi();

          /* Control bit CCS dans le registre OCR -- */
          if (ocr[0] & 0x40) {
            SDType = SD2_BLOCK;

          } else {
            SDType = SD2_BYTE;

          }
        }
      }
    }
  }

  /* Erreur d'initialisation ------------------------------- */
  else {
    SDType = 0;
  }

  /* Deselectionne la liaison SPI -------------------------- */
  release_spi();

  /* Affecte le type de la carte a CardType ---------------- */
  CardType = SDType;

  /* Test si l'initialisation est valide ------------------- */
  if (SDType) {
    /* Initialization succeeded -------------------------- */
    Stat &= ~STA_NOINIT; /* Clear STA_NOINIT */
    interface_speed(INTERFACE_FAST);
//    LED_ON;

  } else {
    /* Initialization failed ----------------------------- */
    power_off();
  }

  return Stat;

}

/*-----------------------------------------------------------------------*/
/* @descr  GET DISK STATUS                                               */
/* @param  drv      Physical drive numbre (0)                            */
/* @retval DSTATUS  Status of Disk Functions (BYTE)                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(uint8_t drv) {
  if (drv)
    return STA_NOINIT; /* Supports only single drive */
  return Stat;
}

/*-----------------------------------------------------------------------*/
/* @descr  READ SECTOR(S)                                                */
/* @param  drv      Physical drive numbre (0)                            */
/* @param  *buff    Pointer to the data buffer to store read data        */
/* @param  sector   Start sector number (LBA)                            */
/* @param  count    Sector count (1..255)                                */
/* @retval DRESULT  Results of Disk Functions                            */
/*-----------------------------------------------------------------------*/
DRESULT disk_read(uint8_t drv, uint8_t *buff, uint32_t sector, uint8_t count) {
  /* Check parameters -------------------------------------- */
  if (drv || !count)
    return RES_PARERR;
  if (Stat & STA_NOINIT)
    return RES_NOTRDY;

  /* Convert to byte address if needed --------------------- */
  if (!(CardType & CT_BLOCK))
    sector *= 512;

  /* Test : Single or Multiple block read ------------------ */
  if (count == 1) {
    /* Single block read */
    if (send_cmd(CMD17, sector) == 0) {
      /* READ_SINGLE_BLOCK */
      if (rcvr_datablock(buff, 512)) {
        count = 0;
      }
    }
  } else {
    /* Multiple block read */
    if (send_cmd(CMD18, sector) == 0) {
      /* READ_MULTIPLE_BLOCK */
      do {
        if (!rcvr_datablock(buff, 512)) {
          break;
        }
        buff += 512;
      } while (--count);

      /* STOP_TRANSMISSION */
      send_cmd(CMD12, 0);
    }
  }
  release_spi();

  return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* @descr  WRITE SECTOR(S)                                               */
/* @param  drv      Physical drive numbre (0)                            */
/* @param  *buff    Pointer to the data to be written                    */
/* @param  sector   Start sector number (LBA)                            */
/* @param  count    Sector count (1..255)                                */
/* @retval DRESULT  Results of Disk Functions                            */
/*-----------------------------------------------------------------------*/
DRESULT disk_write(uint8_t drv, const uint8_t *buff, uint32_t sector,
    uint8_t count) {
  /* Check parameters -------------------------------------- */
  if (drv || !count)
    return RES_PARERR;
  if (Stat & STA_NOINIT)
    return RES_NOTRDY;
  if (Stat & STA_PROTECT)
    return RES_WRPRT;

  /* Convert to byte address if needed --------------------- */
  if (!(CardType & CT_BLOCK))
    sector *= 512;

  /* Test : Single or Multiple block write ----------------- */
  if (count == 1) {
    /* Single block write -------------------------------- */
    if ((send_cmd(CMD24, sector) == 0) /* WRITE_BLOCK */
    && xmit_datablock(buff, 0xFE))
      count = 0;
  } else {
    /* Multiple block write */
    if (CardType & CT_SDC)
      send_cmd(ACMD23, count);
    if (send_cmd(CMD25, sector) == 0) {
      /* WRITE_MULTIPLE_BLOCK */
      do {
        if (!xmit_datablock(buff, 0xFC))
          break;
        buff += 512;
      } while (--count);

      /* STOP_TRAN token */
      if (!xmit_datablock(0, 0xFD))
        count = 1;
    }
  }
  release_spi();

  return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* @descr  GET CURRENT TIME INTO A uint32_t VALUE                           */
/* @param  drv      Physical drive number (0)                            */
/* @param  ctrl     Control code                                         */
/* @param  *buff    Buffer to send/receive control data                  */
/* @param  DRESULT  Results of Disk Functions                            */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl(uint8_t drv, uint8_t ctrl, void *buff) {
  DRESULT res;

  res = RES_OK;

  return res;
}

/*-----------------------------------------------------------------------*/
/* @descr  GET CURRENT TIME INTO A uint32_t VALUE , used in FATfs                        */
/* @param  none                                                          */
/* @retval uint32_t    Results of Disk Functions                            */
/*-----------------------------------------------------------------------*/
uint32_t get_fattime(void) {
  uint32_t res;
  res = 42; //dummy time and date, RTC to be implemented
  /*
   RTC_TimeTypeDef RTC_TimeStructure;
   RTC_DateTypeDef RTC_DateStructure;

   RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
   RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

   res =       (((uint32_t)RTC_DateStructure.RTC_Year + 20)   << 25)      // bit31:25     Year from 1980 (0..127)
   |   ((uint32_t)RTC_DateStructure.RTC_Month         << 21)      // bit24:21     Month (1..12)
   |   ((uint32_t)RTC_DateStructure.RTC_Date          << 16)      // bit20:16     Day in month(1..31)
   |   (uint16_t)(RTC_TimeStructure.RTC_Hours          << 11)      // bit15:11     Hour (0..23)
   |   (uint16_t)(RTC_TimeStructure.RTC_Minutes        << 5)       // bit10:5      Minute (0..59)
   |   (uint16_t)(RTC_TimeStructure.RTC_Seconds        >> 1);      // bit4:0       Second / 2 (0..29)
   */

  return res;
}


/*
scan_files() function is an example from the FatFS library.
if called with empty string as a path, it prints out all files on the card.
*/

FRESULT scan_files(char* path) {
  FRESULT res;
  FILINFO fno;
  DIR dir;
  int i;
  char *fn; /* This function is assuming non-Unicode cfg. */
#if _USE_LFN
  static char lfn[_MAX_LFN + 1];
  fno.lfname = lfn;
  fno.lfsize = sizeof lfn;
#endif

  res = f_opendir(&dir, path); /* Open the directory */
  if (res == FR_OK) {
    i = strlen(path);
    for (;;) {
      xprintf("scan...");
      res = f_readdir(&dir, &fno); /* Read a directory item */
      if (res != FR_OK || (fno.fname[0] == 0))
        break; /* Break on error or end of dir */
      if (fno.fname[0] == '.')
        continue; /* Ignore dot entry */
      xprintf("file found\r\n");
#if _USE_LFN
      fn = *fno.lfname ? fno.lfname : fno.fname;
#else
      fn = fno.fname;
#endif
      if (fno.fattrib & AM_DIR) { /* It is a directory */
        xsprintf(&path[i], "/%s", fn);
        res = scan_files(path);
        if (res != FR_OK)
          break;
        path[i] = 0;
      } else { /* It is a file. */
        xprintf("%s/%s\n", path, fn);
      }
    }
  }

  return res;
}


/*
 *
 * basic function test for debugging
 *
 */
/*
void SDtest(void) {

  FILINFO Finfo; // Work register for fs command
  FATFS Fatfs; //to mount the sd card
  DIR Dir; //current directory
  uint8_t res;

//  disk_initialize(0);//done when calling mount

  xprintf("mounting SD card: \r\n");

  // Mount Fatfs Drive
  f_mount(0, &Fatfs, 1); // Mount immediately

  const char newline[] = "\r\n";
  const char nullstring[] = "";

  if (f_opendir(&Dir, (const char*) nullstring) == FR_OK) {

    xprintf("root dir opened\r\n");

    while (((res = f_readdir(&Dir, &Finfo)) == FR_OK) && Finfo.fname[0]) {
      //  xprintf("reading directory... ");
      //  xprintf(Finfo.fname);
      //  xprintf(newline);
      uint8_t i = 0;
      while (Finfo.fname[i] && Finfo.fname[i + 2]) //scan file name
      {
        if (Finfo.fname[i] == 'B' || Finfo.fname[i] == 'b') {
          if (Finfo.fname[i + 1] == 'M'
              || Finfo.fname[i + 1] == 'm') {
            if (Finfo.fname[i + 2] == 'P'
                || Finfo.fname[i + 2] == 'p') {
              //xprintf("BMP found! name: ");
              xprintf(Finfo.fname);
              xprintf(newline);
              showimage((char*) Finfo.fname); //display image
              break;
            }
          }
        }
        i++;
      }

    }
  }

}
*/
