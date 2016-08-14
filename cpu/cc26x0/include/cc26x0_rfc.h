/*
 * Copyright (C) 2016 Leon George, Florent-Valéry Coen
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_cc26x0_rfc_definitions
 * @{
 *
 * @file
 * @brief           CC26x0 RFC register definitions
 *
 * @author          Leon George <leon@georgemail.eu>
 * @author          Florent-Valéry Coen <florent.coen@gmail.com>
 */

#ifndef CC26x0_RFC_H
#define CC26x0_RFC_H

#include "cc26x0.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @ingroup cpu_specific_peripheral_memory_map
 * @{
 */
#define RFC_RAT_BASE           (0x40043000)
/** @} */


typedef struct {
    reg32_t __reserved0;
    reg32_t CNT;
    reg32_t __reserved1[30];
    reg32_t CH0VAL;
    reg32_t CH1VAL;
    reg32_t CH2VAL;
    reg32_t CH3VAL;
    reg32_t CH4VAL;
    reg32_t CH5VAL;
    reg32_t CH6VAL;
    reg32_t CH7VAL;
} cc26x0_rfc_rat_regs_t;

#define RFC_RAT ((cc26x0_rfc_rat_regs_t *)(RFC_RAT_BASE))



/** @ingroup cpu_specific_peripheral_memory_map
 * @{
 */
#define RFC_DBELL_BASE           (0x40041000)
/** @} */


typedef struct {
    reg32_t CMDR; /** Doorbell Command Register */
    reg32_t CMDSTA; /** Doorbell Command Status Register */
    reg32_t RFHWIFG; /** Interrupt Flags From RF Hardware Modules */
    reg32_t RFHWIEN; /** Interrupt Enable For RF Hardware Modules */
    reg32_t RFCPEIFG; /** Interrupt Flags For Command and Packet Engine Generated Interrupts */
    reg32_t RFCPEIEN; /** Interrupt Enable For Command and Packet Engine Generated Interrupts */
    reg32_t RFCPEISL; /** Interrupt Vector Selection For Command and Packet Engine Generated Interrupts */
    reg32_t RFACKIFG; /** Doorbell Command Acknowledgement Interrupt Flag */
    reg32_t SYSGPOCTL; /** RF Core General Purpose Output Control */
} cc26x0_rfc_dbell_regs_t;

#define RFC_DBELL ((cc26x0_rfc_dbell_regs_t *)(RFC_DBELL_BASE))

#define CMDR_TYPE_MASK                  (0x3)
#define CMDR_TYPE_ROP_IMM               (0x0)
/* pointer is 32-bit word aligned and the 2 LSB are not stored (implicit 0) */
#define CMDR_CMDPTR_POS                 (0x2) /* used for immediate and radio commands */
#define CMDR_CMDPTR_MASK                (0xFFFFFFFC)
#define CMDR_TYPE_DIRECT                (0x1)
#define CMDR_OPTPAREXT_POS              (0x2) /* used for direct commands */
#define CMDR_OPTPAREXT_MASK             (0xFC)
#define CMDR_OPTPAR_POS                 (0x8)
#define CMDR_OPTPAR_MASK                (0xFF00)
#define CMDR_CMDID_POS                  (0x10)
#define CMDR_CMDID_MASK                 (0xFFFF0000)
/**
 * @addtogroup cc2x0_rop_command_ids
 * @{
 */
/* radio operation */
#define CMDR_CMDID_NOP                  (0x0801)
#define CMDR_CMDID_SETUP                (0x0802)
#define CMDR_CMDID_FS_POWERUP           (0x080C)
#define CMDR_CMDID_FS_POWERDOWN         (0x080D)
#define CMDR_CMDID_FS                   (0x0803)
#define CMDR_CMDID_FS_OFF               (0x0804)
#define CMDR_CMDID_FS_RX_TEST           (0x0807)
#define CMDR_CMDID_FS_TX_TEST           (0x0808)
#define CMDR_CMDID_SYNC_STOP_RAT        (0x0809)
#define CMDR_CMDID_SYNC_START_RAT       (0x080A)
#define CMDR_CMDID_COUNT                (0x080B)
#define CMDR_CMDID_SCH_IMM              (0x0810)
#define CMDR_CMDID_COUNT_BRANCH         (0x0812)
#define CMDR_CMDID_PATTERN_CHECK        (0x0813)
/* immediate */
#define CMDR_CMDID_ABORT                (0x0401)
#define CMDR_CMDID_STOP                 (0x0402)
#define CMDR_CMDID_GET_RSSI             (0x0403)
#define CMDR_CMDID_UPD_RADIO_SET        (0x0001)
#define CMDR_CMDID_TRIGGER              (0x0404)
#define CMDR_CMDID_GET_FW_INFO          (0x0002)
#define CMDR_CMDID_START_RAT            (0x0405)
#define CMDR_CMDID_PING                 (0x0406)
#define CMDR_CMDID_READ_RF_REG          (0x0601)
#define CMDR_CMDID_SET_RAT_CMP          (0x000A)
#define CMDR_CMDID_SET_RAT_CPT          (0x0603)
#define CMDR_CMDID_DISABLE_RAT_CH       (0x0408)
#define CMDR_CMDID_SET_RAT_OUT          (0x0604)
#define CMDR_CMDID_SET_ARM_RAT_CH       (0x0409)
#define CMDR_CMDID_DISARM_RAT_CH        (0x040A)
#define CMDR_CMDID_SET_TX_POWER         (0x0010)
#define CMDR_CMDID_UPDATE_FS            (0x0011)
#define CMDR_CMDID_BUS_REQUEST          (0x040E)
/* immediate queue */
#define CMDR_CMDID_ADD_DATA_ENTRY       (0x0005)
#define CMDR_CMDID_REM_DATA_ENTRY       (0x0006)
#define CMDR_CMDID_FLUSH_QUEUE          (0x0007)
#define CMDR_CMDID_CLEAR_RX             (0x0008)
#define CMDR_CMDID_REM_PENDING_ENTRIES  (0x0009) /* TODO format */
/**@}*/

#define CMDSTA_RESULT_MASK              (0xFF)
#define CMDSTA_RET_VAL_POS              (0x8) /* MSB first */
#define CMDSTA_RET_VAL_MASK             (0xFFFFFF00)

#define CMDSTA_RESULT_PENDING           (0x00)
#define CMDSTA_RESULT_DONE              (0x01)
#define CMDSTA_RESULT_ILLEGALPOINTER    (0x81)
#define CMDSTA_RESULT_UNKNOWNCMD        (0x82)
#define CMDSTA_RESULT_UNKNOWNDIRCMD     (0x83)
#define CMDSTA_RESULT_CONTEXTERROR      (0x85)
#define CMDSTA_RESULT_SCHEDULINGERROR   (0x86)
#define CMDSTA_RESULT_PARERROR          (0x87)
#define CMDSTA_RESULT_QUEUEERROR        (0x88)
#define CMDSTA_RESULT_QUEUEBUSY         (0x89)



#define RFHWIFG_FSCA          (0x00000002)
#define RFHWIFG_MDMDONE       (0x00000004)
#define RFHWIFG_MDMIN         (0x00000008)
#define RFHWIFG_MDMOUT        (0x00000010)
#define RFHWIFG_MDMSOFT       (0x00000020)
#define RFHWIFG_TRCTK         (0x00000040)
#define RFHWIFG_RFEDONE       (0x00000100)
#define RFHWIFG_RFESOFT1      (0x00000200)
#define RFHWIFG_RFESOFT2      (0x00000400)
#define RFHWIFG_RFESOFT3      (0x00000800)
#define RFHWIFG_RATCH0        (0x00001000)
#define RFHWIFG_RATCH1        (0x00002000)
#define RFHWIFG_RATCH2        (0x00004000)
#define RFHWIFG_RATCH3        (0x00008000)
#define RFHWIFG_RATCH4        (0x00010000)
#define RFHWIFG_RATCH5        (0x00020000)
#define RFHWIFG_RATCH6        (0x00040000)
#define RFHWIFG_RATCH7        (0x00080000)

#define RFHWIEN_FSCA          (0x00000002)
#define RFHWIEN_MDMDONE       (0x00000004)
#define RFHWIEN_MDMIN         (0x00000008)
#define RFHWIEN_MDMOUT        (0x00000010)
#define RFHWIEN_MDMSOFT       (0x00000020)
#define RFHWIEN_TRCTK         (0x00000040)
#define RFHWIEN_RFEDONE       (0x00000100)
#define RFHWIEN_RFESOFT1      (0x00000200)
#define RFHWIEN_RFESOFT2      (0x00000400)
#define RFHWIEN_RFESOFT3      (0x00000800)
#define RFHWIEN_RATCH0        (0x00001000)
#define RFHWIEN_RATCH1        (0x00002000)
#define RFHWIEN_RATCH2        (0x00004000)
#define RFHWIEN_RATCH3        (0x00008000)
#define RFHWIEN_RATCH4        (0x00010000)
#define RFHWIEN_RATCH5        (0x00020000)
#define RFHWIEN_RATCH6        (0x00040000)
#define RFHWIEN_RATCH7        (0x00080000)

#define RFCPEIFG_COMMAND_DONE         (0x00000001)
#define RFCPEIFG_LAST_COMMAND_DONE    (0x00000002)
#define RFCPEIFG_FG_COMMAND_DONE      (0x00000004)
#define RFCPEIFG_FG_LAST_COMMAND_DONE (0x00000008)
#define RFCPEIFG_TX_DONE              (0x00000010)
#define RFCPEIFG_TX_ACK               (0x00000020)
#define RFCPEIFG_TX_CTRL              (0x00000030)
#define RFCPEIFG_TX_CTRL_ACK          (0x00000040)
#define RFCPEIFG_TX_CTRL_ACK_ACK      (0x00000100)
#define RFCPEIFG_TX_RETRANS           (0x00000200)
#define RFCPEIFG_TX_ENTRY_DONE        (0x00000400)
#define RFCPEIFG_TX_BUFFER_CHANGED    (0x00000800)
#define RFCPEIFG_TX_BUFFER_IRQ12      (0x00001000)
#define RFCPEIFG_TX_BUFFER_IRQ13      (0x00002000)
#define RFCPEIFG_TX_BUFFER_IRQ14      (0x00004000)
#define RFCPEIFG_TX_BUFFER_IRQ15      (0x00008000)
#define RFCPEIFG_RX_OK                (0x00010000)
#define RFCPEIFG_RX_NOK               (0x00020000)
#define RFCPEIFG_RX_IGNORED           (0x00040000)
#define RFCPEIFG_RX_EMPTY             (0x00080000)
#define RFCPEIFG_RX_CTRL              (0x00100000)
#define RFCPEIFG_RX_CTRL_ACK          (0x00200000)
#define RFCPEIFG_RX_BUF_FULL          (0x00400000)
#define RFCPEIFG_RX_ENTRY_DONE        (0x00800000)
#define RFCPEIFG_RX_DATA_WRITTEN      (0x01000000)
#define RFCPEIFG_RX_N_DATA_WRITTEN    (0x02000000)
#define RFCPEIFG_RX_ABORTED           (0x04000000)
#define RFCPEIFG_IRQ27                (0x08000000)
#define RFCPEIFG_SYNTH_NO_LOCK        (0x10000000)
#define RFCPEIFG_MODULES_UNLOCKED     (0x20000000)
#define RFCPEIFG_BOOT_DONE            (0x40000000)
#define RFCPEIFG_INTERNAL_ERROR       (0x80000000)

#define RFCPEIEN_COMMAND_DONE         (0x00000001)
#define RFCPEIEN_LAST_COMMAND_DONE    (0x00000002)
#define RFCPEIEN_FG_COMMAND_DONE      (0x00000004)
#define RFCPEIEN_FG_LAST_COMMAND_DONE (0x00000008)
#define RFCPEIEN_TX_DONE              (0x00000010)
#define RFCPEIEN_TX_ACK               (0x00000020)
#define RFCPEIEN_TX_CTRL              (0x00000030)
#define RFCPEIEN_TX_CTRL_ACK          (0x00000040)
#define RFCPEIEN_TX_CTRL_ACK_ACK      (0x00000100)
#define RFCPEIEN_TX_RETRANS           (0x00000200)
#define RFCPEIEN_TX_ENTRY_DONE        (0x00000400)
#define RFCPEIEN_TX_BUFFER_CHANGED    (0x00000800)
#define RFCPEIEN_TX_BUFFER_IRQ12      (0x00001000)
#define RFCPEIEN_TX_BUFFER_IRQ13      (0x00002000)
#define RFCPEIEN_TX_BUFFER_IRQ14      (0x00004000)
#define RFCPEIEN_TX_BUFFER_IRQ15      (0x00008000)
#define RFCPEIEN_RX_OK                (0x00010000)
#define RFCPEIEN_RX_NOK               (0x00020000)
#define RFCPEIEN_RX_IGNORED           (0x00040000)
#define RFCPEIEN_RX_EMPTY             (0x00080000)
#define RFCPEIEN_RX_CTRL              (0x00100000)
#define RFCPEIEN_RX_CTRL_ACK          (0x00200000)
#define RFCPEIEN_RX_BUF_FULL          (0x00400000)
#define RFCPEIEN_RX_ENTRY_DONE        (0x00800000)
#define RFCPEIEN_RX_DATA_WRITTEN      (0x01000000)
#define RFCPEIEN_RX_N_DATA_WRITTEN    (0x02000000)
#define RFCPEIEN_RX_ABORTED           (0x04000000)
#define RFCPEIEN_IRQ27                (0x08000000)
#define RFCPEIEN_SYNTH_NO_LOCK        (0x10000000)
#define RFCPEIEN_MODULES_UNLOCKED     (0x20000000)
#define RFCPEIEN_BOOT_DONE            (0x40000000)
#define RFCPEIEN_INTERNAL_ERROR       (0x80000000)

#define SYSGPOCTL_GPOCTL0_POS         (0x0)
#define SYSGPOCTL_GPOCTL0_LEN         (0x4)
#define SYSGPOCTL_GPOCTL1_POS         (0x4)
#define SYSGPOCTL_GPOCTL1_LEN         (0x4)
#define SYSGPOCTL_GPOCTL2_POS         (0x8)
#define SYSGPOCTL_GPOCTL2_LEN         (0x4)
#define SYSGPOCTL_GPOCTL3_POS         (0xC)
#define SYSGPOCTL_GPOCTL3_LEN         (0x4)

#define SYSGPOCTL_GPOCTL_CPE0         (0x0)
#define SYSGPOCTL_GPOCTL_CPE1         (0x1)
#define SYSGPOCTL_GPOCTL_CPE2         (0x2)
#define SYSGPOCTL_GPOCTL_CPE3         (0x3)
#define SYSGPOCTL_GPOCTL_MCE0         (0x4)
#define SYSGPOCTL_GPOCTL_MCE1         (0x5)
#define SYSGPOCTL_GPOCTL_MCE2         (0x6)
#define SYSGPOCTL_GPOCTL_MCE3         (0x7)
#define SYSGPOCTL_GPOCTL_RFE0         (0x8)
#define SYSGPOCTL_GPOCTL_RFE1         (0x9)
#define SYSGPOCTL_GPOCTL_RFE2         (0xa)
#define SYSGPOCTL_GPOCTL_RFE3         (0xb)
#define SYSGPOCTL_GPOCTL_RAT0         (0xc)
#define SYSGPOCTL_GPOCTL_RAT1         (0xd)
#define SYSGPOCTL_GPOCTL_RAT2         (0xe)
#define SYSGPOCTL_GPOCTL_RAT3         (0xf)




/** @ingroup cpu_specific_peripheral_memory_map
 * @{
 */
#define RFC_PWR_BASE           (0x40040000)
/** @} */

typedef struct {
    reg32_t PWMCLKEN;
} cc26x0_rfc_pwr_regs_t;

#define RFC_PWR ((cc26x0_rfc_pwr_regs_t *)(RFC_PWR_BASE))

#define PWMCLKEN_RFC        (0x00000001)
#define PWMCLKEN_CPE        (0x00000002)
#define PWMCLKEN_CPERAM     (0x00000004)
#define PWMCLKEN_MDM        (0x00000008)
#define PWMCLKEN_MDMRAM     (0x00000010)
#define PWMCLKEN_RFE        (0x00000020)
#define PWMCLKEN_RFERAM     (0x00000040)
#define PWMCLKEN_RAT        (0x00000080)
#define PWMCLKEN_PHA        (0x00000100)
#define PWMCLKEN_FSCA       (0x00000200)
#define PWMCLKEN_RFCTRC     (0x00000400)


/* direct command type*/
typedef struct {
    uint16_t commandID;
    uint8_t optParam;
    unsigned int optParamExt: 6;
    unsigned int commandType: 2; /*must be 01 for direct command*/
} direct_command_t;

/// Type definition for a data queue
typedef struct {
    uint8_t *pCurrEntry;   ///< Pointer to the data queue entry to be used, NULL for an empty queue
    uint8_t *pLastEntry;   ///< Pointer to the last entry in the queue, NULL for a circular queue
} dataQueue_t;

/*
 * R: system CPU can read a result back; the radio CPU does not read the field
 * W: system CPU writes a value, the radio CPU reads it and does not modify it
 * R/W: system CPU writes an initial value, the radio CPU may modify it
 */

typedef struct radio_op_command_s radio_op_command_t;
typedef struct rfc_CMD_BLE_ADV_NC_s rfc_CMD_BLE_ADV_NC_t;
typedef struct rfc_bleAdvPar_s rfc_bleAdvPar_t;
typedef struct rfc_bleAdvOutput_s rfc_bleAdvOutput_t;
typedef struct rfc_bleWhiteListEntry_s rfc_bleWhiteListEntry_t;


struct __attribute__ ((packed)) radio_op_command_s {
    uint16_t commandNo; /* W */
    uint16_t status; /* R/W */
    radio_op_command_t *pNextOp; /* W */
    uint32_t startTime; /* W */
    struct {
        uint8_t triggerType:4;
        uint8_t bEnaCmd:1;
        uint8_t triggerNo:2;
        uint8_t pastTrig:1;
    } startTrigger;       /* missing from datashiet... */
    struct {
        uint8_t rule:4;
        uint8_t nSkip:4;
    } condition; /* W */
}; /* rop require 32-bit word alignement, i.e. the 2 LSB of cmd addr are 0 */

struct __attribute__ ((packed)) rfc_CMD_BLE_ADV_NC_s {
   uint16_t commandNo;                  //!<        The command ID number 0x1805
   uint16_t status;                     //!< \brief An integer telling the status of the command. This value is
                                        //!<        updated by the radio CPU during operation and may be read by the
                                        //!<        system CPU at any time.
   radio_op_command_t *pNextOp;         //!<        Pointer to the next operation to run after this operation is done
   uint32_t startTime;                  //!<        Absolute or relative start time (depending on the value of <code>startTrigger</code>)
   struct {
      uint8_t triggerType:4;            //!<        The type of trigger
      uint8_t bEnaCmd:1;                //!< \brief 0: No alternative trigger command<br>
                                        //!<        1: CMD_TRIGGER can be used as an alternative trigger
      uint8_t triggerNo:2;              //!<        The trigger number of the CMD_TRIGGER command that triggers this action
      uint8_t pastTrig:1;               //!< \brief 0: A trigger in the past is never triggered, or for start of commands, give an error<br>
                                        //!<        1: A trigger in the past is triggered as soon as possible
   } startTrigger;                      //!<        Identification of the trigger that starts the operation
   struct {
      uint8_t rule:4;                   //!<        Condition for running next command: Rule for how to proceed
      uint8_t nSkip:4;                  //!<        Number of skips if the rule involves skipping
   } condition;
   uint8_t channel;                     //!< \brief Channel to use<br>
                                        //!<        0&ndash;39: BLE advertising/data channel number
                                        //!<        60&ndash;207: Custom frequency; (2300 + <code>channel</code>) MHz
                                        //!<        255: Use existing frequency
                                        //!<        Others: <i>Reserved<i>
   struct {
      uint8_t init:7;                   //!< \brief If <code>bOverride</code> = 1 or custom frequency is used:<br>
                                        //!<        0: Do not use whitening<br>
                                        //!<        Other value: Initialization for 7-bit LFSR whitener
      uint8_t bOverride:1;              //!< \brief 0: Use default whitening for BLE advertising/data channels<br>
                                        //!<        1: Override whitening initialization with value of init
   } whitening;
   rfc_bleAdvPar_t *pParams;            //!<        Pointer to command specific parameter structure
   rfc_bleAdvOutput_t *pOutput;         //!<        Pointer to command specific output structure
};

struct __attribute__ ((packed)) rfc_bleAdvPar_s {
   dataQueue_t* pRxQ;                   //!<        Pointer to receive queue
   struct {
      uint8_t bAutoFlushIgnored:1;      //!<        If 1, automatically remove ignored packets from Rx queue
      uint8_t bAutoFlushCrcErr:1;       //!<        If 1, automatically remove packets with CRC error from Rx queue
      uint8_t bAutoFlushEmpty:1;        //!<        If 1, automatically remove empty packets from Rx queue
      uint8_t bIncludeLenByte:1;        //!<        If 1, include the received length byte in the stored packet; otherwise discard it
      uint8_t bIncludeCrc:1;            //!<        If 1, include the received CRC field in the stored packet; otherwise discard it
      uint8_t bAppendRssi:1;            //!<        If 1, append an RSSI byte to the packet in the Rx queue
      uint8_t bAppendStatus:1;          //!<        If 1, append a status byte to the packet in the Rx queue
      uint8_t bAppendTimestamp:1;       //!<        If 1, append a timestamp to the packet in the Rx queue
   } rxConfig;                          //!<        Configuration bits for the receive queue entries
   struct {
      uint8_t advFilterPolicy:2;        //!< \brief The advertiser filter policy, as defined in Volume 2, Part E, Section 7.8.5 of
                                        //!<        the Bluetooth 4.0 spec
      uint8_t deviceAddrType:1;         //!<        The type of the device address &ndash; public (0) or random (1)
      uint8_t peerAddrType:1;           //!<        Directed advertiser: The type of the peer address &ndash; public (0) or random (1)
      uint8_t bStrictLenFilter:1;       //!<        1: Discard messages with illegal length
   } advConfig;
   uint8_t advLen;                      //!<        Size of advertiser data
   uint8_t scanRspLen;                  //!<        Size of scan response data
   uint8_t* pAdvData;                   //!<        Pointer to buffer containing ADV*_IND data
   uint8_t* pScanRspData;               //!<        Pointer to buffer containing SCAN_RSP data
   uint16_t* pDeviceAddress;            //!<        Pointer to device address used for this device
   rfc_bleWhiteListEntry_t *pWhiteList; //!<        Pointer to white list or peer address (directed advertiser)
   uint16_t __dummy0;
   uint8_t __dummy1;
   struct {
      uint8_t triggerType:4;            //!<        The type of trigger
      uint8_t bEnaCmd:1;                //!< \brief 0: No alternative trigger command<br>
                                        //!<        1: CMD_TRIGGER can be used as an alternative trigger
      uint8_t triggerNo:2;              //!<        The trigger number of the CMD_TRIGGER command that triggers this action
      uint8_t pastTrig:1;               //!< \brief 0: A trigger in the past is never triggered, or for start of commands, give an error<br>
                                        //!<        1: A trigger in the past is triggered as soon as possible
   } endTrigger;                        //!<        Trigger that causes the device to end the advertiser event as soon as allowed
   uint32_t endTime;                     //!< \brief Time used together with <code>endTrigger</code> that causes the device to end the
                                        //!<        advertiser event as soon as allowed
};

struct __attribute__ ((packed)) rfc_bleAdvOutput_s {
   uint16_t nTxAdvInd;                  //!<        Number of ADV*_IND packets completely transmitted
   uint8_t nTxScanRsp;                  //!<        Number of  SCAN_RSP packets transmitted
   uint8_t nRxScanReq;                  //!<        Number of SCAN_REQ packets received OK and not ignored
   uint8_t nRxConnectReq;               //!<        Number of CONNECT_REQ packets received OK and not ignored
   uint8_t __dummy0;
   uint16_t nRxNok;                     //!<        Number of packets received with CRC error
   uint16_t nRxIgnored;                 //!<        Number of packets received with CRC OK, but ignored
   uint8_t nRxBufFull;                  //!<        Number of packets received that did not fit in Rx queue
   int8_t lastRssi;                     //!<        The RSSI of the last received packet
   uint32_t timeStamp;                   //!<        Time stamp of the last received packet
};

struct __attribute__ ((packed)) rfc_bleWhiteListEntry_s {
   uint8_t size;                        //!<        Number of while list entries. Used in the first entry of the list only
   struct {
      uint8_t bEnable:1;                //!<        1 if the entry is in use, 0 if the entry is not in use
      uint8_t addrType:1;               //!<        The type address in the entry &ndash; public (0) or random (1)
      uint8_t bWlIgn:1;                 //!< \brief 1 if the entry is to be ignored by a scanner, 0 otherwise. Used to mask out
                                        //!<        entries that have already been scanned and reported.
   } conf;
   uint16_t address;                    //!<        Least significant 16 bits of the address contained in the entry
   uint32_t addressHi;                  //!<        Most significant 32 bits of the address contained in the entry
};

/**
 * @addtogroup cc26x0_rop_status_codes
 * @{
 */
#define R_OP_STATUS_IDLE                (0x0000)
#define R_OP_STATUS_PENDING             (0x0001)
#define R_OP_STATUS_ACTIVE              (0x0002)
#define R_OP_STATUS_SKIPPED             (0x0003) /* TODO value */

#define R_OP_STATUS_DONE_OK             (0x0400)
#define R_OP_STATUS_DONE_CNTDWN         (0x0401)
#define R_OP_STATUS_DONE_RXERR          (0x0402)
#define R_OP_STATUS_DONE_TIMEOUT        (0x0403)
#define R_OP_STATUS_DONE_STOPPED        (0x0404)
#define R_OP_STATUS_DONE_ABORT          (0x0405)

#define R_OP_STATUS_ERROR_PAST_START    (0x0800)
#define R_OP_STATUS_ERROR_START_TRIG    (0x0801)
#define R_OP_STATUS_ERROR_CONDITION     (0x0802)
#define R_OP_STATUS_ERROR_PAR           (0x0803)
#define R_OP_STATUS_ERROR_POINTER       (0x0804)
#define R_OP_STATUS_ERROR_CMDID         (0x0805)
#define R_OP_STATUS_ERROR_NO_SETUP      (0x0807)
#define R_OP_STATUS_ERROR_NO_FS         (0x0808)
#define R_OP_STATUS_ERROR_SYNTH_PROG    (0x0809)
#define R_OP_STATUS_ERROR_TXUNF         (0x080a)
#define R_OP_STATUS_ERROR_RXOVF         (0x080b)
#define R_OP_STATUS_ERROR_NO_RX         (0x080c)

#define R_OP_STARTTRIG_TYPE_MASK            (0xF)
#define R_OP_STARTTRIG_TYPE_TRIG_NOW        (0x0)
#define R_OP_STARTTRIG_TYPE_TRIG_NEVER      (0x1)
#define R_OP_STARTTRIG_TYPE_TRIG_ABSTIME    (0x2)
#define R_OP_STARTTRIG_TYPE_REL_SUBMIT      (0x3)
#define R_OP_STARTTRIG_TYPE_REL_START       (0x4)
#define R_OP_STARTTRIG_TYPE_REL_PREVSTART   (0x5)
#define R_OP_STARTTRIG_TYPE_REL_FIRSTSTART  (0x6)
#define R_OP_STARTTRIG_TYPE_REL_PREVEND     (0x7)
#define R_OP_STARTTRIG_TYPE_REL_EVT1        (0x8)
#define R_OP_STARTTRIG_TYPE_REL_EVT2        (0x9)
#define R_OP_STARTTRIG_TYPE_EXTERNAL        (0xA)
#define R_OP_STARTTRIG_ALT_TRIG             (0x10)
#define R_OP_STARTTRIG_TRIG_NO_MASK         (0x60)
#define R_OP_STARTTRIG_TRIG_NO_POS          (0x5)
#define R_OP_STARTTRIG_PAST_TRIG            (0x80)

/* for R_OP_STARTTRIG_TYPE_EXTERNAL startTime describes the trigger this way: */
#define R_OP_STARTTIME_INPUT_RISING         (0x0)
#define R_OP_STARTTIME_INPUT_FALLING        (0x4)
#define R_OP_STARTTIME_INPUT_BOTH           (0x8)
#define R_OP_STARTTIME_SOURCE_GPI0          (0x1200) /* TODO wtf does this mean for a 5-bit field? */
#define R_OP_STARTTIME_SOURCE_GPI1          (0x1300) /* 22: RFC_GPI0, 23: RFC_GPI1 assuming octal - above is binary... */

#define R_OP_CONDITION_RULE_MASK            (0x0F)
#define R_OP_CONDITION_RULE_ALWAYS          (0x00)
#define R_OP_CONDITION_RULE_NEVER           (0x01)
#define R_OP_CONDITION_RULE_STOP_ON_FALSE   (0x02)
#define R_OP_CONDITION_RULE_STOP_ON_TRUE    (0x03)
#define R_OP_CONDITION_RULE_SKIP_ON_FALSE   (0x04)
#define R_OP_CONDITION_RULE_SKIP_ON_TRUE    (0x05)
#define R_OP_CONDITION_NSKIP_POS            (0x4)
#define R_OP_CONDITION_NSKIP_MASK           (0xF0)
/**@}*/


typedef struct __attribute__ ((aligned(4))) {
    radio_op_command_t ropCmd;
} nop_cmd_t;


typedef struct __attribute__ ((aligned(4))) {
    radio_op_command_t ropCmd;
    uint8_t mode; /* W */
    /*
     * supported values: 0 (equivalent to 2), 2, 5, 6, 10, 12, 15, and 30
     * 0 or 2 only supported for devices that support 2.4-GHz operation
     */
    uint8_t IoDivider; /* W */
    struct {
        uint16_t frontEndMode:3;
        uint16_t biasMode:1;
        uint16_t :6;
        uint16_t bNoFsPowerUp:1;
    } config; /* W */
    struct {
        uint16_t IB:6;
        uint16_t GC:2;
        uint16_t boost:1;
        uint16_t tempCoeff:7;
    } txPower; /* W */
    uint32_t *pRegOverride; /* W */
} radio_setup_cmd_t;

#define RADIO_SETUP_MODE_BLE                (0x0)
#define RADIO_SETUP_MODE_IEEE8021504        (0x1)
#define RADIO_SETUP_MODE_2MBPS_GFSK         (0x2)
#define RADIO_SETUP_MODE_5MBPS_CODED_8FSK   (0x5)
#define RADIO_SETUP_MODE_KEEP               (0xFF) // update overrides only

/*
 * http://processors.wiki.ti.com/images/4/45/CC26xx_HW_training_RF_Frontends_and_Antennas.pdf
*/
#define RADIO_SETUP_CONFIG_FEM_DIFF             (0x0) /* front-end mode */
#define RADIO_SETUP_CONFIG_FEM_SINGLE_END_RFP   (0x1)
#define RADIO_SETUP_CONFIG_FEM_SINGLE_END_RFN   (0x2)
#define RADIO_SETUP_CONFIG_FEM_START_RFP        (0x3)
#define RADIO_SETUP_CONFIG_FEM_START_RFN        (0x4)
#define RADIO_SETUP_CONFIG_FEM_SINGLE_END_RFP_E (0x5) /* ext front end control on rf pins */
#define RADIO_SETUP_CONFIG_FEM_SINGLE_END_RFN_E (0x6)




typedef struct {
    uint32_t pCurrEntry; /* R/W NULL -> empty*/
    uint32_t pLastEntry; /* R/W */
} data_entry_queue_t;

typedef struct {
    uint32_t pNextEntry; /* R/W */
    uint8_t status; /* R/W */
    uint8_t config; /* W */
    uint16_t length; /* W TODO find out what the difference between this and the value referenced by config.lensz is */
    /* maybe you can think of this struct being extended like this:
     * DATA_ENTRY_CONFIG_TYPE_GENERAL:
     * uint8_t data[length];
     * DATA_ENTRY_CONFIG_TYPE_MULTIRX:
     * uint16_t dataCnt; // R
     * uint16_t nextIndex; // R
     * uint16_t data[length]; // R
     * DATA_ENTRY_CONFIG_TYPE_PTR:
     * uint32_t pData;
     */
} data_entry_t;

typedef struct {
	uint16_t commandNo;
	uint16_t __reserved;
	uint32_t pQueue;
	uint32_t pEntry;
} data_queue_t;


#define DATA_ENTRY_STATUS_PENDING       (0x0)
#define DATA_ENTRY_STATUS_ACTIVE        (0x1)
#define DATA_ENTRY_STATUS_BUSY          (0x2)
#define DATA_ENTRY_STATUS_FINISHED      (0x3)

#define DATA_ENTRY_CONFIG_TYPE_MASK     (0x3)
#define DATA_ENTRY_CONFIG_TYPE_GENERAL  (0x0)
#define DATA_ENTRY_CONFIG_TYPE_MULTIRX  (0x1)
#define DATA_ENTRY_CONFIG_TYPE_PTR      (0x2)
#define DATA_ENTRY_CONFIG_LENSZ_POS     (0x4)
#define DATA_ENTRY_CONFIG_LENSZ_MASK    (0xC)
#define DATA_ENTRY_CONFIG_LENSZ_NOLEN   (0x0)
#define DATA_ENTRY_CONFIG_LENSZ_1BYTE   (0x1)
#define DATA_ENTRY_CONFIG_LENSZ_2BYTE   (0x2)





#include "cc26x0_rfc_ble.h"

#ifdef __cplusplus
}
#endif

#endif /* CC26x0_RFC_H */

/*@}*/
