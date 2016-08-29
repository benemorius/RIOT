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
 * @brief           CC26x0 RFC IEEE definitions
 *
 * @author          Leon George <leon@georgemail.eu>
 * @author          Florent-Valéry Coen <florent.coen@gmail.com>
 */

#ifndef CC26x0_RFC_IEEE_H
#define CC26x0_RFC_IEEE_H

#include "cc26x0_rfc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup cc2x0_rop_command_ids
 * @{
 */
/* background */
#define CMDR_CMDID_IEEE_RX              (0x2801)
#define CMDR_CMDID_IEEE_ED_SCAN         (0x2802)
/* foreground */
#define CMDR_CMDID_IEEE_TX              (0x2C01)
#define CMDR_CMDID_IEEE_CSMA            (0x2C02)
#define CMDR_CMDID_IEEE_RX_ACK          (0x2C03)
#define CMDR_CMDID_IEEE_ABORT_BG        (0x2C04)
/* immediate */
#define CMDR_CMDID_IEEE_MOD_CCA         (0x2001)
#define CMDR_CMDID_IEEE_MOD_FILT        (0x2002)
#define CMDR_CMDID_IEEE_MOD_SRC_MATCH   (0x2003)
#define CMDR_CMDID_IEEE_ABORT_FG        (0x2401)
#define CMDR_CMDID_IEEE_STOP_FG         (0x2402)
#define CMDR_CMDID_IEEE_CCA_REQ         (0x2403)
/**@}*/

typedef struct rfc_CMD_IEEE_TX_s rfc_CMD_IEEE_TX_t;
typedef struct rfc_CMD_IEEE_RX_s rfc_CMD_IEEE_RX_t;
typedef struct rfc_ieeeRxOutput_s rfc_ieeeRxOutput_t;
typedef struct rfc_shortAddrEntry_s rfc_shortAddrEntry_t;

struct __attribute__ ((packed)) rfc_CMD_IEEE_TX_s {
   uint16_t commandNo;                  //!<        The command ID number 0x2C01
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
   struct {
      uint8_t bIncludePhyHdr:1;         //!< \brief 0: Find PHY header automatically<br>
                                        //!<        1: Insert PHY header from the buffer
      uint8_t bIncludeCrc:1;            //!< \brief 0: Append automatically calculated CRC<br>
                                        //!<        1: Insert FCS (CRC) from the buffer
      uint8_t :1;
      uint8_t payloadLenMsb:5;          //!< \brief Most significant bits of payload length. Should only be non-zero to create long
                                        //!<        non-standard packets for test purposes
   } txOpt;
   uint8_t payloadLen;                  //!<        Number of bytes in the payload
   uint8_t* pPayload;                   //!<        Pointer to payload buffer of size <code>payloadLen</code>
   uint32_t timeStamp;                  //!<        Time stamp of transmitted frame
};

struct __attribute__ ((packed)) rfc_CMD_IEEE_RX_s {
   uint16_t commandNo;                  //!<        The command ID number 0x2801
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
   uint8_t channel;                     //!< \brief Channel to tune to in the start of the operation<br>
                                        //!<        0: Use existing channel<br>
                                        //!<        11&ndash;26: Use as IEEE 802.15.4 channel, i.e. frequency is (2405 + 5 &times; (channel - 11)) MHz<br>
                                        //!<        60&ndash;207: Frequency is  (2300 + channel) MHz<br>
                                        //!<        Others: <i>Reserved</i>
   struct {
      uint8_t bAutoFlushCrc:1;          //!<        If 1, automatically remove packets with CRC error from Rx queue
      uint8_t bAutoFlushIgn:1;          //!<        If 1, automatically remove packets that can be ignored according to frame filtering from Rx queue
      uint8_t bIncludePhyHdr:1;         //!<        If 1, include the received PHY header field in the stored packet; otherwise discard it
      uint8_t bIncludeCrc:1;            //!<        If 1, include the received CRC field in the stored packet; otherwise discard it
      uint8_t bAppendRssi:1;            //!<        If 1, append an RSSI byte to the packet in the Rx queue
      uint8_t bAppendCorrCrc:1;         //!<        If 1, append a correlation value and CRC result byte to the packet in the Rx queue
      uint8_t bAppendSrcInd:1;          //!<        If 1, append an index from the source matching algorithm
      uint8_t bAppendTimestamp:1;       //!<        If 1, append a timestamp to the packet in the Rx queue
   } rxConfig;
   dataQueue_t* pRxQ;                   //!<        Pointer to receive queue
   rfc_ieeeRxOutput_t *pOutput;         //!<        Pointer to output structure (NULL: Do not store results)
   struct {
      uint16_t frameFiltEn:1;           //!< \brief 0: Disable frame filtering<br>
                                        //!<        1: Enable frame filtering
      uint16_t frameFiltStop:1;         //!< \brief 0: Receive all packets to the end<br>
                                        //!<        1: Stop receiving frame once frame filtering has caused the frame to be rejected.
      uint16_t autoAckEn:1;             //!< \brief 0: Disable auto ACK<br>
                                        //!<        1: Enable auto ACK.
      uint16_t slottedAckEn:1;          //!< \brief 0: Non-slotted ACK<br>
                                        //!<        1: Slotted ACK.
      uint16_t autoPendEn:1;            //!< \brief 0: Auto-pend disabled<br>
                                        //!<        1: Auto-pend enabled
      uint16_t defaultPend:1;           //!<        The value of the pending data bit in auto ACK packets that are not subject to auto-pend
      uint16_t bPendDataReqOnly:1;      //!< \brief 0: Use auto-pend for any packet<br>
                                        //!<        1: Use auto-pend for data request packets only
      uint16_t bPanCoord:1;             //!< \brief 0: Device is not PAN coordinator<br>
                                        //!<        1: Device is PAN coordinator
      uint16_t maxFrameVersion:2;       //!<        Reject frames where the frame version field in the FCF is greater than this value
      uint16_t fcfReservedMask:3;       //!<        Value to be AND-ed with the reserved part of the FCF; frame rejected if result is non-zero
      uint16_t modifyFtFilter:2;        //!< \brief Treatment of MSB of frame type field before frame-type filtering:<br>
                                        //!<        0: No modification<br>
                                        //!<        1: Invert MSB<br>
                                        //!<        2: Set MSB to 0<br>
                                        //!<        3: Set MSB to 1
      uint16_t bStrictLenFilter:1;      //!< \brief 0: Accept acknowledgement frames of any length >= 5<br>
                                        //!<        1: Accept only acknowledgement frames of length 5
   } frameFiltOpt;                      //!<        Frame filtering options
   struct {
      uint8_t bAcceptFt0Beacon:1;       //!< \brief Treatment of frames with frame type 000 (beacon):<br>
                                        //!<        0: Reject<br>
                                        //!<        1: Accept
      uint8_t bAcceptFt1Data:1;         //!< \brief Treatment of frames with frame type 001 (data):<br>
                                        //!<        0: Reject<br>
                                        //!<        1: Accept
      uint8_t bAcceptFt2Ack:1;          //!< \brief Treatment of frames with frame type 010 (ACK):<br>
                                        //!<        0: Reject, unless running ACK receive command<br>
                                        //!<        1: Always accept
      uint8_t bAcceptFt3MacCmd:1;       //!< \brief Treatment of frames with frame type 011 (MAC command):<br>
                                        //!<        0: Reject<br>
                                        //!<        1: Accept
      uint8_t bAcceptFt4Reserved:1;     //!< \brief Treatment of frames with frame type 100 (reserved):<br>
                                        //!<        0: Reject<br>
                                        //!<        1: Accept
      uint8_t bAcceptFt5Reserved:1;     //!< \brief Treatment of frames with frame type 101 (reserved):<br>
                                        //!<        0: Reject<br>
                                        //!<        1: Accept
      uint8_t bAcceptFt6Reserved:1;     //!< \brief Treatment of frames with frame type 110 (reserved):<br>
                                        //!<        0: Reject<br>
                                        //!<        1: Accept
      uint8_t bAcceptFt7Reserved:1;     //!< \brief Treatment of frames with frame type 111 (reserved):<br>
                                        //!<        0: Reject<br>
                                        //!<        1: Accept
   } frameTypes;                        //!<        Frame types to receive in frame filtering
   struct {
      uint8_t ccaEnEnergy:1;            //!<        Enable energy scan as CCA source
      uint8_t ccaEnCorr:1;              //!<        Enable correlator based carrier sense as CCA source
      uint8_t ccaEnSync:1;              //!<        Enable sync found based carrier sense as CCA source
      uint8_t ccaCorrOp:1;              //!< \brief Operator to use between energy based and correlator based CCA<br>
                                        //!<        0: Report busy channel if either ccaEnergy or ccaCorr are busy<br>
                                        //!<        1: Report busy channel if both ccaEnergy and ccaCorr are busy
      uint8_t ccaSyncOp:1;              //!< \brief Operator to use between sync found based CCA and the others<br>
                                        //!<        0: Always report busy channel if ccaSync is busy<br>
                                        //!<        1: Always report idle channel if ccaSync is idle
      uint8_t ccaCorrThr:2;             //!<        Threshold for number of correlation peaks in correlator based carrier sense
   } ccaOpt;                            //!<        CCA options
   int8_t ccaRssiThr;                   //!<        RSSI threshold for CCA
   uint8_t __dummy0;
   uint8_t numExtEntries;               //!<        Number of extended address entries
   uint8_t numShortEntries;             //!<        Number of short address entries
   uint32_t* pExtEntryList;             //!<        Pointer to list of extended address entries
   rfc_shortAddrEntry_t *pShortEntryList;//!<        Pointer to list of short address entries
   uint64_t localExtAddr;               //!<        The extended address of the local device
   uint16_t localShortAddr;             //!<        The short address of the local device
   uint16_t localPanID;                 //!<        The PAN ID of the local device
   uint16_t __dummy1;
   uint8_t __dummy2;
   struct {
      uint8_t triggerType:4;            //!<        The type of trigger
      uint8_t bEnaCmd:1;                //!< \brief 0: No alternative trigger command<br>
                                        //!<        1: CMD_TRIGGER can be used as an alternative trigger
      uint8_t triggerNo:2;              //!<        The trigger number of the CMD_TRIGGER command that triggers this action
      uint8_t pastTrig:1;               //!< \brief 0: A trigger in the past is never triggered, or for start of commands, give an error<br>
                                        //!<        1: A trigger in the past is triggered as soon as possible
   } endTrigger;                        //!<        Trigger that causes the device to end the Rx operation
   uint32_t endTime;                    //!< \brief Time used together with <code>endTrigger</code> that causes the device to end the Rx
                                        //!<        operation
};

struct __attribute__ ((packed)) rfc_ieeeRxOutput_s {
   uint8_t nTxAck;                      //!<        Total number of transmitted ACK frames
   uint8_t nRxBeacon;                   //!<        Number of received beacon frames
   uint8_t nRxData;                     //!<        Number of received data frames
   uint8_t nRxAck;                      //!<        Number of received acknowledgement frames
   uint8_t nRxMacCmd;                   //!<        Number of received MAC command frames
   uint8_t nRxReserved;                 //!<        Number of received frames with reserved frame type
   uint8_t nRxNok;                      //!<        Number of received frames with CRC error
   uint8_t nRxIgnored;                  //!<        Number of frames received that are to be ignored
   uint8_t nRxBufFull;                  //!<        Number of received frames discarded because the Rx buffer was full
   int8_t lastRssi;                     //!<        RSSI of last received frame
   int8_t maxRssi;                      //!<        Highest RSSI observed in the operation
   uint8_t __dummy0;
   uint32_t beaconTimeStamp;            //!<        Time stamp of last received beacon frame
};

struct __attribute__ ((packed)) rfc_shortAddrEntry_s {
   uint16_t shortAddr;                  //!<        Short address
   uint16_t panId;                      //!<        PAN ID
};

#ifdef __cplusplus
}
#endif

#endif /* CC26x0_RFC_IEEE_H */

/*@}*/
