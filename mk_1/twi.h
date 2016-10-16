/*
 * file: twi.h
 * created: 20160906
 * author(s): mr-augustine
 *
 * These are the Two-Wire Interface (TWI) bit mask definitions
 * They were copied from the following site:
 * http://www.nongnu.org/avr-libc/user-manual/group__util__twi.html
 *
 * The mnemonics are defined as follows:
 * TW_MT_xxx: Master Transmitter
 * TW_MR_xxx: Master Receiver
 * TW_ST_xxx: Slave Transmitter
 * TW_SR_xxx: Slave Receiver
 *
 * SLA: Slave Address
 * Comments are appended to the mask definitions we use
 */
#ifndef _TWI_H_
#define _TWI_H_

#define TW_START                   0x08 // Start condition transmitted
#define TW_REP_START               0x10 // Repeated Start condition transmitted
#define TW_MT_SLA_ACK              0x18 // SLA+W transmitted, ACK received
#define TW_MT_SLA_NACK             0x20
#define TW_MT_DATA_ACK             0x28 // Data transmitted, ACK received
#define TW_MT_DATA_NACK            0x30
#define TW_MT_ARB_LOST             0x38
#define TW_MR_ARB_LOST             0x38
#define TW_MR_SLA_ACK              0x40 // SLA+R transmitted, ACK received
#define TW_MR_SLA_NACK             0x48
#define TW_MR_DATA_ACK             0x50 // Data received, ACK returned
#define TW_MR_DATA_NACK            0x58 // Data received, NACK returned
#define TW_ST_SLA_ACK              0xA8
#define TW_ST_ARB_LOST_SLA_ACK     0xB0
#define TW_ST_DATA_ACK             0xB8
#define TW_ST_DATA_NACK            0xC0
#define TW_ST_LAST_DATA            0xC8
#define TW_SR_SLA_ACK              0x60
#define TW_SR_ARB_LOST_SLA_ACK     0x68
#define TW_SR_GCALL_ACK            0x70
#define TW_SR_ARB_LOST_GCALL_ACK   0x78
#define TW_SR_DATA_ACK             0x80
#define TW_SR_DATA_NACK            0x88
#define TW_SR_GCALL_DATA_ACK       0x90
#define TW_SR_GCALL_DATA_NACK      0x98
#define TW_SR_STOP                 0xA0
#define TW_NO_INFO                 0xF8
#define TW_BUS_ERROR               0x00
#define TW_STATUS                  (TWSR & TW_NO_INFO) // Grab the status bits
#define TW_READ                    1    // Read-mode flag
#define TW_WRITE                   0    // Write-mode flag

#endif // #ifndef _TWI_H_
