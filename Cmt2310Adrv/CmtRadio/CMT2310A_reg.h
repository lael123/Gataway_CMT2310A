/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-29     Tobby       the first version
 */
#ifndef RADIOCMT2310_CMT2310A_REGISTER_H_
#define RADIOCMT2310_CMT2310A_REGISTER_H_

/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, CMOSTEK SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Copyright (C) CMOSTEK SZ.
 */

/*!
 * @file    CMT2310A_reg.h
 * @brief   CMT2310A transceiver RF chip driver
 *
 * @version 1.0
 * @date    Dec 7 2021
 * @author  CMOSTEK R&D
 */

#ifndef __CMT2310A_REG_H

#define 	__CMT2310A_REG_H

#define 		CMT2310A_PAGE0_SIZE         (0x77-0x28+1)
#define 		CMT2310A_PAGE1_SIZE         (0xef-0x80+1)//�ܹ�0X6F+1 ���������Ӧ����0X68��
#define 		CMT2310A_PAGE2_SIZE         (0x3f-0x00+1)

//---------------------- CUS PAGE0 defines -------------------------------
//------------------------------------------------------------------------	
#define 		CMT2310A_CTL_REG_00         	0x00					// PU_BOOT<7:0>
#define		CMT2310A_REBOOT					0x03

#define 		CMT2310A_CTL_REG_01         	0x01					// radio chip status switch
#define		CMT2310A_GO_SLEEP				(1<<0)					// radio go sleep
#define		CMT2310A_GO_READY				(1<<1)					// radio go ready
#define		CMT2310A_GO_TX					(1<<2)					// radio go tx
#define		CMT2310A_GO_RX					(1<<3)					// radio go rx
#define		CMT2310A_GO_TFS					(1<<4)					// radio go tfs
#define		CMT2310A_GO_RFS					(1<<5)					// radio go rfs

#define 		CMT2310A_CTL_REG_02        0x02					// Antenna Diversity
#define		CMT2310A_ANT_DIV_MANU			(1<<1)					// 0=disable antenna diversity,  1=enable antenna diversity
#define		CMT2310A_ANT_SELECT				(1<<0)					// 0=select antenna_1,           1=select antenna_2,  when antenna diversity manual active

#define 		CMT2310A_CTL_REG_03        0x03					// frequencry channel value, by manual set

#define 		CMT2310A_CTL_REG_04        0x04					// gpio ctrl 0
#define		CMT2310A_TX_DIN_EN				(1<<6)					// 0=disable TX_DATA input to GPIO,  1=enable TX_DATA input to GPIO

#define		CMT2310A_GPIO1_SEL				(7<<3)					// masker
#define		CMT2310A_GPIO1_DCLK				(0<<3)					// gpio1 as DCLK
#define		CMT2310A_GPIO1_INT1				(1<<3)					// gpio1 as INT1
#define		CMT2310A_GPIO1_INT2				(2<<3)					// gpio1 as INT2
#define		CMT2310A_GPIO1_DOUT				(3<<3)					// goio1 as DOUT

#define		CMT2310A_GPIO0_SEL				(7<<0)					// masker
#define		CMT2310A_GPIO0_DOUT				(0<<0)					// gpio0 as DOUT
#define		CMT2310A_GPIO0_INT1				(1<<0)					// gpio0 as INT1
#define		CMT2310A_GPIO0_INT2				(2<<0)					// gpio0 as INT2
#define		CMT2310A_GPIO0_DCLK				(3<<0)					// goio0 as DCLK
#define		CMT2310A_GPIO0_INT3				(6<<0)					// gpio0 as INT3

#define 		CMT2310A_CTL_REG_05         	0x05					// gpio ctrl 1
#define		CMT2310A_TX_DIN_SEL				(3<<6)					// masker
#define		CMT2310A_TX_DIN_GPIO3			(0<<6)					// TX_DIN from GPIO3
#define		CMT2310A_TX_DIN_GPIO4			(1<<6)					// TX_DIN from GPIO4
#define		CMT2310A_TX_DIN_nIRQ			(2<<6)					// TX_DIN from nIRQ

#define		CMT2310A_GPIO3_SEL				(7<<3)					//masker
#define		CMT2310A_GPIO3_INT2				(0<<3)					// gpio3 as INT2
#define		CMT2310A_GPIO3_INT1				(1<<3)					// gpio3 as INT1
#define		CMT2310A_GPIO3_DCLK				(2<<3)					// gpio3 as DCLK
#define		CMT2310A_GPIO3_DOUT				(3<<3)					// gpio3 as DOUT
#define		CMT2310A_GPIO3_DIN				(5<<3)					// gpio3 as DIN (TX_DIN)

#define		CMT2310A_GPIO2_SEL				(7<<0)					//masker
#define		CMT2310A_GPIO2_INT1				(0<<0)					// gpio2 as INT1
#define		CMT2310A_GPIO2_INT2				(1<<0)					// gpio2 as INT2
#define		CMT2310A_GPIO2_DCLK				(2<<0)					// gpio2 as DCLK
#define		CMT2310A_GPIO2_DOUT				(3<<0)					// gpio2 as DOUT
#define		CMT2310A_GPIO2_INT3				(6<<0)					// gpio2 as INT3

#define 		CMT2310A_CTL_REG_06         	0x06					// gpio ctrl 2
#define		CMT2310A_DIG_CLKOUT_EN			(1<<6)					// 0=bypass,  1=enable digital clock out to GPIO4

#define		CMT2310A_GPIO5_SEL				(7<<3)					//masker
#define		CMT2310A_GPIO5_nRST				(0<<3)					// gpio5 as nRST
#define		CMT2310A_GPIO5_INT1				(1<<3)					// goio5 as INT1
#define		CMT2310A_GPIO5_INT2				(2<<3)					// gpio5 as INT2
#define		CMT2310A_GPIO5_DOUT				(3<<3)					// gpio5 as DOUT
#define		CMT2310A_GPIO5_DCLK				(4<<3)					// gpio5 as DCLK

#define		CMT2310A_GPIO4_SEL				(7<<0)					//masker
#define		CMT2310A_GPIO4_DOUT				(0<<0)					// gpio4 as DOUT
#define		CMT2310A_GPIO4_INT1				(1<<0)					// gpio4 as INT1
#define		CMT2310A_GPIO4_INT2				(2<<0)					// gpio4 as INT2
#define		CMT2310A_GPIO4_DCLK				(3<<0)					// gpio4 as DCLK
#define		CMT2310A_GPIO4_DIN				(5<<0)					// gpio4 as DIN (TX_DIN)

#define 		CMT2310A_CTL_REG_07        		0x07					// nIRQ select
#define		CMT2310A_CTL_REG_07_MASK		(3<<6)
#define		CMT2310A_LFXO_PAD_EN			(1<<5)					// 0=disable,       1=enable GPIO2 & GPIO3 as 32.768kHz crystal pin
#define		CMT2310A_API_STOP				(1<<4)					// 0=API go on,     1=stop API
#define		CMT2310A_SPI_3W_EN				(1<<3)					// 0=select SPI-4,  1=select SPI-3

#define		CMT2310A_nIRQ_SEL				  (7<<0)					//masker
#define		CMT2310A_nIRQ_INT1				(0<<0)					// nIRQ as INT1
#define		CMT2310A_nIRQ_INT2				(1<<0)					// nIRQ as INT2
#define		CMT2310A_nIRQ_DCLK				(2<<0)					// nIRQ as DCLK
#define		CMT2310A_nIRQ_DOUT				(3<<0)					// nIRQ as DOUT
#define		CMT2310A_nIRQ_TCXO				(4<<0)					// nIRQ as TCXO control pin
#define		CMT2310A_nIRQ_DIN				(5<<0)					// nIRQ as DIN (TX_DIN)

#define 		CMT2310A_CTL_REG_08        		0x08					//API Command interface
#define		CMT2310A_API_CMD_REG			0x08

#define 		CMT2310A_CTL_REG_09        		0x09
#define		CMT2310A_API_CMD_FLAG			(1<<7)					//API Command flag
#define		CMT2310A_API_RESP_MASK			0x7F					//API Respond value

#define 		CMT2310A_CTL_REG_10         	0x0A					// radio status register 	(-RO)
#define		CMT2310A_CHIP_MODE_STA_REG		0x0A					//
#define		CMT2310A_STATE_IS_IDLE			0x00					// radio is in idle
#define		CMT2310A_STATE_IS_SLEEP			0x81					// radio is in sleep
#define		CMT2310A_STATE_IS_READY			0x82					// radio is in ready
#define		CMT2310A_STATE_IS_RFS			  0x84					// radio is in RFS
#define		CMT2310A_STATE_IS_TFS			  0x88					// radio is in TFS
#define		CMT2310A_STATE_IS_RX			  0x90					// radio is in RX
#define		CMT2310A_STATE_IS_TX			  0xA0					// radio is in TX

#define 		CMT2310A_CTL_REG_11         	0x0B					// auto frequencry hopping done times
// range 0-63

#define 		CMT2310A_CTL_REG_12         	0x0C					// auto frequencry hopping channel space
#define		  CMT2310A_FREQ_SPACE_REG		  	0x0C

#define 		CMT2310A_CTL_REG_13         	0x0D					// auto frequencry hopping times
#define		  CMT2310A_FREQ_TIMES_REG			  0x0D					// range 1-64(n+1)

#define 		CMT2310A_CTL_REG_14         	0x0E					// fifo status interrupt control register
#define		CMT2310A_RX_FIFO_FULL_EN		 (1<<7)					// 0=disable,  1=enable rx fifo full interrupt
#define		CMT2310A_RX_FIFO_NMTY_EN		 (1<<6)					// 0=disable,  1=enable rx fifo non-empty interrupt
#define		CMT2310A_RX_FIFO_TH_EN			 (1<<5)					// 0=disable,  1=enable rx fifo threshold interrupt
#define		CMT2310A_RX_FIFO_OVF_EN			 (1<<3)					// 0=disable,  1=enable rx fifo overflow interrupt
#define		CMT2310A_TX_FIFO_FULL_EN		 (1<<2)					// 0=disable,  1=enable tx fifo full interrupt
#define		CMT2310A_TX_FIFO_NMTY_EN		 (1<<1)					// 0=disable,  1=enable tx fifo non-empty interrupt
#define		CMT2310A_TX_FIFO_TH_EN			 (1<<0)					// 0=disable,  1=enable tx fifo threshold interrupt

#define 		CMT2310A_CTL_REG_15         	0x0F					// antenna diversity indicate
#define		CMT2310A_ANT_INSTR				   (1<<0)					// indicate antenna diversity lock which antenna

#define 		CMT2310A_CTL_REG_16         	0x10					// interrupt1 selection
#define		CMT2310A_INT1_SEL_REG			    0x10
#define		CMT2310A_INT1_SEL_MASK			  0x3F

#define 		CMT2310A_CTL_REG_17             0x11					// interrupt2 selection
#define		CMT2310A_INT2_SEL_REG			0x11
#define		CMT2310A_INT1_POLAR				(1<<7)					// int1 polar selection, 0=high acitve,  1=low acitve;
#define		CMT2310A_INT2_POLAR				(1<<6)					// int1 polar selection, 0=high acitve,  1=low acitve;
#define		CMT2310A_INT2_SEL_MASK			0x3F

#define		CMT2310A_INT_MIX				(0<<0)					// all interrupt mixed together
#define		CMT2310A_INT_ANT_LOCK			(1<<0)					// antenna locked
#define		CMT2310A_INT_RSSI_PJD_VALID		(2<<0)					// rssi and/or pjd valid
#define		CMT2310A_INT_PREAM_PASS			(3<<0)					// preamble detected
#define		CMT2310A_INT_SYNC_PASS			(4<<0)					// sync word detected
#define		CMT2310A_INT_ADDR_PASS			(5<<0)					// node address detected
#define		CMT2310A_INT_CRC_PASS			  (6<<0)					// crc ok detected
#define		CMT2310A_INT_PKT_OK				  (7<<0)					// packet received detected
#define		CMT2310A_INT_PKT_DONE			  (8<<0)					// packet received detected, even wrong packet or collision
#define		CMT2310A_INT_SLEEP_TMO			(9<<0)					// sleep timer time-out
#define		CMT2310A_INT_RX_TMO				  (10<<0)					// rx timer time-out
#define		CMT2310A_INT_RX_FIFO_NMTY		  (11<<0)					// rx fifo non-empty
#define		CMT2310A_INT_RX_FIFO_TH			  (12<<0)					// rx fifo threshold
#define		CMT2310A_INT_RX_FIFO_FULL		  (13<<0)					// rx fifo full
#define		CMT2310A_INT_RX_FIFO_WBYTE	  (14<<0)					// rx fifo write byte trigger
#define		CMT2310A_INT_RX_FIFO_OVF		  (15<<0)					// rx fifo overflow
#define		CMT2310A_INT_TX_DONE			    (16<<0)					// tx done
#define		CMT2310A_INT_TX_FIFO_NMTY		  (17<<0)					// tx fifo non-empty
#define		CMT2310A_INT_TX_FIFO_TH			  (18<<0)					// tx fifo threshold
#define		CMT2310A_INT_TX_FIFO_FULL		  (19<<0)					// tx fifo full
#define		CMT2310A_INT_STATE_IS_READY	  (20<<0)					// state is ready
#define		CMT2310A_INT_STATE_IS_FS		  (21<<0)					// state is FS
#define		CMT2310A_INT_STATE_IS_RX		  (22<<0)					// state is rx
#define		CMT2310A_INT_STATE_IS_TX		  (23<<0)					// state is tx
#define		CMT2310A_INT_LBD_STATUS			  (24<<0)					// LBD status
#define		CMT2310A_INT_API_CMD_FAILED	  (25<<0)					// API Command failed
#define		CMT2310A_INT_API_DONE			    (26<<0)					// API execute finish
#define		CMT2310A_INT_TX_DC_DONE			  (27<<0)					// ??
#define		CMT2310A_INT_ACK_RECV_FAILED	(28<<0)					// ack recieve failed
#define		CMT2310A_INT_TX_RESEND_DONE		(29<<0)					// tx re-send done
#define		CMT2310A_INT_NACK_RECV			  (30<<0)					// non-ack received
#define		CMT2310A_INT_SEQ_MATCH			  (31<<0)					// sequence number match
#define		CMT2310A_INT_CSMA_DONE			  (32<<0)					// CSMA done
#define		CMT2310A_INT_CCA_STATUS			  (33<<0)					// CCA status match

#define 		CMT2310A_CTL_REG_18             0x12					// interrupt enable control 1
#define		CMT2310A_INT_CTL1_REG			0x12
#define		CMT2310A_SLEEP_TMO_EN			(1<<7)					// 0=disable, 1=enable sleep timer time-out
#define		CMT2310A_RX_TMO_EN				(1<<6)					// 0=disable, 1=enable rx timer time-out
#define		CMT2310A_TX_DONE_EN				(1<<5)					// 0=disable, 1=enable tx done
#define		CMT2310A_PREAM_PASS_EN			(1<<4)					// 0=disable, 1=enable preamble detect
#define		CMT2310A_SYNC_PASS_EN			(1<<3)					// 0=disable, 1=enable syncword detect
#define		CMT2310A_ADDR_PASS_EN			(1<<2)					// 0=disable, 1=enable node address detect
#define		CMT2310A_CRC_PASS_EN			(1<<1)					// 0=disable, 1=enable packet crc detect
#define		CMT2310A_PKT_DONE_EN			(1<<0)					// 0=disable, 1=enable packet received done

#define 		CMT2310A_CTL_REG_19             0x13					// interrupt enable control 2
#define		CMT2310A_INT3_POLAR				(1<<7)					// int1 polar selection, 0=high acitve,  1=low acitve;
#define		CMT2310A_PD_FIFO				(1<<6)					// 0=retain fifo in sleep,   1=not retain fifo in sleep
#define		CMT2310A_FIFO_TH_BIT8			(1<<5)					// fifo threshold bit8
#define		CMT2310A_FIFO_AUTO_CLR_RX_EN	(1<<4)					// 0=not clear fifo,  1=clear fifo, when trigger go to rx
#define		CMT2310A_FIFO_AUTO_RES_TX_EN	(1<<3)					// 0=auto restore tx fifo,    1=don't auto restore tx fifo, when tx done. (if TX_PKT_NUM is not 0, should be set this bit)
#define		CMT2310A_FIFO_TX_TEST_EN		(1<<2)					// 0=tx fifo only write,      1=tx fifo can be read.  suggest only for testing, normal useage should be clear this bit
#define		CMT2310A_FIFO_MERGE_EN			(1<<1)					// 0=split fifo to tx and rx, 1=fifo merge together
#define		CMT2310A_FIFO_TX_RX_SEL			(1<<0)					// 0=fifo for tx useage, 	  1=fifo for rx useage, when CMT2310A_FIFO_MERGE_EN=1

#define 		CMT2310A_CTL_REG_20             0x14					// fifo threshold value
#define		CMT2310A_FIFO_TH_REG			0x14

#define 		CMT2310A_CTL_REG_21             0x15					// interrupt enable control 3
#define		CMT2310A_RSSI_PJD_VALID_EN		(1<<6)					// 0=disable, 1=enable RSSI and/or PJD valid interrupt
#define		CMT2310A_OP_CMD_FAILED_EN		(1<<5)					// 0=disable, 1=enable API Command failed interrupt
#define		CMT2310A_RSSI_COLL_EN			(1<<4)					// 0=disable, 1=enable signal collision interrupt
#define		CMT2310A_PKT_ERR_EN				(1<<3)					// 0=disable, 1=enable packet recieve error interrupt
#define		CMT2310A_LBD_STATUS_EN			(1<<2)					// 0=disable, 1=enable LBD status interrupt
#define		CMT2310A_LBD_STOP_EN			(1<<1)					// 0=disable, 1=enable LBD stop interrupt
#define		CMT2310A_LD_STOP_EN				(1<<0)					// 0=disable, 1=enable PLL lock detect interrupt

#define 	CMT2310A_CTL_REG_22             0x16			// function enable config 0
#define		CMT2310A_FREQ_HOP_MANU_EN		(1<<7)				// 0=disable, 1=enable manual frequencry hopping
#define		CMT2310A_RX_HOP_PERSIST			(1<<6)				// 0=finish,  1=on going, Rx auto frequencry hopping
#define		CMT2310A_FREQ_SW_STATE			(1<<5)				// 0=return to READY,  1=return to RFS,  for Rx auto frequencry hopping mode, every times after RX time-out state
#define		CMT2310A_TX_DATA_INV			(1<<4)					// 0=don't invert,     1=invert, for TX data input from GPIO
#define		CMT2310A_PA_DIFF_SEL			(1<<3)					// 0=single-end PA,    1=differential-end PA,  for CMT2310A need select single-end PA
#define		CMT2310A_TRX_SWT_INV			(1<<2)					// 0=don't invert,     1=invert for TX/RX antenna switch
#define		CMT2310A_TRX_SWT_EN				(1<<1)					// 0=disable, 1=enable TX/RX antenna switch
#define		CMT2310A_ANT_LOCK_EN			(1<<0)					// 0=disable, 1=enable antenna diversity lock interrupt

#define 		CMT2310A_CTL_REG_23             0x17					// interrupt enable control 4
#define		CMT2310A_API_DONE_EN			(1<<7)					// 0=disable, 1=enable API done interrupt
#define		CMT2310A_CCA_STATUS_EN			(1<<6)					// 0=disable, 1=enable CCA status interrupt
#define		CMT2310A_CSMA_DONE_EN			(1<<5)					// 0=disable, 1=enable CSMA done interrupt
#define		CMT2310A_TX_DC_DONE_EN			(1<<4)					// 0=disable, 1=enable TX DutyCycle done interrupt
#define		CMT2310A_ACK_RECV_FAILED_EN		(1<<3)					// 0=disable, 1=enable ACK received failed interrupt
#define		CMT2310A_TX_RESEND_DONE_EN		(1<<2)					// 0=disable, 1=enable TX re-send done interrupt
#define		CMT2310A_NACK_RECV_EN			(1<<1)					// 0=disable, 1=enable no ACK received interrupt
#define		CMT2310A_SEQ_MATCH_EN			(1<<0)					// 0=disable, 1=enable sequence number match interrupt

#define 		CMT2310A_CTL_REG_24             0x18					// interrupt flag & clear control 1
#define		CMT2310A_SLEEP_TMO_FLG			(1<<5)					// sleep timer time-out flag
#define		CMT2310A_RX_TMO_FLG				(1<<4)					// rx timer time-out flag
#define		CMT2310A_TX_DONE_FLG			(1<<3)					// tx done flag
#define		CMT2310A_SLEEP_TMO_CLR			(1<<2)					// set '1' to clear Sleep timer time-out flag
#define		CMT2310A_RX_TMO_CLR				(1<<1)					// set '1' to clear Rx timer time-out flag
#define		CMT2310A_TX_DONE_CLR			(1<<0)					// set '1' to clear TX done flag

#define 		CMT2310A_CTL_REG_25             0x19					// interrupt flag & clear control 2
#define		CMT2310A_PREAM_PASS_CLR			(1<<4)					// set '1' to clear PREAM_PASS flag
#define		CMT2310A_SYNC_PASS_CLR			(1<<3)					// set '1' to clear SYNC_PASS flag
#define		CMT2310A_ADDR_PASS_CLR			(1<<2)					// set '1' to clear ADDR_PASS flag
#define		CMT2310A_CRC_PASS_CLR			(1<<1)					// set '1' to clear CRC_PASS flag
#define		CMT2310A_PKT_DONE_CLR			(1<<0)					// set '1' to clear PKT_DONE flag

#define 		CMT2310A_CTL_REG_26             0x1A					// interrupt flag & clear control 3	(-RO)
#define		CMT2310A_SYNC1_PASS_FLG			(1<<5)					// sync-word1 match interrupt flag
#define		CMT2310A_PREAM_PASS_FLG			(1<<4)					// preamlbe pass interrupt flag
#define		CMT2310A_SYNC_PASS_FLG			(1<<3)					// sync-word match interrupt flag
#define		CMT2310A_ADDR_PASS_FLG			(1<<2)					// node address match interrupt flag
#define		CMT2310A_CRC_PASS_FLG			(1<<1)					// packet crc pass interrupt flag
#define		CMT2310A_PKT_DONE_FLG			(1<<0)					// packet done interrupt flag

#define 		CMT2310A_CTL_REG_27             0x1B					// fifo control 1, fifo clear
#define		CMT2310A_TX_FIFO_RESTORE		(1<<2)					// 0=disable, 1=enable
#define		CMT2310A_RX_FIFO_CLR			(1<<1)					// set '1' to clear Rx FIFO
#define		CMT2310A_TX_FIFO_CLR			(1<<0)					// set '1' to clear Tx FIFO

#define 		CMT2310A_CTL_REG_28             0x1C					// fifo control 2, fifo flag		(-RO)
#define		CMT2310A_RX_FIFO_FULL_FLG		(1<<7)					//
#define		CMT2310A_RX_FIFO_NMTY_FLG		(1<<6)					//
#define		CMT2310A_RX_FIFO_TH_FLG			(1<<5)					//
#define		CMT2310A_RX_FIFO_OVF_FLG		(1<<3)					//
#define		CMT2310A_TX_FIFO_FULL_FLG		(1<<2)					//
#define		CMT2310A_TX_FIFO_NMTY_FLG		(1<<1)					//
#define		CMT2310A_TX_FIFO_TH_FLG			(1<<0)					//

#define 		CMT2310A_CTL_REG_29             0x1D					// auxrl function clear control
#define		CMT2310A_ANT_LOCK_CLR			(1<<4)					// set '1' to clear antenna lock
#define		CMT2310A_OP_CMD_FAILED_CLR		(1<<3)					// set '1' to clear command operate failed
#define		CMT2310A_RSSI_COLL_CLR			(1<<2)					// set '1' to clear rssi collision
#define		CMT2310A_PKT_ERR_CLR			(1<<1)					// set '1' to clear packet receive error
#define		CMT2310A_LBD_STATUS_CLR			(1<<0)					// set '1' to clear LBD status

#define 		CMT2310A_CTL_REG_30             0x1E					// auxrl function flag				(-RO)
#define		CMT2310A_ANT_LOCK_FLG			(1<<4)					//
#define		CMT2310A_OP_CMD_FAILED_FLG		(1<<3)					//
#define		CMT2310A_RSSI_COLL_FLG			(1<<2)					//
#define		CMT2310A_PKT_ERR_FLG			(1<<1)					//
#define		CMT2310A_LBD_STATUS_FLG			(1<<0)					//

#define 		CMT2310A_CTL_REG_31             0x1F					// mac function clear control
#define		CMT2310A_API_DONE_CLR			(1<<7)					// set '1' to clear API done
#define		CMT2310A_CCA_STATUS_CLR			(1<<6)					// set '1' to clear CCA status
#define		CMT2310A_CSMA_DONE_CLR			(1<<5)					// set '1' to clear CSMA done
#define		CMT2310A_TX_DC_DONE_CLR			(1<<4)					// set '1' to clear TX_DC_DONE
#define		CMT2310A_ACK_RECV_FAILED_CLR	(1<<3)					// set '1' to clear ACK receive failed
#define		CMT2310A_TX_RESEND_DONE_CLR		(1<<2)					// set '1' to clear Tx resend done
#define		CMT2310A_NACK_RECV_CLR			(1<<1)					// set '1' to clear NACK receive
#define		CMT2310A_SEQ_MATCH_CLR			(1<<0)					// set '1' to clear sequence number match

#define 		CMT2310A_CTL_REG_32             0x20					// mac function flag					(-RO)
#define		CMT2310A_API_DONE_FLG			(1<<7)					//
#define		CMT2310A_CCA_STATUS_FLG			(1<<6)					//
#define		CMT2310A_CSMA_DONE_FLG		    (1<<5)					//
#define		CMT2310A_TX_DC_DONE_FLG			(1<<4)					//
#define		CMT2310A_ACK_RECV_FAILED_FLG	(1<<3)					//
#define		CMT2310A_TX_RESEND_DONE_FLG		(1<<2)					//
#define		CMT2310A_NACK_RECV_FLG			(1<<1)					//
#define		CMT2310A_SEQ_MATCH_FLG			(1<<0)					//

#define 		CMT2310A_CTL_REG_33             0x21					// RSSI value minimum	  				(-RO)
#define		CMT2310A_RSSI_MIN_REG			0x21

#define 		CMT2310A_CTL_REG_34             0x22					// RSSI value							(-RO)
#define		CMT2310A_RSSI_REG				0x22

#define 		CMT2310A_CTL_REG_35             0x23					// LBD value							(-RO)
#define		CMT2310A_LBD_REG				0x23

#define 		CMT2310A_CTL_REG_36             0x24					// temperature value					(-RO)
#define		CMT2310A_TEMP_REG				0x24

#define 		CMT2310A_CTL_REG_37             0x25					// frequence channel current active 	(-RO)
#define		CMT2310A_FREQ_CHANL_ACT_REG		0x25

#define 		CMT2310A_CTL_REG_38             0x26					// sequence number tx out[7:0]			(-RO)
#define 		CMT2310A_CTL_REG_39             0x27					// sequence number tx out[15:8]			(-RO)
#define		CMT2310A_SEQNUM_TX_OUT_L_REG	0x26
#define		CMT2310A_SEQNUM_TX_OUT_H_REG	0x27
//--------------------------------- packet config -----------------------------
#define 		CMT2310A_CTL_REG_40             0x28					// rx preamble size[4:0] + preamble length unit + data mode[1:0]
#define		CMT2310A_RX_PREAM_SIZE_MASK		0xF8					// rx preamble detect length, 0=don't detect
#define		CMT2310A_PREAM_LENG_UNIIT		(1<<2)					// 0=8bits/unit,  1=4bits/unit (nibble)

#define		CMT2310A_DATA_MODE_MASK			(3<<0)
#define		CMT2310A_DIRECT_MODE			(0<<0)					// Direct mode
#define		CMT2310A_PACKET_MODE			(2<<0)					// Packet mode

#define 		CMT2310A_CTL_REG_41             0x29					// tx preamble size[7:0]
#define 		CMT2310A_CTL_REG_42             0x2A					// tx preamble size[15:8]
#define		CMT2310A_TX_PREAM_SIZE_L_REG	0x29					// when TX_PREAM_SIZE=0, means do not send preamble
#define		CMT2310A_TX_PREAM_SIZE_H_REG	0x2A

#define 		CMT2310A_CTL_REG_43             0x2B					// preamble value
#define		CMT2310A_PREAM_VALUE_REG		0x2B					// when PREAM_LENG_UNIIT = 0, 8bits active
// when PREAM_LENG_UNIIT = 1, 4bits(LSB) active
#define 		CMT2310A_CTL_REG_44             0x2C					// sync control, sync_mode_sel + sync_tolerance[2:0] + sync_size[2:0] + sync_man_en
#define		CMT2310A_SYNC_CTL_REG			0x2C
#define		CMT2310A_SYNC_MODE_SEL			(1<<7)					// 0=compatible S2LP,   1=compatible 802.15.4
#define		CMT2310A_SYNC_TOL_MASK			(7<<4)
#define		CMT2310A_SYNC_SIZE_MASK			(7<<1)					// n+1 for SyncWord length
#define		CMT2310A_SYNC_MAN_EN			(1<<0)					// 0=disable, 1=enable SyncWord manchester encoding

#define 		CMT2310A_CTL_REG_45             0x2D					// sync value 0 [7:0]		send last
#define		CMT2310A_SYNC_VALUE_7_REG		0x2D
#define 		CMT2310A_CTL_REG_46             0x2E					// sync value 1 [15:8]
#define		CMT2310A_SYNC_VALUE_6_REG		0x2E
#define 		CMT2310A_CTL_REG_47             0x2F					// sync value 2 [23:16]
#define		CMT2310A_SYNC_VALUE_5_REG		0x2F
#define 		CMT2310A_CTL_REG_48             0x30					// sync value 3 [31:24]
#define		CMT2310A_SYNC_VALUE_4_REG		0x30
#define 		CMT2310A_CTL_REG_49             0x31					// sync value 4 [39:32]
#define		CMT2310A_SYNC_VALUE_3_REG		0x31
#define 		CMT2310A_CTL_REG_50             0x32					// sync value 5 [47:40]
#define		CMT2310A_SYNC_VALUE_2_REG		0x32
#define 		CMT2310A_CTL_REG_51             0x33					// sync value 6 [55:48]
#define		CMT2310A_SYNC_VALUE_1_REG		0x33
#define 		CMT2310A_CTL_REG_52             0x34					// sync value 7 [63:56]  	send first
#define		CMT2310A_SYNC_VALUE_0_REG		0x34

#define 		CMT2310A_CTL_REG_53             0x35					// sync fec value 0 [7:0]
#define		CMT2310A_FEC_SYNC_7_REG			0x35
#define 		CMT2310A_CTL_REG_54             0x36					// sync fec value 1 [15:8]
#define		CMT2310A_FEC_SYNC_6_REG			0x36
#define 		CMT2310A_CTL_REG_55             0x37					// sync fec value 2 [23:16]
#define		CMT2310A_FEC_SYNC_5_REG			0x37
#define 		CMT2310A_CTL_REG_56             0x38					// sync fec value 3 [31:24]
#define		CMT2310A_FEC_SYNC_4_REG			0x38
#define 		CMT2310A_CTL_REG_57             0x39					// sync fec value 4 [39:32]
#define		CMT2310A_FEC_SYNC_3_REG			0x39
#define 		CMT2310A_CTL_REG_58             0x3A					// sync fec value 5 [47:40]
#define		CMT2310A_FEC_SYNC_2_REG			0x3A
#define 		CMT2310A_CTL_REG_59             0x3B					// sync fec value 6 [55:48]
#define		CMT2310A_FEC_SYNC_1_REG			0x3B
#define 		CMT2310A_CTL_REG_60             0x3C					// sync fec	value 7 [63:56]
#define		CMT2310A_FEC_SYNC_0_REG			0x3C

#define 		CMT2310A_CTL_REG_61             0x3D					// payload length [7:0]
#define 		CMT2310A_CTL_REG_62             0x3E					// payload length [15:8]
#define		CMT2310A_PAYLOAD_LENGTH_L_REG	0x3D
#define		CMT2310A_PAYLOAD_LENGTH_H_REG	0x3E

#define 		CMT2310A_CTL_REG_63             0x3F					// packet config 1,  interleave_en  +        x        +  length_size + piggybacking_en + x + addr_leng_conf + payload_bit_order + pkt_type
#define		CMT2310A_INTERLEAVE_EN			(1<<7)					// 0=disable,  	1=enable interleave
#define		CMT2310A_LENGTH_SIZE			(1<<5)					// 0=1Byte, for up to 255 bytes variable length packet��
// 1=2Bytes, for up to 65535 bytes variable length packet
#define		CMT2310A_PAGGYBACKING_EN		(1<<4)					// 0=disable,  	1=enable for auto paggy backing payload for auto-ack
#define		CMT2310A_ADDR_LEN_CONF			(1<<2)					// 0=node address before length filed, 	1=node address after length filed.  Note! It is affect payload length
#define		CMT2310A_PAYLOAD_BIT_ORDER		(1<<1)					// 0=MSB first, 1=LSB first for decode (payload+crc filed)
#define		CMT2310A_PKT_TYPE				(1<<0)					// 0=fixed length packet,  	1=variable length packet

#define 		CMT2310A_CTL_REG_64             0x40					// packet config 2,  sync_value_sel + addr_split_mode + addr_free_en +  addr_err_mask  +   addr_size[1:0]   + addr_det_mode[1:0]
#define		CMT2310A_SYNC_VALUE_SEL			(1<<7)					// 0=select SYNC_VALUE,  	1=select SYNC_FEC_VALUE,  only for TX used
#define		CMT2310A_ADDR_SPLIT_MODE		(1<<6)					// 0=only DEST_ADDR filed active, 	1=DEST_ADDR+SRC_ADDR filed both active
#define		CMT2310A_ADDR_FREE_EN			(1<<5)					// 0=disable,  	1=enable ADDR detect stand-alone
#define		CMT2310A_ADDR_ERR_MASK			(1<<4)					// 0=allow, 	1=not allow for reset decoding when ADDR mis-matching
#define		CMT2310A_ADDR_SIZE_MASK			(3<<2)					//

#define		CMT2310A_ADDR_DET_MODE_MASK		(3<<0)
#define		CMT2310A_ADDR_DET_DISABLE		(0<<0)					// disable node address detect
#define		CMT2310A_ADDR_DET_MATCH			(1<<0)					// rx only detect ADDR_VALUE,    tx send ADDR_VALUE
#define		CMT2310A_ADDR_DET_BOARDCAST_0	(2<<0)					// rx detect ADDR_VALUE & all-0, tx send ADDR_VALUE
#define		CMT2310A_ADDR_DET_BOARDCAST_ALL	(3<<0)					// rx detect ADDR_VALUE & all-0 & all-1,  tx send ADDR_VALUE

#define 		CMT2310A_CTL_REG_65             0x41					// SRC_ADDR  [7:0]
#define 		CMT2310A_CTL_REG_66             0x42					// SRC_ADDR  [15:8]
#define		CMT2310A_SRC_ADDR_L_REG			0x41
#define		CMT2310A_SRC_ADDR_H_REG			0x42

#define 		CMT2310A_CTL_REG_67             0x43					// DEST_ADDR [7:0]
#define 		CMT2310A_CTL_REG_68             0x44					// DEST_ADDR [15:8]
#define		CMT2310A_DEST_ADDR_L_REG		0x43
#define		CMT2310A_DEST_ADDR_H_REG		0x44

#define 		CMT2310A_CTL_REG_69             0x45					// SRC_BITMASK [7:0]
#define 		CMT2310A_CTL_REG_70             0x46					// SRC_BITMASK [15:8]
#define		CMT2310A_SRC_BITMASK_L_REG		0x45
#define		CMT2310A_SRC_BITMASK_H_REG		0x46

#define 		CMT2310A_CTL_REG_71             0x47					// DEST_BITMASK [7:0]
#define 		CMT2310A_CTL_REG_72             0x48					// DEST_BITMASK [15:8]
#define		CMT2310A_DEST_BITMASK_L_REG		0x47
#define		CMT2310A_DEST_BITMASK_H_REG		0x48

#define 		CMT2310A_CTL_REG_73             0x49					// crc config,       crc_size[1:0]  +  crc_byte_swap   + crc_bit_inv  +   crc_range   +  crc_refin  +  crc_bit_order  +  crc_en
#define		CMT2310A_CRC_CFG_REG			0x49
#define		CMT2310A_CRC_SIZE_MASK			(3<<6)
#define		CMT2310A_CRC_SLE_CRC8			(0<<6)					// select crc-8
#define		CMT2310A_CRC_SLE_CRC16			(1<<6)					// select crc-16
#define		CMT2310A_CRC_SLE_CRC24			(2<<6)					// select crc-24
#define		CMT2310A_CRC_SLE_CRC32			(3<<6)					// select crc-32

#define		CMT2310A_CRC_BYTE_SWAP			(1<<5)					// crc16 for example, 0=[15:8]+[7:0],  1=[7:0]+[15:8],
#define		CMT2310A_CRC_BIT_INV			(1<<4)					// 0=disable,  		  1=enable for crc result inver
#define		CMT2310A_CRC_RANGE				(1<<3)					// 0=whole payload,   1=only data
#define		CMT2310A_CRC_REFIN				(1<<2)					// 0=normal bit input flow,  1=inver bit input flow, for byte as unit
#define		CMT2310A_CRC_BIT_ORDER			(1<<1)					// 0=MSB send first,  1=LSB send first
#define		CMT2310A_CRC_EN					(1<<0)					// 0=disable, 		  1=enable CRC function

#define 		CMT2310A_CTL_REG_74             0x4A					// crc seed 0 [7:0]
#define 		CMT2310A_CTL_REG_75             0x4B					// crc seed 1 [15:8]
#define 		CMT2310A_CTL_REG_76             0x4C					// crc seed 2 [23:16]
#define 		CMT2310A_CTL_REG_77             0x4D					// crc seed 3 [31:24]
#define		CMT2310A_CRC_SEED_0_REG			0x4A
#define		CMT2310A_CRC_SEED_1_REG			0x4B
#define		CMT2310A_CRC_SEED_2_REG			0x4C
#define		CMT2310A_CRC_SEED_3_REG			0x4D

#define 		CMT2310A_CTL_REG_78             0x4E					// crc polynomial 0 [7:0]
#define 		CMT2310A_CTL_REG_79             0x4F					// crc polynomial 1 [15:8]
#define 		CMT2310A_CTL_REG_80             0x50					// crc polynomial 2 [23:16]
#define 		CMT2310A_CTL_REG_81             0x51					// crc polynomial 3 [31:24]
#define		CMT2310A_CRC_POLY_0_REG			0x4E
#define		CMT2310A_CRC_POLY_1_REG			0x4F
#define		CMT2310A_CRC_POLY_2_REG			0x50
#define		CMT2310A_CRC_POLY_3_REG			0x51

#define 		CMT2310A_CTL_REG_82             0x52					// coding config,    crc_refout  +  whiten_seed[8]  +  whiten_seed_type  +  whiten_type[1:0]  +  whiten_en  +  manch_type  + manch_en
#define		CMT2310A_CRC_REFOUT				(1<<7)					// 0=MSB->LSB,  1=LSB->MSB, crc result turn over, byte as unit
#define		CMT2310A_WHITEN_SEED_B8			(1<<6)					// whiten_seed[8]
#define		CMT2310A_WHITEN_SEED_TYPE		(1<<5)					// 0=compatible A7139 PN7 seed,  1=PN7 seed by whien_seed

#define		CMT2310A_WHITEN_TYPE_MASK		(3<<3)
#define		CMT2310A_WHITEN_PN9_CCITT		(0<<3)					// whiten type select PN9 CCITT
#define		CMT2310A_WHITEN_PN9_IBM			(1<<3)					// whiten type select PN9 IBM
#define		CMT2310A_WHITEN_PN7				(2<<3)					// whiten type select PN7

#define		CMT2310A_WHITEN_EN				(1<<2)					// 0=disable, 1=enable whien encoding/decoding

#define		CMT2310A_MANCH_TYPE				(1<<1)					// 0= 2'b01 as logic'1', 2'b10 as logic'0'
// 1= 2'b10 as logic'1', 2'b01 as logic'0'

#define		CMT2310A_MANCH_EN				(1<<0)					// 0=disable, 1=enable	manchester encoding/decoding

#define 		CMT2310A_CTL_REG_83             0x53					// whiten seed [7:0]
#define		CMT2310A_WHITEN_SEED_REG		0x53

#define 		CMT2310A_CTL_REG_84             0x54					// packet config 3,  crc_err_clr_fifo_en + fcs2_en  +  seqnum_match_en  +  seqnum_size  +  seqnum_auto_inc  +  seqnum_en  +  tx_prefix_type[1:0]
#define		CMT2310A_CRCERR_CLR_FIFO_EN		(1<<7)					// 0=not clear,  1=clear Rx FIFO, when receive packet with CRC mis-matching
#define		CMT2310A_FCS2_EN				(1<<6)					// 0=disable,    1=enable FCS2 filed
#define		CMT2310A_SEQNUM_MATCH_EN		(1<<5)					// 0=disable,    1=enable matching for TX_ACK used, receive ack packet matching seqnum number which sent by local
#define		CMT2310A_SEQNUM_SIZE			(1<<4)					// 0=1Byte,      1=2Byte for sequence number filed
#define		CMT2310A_SEQNUM_AUTO_INC		(1<<3)					// 0=disable, 	 1=enable sequence number auto increase in TX, step=1
#define		CMT2310A_SEQNUM_EN				(1<<2)					// 0=disable,    1=enable sequence number filed (FCS1)

#define		CMT2310A_TX_PREFIX_TYPE_MASK	(3<<0)
#define		CMT2310A_TX_PREFIX_IS_0			(0<<0)					// tx prefix = 0
#define		CMT2310A_TX_PREFIX_IS_1			(1<<0)					// tx prefix = 1
#define		CMT2310A_TX_PREFIX_IS_PREAM		(2<<0)					// tx prefix = preamble

#define	 		CMT2310A_CTL_REG_85             0x55					// tx packet number [7:0]
#define 		CMT2310A_CTL_REG_86             0x56					// tx packet number [15:8]
#define		CMT2310A_TX_PKT_NUM_L_REG		0x55
#define		CMT2310A_TX_PKT_NUM_H_REG		0x56

#define 		CMT2310A_CTL_REG_87             0x57					// sequence number tx inital value [7:0]
#define 		CMT2310A_CTL_REG_88             0x58					// sequence number tx inital value [15:8]
#define		CMT2310A_SEQNUM_TX_IN_L_REG		0x57
#define		CMT2310A_SEQNUM_TX_IN_H_REG		0x58

#define 		CMT2310A_CTL_REG_89             0x59					// tx packet gap [7:0]
#define		CMT2310A_TX_PKT_GAP_REG			0x59

#define 		CMT2310A_CTL_REG_90             0x5A					// Rssi calibrate offset
#define		CMT2310A_RSSI_CAL_OFFSET_REG	0x5A

#define 		CMT2310A_CTL_REG_91             0x5B					// tx side FCS2 filed inital value [7:0]
#define		CMT2310A_FCS2_TX_IN_REG			0x5B

#define 		CMT2310A_CTL_REG_92             0x5C					// rx side FCC2 filed receive value [7:0]					(-RO)
#define		CMT2310A_FCS2_RX_OUT_REG		0x5C

#define 		CMT2310A_CTL_REG_93             0x5D					// fec config 1, FEC_TICC  +  fec_padding_code[12:8]  +  fec_rcs_nrnsc_sel  + fec_en
#define		CMT2310A_FEC_TICC				(1<<7)					// 0=invert, 	1=not invert, for FEC ui
#define 	CMT2310A_FEC_PAD_CODE_H_MASK	(0x1F<<2)				// fec_padding_code[12:8]
#define		CMT2310A_FEC_RSC_NRNSC_SEL		(1<<1)					// 0=RSC mode,  1=NRNSC mode
#define		CMT2310A_FEC_EN					(1<<0)					// 0=disable,   1=enable FEC encoding/decoding

#define 		CMT2310A_CTL_REG_94             0x5E					// fec config 2,   fec_padding_code[7:0]
#define		CMT2310A_FEC_PAD_CODE_L_REG		0x5E

#define 		CMT2310A_CTL_REG_95             0x5F					// 4-FSK config,   4fsk_3_level[1:0]  +  4fsk_2_level[1:0]  +  4fsk_1_level[1:0]  +  4fsk_0_level[1:0]
#define		CMT2310A_MAP_4FSK_LEVEL_REG		0x5F
//--------------------------------- system control config --------------------------------
#define 		CMT2310A_CTL_REG_96             0x60					// sysctrl 1,      fw_nk_sel  +  tx_exit_state[2:0]  +      x      +  tx_ack_en  +  tx_dc_persist_en  +  tx_dc_en

#define		CMT2310A_TX_EXIT_STATE_MASK		(7<<4)					// tx done exit state masker
#define		CMT2310A_TX_EXIT_TO_SLEEP		(1<<4)					// tx done exit to sleep
#define		CMT2310A_TX_EXIT_TO_READY		(2<<4)					// tx done exit to ready
#define		CMT2310A_TX_EXIT_TO_TFS			(3<<4)					// tx done exit to TFS
#define		CMT2310A_TX_EXIT_TO_TX			(4<<4)					// tx done exit to TX
#define		CMT2310A_TX_EXIT_TO_RFS			(5<<4)					// tx done exit to RFS
#define		CMT2310A_TX_EXIT_TO_RX			(6<<4)					// tx done exit to RX
// others to sleep, & only for packet mode tx done
#define		CMT2310A_TX_AUTO_HOP_EN			(1<<3)					// 0=disable, 	1=enable tx auto hopping
#define		CMT2310A_TX_ACK_EN				(1<<2)					// 0=disable, 	1=enable tx ack function
#define		CMT2310A_TX_DC_PERSIST_EN		(1<<1)					// 0=auto exit tx duty-cycle when reach TX_DC_TIMES,  1=hold on until set this bit to '0'
#define		CMT2310A_TX_DC_EN				(1<<0)					// 0=disable,  	1=enable tx duty-cycle function

#define 		CMT2310A_CTL_REG_97             0x61					// sysctrl 2,      csma_en    +  rx_exit_state[2:0]  + timer_rx_en +  rx_ack_en  +   rx_auto_hop_en   +  rx_dc_en
#define		CMT2310A_CSMA_EN				(1<<7)					// 0=disable, 	1=enable CSMA

#define		CMT2310A_RX_EXIT_STATE_MASK		(7<<4)					// rx done exit state masker
#define		CMT2310A_RX_EXIT_TO_SLEEP		(1<<4)					// rx done exit to sleep
#define		CMT2310A_RX_EXIT_TO_READY		(2<<4)					// rx done exit to ready
#define		CMT2310A_RX_EXIT_TO_TFS			(3<<4)					// rx done exit to TFS
#define		CMT2310A_RX_EXIT_TO_TX			(4<<4)					// rx done exit to TX
#define		CMT2310A_RX_EXIT_TO_RFS			(5<<4)					// rx done exit to RFS
#define		CMT2310A_RX_EXIT_TO_RX			(6<<4)					// rx done exit to RX
// others to sleep, & only for packet mode rx done
#define		CMT2310A_RX_TIMER_EN			(1<<3)					// 0=disable, 	1=enable rx timer
#define		CMT2310A_RX_ACK_EN				(1<<2)					// 0=disable, 	1=enable rx ack function
#define		CMT2310A_RX_AUTO_HOP_EN			(1<<1)					// 0=disable,   1=enable rx auto hopping function
#define		CMT2310A_RX_DC_EN				(1<<0)					// 0=disable, 	1=enable rx duty-cycle function

#define 		CMT2310A_CTL_REG_98             0x62					// sysctrl 3,      pkt_done_exit_en  +  rx_hop_slp_mode[2:0]  +  slp_mode[3:0]
#define		CMT2310A_PKT_DONE_EXIT_EN		(1<<7)					// 0=keep current, 	1=depends on RX_EXIT_STATE
#define		CMT2310A_RX_HOP_SLP_MODE_MASK	(7<<4)					// rx hopping supper low power mode selection
#define		CMT2310A_SLP_MODE_MASK			(15<<0)					// rx supper low power mode selection

#define 		CMT2310A_CTL_REG_99             0x63                    // sysctrl 4,      timer_m_sleep [7:0]
#define 		CMT2310A_CTL_REG_100            0x64					// sysctrl 5, 	   timer_m_sleep [10:8]  +  timer_r_sleep [4:0]
#define		CMT2310A_SLEEP_TIMER_M_REG		0x63					// Tsleep = M*2^(R+1)*31.25us, R from 0 to 26
#define		CMT2310A_SLEEP_TIMER_R_REG		0x64
#define		CMT2310A_SLEEP_TIMER_M_H_MASK	(7<<5)

#define 		CMT2310A_CTL_REG_101            0x65					// sysctrl 6,      timer_m_rx_T1 [7:0]
#define 		CMT2310A_CTL_REG_102            0x66					// sysctrl 7,      timer_m_rx_T1 [10:8]  +  timer_r_rx_T1 [4:0]
#define		CMT2310A_RX_TIMER_T1_M_REG		0x65					// Trx_t1 = M*2^(R+1)*20us, R from 0 to 21,
#define		CMT2310A_RX_TIMER_T1_R_REG		0x66
#define		CMT2310A_RX_TIMER_T1_M_H_MASK	(7<<5)

#define 		CMT2310A_CTL_REG_103            0x67					// sysctrl 8, 	   timer_m_rx_T2 [7:0]
#define 		CMT2310A_CTL_REG_104            0x68					// sysctrl 9,      timer_m_rx_T2 [10:8]  +  timer_r_rx_T2 [4:0]
#define		CMT2310A_RX_TIMER_T2_M_REG		0x67					// Trx_t1 = M*2^(R+1)*20us, R from 0 to 21,
#define		CMT2310A_RX_TIMER_T2_R_REG		0x68
#define		CMT2310A_RX_TIMER_T2_M_H_MASK	(7<<5)

#define 		CMT2310A_CTL_REG_105            0x69					// sysctrl 10,     tx_allow_rx  +  pup_done_state  +  timer_random_mode[1:0]  +  timer_sleep_en  +  rx_allow_tx  +  lfclk_sel  +  lfclk_out_en(lfosc/lfxo)

#define		CMT2310A_TIMER_RAND_MODE_MASK	(3<<4)					// CSMA mode, CSMA sleep timer random selection:
#define		CMT2310A_TIMER_RAND_R			(0<<4)					// select random R
#define		CMT2310A_TIMER_RAND_M 			(1<<4)					// select random M
#define		CMT2310A_TIMER_RAND_M_R			(2<<4)					// select random both R & M
#define		CMT2310A_TIMER_RAND_OFF			(3<<4)					// select random off, used fixed value

#define		CMT2310A_SLEEP_TIMER_EN			(1<<3)					// 0=disable, 	1=enable sleep timer
#define		CMT2310A_LFCLK_SEL				(1<<1)					// 0=internal LFOSC,  	1=LFXO (32768Hz)
#define		CMT2310A_LFCLK_OUT_EN			(1<<0)					// 0=disable, 	1=enable GPIO4 for LFCLK out, priority lower than DIG_CLKOUT_EN, but higher than GPIO4_SEL

#define 		CMT2310A_CTL_REG_106            0x6A					// sysctrl 11,     cca_mode[2:0]  +  csma_persist_en  +  cca_int_sel[1:0]  +  cca_win_sel[1:0]
#define		CMT2310A_CSMA_CCA_MODE_MASK		(7<<5)
#define		CMT2310A_CSMA_CCA_DISABLE		(0<<5)					// 0= channel is always idle
#define		CMT2310A_CSMA_CCA_RSSI			(1<<5)					// 1= active >=1 of 4 times detect window, by Rssi
#define		CMT2310A_CSMA_CCA_PJD			(2<<5)					// 2= active >=1 of 4 times detect window, by PJD
#define		CMT2310A_CSMA_CCA_RSSI_PJD		(3<<5)					// 3= active >=1 of 4 times detect window, by PJD or Rssi
#define		CMT2310A_CSMA_CCA_SYNC			(4<<5)					// 4= detected sync word
#define		CMT2310A_CSMA_CCA_SYNC_RSSI		(5<<5)					// 5= detected sync word, or active >=1 of 4 times detect window, by Rssi
#define		CMT2310A_CSMA_CCA_SYNC_PJD		(6<<5)					// 6= detected sync word, or active >=1 of 4 times detect window, by PJD
#define		CMT2310A_CSMA_CCA_SYNC_RSSI_PJD	(7<<5)					// 7= detected sync word, or active >=1 of 4 times detect window, by PJD or Rssi

#define		CMT2310A_CSMA_PERSIST_EN		(1<<4)					// 0=auto exit when reach max & channel still busy, 	1=keep on work until send out

#define		CMT2310A_CCA_INT_SEL_MASK		(3<<2)
#define		CMT2310A_CCA_INT_BY_PJD			(0<<2)					// CSMA_CCA interrupt condition by PJD
#define		CMT2310A_CCA_INT_BY_RSSI		(1<<2)					// CSMA_CCA interrupt condition by RSSI
#define		CMT2310A_CCA_INT_BY_PJD_RSSI	(2<<2)					// CSMA_CCA interrupt condition by PJD & RSSI

#define		CMT2310A_CCA_WIN_SEL_MASK		(3<<0)
#define		CMT2310A_CCA_WIN_32_SYMBOL		(0<<0)					// CSMA detect window select 32symbols
#define		CMT2310A_CCA_WIN_64_SYMBOL		(1<<0)					// CSMA detect window select 64symbols
#define		CMT2310A_CCA_WIN_128_SYMBOL		(2<<0)					// CSMA detect window select 128symbols
#define		CMT2310A_CCA_WIN_256_SYMBOL		(3<<0)					// CSMA detect window select 256symbols

#define 		CMT2310A_CTL_REG_107            0x6B					// sysctrl 12, 	   timer_m_rx_csma [7:0]
#define 		CMT2310A_CTL_REG_108            0x6C					// sysctrl 13,     timer_m_rx_csma [10:8]  +  timer_r_rx_csma [4:0]
#define		CMT2310A_RX_TIMER_CSMA_M_REG	0x6B					// Trx_csma = M*2^(R+1)*20us, R from 0 to 21,
#define		CMT2310A_RX_TIMER_CSMA_R_REG	0x6C
#define		CMT2310A_RX_TIMER_CSMA_M_H_MASK	(15<<4)

#define 		CMT2310A_CTL_REG_109            0x6D					// low battery detect threshold, lbd_th[7:0]
#define		CMT2310A_LBD_TH_REG				0x6D

#define 		CMT2310A_CTL_REG_110            0x6E					// sysctrl 14,     tx_dc_timer [7:0]
#define		CMT2310A_TX_DC_TIMES_REG		0x6E

#define 		CMT2310A_CTL_REG_111            0x6F					//
#define		CMT2310A_LENGTH_MODE			(1<<7)					// 0=normal, length filed as payload length,  	1=as Wi-SUN mode
#define		CMT2310A_WISUN_ALLIN			(1<<6)
#define		CMT2310A_WHITEN_WISUN			(1<<5)
#define		CMT2310A_WISUN_MS				(1<<4)
#define		CMT2310A_WISUN_FCS				(1<<1)
#define		CMT2310A_WISUN_DW				(1<<0)

#define 		CMT2310A_CTL_REG_112            0x70					// sysctrl 15,     tx_dc_done_timer [7:0]		(-RO)
#define		CMT2310A_TX_DC_DONE_TIMES_REG	0x70

#define 		CMT2310A_CTL_REG_113            0x71					// sysctrl 16,     tx_resend_timer [7:0]
#define		CMT2310A_TX_RS_TIMES_REG		0x71

#define 		CMT2310A_CTL_REG_114            0x72					// sysctrl 17,     tx_resend_done_timer [7:0]	(-RO)
#define		CMT2310A_TX_RS_DONE_TIMES_REG	0x72

#define 		CMT2310A_CTL_REG_115            0x73					// sysctrl 18,     csma_timer [7:0]
#define		CMT2310A_CSMA_TIMES_REG			0x73

#define 		CMT2310A_CTL_REG_116            0x74					// sysctrl 19,     csma_done_timer [7:0]		(-RO)
#define		CMT2310A_CSMA_DONE_TIMES_REG	0x74

#define 		CMT2310A_CTL_REG_117            0x75

#define 		CMT2310A_CTL_REG_118            0x76					// sysctrl 20, 	   timer_m_sleep_csma [7:0]
#define 		CMT2310A_CTL_REG_119            0x77					// sysctrl 21, 	   timer_m_sleep_csma [10:8]  +  timer_r_sleep_csma [4:0]
#define		CMT2310A_SLEEP_TIMER_CSMA_M_REG	0x76					// Tsleep_timer_csma = M*2^(R+1)*31.25us, R from 0 to 26,
#define		CMT2310A_SLEEP_TIMER_CSMA_R_REG	0x77

#define			CMT2310A_FIFO_RW_PORT_REG		0x7A					// FIFO R/W interface
#define			CMT2310A_FIFO_RW_PORT			0x7A

#define			CMT2310A_CRW_PORT_REG			0x7B
#define			CMT2310A_CRW_PORT				0x7B

#define			CMT2310A_CLT_REG_126			0x7E
#define 		CMT2310A_PAGE_CTL_REG	        0x7E					// hv_page_sel
#define		CMT2310A_PAGE_SEL_MASK			(3<<6)
#define		CMT2310A_PAGE_0					(0<<6)					// 0=page 0
#define		CMT2310A_PAGE_1					(1<<6)					// 1=page 1
#define		CMT2310A_PAGE_2					(2<<6)					// 2=page 2

#define 		CMT2310A_SOFT_RST               0x7F					// soft_reset

// --------------------------- CUS PAGE1 defines -----------------------------------------
#define 		CMT2310A_CMT_REG_00             0x00              		// export by RFPDK
#define 		CMT2310A_CMT_REG_01             0x01                    // export by RFPDK
#define 		CMT2310A_CMT_REG_02             0x02                    // export by RFPDK
#define 		CMT2310A_CMT_REG_03             0x03                    // export by RFPDK
#define 		CMT2310A_CMT_REG_04             0x04                    // export by RFPDK
#define 		CMT2310A_CMT_REG_05             0x05                    // export by RFPDK
#define 		CMT2310A_CMT_REG_06             0x06                    // export by RFPDK
#define 		CMT2310A_CMT_REG_07             0x07                    // export by RFPDK
#define 		CMT2310A_CMT_REG_08             0x08                    // export by RFPDK
#define 		CMT2310A_CMT_REG_09             0x09                    // export by RFPDK
#define 		CMT2310A_CMT_REG_10             0x0A                    // export by RFPDK
#define 		CMT2310A_CMT_REG_11             0x0B                    // export by RFPDK
#define 		CMT2310A_CMT_REG_12             0x0C                    // export by RFPDK
#define 		CMT2310A_CMT_REG_13             0x0D                    // export by RFPDK
#define 		CMT2310A_CMT_REG_14             0x0E                    // export by RFPDK
#define 		CMT2310A_CMT_REG_15             0x0F                    // export by RFPDK
#define 		CMT2310A_TX_FREQ_REG_00         0x10                    // export by RFPDK
#define 		CMT2310A_TX_FREQ_REG_01         0x11                    // export by RFPDK
#define 		CMT2310A_TX_FREQ_REG_02         0x12                    // export by RFPDK
#define 		CMT2310A_TX_FREQ_REG_03         0x13                    // export by RFPDK
#define 		CMT2310A_TX_MODE_REG_00         0x14                    // export by RFPDK
#define 		CMT2310A_TX_DR_REG_00           0x15                    // export by RFPDK
#define 		CMT2310A_TX_DR_REG_01           0x16                    // export by RFPDK
#define 		CMT2310A_TX_DR_REG_02           0x17                    // export by RFPDK
#define 		CMT2310A_TX_DR_REG_03           0x18                    // export by RFPDK
#define 		CMT2310A_TX_DR_REG_04           0x19                    // export by RFPDK
#define 		CMT2310A_TX_DR_REG_05           0x1A                    // export by RFPDK
#define 		CMT2310A_TX_DEV_REG_00          0x1B                    // export by RFPDK
#define 		CMT2310A_TX_DEV_REG_01          0x1C                    // export by RFPDK
#define 		CMT2310A_TX_DEV_REG_02          0x1D                    // export by RFPDK
#define 		CMT2310A_TX_PWR_REG_00          0x1E                    // export by RFPDK
#define 		CMT2310A_TX_PWR_REG_01          0x1F                    // export by RFPDK
#define 		CMT2310A_TX_PWR_REG_02          0x20                    // export by RFPDK
#define 		CMT2310A_TX_PWR_REG_03          0x21                    // export by RFPDK
#define 		CMT2310A_TX_PWR_REG_04          0x22                    // export by RFPDK
#define 		CMT2310A_TX_PWR_REG_05          0x23                    // export by RFPDK
#define 		CMT2310A_TX_PWR_REG_06          0x24                    // export by RFPDK
#define 		CMT2310A_TX_MISC_REG_00         0x25                    // export by RFPDK
#define 		CMT2310A_TX_MISC_REG_01         0x26                    // export by RFPDK
#define 		CMT2310A_TX_MISC_REG_02         0x27                    // export by RFPDK
#define 		CMT2310A_TX_RESEV_00            0x28                    // export by RFPDK
#define 		CMT2310A_TX_RESEV_01            0x29                    // export by RFPDK
#define 		CMT2310A_TX_RESEV_02            0x2A                    // export by RFPDK
#define 		CMT2310A_TX_RESEV_03            0x2B                    // export by RFPDK
#define 		CMT2310A_TX_RESEV_04            0x2C                    // export by RFPDK
#define 		CMT2310A_TX_RESEV_05            0x2D                    // export by RFPDK
#define 		CMT2310A_TX_RESEV_06            0x2E                    // export by RFPDK
#define 		CMT2310A_TX_RESEV_07            0x2F                    // export by RFPDK
#define 		CMT2310A_RX_FREQ_REG_00         0x30                    // export by RFPDK
#define 		CMT2310A_RX_FREQ_REG_01         0x31                    // export by RFPDK
#define 		CMT2310A_RX_FREQ_REG_02         0x32                    // export by RFPDK
#define 		CMT2310A_RX_FREQ_REG_03         0x33                    // export by RFPDK
#define 		CMT2310A_RX_FREQ_REG_04         0x34                    // export by RFPDK
#define 		CMT2310A_RX_IRF_REG_00          0x35                    // export by RFPDK
#define 		CMT2310A_RX_IRF_REG_01          0x36                    // export by RFPDK
#define 		CMT2310A_RX_IRF_REG_02          0x37                    // export by RFPDK
#define 		CMT2310A_RX_IRF_REG_03          0x38                    // export by RFPDK
#define 		CMT2310A_RX_PWR_REG_00          0x39                    // export by RFPDK
#define 		CMT2310A_RX_PWR_REG_01          0x3A                    // export by RFPDK
#define 		CMT2310A_RX_PWR_REG_02          0x3B                    // export by RFPDK
#define 		CMT2310A_RX_DR_REG_00           0x3C                    // export by RFPDK
#define 		CMT2310A_RX_DR_REG_01           0x3D                    // export by RFPDK
#define 		CMT2310A_RX_DR_REG_02           0x3E                    // export by RFPDK
#define 		CMT2310A_RX_DR_REG_03           0x3F                    // export by RFPDK
#define 		CMT2310A_RX_DR_REG_04           0x40                    // export by RFPDK
#define 		CMT2310A_RX_DR_REG_05           0x41                    // export by RFPDK
#define 		CMT2310A_RX_DR_REG_06           0x42                    // export by RFPDK
#define 		CMT2310A_RX_DR_REG_07           0x43                    // export by RFPDK
#define 		CMT2310A_RX_CDR_REG_00          0x44                    // export by RFPDK
#define 		CMT2310A_RX_CDR_REG_01          0x45                    // export by RFPDK
#define 		CMT2310A_RX_CDR_REG_02          0x46                    // export by RFPDK
#define 		CMT2310A_RX_CDR_REG_03          0x47                    // export by RFPDK
#define 		CMT2310A_RX_CHNL_REG_00         0x48                    // export by RFPDK
#define 		CMT2310A_RX_CHNL_REG_01         0x49                    // export by RFPDK
#define 		CMT2310A_RX_CHNL_REG_02         0x4A                    // export by RFPDK
#define 		CMT2310A_RX_AGC_REG_00          0x4B                    // export by RFPDK
#define 		CMT2310A_RX_AGC_REG_01          0x4C                    // export by RFPDK
#define 		CMT2310A_RX_AGC_REG_02          0x4D                    // export by RFPDK
#define 		CMT2310A_RX_AGC_REG_03          0x4E                    // export by RFPDK
#define 		CMT2310A_RX_AGC_REG_04          0x4F                    // export by RFPDK
#define 		CMT2310A_RX_AGC_REG_05          0x50                    // export by RFPDK
#define 		CMT2310A_RX_AGC_REG_06          0x51                    // export by RFPDK
#define 		CMT2310A_RX_2FSK_REG_00         0x52                    // export by RFPDK
#define 		CMT2310A_RX_2FSK_REG_01         0x53                    // export by RFPDK
#define 		CMT2310A_RX_2FSK_REG_02         0x54                    // export by RFPDK
#define 		CMT2310A_RX_2FSK_REG_03         0x55                    // export by RFPDK
#define 		CMT2310A_RX_AFC_REG_00          0x56                    // export by RFPDK
#define 		CMT2310A_RX_AFC_REG_01          0x57                    // export by RFPDK
#define 		CMT2310A_RX_AFC_REG_02          0x58                    // export by RFPDK
#define 		CMT2310A_RX_AFC_REG_03          0x59                    // export by RFPDK
#define 		CMT2310A_RX_4FSK_REG_00         0x5A                    // export by RFPDK
#define 		CMT2310A_RX_4FSK_REG_01         0x5B                    // export by RFPDK
#define 		CMT2310A_RX_4FSK_REG_02         0x5C                    // export by RFPDK
#define 		CMT2310A_RX_4FSK_REG_03         0x5D                    // export by RFPDK
#define 		CMT2310A_RX_OOK_REG_00          0x5E                    // export by RFPDK
#define 		CMT2310A_RX_OOK_REG_01          0x5F                    // export by RFPDK
#define 		CMT2310A_RX_OOK_REG_02          0x60                    // export by RFPDK
#define 		CMT2310A_RX_OOK_REG_03          0x61					// export by RFPDK

#define 		CMT2310A_RX_RSSI_REG_00         0x62					// rf_rssi_reg 0, 	coll_handle[1:0] + coll_step_sel[1:0] + rssi_update_sel[1:0] + nc + coll_det_en

#define		CMT2310A_COLL_STEP_SEL_MASK		(3<<4)
#define		CMT2310A_COLL_STEP_SEL_6dB		(0<<4)					// collision select 6dB
#define		CMT2310A_COLL_STEP_SEL_10dB		(1<<4)					// collision select 10dB
#define		CMT2310A_COLL_STEP_SEL_16dB		(2<<4)					// collision select	16dB
#define		CMT2310A_COLL_STEP_SEL_20dB		(3<<4)					// collision select	20dB

#define		CMT2310A_RSSI_UPDATE_SEL_MASK	(3<<2)
#define		CMT2310A_RSSI_UPDATE_ALWAYS		(0<<2)					// rssi update always on
#define		CMT2310A_RSSI_UPDATE_PREAM_OK	(1<<2)					// rssi	update by preamble detected
#define		CMT2310A_RSSI_UPDATE_SYNC_OK	(2<<2)					// rssi update by syncword detected
#define		CMT2310A_RSSI_UPDATE_PKT_DONE	(3<<2)					// rssi update by packet done

#define		CMT2310A_COLL_DET_EN			(1<<0)					// 0=disable,  	1=enable collision detect

#define 		CMT2310A_RX_RSSI_REG_01         0x63	 				// rf_rssi_reg 1, 	rssi_absolute_th[7:0]
#define		CMT2310A_RSSI_ABS_TH_REG		0x63

#define 		CMT2310A_RX_DOUT_REG_00         0x64					// rf_dout_reg, 	nc + nc + nc + dout_adjust_sel[2:0] + dout_adjust_mode + dout_adjust_en
#define		CMT2310A_DOUT_ADJUST_SEL_MASK	(7<<2)
#define		CMT2310A_DOUT_ADJUST_3_33		(0<<2)
#define		CMT2310A_DOUT_ADJUST_6_66		(1<<2)
#define		CMT2310A_DOUT_ADJUST_9_99		(2<<2)
#define		CMT2310A_DOUT_ADJUST_13_32		(3<<2)
#define		CMT2310A_DOUT_ADJUST_16_65		(4<<2)
#define		CMT2310A_DOUT_ADJUST_19_98		(5<<2)
#define		CMT2310A_DOUT_ADJUST_23_21		(6<<2)
#define		CMT2310A_DOUT_ADJUST_26_64		(7<<2)

#define		CMT2310A_DOUT_ADJUST_MODE		(1<<1)					// 0=+1 duty, 	1=-1 duty
#define		CMT2310A_DOUT_ADJUST_EN			(1<<0)					// 0=disable, 	1=enable dout adjust

#define 		CMT2310A_RX_ANTD_REG_00         0x67					// rf_ant_reg		nc + nc + nc + nc + ant_wait_pmb[1:0] + ant_sw_dis + ant_div_en
#define		CMT2310A_ANT_WAIT_PMB_MASK		(3<<2)
#define		CMT2310A_ANT_WAIT_1_5_UNIT		(0<<2)					// wait preamble size *1.5, when antenna calibrate
#define		CMT2310A_ANT_WAIT_2_UNIT		(1<<2)					// wait preamble size *2, when antenna calibrate
#define		CMT2310A_ANT_WAIT_2_5_UNIT		(2<<2)					// wait preamble size *2.5, when antenna calibrate
#define		CMT2310A_ANT_WAIT_3_UNIT		(3<<2)					// wait preamble size *3, when antenna calibrate

#define		CMT2310A_ANT_SW_DIS				(1<<1)					// 0=enable, 	1=bypass antenna switch
#define		CMT2310A_ANT_DIV_EN				(1<<0)					// 0=disable, 	1=enable antenna diversity

// --------------------------- CUS PAGE2 defines -----------------------------------------

#define 		CMT2310A_FREQ_CHANL_00          0x00
#define 		CMT2310A_FREQ_CHANL_01          0x01
#define 		CMT2310A_FREQ_CHANL_02          0x02
#define 		CMT2310A_FREQ_CHANL_03          0x03
#define 		CMT2310A_FREQ_CHANL_04          0x04
#define 		CMT2310A_FREQ_CHANL_05          0x05
#define 		CMT2310A_FREQ_CHANL_06          0x06
#define 		CMT2310A_FREQ_CHANL_07          0x07
#define 		CMT2310A_FREQ_CHANL_08          0x08
#define 		CMT2310A_FREQ_CHANL_09          0x09
#define 		CMT2310A_FREQ_CHANL_10          0x0A
#define 		CMT2310A_FREQ_CHANL_11          0x0B
#define 		CMT2310A_FREQ_CHANL_12          0x0C
#define 		CMT2310A_FREQ_CHANL_13          0x0D
#define 		CMT2310A_FREQ_CHANL_14          0x0E
#define 		CMT2310A_FREQ_CHANL_15          0x0F
#define 		CMT2310A_FREQ_CHANL_16          0x10
#define 		CMT2310A_FREQ_CHANL_17          0x11
#define 		CMT2310A_FREQ_CHANL_18          0x12
#define 		CMT2310A_FREQ_CHANL_19          0x13
#define 		CMT2310A_FREQ_CHANL_20          0x14
#define 		CMT2310A_FREQ_CHANL_21          0x15
#define 		CMT2310A_FREQ_CHANL_22          0x16
#define 		CMT2310A_FREQ_CHANL_23          0x17
#define 		CMT2310A_FREQ_CHANL_24          0x18
#define 		CMT2310A_FREQ_CHANL_25          0x19
#define 		CMT2310A_FREQ_CHANL_26          0x1A
#define 		CMT2310A_FREQ_CHANL_27          0x1B
#define 		CMT2310A_FREQ_CHANL_28          0x1C
#define 		CMT2310A_FREQ_CHANL_29          0x1D
#define 		CMT2310A_FREQ_CHANL_30          0x1E
#define 		CMT2310A_FREQ_CHANL_31          0x1F
#define 		CMT2310A_FREQ_CHANL_32          0x20
#define 		CMT2310A_FREQ_CHANL_33          0x21
#define 		CMT2310A_FREQ_CHANL_34          0x22
#define 		CMT2310A_FREQ_CHANL_35          0x23
#define 		CMT2310A_FREQ_CHANL_36          0x24
#define 		CMT2310A_FREQ_CHANL_37          0x25
#define 		CMT2310A_FREQ_CHANL_38          0x26
#define 		CMT2310A_FREQ_CHANL_39          0x27
#define 		CMT2310A_FREQ_CHANL_40          0x28
#define 		CMT2310A_FREQ_CHANL_41          0x29
#define 		CMT2310A_FREQ_CHANL_42          0x2A
#define 		CMT2310A_FREQ_CHANL_43          0x2B
#define 		CMT2310A_FREQ_CHANL_44          0x2C
#define 		CMT2310A_FREQ_CHANL_45          0x2D
#define 		CMT2310A_FREQ_CHANL_46          0x2E
#define 		CMT2310A_FREQ_CHANL_47          0x2F
#define 		CMT2310A_FREQ_CHANL_48          0x30
#define 		CMT2310A_FREQ_CHANL_49          0x31
#define 		CMT2310A_FREQ_CHANL_50          0x32
#define 		CMT2310A_FREQ_CHANL_51          0x33
#define 		CMT2310A_FREQ_CHANL_52          0x34
#define 		CMT2310A_FREQ_CHANL_53          0x35
#define 		CMT2310A_FREQ_CHANL_54          0x36
#define 		CMT2310A_FREQ_CHANL_55          0x37
#define 		CMT2310A_FREQ_CHANL_56          0x38
#define 		CMT2310A_FREQ_CHANL_57          0x39
#define 		CMT2310A_FREQ_CHANL_58          0x3A
#define 		CMT2310A_FREQ_CHANL_59          0x3B
#define 		CMT2310A_FREQ_CHANL_60          0x3C
#define 		CMT2310A_FREQ_CHANL_61          0x3D
#define 		CMT2310A_FREQ_CHANL_62          0x3E
#define 		CMT2310A_FREQ_CHANL_63          0x3F

#define			CMT2310A_CHIP_VERSION_00		0x40
#define			CMT2310A_CHIP_VERSION_01		0x41
#define			CMT2310A_CHIP_VERSION_02		0x42

// -------------------------------- Marco defines ----------------------------------------- 
#define 		VAL_BIT0                        0x01
#define 		VAL_BIT1                        0x02
#define 		VAL_BIT2                        0x04
#define 		VAL_BIT3                        0x08
#define 		VAL_BIT4                        0x10
#define 		VAL_BIT5                        0x20
#define 		VAL_BIT6                        0x40
#define 		VAL_BIT7                        0x80

#define 		BIT0                            0
#define 		BIT1                            1
#define 		BIT2                            2
#define 		BIT3                            3
#define 		BIT4                            4
#define 		BIT5                            5
#define 		BIT6                            6
#define 		BIT7                            7

//## CMT2310A_CTL_REG_00 registers
#define 		M_BOOT_MAIN                     VAL_BIT1
#define 		M_POWERUP                     	VAL_BIT0

//## CMT2310A_CTL_REG_01 registers
#define 		M_GO_RXFS                     	VAL_BIT5
#define 		M_GO_TXFS                     	VAL_BIT4
#define 		M_GO_RX                       	VAL_BIT3
#define 		M_GO_TX                       	VAL_BIT2
#define 		M_GO_READY                    	VAL_BIT1
#define 		M_GO_SLEEP                    	VAL_BIT0

//##  CMT2310A_CTL_REG_02 registers
#define 		M_MCU_GPIO_EN                   VAL_BIT2
#define 		M_ANT_DIV_MANU                  VAL_BIT1
#define 		M_ANT_SELECT                    VAL_BIT0

//## CMT2310A_CTL_REG_04 registers
#define 		M_TX_DIN_EN                  	VAL_BIT6
#define	 		M_GPIO1_SEL                    	(VAL_BIT5|VAL_BIT4|VAL_BIT3)
#define 		M_GPIO0_SEL                     (VAL_BIT2|VAL_BIT1|VAL_BIT0)

//## CMT2310A_CTL_REG_05 registers
#define 		M_TX_DIN_SEL                   	(VAL_BIT7|VAL_BIT6)
#define 		M_GPIO3_SEL                     (VAL_BIT5|VAL_BIT4|VAL_BIT3)
#define 		M_GPIO2_SEL                     (VAL_BIT2|VAL_BIT1|VAL_BIT0)

//## CMT2310A_CTL_REG_06 registers
#define 		M_DIG_CLKOUT_EN                	VAL_BIT6
#define 		M_GPIO5_SEL                     (VAL_BIT5|VAL_BIT4|VAL_BIT3)
#define 		M_GPIO4_SEL                     (VAL_BIT2|VAL_BIT1|VAL_BIT0)

//## CMT2310A_CTL_REG_07 registers
#define 		M_LFXO_PAD_EN                  	VAL_BIT5
#define 		M_API_STOP                     	VAL_BIT4
#define 		M_SPI_3W_EN                    	VAL_BIT3
#define 		M_NIRQ_SEL                     	(VAL_BIT2|VAL_BIT1|VAL_BIT0)

//## CMT2310A_CTL_REG_09 registers
#define 		M_API_CMD_FLAG                  VAL_BIT7
#define 		M_API_RESP                      (VAL_BIT6|VAL_BIT5|VAL_BIT4|VAL_BIT3|VAL_BIT2|VAL_BIT1|VAL_BIT0)

//## CMT2310A_CTL_REG_10 registers
#define 		M_STATE_IS_TX                   VAL_BIT5
#define 		M_STATE_IS_RX                   VAL_BIT4
#define 		M_STATE_IS_TFS                  VAL_BIT3
#define 		M_STATE_IS_RFS                  VAL_BIT2
#define 		M_STATE_IS_READY                VAL_BIT1
#define 		M_STATE_IS_SLEEP                VAL_BIT0

//## CMT2310A_CTL_REG_14 registers
#define 		M_RX_FIFO_FULL_RX_EN            VAL_BIT7
#define 		M_RX_FIFO_NMTY_RX_EN            VAL_BIT6
#define 		M_RX_FIFO_TH_RX_EN              VAL_BIT5
#define 		M_RX_FIFO_OVF_EN                VAL_BIT3
#define 		M_TX_FIFO_FULL_EN               VAL_BIT2
#define 		M_TX_FIFO_NMTY_EN               VAL_BIT1
#define 		M_TX_FIFO_TH_EN                 VAL_BIT0

//## CMT2310A_CTL_REG_15 registers
#define 		M_ANT_INSTR                     VAL_BIT0

//## CMT2310A_CTL_REG_16 registers
#define 		M_INT1_SEL                      (VAL_BIT5|VAL_BIT4|VAL_BIT3|VAL_BIT2|VAL_BIT1|VAL_BIT0)

//## CMT2310A_CTL_REG_17 registers
#define 		M_INT1_POLAR    	           	VAL_BIT7
#define 		M_INT2_POLAR                   	VAL_BIT6
#define 		M_INT2_SEL                      (VAL_BIT5|VAL_BIT4|VAL_BIT3|VAL_BIT2|VAL_BIT1|VAL_BIT0)

//## CMT2310A_CTL_REG_18 registers
#define 		M_SLEEP_TMO_EN                 	VAL_BIT7
#define 		M_RX_TMO_EN                    	VAL_BIT6
#define 		M_TX_DONE_EN                   	VAL_BIT5
#define 		M_PREAM_PASS_EN                	VAL_BIT4
#define 		M_SYNC_PASS_EN                 	VAL_BIT3
#define 		M_ADDR_PASS_EN                 	VAL_BIT2
#define 		M_CRC_PASS_EN                  	VAL_BIT1
#define 		M_PKT_DONE_EN                  	VAL_BIT0

//## CMT2310A_CTL_REG_19 registers
#define 		M_PD_FIFO                    	VAL_BIT6
#define 		M_FIFO_TH_8         	    	VAL_BIT5
#define 		M_FIFO_AUTO_CLR_RX_EN        	VAL_BIT4
#define 		M_FIFO_AUTO_RES_TX_EN        	VAL_BIT3
#define 		M_FIFO_TX_TEST_EN            	VAL_BIT2
#define 		M_FIFO_MERGE_EN              	VAL_BIT1
#define 		M_FIFO_TX_RX_SEL             	VAL_BIT0

//## CMT2310A_CTL_REG_21 registers
#define 		M_RSSI_PJD_VALID_EN             VAL_BIT6
#define 		M_OP_CMD_FAILED_EN              VAL_BIT5
#define 		M_RSSI_COLL_EN                  VAL_BIT4
#define 		M_PKT_ERR_EN                    VAL_BIT3
#define 		M_LBD_STATUS_EN                 VAL_BIT2
#define 		M_LBD_STOP_EN                   VAL_BIT1
#define 		M_LD_STOP_EN                    VAL_BIT0

//## CMT2310A_CTL_REG_22 registers
#define 		M_FREQ_HOP_MANU_EN              VAL_BIT7
#define 		M_RX_HOP_PERSIST                VAL_BIT5
#define 		M_FREQ_SW_STATE                 VAL_BIT6
#define 		M_TX_DATA_INV                   VAL_BIT4
#define 		M_PA_DIFF_SEL                   VAL_BIT3
#define 		M_TRX_SWT_INV                   VAL_BIT2
#define 		M_TRX_SWT_EN                    VAL_BIT1
#define 		M_ANT_LOCK_EN                   VAL_BIT0

//## CMT2310A_CTL_REG_23 registers
#define 		M_API_DONE_EN                 	VAL_BIT7
#define 		M_CCA_STATUS_EN               	VAL_BIT6
#define 		M_CSMA_DONE_EN                 	VAL_BIT5
#define 		M_TX_DC_DONE_EN                	VAL_BIT4
#define 		M_ACK_RECV_FAILED_EN          	VAL_BIT3
#define 		M_TX_RESEND_DONE_EN            	VAL_BIT2
#define 		M_NACK_RECV_EN                	VAL_BIT1
#define 		M_SEQ_MATCH_EN                	VAL_BIT0

//## CMT2310A_CTL_REG_24 registers
#define 		M_SLEEP_TMO_FLG              	VAL_BIT5
#define 		M_RX_TMO_FLG                 	VAL_BIT4
#define 		M_TX_DONE_FLG                	VAL_BIT3
#define 		M_TX_DONE_CLR                	VAL_BIT2
#define 		M_RX_TMO_CLR                 	VAL_BIT1
#define 		M_SLEEP_TMO_CLR              	VAL_BIT0

//## CMT2310A_CTL_REG_25 registers
#define 		M_PREAM_PASS_CLR               	VAL_BIT4
#define 		M_SYNC_PASS_CLR                	VAL_BIT3
#define 		M_ADDR_PASS_CLR                	VAL_BIT2
#define 		M_CRC_PASS_CLR                 	VAL_BIT1
#define 		M_PKT_DONE_CLR                 	VAL_BIT0

//## CMT2310A_CTL_REG_26 registers
#define 		M_SYNC1_PASS_FLG                VAL_BIT5
#define 		M_PREAM_PASS_FLG                VAL_BIT4
#define 		M_SYNC_PASS_FLG                 VAL_BIT3
#define 		M_ADDR_PASS_FLG                 VAL_BIT2
#define 		M_CRC_PASS_FLG                  VAL_BIT1
#define 		M_PKT_DONE_FLG                  VAL_BIT0

//## CMT2310A_CTL_REG_27 registers
#define 		M_TX_FIFO_RESTORE             	VAL_BIT2
#define 		M_RX_FIFO_CLR                 	VAL_BIT1
#define 		M_TX_FIFO_CLR                 	VAL_BIT0

//## CMT2310A_CTL_REG_28 registers
#define 		M_RX_FIFO_FULL_FLG            	VAL_BIT7
#define 		M_RX_FIFO_NMTY_FLG            	VAL_BIT6
#define 		M_RX_FIFO_TH_FLG              	VAL_BIT5
#define 		M_RX_FIFO_OVF_FLG             	VAL_BIT3
#define 		M_TX_FIFO_FULL_FLG            	VAL_BIT2
#define 		M_TX_FIFO_NMTY_FLG            	VAL_BIT1
#define 		M_TX_FIFO_TH_FLG              	VAL_BIT0

//## CMT2310A_CTL_REG_29 registers
#define 		M_ANT_LOCK_CLR                    VAL_BIT4
#define 		M_OP_CMD_FAILED_CLR               VAL_BIT3
#define 		M_RSSI_COLL_CLR                   VAL_BIT2
#define 		M_PKT_ERR_CLR                     VAL_BIT1
#define 		M_LBD_STATUS_CLR                  VAL_BIT0

//## CMT2310A_CTL_REG_30 registers
#define 		M_ANT_LOCK_FLG                    VAL_BIT4
#define 		M_OP_CMD_FAILED_FLG               VAL_BIT3
#define 		M_RSSI_COLL_FLG                   VAL_BIT2
#define 		M_PKT_ERR_FLG                     VAL_BIT1
#define 		M_LBD_STATUS_FLG                  VAL_BIT0

//## CMT2310A_CTL_REG_31 registers
#define 		M_API_DONE_CLR                    VAL_BIT7
#define 		M_CCA_STATUS_CLR                  VAL_BIT6
#define 		M_CSMA_DONE_CLR                   VAL_BIT5
#define 		M_TX_DC_DONE_CLR                  VAL_BIT4
#define 		M_ACK_RECV_FAILED_CLR             VAL_BIT3
#define 		M_TX_RESEND_DONE_CLR              VAL_BIT2
#define 		M_NACK_RECV_CLR                   VAL_BIT1
#define 		M_SEQ_MATCH_CLR                   VAL_BIT0

//##  CMT2310A_CTL_REG_32 registers
#define 		M_API_DONE_FLG                    VAL_BIT7
#define 		M_CCA_STATUS_FLG                  VAL_BIT6
#define 		M_CSMA_DONE_FLG                   VAL_BIT5
#define 		M_TX_DC_DONE_FLG                  VAL_BIT4
#define 		M_ACK_RECV_FAILED_FLG             VAL_BIT3
#define 		M_TX_RESEND_DONE_FLG              VAL_BIT2
#define 		M_NACK_RECV_FLG                   VAL_BIT1
#define 		M_SEQ_MATCH_FLG                   VAL_BIT0

//## CMT2310A_CTL_REG_126 registers
#define 		M_HV_PAGE_SEL                     (VAL_BIT7|VAL_BIT6)

//##  CMT2310A STATE
#define 		STATE_IS_IDLE                	0x00
#define 		STATE_IS_SLEEP               	0x81
#define 		STATE_IS_READY               	0x82
#define 		STATE_IS_RFS                 	0x84
#define 		STATE_IS_TFS                 	0x88
#define 		STATE_IS_RX                  	0x90
#define 		STATE_IS_TX                  	0xA0

//##  GPIO0_SEL
#define 		GPIO0_SEL_DOUT                	0x00
#define 		GPIO0_SEL_INT1                	0x01
#define 		GPIO0_SEL_INT2                	0x02
#define 		GPIO0_SEL_DCLK                	0x03

//##  GPIO1_SEL
#define 		GPIO1_SEL_DCLK                	0x00
#define 		GPIO1_SEL_INT1                	0x01
#define 		GPIO1_SEL_INT2                	0x02
#define 		GPIO1_SEL_DOUT                	0x03

//##  GPIO2_SEL
#define 		GPIO2_SEL_INT1                 	0x00
#define 		GPIO2_SEL_INT2                 	0x01
#define 		GPIO2_SEL_DCLK                 	0x02
#define 		GPIO2_SEL_DOUT                 	0x03

//##  GPIO3_SEL
#define 		GPIO3_SEL_INT2                	0x00
#define 		GPIO3_SEL_INT1                	0x01
#define 		GPIO3_SEL_DCLK                	0x02
#define 		GPIO3_SEL_DOUT                	0x03
#define 		GPIO3_SEL_DIN                 	0x05

//##  GPIO4_SEL
#define 		GPIO4_SEL_DOUT                	0x00
#define 		GPIO4_SEL_INT1                	0x01
#define 		GPIO4_SEL_INT2                	0x02
#define 		GPIO4_SEL_DCLK                	0x03
#define 		GPIO4_SEL_DIN                 	0x05

//## GPIO5_SEL
#define 		GPIO5_SEL_RSTN                	0x00
#define 		GPIO5_SEL_INT1                	0x01
#define 		GPIO5_SEL_INT2                	0x02
#define 		GPIO5_SEL_DOUT                	0x03
#define 		GPIO5_SEL_DCLK                	0x04

//## TX_DIN_SEL
#define 		TX_DIN_SEL_GPIO3              	0x00
#define 		TX_DIN_SEL_GPIO4              	0x01
#define 		TX_DIN_SEL_NIRQ               	0x02

//## NIRQ_SEL
#define 		NIRQ_SEL_INT1                 	0x00
#define 		NIRQ_SEL_INT2                 	0x01
#define 		NIRQ_SEL_DCLK                 	0x02
#define 		NIRQ_SEL_DOUT                 	0x03
#define 		NIRQ_SEL_DIN                  	0x04

//## INT1/2_SEL
#define 		INT_SRC_MIX                     0x00
#define 		INT_SRC_ANT_LOCK                0x01
#define 		INT_SRC_RSSI_PJD_VALID          0x02
#define 		INT_SRC_PREAM_PASS              0x03
#define 		INT_SRC_SYNC_PASS               0x04
#define 		INT_SRC_ADDR_PASS               0x05
#define 		INT_SRC_CRC_PASS                0x06
#define 		INT_SRC_PKT_OK                  0x07
#define 		INT_SRC_PKT_DONE                0x08
#define 		INT_SRC_SLEEP_TMO               0x09
#define 		INT_SRC_RX_TMO                  0x0A
#define 		INT_SRC_RX_FIFO_NMTY            0x0B
#define 		INT_SRC_RX_FIFO_TH              0x0C
#define 		INT_SRC_RX_FIFO_FULL            0x0D
#define 		INT_SRC_RX_FIFO_WBYTE           0x0E
#define 		INT_SRC_RX_FIFO_OVF             0x0F
#define 		INT_SRC_TX_DONE                 0x10
#define 		INT_SRC_TX_FIFO_NMTY            0x11
#define 		INT_SRC_TX_FIFO_TH              0x12
#define 		INT_SRC_TX_FIFO_FULL            0x13
#define 		INT_SRC_STATE_IS_READY          0x14
#define 		INT_SRC_STATE_IS_FS             0x15
#define 		INT_SRC_STATE_IS_RX             0x16
#define 		INT_SRC_STATE_IS_TX             0x17
#define 		INT_SRC_LBD_STATUS              0x18
#define 		INT_SRC_API_CMD_FAILED          0x19
#define 		INT_SRC_API_DONE                0x1A
#define 		INT_SRC_TX_DC_DONE              0x1B
#define 		INT_SRC_ACK_RECV_FAILED         0x1C
#define 		INT_SRC_TX_RESEND_DONE          0x1D
#define 		INT_SRC_NACK_RECV               0x1E
#define 		INT_SRC_SEQ_MATCH               0x1F
#define 		INT_SRC_CSMA_DONE               0x20
#define 		INT_SRC_CCA_STATUS              0x21

#endif

//******************************************************************************
//* EOF (not truncated)
//******************************************************************************
#endif /* RADIOCMT2310_CMT2310A_REG_H_ */
