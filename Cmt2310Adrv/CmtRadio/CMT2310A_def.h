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
 * @file    CMT2310A_def.h
 * @brief   CMT2310A transceiver RF chip driver
 *
 * @version 1.0
 * @date    Dec 7 2021
 * @author  CMOSTEK R&D
 */

#ifndef __CMT2310A_DEF_H

#define __CMT2310A_DEF_H

//Exit State
#define		EXIT_TO_SLEEP					1
#define		EXIT_TO_READY					2
#define		EXIT_TO_TFS						3
#define		EXIT_TO_TX						4
#define		EXIT_TO_RFS						5
#define		EXIT_TO_RX						6

//Prefxi select
#define		TX_PREFIX_SEL_0					0
#define		TX_PREFIX_SEL_1					1
#define		TX_PREFIX_SEL_PREAMBLE			2

//## interrupt source config
typedef union
{
    struct
    {
        unsigned char PKT_DONE_EN :1;		//pkt_done_en				//bit0
        unsigned char CRC_PASS_EN :1;	 	//crc_pass_en				//bit1
        unsigned char ADDR_PASS_EN :1;		//node_address_pass_en		//bit2
        unsigned char SYNC_PASS_EN :1;		//sync_pass_en				//bit3
        unsigned char PREAM_PASS_EN :1;		//preamble_pass_en			//bit4
        unsigned char TX_DONE_EN :1;		//tx_done_en				//bit5
        unsigned char RX_TOUT_EN :1;		//rx_timeout_en				//bit6
        unsigned char SLP_TOUT_EN :1;		//sleep_timeout_en			//bit7

        unsigned char LD_STOP_EN :1;		//ld_stop_en(PLL lock detect)//bit0
        unsigned char LBD_STOP_EN :1;		//lbd_stop_en				//bit1
        unsigned char LBD_STAT_EN :1;		//lbd_status_en				//bit2
        unsigned char PKT_ERR_EN :1;		//pkt_err_en				//bit3
        unsigned char RSSI_COLL_EN :1;		//rssi_collision_en			//bit4
        unsigned char OP_CMD_FAILED_EN :1;		//op_cmd_failed_en			//bit5
        unsigned char RSSI_PJD_EN :1;		//rssi_pjd_valid_en			//bit6
        unsigned char RESV_2_1 :1;		//reserve, don't used		//bit7

        unsigned char SEQ_MATCH_EN :1;		//seq_match_en				//bit0
        unsigned char NACK_RECV_EN :1;		//nack_recv_en				//bit1
        unsigned char TX_RESEND_DONE_EN :1;		//tx_resend_done_en			//bit2
        unsigned char ACK_RECV_FAILED_EN :1;		//ack_recv_failed_en		//bit3
        unsigned char TX_DC_DONE_EN :1;		//tx_dc_done_en				//bit4
        unsigned char CSMA_DONE_EN :1;		//csma_max_en				//bit5
        unsigned char CCA_STAT_EN :1;		//cca_status_en				//bit6
        unsigned char API_DONE_EN :1;		//api_done_en				//bit7

        unsigned char TX_FIFO_TH_EN :1;		//tx_fifo_threshold_en		//bit0
        unsigned char TX_FIFO_NMTY_EN :1;		//tx_fifo_non_empty_en		//bit1
        unsigned char TX_FIFO_FULL_EN :1;		//tx_fifo_full_en 			//bit2
        unsigned char RX_FIFO_OVF_EN :1;		//rx_fifo_overflow_en		//bit3
        unsigned char RESV_4_1 :1;		//reserve, don't used		//bit4
        unsigned char RX_FIFO_TH_EN :1;		//rx_fifo_threshold_en		//bit5
        unsigned char RX_FIFO_NMTY_EN :1;		//rx_fifo_non_empty_en		//bit6
        unsigned char RX_FIFO_FULL_EN :1;		//rx_fifo_full_en  			//bit7
    } _BITS;
    struct
    {
        unsigned char INT_CTL1_REG;					//CTL_REG_18, 0x12
        unsigned char INT_CTL2_REG;					//CTL_REG_21, 0x15
        unsigned char INT_CTL3_REG;					//CTL_REG_23, 0x17
        unsigned char INT_CTL4_REG;					//CTL_REG_14, 0x0E
    } _BYTE;
} INT_SRC_CFG;

//## interrupt flag
typedef union
{
    struct
    {
        unsigned char RESV_3 :3;		//reserve, don't used		//bit0/1/2
        unsigned char TX_DONE_FLG :1;	 	//tx_done_flag				//bit3
        unsigned char RX_TMO_FLG :1;		//rx_timer_timeout_flag		//bit4
        unsigned char SLEEP_TMO_FLG :1;		//sleep_timer_timeout_flag	//bit5
        unsigned char RESV_2 :2;		//reserve, don't used		//bit6/7

        unsigned char PKT_DONE_FLG :1;		//pkt_done_flag				//bit0
        unsigned char CRC_PASS_FLG :1;		//crc_pass_flag				//bit1
        unsigned char ADDR_PASS_FLG :1;		//addr_pass_flag			//bit2
        unsigned char SYNC_PASS_FLG :1;		//sync_pass_flag			//bit3
        unsigned char PREAM_PASS_FLG :1;		//preamble_pass_flag		//bit4
        unsigned char SYNC1_PASS_FLG :1;		//sync1_pass_flag			//bit5
        unsigned char RESV_2_2 :2;		//reserve, don't used		//bit6/7

        unsigned char LBD_STATUS_FLG :1;		//lbd_status_flag			//bit0
        unsigned char PKT_ERR_FLG :1;		//pkt_err_flag				//bit1
        unsigned char RSSI_COLL_FLG :1;		//rssi_collision_flag		//bit2
        unsigned char OP_CMD_FAILED_FLG :1;		//op_cmd_failed_flag		//bit3
        unsigned char ANT_LOCK_FLG :1;		//ant_lock_flag				//bit4
        unsigned char RESV_3_3 :3;		//reserve, don't used		//bit5/6/7

        unsigned char SEQ_MATCH_FLG :1;		//seq_match_flag			//bit0
        unsigned char NACK_RECV_FLG :1;		//nack_recv_flag			//bit1
        unsigned char TX_RESEND_DONE_FLG :1;		//tx_resend_max_flag		//bit2
        unsigned char ACK_RECV_FAILED_FLG :1;		//ack_recv_failed_flag		//bit3
        unsigned char TX_DC_DONE_FLG :1;		//tx_dc_done_flag			//bit4
        unsigned char CSMA_DONE_FLG :1;		//csma_done_flag			//bit5
        unsigned char CCA_STATUS_FLG :1;		//cca_status_flag			//bit6
        unsigned char API_DONE_FLG :1;		//api_done_flag				//bit7
    } _BITS;
    struct
    {
        unsigned char INT_FLAG1_REG;				//CTL_REG_24, 0x18
        unsigned char INT_FLAG2_REG;				//CTL_REG_26, 0x1A
        unsigned char INT_FLAG3_REG;				//CTL_REG_30, 0x1E
        unsigned char INT_FLAG4_REG;				//CTL_REG_32, 0x20
    } _BYTE;
} INT_SRC_FLG;

//## interrupt clear
typedef union
{
    struct
    {
        unsigned char TX_DONE_CLR :1;		//tx_done_clr					//bit0
        unsigned char RX_TMO_CLR :1;		//rx_timer_timeout_clr			//bit1
        unsigned char SLEEP_TMO_CLR :1;		//sleep_timer_timeout_clr		//bit2
        unsigned char RESV_5 :5;		//reserve, don't used			//bit3-7

        unsigned char PKT_DONE_CLR :1;		//pkt_done_clr            		//bit0
        unsigned char CRC_PASS_CLR :1;		//crc_pass_clr                  //bit1
        unsigned char ADDR_PASS_CLR :1;		//addr_pass_clr                 //bit2
        unsigned char SYNC_PASS_CLR :1;		//sync_pass_clr                 //bit3
        unsigned char PREAM_PASS_CLR :1;		//preamble_pass_clr				//bit4
        unsigned char RESV_3 :3;		//reserve, don't used			//bit5/6/7

        unsigned char LBD_STAT_CLR :1;		//lbd_status_clr                //bit0
        unsigned char PKT_ERR_CLR :1;		//packet_err_clr                //bit1
        unsigned char RSSI_COLL_CLR :1;		//rssi_collision_clr            //bit2
        unsigned char OP_CMD_FAILED_CLR :1;		//op_cmd_failed_clr             //bit3
        unsigned char ANT_LOCK_CLR :1;		//antenna_lock_clr              //bit4
        unsigned char RESV_2_3 :3;		//reserve, don't used			//bit5/6/7

        unsigned char SEQ_MATCH_CLR :1;		//seq_match_clr                	//bit0
        unsigned char NACK_RECV_CLR :1;		//nack_recv_clr                 //bit1
        unsigned char TX_RESEND_DONE_CLR :1;		//tx_resend_done_clr            //bit2
        unsigned char ACK_RECV_FAILED_CLR :1;		//ack_recv_failed_clr           //bit3
        unsigned char TX_DC_DONE_CLR :1;		//tx_dc_done_clr                //bit4
        unsigned char CSMA_DONE_CLR :1;		//csma_done_clr					//bit5
        unsigned char CCA_STATUS_CLR :1;		//cca_status_clr                //bit6
        unsigned char API_DONE_CLR :1;		//api_done_clr                  //bit7
    } _BITS;
    struct
    {
        unsigned char INT_CLR1_REG;					//CTL_REG_24, 0x18
        unsigned char INT_CLR2_REG;					//CTL_REG_25, 0x19
        unsigned char INT_CLR3_REG;					//CTL_REG_29, 0x1D
        unsigned char INT_CLR4_REG;					//CTL_REG_31, 0x1F
    } _BYTE;
} INT_SRC_CLR;

//## fifo status flag
typedef union
{
    struct
    {
        unsigned char TX_FIFO_TH_FLG :1;	 	//tx_fifo_threshold_flag		//bit0
        unsigned char TX_FIFO_NMTY_FLG :1;		//tx_fifo_non_empty_flag		//bit1
        unsigned char TX_FIFO_FULL_FLG :1;		//tx_fifo_full_flag				//bit2
        unsigned char RX_FIFO_OVF_FLG :1;		//rx_fifo_overflow_flag			//bit3
        unsigned char RESV_1 :1;		//reserve, don't used			//bit4
        unsigned char RX_FIFO_TH_FLG :1;		//rx_fifo_threshold_flag		//bit5
        unsigned char RX_FIFO_NMTY_FLG :1;		//rx_fifo_non_empty_flag		//bit6
        unsigned char RX_FIFO_FULL_FLG :1;		//rx_fifo_full_flag				//bit7
    } _BITS;
    unsigned char FIFO_FLG_REG;						//CTL_REG_28, 0x1c
} FIFO_STATUS_FLG;

//## packet preamble config
typedef struct										//Packet Preamble
{
    unsigned char PREAM_LENG_UNIT;				//0=preamble unit as 8bits,   !0=preamble unit as 4bits (nibble mode)
    unsigned char PREAM_VALUE;						//preamble value
    unsigned char RX_PREAM_SIZE;					//rx preamble length, range: 0-31, 0==disable rx preamble detect
    unsigned short int TX_PREAM_SIZE;//tx preamble length					//## note: for arm mcu unsigned short int is 16bits
} PREAMBLE_CFG;

//## packet syncword config
typedef struct
{
    union
    {
        struct
        {
            unsigned char SYNC_MAN_EN :1;	 	//sync word manchester coding enable
            unsigned char SYNC_SIZE :3;		// n+1 bytes
            unsigned char SYNC_TOL :3;		// n bits tolerence error
            unsigned char SYNC_MODE_SEL :1;		// 0: compatible S2LP;  1: compatible 802.15.4
        } _BITS;
        unsigned char SYNC_CFG_REG;					//CTL_REG_44, 0x2c
    } SYN_CFG_u;
    unsigned char SYNC_VALUE[8];					//CTL_REG_45-CTL_REG_53, 0x2D-0x34
    unsigned char SYNC_FEC_VALUE[8];				//CTL_REG_53-CTL_REG_60, 0x35-0x3C
    unsigned char SYNC_VALUE_SEL;					// 0: select SYN_VAL;  !0: select SYN_FEC_VAL
} SYNC_CFG;

//## packet node address config
typedef struct
{
    union
    {
        struct
        {
            unsigned char ADDR_DET_MODE :2;	//node address mode: 0,disable;  1,only match;  2,add all 0;  3,add all 1 & 0
            unsigned char ADDR_SIZE :2;		// n+1 bytes
            unsigned char ADDR_ERR_MASK :1;	// 0: trigger PKT_ERR flag, when node address not match, and reset decode;   1: non-reset decode, when node address not match
            unsigned char ADDR_FREE_EN :1;		// 0: disable;   1: enable node address match as stand-alone working
            unsigned char ADDR_SPLIT_MODE :1;// 0: disable, dest_addr==node_addr;    1: enable, node_addr==src_addr+des_addr
            unsigned char RESV_1 :1;
        } _BITS;
        unsigned char ADDR_CFG_REG;					//CTL_REG_64, 0x40
    } ADDR_CFG_u;
    unsigned char SRC_ADDR[2];						//CTL_REG_65, CTL_REG_66
    unsigned char DEST_ADDR[2];						//CTL_REG_67, CTL_REG_68
    unsigned char SRC_BITMASK[2];					//CTL_REG_69, CTL_REG_70
    unsigned char DEST_BITMASK[2];					//CTL_REG_71, CTL_REG_72
} ADDR_CFG;

//## packet crc config
typedef struct
{
    union
    {
        struct
        {
            unsigned char CRC_EN :1;	// 0:disable CRC;                       1:enable CRC
            unsigned char CRC_BIT_ORDER :1;	// 0:crc result MSB output first;  		1:crc result LSB output first; 		active unit is byte
            unsigned char CRC_REFIN :1;	// 0:normal input;                  	1:inverse input;
            unsigned char CRC_RANGE :1;	// 0:whole payload;                 	1:only data
            unsigned char CRC_BIT_INV :1;	// 0:crc result non-invert;	    		1:crc result all bits invert
            unsigned char CRC_BYTE_SWAP :1;	// 0:crc result HighByte output first;  1:crc result LowByte output first
            unsigned char CRC_SIZE :2;	// 0:crc8;  1:crc16;  2:crc24;  3:crc32
            unsigned char CRC_REFOUT :1;	// 0:result whole bit MSB->LSB   		1:result whole bit LSB->MSB
            unsigned char CRCERR_CLR_FIFO_EN :1;// 0:disable                            1:enable when crc error, clear fifo function
            unsigned char RESV_6 :6;
        } _BITS;
        unsigned short int CRC_CFG_REG;				//CTL_REG_73, 0x49 & CTL_REG_82[7] & CTL_REG_84[7]
    } CRC_CFG_u;
    union					//CTL_REG_74-CTL_REG_77, 0x4A-0x4D			//## note: for arm mcu unsigned int is 32bits
    {
        unsigned char u8_SEED[4];
        unsigned int u32_SEED;
    } CRC_SEED_u;
    union					//CTL_REG_78-CTL_REG_81, 0x4E-0x51			//## note: for arm mcu unsigned int is 32bits
    {
        unsigned char u8_POLY[4];
        unsigned int u32_POLY;
    } CRC_POLY_u;
} CRC_CFG;

//## packet encode/decode format config
typedef struct
{
    union
    {
        struct
        {
            unsigned char MANCH_EN :1;	// 0:disable manchester coding;			1:enable manchester coding
            unsigned char MANCH_TYPE :1;	// 0: 2'b01=1, 2'b10=0;					1: 2'b01=0, 2'b10=1
            unsigned char WHITEN_EN :1;	// 0:disable whiten coding; 			1:enable whiten coding
            unsigned char WHITEN_TYPE :2;	// 0:PN9-CCITT,  1:PN9-IBM,   2:PN7,    3:NA
            unsigned char WHITEN_SEED_TYP :1;// 0:compatible A7139;					1:customer seed;    	  note: when select PN7 active
            unsigned char RESV_2 :2;
            unsigned char FEC_EN :1;	// 0:disable FEC Coding;				1:enable FEC Coding
            unsigned char FEC_RSC_NRNSC_SEL :1;	// 0:RSC Mode;							1:NRNSC Mode
            unsigned char RESV_5 :5;
            unsigned char FEC_TICC :1;	// 0:ui invert ouput in FEC calucate;	1:ui non-invert ouput in FEC calucate
        } _BITS;
        unsigned short int CODING_CFG_REG;			// CTL_REG_82, 0x52 & CTL_REG_93, 0x5D
    } CODING_FORMAT_CFG_u;
    unsigned short int WHITEN_SEED;					// CTL_REG_83, 0x53 & CTL_REG_82[6]
    unsigned short int FEC_PAD_CODE;				// CTL_REG_94, 0x5E & CTL_REG_93[6:2], 0x5D
} CODING_FORMAT_CFG;

//## frame struct config
typedef struct
{
    unsigned char DATA_MODE;						//0: direct mode;   		2: packet mode;  	 1&3: na
    union
    {
        struct
        {
            unsigned char PKT_TYPE :1;	//0: fixed length;			1: variable length
            unsigned char PAYLOAD_BIT_ORDER :1;	//0: MSB output first;		1: LSB output first
            unsigned char ADDR_LEN_CONF :1;	//0: node_addr+length;		1: length+node_addr
            unsigned char RESV_1 :1;
            unsigned char PAGGYBACKING_EN :1;	//0: disable 				1: enable
            unsigned char LENGTH_SIZE :1;	//0: length range 1 byte;   1: length range 2 bytes
            unsigned char RESV_1_2 :1;
            unsigned char INTERLEAVE_EN :1;	//0: disable interleave		1: enable interleave
        } _BITS;
        unsigned char FRAME_CFG1_REG;				//CTL_REG_63, 0x3F
    } FRAME_CFG1_u;
    union
    {
        struct
        {
            unsigned char TX_PREFIX_TYPE :2;	//0:transmit_0;  	1:transmit_1; 		2:transmit_preamble		3:NA
            unsigned char SEQNUM_EN :1;	//0:disable;		1:enable;     note: SEQ_NUM is the same with FCS1
            unsigned char SEQNUM_AUTO_INC :1; 	//0:disable SEQNUM increase;	1:enable
            unsigned char SEQNUM_SIZE :1;	//0:1byte for SEQNUM;			1:2bytes for SEQNUM
            unsigned char SEQNUM_MACH_EN :1;//0:disable;					1:enable compare with local SEQNUM, when TX_ACK enable
            unsigned char FCS2_EN :1;	//0:not include FCS2 filed;		1:include FCS2 filed
            unsigned char RESV_1 :1;
        } _BITS;
        unsigned char FRAME_CFG2_REG;				//CTL_REG_84, 0x54
    } FRAME_CFG2_u;

    unsigned short int TX_PKT_NUM;					//CTL_REG_85, 0x55[7:0] & CTL_REG_86, 0x56[15:8]
    unsigned short int SEQNUM_TX_IN;				//CTL_REG_88/87, 0x58[15:8], 0x57[7:0]
    unsigned short int SEQNUM_TX_CURRENT_OUT;		//CTL_REG_39/38, 0x27[15:8]/0x26[7:0]
    unsigned char TX_PKT_GAP;					//CTL_REG_89, 0x59
    unsigned char FCS2_TX_IN;					//CTL_REG_91, 0x5B
    unsigned char FCS2_RX_OUT;					//CTL_REG_92, 0x5C
    unsigned short int PAYLOAD_LENGTH;				//CTL_REG_62/61, 0x3E[15:8], 0x3D[7:0]
} FRAME_CFG;

//## Wi-SUN V1.0 packet format compatible
typedef union
{
    struct
    {
        unsigned char WISUN_DW :1;	 	// 0:not whiten, 						1: whiten
        unsigned char WISUN_FCS :1;		// 0:CRC-32 for PSDU,   				1: CRC-16 for PSDU
        unsigned char RESV_2 :2;		//
        unsigned char WISUN_MS :1;		// default for 0
        unsigned char WHITEN_WISUN :1;		// 0: for normal used, 					1: for Wi-SUN whitenning
        unsigned char WISUN_ALLIN :1;// 0: WISUN_FCS & WISUN_DW not active, 	1: FCS & DW depend on WISUN_FCS & WISUN_DW,
        unsigned char LENGTH_MODE :1;// 0: for normal used, 					1: for Wi-SUN used, PSDU length filed is 11bits
    } _BITS;
    unsigned char WI_SUN_REG;						//CTL_REG_111, 0x6F
} WI_SUN_CFG;

//## working mode
typedef struct
{
    union
    {
        struct
        {
            unsigned char TX_DC_EN :1;	//0:disable TxDutyCycle;	 1:enable
            unsigned char TX_DC_PERSIST_EN :1;	//0:run TX_DC_TIMES;		1:always run until this bit set 0
            unsigned char TX_ACK_EN :1;	//0:disable				  	1:enable
            unsigned char TX_AUTO_HOP_EN :1;	//0:disable 				1:enable tx frequency auto hopping
            unsigned char TX_EXIT_STATE :3;	//1:Sleep, 2:Ready, 3:TFS, 4:TX, 5:RFS, 6:RX, other Sleep
            unsigned char RESV :1;
        } _BITS;
        unsigned char WORK_MODE_CFG1_REG;			//CTL_REG_96, 0x60
    } WORK_MODE_CFG1_u;

    union
    {
        struct
        {
            unsigned char RX_DC_EN :1;	//0:disable RxDutyCycle;	1:enable
            unsigned char RX_AUTO_HOP_EN :1;	//0:disable RxAutoHop;		1:enable
            unsigned char RX_ACK_EN :1;	//0:disable					1:enable
            unsigned char RX_TIMER_EN :1;	//0:disable					1:enable
            unsigned char RX_EXIT_STATE :3;	//1:Sleep, 2:Ready, 3:TFS, 4:TX, 5:RFS, 6:RX, other Sleep
            unsigned char CSMA_EN :1;	//0:disable					1:enable
        } _BITS;
        unsigned char WORK_MODE_CFG2_REG;			//CTL_REG_97, 0x61
    } WORK_MODE_CFG2_u;

    union
    {
        struct
        {
            unsigned char SLP_MODE :4;	//14 items for select
            unsigned char RX_HOP_SLP_MODE :3;	//7 items for select
            unsigned char PKT_DONE_EXIT_EN :1;	//0:keep on current state;		1:depends on RX_EXIT_STATE
        } _BITS;
        unsigned char WORK_MODE_CFG3_REG;			//CTL_REG_98, 0x62
    } WORK_MODE_CFG3_u;

    union
    {
        struct
        {
            unsigned char LFCLK_OUT_EN :1;	//0:disable;	1:enable LFCLK output to GPIO4
            unsigned char LFCLK_SEL :1;	//0:LFOSC;		1:LFXO (32768Hz)
            unsigned char RESV_1 :1;	//
            unsigned char SLEEP_TIMER_EN :1;	//0:disable		1:enable
            unsigned char TIMER_RAND_MODE :2;//00:random R,   01:random M,   10:both M&R random,   11:depends on config value
            unsigned char RESV_2 :2;
        } _BITS;
        unsigned char WORK_MODE_CFG4_REG;			//CTL_REG_105, 0x69
    } WORK_MODE_CFG4_u;

    union
    {
        struct
        {
            unsigned char CSMA_CCA_WIN_SEL :2;	//00:32symbol,  01:64symbol,  10:128symbol,   11:256symbol
            unsigned char CSMA_CCA_INT_SEL :2;	//00:PJD, 		01:RSSI,   	  10:PJD & RSSI,  11:NA
            unsigned char CSMA_PERSIST_EN :1;//0:auto exit when reach max & channel still busy, 	1:keep on work until send out
            unsigned char CSMA_CCA_MODE :3;	//000:idle,     001:>=1 RSSI,          010:>=1 PJD,          011:>=1 RSSI or PJD,
                                            //100:>=1 SYNC, 101:>=1 SYNC or RSSI,  110:>=1 SYNC or PJD,  111:>=1 SYNC or PJD or RSSI
        } _BITS;
        unsigned char WORK_MODE_CFG5_REG;			//CTL_REG_106, 0x6A
    } WORK_MODE_CFG5_u;

    union
    {
        struct
        {
            unsigned char RESV_6 :6;
            unsigned char RX_HOP_PERSIST :1;	//
            unsigned char FREQ_HOP_MANU_EN :1;	//0:auto hopping mode;  1:hopping by manual

        } _BITS;
        unsigned char WORK_MODE_CFG6_REG;
    } WORK_MODE_CFG6_u;							//CTL_REG_22, 0x16

    unsigned char FREQ_CHANL_NANU;				//CTL_REG_3,  0x03, frequency hopping by manual, set channel number
    unsigned char FREQ_DONE_TIMES;				//CTL_REG_11, 0x0B, auto frequency hopping times have done
    unsigned char FREQ_SPACE;					//CTL_REG_12, 0x0C, auto frequency channel interval
    unsigned char FREQ_TIMES;					//CTL_REG_13, 0x0D, auto frequency hopping set range, 1-64

    unsigned short int SLEEP_TIMER_M;				//CTL_REG_100[7:5]|CLT_REG_99, 11bits, 0x64,0x63
    unsigned char SLEEP_TIMER_R;				//CTL_REG_100[4:0], 0x64
    unsigned short int RX_TIMER_T1_M;				//CTL_REG_102[7:5]|CLT_REG_101, 11bits, 0x66, 0x65
    unsigned char RX_TIMER_T1_R;				//CTL_REG_102[4:0], 0x66
    unsigned short int RX_TIMER_T2_M;				//CTL_REG_104[7:5]|CLT_REG_103, 11bits, 0x68, 0x67
    unsigned char RX_TIMER_T2_R;				//CTL_REG_104[4:0], 0x68
    unsigned short int RX_TIMER_CSMA_M;				//CTL_REG_108[7:5]|CLT_REG_107, 11bits, 0x6C,0x6B
    unsigned char RX_TIMER_CSMA_R;				//CTL_REG_108[4:0], 0x6C
    unsigned char TX_DC_TIMES;					//CTL_REG_110, 0x6E, tx duty cycle, set maximum times
    unsigned char TX_DC_DONE_TIMES;			//CTL_REG_112, 0x70, tx duty cycle, times have done
    unsigned char TX_RS_TIMES;					//CTL_REG_113, 0x71, tx ack mode, set maximum times
    unsigned char TX_RS_DONE_TIMES;			//CLT_REG_114, 0x72, tx ack mode, times have done
    unsigned char CSMA_TIMES;					//CTL_REG_115, 0x73, csma, set maximum times
    unsigned char CSMA_DONE_TIMES;				//CTL_REG_116, 0x74, csma, times have done
    unsigned short int SLEEP_TIMER_CSMA_M;			//CTL_REG_119[7:5]|CTL_REG_118, 11bits
    unsigned char SLEEP_TIMER_CSMA_R;			//CTL_REG_119, 0x77[4:0]
} WORK_MODE_CFG;

//## RSSI config (in page1)
typedef struct
{
    union
    {
        struct
        {
            unsigned char COLL_DET_EN :1;	//0: disable, 		 1: enable collision detect;
            unsigned char RESV_1 :1;
            unsigned char RSSI_UPDATE_SEL :2;//0:always update,   1:when PREAM_OK,   2:when SYNC_OK,     3:when PKT_DONE
            unsigned char COLL_STEP_SEL :2;	//0:6dB, 			 1:10dB,  			2:16dB			 	3:20dB
            unsigned char RESV_2 :2;
        } _BITS;
        unsigned char RSSI_CFG_REG;					//RX_RSSI_REG_00, 0x62, page1
    } FRAME_CFG1_u;
    unsigned char RSSI_ABS_TH;					//RX_RSSI_REG_01, 0x63, page1
} RSSI_CFG;

//## Antenna config (in page0 & page1)
typedef struct
{
    union
    {
        struct
        {
            unsigned char ANT_SELECT :1;//0: antenna1, 					1: antenna2, for antenna diversity manual mode
            unsigned char ANT_DIV_MANU :1;	//0: auto antenna diversity, 	1: manual antenna diversity
            unsigned char RESV_6 :6;
        } _BITS;
        unsigned char ANT_CFG1_REG;					// CTL_REG_02, 0x02, page0
    } ANT_CFG1_u;

    union
    {
        struct
        {
            unsigned char ANT_INSTR :1;	//0: antenna1 was used, 		1: antenna2 was used, for indicate which antenna was used in antenna diversity auto mode
            unsigned char RESV_7 :6;
        } _BITS;
        unsigned char ANT_CFG2_REG;					// CTL_REG_14, 0x0E, page0
    } ANT_CFG2_u;

    union
    {
        struct
        {
            unsigned char ANT_DIV_EN :1;	//0: disable, 				1: enable antenna diversity
            unsigned char ANT_SW_DIS :1;	//0: enable,				1: disable antenna diversity switch
            unsigned char ANT_WAIT_PMB :2;//0: RX_PREAM_SIZE x1.5,    1: RX_PREAM_SIZE x2,    2: RX_PREAM_SIZE x2.5,     3: RX_PREAM_SIZE x3
            unsigned char RESV_4 :4;
        } _BITS;
        unsigned char RX_ANTD_REG;					//RX_ANTD_REG_00, 0x67, page1
    } ANT_CFG3_u;
} ANT_DIV_CFG;

//## CDR Tracing (in page1)
typedef struct
{
    union
    {
        struct
        {
            unsigned char CDR_MODE :2;	//0:tracing,  1:counting,  2:manchester,  3:no_cdr
            unsigned char CDR_RANGE_SEL :2;	//0:+/-6.3%,  1:+/-9.4%,   2:+/-12.5%,    3:+/-15.6%
            unsigned char CDR_AVG_SEL :3;//0:48/64,    1:32/64      2:24/64        3:16/64,    4:11/64     5:8/64     6:6/64     7:4/64
            unsigned char CDR_DET_SEL :1;	//0:mode0,    1:mode1(recommand)
        } _BITS;
        unsigned char CDR_CFG0_REG;					//RX_CDR_REG_00, 0x44, page1
    } CDR_CFG0_u;
    union
    {
        struct
        {
            unsigned char RESV_3 :3;	//CDR_BR_TH<18:16>
            unsigned char CDR_3RD_EN :1;	//0:disable,   1:enable
            unsigned char CDR_4TH_EN :1;	//0:disable,   1:enable
            unsigned char RESV_3_2 :3;	//0:mode0,     1:mode1(recommend)
        } _BITS;
        unsigned char CDR_CFG1_REG;					//RX_CDR_REG_00, 0x44, page1
    } CDR_CFG1_u;
    unsigned int CDR_BR_TH;		//RX_CDR_REG_03<18:16> & RX_CDR_REG_02<15:8> & RX_CDR_REG_01<7:0>, page1(0x47,0x45,0x46)
} CDR_TRACING_CFG;

enum CDR_MODE
{
    CDR_SEL_TRACING = 0, CDR_SEL_COUNTING = 1, CDR_SEL_MANCHESTER = 2, CDR_SEL_RAW = 3,
};

typedef struct
{
    INT_SRC_CFG int_src_en;
    INT_SRC_FLG int_src_flag;
    INT_SRC_CLR int_src_clear;
    FIFO_STATUS_FLG fifo_status_flag;
    PREAMBLE_CFG preamble_cfg;
    SYNC_CFG sync_cfg;
    ADDR_CFG addr_cfg;
    CRC_CFG crc_cfg;
    CODING_FORMAT_CFG coding_format_cfg;
    FRAME_CFG frame_cfg;
    WI_SUN_CFG wi_sun_cfg;
    WORK_MODE_CFG word_mode_cfg;
    RSSI_CFG rssi_cfg;
    ANT_DIV_CFG antenna_cfg;
    CDR_TRACING_CFG cdr_tracing_cfg;
} CMT2310A_CFG;

#endif

