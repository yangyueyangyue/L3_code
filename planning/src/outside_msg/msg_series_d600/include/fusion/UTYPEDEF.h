/**********************************************************************/
/* Copyright (C) 2014 by Hitachi Automotive Systems, Ltd. Japan       */
/* All rights reserved.                                               */
/*! @addtogroup R-Car */
/* @{ */

/*********************************************************************/
/*! @file       UTYPEDEF.h
 *  @remarks
 *
 *  @version    ID -- DATE -------- NOTE --------------------------------------------
 *  @version    [00]  14.09.02   T.Takami  弶婜嶌惉
 *  @version    [01]  15.03.30   T.Takami  0,1抣偵(boolean)傪晅壛(QAC懳嶔)
 *  @version    [02]  15.11.28   M.Murakawa   Doxygen僐儊儞僩
 *  @version    [03]  18.05.04   HIENG   L53H_RCar_僨乕僞峔憿巇條.xlsx偺17,18,19斉曄峏懳墳
**********************************************************************/
#ifndef UTYPEDEF_H
#define	UTYPEDEF_H

/**********************************************************************/
/*	Type Definition													  */
/**********************************************************************/
typedef unsigned char	uchar8;
	#define UCHAR8MIN	(0)
	#define UCHAR8MAX	(255)
typedef signed char		schar8;
	#define SCHAR8MIN	(-128)
	#define SCHAR8MAX	(127)
typedef unsigned char	uint8;
	#define UINT8MIN	(0)
	#define UINT8MAX	(255)
typedef signed char		sint8;
	#define SINT8MIN	(-128)
	#define SINT8MAX	(127)
typedef unsigned short	uint16;
	#define UINT16MIN	(0)
	#define UINT16MAX	(65535)
typedef signed short	sint16;
	#define SINT16MIN	(-32768)
	#define SINT16MAX	(32767)
#ifdef SW_64OS
typedef unsigned int	uint32;
#else /* SW_64OS */
typedef unsigned long	uint32;
#endif /* SW_64OS */
	#define UINT32MIN	(0)
	#define UINT32MAX	(4294967295u)
#ifdef SW_64OS
typedef signed int		sint32;
#else /* SW_64OS */
typedef signed long		sint32;
#endif /* SW_64OS */
	#define SINT32MIN	(-2147483648)
	#define SINT32MAX	(2147483647)
typedef unsigned long long	uint64;
typedef signed long long	sint64;			/*[03]*/
#define UINT64MIN (0)
#define UINT64MAX (18446744073709551615ULL)
typedef float               float32;
typedef double              float64;
typedef unsigned char       boolean; 

typedef void       (* FUNC_VOID)(void);

/**********************************************************************/
/*	Macro Definition												  */
/**********************************************************************/
#define TRUE    (boolean)1
#define FALSE   (boolean)0

#define	YES		(boolean)1
#define	NO		(boolean)0

#define OK		(boolean)0
#define NG		(boolean)1
#define OK_OR_NG 2

#define	HI		(boolean)1
#define	LO		(boolean)0

#define	ON		(boolean)1
#define	OFF		(boolean)0

#define UNUSED(variable) (void) variable

#define	BIT_00	(   0x00000001uL)
#define	BIT_01	(   0x00000002uL)
#define	BIT_02	(   0x00000004uL)
#define	BIT_03	(   0x00000008uL)
#define	BIT_04	(   0x00000010uL)
#define	BIT_05	(   0x00000020uL)
#define	BIT_06	(   0x00000040uL)
#define	BIT_07	(   0x00000080uL)
#define	BIT_08	(   0x00000100uL)
#define	BIT_09	(   0x00000200uL)
#define	BIT_10	(   0x00000400uL)
#define	BIT_11	(   0x00000800uL)
#define	BIT_12	(   0x00001000uL)
#define BIT_13	(   0x00002000uL)
#define	BIT_14	(   0x00004000uL)
#define	BIT_15	(   0x00008000uL)
#define	BIT_16	(   0x00010000uL)
#define	BIT_17	(   0x00020000uL)
#define	BIT_18	(   0x00040000uL)
#define	BIT_19	(   0x00080000uL)
#define	BIT_20	(   0x00100000uL)
#define	BIT_21	(   0x00200000uL)
#define	BIT_22	(   0x00400000uL)
#define	BIT_23	(   0x00800000uL)
#define	BIT_24	(   0x01000000uL)
#define	BIT_25	(   0x02000000uL)
#define	BIT_26	(   0x04000000uL)
#define	BIT_27	(   0x08000000uL)
#define	BIT_28	(   0x10000000uL)
#define	BIT_29	(   0x20000000uL)
#define	BIT_30	(   0x40000000uL)
#define	BIT_31	(   0x80000000uL)

#define	BYTE0		 0u		/* 0兽材栚  */
#define	BYTE1		 1u		/* 1兽材栚  */
#define	BYTE2		 2u		/* 2兽材栚  */
#define	BYTE3		 3u		/* 3兽材栚  */
#define	BYTE4		 4u		/* 4兽材栚  */
#define	BYTE5		 5u		/* 5兽材栚  */
#define	BYTE6		 6u		/* 6兽材栚  */
#define	BYTE7		 7u		/* 7兽材栚  */
#define	BYTE8		 8u		/* 8兽材栚  */
#define	BYTE9		 9u		/* 9兽材栚  */
#define	BYTE10		10u		/* 10兽材栚  */
#define	BYTE11		11u		/* 11兽材栚  */
#define	BYTE12		12u		/* 12兽材栚  */

/*泡兽白娃?/
#define ZERO		0u
#define ONE			1u
#define TWO			2u
#define THREE		3u
#define FOUR		4u
#define FIVE		5u
#define SIX			6u
#define SEVEN		7u
#define EIGHT		8u
#define NINE		9u
#define TEN			10u
#define ELEVEN		11u
#define TWELVE		12u
#define THIRTEEN	13u
#define FOURTEEN	14u

/*bit扨埵嵟戝抣*/
#define BIT1MAX  (0x01u)
#define BIT2MAX  (0x03u)
#define BIT3MAX  (0x07u)
#define BIT4MAX  (0x0Fu)
#define BIT5MAX  (0x1Fu)
#define BIT6MAX  (0x3Fu)
#define BIT7MAX  (0x7Fu)

#define	MSK1BIT		0x01u
#define	MSK2BIT		0x03u
#define	MSK3BIT		0x07u
#define	MSK4BIT		0x0Fu
#define	MSK5BIT		0x1Fu
#define	MSK6BIT		0x3Fu
#define	MSK7BIT		0x7Fu
#define	MSK8BIT		0xFFu
#define	MSK9BIT		0x1FFu
#define	MSK10BIT	0x3FFu
#define	MSK11BIT	0x7FFu

#define	MSK32BIT	0xFFFFFFFFul

#define Z0       0                              /*!< 崱夞抣*/
#define Z1       1                              /*!< 慜夞抣*/

/**********************************************************************/
/*	Enum Definition	   												  */
/**********************************************************************/
typedef enum
{
	TYPE_BYTE,	/* Byte(1byte)宆      */
	TYPE_HWORD,	/* Half Word(2byte)宆 */
	TYPE_WORD	/* Word(4byte)宆      */
}EN_SIZE_TYPE;

/**********************************************************************/
/*	Macro Function Definition										  */
/***********************************************************************/
#define BIT(n)			(1u << (n))
#define BITS(e,s)		(((BIT((e)-(s)) << 1) - 1) << (s))
#define SIZE(A) 		((sizeof A)/(sizeof A[0U]))
#define SETB(DATA,BIT)	((DATA)|=(BIT))
#define CLRB(DATA,BIT)	((DATA)&=(~(BIT)))
#define TSTB(DATA,BIT)	(((DATA)&(BIT))!=0U)
#define CHGB(DATA,BIT)  ((DATA)^=(BIT))
#define CHKB(DATA,BIT)  (((DATA)&(BIT))==BIT)

#define INC(VAL,SIZE)  if(VAL<SIZE)\
                       {\
                           VAL++;\
                       }
#define DEC(VAL,SIZE)  if(VAL>SIZE)\
                       {\
                           VAL--;\
                       }

#define ABS(a)		   (((a) >= 0L) ? (a) : -(a))

#define IS_4BYTE_ALIGN(X)	(((X) & 0x03u)==0u)	/* 傾僪儗僗X偑4byte嫬奅偐斲偐 */
#define IS_2BYTE_ALIGN(X)	(((X) & 0x01u)==0u)	/* 傾僪儗僗X偑2byte嫬奅偐斲偐 */

#define WORD_HI(X) 	   ((uint8)(((X) & 0xFF000000u )>>24))
#define WORD_MID_HI(X) ((uint8)(((X) & 0x00FF0000u )>>16))
#define WORD_MID_LO(X) ((uint8)(((X) & 0x0000FF00u )>>8 ))
#define WORD_LO(X) 	   ((uint8)(((X) & 0x000000FFu )>>0 ))

#define HWORD_HI(X) ((uint8)(((X) & 0xFF00u )>>8 ))
#define HWORD_LO(X) ((uint8)(((X) & 0x00FFu )>>0 ))

#define NIBBLE_LO(X)  ( (X)      & 0x0Fu)
#define NIBBLE_HI(X)  (((X)>>4U) & 0x0Fu)

/**********************************************************************/
/*	Struct Definition												  */
/**********************************************************************/
typedef union 
{
	struct
	{
		uint8 b7:1;
		uint8 b6:1;
		uint8 b5:1;
		uint8 b4:1;
		uint8 b3:1;
		uint8 b2:1;
		uint8 b1:1;
		uint8 b0:1;
	}Bit;
	uint8 Byte;
}ByteData;

typedef union 
{
	uint8  Byte[8];
	uint16 Word[4];
	uint32 DWord[2];
}BlockData;

#endif	/*UTYPEDEF_H*/
