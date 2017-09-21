/**	----------------------------------------------------------------------
**
**	@file			accel_mag.h
**
**  @defgroup 		accelerometer
**  @{
**  @ingroup 		accelerometer
**  @brief accelerometer driver module.
**  
**  @details This module implements the PAVLOK accelerometer driver code
**  
**  @note
**  
**  @note
**   
**
**	----------------------------------------------------------------------
*/
#ifndef _ACCELEROMETER_H_
#define _ACCELEROMETER_H_

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdint.h>

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
uint8_t accelerometer_whoami(void);
extern volatile uint8_t demoSemaphore1;
extern volatile uint8_t demoSemaphore2;

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ Definitions
**	----------------------------------------------------------------------
*/
// Debug enable/disable
// This is used to toggle between using the DK board or the target hardware
#define ACCEL_MAG_DEBUG (0)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ Definitions
**	----------------------------------------------------------------------
*/
#define FXOS8700CQ_REG_RANGE_LOW							(0x00)
#define FXOS8700CQ_REG_RANGE_HIGH							(0x78)
#define FXOS8700CQ_REG_SIZE										(2)
#define I2C_ACC_MAG_ADDRESS										(0x1E)
#define I2C_ACC_MAG_MAX_TRANSFER							(16)
#define ACC_MAG_FXOS8700CQ_ADDRESS_SIZE				(1)
#define ACC_MAG_FXOS8700CQ_REGISTER_SIZE			(1)
#define ACC_MAG_2G_PER_LSB										(0.244)
#define ACC_MAG_4G_PER_LSB										(0.488)
#define ACC_MAG_8G_PER_LSB										(0.976)
#define ACC_MAG_SCALING_PER_LSB								ACC_MAG_4G_PER_LSB
#if ACCEL_MAG_DEBUG
#define ACC_MAG_INT1_PIN 											(13)
#define ACC_MAG_INT2_PIN											(15)
#else
#define ACC_MAG_INT1_PIN 											(28)
#define	ACC_MAG_INT2_PIN											(27)
#endif
#define DK_CONT_READ_DEMO											(1)
#define DK_TRANSIENT_DEMO											(0)
#define DK_PULSE_DEMO													(0)
#if (DK_CONT_READ_DEMO && (DK_CONT_TRANSIENT_DEMO || DK_PULSE_DEMO))
#error "Cannot enable both demos!
#endif

// Objects
typedef struct 
{
	uint8_t address;
	uint8_t data[I2C_ACC_MAG_MAX_TRANSFER];
} accmagTXRXPacket_T;

typedef struct 
{
	int16_t x;
	int16_t y;
	int16_t z;
} ACCEL_DATA_T;

// Enumerations

typedef enum 
{
	ACC_MAG_INIT_SUCCESS,
	ACC_MAG_INIT_FAIL
} ACC_MAG_INIT_RET_T;

typedef enum 
{
	ACC_MAG_SUCCESS,
	ACC_MAG_NULL_DATA,
	ACC_MAG_INVALID_ADDRESS,
	ACC_MAG_INVALID_SIZE,
	ACC_MAG_TX_ERROR		
} ACC_MAG_TXRX_RET_T;

// Prototypes	
ACC_MAG_TXRX_RET_T accel_magnet_write(uint8_t, uint8_t *, uint8_t);
ACC_MAG_TXRX_RET_T accel_magnet_read(uint8_t, uint8_t *, uint8_t);
ACC_MAG_INIT_RET_T accelerometer_init(void);
ACC_MAG_TXRX_RET_T get_acceleration(ACCEL_DATA_T *accelData);
ACC_MAG_TXRX_RET_T accel_mag_unlatch_int1(void);
ACC_MAG_TXRX_RET_T accel_mag_unlatch_int2(void);
void poll_acceleration_data(void);
void verify_acc_mag_loop(void);
void read_acceleration_after_threshold(ACCEL_DATA_T * data);
uint32_t read_stream_data(uint8_t * p_data);
ACC_MAG_INIT_RET_T accel_main_init(void);
void accelerometer_init_continuouse_read(void);
void accelerometer_init_pulse(void);
void accelerometer_init_transient(void);
uint8_t accelerometer_whoami(void);

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ internal register addresses
**	----------------------------------------------------------------------
*/
#define FXOS8700CQ_STATUS 										0x00
#define FXOS8700CQ_WHOAMI 										0x0D
#define FXOS8700CQ_XYZ_DATA_CFG  							0x0E
#define FXOS8700CQ_CTRL_REG1 									0x2A
#define FXOS8700CQ_M_CTRL_REG1 								0x5B
#define FXOS8700CQ_M_CTRL_REG2 								0x5C
#define FXOS8700CQ_WHOAMI_VAL 								0xC7
#define FXOS8700CQ_TRANSIENT_CFG							0x1D
#define FXOS8700CQ_TRANSIENT_SRC							0x1E
#define FXOS8700CQ_TRANSIENT_THS							0x1F
#define FXOS8700CQ_TRANSIENT_COUNT						0x20
#define FXOS8700CQ_CTRL_REG4									0x2D
#define FXOS8700CQ_CTRL_REG5									0x2E
#define FXOS8700CQ_PULSE_CFG									0x21
#define FXOS8700CQ_PULSE_SRC									0x22
#define FXOS8700CQ_PULSE_THSX									0x23
#define FXOS8700CQ_PULSE_THSY									0x24
#define FXOS8700CQ_PULSE_THSZ									0x25
#define FXOS8700CQ_PULSE_TMLT									0x26
#define FXOS8700CQ_PULSE_LTCY									0x27
#define FXOS8700CQ_PULSE_WIND									0x28

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ internal register manipulation macros
**	----------------------------------------------------------------------
*/
/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ CTRL_REG1 macros
**	----------------------------------------------------------------------
*/
#define CTRL_REG1_ACTIVE_SHIFT								0
#define CTRL_REG1_ACTIVE_MASK									0x01
#define CTRL_REG1_ACTIVE(x)										(((uint8_t)(((uint8_t)(x))<<CTRL_REG1_ACTIVE_SHIFT))&CTRL_REG1_ACTIVE_MASK)

#define CTRL_REG1_F_READ_SHIFT								1
#define CTRL_REG1_F_READ_MASK									0x02
#define CTRL_REG1_F_READ(x)										(((uint8_t)(((uint8_t)(x))<<CTRL_REG1_F_READ_SHIFT))&CTRL_REG1_F_READ_MASK)

#define CTRL_REG1_INOISE_SHIFT								2
#define CTRL_REG1_INOISE_MASK									0x04
#define CTRL_REG1_INOISE(x)										(((uint8_t)(((uint8_t)(x))<<CTRL_REG1_INOISE_SHIFT))&CTRL_REG1_INOISE_MASK)				

#define CTRL_REG1_DR_SHIFT										3
#define CTRL_REG1_DR_MASK											0x38
#define CTRL_REG1_DR(x)												(((uint8_t)(((uint8_t)(x))<<CTRL_REG1_DR_SHIFT))&CTRL_REG1_DR_MASK)

#define CTRL_REG1_ASLP_RATE_SHIFT							6
#define CTRL_REG1_ASLP_RATE_MASK							0xC0
#define CTRL_REG1_ASLP_RATE(x)								(((uint8_t)(((uint8_t)(x))<<CTRL_REG1_ASLP_RATE_SHIFT))&CTRL_REG1_ASLP_RATE_MASK)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ XZY_DATA_CFG macros
**	----------------------------------------------------------------------
*/
#define XZY_DATA_CFG_FS_SHIFT									0
#define XZY_DATA_CFG_FS_MASK									0x03
#define XZY_DATA_CFG_FS(x)										(((uint8_t)(((uint8_t)(x))<<XZY_DATA_CFG_FS_SHIFT))&XZY_DATA_CFG_FS_MASK)

#define XZY_DATA_CFG_HPF_OUT_SHIFT						4
#define XZY_DATA_CFG_HPF_OUT_MASK							0x10
#define XZY_DATA_CFG_HPF_OUT(x)								(((uint8_t)(((uint8_t)(x))<<XZY_DATA_CFG_HPF_OUT_SHIFT))&XZY_DATA_CFG_HPF_OUT_MASK)	

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ M_CTRL_REG1 macros
**	----------------------------------------------------------------------
*/
#define M_CTRL_REG1_M_HMS_SHIFT								0
#define M_CTRL_REG1_M_HMS_MASK								0x03
#define M_CTRL_REG1_M_HMS(x)									(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG1_M_HMS_SHIFT))&M_CTRL_REG1_M_HMS_MASK)

#define M_CTRL_REG1_M_OS_SHIFT								2
#define M_CTRL_REG1_M_OS_MASK									0x1C
#define M_CTRL_REG1_M_OS(x)										(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG1_M_OS_SHIFT))&M_CTRL_REG1_M_OS_MASK)

#define M_CTRL_REG1_M_OST_SHIFT								5
#define M_CTRL_REG1_M_OST_MASK								0x20
#define M_CTRL_REG1_M_OST(x)									(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG1_M_OST_SHIFT))&M_CTRL_REG1_M_OST_MASK)

#define M_CTRL_REG1_M_RST_SHIFT								6
#define M_CTRL_REG1_M_RST_MASK								0x40
#define M_CTRL_REG1_M_RST(x)									(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG1_M_RST_SHIFT))&M_CTRL_REG1_M_RST_MASK)

#define M_CTRL_REG1_M_ACAL_SHIFT							7
#define M_CTRL_REG1_M_ACAL_MASK								0x80
#define M_CTRL_REG1_M_ACAL(x)									(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG1_M_ACAL_SHIFT))&M_CTRL_REG1_M_ACAL_MASK)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ M_CTRL_REG2 macros
**	----------------------------------------------------------------------
*/
#define M_CTRL_REG2_M_RST_CNT_SHIFT						0
#define M_CTRL_REG2_M_RST_CNT_MASK						0x03
#define M_CTRL_REG2_M_RST_CNT(x)							(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG2_M_RST_CNT_SHIFT))&M_CTRL_REG2_M_RST_CNT_MASK)

#define M_CTRL_REG2_M_MAXMIN_RST_SHIFT				2
#define M_CTRL_REG2_M_MAXMIN_RST_MASK					0x04
#define M_CTRL_REG2_M_MAXMIN_RST(x)						(((uint8_t)(((uint8_t (x))<<M_CTRL_REG2_M_MAXMIN_RST_SHIFT))&M_CTRL_REG2_M_MAXMIN_RST_MASK)

#define M_CTRL_REG2_M_MAXMIN_DIS_THS_SHIFT		3
#define M_CTRL_REG2_M_MAXMIN_DIS_THS_MASK			0x08
#define M_CTRL_REG2_M_MAXMIN_DIS_THS(x)				(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG2_M_MAXMIN_DIS_THS_SHIFT))&M_CTRL_REG2_M_MAXMIN_DIS_THS_MASK)

#define M_CTRL_REG2_M_MAXMIN_DIS_SHIFT				4
#define M_CTRL_REG2_M_MAXMIN_DIS_MASK					0x10
#define M_CTRL_REG2_M_MAXMIN_DIS(x)						(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG2_M_MAXMIN_DIS_SHIFT))&M_CTRL_REG2_M_MAXMIN_DIS_MASK)

#define M_CTRL_REG2_HYB_AUTOINC_MODE_SHIFT		5
#define M_CTRL_REG2_HYB_AUTOINC_MODE_MASK			0x20
#define M_CTRL_REG2_HYB_AUTOINC_MODE(x)				(((uint8_t)(((uint8_t)(x))<<M_CTRL_REG2_HYB_AUTOINC_MODE_SHIFT))&M_CTRL_REG2_HYB_AUTOINC_MODE_MASK)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ A_VECM_CFG_VECM macros
**	----------------------------------------------------------------------
*/
#define A_VECM_CFG_VECM_EN_SHIFT							3
#define A_VECM_CFG_VECM_EN_MASK								0x08
#define A_VECM_CFG_VECM_EN(x)									(((uint8_t)(((uint8_t)(x))<<A_VECM_CFG_VECM_EN_SHIFT))&A_VECM_CFG_VECM_EN_MASK)

#define A_VECM_CFG_VECM_UPDM_SHIFT						4
#define A_VECM_CFG_VECM_UPDM_MASK							0x10
#define A_VECM_CFG_VECM_UPDM(x)								(((uint8_t)(((uint8_t)(x))<<A_VECM_CFG_VECM_UPDM_SHIFT))&A_VECM_CFG_VECM_UPDM_MASK)

#define A_VECM_CFG_VECM_INITM_SHIFT						5
#define A_VECM_CFG_VECM_INITM_MASK						0x20
#define A_VECM_CFG_VECM_INITM(x)							(((uint8_t)(((uint8_t)(x))<<A_VECM_CFG_VECM_INITM_SHIFT))&A_VECM_CFG_VECM_INITM_MASK)

#define A_VECM_CFG_VECM_ELE_SHIFT							6
#define A_VECM_CFG_VECM_ELE_MASK							0x40
#define A_VECM_CFG_VECM_ELE(x)								(((uint8_t)(((uint8_t)(x))<<A_VECM_CFG_VECM_ELE_SHIFT))&A_VECM_CFG_VECM_ELE_MASK)

#define A_VECM_THS_MSB_THS_SHIFT							0
#define A_VECM_THS_MSB_THS_MASK								0x0F
#define A_VECM_THS_MSB(x)											(((uint8_t)(((uint8_t)(x))<<A_VECM_THS_MSB_THS_SHIFT))&A_VECM_THS_MSB_THS_MASK)

#define A_VECM_THS_MSB_DBCNTM_SHIFT						7
#define A_VECM_THS_MSB_DBCNTM_MASK						0x80
#define A_VECM_THS_MSB_DBCNTM(x)							(((uint8_t)(((uint8_t)(x))<<A_VECM_THS_MSB_DBCNTM_SHIFT))&A_VECM_THS_MSB_DBCNTM_MASK)

#define A_VECM_THS_LSB_SHIFT									0
#define A_VECM_THS_LSB_MASK										0xFF
#define A_VECM_THS_LSB(x)											(((uint8_t)(((uint8_t)(x))<<A_VECM_THS_LSB_SHIFT))&A_VECM_THS_LSB_MASK)

#define A_VECM_CNT_SHIFT											0
#define A_VECM_CNT_MASK												0xFF
#define A_VECM_CNT(x)													(((uint8_t)(((uint8_t)(x))<<A_VECM_CNT_SHIFT))&A_VECM_CNT_MASK)

#define A_VECM_INITX_MSB_SHIFT								0
#define A_VECM_INITX_MSB_MASK									0x3F
#define A_VECM_INITX_MSB(x)										(((uint8_t)(((uint8_t)(x))<<A_VECM_INITX_MSB_SHIFT))&A_VECM_INITX_MSB_MASK)

#define A_VECM_INITX_LSB_SHIFT								0
#define A_VECM_INITX_LSB_MASK									0xFF
#define A_VECM_INITX_LSB(x)										(((uint8_t)(((uint8_t)(x))<<A_VECM_INITX_LSB_SHIFT))&A_VECM_INITX_LSB_MASK)

#define A_VECM_INITY_MSB_SHIFT								0
#define A_VECM_INITY_MSB_MASK									0x3F
#define A_VECM_INITY_MSB(x)										(((uint8_t)(((uint8_t)(x))<<A_VECM_INITY_MSB_SHIFT))&A_VECM_INITY_MSB_MASK)

#define A_VECM_INITY_LSB_SHIFT								0
#define A_VECM_INITY_LSB_MASK									0xFF
#define A_VECM_INITY_LSB(x)										(((uint8_t)(((uint8_t)(x))<<A_VECM_INITY_LSB_SHIFT))&A_VECM_INITY_LSB_MASK)

#define A_VECM_INITZ_MSB_SHIFT								0
#define A_VECM_INITZ_MSB_MASK									0x3F
#define A_VECM_INITZ_MSB(x)										(((uint8_t)(((uint8_t)(x))<<A_VECM_INITZ_MSB_SHIFT))&A_VECM_INITZ_MSB_MASK)

#define A_VECM_INITZ_LSB_SHIFT								0
#define A_VECM_INITZ_LSB_MASK									0xFF
#define A_VECM_INITZ_LSB(x)										(((uint8_t)(((uint8_t)(x))<<A_VECM_INITZ_LSB_SHIFT))&A_VECM_INITZ_LSB_MASK)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ A_FTMT_CFG macros
**	----------------------------------------------------------------------
*/
#define A_FFMT_CFG_XEFE_SHIFT									3
#define A_FFMT_CFG_XEFE_MASK									0x08
#define A_FFMT_CFG_XEFE(x)										(((uint8_t)(((uint8_t)(x))<<A_FFMT_CFG_XEFE_SHIFT))&A_FFMT_CFG_XEFE_MASK)

#define A_FFMT_CFG_YEFE_SHIFT									4
#define A_FFMT_CFG_YEFE_MASK									0x10
#define A_FFMT_CFG_YEFE(x)										(((uint8_t)(((uint8_t)(x))<<A_FFMT_CFG_YEFE_SHIFT))&A_FFMT_CFG_XEFE_MASK)

#define A_FFMT_CFG_ZEFE_SHIFT									5
#define A_FFMT_CFG_ZEFE_MASK									0x20
#define A_FFMT_CFG_ZEFE(x)										(((uint8_t)(((uint8_t)(x))<<A_FFMT_CFG_ZEFE_SHIFT))&A_FFMT_CFG_ZEFE_MASK)

#define A_FFMT_CFG_OAE_SHIFT									6
#define A_FFMT_CFG_OAE_MASK										0x40
#define A_FFMT_CFG_OAE(x)											(((uint8_t)(((uint8_t)(x))<<A_FFMT_CFG_OAE_SHIFT))&A_FFMT_CFG_OAE_MASK)

#define A_FFMT_CFG_ELE_SHIFT									7
#define A_FFMT_CFG_ELE_MASK										0x80
#define A_FFMT_CFG_ELE(x)											(((uint8_t)(((uint8_t)(x))<<A_FFMT_CFG_ELE_SHIFT))&A_FFMT_CFG_ELE_MASK)

#define A_FFMT_THS_SHIFT											0
#define A_FFMT_THS_MASK												0x7F
#define A_FFMT_THS(x)													(((uint8_t)(((uint8_t)(x))<<A_FFMT_THS_SHIFT))&A_FFMT_THS_MASK)

#define A_FFMT_DBCNTM_SHIFT										7
#define A_FFMT_DBCNTM_MASK										0x80
#define A_FFTM_DBCNTM(x)											(((uint8_t)(((uint8_t)(x))<<A_FFMT_DBCNTM_SHIFT))&A_FFMT_DBCNTM_MASK)

#define A_FFMT_THS_X_MSB_SHIFT								0
#define A_FFMT_THS_X_MSB_MASK									0x7F
#define A_FFMT_THS_X_MSB(x)										(((uint8_t)(((uint8_t)(x))<<A_FFMT_THS_X_MSB_SHIFT))&A_FFMT_THS_X_MSB_MASK)

#define A_FFMT_THS_XYZ_EN_SHIFT								7
#define A_FFMT_THS_XYZ_EN_MASK								0x80
#define A_FFMT_THS_XYZ_EN(x)									(((uint8_t)(((uint8_t)(x))<<A_FFMT_THS_XYZ_EN_SHIFT))&A_FFMT_THS_XYZ_EN_MASK)

#define A_FFMT_THS_Y_MSB_SHIFT								0
#define A_FFMT_THS_Y_MSB_MASK									0x7F
#define A_FFMT_THS_Y_MSB(x)										(((uint8_t)(((uint8_t)(x))<<A_FFMT_THS_Y_MSB_SHIFT))&A_FFMT_THS_Y_MSB_MASK)

#define A_FFMT_THS_TRANS_THS_EN_SHIFT					7
#define A_FFMT_THS_TRANS_THS_EN_MASK					0x08
#define A_FFMT_THS_TRANS_THS_EN(x)						(((uint8_t)(((uint8_t)(x))<<A_FFMT_THS_TRANS_THS_EN_SHIFT))&A_FFMT_THS_TRANS_THS_EN_MASK)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ CTRL_REG4_INT macros
**	----------------------------------------------------------------------
*/
#define CTRL_REG4_INT_EN_DRDY_SHIFT						0
#define CTRL_REG4_INT_EN_DRDY_MASK						0x01
#define CTRL_REG4_INT_EN_DRDY(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG4_INT_EN_DRDY_SHIFT))&CTRL_REG4_INT_EN_DRDY_MASK)

#define CTRL_REG4_INT_EN_A_VECM_SHIFT					1
#define CTRL_REG4_INT_EN_A_VECM_MASK					0x02
#define CTRL_REG4_INT_EN_A_VECM(x)						(((uint8_t)(((uint8_t)(x))<<CTRL_REG4_INT_EN_A_VECM_SHIFT))&CTRL_REG4_INT_EN_A_VECM_MASK)

#define CTRL_REG4_INT_EN_FFMT_SHIFT						2
#define CTRL_REG4_INT_EN_FFMT_MASK						0x04
#define CTRL_REG4_INT_EN_FFMT(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG4_INT_EN_FFMT_SHIFT))&CTRL_REG4_INT_EN_FFMT_MASK)

#define CTRL_REG4_INT_EN_PULSE_SHIFT					3
#define CTRL_REG4_INT_EN_PULSE_MASK						0x08
#define CTRL_REG4_INT_EN_PULSE(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG4_INT_EN_PULSE_SHIFT))&CTRL_REG4_INT_EN_PULSE_MASK)

#define CTRL_REG4_INT_EN_LNDPRT_SHIFT					4
#define CTRL_REG4_INT_EN_LNDPRT_MASK					0x10
#define CTRL_REG4_INT_EN_LNDPRT(x)						(((uint8_t)(((uint8_t)(x))<<CTRL_REG4_INT_EN_LNDPRT_SHIFT))&CTRL_REG4_INT_EN_LNDPRT_MASK)

#define CTRL_REG4_INT_EN_TRANS_SHIFT					5
#define CTRL_REG4_INT_EN_TRANS_MASK						0x20
#define CTRL_REG4_INT_EN_TRANS(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG4_INT_EN_TRANS_SHIFT))&CTRL_REG4_INT_EN_TRANS_MASK)

#define CTRL_REG4_INT_EN_FIFO_SHIFT						6
#define CTRL_REG4_INT_EN_FIFO_MASK						0x40
#define CTRL_REG4_INT_EN_FIFO(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG4_INT_EN_FIFO_SHIFT))&CTRL_REG4_INT_EN_FIFO_MASK)

#define CTRL_REG4_INT_EN_ASLP_SHIFT						7
#define CTRL_REG4_INT_EN_ASLP_MASK						0x80
#define CTRL_REG4_INT_EN_ASLP(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG4_INT_EN_ASLP_SHIFT))&CTRL_REG4_INT_EN_ASLP_MASK)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ CTRL_REG5_INT macros
**	----------------------------------------------------------------------
*/
#define CTRL_REG5_INT_CFG_DRDY_SHIFT					0
#define CTRL_REG5_INT_CFG_DRDY_MASK						0x01
#define CTRL_REG5_INT_CFG_DRDY(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG5_INT_CFG_DRDY_SHIFT))&CTRL_REG5_INT_CFG_DRDY_MASK)

#define CTRL_REG5_INT_CFG_A_VECM_SHIFT				1
#define CTRL_REG5_INT_CFG_A_VECM_MASK					0x02
#define CTRL_REG5_INT_CFG_A_VECM(x)						(((uint8_t)(((uint8_t)(x))<<CTRL_REG5_INT_CFG_A_VECM_SHIFT))&CTRL_REG5_INT_CFG_A_VECM_MASK)

#define CTRL_REG5_INT_CFG_FFMT_SHIFT					2
#define CTRL_REG5_INT_CFG_FFMT_MASK						0x04
#define CTRL_REG5_INT_CFG_FFMT(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG5_INT_CFG_FFMT_SHIFT))&CTRL_REG5_INT_CFG_FFMT_MASK)

#define CTRL_REG5_INT_CFG_PULSE_SHIFT					3
#define CTRL_REG5_INT_CFG_PULSE_MASK					0x08
#define CTRL_REG5_INT_CFG_PULSE(x)						(((uint8_t)(((uint8_t)(x))<<CTRL_REG5_INT_CFG_PULSE_SHIFT))&CTRL_REG5_INT_CFG_PULSE_MASK)

#define CTRL_REG5_INT_CFG_LNDPRT_SHIFT				4
#define CTRL_REG5_INT_CFG_LNDPRT_MASK					0x10
#define CTRL_REG5_INT_CFG_LNDPRT(x)						(((uint8_t)(((uint8_t)(x))<<CTRL_REG5_INT_CFG_LNDPRT_SHIFT))&CTRL_REG5_INT_CFG_LNDPRT_MASK)

#define CTRL_REG5_INT_CFG_TRANS_SHIFT					5
#define CTRL_REG5_INT_CFG_TRANS_MASK					0x20
#define CTRL_REG5_INT_CFG_TRANS(x)						(((uint8_t)(((uint8_t)(x))<<CTRL_REG5_INT_CFG_TRANS_SHIFT))&CTRL_REG5_INT_CFG_TRANS_MASK)

#define CTRL_REG5_INT_CFG_FIFO_SHIFT					6
#define CTRL_REG5_INT_CFG_FIFO_MASK						0x40
#define CTRL_REG5_INT_CFG_FIFO(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG5_INT_CFG_FIFO_SHIFT))&CTRL_REG5_INT_CFG_FIFO_MASK)

#define CTRL_REG5_INT_CFG_ASLP_SHIFT					7
#define CTRL_REG5_INT_CFG_ASLP_MASK						0x80
#define CTRL_REG5_INT_CFG_ASLP(x)							(((uint8_t)(((uint8_t)(x))<<CTRL_REG5_INT_CFG_ASLP_SHIFT))&CTRL_REG5_INT_CFG_ASLP_MASK)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ TRANSIENT macros
**	----------------------------------------------------------------------
*/
#define TRANSIENT_THS_TR_DBCNTM_SHIFT					7
#define TRANSIENT_THS_TR_DBCNTM_MASK					0x80
#define TRANSIENT_THS_TR_DBCNTM(x)						(((uint8_t)(((uint8_t)(x))<<TRANSIENT_THS_TR_DBCNTM_SHIFT))&TRANSIENT_THS_TR_DBCNTM_MASK)

#define TRANSIENT_THS_TR_THS_SHIFT						0
#define TRANSIENT_THS_TR_THS_MASK							0x7F
#define TRANSIENT_THS_TR_THS(x)								(((uint8_t)(((uint8_t)(x))<<TRANSIENT_THS_TR_THS_SHIFT))&TRANSIENT_THS_TR_THS_MASK)

#define TRANSIENT_COUNT_SHIFT									0
#define TRANSIENT_COUNT_MASK									0xFF
#define TRANSIENT_COUNT(x)										(((uint8_t)(((uint8_t)(x))<<TRANSIENT_COUNT_SHIFT))&TRANSIENT_COUNT_MASK)

#define TRANSIENT_CFG_TRAN_HPF_BYP_SHIFT			0
#define TRANSIENT_CFG_TRAN_HPP_BYP_MASK				0x01
#define TRANSIENT_CFG_TRAN_HPF_BYP(x)					(((uint8_t)(((uint8_t)(x))<<TRANSIENT_CFG_TRAN_HPF_BYP_SHIFT))&TRANSIENT_CFG_TRAN_HPP_BYP_MASK)

#define TRANSIENT_CFG_TRAN_XEFE_SHIFT					1
#define TRANSIENT_CFG_TRAN_XEFE_MASK					0x02
#define TRANSIENT_CFG_TRAN_XEFE(x)						(((uint8_t)(((uint8_t)(x))<<TRANSIENT_CFG_TRAN_XEFE_SHIFT))&TRANSIENT_CFG_TRAN_XEFE_MASK)

#define TRANSIENT_CFG_TRAN_YEFE_SHIFT					2
#define TRANSIENT_CFG_TRAN_YEFE_MASK					0x04
#define TRANSIENT_CFG_TRAN_YEFE(x)						(((uint8_t)(((uint8_t)(x))<<TRANSIENT_CFG_TRAN_YEFE_SHIFT))&TRANSIENT_CFG_TRAN_YEFE_MASK)

#define TRANSIENT_CFG_TRAN_ZEFE_SHIFT					3
#define TRANSIENT_CFG_TRAN_ZEFE_MASK					0x08
#define TRANSIENT_CFG_TRAN_ZEFE(x)						(((uint8_t)(((uint8_t)(x))<<TRANSIENT_CFG_TRAN_ZEFE_SHIFT))&TRANSIENT_CFG_TRAN_ZEFE_MASK)

#define TRANSIENT_CFG_TRAN_ELE_SHIFT					4
#define TRANSIENT_CFG_TRAN_ELE_MASK						0x10
#define TRANSIENT_CFG_TRAN_ELE(x)							(((uint8_t)(((uint8_t)(x))<<TRANSIENT_CFG_TRAN_ELE_SHIFT))&TRANSIENT_CFG_TRAN_ELE_MASK)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ PULSE macros
**	----------------------------------------------------------------------
*/
#define PULSE_CFG_PLS_XSPEFE_SHIFT						0
#define PULSE_CFG_PLS_XSPEFE_MASK							0x01
#define PULSE_CFG_PLS_XSPEFE(x)								(((uint8_t)(((uint8_t)(x))<<PULSE_CFG_PLS_XSPEFE_SHIFT))&PULSE_CFG_PLS_XSPEFE_MASK)

#define PULSE_CFG_PLS_XDPEFE_SHIFT						1
#define PULSE_CFG_PLS_XDPEFE_MASK							0x02
#define PULSE_CFG_PLS_XDPEFE(x)								(((uint8_t)(((uint8_t)(x))<<PULSE_CFG_PLS_XDPEFE_SHIFT))&PULSE_CFG_PLS_XDPEFE_MASK)

#define PULSE_CFG_PLS_YSPEFE_SHIFT						2
#define PULSE_CFG_PLS_YSPEFE_MASK							0x04
#define PULSE_CFG_PLS_YSPEFE(x)								(((uint8_t)(((uint8_t)(x))<<PULSE_CFG_PLS_YSPEFE_SHIFT))&PULSE_CFG_PLS_YSPEFE_MASK)

#define PULSE_CFG_PLS_YDPEFE_SHIFT						3
#define PULSE_CFG_PLS_YDPEFE_MASK							0x08
#define PULSE_CFG_PLS_YDPEFE(x)								(((uint8_t)(((uint8_t)(x))<<PULSE_CFG_PLS_YDPEFE_SHIFT))&PULSE_CFG_PLS_YDPEFE_MASK)

#define PULSE_CFG_PLS_ZSPEFE_SHIFT						4
#define PULSE_CFG_PLS_ZSPEFE_MASK							0x10
#define PULSE_CFG_PLS_ZSPEFE(x)								(((uint8_t)(((uint8_t)(x))<<PULSE_CFG_PLS_ZSPEFE_SHIFT))&PULSE_CFG_PLS_ZSPEFE_MASK)

#define PULSE_CFG_PLS_ZDPEFE_SHIFT						5
#define PULSE_CFG_PLS_ZDPEFE_MASK							0x20
#define PULSE_CFG_PLS_ZDPEFE(x)								(((uint8_t)(((uint8_t)(x))<<PULSE_CFG_PLS_ZDPEFE_SHIFT))&PULSE_CFG_PLS_ZDPEFE_MASK)

#define PULSE_CFG_PLS_ELE_SHIFT								6
#define PULSE_CFG_PLS_ELE_MASK								0x40
#define PULSE_CFG_PLS_ELE(x)									(((uint8_t)(((uint8_t)(x))<<PULSE_CFG_PLS_ELE_SHIFT))&PULSE_CFG_PLS_ELE_MASK)

#define PULSE_CFG_PLS_DPA_SHIFT								7
#define PULSE_CFG_PLS_DPA_MASK								0x80
#define PULSE_CFG_PLS_DPA(x)									(((uint8_t)(((uint8_t)(x))<<PULSE_CFG_PLS_DPA_SHIFT))&PULSE_CFG_PLS_DPA_MASK)

#define PULSE_THSX_SHIFT											0
#define PULSE_THSX_MASK												0x7F
#define PULSE_THSX(x)													(((uint8_t)(((uint8_t)(x))<<PULSE_THSX_SHIFT))&PULSE_THSX_MASK)

#define PULSE_THSY_SHIFT											0
#define PULSE_THSY_MASK												0x7F
#define PULSE_THSY(x)													(((uint8_t)(((uint8_t)(x))<<PULSE_THSY_SHIFT))&PULSE_THSY_MASK)

#define PULSE_THSZ_SHIFT											0
#define PULSE_THSZ_MASK												0x7F
#define PULSE_THSZ(x)													(((uint8_t)(((uint8_t)(x))<<PULSE_THSZ_SHIFT))&PULSE_THSZ_MASK)

#define PULSE_TMLT_SHIFT											0
#define PULSE_TMLT_MASK												0xFF
#define PULSE_TMLT(x)													(((uint8_t)(((uint8_t)(x))<<PULSE_TMLT_SHIFT))&PULSE_TMLT_MASK)

#define PULSE_LTCY_SHIFT											0
#define PULSE_LTCY_MASK												0xFF
#define PULSE_LTCY(x)													(((uint8_t)(((uint8_t)(x))<<PULSE_LTCY_SHIFT))&PULSE_LTCY_MASK)

#define PULSE_WIND_SHIFT											0
#define PULSE_WIND_MASK												0xFF
#define PULSE_WIND(x)													(((uint8_t)(((uint8_t)(x))<<PULSE_WIND_SHIFT))&PULSE_WIND_MASK)
 
#define PULSE_SOURCE_SRC_POLX									(uint8_t)(1<<0)
#define PULSE_SOURCE_SRC_POLY									(uint8_t)(1<<1)
#define PULSE_SOURCE_SRC_POLZ									(uint8_t)(1<<2)
#define PULSE_SOURCE_SRC_DPE									(uint8_t)(1<<3)
#define PULSE_SOURCE_SRC_AXX									(uint8_t)(1<<4)
#define PULSE_SOURCE_SRC_AXY									(uint8_t)(1<<5)
#define PULSE_SOURCE_SRC_AXZ									(uint8_t)(1<<6)
#define PULSE_SOURCE_SRC_EA										(uint8_t)(1<<7)	

#define IS_PULSE_SOURCE_TRUE(x, y)						(uint8_t)(x & y)

/** ----------------------------------------------------------------------
**	@def	FXOS8700CQ Interrupt register macros
**	----------------------------------------------------------------------
*/
#define INT_SOURCE_SRC_DRDY										(uint8_t)(1<<0)
#define INT_SOURCE_SRC_A_VECM									(uint8_t)(1<<1)
#define INT_SOURCE_SRC_FFMT										(uint8_t)(1<<2)
#define INT_SOURCE_SRC_PULSE									(uint8_t)(1<<3)
#define INT_SOURCE_SRC_LNDPRT									(uint8_t)(1<<4)
#define INT_SOURCE_SRC_TRANS									(uint8_t)(1<<5)
#define INT_SOURCE_SRC_FIFO										(uint8_t)(1<<6)
#define INT_SOURCE_SRC_ASLP										(uint8_t)(1<<7)	

#define IS_INT_SOURCE_TRUE(x, y)							(uint8_t)(x & y)		

#endif
