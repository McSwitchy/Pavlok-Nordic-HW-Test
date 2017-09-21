/*------------------------------------------------------------------------
**
**	@file				train.c
**
**  @brief			training service functions
**  
**  @details 		This module implements PAVLOK application service 
**							functions
**  
**  @note Attention!
**   
**
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <nrf_assert.h>

#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_log.h"
#include "nrf_delay.h"

/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "debug.h"
#include "accel_mag.h"

#define INCLUDE_CCCD (1)

#include "training.h"
#include "accel_mag.h"
#include "pwm.h"

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/


#define PLOK_DATA_DESC				"Training Data"
#define PLOK_TT_DESC			"Training Contol"


#define PLOK_TRAIN_CHAR_TT_LEN		(1)
#define PLOK_TRAIN_CHAR_DATA_LEN	(6)


#define PLOK_TRAIN_HD_TYPE_VIBRATE		(1)
#define PLOK_TRAIN_HD_TYPE_BEEP			(2)
#define PLOK_TRAIN_HD_TYPE_ZAP			(3)
#define PLOK_TRAIN_BP_TYPE_VIBRATE		(1)
#define PLOK_TRAIN_BP_TYPE_BEEP			(2)
#define PLOK_TRAIN_BP_TYPE_ZAP			(3)


#define HD_LR_VALUE							(0)
#define D_T_VALUE								(0x01)
#define BTN_LK_VALUE							(0x02)
#define S_P_VALUE							(0x04)
#define HD_L_VALUE							(0x10)
#define SL_TR_VALUE							(0x11)
#define DT_2_VALUE							(0x08)

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/
typedef struct
{
  uint8_t		HD_LR		: 1;
  uint8_t		D_T			: 1;
  uint8_t		BTN_LK		: 1;
  uint8_t		S_P			: 1;
  uint8_t		HD_L		: 1;
  uint8_t		SL_TR		: 1;
  uint8_t		DT_2		: 1;
  uint8_t		RESERVED	: 1;
} sTRAINControlByte_t;

typedef struct
{
  uint8_t		RESERVED_1	: 2;
  uint8_t		HD_TYPE		: 3;
  uint8_t		RESERVED_2  : 3;
  uint8_t		BP_TYPE		: 2;
} sTRAINDtapByte_t;

typedef struct
{
	bool								dirty_bit;
  uint8_t							data_value[PLOK_TRAIN_CHAR_DATA_LEN];
  sTRAINControlByte_t	stream_value;
  sTRAINDtapByte_t		dtap_value;

} sTRAINInfo_t;

/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/
static uint8_t * pavlok_data_descriptor 				= (uint8_t *)PLOK_DATA_DESC;
static uint8_t * pavlok_tt_descriptor 					= (uint8_t *)PLOK_TT_DESC;

static training_t			    m_training;
static training_init_t		m_training_init;

static training_t 		* 	p_training = &m_training;
static sTRAINInfo_t				service_info;

#define APP_TIMERS (1)
#ifdef APP_TIMERS
#define TRAIN_SEND_INTERVAL             pdMS_TO_TICKS(10)  /**< TRAIN_SEND_INTERVAL (ms). */

static TimerHandle_t                	m_total_timer = NULL;     /**< Definition of m_total_timer. */
static TimerHandle_t                	m_interval_timer = NULL;     /**< Definition of m_interval_timer. */

static void train_timers_start(void);
static void train_timers_stop(void);
#endif

static uint16_t	total_test_time         = 0;
static uint8_t 	granularity_event_time  = 0;

/*------------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/
static uint32_t training_send(eTRAIN_CHAR_LIST char_type);

#ifdef REMOVED // not used
/*------------------------------------------------------------------------
**
**	@fn		Function			training_service_get_handle
**
**	@brief	Description	get service handle
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@warn
**
**------------------------------------------------------------------------
*/

uint16_t training_service_get_handle(void)
{
  return p_training->service_handle;
}
#endif
#ifdef REMOVED // not used
/*------------------------------------------------------------------------
**
**	@fn		Function			training_service_get_connection_handle
**
**	@brief	Description	get service connection handle
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									connection handle is a zero when enabled
**
**	@warn
**
**------------------------------------------------------------------------
*/
uint16_t training_service_get_connection_handle(void)
{
  return p_training->conn_handle;
}
#endif 


#ifdef APP_TIMERS

/*------------------------------------------------------------------------
**
**	@fn		Function			train_timeout_handle
**
**	@brief	Description	service timer timeout function
**                      Send data to the phone app every tick timeout
**                      Stop timer at the end of the  total time
**
**	@param [in]						TimerHandle_t xTimer - not used
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void train_timeout_handle(TimerHandle_t xTimer)
{
	if (BLE_SERVICE_ENABLED == p_training->char_enable[TRAIN_CHAR_STREAM])
	{
		(void)training_send(TRAIN_CHAR_STREAM);
	}
}


static void train_total_test_timeout_handle(TimerHandle_t xTimer)
{
	pavlok_set_training_started(TRAINING_STATE_COMPLETE);
  train_timers_stop();
}

/*------------------------------------------------------------------------
**
**	@fn		Function			train_timers_init
**
**	@brief	Description	service timer initialization function
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void train_timers_init(void)
{
	if (0 < granularity_event_time)
	{
		pavlok_timers_init("TRNI", &m_interval_timer, granularity_event_time, train_timeout_handle);
		pavlok_timers_init("TRNT", &m_total_timer, total_test_time, train_total_test_timeout_handle);
	}
}

/*------------------------------------------------------------------------
**
**	@fn		Function			train_timers_start
**
**	@brief	Description	service timer start function
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void train_timers_start(void)
{
  if (pdFALSE == xTimerIsTimerActive(m_interval_timer))
  {
    if(pdPASS != xTimerChangePeriod(m_interval_timer, granularity_event_time, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
  else
  {
    APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
  }
	
  if (pdFALSE == xTimerIsTimerActive(m_total_timer))
  {
    // Start application timers.
    if(pdPASS != xTimerStart(m_total_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
  else
  {
    APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
  }
}

/*------------------------------------------------------------------------
**
**	@fn		Function			train_timers_stop
**
**	@brief	Description	service timer stop function
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void train_timers_stop(void)
{
  if (pdPASS == xTimerIsTimerActive(m_interval_timer))
  {
    if(pdPASS != xTimerStop(m_interval_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
      APP_ERROR_HANDLER(NRF_ERROR_BUSY);
    } 
 	}
  if (pdPASS == xTimerIsTimerActive(m_total_timer))
  {
    // Start application timers.
    if(pdPASS != xTimerStop(m_total_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
      APP_ERROR_HANDLER(NRF_ERROR_BUSY);
    } 
  }
  accelerometer_init_transient();
  
}
#endif

/*------------------------------------------------------------------------
**
**	@fn		Function			update_cccd_enabled
**
**	@brief	Description	enable the service - this comes from the app
**											zero disables and one enables
**
**	@param [in]					ble_evt_t * p_ble_evt - pointer the latest BLE event
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void update_cccd_enabled(ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (CCCD_MSG_SIZE == p_evt_write->len)
    {
    	if (p_evt_write->handle == p_training->stream_handle.cccd_handle)
    	{
    		p_training->char_enable[TRAIN_CHAR_STREAM] = p_evt_write->data[0];
     	}
    	else if (p_evt_write->handle == p_training->tt_handle.cccd_handle)
    	{
    		p_training->char_enable[TRAIN_CHAR_TT] = p_evt_write->data[0];
    	}
	}
}

/*------------------------------------------------------------------------
**
**	@fn		Function			training_send
**
**	@brief	Description	send service events/data to application
**
**	@param [in]					eTRAIN_CHAR_LIST char_type - the sub service that 
**											was requested by the application
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**------------------------------------------------------------------------
*/
static uint32_t training_send(eTRAIN_CHAR_LIST char_type)
{
    uint32_t err_code	=	NRF_ERROR_INVALID_PARAM;

    // Send value if connected and notifying
    if (p_training->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
				uint32_t							rtn_code = 0;
        uint16_t              hvx_len = 0;
				uint8_t								stream_buffer[PLOK_TRAIN_CHAR_DATA_LEN];

        /** ---------------------------------------------------------------
         **	Send data
         **	all data bytes are sent even if set to zero
         **	---------------------------------------------------------------
         */
        memset(&hvx_params, 0, sizeof(hvx_params));
        memset(stream_buffer, 0, PLOK_TRAIN_CHAR_DATA_LEN);

				rtn_code = read_stream_data(stream_buffer);
				if (0 == rtn_code)
				{
					hvx_len = PLOK_TRAIN_CHAR_DATA_LEN;
				}
				else
				{
					hvx_len = 0;  // this won't send
				}

        if (0 < hvx_len)
        { 
        	uint8_t * encoded_info = pavlok_common_get_encode_buffer();  // get common buffer
        	pavlok_encode(stream_buffer, encoded_info, hvx_len);

          hvx_params.p_data = encoded_info;
					hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
					hvx_params.handle = p_training->stream_handle.value_handle;
					hvx_params.offset = 0;
					hvx_params.p_len  = &hvx_len;
        }

        err_code = sd_ble_gatts_hvx(p_training->conn_handle, &hvx_params);
				DEBUGI_APP_500("SEND", err_code);
        if (err_code == NRF_SUCCESS)
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

// called from main/butten event to start the training
void start_training_session(void)
{
	train_timers_init();			
	train_timers_start();
	pavlok_set_training_started(TRAINING_STATE_ACTIVE);
}

void stop_training_session(void)
{
	train_timers_stop();					
}

/*------------------------------------------------------------------------
**
**	@fn		Function			on_write
**
**	@brief	Description	called from a ble event write
**
**	@param [in]					ble_evt_t * p_ble_evt 
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void on_write(ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if ((CCCD_MSG_SIZE == p_evt_write->len)
			&& ((p_evt_write->handle == p_training->stream_handle.cccd_handle)
			|| (p_evt_write->handle == p_training->tt_handle.cccd_handle)))
	{
		// update cccd
		update_cccd_enabled(p_ble_evt);
	}

 if ((p_evt_write->handle == p_training->tt_handle.value_handle)
		  && (BLE_SERVICE_ENABLED == p_training->char_enable[TRAIN_CHAR_TT]))
	{
#		define TT_CHAR_MIN_VALUE	(1)
#		define GT_CHAR_MIN_VALUE	(0x0A) // 10ms
#		define TT_CHAR_MAX_VALUE	(30)
#		define GT_CHAR_MAX_VALUE	(0xFA) // 250ms
		if (0 == p_evt_write->data[0])
		{
			if ((TT_CHAR_MIN_VALUE <= p_evt_write->data[1])
          && (TT_CHAR_MAX_VALUE >= p_evt_write->data[1]))
			{
				stop_training_session();	
				total_test_time = (PLOK_1_SEC * p_evt_write->data[1]);
			}
		}
		else if (1 == p_evt_write->data[0])
		{
			// values come in here in 0x10 count increments
			if ((GT_CHAR_MIN_VALUE <= p_evt_write->data[1])
          && (GT_CHAR_MAX_VALUE >= p_evt_write->data[1]))
			{
				stop_training_session();	
				granularity_event_time = p_evt_write->data[1];
			}
		}
		else if (2 == p_evt_write->data[0])
		{
			// set the accel gforce level
			// TODO enable later
			stop_training_session();	
		}
		else if (3 == p_evt_write->data[0])
		{
			stop_training_session();	
			// enable disable
			if ((1 == p_evt_write->data[1])
					&& (PLOK_10_MS <= granularity_event_time)
					&& (PLOK_1_SEC <= total_test_time))
			{
				pavlok_training_notifiy_user();
			}
		}
		else if (4 == p_evt_write->data[0])
		{
			// write to terminal
			stop_training_session();	
			// enable disable
			if ((1 == p_evt_write->data[1])
					&& (PLOK_10_MS <= granularity_event_time)
					&& (PLOK_1_SEC <= total_test_time))
			{
        accelerometer_init_continuouse_read();
        
				pavlok_training_start_stimulus();
				nrf_delay_ms(1100);
				start_training_session();
			}
		}
	}
}


/** ----------------------------------------------------------------------
**
**	@fn		Function			training_on_ble_evt
**
**	@brief	Description	called from the ble event handler in main
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void training_on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//			DEBUGS_APP_400("BLE_GAP_EVT_CONNECTED");
        	p_training->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
           break;

        case BLE_GAP_EVT_DISCONNECTED:
        	p_training->conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef APP_TIMERS
      train_timers_stop();
#endif
            break;

        case BLE_GATTS_EVT_WRITE:
           on_write(p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			training_data_char_add
**
**	@brief	Description	start the data gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t training_data_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code = NRF_ERROR_INVALID_PARAM;

  // use the  CCD security only in the first characteristic add
  ble_gatts_attr_md_t cccd_md;

  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vlen                        = 1;  // must be a one ????
  cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
  char_md.char_props.read   			= 1;
  char_md.char_props.notify   		= 1;
  char_md.p_char_user_desc            = pavlok_data_descriptor;
  char_md.char_user_desc_size         = strlen((char *)pavlok_data_descriptor); // use sizeof so no NULL is xmitted
  char_md.char_user_desc_max_size     = strlen((char *)pavlok_data_descriptor);
  char_md.p_cccd_md                   = &cccd_md;

	BLE_UUID_BLE_ASSIGN(service_uuid, eTRAINING_CHAR_READ_DATA_UUID);
  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth                     = 0;  // no acks required
  attr_md.wr_auth                     = 0;  // no acks required
  attr_md.vlen                        = 1;  // must be a one ????

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid              = &service_uuid;
  attr_char_value.p_attr_md           = &attr_md;
  attr_char_value.init_len            = PLOK_TRAIN_CHAR_DATA_LEN;
  attr_char_value.init_offs           = 0;
  attr_char_value.max_len             = PLOK_TRAIN_CHAR_DATA_LEN;
  attr_char_value.p_value             = NULL;

  err_code = sd_ble_gatts_characteristic_add(p_training->service_handle,
   			&char_md,
				&attr_char_value,
				&p_training->stream_handle);

   return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			training_control_char_add
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t training_control_char_add(void)
{
	ble_uuid_t          	service_uuid;
    ble_gatts_char_md_t 	char_md;
         ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
   uint32_t				err_code;

   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.auth_signed_wr   = 1;
    char_md.char_props.write   			= 1;
    char_md.char_props.read   			= 1;
    char_md.char_props.notify   		= 1;
    char_md.p_char_user_desc            = pavlok_tt_descriptor;
    char_md.char_user_desc_size         = strlen((char *)pavlok_tt_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)pavlok_tt_descriptor);
	char_md.p_char_pf = &presentation;

    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;


        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

		BLE_UUID_BLE_ASSIGN(service_uuid, eTRAINING_CHAR_CONTROL_UUID);

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = 2;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = 2;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_training->service_handle,
    			&char_md,
				&attr_char_value,
				&p_training->tt_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				get_train_service_data_cccd_handle
**
**	@brief	Description		return the cccd handle
**
**  @note   See include file for further infor on this function 
**
**	@warn		TODO remove if not used
**
**  ----------------------------------------------------------------------
*/
uint16_t	get_train_service_data_cccd_handle(void)
{
	return m_training.stream_handle.cccd_handle;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				training_service_init
**
**	@brief	Description		initialize the train service
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
uint32_t training_service_init()
{
	ble_uuid_t          service_uuid;
  uint32_t   					err_code = NRF_ERROR_NO_MEM;
  uint8_t             address_type = 0;

	service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;

    // Initialize service structure
    p_training->conn_handle                = BLE_CONN_HANDLE_INVALID;

    (void)memset(&m_training, 0, sizeof(m_training));
    (void)memset(&m_training_init, 0, sizeof(m_training_init));
    (void)memset(&service_uuid, 0, sizeof(service_uuid));
    (void)memset(&service_info, 0, sizeof(service_info));

  err_code = sd_ble_uuid_vs_add(&base_uuid128, &address_type);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  
  service_uuid.uuid = eTRAINING_SVC_UUID;
  service_uuid.type = address_type;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_training->service_handle);

    if (NRF_SUCCESS == err_code)
    {
        err_code = training_data_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = training_control_char_add();
    }

    return err_code;
}


/** @} */
