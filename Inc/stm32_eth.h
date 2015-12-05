#include "stm32f4xx_hal.h"
#if !defined (USE_LAN8700_PHY) && !defined (USE_DP83848_PHY) && !defined (USE_ST802RT1_PHY)
  /* #define USE_LAN8700_PHY */   /*!< USE_LAN8700_PHY: SMSC Ethernet PHY LAN8700 */
  /* #define USE_DP83848_PHY */  /*!< USE_DP83848_PHY: National Instruments Ethernet PHY DP83848 */
  /* #define USE_ST802RT1_PHY */   /*!< USE_ST802RT1_PHY: ST Ethernet PHY ST802RT1 */
#endif

/**--------------------------------------------------------------------------**/
/** 
  * @brief                           Ethernet PTP defines
  */ 
/**--------------------------------------------------------------------------**/
/**
  * @}
  */

/** @defgroup ETH_PTP_time_update_method 
  * @{
  */ 
#define ETH_PTP_FineUpdate        ((uint32_t)0x00000001)  /*!< Fine Update method */
#define ETH_PTP_CoarseUpdate      ((uint32_t)0x00000000)  /*!< Coarse Update method */
#define IS_ETH_PTP_UPDATE(UPDATE) (((UPDATE) == ETH_PTP_FineUpdate) || \
                                   ((UPDATE) == ETH_PTP_CoarseUpdate))

/**
  * @}
  */ 
#define ETH_MAC_IT_TST       ((uint32_t)0x00000200)  /*!< Time stamp trigger interrupt (on MAC) */


/** @defgroup ETH_PTP_Flags 
  * @{
  */ 
#define ETH_PTP_FLAG_TSARU        ((uint32_t)0x00000020)  /*!< Addend Register Update */
#define ETH_PTP_FLAG_TSITE        ((uint32_t)0x00000010)  /*!< Time Stamp Interrupt Trigger */
#define ETH_PTP_FLAG_TSSTU        ((uint32_t)0x00000008)  /*!< Time Stamp Update */
#define ETH_PTP_FLAG_TSSTI        ((uint32_t)0x00000004)  /*!< Time Stamp Initialize */
#define IS_ETH_PTP_GET_FLAG(FLAG) (((FLAG) == ETH_PTP_FLAG_TSARU) || \
                                   ((FLAG) == ETH_PTP_FLAG_TSITE) || \
                                   ((FLAG) == ETH_PTP_FLAG_TSSTU) || \
                                   ((FLAG) == ETH_PTP_FLAG_TSSTI))
/** 
  * @brief  ETH PTP subsecond increment  
  */ 
#define IS_ETH_PTP_SUBSECOND_INCREMENT(SUBSECOND) ((SUBSECOND) <= 0xFF)

/**
  * @}
  */ 


/** @defgroup ETH_PTP_time_sign 
  * @{
  */ 
#define ETH_PTP_PositiveTime      ((uint32_t)0x00000000)  /*!< Positive time value */
#define ETH_PTP_NegativeTime      ((uint32_t)0x80000000)  /*!< Negative time value */
#define IS_ETH_PTP_TIME_SIGN(SIGN) (((SIGN) == ETH_PTP_PositiveTime) || \
                                    ((SIGN) == ETH_PTP_NegativeTime))

/** 
  * @brief  ETH PTP time stamp low update  
  */ 
#define IS_ETH_PTP_TIME_STAMP_UPDATE_SUBSECOND(SUBSECOND) ((SUBSECOND) <= 0x7FFFFFFF)


/**
  * @}
  */

/** @defgroup ETH_Exported_Macros
  * @{
  */ 
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 


/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
