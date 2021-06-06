/*
 * stm32f407xx_dma_driver.h
 *
 *  Created on: 2021. 3. 9.
 *      Author: Ganghyeok Lim
 */

#ifndef INC_STM32F407XX_DMA_DRIVER_H_
#define INC_STM32F407XX_DMA_DRIVER_H_

#include "stm32f407xx.h"


/**
  * @brief  DMA Configuration Structure definition
  */
typedef struct
{
  uint32_t Channel;              /*!< Specifies the channel used for the specified stream.
                                      This parameter can be a value of @ref DMA_Channel_selection                    */

  uint32_t Direction;            /*!< Specifies if the data will be transferred from memory to peripheral,
                                      from memory to memory or from peripheral to memory.
                                      This parameter can be a value of @ref DMA_Data_transfer_direction              */

  uint32_t PeriphInc;            /*!< Specifies whether the Peripheral address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Peripheral_incremented_mode          */

  uint32_t MemInc;               /*!< Specifies whether the memory address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Memory_incremented_mode              */

  uint32_t PeriphDataAlignment;  /*!< Specifies the Peripheral data width.
                                      This parameter can be a value of @ref DMA_Peripheral_data_size                 */

  uint32_t MemDataAlignment;     /*!< Specifies the Memory data width.
                                      This parameter can be a value of @ref DMA_Memory_data_size                     */

  uint32_t Mode;                 /*!< Specifies the operation mode of the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_mode
                                      @note The circular buffer mode cannot be used if the memory-to-memory
                                            data transfer is configured on the selected Stream                        */

  uint32_t Priority;             /*!< Specifies the software priority for the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_Priority_level                       */

  uint32_t FIFOMode;             /*!< Specifies if the FIFO mode or Direct mode will be used for the specified stream.
                                      This parameter can be a value of @ref DMA_FIFO_direct_mode
                                      @note The Direct mode (FIFO mode disabled) cannot be used if the
                                            memory-to-memory data transfer is configured on the selected stream       */

  uint32_t FIFOThreshold;        /*!< Specifies the FIFO threshold level.
                                      This parameter can be a value of @ref DMA_FIFO_threshold_level                  */

  uint32_t MemBurst;             /*!< Specifies the Burst transfer configuration for the memory transfers.
                                      It specifies the amount of data to be transferred in a single non interruptible
                                      transaction.
                                      This parameter can be a value of @ref DMA_Memory_burst
                                      @note The burst mode is possible only if the address Increment mode is enabled. */

  uint32_t PeriphBurst;          /*!< Specifies the Burst transfer configuration for the peripheral transfers.
                                      It specifies the amount of data to be transferred in a single non interruptible
                                      transaction.
                                      This parameter can be a value of @ref DMA_Peripheral_burst
                                      @note The burst mode is possible only if the address Increment mode is enabled. */
} DMA_InitTypeDef;


/**
  * @brief  DMA State structures definition
  */
typedef enum
{
  DMA_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled */
  DMA_STATE_READY             = 0x01U,  /*!< DMA initialized and ready for use   */
  DMA_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing              */
  DMA_STATE_TIMEOUT           = 0x03U,  /*!< DMA timeout state                   */
  DMA_STATE_ERROR             = 0x04U,  /*!< DMA error state                     */
  DMA_STATE_ABORT             = 0x05U,  /*!< DMA Abort state                     */
} DMA_StateTypeDef;


/**
  * @brief  DMA Error Code structure definition
  */
typedef enum
{
  DMA_FULL_TRANSFER           = 0x00U,  /*!< Full transfer     */
  DMA_HALF_TRANSFER           = 0x01U   /*!< Half Transfer     */
} DMA_LevelCompleteTypeDef;


/**
  * @brief  DMA Error Code structure definition
  */
typedef enum
{
  DMA_XFER_CPLT_CB_ID         = 0x00U,  /*!< Full transfer     */
  DMA_XFER_HALFCPLT_CB_ID     = 0x01U,  /*!< Half Transfer     */
  DMA_XFER_M1CPLT_CB_ID       = 0x02U,  /*!< M1 Full Transfer  */
  DMA_XFER_M1HALFCPLT_CB_ID   = 0x03U,  /*!< M1 Half Transfer  */
  DMA_XFER_ERROR_CB_ID        = 0x04U,  /*!< Error             */
  DMA_XFER_ABORT_CB_ID        = 0x05U,  /*!< Abort             */
  DMA_XFER_ALL_CB_ID          = 0x06U   /*!< All               */
} DMA_CallbackIDTypeDef;


/**
  * @brief  DMA handle Structure definition
  */
typedef struct __DMA_HandleTypeDef
{
  DMA_Stream_TypeDef         *Instance;                                                        /*!< Register base address                  */

  DMA_InitTypeDef            Init;                                                             /*!< DMA communication parameters           */

  __IO DMA_StateTypeDef  State;                                                            /*!< DMA transfer state                     */

  void                       *Parent;                                                          /*!< Parent object state                    */

  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);         /*!< DMA transfer complete callback         */

  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);     /*!< DMA Half transfer complete callback    */

  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);       /*!< DMA transfer complete Memory1 callback */

  void                       (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);   /*!< DMA transfer Half complete Memory1 callback */

  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);        /*!< DMA transfer error callback            */

  void                       (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);        /*!< DMA transfer Abort callback            */

  __IO uint32_t              ErrorCode;                                                        /*!< DMA Error code                          */

  uint32_t                   StreamBaseAddress;                                                /*!< DMA Stream Base Address                */

  uint32_t                   StreamIndex;                                                      /*!< DMA Stream Index                       */

} DMA_HandleTypeDef;


typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


/** @defgroup DMA_Error_Code DMA Error Code
  * @brief    DMA Error Code
  * @{
  */
#define DMA_ERROR_NONE            0x00000000U    /*!< No error                               */
#define DMA_ERROR_TE              0x00000001U    /*!< Transfer error                         */
#define DMA_ERROR_FE              0x00000002U    /*!< FIFO error                             */
#define DMA_ERROR_DME             0x00000004U    /*!< Direct Mode error                      */
#define DMA_ERROR_TIMEOUT         0x00000020U    /*!< Timeout error                          */
#define DMA_ERROR_PARAM           0x00000040U    /*!< Parameter error                        */
#define DMA_ERROR_NO_XFER         0x00000080U    /*!< Abort requested with no Xfer ongoing   */
#define DMA_ERROR_NOT_SUPPORTED   0x00000100U    /*!< Not supported mode                     */

/** @defgroup DMA_Channel_selection DMA Channel selection
  * @brief    DMA channel selection
  * @{
  */
#define DMA_CHANNEL_0                 0x00000000U    /*!< DMA Channel 0 */
#define DMA_CHANNEL_1                 0x02000000U    /*!< DMA Channel 1 */
#define DMA_CHANNEL_2                 0x04000000U    /*!< DMA Channel 2 */
#define DMA_CHANNEL_3                 0x06000000U    /*!< DMA Channel 3 */
#define DMA_CHANNEL_4                 0x08000000U    /*!< DMA Channel 4 */
#define DMA_CHANNEL_5                 0x0A000000U    /*!< DMA Channel 5 */
#define DMA_CHANNEL_6                 0x0C000000U    /*!< DMA Channel 6 */
#define DMA_CHANNEL_7                 0x0E000000U    /*!< DMA Channel 7 */

/** @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @brief    DMA data transfer direction
  * @{
  */
#define DMA_PERIPH_TO_MEMORY          0x00000000U                 /*!< Peripheral to memory direction */
#define DMA_MEMORY_TO_PERIPH          ((uint32_t)DMA_SxCR_DIR_0)  /*!< Memory to peripheral direction */
#define DMA_MEMORY_TO_MEMORY          ((uint32_t)DMA_SxCR_DIR_1)  /*!< Memory to memory direction     */

/** @defgroup DMA_Peripheral_incremented_mode DMA Peripheral incremented mode
  * @brief    DMA peripheral incremented mode
  * @{
  */
#define DMA_PINC_ENABLE               ((uint32_t)DMA_SxCR_PINC)   /*!< Peripheral increment mode enable  */
#define DMA_PINC_DISABLE              0x00000000U                 /*!< Peripheral increment mode disable */

/** @defgroup DMA_Memory_incremented_mode DMA Memory incremented mode
  * @brief    DMA memory incremented mode
  * @{
  */
#define DMA_MINC_ENABLE               ((uint32_t)DMA_SxCR_MINC)   /*!< Memory increment mode enable  */
#define DMA_MINC_DISABLE              0x00000000U                 /*!< Memory increment mode disable */

/** @defgroup DMA_Peripheral_data_size DMA Peripheral data size
  * @brief    DMA peripheral data size
  * @{
  */
#define DMA_PDATAALIGN_BYTE           0x00000000U                  /*!< Peripheral data alignment: Byte     */
#define DMA_PDATAALIGN_HALFWORD       ((uint32_t)DMA_SxCR_PSIZE_0) /*!< Peripheral data alignment: HalfWord */
#define DMA_PDATAALIGN_WORD           ((uint32_t)DMA_SxCR_PSIZE_1) /*!< Peripheral data alignment: Word     */

/** @defgroup DMA_Memory_data_size DMA Memory data size
  * @brief    DMA memory data size
  * @{
  */
#define DMA_MDATAALIGN_BYTE           0x00000000U                  /*!< Memory data alignment: Byte     */
#define DMA_MDATAALIGN_HALFWORD       ((uint32_t)DMA_SxCR_MSIZE_0) /*!< Memory data alignment: HalfWord */
#define DMA_MDATAALIGN_WORD           ((uint32_t)DMA_SxCR_MSIZE_1) /*!< Memory data alignment: Word     */

/** @defgroup DMA_mode DMA mode
  * @brief    DMA mode
  * @{
  */
#define DMA_NORMAL                    0x00000000U                  /*!< Normal mode                  */
#define DMA_CIRCULAR                  ((uint32_t)DMA_SxCR_CIRC)    /*!< Circular mode                */
#define DMA_PFCTRL                    ((uint32_t)DMA_SxCR_PFCTRL)  /*!< Peripheral flow control mode */

/** @defgroup DMA_Priority_level DMA Priority level
  * @brief    DMA priority levels
  * @{
  */
#define DMA_PRIORITY_LOW              0x00000000U                 /*!< Priority level: Low       */
#define DMA_PRIORITY_MEDIUM           ((uint32_t)DMA_SxCR_PL_0)   /*!< Priority level: Medium    */
#define DMA_PRIORITY_HIGH             ((uint32_t)DMA_SxCR_PL_1)   /*!< Priority level: High      */
#define DMA_PRIORITY_VERY_HIGH        ((uint32_t)DMA_SxCR_PL)     /*!< Priority level: Very High */

/** @defgroup DMA_FIFO_direct_mode DMA FIFO direct mode
  * @brief    DMA FIFO direct mode
  * @{
  */
#define DMA_FIFOMODE_DISABLE          0x00000000U                 /*!< FIFO mode disable */
#define DMA_FIFOMODE_ENABLE           ((uint32_t)DMA_SxFCR_DMDIS) /*!< FIFO mode enable  */

/** @defgroup DMA_FIFO_threshold_level DMA FIFO threshold level
  * @brief    DMA FIFO level
  * @{
  */
#define DMA_FIFO_THRESHOLD_1QUARTERFULL       0x00000000U                  /*!< FIFO threshold 1 quart full configuration  */
#define DMA_FIFO_THRESHOLD_HALFFULL           ((uint32_t)DMA_SxFCR_FTH_0)  /*!< FIFO threshold half full configuration     */
#define DMA_FIFO_THRESHOLD_3QUARTERSFULL      ((uint32_t)DMA_SxFCR_FTH_1)  /*!< FIFO threshold 3 quarts full configuration */
#define DMA_FIFO_THRESHOLD_FULL               ((uint32_t)DMA_SxFCR_FTH)    /*!< FIFO threshold full configuration          */

/** @defgroup DMA_Memory_burst DMA Memory burst
  * @brief    DMA memory burst
  * @{
  */
#define DMA_MBURST_SINGLE             0x00000000U
#define DMA_MBURST_INC4               ((uint32_t)DMA_SxCR_MBURST_0)
#define DMA_MBURST_INC8               ((uint32_t)DMA_SxCR_MBURST_1)
#define DMA_MBURST_INC16              ((uint32_t)DMA_SxCR_MBURST)

/** @defgroup DMA_Peripheral_burst DMA Peripheral burst
  * @brief    DMA peripheral burst
  * @{
  */
#define DMA_PBURST_SINGLE             0x00000000U
#define DMA_PBURST_INC4               ((uint32_t)DMA_SxCR_PBURST_0)
#define DMA_PBURST_INC8               ((uint32_t)DMA_SxCR_PBURST_1)
#define DMA_PBURST_INC16              ((uint32_t)DMA_SxCR_PBURST)

/** @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @brief    DMA interrupts definition
  * @{
  */
#define DMA_IT_TC                     ((uint32_t)DMA_SxCR_TCIE)
#define DMA_IT_HT                     ((uint32_t)DMA_SxCR_HTIE)
#define DMA_IT_TE                     ((uint32_t)DMA_SxCR_TEIE)
#define DMA_IT_DME                    ((uint32_t)DMA_SxCR_DMEIE)
#define DMA_IT_FE                     0x00000080U

/** @defgroup DMA_flag_definitions DMA flag definitions
  * @brief    DMA flag definitions
  * @{
  */
#define DMA_FLAG_FEIF0_4              0x00000001U
#define DMA_FLAG_DMEIF0_4             0x00000004U
#define DMA_FLAG_TEIF0_4              0x00000008U
#define DMA_FLAG_HTIF0_4              0x00000010U
#define DMA_FLAG_TCIF0_4              0x00000020U
#define DMA_FLAG_FEIF1_5              0x00000040U
#define DMA_FLAG_DMEIF1_5             0x00000100U
#define DMA_FLAG_TEIF1_5              0x00000200U
#define DMA_FLAG_HTIF1_5              0x00000400U
#define DMA_FLAG_TCIF1_5              0x00000800U
#define DMA_FLAG_FEIF2_6              0x00010000U
#define DMA_FLAG_DMEIF2_6             0x00040000U
#define DMA_FLAG_TEIF2_6              0x00080000U
#define DMA_FLAG_HTIF2_6              0x00100000U
#define DMA_FLAG_TCIF2_6              0x00200000U
#define DMA_FLAG_FEIF3_7              0x00400000U
#define DMA_FLAG_DMEIF3_7             0x01000000U
#define DMA_FLAG_TEIF3_7              0x02000000U
#define DMA_FLAG_HTIF3_7              0x04000000U
#define DMA_FLAG_TCIF3_7              0x08000000U

/**************************************************************************************************************
 * 																											  *
 * 												User Macro Definition										  *
 * 									  																		  *
 **************************************************************************************************************/






/**************************************************************************************************************
 * 																											  *
 * 												User Macro Function											  *
 * 									  																		  *
 **************************************************************************************************************/

#define DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->Instance->CR |=  DMA_SxCR_EN)

#define DMA_DISABLE(__HANDLE__)     ((__HANDLE__)->Instance->CR &=  ~DMA_SxCR_EN)

#define DMA_GET_FLAG(__HANDLE__, __FLAG__)\
													(((uint32_t)((__HANDLE__)->Instance) > (uint32_t)DMA2_Stream3)? (DMA2->HISR & (__FLAG__)) :\
													((uint32_t)((__HANDLE__)->Instance) > (uint32_t)DMA1_Stream7)? (DMA2->LISR & (__FLAG__)) :\
													((uint32_t)((__HANDLE__)->Instance) > (uint32_t)DMA1_Stream3)? (DMA1->HISR & (__FLAG__)) : (DMA1->LISR & (__FLAG__)))

#define DMA_CLEAR_FLAG(__HANDLE__, __FLAG__) \
													(((uint32_t)((__HANDLE__)->Instance) > (uint32_t)DMA2_Stream3)? (DMA2->HIFCR = (__FLAG__)) :\
													 ((uint32_t)((__HANDLE__)->Instance) > (uint32_t)DMA1_Stream7)? (DMA2->LIFCR = (__FLAG__)) :\
													 ((uint32_t)((__HANDLE__)->Instance) > (uint32_t)DMA1_Stream3)? (DMA1->HIFCR = (__FLAG__)) : (DMA1->LIFCR = (__FLAG__)))

#define DMA_ENABLE_IT(__HANDLE__, __INTERRUPT__)   (((__INTERRUPT__) != DMA_IT_FE)? \
														((__HANDLE__)->Instance->CR |= (__INTERRUPT__)) : ((__HANDLE__)->Instance->FCR |= (__INTERRUPT__)))

#define DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  (((__INTERRUPT__) != DMA_IT_FE)? \
                                                        ((__HANDLE__)->Instance->CR & (__INTERRUPT__)) : \
                                                        ((__HANDLE__)->Instance->FCR & (__INTERRUPT__)))

#define LINKDMA(__HANDLE__, __PPP_DMA_FIELD__, __DMA_HANDLE__)               \
                        do{                                                      \
                              (__HANDLE__)->__PPP_DMA_FIELD__ = &(__DMA_HANDLE__); \
                              (__DMA_HANDLE__).Parent = (__HANDLE__);             \
                          } while(0U)

/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void DMA_Init(DMA_HandleTypeDef *pDMAHandle);
void DMA_Start_IT(DMA_HandleTypeDef *pDMAHandle, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
void DMA_IRQHandler(DMA_HandleTypeDef *pDMAHandle);
void DMA_RegisterCallback(DMA_HandleTypeDef *pDMAHandle, DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_pDMAHandle));





#endif /* INC_STM32F407XX_DMA_DRIVER_H_ */
