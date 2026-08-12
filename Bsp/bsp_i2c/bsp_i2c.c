/**
 * @file bsp_i2c.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP I2C 外设抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供平台无关的 I2C 主设备操作，包括基础收发、寄存器读写
 *       和设备就绪轮询。所有操作会先校验实例状态，再委托给
 *       平台特定的驱动操作。
 */

#include "bsp_i2c.h"

/* ======================== 内部验证函数 ======================== */

/**
 * @brief 校验 I2C 实例是否已正确初始化
 * @param me I2C 对象指针
 * @return 有效返回 BSP_STATUS_OK，空指针返回 BSP_STATUS_INVALID_ARGUMENT，
 *         未初始化返回 BSP_STATUS_NOT_INITIALIZED
 */
static bsp_status_t validate(const bsp_i2c_t *me){if(me==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized?BSP_STATUS_OK:BSP_STATUS_NOT_INITIALIZED;}

/**
 * @brief 检查 7 位 I2C 地址是否有效
 * @param address 7 位 I2C 从设备地址
 * @return 地址在有效范围内 (0-0x7F) 返回 true
 */
static bool address_valid(uint16_t address){return address<=0x7FU;}

/**
 * @brief 校验数据传输参数
 * @param address I2C 从设备地址
 * @param data 数据缓冲区指针
 * @param size 传输字节数
 * @param mode 传输模式（阻塞/中断/DMA）
 * @return 所有参数有效返回 true
 */
static bool transfer_valid(uint16_t address,const void *data,size_t size,bsp_transfer_mode_t mode){
    return address_valid(address)&&(data!=NULL)&&(size>0U)&&bsp_transfer_mode_is_valid(mode);}

/* ======================== 公共 API - 生命周期 ======================== */

/**
 * @brief 使用平台特定配置初始化 I2C 实例
 * @param me I2C 对象指针
 * @param config I2C 配置，包含设备句柄和驱动操作
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT，
 *         或驱动初始化返回的平台错误码
 */
bsp_status_t bsp_i2c_init(bsp_i2c_t *me,const bsp_i2c_config_t *config){bsp_status_t status;
    if((me==NULL)||(config==NULL)||(config->device_handle==NULL)||(config->driver_ops==NULL)||
       (config->driver_ops->transmit==NULL)||(config->driver_ops->receive==NULL))return BSP_STATUS_INVALID_ARGUMENT;
    *me=(bsp_i2c_t){0}; if(config->driver_ops->init!=NULL){status=config->driver_ops->init(config->device_handle);
        if(status!=BSP_STATUS_OK)return status;} me->device_handle=config->device_handle;
    me->driver_ops=config->driver_ops; me->callback=config->callback; me->user_context=config->user_context;
    me->is_initialized=true; return BSP_STATUS_OK;}

/**
 * @brief 反初始化 I2C 实例并释放硬件资源
 * @param me I2C 对象指针
 * @return 成功返回 BSP_STATUS_OK，或校验/驱动反初始化返回的错误码
 */
bsp_status_t bsp_i2c_deinit(bsp_i2c_t *me){bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    s=me->driver_ops->deinit?me->driver_ops->deinit(me->device_handle):BSP_STATUS_OK;
    if(s==BSP_STATUS_OK)me->is_initialized=false;return s;}

/**
 * @brief 注册 I2C 实例的异步事件回调
 * @param me I2C 对象指针
 * @param callback 事件回调函数指针
 * @param context 传递给回调函数的用户自定义上下文
 * @return 成功返回 BSP_STATUS_OK，或校验错误码
 */
bsp_status_t bsp_i2c_set_callback(bsp_i2c_t *me,bsp_event_callback_t callback,void *context){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;me->callback=callback;
    me->user_context=context;return BSP_STATUS_OK;}

/* ======================== 公共 API - 基础传输 ======================== */

/**
 * @brief 向 I2C 从设备发送数据
 * @param me I2C 对象指针
 * @param address 7 位 I2C 从设备地址
 * @param data 发送数据缓冲区
 * @param size 发送字节数
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout 操作超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，或校验/驱动错误码
 */
bsp_status_t bsp_i2c_transmit(bsp_i2c_t *me,uint16_t address,const uint8_t *data,size_t size,
                              bsp_transfer_mode_t mode,uint32_t timeout){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(!transfer_valid(address,data,size,mode))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->transmit(me->device_handle,address,data,size,mode,timeout);}

/**
 * @brief 从 I2C 从设备接收数据
 * @param me I2C 对象指针
 * @param address 7 位 I2C 从设备地址
 * @param data 接收数据缓冲区
 * @param size 接收字节数
 * @param mode 传输模式（阻塞/中断/DMA）
 * @param timeout 操作超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，或校验/驱动错误码
 */
bsp_status_t bsp_i2c_receive(bsp_i2c_t *me,uint16_t address,uint8_t *data,size_t size,
                             bsp_transfer_mode_t mode,uint32_t timeout){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(!transfer_valid(address,data,size,mode))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->receive(me->device_handle,address,data,size,mode,timeout);}

/* ======================== 公共 API - 存储器访问 ======================== */

/**
 * @brief 校验存储器操作参数，包括地址宽度
 * @param address 7 位 I2C 从设备地址
 * @param address_size 寄存器地址宽度（8 位或 16 位）
 * @param data 数据缓冲区指针
 * @param size 传输字节数
 * @param mode 传输模式
 * @return 所有参数（含地址宽度）有效返回 true
 */
static bool memory_valid(uint16_t address,bsp_i2c_memory_address_size_t address_size,const void *data,
                         size_t size,bsp_transfer_mode_t mode){return transfer_valid(address,data,size,mode)&&
    ((address_size==BSP_I2C_MEMORY_ADDRESS_8_BIT)||(address_size==BSP_I2C_MEMORY_ADDRESS_16_BIT));}

/**
 * @brief 向 I2C 设备的指定寄存器地址写入数据
 * @param me I2C 对象指针
 * @param address 7 位 I2C 从设备地址
 * @param memory 从设备内的寄存器/子地址
 * @param address_size 寄存器地址宽度（8 位或 16 位）
 * @param data 待写入数据
 * @param size 写入字节数
 * @param mode 传输模式
 * @param timeout 操作超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，驱动不支持返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_i2c_memory_write(bsp_i2c_t *me,uint16_t address,uint16_t memory,
    bsp_i2c_memory_address_size_t address_size,const uint8_t *data,size_t size,bsp_transfer_mode_t mode,
    uint32_t timeout){const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if(!memory_valid(address,address_size,data,size,mode))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->memory_write?me->driver_ops->memory_write(me->device_handle,address,memory,
        address_size,data,size,mode,timeout):BSP_STATUS_UNSUPPORTED;}

/**
 * @brief 从 I2C 设备的指定寄存器地址读取数据
 * @param me I2C 对象指针
 * @param address 7 位 I2C 从设备地址
 * @param memory 从设备内的寄存器/子地址
 * @param address_size 寄存器地址宽度（8 位或 16 位）
 * @param data 接收数据缓冲区
 * @param size 读取字节数
 * @param mode 传输模式
 * @param timeout 操作超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，驱动不支持返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_i2c_memory_read(bsp_i2c_t *me,uint16_t address,uint16_t memory,
    bsp_i2c_memory_address_size_t address_size,uint8_t *data,size_t size,bsp_transfer_mode_t mode,
    uint32_t timeout){const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if(!memory_valid(address,address_size,data,size,mode))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->memory_read?me->driver_ops->memory_read(me->device_handle,address,memory,
        address_size,data,size,mode,timeout):BSP_STATUS_UNSUPPORTED;}

/* ======================== 公共 API - 设备状态 ======================== */

/**
 * @brief 轮询 I2C 设备是否准备好通信
 * @param me I2C 对象指针
 * @param address 7 位 I2C 从设备地址
 * @param trials 轮询尝试次数
 * @param timeout 每次尝试间隔超时时间（毫秒）
 * @return 设备就绪返回 BSP_STATUS_OK，驱动不支持返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_i2c_is_device_ready(bsp_i2c_t *me,uint16_t address,uint32_t trials,uint32_t timeout){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if(!address_valid(address)||(trials==0U))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->is_device_ready?me->driver_ops->is_device_ready(me->device_handle,address,trials,timeout)
        :BSP_STATUS_UNSUPPORTED;}

/**
 * @brief 中止正在进行的针对某设备的 I2C 传输
 * @param me I2C 对象指针
 * @param address 7 位 I2C 从设备地址
 * @return 成功返回 BSP_STATUS_OK，驱动不支持返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_i2c_abort(bsp_i2c_t *me,uint16_t address){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(!address_valid(address))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->abort?me->driver_ops->abort(me->device_handle,address):BSP_STATUS_UNSUPPORTED;}

/**
 * @brief 查询 I2C 总线当前是否忙碌
 * @param me I2C 对象指针（只读）
 * @param busy 输出：true 表示忙碌，false 表示空闲
 * @return 成功返回 BSP_STATUS_OK，驱动不支持返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_i2c_get_busy(const bsp_i2c_t *me,bool *busy){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(busy==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->get_busy?me->driver_ops->get_busy(me->device_handle,busy):BSP_STATUS_UNSUPPORTED;}

/* ======================== 内部函数 - 事件通知 ======================== */

/**
 * @brief I2C 传输完成事件的内部通知分发器
 * @param me I2C 对象指针
 * @param event 事件类型（如传输完成）
 * @param status 传输状态
 * @param size 已传输字节数
 * @note 由平台驱动调用，用于通知上层异步事件。
 *       无效或空实例将被静默忽略。
 */
void bsp_i2c_notify(bsp_i2c_t *me,bsp_event_t event,bsp_status_t status,size_t size){
    if((me!=NULL)&&me->is_initialized&&(me->callback!=NULL))me->callback(event,status,size,me->user_context);}
