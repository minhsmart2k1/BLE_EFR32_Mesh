/*
 * driver_max30102.c
 *
 *  Created on: 19 thg 2, 2023
 *      Author: Anh Minh
 */
/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_max30102.c
 * @brief     driver max30102 source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-11-13
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/11/13  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_max30102.h"

/**
 * @brief chip information definition
 */
#define CHIP_NAME                 "Maxim Integrated MAX30102"        /**< chip name */
#define MANUFACTURER_NAME         "Maxim Integrated"                 /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        1.7f                               /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        2.0f                               /**< chip max supply voltage */
#define MAX_CURRENT               20.0f                              /**< chip max current */
#define TEMPERATURE_MIN           -40.0f                             /**< chip min operating temperature */
#define TEMPERATURE_MAX           85.0f                              /**< chip max operating temperature */
#define DRIVER_VERSION            1000                               /**< driver version */

/**
 * @brief iic address definition
 */
#define MAX30102_ADDRESS        0x57        /**< iic address */

/**
 * @brief chip register definition
 */
#define MAX30102_REG_INTERRUPT_STATUS_1          0x00        /**< interrupt status 1 register */
#define MAX30102_REG_INTERRUPT_STATUS_2          0x01        /**< interrupt status 2 register */
#define MAX30102_REG_INTERRUPT_ENABLE_1          0x02        /**< interrupt enable 1 register */
#define MAX30102_REG_INTERRUPT_ENABLE_2          0x03        /**< interrupt enable 2 register */
#define MAX30102_REG_FIFO_WRITE_POINTER          0x04        /**< fifo write pointer register */
#define MAX30102_REG_OVERFLOW_COUNTER            0x05        /**< overflow counter register */
#define MAX30102_REG_FIFO_READ_POINTER           0x06        /**< fifo read pointer register */
#define MAX30102_REG_FIFO_DATA_REGISTER          0x07        /**< fifo data register */
#define MAX30102_REG_FIFO_CONFIG                 0x08        /**< fifo config register */
#define MAX30102_REG_MODE_CONFIG                 0x09        /**< mode config register */
#define MAX30102_REG_SPO2_CONFIG                 0x0A        /**< spo2 config register */
#define MAX30102_REG_LED_PULSE_1                 0x0C        /**< led pulse amplitude 1 register */
#define MAX30102_REG_LED_PULSE_2                 0x0D        /**< led pulse amplitude 2 register */
#define MAX30102_REG_MULTI_LED_MODE_CONTROL_1    0x11        /**< multi led mode control 1 register */
#define MAX30102_REG_MULTI_LED_MODE_CONTROL_2    0x12        /**< multi led mode control 2 register */
#define MAX30102_REG_DIE_TEMP_INTEGER            0x1F        /**< die temperature integer register */
#define MAX30102_REG_DIE_TEMP_FRACTION           0x20        /**< die temperature fraction register */
#define MAX30102_REG_DIE_TEMP_CONFIG             0x21        /**< die temperature config register */
#define MAX30102_REG_REVISION_ID                 0xFE        /**< revision id register */
#define MAX30102_REG_PART_ID                     0xFF        /**< part id register */


  uint32_t raw_red[32];
  uint32_t raw_ir[32];
  uint8_t len;

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
void receive_callback(max30102_handle_t gs_handle, uint8_t type)
{
    switch (type)
    {
        case MAX30102_INTERRUPT_STATUS_FIFO_FULL :
        {
//          uint8_t res;
//                      uint8_t len;
//
//                      /* read data */
//                      len = 32;
//                      res = max30102_read(&gs_handle, (uint32_t *)raw_red, (uint32_t *)raw_ir, (uint8_t *)&len);
//                      if (res != 0)
//                      {
//                          app_log_debug("max30102: read failed.\n");
//                      }
            app_log_debug("max30102: irq fifo full.\n");

            break;
        }
        case MAX30102_INTERRUPT_STATUS_PPG_RDY :
        {
            app_log_debug("max30102: irq ppg rdy.\n");

            break;
        }
        case MAX30102_INTERRUPT_STATUS_ALC_OVF :
        {
            app_log_debug("max30102: irq alc ovf.\n");

            break;
        }
        case MAX30102_INTERRUPT_STATUS_PWR_RDY :
        {
            app_log_debug("max30102: irq pwr rdy.\n");

            break;
        }
        case MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY :
        {
            app_log_debug("max30102: irq die temp rdy.\n");

            break;
        }
        default :
        {
            app_log_debug("max30102: unknown code.\n");

            break;
        }
    }
}

/**
 * @brief     initialize the chip
 * @param[in] *handle points to a max30102 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 id is invalid
 *            - 5 reset failed
 *            - 6 reset fifo failed
 * @note      none
 */
uint8_t max30102_init(max30102_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    uint8_t part_id;

    if (handle == NULL)                                                                                     /* check handle */
    {
        return 2;                                                                                           /* return error */
    }

    part_id = readRegister8(MAX30102_ADDRESS, MAX30102_REG_PART_ID);                 /* read part id */

    if (part_id != 0x15)                                                                                    /* check part id */
    {
        app_log_debug("max30102: id is invalid.\n");                                                  /* id is invalid */                                                                     /* iic deinit */
        return 4;                                                                                           /* return error */
    }
    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);                /* read mode config */

    prev &= ~(1 << 6);                                                                                      /* clear config */
    prev |= 1 << 6;                                                                                         /* set 1 */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG, prev);               /* write mode config */
    if (res != 0)                                                                                           /* check result */
        {
        app_log_debug("max30102: write mode config failed.\n");                                       /* write mode config failed */                                                                      /* iic deinit */
            return 5;                                                                                           /* return error */
        }
    sl_sleeptimer_delay_millisecond(10);                                                                                  /* delay 10 ms */
    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);                /* read mode config */

    if ((prev & (1 << 6)) != 0)                                                                             /* check result */
    {
        app_log_debug("max30102: reset failed.\n");                                                   /* reset failed */

        return 5;                                                                                           /* return error */
    }
    prev = 0;                                                                                               /* set zero */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_FIFO_READ_POINTER, prev);         /* write fifo read pointer */
    if (res != 0)                                                                                           /* check result */
    {
        app_log_debug("max30102: write fifo read pointer failed.\n");                                 /* write fifo read pointer failed */
        return 6;                                                                                           /* return error */
    }
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_FIFO_WRITE_POINTER, prev);        /* write fifo write pointer */
    if (res != 0)                                                                                           /* check result */
    {
        app_log_debug("max30102: write fifo write pointer failed.\n");                                /* write fifo write pointer failed */
        return 6;                                                                                           /* return error */
    }
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_OVERFLOW_COUNTER, prev);          /* write overflow counter */
    if (res != 0)                                                                                           /* check result */
    {
        app_log_debug("max30102: write overflow counter failed.\n");                                  /* write overflow counter failed */
                                                                        /* iic deinit */
        return 6;                                                                                           /* return error */
    }
    handle->inited = 1;                                                                                     /* flag finish initialization */

    return 0;                                                                                               /* success return 0 */
}

/**
 * @brief     close the chip
 * @param[in] *handle points to a max30102 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 power down failed
 * @note      none
 */
uint8_t max30102_deinit(max30102_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);         /* read mode config */
    if (prev == 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: read mode config failed.\n");                                 /* read mode config failed */

        return 4;                                                                                    /* return error */
    }
    prev &= ~(1 << 7);                                                                               /* clear config */
    prev |= 1 << 7;                                                                                  /* set bool */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG, prev);        /* write mode config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write mode config failed.\n");                                /* write mode config failed */

        return 4;                                                                                    /* return error */
    }

    handle->inited = 0;                                                                              /* flag close */

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     irq handler
 * @param[in] *handle points to a max30102 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 run failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */

uint8_t max30102_irq_handler(max30102_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                            /* check handle */
    {
        return 2;                                                                                                  /* return error */
    }
    if (handle->inited != 1)                                                                                       /* check handle initialization */
    {
        return 3;                                                                                                  /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_STATUS_1);                /* read interrupt status1 */
    if ((prev & (1 << MAX30102_INTERRUPT_STATUS_FIFO_FULL)) != 0)                                                  /* check fifo full */
    {

        receive_callback(*handle, MAX30102_INTERRUPT_STATUS_FIFO_FULL);                                         /* run callback */
    }
    if ((prev & (1 << MAX30102_INTERRUPT_STATUS_PPG_RDY)) != 0)                                                    /* check ppg ready */
    {

        receive_callback(*handle,MAX30102_INTERRUPT_STATUS_PPG_RDY);                                           /* run callback */

    }
    if ((prev & (1 << MAX30102_INTERRUPT_STATUS_ALC_OVF)) != 0)                                                    /* check alc ovf */
    {
        receive_callback(*handle,MAX30102_INTERRUPT_STATUS_ALC_OVF);                                           /* run callback */

    }
    if ((prev & (1 << MAX30102_INTERRUPT_STATUS_PWR_RDY)) != 0)                                                    /* check pwr ready */
    {
            receive_callback(*handle,MAX30102_INTERRUPT_STATUS_PWR_RDY);                                           /* run callback */
    }
    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_STATUS_2);                /* read interrupt status2 */

    if ((prev & (1 << MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY)) != 0)                                               /* check die temp ready */
    {
        uint8_t prev1;

        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_DIE_TEMP_INTEGER);              /* read die temp integer */

        handle->raw = (uint16_t)prev << 4;                                                                         /* set integer part */
        prev1 = readRegister8(MAX30102_ADDRESS, MAX30102_REG_DIE_TEMP_FRACTION);            /* read die temp fraction */

        handle->raw = handle->raw | prev1;                                                                         /* set fraction part */
        handle->temperature = (float)(prev) + (float)(prev1) * 0.0625f;                                            /* set the temperature */
        handle->finished_flag = 1;                                                                                 /* set flag */

           receive_callback(*handle, MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY);                                      /* run callback */
    }

    return 0;                                                                                                      /* success return 0 */
}

/**
 * @brief         read the data
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *raw_red points to a red raw data buffer
 * @param[out]    *raw_ir points to an ir raw data buffer
 * @param[in,out] *len points to a length buffer
 * @return        status code
 *                - 0 success
 *                - 1 read failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 *                - 4 fifo overrun
 *                - 5 mode is invalid
 * @note          none
 */
uint8_t max30102_read(max30102_handle_t *handle, uint32_t *raw_red, uint32_t *raw_ir, uint8_t *len)
{
    uint8_t res;
    uint8_t prev;
    uint8_t mode;
    uint8_t k;
    uint8_t read_point;
    uint8_t write_point;
    uint8_t l;
    uint8_t bit;
    uint8_t i;
    uint8_t r;

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_STATUS_1);                /* read interrupt status1 */
    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_STATUS_2);                /* read interrupt status2 */
    if (handle == NULL)                                                                                           /* check handle */
    {
        app_log_debug("max30102: not exit.\n");
        return 2;                                                                                                 /* return error */
    }
    if (handle->inited != 1)                                                                                      /* check handle initialization */
    {
        app_log_debug("max30102: not init.\n");
        return 3;                                                                                                 /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_OVERFLOW_COUNTER);                 /* read overflow counter */

    r = 0;                                                                                                        /* set 0 */
    if (prev == 0)                                                                                                /* check overflow */
    {
        r = 4;                                                                                                    /* set 4 */

        app_log_debug("max30102: fifo overrun.\n");                                                         /* fifo overrun*/
    }
    read_point = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_READ_POINTER);          /* read fifo read point */

    write_point = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_WRITE_POINTER);        /* read fifo write point */


    if (write_point > read_point)                                                                                 /* check point */
    {
        l = write_point - read_point;                                                                             /* get length */
    }
    else
    {
        l = 32 + write_point - read_point;                                                                        /* get length */
    }

    *len = ((*len) > l) ? l : (*len);                                                                             /* set read length */
    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);                      /* read mode config */
    if (prev == 0)                                                                                                 /* check result */
    {
        app_log_debug("max30102: read mode config failed.\n");                                              /* read mode config failed */

        return 1;                                                                                                 /* return error */
    }
    app_log_debug("length: %d\n\r", *len);
    mode = (max30102_mode_t)(prev & 0x7);                                                                         /* get mode */
    if (mode == MAX30102_MODE_HEART_RATE)                                                                         /* check heart rate mode */
    {
        k = 3;                                                                                                    /* 3 */
    }
    else if (mode == MAX30102_MODE_SPO2)                                                                          /* check spo2 mode*/
    {
        k = 6;                                                                                                    /* 6 */
    }
    else if (mode == MAX30102_MODE_MULTI_LED)                                                                     /* check multi led mode */
    {
        k = 6;                                                                                                    /* 6 */
    }
    else
    {
        app_log_debug("max30105: mode is invalid.\n");                                                      /* mode is invalid */

        return 5;                                                                                                 /* return error */
    }

    res = readRegister8_buf(MAX30102_ADDRESS, MAX30102_REG_FIFO_DATA_REGISTER, handle->buf, (*len) * k);           /* read fifo read point */

    if (res == 0)                                                                                                 /* check result */
    {
        app_log_debug("max30102: read fifo data register failed.\n");                                       /* read fifo data register failed */

        return 1;                                                                                                 /* return error */
    }
    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG);                      /* read spo2 config */
    if (prev == 0)                                                                                                 /* check result */
    {
        app_log_debug("max30102: read spo2 config failed.\n");                                              /* read spo2 config failed */

        return 1;                                                                                                 /* return error */
    }
    prev = prev & 0x3;                                                                                            /* get config */
    if (prev == 0)                                                                                                /* if 0 */
    {
        bit = 3;                                                                                                  /* 15 bits */
    }
    else if (prev == 1)                                                                                           /* if 1 */
    {
        bit = 2;                                                                                                  /* 16 bits */
    }
    else if (prev == 2)                                                                                           /* if 2 */
    {
        bit = 1;                                                                                                  /* 17 bits */
    }
    else                                                                                                          /* if 3 */
    {
        bit = 0;                                                                                                  /* 18 bits */
    }

    for (i = 0; i < (*len); i++)                                                                                  /* copy data */
    {
        if (mode == MAX30102_MODE_HEART_RATE)                                                                     /* check red mode */
        {
            raw_red[i] = ((uint32_t)handle->buf[i * 3 + 0] << 16) |                                               /* get raw red data */
                         ((uint32_t)handle->buf[i * 3 + 1] << 8) |                                                /* get raw red data */
                         ((uint32_t)handle->buf[i * 3 + 2] << 0);                                                 /* get raw red data */
            raw_red[i] = (raw_red[i] & 0x3FFFF) >> bit;                                                                       /* right shift bit */
        }
        else
        {
            raw_red[i] = ((uint32_t)handle->buf[i * 6 + 0] << 16) |                                               /* get raw red data */
                         ((uint32_t)handle->buf[i * 6 + 1] << 8) |                                                /* get raw red data */
                         ((uint32_t)handle->buf[i * 6 + 2] << 0);                                                 /* get raw red data */
            raw_red[i] = raw_red[i] >> bit;                                                                       /* right shift bit */
            raw_ir[i] = ((uint32_t)handle->buf[i * 6 + 3] << 16) |                                                /* get raw ir data */
                        ((uint32_t)handle->buf[i * 6 + 4] << 8) |                                                 /* get raw ir data */
                        ((uint32_t)handle->buf[i * 6 + 5] << 0);                                                  /* get raw ir data */
            raw_ir[i] = (raw_ir[i] & 0x3FFFF) >> bit;                                                                         /* right shift bit */
        }
    }

    return r;                                                                                                     /* success return 0 */
}

/**
 * @brief      read the temperature
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *raw points to a raw data buffer
 * @param[out] *temp points to a converted temperature buffer
 * @return     status code
 *             - 0 success
 *             - 1 read temperature failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_read_temperature(max30102_handle_t *handle, uint16_t *raw, float *temp)
{
    uint8_t res;
    uint8_t prev;
    uint16_t timeout;

    if (handle == NULL)                                                                                        /* check handle */
    {
        return 2;                                                                                              /* return error */
    }
    if (handle->inited != 1)                                                                                   /* check handle initialization */
    {
        return 3;                                                                                              /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_ENABLE_2);            /* read interrupt enable2 */

    if ((prev & (1 << 1)) == 0)                                                                                /* check config */
    {
        prev &= ~(1 << 1);                                                                                     /* clear interrupt */
        prev |= 1 << 1;                                                                                        /* set interrupt */
        res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_ENABLE_2, prev);       /* write interrupt enable2 */
        if (res != 0)                                                                                          /* check result */
        {
            app_log_debug("max30102: write interrupt enable2 failed.\n");                                /* write interrupt enable2 failed */

            return 1;                                                                                          /* return error */
        }
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_DIE_TEMP_CONFIG);               /* read die temp config */
    if (prev == 0)                                                                                              /* check result */

    prev &= ~(1 << 0);                                                                                         /* clear config */
    prev |= (1 << 0);                                                                                          /* set bool */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_DIE_TEMP_CONFIG, prev);              /* write die temp config */
    if (res != 0)                                                                                              /* check result */
    {
        app_log_debug("max30102: write die temp config failed.\n");                                      /* write die temp config failed */

        return 1;                                                                                              /* return error */
    }

    timeout = 5000;                                                                                            /* set 5000 ms */
    handle->finished_flag = 0;                                                                                 /* clear finished flag */
    while (timeout != 0)                                                                                       /* timeout */
    {
        sl_sleeptimer_delay_millisecond(1);                                                                                   /* delay 1 ms */
        //app_log_debug("%d\n\r", timeout);
        timeout--;                                                                                             /* timeout */
        max30102_irq_handler(handle);
        if (handle->finished_flag != 0)                                                                        /* check finished flag */
        {
            break;                                                                                             /* break */
        }
    }
    if (timeout == 0)                                                                                          /* check timeout */
    {
        app_log_debug("max30102: read timeout.\n");                                                      /* read timeout */

        return 1;                                                                                              /* return error */
    }
    *raw = handle->raw;                                                                                        /* get raw */
    *temp = handle->temperature;                                                                               /* get temperature */

    return 0;                                                                                                  /* success return 0 */
}

/**
 * @brief      get the interrupt status
 * @param[in]  *handle points to a max30102 handle structure
 * @param[in]  status is the interrupt status
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt status failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_interrupt_status(max30102_handle_t *handle, max30102_interrupt_status_t status, max30102_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                        /* check handle */
    {
        return 2;                                                                                              /* return error */
    }
    if (handle->inited != 1)                                                                                   /* check handle initialization */
    {
        return 3;                                                                                              /* return error */
    }

    if (status == MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY)                                                      /* if die temp ready status */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_STATUS_2);        /* read interrupt status2 */

        *enable = (max30102_bool_t)((prev >> status) & 0x01);                                                  /* get bool */

        return 0;                                                                                              /* success return 0 */
    }
    else
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_STATUS_1);        /* read interrupt status1 */
        *enable = (max30102_bool_t)((prev >> status) & 0x01);                                                  /* get bool */

        return 0;                                                                                              /* success return 0 */
    }
}

/**
 * @brief     set the interrupt bool
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] type is the interrupt type
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set interrupt failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_interrupt(max30102_handle_t *handle, max30102_interrupt_t type, max30102_bool_t enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                        /* check handle */
    {
        return 2;                                                                                              /* return error */
    }
    if (handle->inited != 1)                                                                                   /* check handle initialization */
    {
        return 3;                                                                                              /* return error */
    }

    if (type == MAX30102_INTERRUPT_DIE_TEMP_RDY_EN)                                                            /* if internal temperature enable */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_ENABLE_2);        /* read interrupt enable2 */

        prev &= ~(1 << type);                                                                                  /* clear interrupt */
        prev |= enable << type;                                                                                /* set interrupt */
        res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_ENABLE_2, prev);       /* write interrupt enable2 */
        if (res != 0)                                                                                          /* check result */
        {
            app_log_debug("max30102: write interrupt enable2 failed.\n");                                /* write interrupt enable2 failed */

            return 1;                                                                                          /* return error */
        }

        return 0;                                                                                              /* success return 0 */
    }
    else
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_ENABLE_1);        /* read interrupt enable1 */

        prev &= ~(1 << type);                                                                                  /* clear interrupt */
        prev |= enable << type;                                                                                /* set interrupt */
        res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_ENABLE_1, prev);       /* write interrupt enable1 */
        if (res != 0)                                                                                          /* check result */
        {
            app_log_debug("max30102: write interrupt enable1 failed.\n");                                /* write interrupt enable1 failed */

            return 1;                                                                                          /* return error */
        }

        return 0;                                                                                              /* success return 0 */
    }
}

/**
 * @brief      get the interrupt bool
 * @param[in]  *handle points to a max30102 handle structure
 * @param[in]  type is the interrupt type
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_interrupt(max30102_handle_t *handle, max30102_interrupt_t type, max30102_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                        /* check handle */
    {
        return 2;                                                                                              /* return error */
    }
    if (handle->inited != 1)                                                                                   /* check handle initialization */
    {
        return 3;                                                                                              /* return error */
    }

    if (type == MAX30102_INTERRUPT_DIE_TEMP_RDY_EN)                                                            /* if internal temperature enable */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_ENABLE_2);        /* read interrupt enable2 */

        *enable = (max30102_bool_t)((prev >> type) & 0x01);                                                    /* get bool */

        return 0;                                                                                              /* success return 0 */
    }
    else
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_INTERRUPT_ENABLE_1);        /* read interrupt enable1 */

        *enable = (max30102_bool_t)((prev >> type) & 0x01);                                                    /* get bool */

        return 0;                                                                                              /* success return 0 */
    }
}

/**
 * @brief     set the fifo write pointer
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] pointer is the written pointer
 * @return    status code
 *            - 0 success
 *            - 1 set fifo write pointer failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 pointer can't be over 0x1F
 * @note      pointer <= 0x1F
 */
uint8_t max30102_set_fifo_write_pointer(max30102_handle_t *handle, uint8_t pointer)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                     /* check handle */
    {
        return 2;                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                /* check handle initialization */
    {
        return 3;                                                                                           /* return error */
    }
    if (pointer > 0x1F)                                                                                     /* check pointer */
    {
        app_log_debug("max30102: pointer can't be over 0x1F.\n");                                     /* pointer can't be over 0x1F */

        return 4;                                                                                           /* return error */
    }

    prev = pointer & 0x1F;                                                                                  /* set pointer */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_FIFO_WRITE_POINTER, prev);        /* write fifo write pointer */
    if (res != 0)                                                                                           /* check result */
    {
        app_log_debug("max30102: write fifo write pointer failed.\n");                                /* write fifo write pointer failed */

        return 1;                                                                                           /* return error */
    }

    return 0;                                                                                               /* success return 0 */
}

/**
 * @brief      get the fifo write pointer
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *pointer points to a pointer buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fifo write pointer failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_fifo_write_pointer(max30102_handle_t *handle, uint8_t *pointer)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                    /* check handle */
    {
        return 2;                                                                                          /* return error */
    }
    if (handle->inited != 1)                                                                               /* check handle initialization */
    {
        return 3;                                                                                          /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_WRITE_POINTER);        /* read fifo write pointer */

    *pointer = prev & 0x1F;                                                                                /* get pointer */

    return 0;                                                                                              /* success return 0 */
}

/**
 * @brief     set the fifo overflow counter
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] counter is the overflow counter
 * @return    status code
 *            - 0 success
 *            - 1 set fifo overflow counter failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 counter can't be over 0x1F
 * @note      counter <= 0x1F
 */
uint8_t max30102_set_fifo_overflow_counter(max30102_handle_t *handle, uint8_t counter)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                   /* check handle */
    {
        return 2;                                                                                         /* return error */
    }
    if (handle->inited != 1)                                                                              /* check handle initialization */
    {
        return 3;                                                                                         /* return error */
    }
    if (counter > 0x1F)                                                                                   /* check counter */
    {
        app_log_debug("max30102: counter can't be over 0x1F.\n");                                   /* counter can't be over 0x1F */

        return 4;                                                                                         /* return error */
    }

    prev = counter & 0x1F;                                                                                /* set counter */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_OVERFLOW_COUNTER, prev);        /* set fifo overflow counter */
    if (res != 0)                                                                                         /* check result */
    {
        app_log_debug("max30102: set fifo overflow counter failed.\n");                             /* set fifo overflow counter failed */

        return 1;                                                                                         /* return error */
    }

    return 0;                                                                                             /* success return 0 */
}

/**
 * @brief      get the fifo overflow counter
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *counter points to a counter buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fifo overflow counter failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_fifo_overflow_counter(max30102_handle_t *handle, uint8_t *counter)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                  /* check handle */
    {
        return 2;                                                                                        /* return error */
    }
    if (handle->inited != 1)                                                                             /* check handle initialization */
    {
        return 3;                                                                                        /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_OVERFLOW_COUNTER);        /* get fifo overflow counter */

    *counter = prev & 0x1F;                                                                              /* get pointer */

    return 0;                                                                                            /* success return 0 */
}

/**
 * @brief     set the fifo read pointer
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] pointer is the read pointer
 * @return    status code
 *            - 0 success
 *            - 1 set fifo read pointer failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 pointer can't be over 0x1F
 * @note      pointer <= 0x1F
 */
uint8_t max30102_set_fifo_read_pointer(max30102_handle_t *handle, uint8_t pointer)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                    /* check handle */
    {
        return 2;                                                                                          /* return error */
    }
    if (handle->inited != 1)                                                                               /* check handle initialization */
    {
        return 3;                                                                                          /* return error */
    }
    if (pointer > 0x1F)                                                                                    /* check pointer */
    {
        app_log_debug("max30102: pointer can't be over 0x1F.\n");                                    /* pointer can't be over 0x1F */

        return 4;                                                                                          /* return error */
    }

    prev = pointer & 0x1F;                                                                                 /* set pointer */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_FIFO_READ_POINTER, prev);        /* write fifo read pointer */
    if (res != 0)                                                                                          /* check result */
    {
        app_log_debug("max30102: write fifo read pointer failed.\n");                                /* write fifo read pointer failed */

        return 1;                                                                                          /* return error */
    }

    return 0;                                                                                              /* success return 0 */
}

/**
 * @brief      get the fifo read pointer
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *pointer points to a pointer buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fifo read pointer failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_fifo_read_pointer(max30102_handle_t *handle, uint8_t *pointer)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                   /* check handle */
    {
        return 2;                                                                                         /* return error */
    }
    if (handle->inited != 1)                                                                              /* check handle initialization */
    {
        return 3;                                                                                         /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_READ_POINTER);        /* read fifo read pointer */

    *pointer = prev & 0x1F;                                                                               /* get pointer */

    return 0;                                                                                             /* success return 0 */
}

/**
 * @brief     set the fifo data
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] data is the fifo data
 * @return    status code
 *            - 0 success
 *            - 1 set fifo data failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_fifo_data(max30102_handle_t *handle, uint8_t data)
{
    uint8_t res;

    if (handle == NULL)                                                                                     /* check handle */
    {
        return 2;                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                /* check handle initialization */
    {
        return 3;                                                                                           /* return error */
    }

    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_FIFO_DATA_REGISTER, data);        /* write fifo data register */
    if (res != 0)                                                                                           /* check result */
    {
        app_log_debug("max30102: write fifo data register failed.\n");                                /* write fifo data register failed */

        return 1;                                                                                           /* return error */
    }

    return 0;                                                                                               /* success return 0 */
}

/**
 * @brief      get the fifo data
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *data points to a fifo data buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fifo data failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_fifo_data(max30102_handle_t *handle, uint8_t *data)
{
    uint8_t res;

    if (handle == NULL)                                                                                   /* check handle */
    {
        return 2;                                                                                         /* return error */
    }
    if (handle->inited != 1)                                                                              /* check handle initialization */
    {
        return 3;                                                                                         /* return error */
    }

    *data = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_DATA_REGISTER);        /* read fifo data register */


    return 0;                                                                                             /* success return 0 */
}

/**
 * @brief     set the fifo sample averaging
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] sample is the fifo sample averaging
 * @return    status code
 *            - 0 success
 *            - 1 set fifo sample averaging failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_fifo_sample_averaging(max30102_handle_t *handle, max30102_sample_averaging_t sample)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG);         /* read fifo config */

    prev &= ~(0x7 << 5);                                                                             /* clear config */
    prev |= sample << 5;                                                                             /* set sample */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG, prev);        /* write fifo config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write fifo config failed.\n");                                /* write fifo config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get the fifo sample averaging
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *sample points to a fifo sample averaging buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fifo sample averaging failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_fifo_sample_averaging(max30102_handle_t *handle, max30102_sample_averaging_t *sample)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG);         /* read fifo config */

    *sample = (max30102_sample_averaging_t)(0x7 & (prev >> 5));                                      /* get sample averaging */

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     enable or disable the fifo roll
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set fifo roll failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_fifo_roll(max30102_handle_t *handle, max30102_bool_t enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG);         /* read fifo config */

    prev &= ~(0x1 << 4);                                                                             /* clear config */
    prev |= enable << 4;                                                                             /* set enable */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG, prev);        /* write fifo config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write fifo config failed.\n");                                /* write fifo config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get the fifo roll status
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fifo roll failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_fifo_roll(max30102_handle_t *handle, max30102_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG);         /* read fifo config */

    *enable = (max30102_bool_t)(0x1 & (prev >> 4));                                                  /* get bool */

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     set the fifo almost full value
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] value is the fifo almost full value
 * @return    status code
 *            - 0 success
 *            - 1 set fifo almost full failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 value can't be over 0xF
 * @note      none
 */
uint8_t max30102_set_fifo_almost_full(max30102_handle_t *handle, uint8_t value)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }
    if (value > 0xF)                                                                                 /* check value */
    {
        app_log_debug("max30102: value can't be over 0xF.\n");                                 /* value can't be over 0xF */

        return 4;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG);         /* read fifo config */

    prev &= ~(0xF << 0);                                                                             /* clear config */
    prev |= value << 0;                                                                              /* set value */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG, prev);        /* write fifo config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write fifo config failed.\n");                                /* write fifo config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get the fifo almost full value
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *value points to a fifo almost full value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fifo almost full failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_get_fifo_almost_full(max30102_handle_t *handle, uint8_t *value)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                             /* check handle */
    {
        return 2;                                                                                   /* return error */
    }
    if (handle->inited != 1)                                                                        /* check handle initialization */
    {
        return 3;                                                                                   /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_FIFO_CONFIG);        /* read fifo config */

    *value = prev & 0xF;                                                                            /* get value */

    return 0;                                                                                       /* success return 0 */
}

/**
 * @brief     set the shutdown
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set shutdown failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_shutdown(max30102_handle_t *handle, max30102_bool_t enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);         /* read mode config */

    prev &= ~(1 << 7);                                                                               /* clear config */
    prev |= enable << 7;                                                                             /* set bool */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG, prev);        /* write mode config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write mode config failed.\n");                                /* write mode config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get the shutdown
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get shutdown failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_shutdown(max30102_handle_t *handle, max30102_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);         /* read mode config */

    *enable = (max30102_bool_t)((prev >> 7) & 0x01);                                                 /* get bool */

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     reset the chip
 * @param[in] *handle points to a max30102 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 reset failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_reset(max30102_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);         /* read mode config */

    prev &= ~(1 << 6);                                                                               /* clear config */
    prev |= 1 << 6;                                                                                  /* set 1 */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG, prev);        /* write mode config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write mode config failed.\n");                                /* write mode config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     set the mode
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] mode is the chip mode
 * @return    status code
 *            - 0 success
 *            - 1 set mode failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_mode(max30102_handle_t *handle, max30102_mode_t mode)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);         /* read mode config */

    prev &= ~(7 << 0);                                                                               /* clear config */
    prev |= mode << 0;                                                                               /* set mode */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG, prev);        /* write mode config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write mode config failed.\n");                                /* write mode config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get the mode
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *mode points to a chip mode buffer
 * @return     status code
 *             - 0 success
 *             - 1 get mode failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_mode(max30102_handle_t *handle, max30102_mode_t *mode)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MODE_CONFIG);         /* read mode config */

    *mode = (max30102_mode_t)(prev & 0x7);                                                           /* get mode */

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     set the spo2 adc range
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] range is the spo2 adc range
 * @return    status code
 *            - 0 success
 *            - 1 set spo2 adc range failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_spo2_adc_range(max30102_handle_t *handle, max30102_spo2_adc_range_t range)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG);         /* read spo2 config */

    prev &= ~(3 << 5);                                                                               /* clear config */
    prev |= range << 5;                                                                              /* set range */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG, prev);        /* write spo2 config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write spo2 config failed.\n");                                /* write spo2 config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get the spo2 adc range
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *range points to an spo2 adc range buffer
 * @return     status code
 *             - 0 success
 *             - 1 get spo2 adc range failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_spo2_adc_range(max30102_handle_t *handle, max30102_spo2_adc_range_t *range)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG);         /* read spo2 config */

    *range = (max30102_spo2_adc_range_t)((prev >> 5) & 0x3);                                         /* get range */

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     set the spo2 sample rate
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] rate is the spo2 sample rate
 * @return    status code
 *            - 0 success
 *            - 1 set spo2 sample rate failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_spo2_sample_rate(max30102_handle_t *handle, max30102_spo2_sample_rate_t rate)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG);         /* read spo2 config */

    prev &= ~(7 << 2);                                                                               /* clear config */
    prev |= rate << 2;                                                                               /* set sample rate */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG, prev);        /* write spo2 config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write spo2 config failed.\n");                                /* write spo2 config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get the spo2 sample rate
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *rate points to an spo2 sample rate buffer
 * @return     status code
 *             - 0 success
 *             - 1 get spo2 sample rate failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_spo2_sample_rate(max30102_handle_t *handle, max30102_spo2_sample_rate_t *rate)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                             /* check handle */
    {
        return 2;                                                                                   /* return error */
    }
    if (handle->inited != 1)                                                                        /* check handle initialization */
    {
        return 3;                                                                                   /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG);        /* read spo2 config */

    *rate = (max30102_spo2_sample_rate_t)((prev >> 2) & 0x7);                                       /* get sample rate */

    return 0;                                                                                       /* success return 0 */
}

/**
 * @brief     set the adc resolution
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] resolution is the adc resolution
 * @return    status code
 *            - 0 success
 *            - 1 set adc resolution failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_adc_resolution(max30102_handle_t *handle, max30102_adc_resolution_t resolution)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG);         /* read spo2 config */

    prev &= ~(3 << 0);                                                                               /* clear config */
    prev |= resolution << 0;                                                                         /* set adc resolution */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG, prev);        /* write spo2 config */
    if (res != 0)                                                                                    /* check result */
    {
        app_log_debug("max30102: write spo2 config failed.\n");                                /* write spo2 config failed */

        return 1;                                                                                    /* return error */
    }

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get the adc resolution
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *resolution points to an adc resolution buffer
 * @return     status code
 *             - 0 success
 *             - 1 get adc resolution failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_adc_resolution(max30102_handle_t *handle, max30102_adc_resolution_t *resolution)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_SPO2_CONFIG);         /* read spo2 config */

    *resolution = (max30102_adc_resolution_t)((prev >> 0) & 0x3);                                    /* set adc resolution */

    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     set the red led pulse amplitude
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] amp is the red led pulse amplitude
 * @return    status code
 *            - 0 success
 *            - 1 set led red pulse amplitude failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_led_red_pulse_amplitude(max30102_handle_t *handle, uint8_t amp)
{
    uint8_t res;

    if (handle == NULL)                                                                             /* check handle */
    {
        return 2;                                                                                   /* return error */
    }
    if (handle->inited != 1)                                                                        /* check handle initialization */
    {
        return 3;                                                                                   /* return error */
    }

    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_LED_PULSE_1, amp);        /* write led pulse 1 */
    if (res != 0)                                                                                   /* check result */
    {
        app_log_debug("max30102: write led pulse 1 failed.\n");                               /* write led pulse 1 failed */

        return 1;                                                                                   /* return error */
    }

    return 0;                                                                                       /* success return 0 */
}

/**
 * @brief      get the red led pulse amplitude
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *amp points to a red led pulse amplitude buffer
 * @return     status code
 *             - 0 success
 *             - 1 get led red pulse amplitude failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_led_red_pulse_amplitude(max30102_handle_t *handle, uint8_t *amp)
{
    uint8_t res;

    if (handle == NULL)                                                                           /* check handle */
    {
        return 2;                                                                                 /* return error */
    }
    if (handle->inited != 1)                                                                      /* check handle initialization */
    {
        return 3;                                                                                 /* return error */
    }

    *amp = readRegister8(MAX30102_ADDRESS, MAX30102_REG_LED_PULSE_1);        /* read led pulse 1 */


    return 0;                                                                                     /* success return 0 */
}

/**
 * @brief     set the ir led pulse amplitude
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] amp is the ir led pulse amplitude
 * @return    status code
 *            - 0 success
 *            - 1 set led ir pulse amplitude failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_led_ir_pulse_amplitude(max30102_handle_t *handle, uint8_t amp)
{
    uint8_t res;

    if (handle == NULL)                                                                             /* check handle */
    {
        return 2;                                                                                   /* return error */
    }
    if (handle->inited != 1)                                                                        /* check handle initialization */
    {
        return 3;                                                                                   /* return error */
    }

    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_LED_PULSE_2, amp);        /* write led pulse 2 */
    if (res != 0)                                                                                   /* check result */
    {
        app_log_debug("max30102: write led pulse 2 failed.\n");                               /* write led pulse 2 failed */

        return 1;                                                                                   /* return error */
    }

    return 0;                                                                                       /* success return 0 */
}

/**
 * @brief      get the ir led pulse amplitude
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *amp points to an ir led pulse amplitude buffer
 * @return     status code
 *             - 0 success
 *             - 1 get led ir pulse amplitude failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_led_ir_pulse_amplitude(max30102_handle_t *handle, uint8_t *amp)
{
    uint8_t res;

    if (handle == NULL)                                                                           /* check handle */
    {
        return 2;                                                                                 /* return error */
    }
    if (handle->inited != 1)                                                                      /* check handle initialization */
    {
        return 3;                                                                                 /* return error */
    }

    *amp = readRegister8(MAX30102_ADDRESS, MAX30102_REG_LED_PULSE_2);        /* read led pulse 2 */
    if (res != 0)                                                                                 /* check result */
    {
        app_log_debug("max30102: read led pulse 2 failed.\n");                              /* read led pulse 2 failed */

        return 1;                                                                                 /* return error */
    }

    return 0;                                                                                     /* success return 0 */
}

/**
 * @brief     set the led slot
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] slot is the slot number
 * @param[in] led is the led mode
 * @return    status code
 *            - 0 success
 *            - 1 set slot failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_slot(max30102_handle_t *handle, max30102_slot_t slot, max30102_led_t led)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                              /* check handle */
    {
        return 2;                                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                                         /* check handle initialization */
    {
        return 3;                                                                                                    /* return error */
    }

    if (slot == MAX30102_SLOT_1)                                                                                     /* slot 1 */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_1);        /* read led slot */

        prev &= ~(0x7 << 0);                                                                                         /* clear config */
        prev |= led << 0;                                                                                            /* set led */
        res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_1, prev);       /* write led slot */
        if (res != 0)                                                                                                /* check result */
        {
            app_log_debug("max30102: write led slot failed.\n");                                               /* write led slot failed */

            return 1;                                                                                                /* return error */
        }

        return 0;                                                                                                    /* success return 0 */
    }
    else if (slot == MAX30102_SLOT_2)                                                                                /* slot 2 */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_1);        /* read led slot */

        prev &= ~(0x7 << 4);                                                                                         /* clear config */
        prev |= led << 4;                                                                                            /* set led */
        res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_1, prev);       /* write led slot */
        if (res != 0)                                                                                                /* check result */
        {
            app_log_debug("max30102: write led slot failed.\n");                                               /* write led slot failed */

            return 1;                                                                                                /* return error */
        }

        return 0;                                                                                                    /* success return 0 */
    }
    else if (slot == MAX30102_SLOT_3)                                                                                /* slot 3 */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_2);        /* read led slot */

        prev &= ~(0x7 << 0);                                                                                         /* clear config */
        prev |= led << 0;                                                                                            /* set led */
        res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_2, prev);       /* write led slot */
        if (res != 0)                                                                                                /* check result */
        {
            app_log_debug("max30102: write led slot failed.\n");                                               /* write led slot failed */

            return 1;                                                                                                /* return error */
        }

        return 0;                                                                                                    /* success return 0 */
    }
    else if (slot == MAX30102_SLOT_4)                                                                                /* slot 4 */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_2);        /* read led slot */

        prev &= ~(0x7 << 4);                                                                                         /* clear config */
        prev |= led << 4;                                                                                            /* set led */
        res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_2, prev);       /* write led slot */
        if (res != 0)                                                                                                /* check result */
        {
            app_log_debug("max30102: write led slot failed.\n");                                               /* write led slot failed */

            return 1;                                                                                                /* return error */
        }

        return 0;                                                                                                    /* success return 0 */
    }
    else                                                                                                             /* unknown */
    {
        app_log_debug("max30102: slot is invalid.\n");                                                         /* slot is invalid */

        return 1;                                                                                                    /* return error */
    }
}

/**
 * @brief      get the led slot
 * @param[in]  *handle points to a max30102 handle structure
 * @param[in]  slot is the slot number
 * @param[out] *led points to a led mode buffer
 * @return     status code
 *             - 0 success
 *             - 1 get slot failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_slot(max30102_handle_t *handle, max30102_slot_t slot, max30102_led_t *led)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                              /* check handle */
    {
        return 2;                                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                                         /* check handle initialization */
    {
        return 3;                                                                                                    /* return error */
    }

    if (slot == MAX30102_SLOT_1)                                                                                     /* slot 1 */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_1);        /* read led slot */

        *led = (max30102_led_t)((prev >> 0) & 0x7);                                                                  /* get led */

        return 0;                                                                                                    /* success return 0 */
    }
    else if (slot == MAX30102_SLOT_2)                                                                                /* slot 2 */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_1);        /* read led slot */

        *led = (max30102_led_t)((prev >> 4) & 0x7);                                                                  /* get led */

        return 0;                                                                                                    /* success return 0 */
    }
    else if (slot == MAX30102_SLOT_3)                                                                                /* slot 3 */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_2);        /* read led slot */

        *led = (max30102_led_t)((prev >> 0) & 0x7);                                                                  /* get led */

        return 0;                                                                                                    /* success return 0 */
    }
    else if (slot == MAX30102_SLOT_4)                                                                                /* slot 4 */
    {
        prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_MULTI_LED_MODE_CONTROL_2);        /* read led slot */

        *led = (max30102_led_t)((prev >> 4) & 0x7);                                                                  /* get led */

        return 0;                                                                                                    /* success return 0 */
    }
    else                                                                                                             /* unknown */
    {
        app_log_debug("max30102: slot is invalid.\n");                                                         /* slot is invalid */

        return 1;                                                                                                    /* return error */
    }
}

/**
 * @brief     enable or disable die temperature
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set die temperature failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_die_temperature(max30102_handle_t *handle, max30102_bool_t enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                  /* check handle */
    {
        return 2;                                                                                        /* return error */
    }
    if (handle->inited != 1)                                                                             /* check handle initialization */
    {
        return 3;                                                                                        /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_DIE_TEMP_CONFIG);         /* read die temp config */

    prev &= ~(1 << 0);                                                                                   /* clear config */
    prev |= (enable << 0);                                                                               /* set bool */
    res = writeRegister8_value(MAX30102_ADDRESS, MAX30102_REG_DIE_TEMP_CONFIG, prev);        /* write die temp config */
    if (res != 0)                                                                                        /* check result */
    {
        app_log_debug("max30102: write die temp config failed.\n");                                /* write die temp config failed */

        return 1;                                                                                        /* return error */
    }

    return 0;                                                                                            /* success return 0 */
}

/**
 * @brief      get the die temperature status
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get die temperature failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_die_temperature(max30102_handle_t *handle, max30102_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;

    if (handle == NULL)                                                                                  /* check handle */
    {
        return 2;                                                                                        /* return error */
    }
    if (handle->inited != 1)                                                                             /* check handle initialization */
    {
        return 3;                                                                                        /* return error */
    }

    prev = readRegister8(MAX30102_ADDRESS, MAX30102_REG_DIE_TEMP_CONFIG);         /* read die temp config */

    *enable = (max30102_bool_t)((prev >> 0) & 0x1);                                                      /* get bool */

    return 0;                                                                                            /* success return 0 */
}

/**
 * @brief      get the chip id
 * @param[in]  *handle points to a max30102 handle structure
 * @param[out] *revision_id points to a revision id buffer
 * @param[out] *part_id points to a part id buffer
 * @return     status code
 *             - 0 success
 *             - 1 get id failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_id(max30102_handle_t *handle, uint8_t *revision_id, uint8_t *part_id)
{
    uint8_t res;

    if (handle == NULL)                                                                                   /* check handle */
    {
        return 2;                                                                                         /* return error */
    }
    if (handle->inited != 1)                                                                              /* check handle initialization */
    {
        return 3;                                                                                         /* return error */
    }

    *revision_id = readRegister8(MAX30102_ADDRESS, MAX30102_REG_REVISION_ID);        /* read revision id */

    *part_id = readRegister8(MAX30102_ADDRESS, MAX30102_REG_PART_ID);                /* read part id */


    return 0;                                                                                             /* success return 0 */
}

/**
 * @brief     set the chip register
 * @param[in] *handle points to a max30102 handle structure
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the data buffer length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t max30102_set_reg(max30102_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (handle == NULL)                                            /* check handle */
    {
        return 2;                                                  /* return error */
    }
    if (handle->inited != 1)                                       /* check handle initialization */
    {
        return 3;                                                  /* return error */
    }

    if (writeRegister8_value(MAX30102_ADDRESS, reg, buf) != 0)   /* write data */
    {
        return 1;                                                  /* return error */
    }
    else
    {
        return 0;                                                  /* success return 0 */
    }
}

/**
 * @brief      get the chip register
 * @param[in]  *handle points to a max30102 handle structure
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the data buffer length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t max30102_get_reg(max30102_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (handle == NULL)                                           /* check handle */
    {
        return 2;                                                 /* return error */
    }
    if (handle->inited != 1)                                      /* check handle initialization */
    {
        return 3;                                                 /* return error */
    }
    *buf = readRegister8(MAX30102_ADDRESS, reg);
    if (readRegister8(MAX30102_ADDRESS, reg) != 0)   /* read data */
    {
        return 1;                                                 /* return error */
    }
    else
    {
        return 0;                                                 /* success return 0 */
    }
}


/**
 * @brief      get chip's information
 * @param[out] *info points to a max30102 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t max30102_info(max30102_info_t *info)
{
    if (info == NULL)                                               /* check handle */
    {
        return 2;                                                   /* return error */
    }

    memset(info, 0, sizeof(max30102_info_t));                       /* initialize max30102 info structure */
    strncpy(info->chip_name, CHIP_NAME, 32);                        /* copy chip name */
    strncpy(info->manufacturer_name, MANUFACTURER_NAME, 32);        /* copy manufacturer name */
    strncpy(info->interface, "IIC", 8);                             /* copy interface name */
    info->supply_voltage_min_v = SUPPLY_VOLTAGE_MIN;                /* set minimal supply voltage */
    info->supply_voltage_max_v = SUPPLY_VOLTAGE_MAX;                /* set maximum supply voltage */
    info->max_current_ma = MAX_CURRENT;                             /* set maximum current */
    info->temperature_max = TEMPERATURE_MAX;                        /* set minimal temperature */
    info->temperature_min = TEMPERATURE_MIN;                        /* set maximum temperature */
    info->driver_version = DRIVER_VERSION;                          /* set driver version */

    return 0;                                                       /* success return 0 */
}



