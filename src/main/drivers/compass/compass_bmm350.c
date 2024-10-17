/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

// #if defined(USE_MAG_BMM350)

#include "compass.h"
#include "compass_bmm350.h"


// TODO: Implement the functions using the sensor's API found in libs.
// The accelerometer driver doesn't use the API despite it being found in the repo, why?

static bool bmm350Read(magDev_t * mag, int16_t *magData)
{
    // static uint8_t buf[6];
    // static bool pendingRead = true;

    // extDevice_t *dev = &mag->dev;

    // if (pendingRead) {
    //     if (busReadRegisterBufferStart(dev, LIS3MDL_REG_OUT_X_L, buf, sizeof(buf))) {
    //         pendingRead = false;
    //     }
    //     return false;
    // }

    // magData[X] = (int16_t)(buf[1] << 8 | buf[0]) / 4;
    // magData[Y] = (int16_t)(buf[3] << 8 | buf[2]) / 4;
    // magData[Z] = (int16_t)(buf[5] << 8 | buf[4]) / 4;

    // pendingRead = true;

    return true;
}

static bool bmm350Init(magDev_t *mag)
{
    // extDevice_t *dev = &mag->dev;

    // busDeviceRegister(dev);

    // busWriteRegister(dev, LIS3MDL_REG_CTRL_REG2, LIS3MDL_FS_4GAUSS);
    // busWriteRegister(dev, LIS3MDL_REG_CTRL_REG1, LIS3MDL_TEMP_EN | LIS3MDL_OM_ULTRA_HI_PROF | LIS3MDL_DO_80);
    // busWriteRegister(dev, LIS3MDL_REG_CTRL_REG5, LIS3MDL_BDU);
    // busWriteRegister(dev, LIS3MDL_REG_CTRL_REG4, LIS3MDL_ZOM_UHP);
    // busWriteRegister(dev, LIS3MDL_REG_CTRL_REG3, 0x00);

    // delay(100);
    // mag->magOdrHz = 80; // LIS3MDL_DO_80
    return true;
}

bool bmm350Detect(magDev_t * mag)
{
    // extDevice_t *dev = &mag->dev;

    // uint8_t sig = 0;

    // if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
    //     dev->busType_u.i2c.address = LIS3MDL_MAG_I2C_ADDRESS;
    // }

    // bool ack = busReadRegisterBuffer(&mag->dev, LIS3MDL_REG_WHO_AM_I, &sig, 1);

    // if (!ack || sig != LIS3MDL_DEVICE_ID) {
    //     return false;
    // }

    // mag->init = lis3mdlInit;
    // mag->read = lis3mdlRead;

    return true;
}
// #endif
