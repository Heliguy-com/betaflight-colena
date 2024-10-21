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
#include <string.h>

#include "flash.h"
#include "flash_impl.h"
#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_MB85RS

#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/flash.h"

#include "drivers/flash/flash_mb85rs.h"


// This is decidedly not flash, but it is persistent memory over SPI that should support the same functions as flash within betafllight
// Implemented as flash for ease 

// TODO:
// [ ] Remove unnecessary include statements added by vscode
// [ ] Remove unused macros
// [-] Implement eraseSector and eraseCompletely
// [ ] Implement pageProgram (Begin, Continue and Finish)
// [ ] Review callbacks
// [ ] Review read status: Make sure command is correct and it is being interpretted correctly
// [ ] Verify that write will work when writing over >2 segments as this will be common with this implementation.


#define MB85RS_INSTRUCTION_RDID             SPIFLASH_INSTRUCTION_RDID
#define MB85RS_INSTRUCTION_READ_BYTES       0x03
#define MB85RS_INSTRUCTION_READ_STATUS_REG  0x05
#define MB85RS_INSTRUCTION_WRITE_STATUS_REG 0x01
#define MB85RS_INSTRUCTION_WRITE_ENABLE     0x06
#define MB85RS_INSTRUCTION_WRITE_DISABLE    0x04
#define MB85RS_INSTRUCTION_PAGE_PROGRAM     0x02
#define MB85RS_INSTRUCTION_SECTOR_ERASE     0xD8
#define MB85RS_INSTRUCTION_BULK_ERASE       0xC7

// --- Taken from 
// Status register
#define MB75RS_STATUS_WPEN       0x80
#define MB75RS_STATUS_BP1        0x08
#define MB75RS_STATUS_BP0        0x04
#define MB75RS_STATUS_WEL        0x02

// OP-CODE
#define MB85RS_INSTRUCTION_WREN       0x06
#define MB85RS_INSTRUCTION_WRDI       0x04
#define MB85RS_INSTRUCTION_RDSR       0x05
#define MB85RS_INSTRUCTION_WRSR       0x01
#define MB85RS_INSTRUCTION_READ       0x03
#define MB85RS_INSTRUCTION_WRITE      0x02
#define MB85RS_INSTRUCTION_SLEEP      0xB9
// ---

#define MB85RS_STATUS_FLAG_WRITE_IN_PROGRESS 0x01
#define MB85RS_STATUS_FLAG_WRITE_ENABLED     0x02

#define MB85RS_FAST_READ_DUMMY_CYCLES       8

// SPI transaction segment indicies for mb85rs_pageProgramContinue()
enum {READ_STATUS, WRITE_ENABLE, PAGE_PROGRAM, DATA1, DATA2};

const flashVTable_t mb85rs_vTable;
static uint32_t maxClkSPIHz;
static uint32_t maxReadClkSPIHz;

static uint8_t mb85rs_readStatus(flashDevice_t *fdevice)
{
    uint8_t status = 0;
    if (fdevice->io.mode == FLASHIO_SPI) {
        STATIC_DMA_DATA_AUTO uint8_t readStatus[1] = { MB85RS_INSTRUCTION_RDSR };
        STATIC_DMA_DATA_AUTO uint8_t readyStatus[1];

        spiReadWriteBuf(fdevice->io.handle.dev, readStatus, readyStatus, sizeof(readStatus));

        status = readyStatus[0];
    }

    return status;
}

static bool mb85rs_isReady(flashDevice_t *fdevice)
{
    // If we're waiting on DMA completion, then SPI is busy
    if (fdevice->io.mode == FLASHIO_SPI) {
        if (fdevice->io.handle.dev->bus->useDMA && spiIsBusy(fdevice->io.handle.dev)) {
            return false;
        }
    }

    return true;
}

static bool mb85rs_waitForReady(flashDevice_t *fdevice)
{
    while (!mb85rs_isReady(fdevice));

    return true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no mb85rs.
 */
bool mb85rs_identify(flashDevice_t *fdevice, uint32_t jedecID)
{
    // TODO: Check the jedecID of the SPI chip
    flashGeometry_t *geometry = &fdevice->geometry;
    uint8_t index;

    maxClkSPIHz = (uint32_t)3e6;
    maxReadClkSPIHz = (uint32_t)3e6;
    geometry->flashType = FLASH_TYPE_FRAM;
    
    // Changed flashSector_t to uint32_t from uint16_t
    geometry->sectors = 1 << 20;
    geometry->pagesPerSector = 1;
    geometry->pageSize = 1;
    geometry->sectorSize = 1;
    
    // 1M storage.
    geometry->totalSize = geometry->sectors * geometry->sectorSize;
    fdevice->couldBeBusy = false;

    // Not sure what this command is for
    if (fdevice->io.mode == FLASHIO_SPI) {
        fdevice->vTable = &mb85rs_vTable;
    }

    return true;
}

void mb85rs_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    if (configurationFlags & FLASH_CF_SYSTEM_IS_MEMORY_MAPPED) {
        return;
    }

    if (fdevice->io.mode == FLASHIO_SPI) {
        // Adjust the SPI bus clock frequency
        spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(maxReadClkSPIHz));
    }
}


static void mb85rs_setCommandAddress(uint8_t *buf, uint32_t address, bool useLongAddress)
{
    *buf++ = (address >> 16) & 0xff;
    *buf++ = (address >> 8) & 0xff;
    *buf = address & 0xff;
}


// Called in ISR context
// A write enable has just been issued
busStatus_e mb85rs_callbackWriteEnable(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    return BUS_READY;
}

// Called in ISR context
// Write operation has just completed
busStatus_e mb85rs_callbackWriteComplete(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    fdevice->currentWriteAddress += fdevice->callbackArg;

    // Call transfer completion callback
    if (fdevice->callback) {
        fdevice->callback(fdevice->callbackArg);
    }

    return BUS_READY;
}

// Called in ISR context
// Check if the status was busy and if so repeat the poll
busStatus_e mb85rs_callbackReady(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;
    extDevice_t *dev = fdevice->io.handle.dev;

    return BUS_READY;
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
static void mb85rs_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { MB85RS_INSTRUCTION_RDSR, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { MB85RS_INSTRUCTION_WREN };
    STATIC_DMA_DATA_AUTO uint8_t sectorErase[5] = { MB85RS_INSTRUCTION_WRITE };

    busSegment_t segments[] = {
            {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, mb85rs_callbackReady},
            {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, mb85rs_callbackWriteEnable},
            {.u.buffers = {sectorErase, NULL}, 5, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    mb85rs_setCommandAddress(&sectorErase[1], address, fdevice->isLargeFlash);

    spiSequence(fdevice->io.handle.dev, segments);

    // Block pending completion of SPI access, but the erase will be ongoing
    spiWait(fdevice->io.handle.dev);
}

// FIXME: Only utilised inside of fs code. Changed to not use.
static void mb85rs_eraseCompletely(flashDevice_t *fdevice)
{

    // // Disable CS up on bulkerase 'completion'
    // // Instead, write fdevice->total_size many empty bytes.
    // // This will take ~0.3 seconds which is quite a while for a useless call
    // // Oh well if a command says 'eraseCompletely' it should erase completely.
    // // Might need to also implement some of the callbacks found in programpage.

    // STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { MB85RS_INSTRUCTION_RDSR, 0 };
    // STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    // STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { MB85RS_INSTRUCTION_WREN };
    // STATIC_DMA_DATA_AUTO bulkEraseData = (uint8_t*)malloc(4 + 1 << 20 * sizeof(uint8_t))

    // bulkEraseData[0] = MB85RS_INSTRUCTION_WRITE

    // busSegment_t segments[] = {
    //         {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, mb85rs_callbackReady},
    //         {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, mb85rs_callbackWriteEnable},
    //         {.u.buffers = {bulkEraseData, NULL}, sizeof(bulkEraseData), true, NULL},
    //         {.u.link = {NULL, NULL}, 0, true, NULL},
    // };

    // spiSequence(fdevice->io.handle.dev, segments);

    // // Block pending completion of SPI access, but the erase will be ongoing
    // spiWait(fdevice->io.handle.dev);
}

static void mb85rs_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
}

// TODO: This is written with the assumption that data will only ever span two sectors.
// With single byte sectors most writes to flash will span more than two sectors.
// What needs to be rewritten to fix this?

// Writes can occur simultaneously over sectors, infinitely with automatic address incrementation.
// > Leave the code as it is but assume that a single buffer will be passed to the function.

static uint32_t mb85rs_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    // The segment list cannot be in automatic storage as this routine is non-blocking
    STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { MB85RS_INSTRUCTION_RDSR, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { MB85RS_INSTRUCTION_WREN };
    STATIC_DMA_DATA_AUTO uint8_t pageProgram[5] = { MB85RS_INSTRUCTION_WRITE };

    static busSegment_t segments[] = {
            {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, mb85rs_callbackReady},
            {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, mb85rs_callbackWriteEnable},
            {.u.buffers = {pageProgram, NULL}, sizeof(pageProgram), false, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    // Patch the pageProgram segment
    segments[PAGE_PROGRAM].len = fdevice->isLargeFlash ? 5 : 4;
    mb85rs_setCommandAddress(&pageProgram[1], fdevice->currentWriteAddress, fdevice->isLargeFlash);

    // Patch the data segments
    segments[DATA1].u.buffers.txData = (uint8_t *)buffers[0];
    segments[DATA1].len = bufferSizes[0];
    fdevice->callbackArg = bufferSizes[0];

    /* As the DATA2 segment may be used as the terminating segment, the rxData and txData may be overwritten
     * with a link to the following transaction (u.link.dev and u.link.segments respectively) so ensure that
     * rxData is reinitialised otherwise it will remain pointing at a chained u.link.segments structure which
     * would result in it being corrupted.
     */
    segments[DATA2].u.buffers.rxData = (uint8_t *)NULL;

    if (bufferCount == 1) {
        segments[DATA1].negateCS = true;
        segments[DATA1].callback = mb85rs_callbackWriteComplete;
        // Mark segment following data as being of zero length
        segments[DATA2].u.buffers.txData = (uint8_t *)NULL;
        segments[DATA2].len = 0;
    } else if (bufferCount == 2) {
        segments[DATA1].negateCS = false;
        segments[DATA1].callback = NULL;
        segments[DATA2].u.buffers.txData = (uint8_t *)buffers[1];
        segments[DATA2].len = bufferSizes[1];
        fdevice->callbackArg += bufferSizes[1];
        segments[DATA2].negateCS = true;
        segments[DATA2].callback = mb85rs_callbackWriteComplete;
    } else {
        return 0;
    }

    spiSequence(fdevice->io.handle.dev, fdevice->couldBeBusy ? &segments[READ_STATUS] : &segments[WRITE_ENABLE]);

    if (fdevice->callback == NULL) {
        // No callback was provided so block
        spiWait(fdevice->io.handle.dev);
    }

    return fdevice->callbackArg;
}

static void mb85rs_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

/**
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */
static void mb85rs_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    mb85rs_pageProgramBegin(fdevice, address, callback);

    mb85rs_pageProgramContinue(fdevice, &data, &length, 1);

    mb85rs_pageProgramFinish(fdevice);
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
static int mb85rs_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { MB85RS_INSTRUCTION_RDSR, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    STATIC_DMA_DATA_AUTO uint8_t readBytes[4] = { MB85RS_INSTRUCTION_READ };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    // TODO: Why is buffer not passed as the RX buffer for readbytes?
    // Because of the difference in length between writing and reading?
    busSegment_t segments[] = {
            {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, mb85rs_callbackReady},
            {.u.buffers = {readBytes, NULL}, sizeof(readBytes), false, NULL},
            {.u.buffers = {NULL, buffer}, length, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    // Patch the readBytes command
    mb85rs_setCommandAddress(&readBytes[1], address, fdevice->isLargeFlash);

    spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(maxReadClkSPIHz));

    // If the device could be busy, pass the segment for reading directlly instead of checking ID first.
    spiSequence(fdevice->io.handle.dev, fdevice->couldBeBusy ? &segments[0] : &segments[1]);

    // Block until code is re-factored to exploit non-blocking
    spiWait(fdevice->io.handle.dev);

    spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(maxClkSPIHz));

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling mb85rs_init() (the result would have totalSize = 0).
 */
static const flashGeometry_t* mb85rs_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t mb85rs_vTable = {
    .configure = mb85rs_configure,
    .isReady = mb85rs_isReady,
    .waitForReady = mb85rs_waitForReady,
    .eraseSector = mb85rs_eraseSector,
    .eraseCompletely = mb85rs_eraseCompletely,
    .pageProgramBegin = mb85rs_pageProgramBegin,
    .pageProgramContinue = mb85rs_pageProgramContinue,
    .pageProgramFinish = mb85rs_pageProgramFinish,
    .pageProgram = mb85rs_pageProgram,
    .readBytes = mb85rs_readBytes,
    .getGeometry = mb85rs_getGeometry,
};


#endif
