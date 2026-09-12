/*********************************************************************************************************************
* Simplified SD card driver (SPI mode)
* File: code/sd/sd_simple.c
*********************************************************************************************************************/

#include "sd_simple.h"
#include "sd_simple_config.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "stdio.h"

static uint8_t sd_initialized = 0;
static sd_simple_info_struct sd_info;
static sd_simple_type_enum sd_type = SD_TYPE_UNKNOWN;
static uint32_t sd_capacity = 0;

#ifndef SD_CMD1
#define SD_CMD1     1U
#endif
#ifndef SD_CMD9
#define SD_CMD9     9U
#endif
#ifndef SD_CMD10
#define SD_CMD10    10U
#endif
#ifndef SD_CMD16
#define SD_CMD16    16U
#endif

#define SD_BLOCK_SIZE_BYTES        512U
#define SD_TRANSFER_CLOCK_SAFE     12000000U

static void sd_reset_info(void)
{
    sd_initialized = 0U;
    sd_type = SD_TYPE_UNKNOWN;
    sd_capacity = 0U;
    sd_info.capacity_mb = 0U;
    sd_info.block_size = SD_BLOCK_SIZE_BYTES;
    sd_info.block_count = 0U;
    sd_info.type = SD_TYPE_UNKNOWN;
    sd_info.manufacturer_id = 0U;
    sd_info.rca = 0U;
}

static void spi_init_sd(uint32_t baud)
{
    spi_init(SD_SPI_INDEX, SPI_MODE0, baud, SD_SCK_PIN, SD_MOSI_PIN, SD_MISO_PIN, SPI_CS_NULL);
}

static void spi_send_byte(uint8_t data)
{
    uint8_t dummy;

    spi_transfer_8bit(SD_SPI_INDEX, &data, &dummy, 1);
}

static uint8_t spi_receive_byte(void)
{
    uint8_t tx = 0xFFU;
    uint8_t rx = 0xFFU;

    spi_transfer_8bit(SD_SPI_INDEX, &tx, &rx, 1);
    return rx;
}

static void cs_low(void)
{
    gpio_low(SD_CS_PIN);
}

static void cs_high(void)
{
    gpio_high(SD_CS_PIN);
}

static void sd_select(void)
{
    cs_low();
    spi_send_byte(0xFF);
}

static void sd_deselect(void)
{
    cs_high();
    spi_send_byte(0xFF);
}

static uint8_t wait_response(uint32_t retry)
{
    uint8_t response = 0xFF;

    while (retry--)
    {
        response = spi_receive_byte();
        if (response != 0xFF)
        {
            break;
        }
    }

    return response;
}

static uint8_t send_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    spi_send_byte((uint8_t)(cmd | 0x40));
    spi_send_byte((uint8_t)(arg >> 24));
    spi_send_byte((uint8_t)(arg >> 16));
    spi_send_byte((uint8_t)(arg >> 8));
    spi_send_byte((uint8_t)(arg));
    spi_send_byte(crc);

    return wait_response(SD_CMD_RETRY_COUNT * 10U);
}

static uint8_t sd_spi_init(void)
{
    uint8_t response;
    uint8_t r7_0, r7_1, r7_2, r7_3;
    uint32_t i;
    uint32_t retry;

    gpio_init(SD_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    sd_deselect();

    // >=74 clocks with CS high after power-up
    for (i = 0; i < 10U; i++)
    {
        spi_send_byte(0xFF);
    }

    // CMD0 -> IDLE(0x01)
    response = 0xFF;
    for (retry = 0; retry < SD_CMD_RETRY_COUNT; retry++)
    {
        sd_select();
        response = send_command(SD_CMD0, 0, 0x95);
        sd_deselect();

        if (response == SD_R1_IDLE_STATE)
        {
            break;
        }
    }
    if (response != SD_R1_IDLE_STATE)
    {
        printf("[SD] CMD0 failed, R1=0x%02X\n", response);
        return 1;
    }

    // CMD8 -> SDv2 check
    sd_select();
    response = send_command(SD_CMD8, 0x1AA, 0x87);
    if (response == SD_R1_IDLE_STATE)
    {
        r7_0 = spi_receive_byte();
        r7_1 = spi_receive_byte();
        r7_2 = spi_receive_byte();
        r7_3 = spi_receive_byte();
        (void)r7_0;
        (void)r7_1;
        (void)r7_2;

        if (r7_3 != 0xAA)
        {
            sd_deselect();
            printf("[SD] CMD8 R7 check failed, r7_3=0x%02X\n", r7_3);
            return 2;
        }

        sd_type = SD_TYPE_SDV2;
    }
    else if (response == 0x05)
    {
        sd_type = SD_TYPE_SDV1;
    }
    else
    {
        sd_deselect();
        printf("[SD] CMD8 failed, R1=0x%02X\n", response);
        return 2;
    }
    sd_deselect();

    // ACMD41 -> leave idle
    response = 0xFF;
    for (retry = 0; retry < (SD_INIT_RETRY_COUNT * 100U); retry++)
    {
        sd_select();
        response = send_command(SD_CMD55, 0, 0x01);
        sd_deselect();

        if ((response != 0x00U) && (response != SD_R1_IDLE_STATE))
        {
            continue;
        }

        sd_select();
        response = send_command(SD_ACMD41, (sd_type == SD_TYPE_SDV2) ? 0x40000000U : 0U, 0x01);
        sd_deselect();

        if (response == 0x00)
        {
            break;
        }
    }
    if (response != 0x00)
    {
        printf("[SD] ACMD41 failed, R1=0x%02X\n", response);
        return 3;
    }

    // CMD58 -> OCR/CCS
    if (sd_type == SD_TYPE_SDV2)
    {
        uint32_t ocr = 0;

        sd_select();
        response = send_command(SD_CMD58, 0, 0x01);
        if (response != 0x00)
        {
            sd_deselect();
            printf("[SD] CMD58 failed, R1=0x%02X\n", response);
            return 4;
        }

        ocr |= (uint32_t)spi_receive_byte() << 24;
        ocr |= (uint32_t)spi_receive_byte() << 16;
        ocr |= (uint32_t)spi_receive_byte() << 8;
        ocr |= (uint32_t)spi_receive_byte();
        sd_deselect();

        if (ocr & (1UL << 30))
        {
            sd_type = SD_TYPE_SDHC;
        }
    }

    if ((sd_type != SD_TYPE_SDHC) && (sd_type != SD_TYPE_SDXC))
    {
        sd_select();
        response = send_command((uint8_t)SD_CMD16, SD_BLOCK_SIZE_BYTES, 0xFFU);
        sd_deselect();
        if (response != 0x00U)
        {
            printf("[SD] CMD16 failed, R1=0x%02X\n", response);
            return 6;
        }
    }

    // switch to high speed after init
    spi_init_sd(SD_TRANSFER_CLOCK_SAFE);

    return 0;
}

static uint8_t wait_data_token(void)
{
    return wait_response((uint32_t)SD_TIMEOUT_MS * 200U);
}

static uint8_t read_register_block(uint8_t cmd, uint8_t *buffer, uint32_t length)
{
    uint8_t response;
    uint8_t token;
    uint32_t i;

    if ((buffer == 0) || (length == 0U))
    {
        return 9U;
    }

    sd_select();
    response = send_command(cmd, 0U, 0xFFU);
    if (response != 0x00U)
    {
        sd_deselect();
        return 1U;
    }

    token = wait_data_token();
    if (token != SD_DATA_TOKEN_START_BLOCK)
    {
        sd_deselect();
        return 2U;
    }

    for (i = 0U; i < length; i++)
    {
        buffer[i] = spi_receive_byte();
    }

    spi_receive_byte();
    spi_receive_byte();
    sd_deselect();

    return 0U;
}

static uint8_t parse_csd(const uint8_t *csd, sd_simple_info_struct *info)
{
    uint8_t csd_structure;
    uint32_t block_count;
    uint32_t block_size;
    uint32_t capacity_mb;

    if ((csd == 0) || (info == 0))
    {
        return 1U;
    }

    csd_structure = (uint8_t)((csd[0] >> 6) & 0x03U);
    if (csd_structure == 1U)
    {
        uint32_t c_size;

        c_size = ((uint32_t)(csd[7] & 0x3FU) << 16)
               | ((uint32_t)csd[8] << 8)
               | (uint32_t)csd[9];
        block_count = (c_size + 1U) * 1024U;
        block_size = SD_BLOCK_SIZE_BYTES;
        capacity_mb = block_count / 2048U;
    }
    else if (csd_structure == 0U)
    {
        uint32_t read_bl_len;
        uint32_t c_size;
        uint32_t c_size_mult;
        uint32_t mult;

        read_bl_len = (uint32_t)(csd[5] & 0x0FU);
        c_size = ((uint32_t)(csd[6] & 0x03U) << 10)
               | ((uint32_t)csd[7] << 2)
               | ((uint32_t)(csd[8] & 0xC0U) >> 6);
        c_size_mult = ((uint32_t)(csd[9] & 0x03U) << 1)
                    | ((uint32_t)(csd[10] & 0x80U) >> 7);

        if (read_bl_len > 12U)
        {
            return 2U;
        }

        block_size = 1UL << read_bl_len;
        mult = 1UL << (c_size_mult + 2U);
        block_count = (c_size + 1U) * mult;
        capacity_mb = (uint32_t)(((uint64_t)block_count * (uint64_t)block_size) / (1024ULL * 1024ULL));

        if (block_size != SD_BLOCK_SIZE_BYTES)
        {
            block_count = (uint32_t)(((uint64_t)block_count * (uint64_t)block_size) / SD_BLOCK_SIZE_BYTES);
            block_size = SD_BLOCK_SIZE_BYTES;
        }
    }
    else
    {
        return 3U;
    }

    if ((block_count == 0U) || (capacity_mb == 0U))
    {
        return 4U;
    }

    info->block_size = block_size;
    info->block_count = block_count;
    info->capacity_mb = capacity_mb;

    return 0U;
}

static uint8_t sd_load_card_info(void)
{
    uint8_t csd[16];
    uint8_t cid[16];
    uint8_t result;

    sd_info.type = sd_type;
    sd_info.block_size = SD_BLOCK_SIZE_BYTES;
    sd_info.block_count = 0U;
    sd_info.capacity_mb = 0U;
    sd_info.manufacturer_id = 0U;
    sd_info.rca = 0U;

    result = read_register_block((uint8_t)SD_CMD9, csd, sizeof(csd));
    if (result != 0U)
    {
        printf("[SD] CMD9/CSD read failed, code=%u\n", result);
        return 1U;
    }

    result = parse_csd(csd, &sd_info);
    if (result != 0U)
    {
        printf("[SD] CSD parse failed, code=%u\n", result);
        return 2U;
    }

    result = read_register_block((uint8_t)SD_CMD10, cid, sizeof(cid));
    if (result == 0U)
    {
        sd_info.manufacturer_id = cid[0];
    }

    sd_info.type = sd_type;
    sd_capacity = sd_info.capacity_mb;
    return 0U;
}

static uint8_t sd_check_sector_range(uint32_t sector, uint32_t count)
{
    if (count == 0U)
    {
        return 1U;
    }

    if (sd_info.block_count == 0U)
    {
        return 0U;
    }

    if (sector >= sd_info.block_count)
    {
        return 1U;
    }

    if (count > (sd_info.block_count - sector))
    {
        return 1U;
    }

    return 0U;
}

uint8_t sd_simple_init(void)
{
    uint8_t result;

    if (sd_initialized)
    {
        return 0;
    }

    sd_reset_info();

    spi_init_sd(SD_SPI_CLOCK_FREQ);

    result = sd_spi_init();
    if (result != 0)
    {
        sd_reset_info();
        return result;
    }

    result = sd_load_card_info();
    if (result != 0U)
    {
        sd_reset_info();
        return 5U;
    }

    sd_initialized = 1U;

    printf("[SD] Init OK: type=%u capacity=%uMB sectors=%u\n",
           (unsigned)sd_info.type,
           (unsigned)sd_info.capacity_mb,
           (unsigned)sd_info.block_count);

    return 0;
}

uint8_t sd_simple_get_info(sd_simple_info_struct *info)
{
    if (info == 0)
    {
        return 1;
    }

    if (!sd_initialized)
    {
        if (sd_simple_init() != 0)
        {
            return 2;
        }
    }

    *info = sd_info;

    return 0;
}

uint8_t sd_simple_read(uint8_t *buffer, uint32_t sector, uint32_t count)
{
    uint8_t response;
    uint8_t token;
    uint32_t i;
    uint32_t j;
    uint32_t address;
    uint32_t address_step;

    if (buffer == 0 || count == 0)
    {
        return 9;
    }

    if (!sd_initialized)
    {
        if (sd_simple_init() != 0)
        {
            return 1;
        }
    }

    if (sd_check_sector_range(sector, count) != 0U)
    {
        return 8;
    }

    address = sector;
    address_step = 1;
    if (sd_type != SD_TYPE_SDHC && sd_type != SD_TYPE_SDXC)
    {
        address *= 512U;
        address_step = 512U;
    }

    for (i = 0; i < count; i++)
    {
        sd_select();
        response = send_command(SD_CMD17, address + i * address_step, 0xFF);
        if (response != 0x00)
        {
            sd_deselect();
            return 2;
        }

        token = wait_data_token();
        if (token != SD_DATA_TOKEN_START_BLOCK)
        {
            sd_deselect();
            return 3;
        }

        for (j = 0; j < 512U; j++)
        {
            buffer[i * 512U + j] = spi_receive_byte();
        }

        spi_receive_byte();
        spi_receive_byte();

        sd_deselect();
    }

    return 0;
}

uint8_t sd_simple_write(uint8_t *buffer, uint32_t sector, uint32_t count)
{
    uint8_t response;
    uint32_t i;
    uint32_t j;
    uint32_t timeout;
    uint32_t address;
    uint32_t address_step;

    if (buffer == 0 || count == 0)
    {
        return 9;
    }

    if (!sd_initialized)
    {
        if (sd_simple_init() != 0)
        {
            return 1;
        }
    }

    if (sd_check_sector_range(sector, count) != 0U)
    {
        return 8;
    }

    address = sector;
    address_step = 1;
    if (sd_type != SD_TYPE_SDHC && sd_type != SD_TYPE_SDXC)
    {
        address *= 512U;
        address_step = 512U;
    }

    for (i = 0; i < count; i++)
    {
        sd_select();
        response = send_command(SD_CMD24, address + i * address_step, 0xFF);
        if (response != 0x00)
        {
            sd_deselect();
            return 2;
        }

        spi_send_byte(SD_DATA_TOKEN_START_BLOCK);

        for (j = 0; j < 512U; j++)
        {
            spi_send_byte(buffer[i * 512U + j]);
        }

        spi_send_byte(0xFF);
        spi_send_byte(0xFF);

        response = wait_response(SD_CMD_RETRY_COUNT * 20U);
        if ((response & 0x1F) != 0x05)
        {
            sd_deselect();
            return 3;
        }

        timeout = (uint32_t)SD_TIMEOUT_MS * 200U;
        do
        {
            response = spi_receive_byte();
            timeout--;
        } while (response == 0x00 && timeout > 0U);

        if (timeout == 0U)
        {
            sd_deselect();
            return 4;
        }

        sd_deselect();
    }

    return 0;
}

uint8_t sd_simple_is_ready(void)
{
    if (!sd_initialized)
    {
        return (sd_simple_init() == 0);
    }

    return 1;
}

void sd_simple_reset(void)
{
    sd_reset_info();
}

uint32_t sd_simple_get_capacity_mb(void)
{
    if (!sd_initialized)
    {
        if (sd_simple_init() != 0)
        {
            return 0;
        }
    }

    return sd_capacity;
}

uint8_t sd_simple_probe(void)
{
    uint8_t result;
    uint8_t sector0[512];

    printf("\n[SD] Probe start...\n");

    result = sd_simple_init();
    if (result != 0)
    {
        printf("[SD] Init failed, code=%u\n", result);
        return result;
    }

    result = sd_simple_read(sector0, 0, 1);
    if (result != 0)
    {
        printf("[SD] Read sector0 failed, code=%u\n", result);
        return (uint8_t)(10U + result);
    }

    printf("[SD] Probe pass. Sector0[510]=0x%02X Sector0[511]=0x%02X\n", sector0[510], sector0[511]);

    if ((sector0[510] == 0x55) && (sector0[511] == 0xAA))
    {
        printf("[SD] Boot signature looks valid (0x55AA).\n");
    }
    else
    {
        printf("[SD] Warning: no 0x55AA signature at sector0 end.\n");
    }

    return 0;
}

void sd_simple_print_info(void)
{
    if (!sd_initialized)
    {
        if (sd_simple_init() != 0)
        {
            printf("SD Card Information: init failed\n");
            return;
        }
    }

    printf("SD Card Information:\n");
    printf("  Type: ");
    switch (sd_info.type)
    {
        case SD_TYPE_UNKNOWN: printf("Unknown\n"); break;
        case SD_TYPE_MMC:     printf("MMC\n"); break;
        case SD_TYPE_SDV1:    printf("SD V1.x\n"); break;
        case SD_TYPE_SDV2:    printf("SD V2.0\n"); break;
        case SD_TYPE_SDHC:    printf("SDHC\n"); break;
        case SD_TYPE_SDXC:    printf("SDXC\n"); break;
        default:              printf("Invalid\n"); break;
    }
    printf("  Capacity: %u MB\n", sd_info.capacity_mb);
    printf("  Block Size: %u bytes\n", sd_info.block_size);
    printf("  Block Count: %u\n", sd_info.block_count);
    printf("  Manufacturer ID: 0x%02X\n", sd_info.manufacturer_id);
}
