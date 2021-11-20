/*
 * Copyright (c) 2021 Kimball Thurston
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/spi.h>

#define POWER_BLINKY_BLINKY PICO_DEFAULT_LED_PIN

#define MOUSE_SPI_SCK_PIN PICO_DEFAULT_SPI_SCK_PIN
#define MOUSE_SPI_SEND_PIN PICO_DEFAULT_SPI_TX_PIN
#define MOUSE_SPI_RECV_PIN PICO_DEFAULT_SPI_RX_PIN
//#define MOUSE_SPI_MOSI_PIN PICO_DEFAULT_SPI_TX_PIN
//#define MOUSE_SPI_MISO_PIN PICO_DEFAULT_SPI_RX_PIN
#define MOUSE_SPI_SELECT_PIN PICO_DEFAULT_SPI_CSN_PIN

// clang-format off
#define PMW3320_PRODUCT_ID       0x00
#define PMW3320_REVISION_ID      0x01
#define PMW3320_MOTION           0x02
#define PMW3320_DELTA_X          0x03
#define PMW3320_DELTA_Y          0x04
#define PMW3320_SQUAL            0x05
#define PMW3320_SHUT_HI          0x06
#define PMW3320_SHUT_LO          0x07
#define PMW3320_PIX_MAX          0x08
#define PMW3320_PIX_ACCUM        0x09
#define PMW3320_PIX_MIN          0x0a
#define PMW3320_PIX_GRAB         0x0b
#define PMW3320_DELTA_XY         0x0c
#define PMW3320_RESOLUTION       0x0d
#define PMW3320_RUN_DOWNSHIFT    0x0e
#define PMW3320_REST1_PERIOD     0x0f
#define PMW3320_REST1_DOWNSHIFT  0x10
#define PMW3320_REST2_PERIOD     0x11
#define PMW3320_REST2_DOWNSHIFT  0x12
#define PMW3320_REST3_PERIOD     0x13
#define PMW3320_MIN_SQUAL_RUN    0x17
#define PMW3320_AXIS_CONTROL     0x1a
#define PMW3320_PERFORMANCE      0x22
#define PMW3320_LOW_MOT_JIT      0x23
#define PMW3320_SHUT_MAX_HI      0x36
#define PMW3320_SHUT_MAX_LO      0x37
#define PMW3320_FRAME_RATE       0x39
#define PMW3320_POWER_UP_RESET   0x3a
#define PMW3320_SHUTDOWN         0x3b
#define PMW3320_NOT_REV_ID       0x3f
#define PMW3320_LED_CONTROL      0x40
#define PMW3320_MOTION_CTRL      0x41
#define PMW3320_BURST_READ_FIRST 0x42
#define PMW3320_REST_MODE_STATUS 0x45
#define PMW3320_NOT_PROD_ID      0x4f
#define PMW3320_BURST_MOTION     0x63
// clang-format on

static inline void cs_select()
{
    asm volatile( "nop \n nop \n nop" );
    gpio_put( MOUSE_SPI_SELECT_PIN, 0 ); // Active low
    asm volatile( "nop \n nop \n nop" );
}

static inline void cs_deselect()
{
    asm volatile( "nop \n nop \n nop" );
    gpio_put( MOUSE_SPI_SELECT_PIN, 1 );
    asm volatile( "nop \n nop \n nop" );
}

static inline void send_byte( uint8_t val )
{
    spi_inst_t *spi = spi_default;

    cs_select();

    while ( !spi_is_writable( spi ) )
        tight_loop_contents();
    spi_get_hw( spi )->dr = (uint32_t)val;

    while ( spi_is_readable( spi ) )
        (void)spi_get_hw( spi )->dr;
    while ( spi_get_hw( spi )->sr & SPI_SSPSR_BSY_BITS )
        tight_loop_contents();
    while ( spi_is_readable( spi ) )
        (void)spi_get_hw( spi )->dr;

    spi_get_hw( spi )->icr = SPI_SSPICR_RORIC_BITS;
    cs_deselect();
}

static inline void send_2_byte( uint8_t val1, uint8_t val2 )
{
    spi_inst_t *spi = spi_default;

    cs_select();

    while ( !spi_is_writable( spi ) )
        tight_loop_contents();
    spi_get_hw( spi )->dr = (uint32_t)val1;
    while ( !spi_is_writable( spi ) )
        tight_loop_contents();
    spi_get_hw( spi )->dr = (uint32_t)val2;

    while ( spi_is_readable( spi ) )
        (void)spi_get_hw( spi )->dr;
    while ( spi_get_hw( spi )->sr & SPI_SSPSR_BSY_BITS )
        tight_loop_contents();
    while ( spi_is_readable( spi ) )
        (void)spi_get_hw( spi )->dr;

    spi_get_hw( spi )->icr = SPI_SSPICR_RORIC_BITS;
    cs_deselect();
}

static inline uint8_t drain_fifo( void )
{
    spi_inst_t *spi = spi_default;
    uint8_t last = 0;
    while ( spi_is_readable( spi ) )
        last = spi_get_hw( spi )->dr;
    while ( spi_get_hw( spi )->sr & SPI_SSPSR_BSY_BITS )
        tight_loop_contents();
    while ( spi_is_readable( spi ) )
        last = spi_get_hw( spi )->dr;
    spi_get_hw( spi )->icr = SPI_SSPICR_RORIC_BITS;
    return last;
}

static void write_register( uint8_t reg, uint8_t data )
{
    spi_inst_t *spi = spi_default;
    uint8_t last;
    cs_select();

    while ( !spi_is_writable( spi ) )
        tight_loop_contents();
    spi_get_hw( spi )->dr = (uint32_t)(reg | 0x80);
    while ( !spi_is_writable( spi ) )
        tight_loop_contents();
    spi_get_hw( spi )->dr = (uint32_t)data;

    sleep_us( 120 );

    last = drain_fifo();
    cs_deselect();

    printf( "write_register( 0x%02x, 0x%02x ) -> 0x%02x\n", reg, data, last );
}

static uint8_t read_register( uint8_t reg )
{
    spi_inst_t *spi = spi_default;
    uint8_t prior, sent, retval;

    cs_select();
    prior = drain_fifo();

    while ( !spi_is_writable( spi ) )
        tight_loop_contents();
    spi_get_hw( spi )->dr = (uint32_t)reg;

    while ( spi_get_hw( spi )->sr & SPI_SSPSR_BSY_BITS )
        tight_loop_contents();
    if ( spi_is_readable( spi ) )
        sent = spi_get_hw( spi )->dr;

    sleep_us( 100 );

    if ( spi_is_writable( spi ) )
        spi_get_hw( spi )->dr = (uint32_t)0;
    if ( spi_is_readable( spi ) )
        retval = spi_get_hw( spi )->dr;

    cs_deselect();

    //sleep_us( 120 );
    printf( "read_reg req reg 0x%02x -> %02x  => 0x%02x\n", reg, sent, retval );
    return retval;
#if 0
    const uint8_t buf[2] = { reg, 0 };
    uint8_t outbuf[2];

    //spi_write_blocking( spi_default, buf, 2 );
    spi_write_read_blocking( spi_default, buf, outbuf, 2 );
    cs_deselect();

    return outbuf[1];
#endif
    //uint8_t readout = 0;
    //
    //spi_write_blocking( spi_default, &reg, 1 );
    //sleep_us( 120 );
    //
    //spi_read_blocking( spi_default, 0, &readout, 1 );
    //return readout;
}

static void setup( void )
{
    uint8_t prod, rev, ledc, cfg;
    write_register( PMW3320_SHUTDOWN, 0xb6 );
    sleep_ms( 300 );

    // bounce the select lines to refresh
    cs_select();
    sleep_us( 40 );
    cs_deselect();
    sleep_us( 40 );
    //cs_select();

    write_register( PMW3320_POWER_UP_RESET, 0x5a );
    sleep_ms( 100 );

    prod = read_register( PMW3320_PRODUCT_ID );
    rev  = read_register( PMW3320_REVISION_ID );

    printf( "Chip ID is 0x%02x, rev 0x%02x\n", prod, rev );
    ledc = read_register( PMW3320_LED_CONTROL );

    cfg = ledc | 1;
    printf( "LED config 0x%02x -> 0x%02x\n", ledc, cfg );

    write_register( PMW3320_LED_CONTROL, cfg );
    ledc = read_register( PMW3320_LED_CONTROL );
    printf( " ==> 0x%02x\n", ledc );
}

//static void read_registers( uint8_t reg, uint8_t *buf, uint16_t len )
//{
//    // For this particular device, we send the device the register we want to read
//    // first, then subsequently read from the device. The register is auto incrementing
//    // so we don't need to keep sending the register we want, just the first.
//    reg |= READ_BIT;
//    cs_select();
//    spi_write_blocking( spi_default, &reg, 1 );
//    sleep_ms( 10 );
//    spi_read_blocking( spi_default, 0, buf, len );
//    cs_deselect();
//    sleep_ms( 10 );
//}

static void read_deltas( void )
{
    uint8_t prod;
    uint8_t mot, rdx, rdy;
    int8_t  sdx, sdy;
    // reading the motion flag freezes deltas until they are read
    mot = read_register( PMW3320_MOTION );
    rdx = read_register( PMW3320_DELTA_X );
    rdy = read_register( PMW3320_DELTA_Y );

    sdx = rdx;
    sdy = rdy;

    prod = ~read_register( PMW3320_NOT_PROD_ID );

    printf(
        "0x%02x motion 0x%02x: 0x%02x (%d), 0x%02x (%d)\n",
        prod,
        mot,
        rdx,
        (int)sdx,
        rdy,
        (int)sdy );
}

int main()
{
    uint32_t counter = 0;

    stdio_init_all();

    gpio_init( POWER_BLINKY_BLINKY );
    gpio_set_dir( POWER_BLINKY_BLINKY, GPIO_OUT );

    // 1MHz
    spi_init( spi_default, 1000 * 1000 );
    spi_set_format( spi_default, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST );

    gpio_set_function( MOUSE_SPI_SEND_PIN, GPIO_FUNC_SPI );
    gpio_set_function( MOUSE_SPI_SCK_PIN, GPIO_FUNC_SPI );
    gpio_set_function( MOUSE_SPI_RECV_PIN, GPIO_FUNC_SPI );

    //gpio_set_drive_strength( MOUSE_SPI_SEND_PIN, GPIO_DRIVE_STRENGTH_8MA );

    // Make the SPI pins available to picotool
    bi_decl( bi_3pins_with_func(
        MOUSE_SPI_RECV_PIN,
        MOUSE_SPI_SEND_PIN,
        MOUSE_SPI_SCK_PIN,
        GPIO_FUNC_SPI ) );

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init( MOUSE_SPI_SELECT_PIN );
    gpio_set_dir( MOUSE_SPI_SELECT_PIN, GPIO_OUT );
    gpio_put( MOUSE_SPI_SELECT_PIN, 1 );

    // Make the CS pin available to picotool
    bi_decl( bi_1pin_with_name( MOUSE_SPI_SELECT_PIN, "SPI CS" ) );

    sleep_ms( 2000 );
    setup();

    while ( true )
    {
        if ( counter % 4 == 0 )
            read_deltas();
        gpio_put( POWER_BLINKY_BLINKY, counter % 2 );
        sleep_ms( 250 );
        ++counter;
    }
    return 0;
}
