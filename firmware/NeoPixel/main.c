/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M0A21 MCU.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NeoPixel.h"
#include "delay.h"

#define LED_PIN     PD6
#define LED_COUNT   60

void UART_Open(UART_T *uart, uint32_t u32baudrate);

void colorWipe(STR_NEOPIXEL_T *pNeoPixel, uint8_t r, uint8_t g, uint8_t b, int wait);
void theaterChase(STR_NEOPIXEL_T *pNeoPixel, uint8_t r, uint8_t g, uint8_t b, int wait);
void rainbow(STR_NEOPIXEL_T *pNeoPixel, int wait);
void theaterChaseRainbow(STR_NEOPIXEL_T *pNeoPixel, int wait);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.6 and TXD=PB.4 */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB4MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk)) |        \
                    (SYS_GPB_MFP1_PB4MFP_UART0_TXD | SYS_GPB_MFP1_PB6MFP_UART0_RXD);

    /* Set PA0, PA1 multi-function pins as GPIO */
    //SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk | SYS_GPA_MFP0_PA1MFP_Msk));

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M0A21 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    /* Create a NeoPixel object */
    STR_NEOPIXEL_T pixels;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\r\n");

    /* Initialize NeoPixel */
    NeoPixel_begin(&pixels, LED_COUNT, &LED_PIN, NEO_GRB);

    /* Turn OFF all pixels ASAP */
    NeoPixel_show(&pixels);

    /* Set BRIGHTNESS to about 1/5 (max = 255) */
    NeoPixel_setBrightness(&pixels, 50);

    /* Got no where to go, just loop forever */
    while(1)
    {
        // Fill along the length of the strip in various colors...
        colorWipe(&pixels, 255,   0,   0, 50); // Red
        colorWipe(&pixels,   0, 255,   0, 50); // Green
        colorWipe(&pixels,   0,   0, 255, 50); // Blue

        // Do a theater marquee effect in various colors...
        theaterChase(&pixels, 127, 127, 127, 50); //White, half brightness
        theaterChase(&pixels, 127,   0,   0, 50); //Red, half brightness
        theaterChase(&pixels,   0,   0, 127, 50); //Blue, half brightness

        rainbow(&pixels, 10);             // Flowing rainbow cycle along the whole strip
        theaterChaseRainbow(&pixels, 50); // Rainbow-enhanced theaterChase variant
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(STR_NEOPIXEL_T *pNeoPixel, uint8_t r, uint8_t g, uint8_t b, int wait) {
    for(int i=0; i<NeoPixel_numPixels(pNeoPixel); i++) {  // For each pixel in strip...
        NeoPixel_setPixelColor(pNeoPixel, i, r, g, b);  //  Set pixel's color (in RAM)
        NeoPixel_show(pNeoPixel);                       //  Update strip to match
        delay(wait);                                       //  Pause for a moment
    }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(STR_NEOPIXEL_T *pNeoPixel, uint8_t r, uint8_t g, uint8_t b, int wait) {
    for(int i=0; i<10; i++) {  // Repeat 10 times...
        for(int j=0; j<3; j++) { //  'j' counts from 0 to 2...
            NeoPixel_clear(pNeoPixel);         //   Set all pixels in RAM to 0 (off)
            // 'k' counts up from 'j' to end of strip in steps of 3...
            for(int k=j; k<NeoPixel_numPixels(pNeoPixel); k += 3) {
                NeoPixel_setPixelColor(pNeoPixel, k, r, g, b); // Set pixel 'k' to value 'color'
            }
            NeoPixel_show(pNeoPixel); // Update strip with new contents
            delay(wait);  // Pause for a moment
        }
    }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(STR_NEOPIXEL_T *pNeoPixel, int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<NeoPixel_numPixels(pNeoPixel); i++) { // For each pixel in strip...
        // Offset pixel hue by an amount to make one full revolution of the
        // color wheel (range of 65536) along the length of the strip
        // (strip.numPixels() steps):
        int pixelHue = firstPixelHue + (i * 65536L / NeoPixel_numPixels(pNeoPixel));
        // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
        // optionally add saturation and value (brightness) (each 0 to 255).
        // Here we're using just the single-argument hue variant. The result
        // is passed through strip.gamma32() to provide 'truer' colors
        // before assigning to each pixel:
        uint32_t color = NeoPixel_gamma32(NeoPixel_ColorHSV(pixelHue, 255, 255));
        uint8_t r = (uint8_t)(color >> 16);
        uint8_t g = (uint8_t)(color >> 8);
        uint8_t b = (uint8_t)color;
        NeoPixel_setPixelColor(pNeoPixel, i, r, g, b);
    }
    NeoPixel_show(pNeoPixel);   // Update strip with new contents
    delay(wait);                // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(STR_NEOPIXEL_T *pNeoPixel, int wait) {
    int firstPixelHue = 0;     // First pixel starts at red (hue 0)
    for(int a=0; a<30; a++) {  // Repeat 30 times...
        for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
            NeoPixel_clear(pNeoPixel);         //   Set all pixels in RAM to 0 (off)
            // 'c' counts up from 'b' to end of strip in increments of 3...
            for(int c=b; c<NeoPixel_numPixels(pNeoPixel); c += 3) {
                // hue of pixel 'c' is offset by an amount to make one full
                // revolution of the color wheel (range 65536) along the length
                // of the strip (strip.numPixels() steps):
                int      hue   = firstPixelHue + c * 65536L / NeoPixel_numPixels(pNeoPixel);
                uint32_t color = NeoPixel_gamma32(NeoPixel_ColorHSV(hue, 255, 255));
                uint8_t r = (uint8_t)(color >> 16);
                uint8_t g = (uint8_t)(color >> 8);
                uint8_t b = (uint8_t)color;
                NeoPixel_setPixelColor(pNeoPixel, c, r, g, b);
            }
            NeoPixel_show(pNeoPixel);    // Update strip with new contents
            delay(wait);                 // Pause for a moment
            firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
        }
    }
}
