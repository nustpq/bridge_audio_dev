/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*
    Title: AUDDSpeakerDriverDescriptors

    About: Purpose
        Declaration of the descriptors required by a USB audio speaker driver.

    About: Usage
        1 - Initialize a USBDDriver instance using the
            <auddSpeakerDriverDescriptors> list.
*/

#ifndef AUDDSPEAKERDRIVERDESCRIPTORS_H
#define AUDDSPEAKERDRIVERDESCRIPTORS_H

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <usb/device/core/USBDDriverDescriptors.h>

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------
/*
    Constants: Endpoint numbers
        AUDDSpeakerDriverDescriptors_DATAOUT - Data out endpoint number.
*/
#if defined(BOARD_USB_UDPHS)
    #define AUDDSpeakerDriverDescriptors_DATAOUT        0x01
    #define AUDDSpeakerDriverDescriptors_DATAIN         0x02
#else
    #define AUDDSpeakerDriverDescriptors_DATAOUT        0x04 //0x01
    #define AUDDSpeakerDriverDescriptors_DATAIN         0x05 //0x02  //data from dev to host  -- record
#endif

/*  
    Constants: Interface IDs
        AUDDSpeakerDriverDescriptors_CONTROL - Audio control interface ID.
        AUDDSpeakerDriverDescriptors_STREAMING - Audio streaming interface ID.
*/
#define AUDDSpeakerDriverDescriptors_CONTROL            0

#define AUDDSpeakerDriverDescriptors_STREAMINGIN        1 //in  -- record

#define AUDDSpeakerDriverDescriptors_STREAMING          2 //out -- play
/*
    Constants: Entity IDs
        AUDDSpeakerDriverDescriptors_INPUTTERMINAL - Input terminal ID.
        AUDDSpeakerDriverDescriptors_OUTPUTTERMINAL - Output terminal ID.
        AUDDSpeakerDriverDescriptors_FEATUREUNIT - Feature unit ID.
*/
//PQ modified
#define AUDDSpeakerDriverDescriptors_INPUTTERMINAL      1 // STREAMING
#define AUDDSpeakerDriverDescriptors_OUTPUTTERMINAL     2

#define AUDDSpeakerDriverDescriptors_INPUTTERMINAL_MIC  3
#define AUDDSpeakerDriverDescriptors_OUTPUTTERMINAL_MIC 4   // STREAMINGIN

#define AUDDSpeakerDriverDescriptors_FEATUREUNIT        5
#define AUDDSpeakerDriverDescriptors_FEATUREUNIT_MIC    6

#define AUDDSpeakerDriverDescriptors_SELECTUNIT         7
//------------------------------------------------------------------------------
//         Exported variables
//------------------------------------------------------------------------------
/*
    Variable: auddSpeakerDriverDescriptors
        List of descriptors required by an USB audio speaker device driver.
*/
extern const USBDDriverDescriptors auddSpeakerDriverDescriptors;


void SetBratInDescriptor(unsigned int fr);
void SetAudioName( void );

#endif //#ifndef AUDDSPEAKERDRIVERDESCRIPTORS_H

