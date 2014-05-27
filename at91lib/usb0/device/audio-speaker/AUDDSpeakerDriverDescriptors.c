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

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "AUDDSpeakerDriverDescriptors.h"
#include "AUDDSpeakerDriver.h"
#include <board.h>
#include <usb/common/core/USBGenericDescriptor.h>
#include <usb/common/core/USBDeviceDescriptor.h>
#include <usb/common/core/USBConfigurationDescriptor.h>
#include <usb/common/core/USBInterfaceDescriptor.h>
#include <usb/common/core/USBEndpointDescriptor.h>
#include <usb/common/core/USBStringDescriptor.h>
#include <usb/common/audio/AUDGenericDescriptor.h>
#include <usb/common/audio/AUDDeviceDescriptor.h>
#include <usb/common/audio/AUDControlInterfaceDescriptor.h>
#include <usb/common/audio/AUDStreamingInterfaceDescriptor.h>
#include <usb/common/audio/AUDEndpointDescriptor.h>
#include <usb/common/audio/AUDDataEndpointDescriptor.h>
#include <usb/common/audio/AUDFormatTypeOneDescriptor.h>
#include <usb/common/audio/AUDHeaderDescriptor.h>
#include <usb/common/audio/AUDFeatureUnitDescriptor.h>
#include <usb/common/audio/AUDInputTerminalDescriptor.h>
#include <usb/common/audio/AUDOutputTerminalDescriptor.h>
#include <app.h>
//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

/*
    Constants: Device IDs
        AUDDSpeakerDriverDescriptors_VENDORID - Device vendor ID.
        AUDDSpeakerDriverDescriptors_PRODUCTID - Device product ID.
        AUDDSpeakerDriverDescriptors_RELEASE - Device release number in BCD
            format.
*/
#define AUDDSpeakerDriverDescriptors_VENDORID                0x138C //0x03EB  // 
#if  defined(AUDIO_DP)
    #define AUDDSpeakerDriverDescriptors_PRODUCTID           0x6128  // 0x6128 DP 0x6127 DC
    #define AUDIO_NAME      USBStringDescriptor_UNICODE('P')
#elif defined(AUDIO_DC)
    #define AUDDSpeakerDriverDescriptors_PRODUCTID           0x6127  //DC
    #define AUDIO_NAME      USBStringDescriptor_UNICODE('C')
#else
    #error No AUDIO definition ?
#endif 
//#define AUDDSpeakerDriverDescriptors_PRODUCTID           0x6128  // 0x6128 DP 0x6127 DC
#define AUDDSpeakerDriverDescriptors_RELEASE             0x0100

//------------------------------------------------------------------------------
//         Internal types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Audio control header descriptor with one slave interface.
//------------------------------------------------------------------------------
#ifdef __ICCARM__          // IAR
#pragma pack(1)            // IAR
#define __attribute__(...) // IAR
#endif                     // IAR
extern volatile unsigned int FramRat ;
typedef struct {

    /// Header descriptor.
    AUDHeaderDescriptor header;
    /// Id of the first grouped interface.
    unsigned char bInterface0 ;
    unsigned char bInterface1 ;
} __attribute__ ((packed)) AUDHeaderDescriptor1; // GCC

//------------------------------------------------------------------------------
/// Feature unit descriptor with 3 channel controls (master, right, left).
//------------------------------------------------------------------------------
typedef struct {

    /// Feature unit descriptor.
    AUDFeatureUnitDescriptor feature;
    /// Available controls for each channel.
    unsigned char bmaControls[3];
    /// Index of a string descriptor for the feature unit.
    unsigned char iFeature;

} __attribute__ ((packed)) AUDFeatureUnitDescriptor3; // GCC

//------------------------------------------------------------------------------
/// List of descriptors for detailling the audio control interface of a
/// device using a USB audio speaker driver.
//------------------------------------------------------------------------------
typedef struct {

    /// Header descriptor (with one slave interface).
    AUDHeaderDescriptor1 header;
    
    AUDInputTerminalDescriptor input_mic;
    /// Output terminal descriptor.
    AUDOutputTerminalDescriptor output_mic;
    /// Input terminal descriptor.
    AUDInputTerminalDescriptor input;
    /// Output terminal descriptor.
    AUDOutputTerminalDescriptor output;
    
    /// Feature unit descriptor.
    //AUDFeatureUnitDescriptor3 feature_mic;
    /// Feature unit descriptor.
    //AUDFeatureUnitDescriptor3 feature;
    
    //AUDSelectUnitDescriptor Select ;  //PQ

} __attribute__ ((packed)) AUDDSpeakerDriverAudioControlDescriptors; // GCC

//------------------------------------------------------------------------------
/// Format type I descriptor with one discrete sampling frequency.
//------------------------------------------------------------------------------
typedef struct {

    /// Format type I descriptor.
    AUDFormatTypeOneDescriptor formatType;
    /// Sampling frequency in Hz.
    unsigned char tSamFreq[3];

} __attribute__ ((packed)) AUDFormatTypeOneDescriptor1; // GCC

//------------------------------------------------------------------------------
/// Holds a list of descriptors returned as part of the configuration of
/// a USB audio speaker device.
//------------------------------------------------------------------------------
typedef struct {

    /// Standard configuration.
    USBConfigurationDescriptor configuration;
    /// Audio control interface.
    USBInterfaceDescriptor control;
    /// Descriptors for the audio control interface.
    AUDDSpeakerDriverAudioControlDescriptors controlDescriptors;
    
    
    /// Streaming out interface descriptor (with no endpoint, required).
    USBInterfaceDescriptor streamingOutNoIsochronous;
    /// Streaming out interface descriptor.
    USBInterfaceDescriptor streamingOut;
    /// Audio class descriptor for the streaming out interface.
    AUDStreamingInterfaceDescriptor streamingOutClass;
    /// Stream format descriptor.
    AUDFormatTypeOneDescriptor1 streamingOutFormatType;
    /// Streaming out endpoint descriptor.
    AUDEndpointDescriptor streamingOutEndpoint;
    /// Audio class descriptor for the streaming out endpoint.
    AUDDataEndpointDescriptor streamingOutDataEndpoint; 

    
    USBInterfaceDescriptor streamingOutNoIsochronous_mic;                    // 无端点的stream 描述符
    /// Streaming out interface descriptor.
    USBInterfaceDescriptor streamingOut_mic;                                 // 有数据端点的stream 描述符
    /// Audio class descriptor for the streaming out interface.
    AUDStreamingInterfaceDescriptor streamingOutClass_mic;                   // audio stream 特殊描述符
    /// Stream format descriptor.
    AUDFormatTypeOneDescriptor1 streamingOutFormatType_mic;                  // 数据流格式描述符 
    /// Streaming out endpoint descriptor.
    AUDEndpointDescriptor streamingOutEndpoint_mic;                          // 端点描述符号
    //AUDEndpointDescriptor streamingOutEndpoint_mic1;    
    /// Audio class descriptor for the streaming out endpoint.
    AUDDataEndpointDescriptor streamingOutDataEndpoint_mic;                  // 端点特殊描述符
    
} __attribute__ ((packed)) AUDDSpeakerDriverConfigurationDescriptors; // GCC

#ifdef __ICCARM__          // IAR
#pragma pack()             // IAR
#endif                     // IAR

//------------------------------------------------------------------------------
//         Exported variables
//------------------------------------------------------------------------------

/// [Device descriptor] for a USB audio speaker driver.
const USBDeviceDescriptor deviceDescriptor = {

    sizeof(USBDeviceDescriptor),
    USBGenericDescriptor_DEVICE,
    USBDeviceDescriptor_USB2_00,
    AUDDeviceDescriptor_CLASS,  //audio
    AUDDeviceDescriptor_SUBCLASS,
    AUDDeviceDescriptor_PROTOCOL,
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(0),
    AUDDSpeakerDriverDescriptors_VENDORID,
    AUDDSpeakerDriverDescriptors_PRODUCTID,
    AUDDSpeakerDriverDescriptors_RELEASE,
    1, // Manufacturer string descriptor index
    2, // Product string descriptor index
    3, // Index of serial number string descriptor
    1  // One possible configuration
};

#if defined(BOARD_USB_UDPHS)

/// USB device qualifier descriptor.
const USBDeviceQualifierDescriptor qualifierDescriptor = {

    sizeof(USBDeviceDescriptor),
    USBGenericDescriptor_DEVICE,
    USBDeviceDescriptor_USB2_00,
    AUDDeviceDescriptor_CLASS,
    AUDDeviceDescriptor_SUBCLASS,
    AUDDeviceDescriptor_PROTOCOL,
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(0),
    1, // Device has one possible configuration
    0 // Reserved
};

#endif



/// Configuration descriptors for a USB audio mic & speaker driver.
 AUDDSpeakerDriverConfigurationDescriptors configurationDescriptors = {

    // [Configuration descriptor]
    {
        sizeof(USBConfigurationDescriptor),
        USBGenericDescriptor_CONFIGURATION,
        sizeof(AUDDSpeakerDriverConfigurationDescriptors), //Length of the total configuration block including this descriptor, in bytes.
        3, // This configuration has 3 interfaces
        1, // This is configuration ID #1
        0, // No string descriptor
        BOARD_USB_BMATTRIBUTES,
        USBConfigurationDescriptor_POWER(100)
    },
    
    ///////////////////////////////////////////////////////////////////////
    // USB interface [standard AC interface descriptor] 
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        AUDDSpeakerDriverDescriptors_CONTROL, //Index of this interface : 0
        0, // Index of this seeting. This is alternate setting #0
        0, // This interface uses no endpoint
        AUDControlInterfaceDescriptor_CLASS,
        AUDControlInterfaceDescriptor_SUBCLASS,
        AUDControlInterfaceDescriptor_PROTOCOL,
        0 // No string descriptor
    },    
    
    // Audio control interface descriptors
    {
        // Header descriptor
        {
            {
                sizeof(AUDHeaderDescriptor1),
                AUDGenericDescriptor_INTERFACE,//CS_INTERFACE : 0x24
                AUDGenericDescriptor_HEADER    ,//HEADER subtype : 01
                AUDHeaderDescriptor_AUD1_00,
                sizeof(AUDDSpeakerDriverAudioControlDescriptors),
                2 //two streaming interface used
            },
            AUDDSpeakerDriverDescriptors_STREAMING,
            AUDDSpeakerDriverDescriptors_STREAMINGIN
        },
        
        
        
        // Input terminal descriptor for mic -> usb
        {
            sizeof(AUDInputTerminalDescriptor),                // 终端描述符大小
            AUDGenericDescriptor_INTERFACE,                    // 普通音频接口描述符
            AUDGenericDescriptor_INPUTTERMINAL,                // 子类 输入终端描述符
            AUDDSpeakerDriverDescriptors_INPUTTERMINAL_MIC,    // 终端ID号  4
            AUDOutputTerminalDescriptor_MICPHON,               // 终端类型 0x0201
            0,//AUDDSpeakerDriverDescriptors_OUTPUTTERMINAL_MIC, /////0,//  // 与该输入有关的输出终端ID号
            AUDDSpeakerDriver_NUMCHANNELS,                     // 通道数量 2通道
            AUDInputTerminalDescriptor_LEFTFRONT
            | AUDInputTerminalDescriptor_RIGHTFRONT,           // 通道说明 左右通道
            0, // No string descriptor for channels            // 没有字符串描述符描述通道
            0  // No string descriptor for input terminal      // 没有字符串描述符描述终端
        },
        
        // Output terminal descriptor mic -> usb
        {//
            sizeof(AUDOutputTerminalDescriptor),               // 终端描述符大小
            AUDGenericDescriptor_INTERFACE,                    // 普通音频接口描述符
            AUDGenericDescriptor_OUTPUTTERMINAL,               // 子类 输出终端描述符
            AUDDSpeakerDriverDescriptors_OUTPUTTERMINAL_MIC,   // 终端ID号
            AUDInputTerminalDescriptor_USBSTREAMING,           // 终端类型 USB
            0, //AUDDSpeakerDriverDescriptors_INPUTTERMINAL_MIC,  //////0,//  // 与之相关的输入终端ID号
            AUDDSpeakerDriverDescriptors_INPUTTERMINAL_MIC, /////7,//      // 与之有关联的单元  ????????       
            0 // No string descriptor                          // 没有字符串描述符描述终端
        },
        
        
        // Input terminal descriptor usb -> spk
        {
            sizeof(AUDInputTerminalDescriptor),
            AUDGenericDescriptor_INTERFACE,
            AUDGenericDescriptor_INPUTTERMINAL,
            AUDDSpeakerDriverDescriptors_INPUTTERMINAL, //
            AUDInputTerminalDescriptor_USBSTREAMING,
            0, //AUDDSpeakerDriverDescriptors_OUTPUTTERMINAL,  //////
            AUDDSpeakerDriver_NUMCHANNELS,
            AUDInputTerminalDescriptor_LEFTFRONT
            | AUDInputTerminalDescriptor_RIGHTFRONT,
            0, // No string descriptor for channels
            0 // No string descriptor for input terminal
        },
        // Output terminal descriptor usb -> spk
        {
            sizeof(AUDOutputTerminalDescriptor),
            AUDGenericDescriptor_INTERFACE,
            AUDGenericDescriptor_OUTPUTTERMINAL,
            AUDDSpeakerDriverDescriptors_OUTPUTTERMINAL,  //////
            AUDOutputTerminalDescriptor_SPEAKER,
            0,//No association. 
            AUDDSpeakerDriverDescriptors_INPUTTERMINAL,
            0 // No string descriptor
        },
        
        
        /*
        // Feature unit descriptor  mic -> spk
        {
            {
                sizeof(AUDFeatureUnitDescriptor3),             // feature 描述符大小
                AUDGenericDescriptor_INTERFACE,                // 普通音频接口描述符
                AUDGenericDescriptor_FEATUREUNIT,              // 子类 feature 单元描述符
                AUDDSpeakerDriverDescriptors_FEATUREUNIT_MIC,      // 单元ID
                AUDDSpeakerDriverDescriptors_INPUTTERMINAL_MIC, // 与之有关的终端
                1, // 1 byte per channel for controls           // 控制字每通道一个字节
            },
            {     
                AUDFeatureUnitDescriptor_MUTE,            // Master channel controls               
                0,  // no Right  channel controls                    
                0,  // no Left   channel controls
            },
            0                                                    // 没有字符串描述符来描述该feature
        } ,
        
        // Feature unit descriptor
        {
            {
                sizeof(AUDFeatureUnitDescriptor3),
                AUDGenericDescriptor_INTERFACE,
                AUDGenericDescriptor_FEATUREUNIT,
                AUDDSpeakerDriverDescriptors_FEATUREUNIT,
                AUDDSpeakerDriverDescriptors_INPUTTERMINAL,
                1, // 1 byte per channel for controls
            },
            {
                AUDFeatureUnitDescriptor_MUTE, // Master channel controls
                0, // no Right channel controls
                0  // no Left channel controls
            },
            0 // No string descriptor
        },
        
        */
        
        //select unit description  //***************  actually not used here PQ
        /*        
        {   //0x06,0x24,0x05,0x07,0x01,AUDDSpeakerDriverDescriptors_FEATUREUNIT_MIC
            sizeof(AUDSelectUnitDescriptor), 
            AUDGenericDescriptor_INTERFACE,
            AUDGenericDescriptor_SELECTORUNIT,
            AUDDSpeakerDriverDescriptors_SELECTUNIT,
            1,
            AUDDSpeakerDriverDescriptors_FEATUREUNIT_MIC,
            0              
        },
        */
        
    },
    
       
    
    
    // mic -> usb   ---- for record
    // Audio streaming interface with 0 endpoints            [zero-bandwidth setting]
    {
        sizeof(USBInterfaceDescriptor),                        // 描述符大小
        USBGenericDescriptor_INTERFACE,                        // 普通接口描述符
        AUDDSpeakerDriverDescriptors_STREAMINGIN,              // 接口描述符编号
        0, // This is alternate setting #0                     // 描述符ID
        0, // This interface uses no endpoints                 // 0 个端点用在该接口
        AUDStreamingInterfaceDescriptor_CLASS,                 // AUDIO 类
        AUDStreamingInterfaceDescriptor_SUBCLASS,              // AUDIO stream 子类
        AUDStreamingInterfaceDescriptor_PROTOCOL,              // 没用字节 必须为 0  
        0 // No string descriptor                              // 没有字符串描述符
    },
    
    // Audio streaming interface with data endpoint            [operational alternate setting]
    {
        sizeof(USBInterfaceDescriptor),                        // 描述符大小
        USBGenericDescriptor_INTERFACE,                        // 普通接口描述符
        AUDDSpeakerDriverDescriptors_STREAMINGIN,              // 接口描述符编号
        1, // This is alternate setting #1                     // 描述符ID
        1, // This interface uses 1 endpoint                   // 1 个端点用在该接口
        AUDStreamingInterfaceDescriptor_CLASS,                 // AUDIO 类
        AUDStreamingInterfaceDescriptor_SUBCLASS,              // AUDIO stream 子类
        AUDStreamingInterfaceDescriptor_PROTOCOL,              // 没用字节 必须为 0  
        0  // No string descriptor                             // 没有字符串描述符
    },
    
    // Audio streaming class-specific descriptor               6
    {
        sizeof(AUDStreamingInterfaceDescriptor),               // 描述符大小
        AUDGenericDescriptor_INTERFACE,                        // 普通音频接口描述符 
        AUDStreamingInterfaceDescriptor_GENERAL,               // audio stream 子类
        AUDDSpeakerDriverDescriptors_OUTPUTTERMINAL_MIC,       // 该接口关联的 终端
        0, // No internal delay because of data path           // 通道延时
        AUDFormatTypeOneDescriptor_PCM                         // 接口 通信的数据格式
    },
    // Format type I descriptor                                7                   
    // 数据格式描述符号
    {
        {
            sizeof(AUDFormatTypeOneDescriptor1),               // 描述符大小
            AUDGenericDescriptor_INTERFACE,                    // 普通接口描述符
            AUDStreamingInterfaceDescriptor_FORMATTYPE,        // 子类 formate type
            AUDFormatTypeOneDescriptor_FORMATTYPEONE,          // 数据格式类型 
            AUDDSpeakerDriver_NUMCHANNELS,                     // 通道个数 两个左右声道
            AUDDSpeakerDriver_BYTESPERSAMPLE,                  // 数据长度 两字节
            AUDDSpeakerDriver_BYTESPERSAMPLE*8,                // 数据长度 2*8 16 bit
            1 // One discrete frequency supported              // 支持采用频率的种数
        },
        {
            AUDDSpeakerDriver_SAMPLERATE_DEF & 0xFF,          // 采样频率 三 byte
            (AUDDSpeakerDriver_SAMPLERATE_DEF >> 8) & 0xFF,
            (AUDDSpeakerDriver_SAMPLERATE_DEF >> 16) & 0xFF
        }
    },
    
    // Audio streaming endpoint standard descriptor            8
    {
        sizeof(AUDEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(
            USBEndpointDescriptor_IN, //data from dev to host --record
            AUDDSpeakerDriverDescriptors_DATAIN),
        USBEndpointDescriptor_ISOCHRONOUS,
        BOARD_USB_ENDPOINTS_MAXPACKETSIZE(AUDDSpeakerDriverDescriptors_DATAIN),
        1, // Polling interval = 2^(x-1) milliseconds (1 ms)
        0, // This is not a synchronization endpoint
        0  // No associated synchronization endpoint
    },
    // Audio streaming endpoint class-specific descriptor      9
    {
        sizeof(AUDDataEndpointDescriptor),
        AUDGenericDescriptor_ENDPOINT,
        AUDDataEndpointDescriptor_SUBTYPE,
        0, // No attributes
        0, // Endpoint is not synchronized
        0  // Endpoint is not synchronized
    },
    
    
        
      
    
     // usb -> spk   ---- for play
    // Audio streaming interface with 0 endpoints    [zero-bandwidth setting]
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        AUDDSpeakerDriverDescriptors_STREAMING,// out
        0, // This is alternate setting #0
        0, // This interface uses no endpoints
        AUDStreamingInterfaceDescriptor_CLASS,
        AUDStreamingInterfaceDescriptor_SUBCLASS,
        AUDStreamingInterfaceDescriptor_PROTOCOL,
        0 // No string descriptor
    },
    // Audio streaming interface with data endpoint    [operational alternate setting]
    {
        sizeof(USBInterfaceDescriptor),
        USBGenericDescriptor_INTERFACE,
        AUDDSpeakerDriverDescriptors_STREAMING,// out
        1, // This is alternate setting #1
        1, // This interface uses 1 endpoint
        AUDStreamingInterfaceDescriptor_CLASS,
        AUDStreamingInterfaceDescriptor_SUBCLASS,
        AUDStreamingInterfaceDescriptor_PROTOCOL,
        0 // No string descriptor
    },
    // Audio streaming class-specific descriptor
    {
        sizeof(AUDStreamingInterfaceDescriptor),
        AUDGenericDescriptor_INTERFACE,
        AUDStreamingInterfaceDescriptor_GENERAL,
        AUDDSpeakerDriverDescriptors_INPUTTERMINAL,
        0, // No internal delay because of data path
        AUDFormatTypeOneDescriptor_PCM
    },
    // Format type I descriptor
    {
        {
            sizeof(AUDFormatTypeOneDescriptor1),
            AUDGenericDescriptor_INTERFACE,
            AUDStreamingInterfaceDescriptor_FORMATTYPE,
            AUDFormatTypeOneDescriptor_FORMATTYPEONE,
            AUDDSpeakerDriver_NUMCHANNELS,
            AUDDSpeakerDriver_BYTESPERSAMPLE,
            AUDDSpeakerDriver_BYTESPERSAMPLE*8,
            1 // One discrete frequency supported
        },
        {
            AUDDSpeakerDriver_SAMPLERATE_DEF & 0xFF,
            (AUDDSpeakerDriver_SAMPLERATE_DEF >> 8) & 0xFF,
            (AUDDSpeakerDriver_SAMPLERATE_DEF >> 16) & 0xFF
        }
    },
    // Audio streaming endpoint standard descriptor
    {
        sizeof(AUDEndpointDescriptor),
        USBGenericDescriptor_ENDPOINT,
        USBEndpointDescriptor_ADDRESS(
            USBEndpointDescriptor_OUT,
            AUDDSpeakerDriverDescriptors_DATAOUT),
        USBEndpointDescriptor_ISOCHRONOUS,
        BOARD_USB_ENDPOINTS_MAXPACKETSIZE(AUDDSpeakerDriverDescriptors_DATAOUT),
        1, // Polling interval = 2^(x-1) milliseconds (1 ms)
        0, // This is not a synchronization endpoint
        0 // No associated synchronization endpoint
    },
    // Audio streaming endpoint class-specific descriptor
    {
        sizeof(AUDDataEndpointDescriptor),
        AUDGenericDescriptor_ENDPOINT,
        AUDDataEndpointDescriptor_SUBTYPE,
        0, // No attributes
        0, // Endpoint is not synchronized
        0 // Endpoint is not synchronized
    }
};



/// String descriptor with the supported languages.
const unsigned char languageIdDescriptor[] = {

    USBStringDescriptor_LENGTH(1),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_ENGLISH_US
};


/// Manufacturer name.
const unsigned char manufacturerDescriptor[] = {

    USBStringDescriptor_LENGTH(10),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('F'),
    USBStringDescriptor_UNICODE('o'),
    USBStringDescriptor_UNICODE('r'),
    USBStringDescriptor_UNICODE('t'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('m'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('d'),
    USBStringDescriptor_UNICODE('i'),
    USBStringDescriptor_UNICODE('a')
};

/// Product name. Jupiter
const unsigned char productDescriptor[] = {

    USBStringDescriptor_LENGTH(15),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('J'),
    USBStringDescriptor_UNICODE('u'),
    USBStringDescriptor_UNICODE('p'),
    USBStringDescriptor_UNICODE('i'),
    USBStringDescriptor_UNICODE('t'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('r'),
    USBStringDescriptor_UNICODE('_'),
    USBStringDescriptor_UNICODE('A'),
    USBStringDescriptor_UNICODE('u'),
    USBStringDescriptor_UNICODE('d'),
    USBStringDescriptor_UNICODE('i'),
    USBStringDescriptor_UNICODE('o'),  
    USBStringDescriptor_UNICODE('D'),
    AUDIO_NAME //USBStringDescriptor_UNICODE('P')
};

/// Product serial number.
const unsigned char serialNumberDescriptor[] = {

    USBStringDescriptor_LENGTH(4),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('0'),
    USBStringDescriptor_UNICODE('1'),
    USBStringDescriptor_UNICODE('2'),
    USBStringDescriptor_UNICODE('3')
};

/// Array of pointers to the four string descriptors.
const unsigned char *stringDescriptors[] = {
  
    languageIdDescriptor,
    manufacturerDescriptor,
    productDescriptor,
    serialNumberDescriptor,
    
};

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

/// List of descriptors required by an USB audio speaker device driver.
const USBDDriverDescriptors auddSpeakerDriverDescriptors = {

    &deviceDescriptor,
    (const USBConfigurationDescriptor *) &configurationDescriptors,
#ifdef BOARD_USB_UDPHS
    &qualifierDescriptor,
    (const USBConfigurationDescriptor *) &configurationDescriptors,
    &deviceDescriptor,
    (const USBConfigurationDescriptor *) &configurationDescriptors,
    &qualifierDescriptor,
    (const USBConfigurationDescriptor *) &configurationDescriptors,
#else
    0, 0, 0, 0, 0, 0,
#endif
    stringDescriptors,
    4 // Number of string descriptors
};

void SetBratInDescriptor(unsigned int fr)
{   
    ChechLenPerFramInt = fr/1000 ;
    configurationDescriptors.streamingOutFormatType.tSamFreq[0] = fr & 0xFF;
    configurationDescriptors.streamingOutFormatType.tSamFreq[1] = (fr >> 8) & 0xFF;
    configurationDescriptors.streamingOutFormatType.tSamFreq[2] = (fr >> 16) & 0xFF;
    configurationDescriptors.streamingOutFormatType_mic.tSamFreq[0] = fr & 0xFF;
    configurationDescriptors.streamingOutFormatType_mic.tSamFreq[1] = (fr >> 8) & 0xFF;
    configurationDescriptors.streamingOutFormatType_mic.tSamFreq[2] = (fr >> 16) & 0xFF; 
}
/*
void SetAudioName(unsigned short prdId)
{
    deviceDescriptor.idProduct = prdId ; 
    if(deviceDescriptor.idProduct == 0x6128)
    {
    }
    else
    {
        productDescriptor[30] = 'C' ;
    }
}
*/
