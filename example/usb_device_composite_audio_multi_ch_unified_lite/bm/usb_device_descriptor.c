/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_audio.h"
#include "usb_device_descriptor.h"
#include "audio_unified.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t g_UsbDeviceCurrentConfigure = 0U;
uint8_t g_UsbDeviceInterface[USB_COMPOSITE_INTERFACE_COUNT];

/* Define device descriptor */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceDescriptor[] = {
    USB_DESCRIPTOR_LENGTH_DEVICE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_DEVICE,   /* DEVICE Descriptor Type */
    USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION), /* USB Specification Release Number in
                                                            Binary-Coded Decimal (i.e., 2.10 is 210H). */
    USB_DEVICE_CLASS,                                    /* Class code (assigned by the USB-IF). */
    USB_DEVICE_SUBCLASS,                                 /* Subclass code (assigned by the USB-IF). */
    USB_DEVICE_PROTOCOL,                                 /* Protocol code (assigned by the USB-IF). */
    USB_CONTROL_MAX_PACKET_SIZE,                         /* Maximum packet size for endpoint zero
                                                            (only 8, 16, 32, or 64 are valid) */
    USB_SHORT_GET_LOW(USB_DEVICE_VID),
    USB_SHORT_GET_HIGH(USB_DEVICE_VID), /* Vendor ID (assigned by the USB-IF) */
    USB_SHORT_GET_LOW(USB_DEVICE_PID),
    USB_SHORT_GET_HIGH(USB_DEVICE_PID), /* Product ID (assigned by the manufacturer) */
    USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION), /* Device release number in binary-coded decimal */
    0x01U,                                           /* Index of string descriptor describing manufacturer */
    0x02U,                                           /* Index of string descriptor describing product */
    0x00U,                                           /* Index of string descriptor describing the
                                                        device's serial number */
    USB_DEVICE_CONFIGURATION_COUNT,                  /* Number of possible configurations */
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceConfigurationDescriptor[] = {
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
    USB_DESCRIPTOR_LENGTH_CONFIGURE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CONFIGURE,   /* CONFIGURATION Descriptor Type */
    USB_SHORT_GET_LOW(USB_DESCRIPTOR_LENGTH_CONFIGURE + 0x08U + USB_DESCRIPTOR_LENGTH_INTERFACE +
                      USB_AUDIO_CONTROL_INTERFACE_HEADER_LENGTH + USB_AUDIO_CLOCK_SOURCE_LENGTH + 0x11U + 0x4AU +
                      0x0CU + USB_AUDIO_CLOCK_SOURCE_LENGTH + 0x11U + 0x4AU + 0x0CU + USB_DESCRIPTOR_LENGTH_INTERFACE +
                      USB_DESCRIPTOR_LENGTH_INTERFACE + 0x10U + 0x06U + USB_AUDIO_STREAMING_ENDP_DESC_SIZE +
                      USB_AUDIO_CLASS_SPECIFIC_ENDPOINT_LENGTH + USB_DESCRIPTOR_LENGTH_INTERFACE +
                      USB_DESCRIPTOR_LENGTH_INTERFACE + 0x10U + 0x06U + USB_AUDIO_STREAMING_ENDP_DESC_SIZE +
                      USB_AUDIO_CLASS_SPECIFIC_ENDPOINT_LENGTH
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#else
                      + USB_AUDIO_STANDARD_AS_ISO_FEEDBACK_ENDPOINT_LENGTH
#endif
                      ),
    USB_SHORT_GET_HIGH(USB_DESCRIPTOR_LENGTH_CONFIGURE + 0x08U + USB_DESCRIPTOR_LENGTH_INTERFACE +
                       USB_AUDIO_CONTROL_INTERFACE_HEADER_LENGTH + USB_AUDIO_CLOCK_SOURCE_LENGTH + 0x11U + 0x4AU +
                       0x0CU + USB_AUDIO_CLOCK_SOURCE_LENGTH + 0x11U + 0x4AU + 0x0CU + USB_DESCRIPTOR_LENGTH_INTERFACE +
                       USB_DESCRIPTOR_LENGTH_INTERFACE + 0x10U + 0x06U + USB_AUDIO_STREAMING_ENDP_DESC_SIZE +
                       USB_AUDIO_CLASS_SPECIFIC_ENDPOINT_LENGTH + USB_DESCRIPTOR_LENGTH_INTERFACE +
                       USB_DESCRIPTOR_LENGTH_INTERFACE + 0x10U + 0x06U + USB_AUDIO_STREAMING_ENDP_DESC_SIZE +
                       USB_AUDIO_CLASS_SPECIFIC_ENDPOINT_LENGTH
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#else
                       + USB_AUDIO_STANDARD_AS_ISO_FEEDBACK_ENDPOINT_LENGTH
#endif
                       ),
    USB_DEVICE_INTERFACE_COUNT,    /* Number of interfaces supported by this configuration */
    USB_COMPOSITE_CONFIGURE_INDEX, /* Value to use as an argument to the
                                      SetConfiguration() request to select this configuration */
    0x00U,                         /* Index of string descriptor describing this configuration */
    (USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK) |
#if defined(USB_DEVICE_CONFIG_SELF_POWER) && (USB_DEVICE_CONFIG_SELF_POWER > 0U)
    (1U << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT) |
#endif
#if defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U)
    (1U << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT) |
#endif
     0U,
    /* Configuration characteristics
       D7: Reserved (set to one)
       D6: Self-powered
       D5: Remote Wakeup
       D4...0: Reserved (reset to zero)
    */
    0xFAU, /* Maximum power consumption of the USB
            * device from the bus in this specific
            * configuration when the device is fully
            * operational. Expressed in 2 mA units
            *  (i.e., 50 = 100 mA).
            */

    0x08U,                                     /* Descriptor size is 8 bytes  */
    USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION, /* INTERFACE_ASSOCIATION Descriptor Type   */
    0x00U,                                     /* The first interface number associated with this function is 0   */
    0x03U,              /* The number of contiguous interfaces associated with this function is 2   */
    USB_AUDIO_CLASS,    /* The function belongs to the Audio Interface Class  */
    0x00U,              /* The function belongs to the SUBCLASS_UNDEFINED Subclass   */
    USB_AUDIO_PROTOCOL, /* Protocol code = 32   */
    0x00U,              /* The Function string descriptor index is 0  */

    USB_DESCRIPTOR_LENGTH_INTERFACE,         /* Size of the descriptor, in bytes  */
    USB_DESCRIPTOR_TYPE_INTERFACE,           /* INTERFACE Descriptor Type   */
    USB_AUDIO_CONTROL_INTERFACE_INDEX,       /* The number of this interface is 0 */
    USB_AUDIO_CONTROL_INTERFACE_ALTERNATE_0, /* The value used to select the alternate setting for this interface is 0
                                              */
    0x00U,                     /* The number of endpoints used by this interface is 0 (excluding endpoint zero)   */
    USB_AUDIO_CLASS,           /* The interface implements the Audio Interface class   */
    USB_SUBCLASS_AUDIOCONTROL, /* The interface implements the AUDIOCONTROL Subclass   */
    USB_AUDIO_PROTOCOL,        /* The Protocol code is 32   */
    0x00U,                     /* The interface string descriptor index is 2  */

    USB_AUDIO_CONTROL_INTERFACE_HEADER_LENGTH,   /* Size of the descriptor, in bytes  */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,      /* CS_INTERFACE Descriptor Type   */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_HEADER, /* HEADER descriptor subtype  */
    0x00U,
    0x02U, /* Audio Device compliant to the USB Audio specification version 2.00  */
    0x04,  /* Undefied(0x00) : Indicating the primary use of this audio function   */
    0xE7U,
    0x00U, /* Total number of bytes returned for the class-specific
             AudioControl interface descriptor. Includes the combined length of this descriptor header and all
             Unit and Terminal descriptors.   */
    0x00U, /* D1..0: Latency Control  */

    USB_AUDIO_CLOCK_SOURCE_LENGTH,                          /* Size of the descriptor, in bytes  */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,                 /* CS_INTERFACE Descriptor Type  */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_SOURCE_UNIT, /* CLOCK_SOURCE descriptor subtype  */
    USB_AUDIO_CONTROL_RECORDER_CLOCK_SOURCE_ENTITY_ID, /* Constant uniquely identifying the Clock Source Entity within
                                                        * the audio funcion
                                                        */
    0x01U,                                             /* D1..0: 01: Internal Fixed Clock
                                                          D2: 0 Clock is not synchronized to SOF
                                                          D7..3: Reserved, should set to 0   */
    0x07U, /* D1..0: Clock Frequency Control is present and Host programmable
              D3..2: Clock Validity Control is present but read-only
              D7..4: Reserved, should set to 0 */
    0x00U, /* This Clock Source has no association   */
    0x00U, /* Index of a string descriptor, describing the Clock Source Entity  */

    0x11U,                                               /* Size of the descriptor, in bytes  */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,              /* CS_INTERFACE Descriptor Type   */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_INPUT_TERMINAL, /* INPUT_TERMINAL descriptor subtype   */
    USB_AUDIO_RECORDER_CONTROL_INPUT_TERMINAL_ID,        /* Constant uniquely identifying the Terminal within the audio
                     function. This value is used in all requests        to address this Terminal.   */
    0x01U,
    0x02U, /* A generic microphone that does not fit under any of the other classifications.  */
    0x00U, /* This Input Terminal has no association   */
    USB_AUDIO_CONTROL_RECORDER_CLOCK_SOURCE_ENTITY_ID, /* ID of the Clock Entity to which this Input Terminal is
                                                          connected.  */
    0x10, /* This Terminal's output audio channel cluster has 16 logical output channels   */
    0xFFU,
    0xFFU,
    0x00U,
    0x00U, /* Describes the spatial location of the logical channels:: Mono, no spatial location */
    0x00U, /* Index of a string descriptor, describing the name of the first logical channel.  */
    0x00U,
    0x00U, /* bmControls D1..0: Copy Protect Control is not present
              D3..2: Connector Control is not present
              D5..4: Overload Control is not present
              D7..6: Cluster Control is not present
              D9..8: Underflow Control is not present
              D11..10: Overflow Control is not present
              D15..12: Reserved, should set to 0*/
    0x00U, /* Index of a string descriptor, describing the Input Terminal.  */

    0x4AU,                                             /* Size of the descriptor, in bytes  : 6 + (16 + 1) * 4 */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,            /* CS_INTERFACE Descriptor Type   */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_FEATURE_UNIT, /* FEATURE_UNIT descriptor subtype   */
    USB_AUDIO_RECORDER_CONTROL_FEATURE_UNIT_ID,   /* Constant uniquely identifying the Unit within the audio function.
                This value is used in all requests to   address this Unit.  */
    USB_AUDIO_RECORDER_CONTROL_INPUT_TERMINAL_ID, /* ID of the Unit or Terminal to which this Feature Unit is connected.
                                                   */
    0x0FU,
    0x00U,
    0x00U,
    0x00U, /* logic channel 0 bmaControls(0)(0x0000000F):  D1..0: Mute Control is present and host
              programmable D3..2: Volume Control is present and host programmable D5..4: Bass
              Control is not present D7..6: Mid Control is not present D9..8: Treble Control is not
              present D11..10: Graphic Equalizer Control is not present D13..12: Automatic Gain
              Control is not present D15..14: Delay Control is not present D17..16: Bass Control is
              not present D19..18: Loudness Control is not present D21..20: Input Gain Control is
              not present D23..22: Input Gain Pad Control is not present D25..24: Phase Inverter
              Control is not present D27..26: Underflow Control is not present D29..28: Overflow
              Control is not present D31..30: Reserved, should set to 0 */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 1. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 2. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 3. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 4. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 5. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 6. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 7. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 8. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 9. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 10. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 11. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 12. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 13. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 14. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 15. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 16. */
    0x00U, /* Index of a string descriptor, describing this Feature Unit.   */

    0x0CU,                                                /* Size of the descriptor, in bytes   */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,               /* CS_INTERFACE Descriptor Type  */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_OUTPUT_TERMINAL, /* OUTPUT_TERMINAL descriptor subtype   */
    USB_AUDIO_RECORDER_CONTROL_OUTPUT_TERMINAL_ID,        /* Constant uniquely identifying the Terminal within the audio
                     function. This value is used in all requests        to address this Terminal.   */
    0x01U,
    0x01U, /* A Terminal dealing with a signal carried over an endpoint in an AudioStreaming interface. The
           AudioStreaming interface descriptor points to the associated Terminal through the bTerminalLink field.  */
    0x00U, /* This Output Terminal has no association  */
    USB_AUDIO_RECORDER_CONTROL_FEATURE_UNIT_ID, /* ID of the Unit or Terminal to which this Terminal is connected.  */
    USB_AUDIO_CONTROL_RECORDER_CLOCK_SOURCE_ENTITY_ID, /* ID of the Clock Entity to which this Output Terminal is
                                                          connected  */
    0x00U,
    0x00U, /* bmControls:   D1..0: Copy Protect Control is not present
              D3..2: Connector Control is not present
              D5..4: Overload Control is not present
              D7..6: Underflow Control is not present
              D9..8: Overflow Control is not present
              D15..10: Reserved, should set to 0   */
    0x00U, /* Index of a string descriptor, describing the Output Terminal.  */

    USB_AUDIO_CLOCK_SOURCE_LENGTH,                    /* Size of the descriptor, in bytes  */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,           /* CS_INTERFACE Descriptor Type  */
    0x0AU,                                            /* CLOCK_SOURCE descriptor subtype  */
    USB_AUDIO_CONTROL_SPEAKER_CLOCK_SOURCE_ENTITY_ID, /* Constant uniquely identifying the Clock Source Entity within
                                                       * the audio funcion
                                                       */
    0x01U,                                            /* D1..0: 01: Internal Fixed Clock
                                                         D2: 0 Clock is not synchronized to SOF
                                                         D7..3: Reserved, should set to 0   */
    0x07U,                                            /* D1..0: Clock Frequency Control is present and Host programmable
                                                         D3..2: Clock Validity Control is present but read-only
                                                         D7..4: Reserved, should set to 0 */
    0x00U,                                            /* This Clock Source has no association   */
    0x00U, /* Index of a string descriptor, describing the Clock Source Entity  */

    0x11U,                                               /* Size of the descriptor, in bytes  */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,              /* CS_INTERFACE Descriptor Type   */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_INPUT_TERMINAL, /* INPUT_TERMINAL descriptor subtype   */
    USB_AUDIO_SPEAKER_CONTROL_INPUT_TERMINAL_ID,         /* Constant uniquely identifying the Terminal within the audio
                      function. This value is used in all requests         to address this Terminal.   */
    0x01U,
    0x01U, /* A Terminal dealing with a signal carried over an endpoint in an AudioStreaming interface. The
              AudioStreaming interface descriptor points to the associated Terminal through the bTerminalLink field. */
    0x00U, /* This Input Terminal has no association   */
    USB_AUDIO_CONTROL_SPEAKER_CLOCK_SOURCE_ENTITY_ID, /* ID of the Clock Entity to which this Input Terminal is
                                                         connected.  */
    0x10U, /* This Terminal's output audio channel cluster has 16 logical output channels   */
    0xFFU,
    0xFFU,
    0x00U,
    0x00U, /* Describes the spatial location of the logical channels:: Mono, no spatial location */
    0x00U, /* Index of a string descriptor, describing the name of the first logical channel.  */
    0x00U,
    0x00U, /* bmControls D1..0: Copy Protect Control is not present
              D3..2: Connector Control is not present
              D5..4: Overload Control is not present
              D7..6: Cluster Control is not present
              D9..8: Underflow Control is not present
              D11..10: Overflow Control is not present
              D15..12: Reserved, should set to 0*/
    0x00U, /* Index of a string descriptor, describing the Input Terminal.  */

    0x4AU,                                             /* Size of the descriptor, in bytes  : 6 + (16 + 1) * 4 */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,            /* CS_INTERFACE Descriptor Type   */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_FEATURE_UNIT, /* FEATURE_UNIT descriptor subtype   */
    USB_AUDIO_SPEAKER_CONTROL_FEATURE_UNIT_ID, /* Constant uniquely identifying the Unit within the audio function. This
              value is used in all requests to address this Unit.  */
    USB_AUDIO_SPEAKER_CONTROL_INPUT_TERMINAL_ID, /* ID of the Unit or Terminal to which this Feature Unit is connected.
                                                  */
    0x0FU,
    0x00U,
    0x00U,
    0x00U, /* logic channel 0 bmaControls(0)(0x0000000F):  D1..0: Mute Control is present and host
              programmable D3..2: Volume Control is present and host programmable D5..4: Bass
              Control is not present D7..6: Mid Control is not present D9..8: Treble Control is not
              present D11..10: Graphic Equalizer Control is not present D13..12: Automatic Gain
              Control is not present D15..14: Delay Control is not present D17..16: Bass Control is
              not present D19..18: Loudness Control is not present D21..20: Input Gain Control is
              not present D23..22: Input Gain Pad Control is not present D25..24: Phase Inverter
              Control is not present D27..26: Underflow Control is not present D29..28: Overflow
              Control is not present D31..30: Reserved, should set to 0 */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 1. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 2. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 3. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 4. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 5. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 6. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 7. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 8. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 9. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 10. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 11. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 12. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 13. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 14. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 15. */
    0x00U,
    0x00U,
    0x00U,
    0x00U, /* The Controls bitmap for logical channel 16. */
    0x00U, /* Index of a string descriptor, describing this Feature Unit.   */

    0x0CU,                                                /* Size of the descriptor, in bytes   */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,               /* CS_INTERFACE Descriptor Type  */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_OUTPUT_TERMINAL, /* OUTPUT_TERMINAL descriptor subtype   */
    USB_AUDIO_SPEAKER_CONTROL_OUTPUT_TERMINAL_ID,         /* Constant uniquely identifying the Terminal within the audio
                      function. This value is used in all requests         to address this Terminal.   */
    0x01U,
    0x03U, /* A generic speaker or set of speakers that does not fit under any of the other classifications. */
    0x00U, /* This Output Terminal has no association  */
    USB_AUDIO_SPEAKER_CONTROL_FEATURE_UNIT_ID, /* ID of the Unit or Terminal to which this Terminal is connected.  */
    USB_AUDIO_CONTROL_SPEAKER_CLOCK_SOURCE_ENTITY_ID, /* ID of the Clock Entity to which this Output Terminal is
                                                         connected  */
    0x00U,
    0x00U, /* bmControls:   D1..0: Copy Protect Control is not present
              D3..2: Connector Control is not present
              D5..4: Overload Control is not present
              D7..6: Underflow Control is not present
              D9..8: Overflow Control is not present
              D15..10: Reserved, should set to 0   */
    0x00U, /* Index of a string descriptor, describing the Output Terminal.  */

    /* Audio Class Specific INTERFACE Descriptor, alternative interface 0  */
    USB_DESCRIPTOR_LENGTH_INTERFACE,                 /* Descriptor size is 9 bytes  */
    USB_DESCRIPTOR_TYPE_INTERFACE,                   /* INTERFACE Descriptor Type   */
    USB_AUDIO_RECORDER_STREAM_INTERFACE_INDEX,       /* The number of this interface is 1.  */
    USB_AUDIO_RECORDER_STREAM_INTERFACE_ALTERNATE_0, /* The value used to select the alternate setting for this
                                                        interface is 0   */
    0x00U,                    /* The number of endpoints used by this interface is 0 (excluding endpoint zero)   */
    USB_AUDIO_CLASS,          /* The interface implements the Audio Interface class   */
    USB_SUBCLASS_AUDIOSTREAM, /* The interface implements the AUDIOSTREAMING Subclass   */
    USB_AUDIO_PROTOCOL,       /* The Protocol code is 32   */
    0x00U,                    /* The device doesn't have a string descriptor describing this iInterface  */

    /* Audio Class Specific INTERFACE Descriptor, alternative interface 1 */
    USB_DESCRIPTOR_LENGTH_INTERFACE,                 /* Descriptor size is 9 bytes  */
    USB_DESCRIPTOR_TYPE_INTERFACE,                   /* INTERFACE Descriptor Type  */
    USB_AUDIO_RECORDER_STREAM_INTERFACE_INDEX,       /*The number of this interface is 1.  */
    USB_AUDIO_RECORDER_STREAM_INTERFACE_ALTERNATE_1, /* The value used to select the alternate setting for this
                                                        interface is 1  */
    0x01U,                    /* The number of endpoints used by this interface is 1 (excluding endpoint zero)    */
    USB_AUDIO_CLASS,          /* The interface implements the Audio Interface class   */
    USB_SUBCLASS_AUDIOSTREAM, /* The interface implements the AUDIOSTREAMING Subclass   */
    USB_AUDIO_PROTOCOL,       /* The Protocol code is 32   */
    0x00U,                    /* The device doesn't have a string descriptor describing this iInterface  */

    0x10U,                                             /* Size of the descriptor, in bytes   */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,            /* CS_INTERFACE Descriptor Type  */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_AS_GENERAL, /* AS_GENERAL descriptor subtype   */
    USB_AUDIO_RECORDER_CONTROL_OUTPUT_TERMINAL_ID,     /* The Terminal ID of the terminal to which this interface is
                                                          connected   */
    0x00U,                   /* bmControls : D1..0: Active Alternate Setting Control is not present
                                D3..2: Valid Alternate Settings Control is not present
                                D7..4: Reserved, should set to 0   */
    USB_AUDIO_FORMAT_TYPE_I, /* The format type AudioStreaming interfae using is FORMAT_TYPE_I (0x01)   */
    0x01U,
    0x00U,
    0x00U,
    0x00U, /* The Audio Data Format that can be Used to communicate with this interface */
    0x10,  /* Number of physical channels in the AS Interface audio channel cluster */
    0xFFU,
    0xFFU,
    0x00U,
    0x00U, /* Describes the spatial location of the logical channels: */
    0x00U, /* Index of a string descriptor, describing the name of the first physical channel   */

    0x06U,                                              /* Size of the descriptor, in bytes   */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,             /* CS_INTERFACE Descriptor Type   */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_FORMAT_TYPE, /* FORMAT_TYPE descriptor subtype   */
    USB_AUDIO_FORMAT_TYPE_I, /* The format type AudioStreaming interfae using is FORMAT_TYPE_I (0x01)   */
    AUDIO_OUT_FORMAT_SIZE,   /* The number of bytes occupied by one audio subslot. Can be 1, 2, 3 or 4.  */
    AUDIO_OUT_FORMAT_BITS,   /* The number of effectively used bits from the available bits in an audio subslot   */

    /* ENDPOINT Descriptor */
    USB_AUDIO_STREAMING_ENDP_DESC_SIZE,                 /* Descriptor size is 7 bytes  */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                       /* ENDPOINT Descriptor Type   */
    USB_AUDIO_RECORDER_STREAM_ENDPOINT | (USB_IN << 7), /* This is an IN endpoint with endpoint number 2   */
    0x0D,                                               /* Types -
                                                                                                 Transfer: ISOCHRONOUS
                                                                                                 Sync: Sync
                                                                                                 Usage: Data EP  */
    USB_SHORT_GET_LOW(FS_ISO_IN_ENDP_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_ISO_IN_ENDP_PACKET_SIZE), /* Maximum packet size for this endpoint */
    FS_ISO_IN_ENDP_INTERVAL, /* The polling interval value is every 1 Frames. If Hi-Speed, every 1 uFrames   */

    /* Audio Class Specific ENDPOINT Descriptor  */
    USB_AUDIO_CLASS_SPECIFIC_ENDPOINT_LENGTH, /*  Size of the descriptor, in bytes  */
    USB_AUDIO_STREAM_ENDPOINT_DESCRIPTOR,     /* CS_ENDPOINT Descriptor Type  */
    USB_AUDIO_EP_GENERAL_DESCRIPTOR_SUBTYPE,  /* AUDIO_EP_GENERAL descriptor subtype  */
    0x00U,
    0x00U,
    0x00U,
    0x00U,
    0x00U,

    USB_DESCRIPTOR_LENGTH_INTERFACE,                /* Descriptor size is 9 bytes   */
    USB_DESCRIPTOR_TYPE_INTERFACE,                  /* INTERFACE Descriptor Type   */
    USB_AUDIO_SPEAKER_STREAM_INTERFACE_INDEX,       /* The number of this interface is 1.   */
    USB_AUDIO_SPEAKER_STREAM_INTERFACE_ALTERNATE_0, /* The value used to select the alternate setting for this interface
                                                       is 0   */
    0x00U,                    /* The number of endpoints used by this interface is 0 (excluding endpoint zero)   */
    USB_AUDIO_CLASS,          /* The interface implements the Audio Interface class   */
    USB_SUBCLASS_AUDIOSTREAM, /* The interface implements the AUDIOSTREAMING Subclass   */
    USB_AUDIO_PROTOCOL,       /* The Protocol code is 32   */
    0x00U,                    /* The interface string descriptor index is 0 */

    USB_DESCRIPTOR_LENGTH_INTERFACE,                /* Descriptor size is 9 bytes   */
    USB_DESCRIPTOR_TYPE_INTERFACE,                  /* INTERFACE Descriptor Type   */
    USB_AUDIO_SPEAKER_STREAM_INTERFACE_INDEX,       /* The number of this interface is 1.  */
    USB_AUDIO_SPEAKER_STREAM_INTERFACE_ALTERNATE_1, /* The value used to select the alternate setting for this interface
                                                       is 1   */
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
    0x01U, /* The number of endpoints used by this interface is 1 (excluding endpoint zero)    */
#else
    0x02U, /* The number of endpoints used by this interface is 2 (excluding endpoint zero)    */
#endif
    USB_AUDIO_CLASS,          /* The interface implements the Audio Interface class   */
    USB_SUBCLASS_AUDIOSTREAM, /* The interface implements the AUDIOSTREAMING Subclass   */
    USB_AUDIO_PROTOCOL,       /* The Protocol code is 32   */
    0x00U,                    /* The interface string descriptor index is 2  */

    0x10U,                                             /* Size of the descriptor, in bytes   */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,            /* CS_INTERFACE Descriptor Type  */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_AS_GENERAL, /* AS_GENERAL descriptor subtype   */
    USB_AUDIO_SPEAKER_CONTROL_INPUT_TERMINAL_ID, /* The Terminal ID of the terminal to which this interface is connected
                                                  */
    0x00U,                                       /* bmControls : D1..0: Active Alternate Setting Control is not present
                                                    D3..2: Valid Alternate Settings Control is not present
                                                    D7..4: Reserved, should set to 0   */
    USB_AUDIO_FORMAT_TYPE_I, /* The format type AudioStreaming interfae using is FORMAT_TYPE_I (0x01)   */
    0x01U,
    0x00U,
    0x00U,
    0x00U,                     /* The Audio Data Format that can be Used to communicate with this interface */
    AUDIO_OUT_FORMAT_CHANNELS, /* Number of physical channels in the AS Interface audio channel cluster */
    0xFFU,
    0xFFU,
    0x00U,
    0x00U, /* Describes the spatial location of the logical channels: */
    0x00U, /* Index of a string descriptor, describing the name of the first physical channel   */

    0x06U,                                              /* Size of the descriptor, in bytes   */
    USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE,             /* CS_INTERFACE Descriptor Type   */
    USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_FORMAT_TYPE, /* FORMAT_TYPE descriptor subtype   */
    USB_AUDIO_FORMAT_TYPE_I, /* The format type AudioStreaming interfae using is FORMAT_TYPE_I (0x01)   */
    0x02U,                   /* The number of bytes occupied by one audio subslot. Can be 1, 2, 3 or 4.   */
    0x10U,                   /* The number of effectively used bits from the available bits in an audio subslot   */

#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
    USB_AUDIO_STREAMING_ENDP_DESC_SIZE, /* Descriptor size is 7 bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,       /* ENDPOINT Descriptor Type*/
    USB_AUDIO_SPEAKER_STREAM_ENDPOINT |
        (USB_OUT
         << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT), /* This is an OUT endpoint with endpoint number 2   */
    0x0D,                                                     /* Types -
                                                                  Transfer: ISOCHRONOUS
                                                                  Sync: Async
                                                                  Usage: Data EP  */
    USB_SHORT_GET_LOW(FS_ISO_OUT_ENDP_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_ISO_OUT_ENDP_PACKET_SIZE), /* Maximum packet size for this endpoint */
    FS_ISO_OUT_ENDP_INTERVAL, /* The polling interval value is every 1 Frames. If Hi-Speed, 1 uFrames/NAK */
#else
    USB_AUDIO_STREAMING_ENDP_DESC_SIZE, /* Descriptor size is 7 bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,       /* ENDPOINT Descriptor Type*/
    USB_AUDIO_SPEAKER_STREAM_ENDPOINT |
        (USB_OUT
         << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT), /* This is an OUT endpoint with endpoint number 2   */
    0x05U,                                                    /* Types -
                                                                 Transfer: ISOCHRONOUS
                                                                 Sync: Async
                                                                 Usage: Data EP  */
    USB_SHORT_GET_LOW(FS_ISO_OUT_ENDP_PACKET_SIZE + AUDIO_OUT_FORMAT_CHANNELS * AUDIO_OUT_FORMAT_SIZE),
    USB_SHORT_GET_HIGH(FS_ISO_OUT_ENDP_PACKET_SIZE +
                       AUDIO_OUT_FORMAT_CHANNELS * AUDIO_OUT_FORMAT_SIZE), /* Maximum packet size for this endpoint */
    FS_ISO_OUT_ENDP_INTERVAL, /* The polling interval value is every 1 Frames. If Hi-Speed, 1 uFrames/NAK */
#endif

    /* Audio Class Specific ENDPOINT Descriptor  */
    USB_AUDIO_CLASS_SPECIFIC_ENDPOINT_LENGTH, /* Size of the descriptor, 8 bytes  */
    USB_AUDIO_STREAM_ENDPOINT_DESCRIPTOR,     /* CS_ENDPOINT Descriptor Type   */
    USB_AUDIO_EP_GENERAL_DESCRIPTOR_SUBTYPE,  /* AUDIO_EP_GENERAL descriptor subtype */
    0x00U,
    0x00U,
    0x00U,
    0x00U,
    0x00U,

#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
#else
    /* Endpoint 3 Feedback ENDPOINT */
    USB_AUDIO_STANDARD_AS_ISO_FEEDBACK_ENDPOINT_LENGTH, /* Descriptor size is 7 bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                       /* bDescriptorType */
    USB_AUDIO_SPEAKER_FEEDBACK_ENDPOINT |
        (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT), /* This is an IN endpoint with endpoint number 3 */
    USB_ENDPOINT_ISOCHRONOUS | 0x10,                                 /*  Types -
                                                                         Transfer: ISOCHRONOUS
                                                                         Sync: Async
                                                                         Usage: Feedback EP   */
#if defined(USB_DEVICE_WORKAROUND_AUDIO_20_WINDOWS) && (USB_DEVICE_WORKAROUND_AUDIO_20_WINDOWS > 0)
    0x04, 0x00,                                                      /* wMaxPacketSize */
#else
    0x03, 0x00, /* wMaxPacketSize */
#endif
    FS_ISO_FEEDBACK_ENDP_INTERVAL,                                   /* interval polling(2^x ms) */
#endif
#endif /* AUDIO_CLASS_2_0 */
};

#if (defined(USB_DEVICE_CONFIG_CV_TEST) && (USB_DEVICE_CONFIG_CV_TEST > 0U))
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceQualifierDescriptor[] = {
    USB_DESCRIPTOR_LENGTH_DEVICE_QUALITIER, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_DEVICE_QUALITIER,   /* DEVICE Descriptor Type */
    USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION), /* USB Specification Release Number in
                                                            Binary-Coded Decimal (i.e., 2.10 is 210H). */
    USB_DEVICE_CLASS,                                    /* Class code (assigned by the USB-IF). */
    USB_DEVICE_SUBCLASS,                                 /* Subclass code (assigned by the USB-IF). */
    USB_DEVICE_PROTOCOL,                                 /* Protocol code (assigned by the USB-IF). */
    USB_CONTROL_MAX_PACKET_SIZE,                         /* Maximum packet size for endpoint zero
                                                            (only 8, 16, 32, or 64 are valid) */
    0x00U,                                               /* Number of Other-speed Configurations */
    0x00U,                                               /* Reserved for future use, must be zero */
};
#endif
/* Define string descriptor */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString0[] = {
    2U + 2U,
    USB_DESCRIPTOR_TYPE_STRING,
    0x09U,
    0x04U,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString1[] = {
    2U + 2U * 18U, USB_DESCRIPTOR_TYPE_STRING,
    'N',           0x00U,
    'X',           0x00U,
    'P',           0x00U,
    ' ',           0x00U,
    'S',           0x00U,
    'E',           0x00U,
    'M',           0x00U,
    'I',           0x00U,
    'C',           0x00U,
    'O',           0x00U,
    'N',           0x00U,
    'D',           0x00U,
    'U',           0x00U,
    'C',           0x00U,
    'T',           0x00U,
    'O',           0x00U,
    'R',           0x00U,
    'S',           0x00U,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString2[] = {
    2U + 2U * 23U, USB_DESCRIPTOR_TYPE_STRING,
    'U',           0x00U,
    'S',           0x00U,
    'B',           0x00U,
    ' ',           0x00U,
    'A',           0x00U,
    'U',           0x00U,
    'D',           0x00U,
    'I',           0x00U,
    'O',           0x00U,
    '+',           0x00U,
    'M',           0x00U,
    'U',           0x00U,
    'L',           0x00U,
    'T',           0x00U,
    'I',           0x00U,
    '+',           0x00U,
    'C',           0x00U,
    'H',           0x00U,
    ' ',           0x00U,
    'D',           0x00U,
    'E',           0x00U,
    'M',           0x00U,
    'O',           0x00U,
};

uint32_t g_UsbDeviceStringDescriptorLength[USB_DEVICE_STRING_COUNT] = {
    sizeof(g_UsbDeviceString0),
    sizeof(g_UsbDeviceString1),
    sizeof(g_UsbDeviceString2),
};

uint8_t *g_UsbDeviceStringDescriptorArray[USB_DEVICE_STRING_COUNT] = {
    g_UsbDeviceString0,
    g_UsbDeviceString1,
    g_UsbDeviceString2,
};

usb_language_t g_UsbDeviceLanguage[USB_DEVICE_LANGUAGE_COUNT] = {{
    g_UsbDeviceStringDescriptorArray,
    g_UsbDeviceStringDescriptorLength,
    (uint16_t)0x0409U,
}};

usb_language_list_t g_UsbDeviceLanguageList = {
    g_UsbDeviceString0,
    sizeof(g_UsbDeviceString0),
    g_UsbDeviceLanguage,
    USB_DEVICE_LANGUAGE_COUNT,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Get the descriptor.
 *
 * The function is used to get the descriptor, including the device descriptor, configuration descriptor, and string
 * descriptor, etc.
 *
 * @param handle              The device handle.
 * @param setup               The setup packet buffer address.
 * @param length              It is an OUT parameter, return the data length need to be sent to host.
 * @param buffer              It is an OUT parameter, return the data buffer address.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetDescriptor(usb_device_handle handle,
                                     usb_setup_struct_t *setup,
                                     uint32_t *length,
                                     uint8_t **buffer)
{
    uint8_t descriptorType  = (uint8_t)((setup->wValue & 0xFF00U) >> 8U);
    uint8_t descriptorIndex = (uint8_t)((setup->wValue & 0x00FFU));
    usb_status_t ret        = kStatus_USB_Success;
    if (USB_REQUEST_STANDARD_GET_DESCRIPTOR != setup->bRequest)
    {
        return kStatus_USB_InvalidRequest;
    }
    switch (descriptorType)
    {
        case USB_DESCRIPTOR_TYPE_STRING:
        {
            /* Get string descriptor */
            if (0U == descriptorIndex)
            {
                *buffer = (uint8_t *)g_UsbDeviceLanguageList.languageString;
                *length = g_UsbDeviceLanguageList.stringLength;
            }
            else
            {
                uint8_t languageId    = 0U;
                uint8_t languageIndex = USB_DEVICE_STRING_COUNT;

                for (; languageId < USB_DEVICE_LANGUAGE_COUNT; languageId++)
                {
                    if (setup->wIndex == g_UsbDeviceLanguageList.languageList[languageId].languageId)
                    {
                        if (descriptorIndex < USB_DEVICE_STRING_COUNT)
                        {
                            languageIndex = descriptorIndex;
                        }
                        break;
                    }
                }

                if (USB_DEVICE_STRING_COUNT == languageIndex)
                {
                    return kStatus_USB_InvalidRequest;
                }
                *buffer = (uint8_t *)g_UsbDeviceLanguageList.languageList[languageId].string[languageIndex];
                *length = g_UsbDeviceLanguageList.languageList[languageId].length[languageIndex];
            }
        }
        break;
        case USB_DESCRIPTOR_TYPE_DEVICE:
        {
            /* Get device descriptor */
            *buffer = g_UsbDeviceDescriptor;
            *length = USB_DESCRIPTOR_LENGTH_DEVICE;
        }
        break;
        case USB_DESCRIPTOR_TYPE_CONFIGURE:
        {
            /* Get configuration descriptor */
            *buffer = g_UsbDeviceConfigurationDescriptor;
            *length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL;
        }
        break;
#if (defined(USB_DEVICE_CONFIG_CV_TEST) && (USB_DEVICE_CONFIG_CV_TEST > 0U))
        case USB_DESCRIPTOR_TYPE_DEVICE_QUALITIER:
        {
            /* Get Qualifier descriptor */
            *buffer = g_UsbDeviceQualifierDescriptor;
            *length = USB_DESCRIPTOR_LENGTH_DEVICE_QUALITIER;
        }
        break;
#endif
        default:
            ret = kStatus_USB_InvalidRequest;
            break;
    } /* End Switch */
    return ret;
}

/*!
 * @brief Set the device configuration.
 *
 * The function is used to set the device configuration.
 *
 * @param handle              The device handle.
 * @param configure           The configuration value.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceSetConfigure(usb_device_handle handle, uint8_t configure)
{
    if (!configure)
    {
        return kStatus_USB_Error;
    }
    g_UsbDeviceCurrentConfigure = configure;
    return USB_DeviceCallback(handle, kUSB_DeviceEventSetConfiguration, &configure);
}

/*!
 * @brief Get the device configuration.
 *
 * The function is used to get the device configuration.
 *
 * @param handle The device handle.
 * @param configure It is an OUT parameter, save the current configuration value.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetConfigure(usb_device_handle handle, uint8_t *configure)
{
    *configure = g_UsbDeviceCurrentConfigure;
    return kStatus_USB_Success;
}

/*!
 * @brief Set an interface alternate setting.
 *
 * The function is used to set an interface alternate setting.
 *
 * @param handle The device handle.
 * @param interface The interface index.
 * @param alternateSetting The new alternate setting value.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceSetInterface(usb_device_handle handle, uint8_t interface, uint8_t alternateSetting)
{
    if (interface < USB_DEVICE_INTERFACE_COUNT)
    {
        g_UsbDeviceInterface[interface] = alternateSetting;
        return USB_DeviceCallback(handle, kUSB_DeviceEventSetInterface, &interface);
    }
    return kStatus_USB_InvalidRequest;
}

/*!
 * @brief Get an interface alternate setting.
 *
 * The function is used to get an interface alternate setting.
 *
 * @param handle The device handle.
 * @param interface The interface index.
 * @param alternateSetting It is an OUT parameter, save the new alternate setting value of the interface.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetInterface(usb_device_handle handle, uint8_t interface, uint8_t *alternateSetting)
{
    if (interface < USB_DEVICE_INTERFACE_COUNT)
    {
        *alternateSetting = g_UsbDeviceInterface[interface];
        return kStatus_USB_Success;
    }
    return kStatus_USB_InvalidRequest;
}

/* Due to the difference of HS and FS descriptors, the device descriptors and configurations need to be updated to match
 * current speed.
 * As the default, the device descriptors and configurations are configured by using FS parameters for both EHCI and
 * KHCI.
 * When the EHCI is enabled, the application needs to call this function to update device by using current speed.
 * The updated information includes endpoint max packet size, endpoint interval, etc. */
usb_status_t USB_DeviceSetSpeed(uint8_t speed)
{
    usb_descriptor_union_t *descriptorHead;
    usb_descriptor_union_t *descriptorTail;

    descriptorHead = (usb_descriptor_union_t *)&g_UsbDeviceConfigurationDescriptor[0];
    descriptorTail =
        (usb_descriptor_union_t *)(&g_UsbDeviceConfigurationDescriptor[USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL - 1U]);

    while (descriptorHead < descriptorTail)
    {
        if (descriptorHead->common.bDescriptorType == USB_DESCRIPTOR_TYPE_ENDPOINT)
        {
            if (USB_SPEED_HIGH == speed)
            {
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
                if ((USB_AUDIO_SPEAKER_STREAM_ENDPOINT ==
                     (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                    ((descriptorHead->endpoint.bEndpointAddress >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) ==
                     USB_OUT))
                {
                    descriptorHead->endpoint.bInterval = HS_ISO_OUT_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS((HS_ISO_OUT_ENDP_PACKET_SIZE),
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
#else
                if ((USB_AUDIO_SPEAKER_STREAM_ENDPOINT ==
                     (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                    ((descriptorHead->endpoint.bEndpointAddress >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) ==
                     USB_OUT))
                {
                    descriptorHead->endpoint.bInterval = HS_ISO_OUT_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(
                        (HS_ISO_OUT_ENDP_PACKET_SIZE + AUDIO_OUT_FORMAT_CHANNELS * AUDIO_OUT_FORMAT_SIZE),
                        descriptorHead->endpoint.wMaxPacketSize);
                }
                else if ((USB_AUDIO_SPEAKER_FEEDBACK_ENDPOINT ==
                          (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                         ((descriptorHead->endpoint.bEndpointAddress >>
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) == USB_IN))
                {
                    descriptorHead->endpoint.bInterval = HS_ISO_FEEDBACK_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_ISO_FEEDBACK_ENDP_PACKET_SIZE,
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
#endif
                else if ((USB_AUDIO_RECORDER_STREAM_ENDPOINT ==
                          (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                         ((descriptorHead->endpoint.bEndpointAddress >>
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) == USB_IN))
                {
                    descriptorHead->endpoint.bInterval = HS_ISO_IN_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS((HS_ISO_IN_ENDP_PACKET_SIZE),
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
                else
                {
                }
            }
            else
            {
#if defined(USB_DEVICE_AUDIO_USE_SYNC_MODE) && (USB_DEVICE_AUDIO_USE_SYNC_MODE > 0U)
                if ((USB_AUDIO_SPEAKER_STREAM_ENDPOINT ==
                     (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                    ((descriptorHead->endpoint.bEndpointAddress >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) ==
                     USB_OUT))
                {
                    descriptorHead->endpoint.bInterval = FS_ISO_OUT_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS((FS_ISO_OUT_ENDP_PACKET_SIZE),
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
                else if ((USB_AUDIO_RECORDER_STREAM_ENDPOINT ==
                          (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                         ((descriptorHead->endpoint.bEndpointAddress >>
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) == USB_IN))
                {
                    descriptorHead->endpoint.bInterval = FS_ISO_IN_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS((FS_ISO_IN_ENDP_PACKET_SIZE),
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
#else
                if ((USB_AUDIO_SPEAKER_STREAM_ENDPOINT ==
                     (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                    ((descriptorHead->endpoint.bEndpointAddress >> USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) ==
                     USB_OUT))
                {
                    descriptorHead->endpoint.bInterval = FS_ISO_OUT_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(
                        (FS_ISO_OUT_ENDP_PACKET_SIZE + AUDIO_OUT_FORMAT_CHANNELS * AUDIO_OUT_FORMAT_SIZE),
                        descriptorHead->endpoint.wMaxPacketSize);
                }
                else if ((USB_AUDIO_SPEAKER_FEEDBACK_ENDPOINT ==
                          (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                         ((descriptorHead->endpoint.bEndpointAddress >>
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) == USB_IN))
                {
                    descriptorHead->endpoint.bInterval = FS_ISO_FEEDBACK_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_ISO_FEEDBACK_ENDP_PACKET_SIZE,
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
                else if ((USB_AUDIO_RECORDER_STREAM_ENDPOINT ==
                          (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                         ((descriptorHead->endpoint.bEndpointAddress >>
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) == USB_IN))
                {
                    descriptorHead->endpoint.bInterval = FS_ISO_IN_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(
                        (FS_ISO_IN_ENDP_PACKET_SIZE + AUDIO_IN_FORMAT_CHANNELS * AUDIO_IN_FORMAT_SIZE),
                        descriptorHead->endpoint.wMaxPacketSize);
                }
#endif
                else if ((USB_AUDIO_RECORDER_STREAM_ENDPOINT ==
                          (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) &&
                         ((descriptorHead->endpoint.bEndpointAddress >>
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) == USB_IN))
                {
                    descriptorHead->endpoint.bInterval = FS_ISO_IN_ENDP_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS((FS_ISO_IN_ENDP_PACKET_SIZE),
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
                else
                {
                }
            }
        }
        descriptorHead = (usb_descriptor_union_t *)((uint8_t *)descriptorHead + descriptorHead->common.bLength);
    }

    return kStatus_USB_Success;
}
