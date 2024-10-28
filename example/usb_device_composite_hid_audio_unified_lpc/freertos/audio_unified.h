/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016,2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __USB_AUDIO_H__
#define __USB_AUDIO_H__ 1

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*audio data buffer depth*/
#define AUDIO_BUFFER_UPPER_LIMIT(x) (((x)*5) / 8)
#define AUDIO_BUFFER_LOWER_LIMIT(x) (((x)*3) / 8)

#define TSAMFREQ2BYTES(f)     (f & 0xFFU), ((f >> 8U) & 0xFFU), ((f >> 16U) & 0xFFU)
#define TSAMFREQ2BYTESHS(f)   (f & 0xFFU), ((f >> 8U) & 0xFFU), ((f >> 16U) & 0xFFU), ((f >> 24U) & 0xFFU)
#define AUDIO_ADJUST_MIN_STEP (0x10)

#define AUDIO_PLAY_TRANSFER_SIZE FS_ISO_OUT_ENDP_PACKET_SIZE

#define MUTE_CODEC_TASK    (1UL << 0U)
#define UNMUTE_CODEC_TASK  (1UL << 1U)
#define VOLUME_CHANGE_TASK (1UL << 2U)

typedef struct _usb_audio_composite_struct
{
    usb_device_handle deviceHandle;    /* USB device handle.                   */
    class_handle_t audioSpeakerHandle; /* USB AUDIO GENERATOR class handle.    */
    class_handle_t audioRecorderHandle;
    uint8_t copyProtect;
    uint8_t curSpeakerMute;
    uint8_t curMicrophoneMute;
    uint8_t curSpeakerVolume[2];
    uint8_t curMicrophoneVolume[2];
    uint8_t minVolume[2];
    uint8_t maxVolume[2];
    uint8_t resVolume[2];
    uint8_t curBass;
    uint8_t minBass;
    uint8_t maxBass;
    uint8_t resBass;
    uint8_t curMid;
    uint8_t minMid;
    uint8_t maxMid;
    uint8_t resMid;
    uint8_t curTreble;
    uint8_t minTreble;
    uint8_t maxTreble;
    uint8_t resTreble;
    uint8_t curAutomaticGain;
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
    uint8_t curDelay[4];
    uint8_t minDelay[4];
    uint8_t maxDelay[4];
    uint8_t resDelay[4];
#else
    uint8_t curDelay[2];
    uint8_t minDelay[2];
    uint8_t maxDelay[2];
    uint8_t resDelay[2];
#endif
    uint8_t curLoudness;
    uint8_t curSamplingFrequency[3];
    uint8_t minSamplingFrequency[3];
    uint8_t maxSamplingFrequency[3];
    uint8_t resSamplingFrequency[3];
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
    uint8_t curSpeakerMute20;
    uint8_t curMicrophoneMute20;
    uint8_t curClockValid;
    uint8_t curSpeakerVolume20[2];
    uint8_t curMicrophoneVolume20[2];
    uint32_t curSpeakerSampleFrequency;
    uint32_t curRecorderSampleFrequency;
    usb_device_control_range_layout3_struct_t speakerFreqControlRange;
    usb_device_control_range_layout3_struct_t recorderFreqControlRange;
    usb_device_control_range_layout2_struct_t volumeControlRange;
#endif
    uint8_t currentConfiguration;
    uint8_t currentInterfaceAlternateSetting[USB_AUDIO_COMPOSITE_INTERFACE_COUNT];
    uint8_t speed;
    uint8_t attach;
    volatile uint8_t startPlayFlag;
    volatile uint8_t startPlayHalfFull;
    volatile uint8_t startRec;
    volatile uint8_t startRecHalfFull;
    volatile uint32_t tdReadNumberPlay;
    volatile uint32_t tdWriteNumberPlay;
    volatile uint32_t audioSpeakerReadDataCount[2];
    volatile uint32_t audioSpeakerWriteDataCount[2];
    volatile uint32_t tdReadNumberRec;
    volatile uint32_t tdWriteNumberRec;
    volatile uint32_t audioSendCount[2];
    volatile uint32_t usbRecvCount;
    volatile uint32_t audioSendTimes;
    volatile uint32_t usbRecvTimes;
    volatile uint32_t audioRecvCount;
    volatile uint32_t usbSendTimes;
    volatile uint32_t speakerIntervalCount;
    volatile uint32_t pllAdjustIntervalCount;
    volatile uint32_t recorderIntervalCount;
    volatile uint32_t speakerReservedSpace;
    volatile uint32_t recorderReservedSpace;
    volatile uint32_t timesFeedbackCalculate;
    volatile uint32_t speakerDetachOrNoInput;
    volatile uint32_t codecSpeakerTask;
    volatile uint32_t codecMicrophoneTask;
    volatile uint32_t curAudioPllFrac;
    volatile uint32_t audioPllTicksPrev;
    volatile int32_t audioPllTicksDiff;
    volatile int32_t audioPllTicksEma;
    volatile int32_t audioPllTickEmaFrac;
    volatile int32_t audioPllTickBasedPrecision;
    volatile uint8_t stopDataLengthAudioSpeakerAdjust;
    volatile uint8_t stopDataLengthAudioRecorderAdjust;
} usb_audio_composite_struct_t;

#endif /* __USB_AUDIO_GENERATOR_H__ */
