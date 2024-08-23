/****************************************************************************
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: MIT
 *****************************************************************************/

#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/return_codes.h>
#include <common/misc.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "vvsensor.h"


CREATE_TRACER( OV5647_INFO , "OV5647: ", INFO,    0);
CREATE_TRACER( OV5647_WARN , "OV5647: ", WARNING, 0);
CREATE_TRACER( OV5647_ERROR, "OV5647: ", ERROR,   1);


#ifdef SUBDEV_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
//#undef TRACE
//#define TRACE(x, ...)
#endif

#define OV5647_ISI_TPG_MAX 4

static const char SensorName[16] = "ov5647";

typedef struct OV5647_Context_s
{
    IsiSensorContext_t  IsiCtx;
    struct vvcam_mode_info_s CurMode;
    IsiSensorAeInfo_t AeInfo;
    IsiSensorIntTime_t IntTime;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    IsiSensorGain_t SensorGain;
    uint32_t minAfps;
    uint64_t AEStartExposure;
} OV5647_Context_t;

static RESULT OV5647_IsiSensorSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    int ret = 0;

    TRACE( OV5647_INFO, "%s: (enter)\n", __func__);
    TRACE( OV5647_INFO, "%s: set power %d\n", __func__,on);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    int32_t power = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_POWER, &power);
    if (ret != 0){
        TRACE(OV5647_ERROR, "%s set power %d error\n", __func__,power);
        return RET_FAILURE;
    }

    TRACE( OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSensorGetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( OV5647_INFO, "%s: (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    if (!pclk)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CLK, pclk);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s get clock error\n", __func__);
        return RET_FAILURE;
    } 
    
    TRACE( OV5647_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);
    TRACE( OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSensorSetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( OV5647_INFO, "%s: (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    if (pclk == NULL)
        return RET_NULL_POINTER;
    
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_CLK, &pclk);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( OV5647_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);

    TRACE( OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiResetSensorIss(IsiSensorHandle_t handle)
{
    int ret = 0;

    TRACE( OV5647_INFO, "%s: (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_RESET, NULL);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s set reset error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiRegisterReadIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        uint32_t * pValue)
{
    int32_t ret = 0;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = 0;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_READ_REG, &sccb_data);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s: read sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    *pValue = sccb_data.data;

    TRACE(OV5647_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiRegisterWriteIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        const uint32_t value)
{
    int ret = 0;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = value;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_WRITE_REG, &sccb_data);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s: write sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    TRACE(OV5647_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_UpdateIsiAEInfo(IsiSensorHandle_t handle)
{
    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    uint32_t exp_line_time = pSensorCtx->CurMode.ae_info.one_line_exp_time_ns;

    IsiSensorAeInfo_t *pAeInfo = &pSensorCtx->AeInfo;
    pAeInfo->oneLineExpTime = (exp_line_time << ISI_EXPO_PARAS_FIX_FRACBITS) / 1000;

    if (pSensorCtx->CurMode.hdr_mode == SENSOR_MODE_LINEAR) {
        pAeInfo->maxIntTime.linearInt =
            pSensorCtx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->minIntTime.linearInt =
            pSensorCtx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->maxAGain.linearGainParas = pSensorCtx->CurMode.ae_info.max_again;
        pAeInfo->minAGain.linearGainParas = pSensorCtx->CurMode.ae_info.min_again;
        pAeInfo->maxDGain.linearGainParas = pSensorCtx->CurMode.ae_info.max_dgain;
        pAeInfo->minDGain.linearGainParas = pSensorCtx->CurMode.ae_info.min_dgain;
    } else {
        switch (pSensorCtx->CurMode.stitching_mode) {
            case SENSOR_STITCHING_DUAL_DCG:
            case SENSOR_STITCHING_3DOL:
            case SENSOR_STITCHING_LINEBYLINE:
                pAeInfo->maxIntTime.triInt.triSIntTime =
                    pSensorCtx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triSIntTime =
                    pSensorCtx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                
                pAeInfo->maxIntTime.triInt.triIntTime =
                    pSensorCtx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triIntTime =
                    pSensorCtx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pSensorCtx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG) {
                    pAeInfo->maxIntTime.triInt.triLIntTime = pAeInfo->maxIntTime.triInt.triIntTime;
                    pAeInfo->minIntTime.triInt.triLIntTime = pAeInfo->minIntTime.triInt.triIntTime;
                } else {
                    pAeInfo->maxIntTime.triInt.triLIntTime =
                        pSensorCtx->CurMode.ae_info.max_longintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.triInt.triLIntTime =
                        pSensorCtx->CurMode.ae_info.min_longintegration_line * pAeInfo->oneLineExpTime;
                }

                pAeInfo->maxAGain.triGainParas.triSGain = pSensorCtx->CurMode.ae_info.max_short_again;
                pAeInfo->minAGain.triGainParas.triSGain = pSensorCtx->CurMode.ae_info.min_short_again;
                pAeInfo->maxDGain.triGainParas.triSGain = pSensorCtx->CurMode.ae_info.max_short_dgain;
                pAeInfo->minDGain.triGainParas.triSGain = pSensorCtx->CurMode.ae_info.min_short_dgain;

                pAeInfo->maxAGain.triGainParas.triGain = pSensorCtx->CurMode.ae_info.max_again;
                pAeInfo->minAGain.triGainParas.triGain = pSensorCtx->CurMode.ae_info.min_again;
                pAeInfo->maxDGain.triGainParas.triGain = pSensorCtx->CurMode.ae_info.max_dgain;
                pAeInfo->minDGain.triGainParas.triGain = pSensorCtx->CurMode.ae_info.min_dgain;

                pAeInfo->maxAGain.triGainParas.triLGain = pSensorCtx->CurMode.ae_info.max_long_again;
                pAeInfo->minAGain.triGainParas.triLGain = pSensorCtx->CurMode.ae_info.min_long_again;
                pAeInfo->maxDGain.triGainParas.triLGain = pSensorCtx->CurMode.ae_info.max_long_dgain;
                pAeInfo->minDGain.triGainParas.triLGain = pSensorCtx->CurMode.ae_info.min_long_dgain;
                break;
            case SENSOR_STITCHING_DUAL_DCG_NOWAIT:
            case SENSOR_STITCHING_16BIT_COMPRESS:
            case SENSOR_STITCHING_L_AND_S:
            case SENSOR_STITCHING_2DOL:
                pAeInfo->maxIntTime.dualInt.dualIntTime =
                    pSensorCtx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.dualInt.dualIntTime =
                    pSensorCtx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pSensorCtx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime = pAeInfo->maxIntTime.dualInt.dualIntTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime = pAeInfo->minIntTime.dualInt.dualIntTime;
                } else {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime =
                        pSensorCtx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime =
                        pSensorCtx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                }
                
                if (pSensorCtx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pSensorCtx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pSensorCtx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pSensorCtx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pSensorCtx->CurMode.ae_info.min_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pSensorCtx->CurMode.ae_info.max_long_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pSensorCtx->CurMode.ae_info.min_long_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pSensorCtx->CurMode.ae_info.max_long_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pSensorCtx->CurMode.ae_info.min_long_dgain;
                } else {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pSensorCtx->CurMode.ae_info.max_short_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pSensorCtx->CurMode.ae_info.min_short_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pSensorCtx->CurMode.ae_info.max_short_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pSensorCtx->CurMode.ae_info.min_short_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pSensorCtx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pSensorCtx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pSensorCtx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pSensorCtx->CurMode.ae_info.min_dgain;
                }
                
                break;
            default:
                break;
        }
    }
    pAeInfo->gainStep = pSensorCtx->CurMode.ae_info.gain_step;
    pAeInfo->currFps  = pSensorCtx->CurMode.ae_info.cur_fps;
    pAeInfo->maxFps   = pSensorCtx->CurMode.ae_info.max_fps;
    pAeInfo->minFps   = pSensorCtx->CurMode.ae_info.min_fps;
    pAeInfo->minAfps  = pSensorCtx->CurMode.ae_info.min_afps;
    pAeInfo->hdrRatio[0] = pSensorCtx->CurMode.ae_info.hdr_ratio.ratio_l_s;
    pAeInfo->hdrRatio[1] = pSensorCtx->CurMode.ae_info.hdr_ratio.ratio_s_vs;

    pAeInfo->intUpdateDlyFrm = pSensorCtx->CurMode.ae_info.int_update_delay_frm;
    pAeInfo->gainUpdateDlyFrm = pSensorCtx->CurMode.ae_info.gain_update_delay_frm;

    if (pSensorCtx->minAfps != 0) {
        pAeInfo->minAfps = pSensorCtx->minAfps;
    } 
    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    memcpy(pMode, &pSensorCtx->CurMode, sizeof(IsiSensorMode_t));

    TRACE(OV5647_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    int ret = 0;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    struct vvcam_mode_info_s sensor_mode;
    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    sensor_mode.index = pMode->index;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s set sensor mode failed", __func__);
        return RET_FAILURE;
    }
    memcpy(&pSensorCtx->CurMode, &sensor_mode, sizeof(struct vvcam_mode_info_s));
    OV5647_UpdateIsiAEInfo(handle);

    TRACE(OV5647_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                                              bool_t on)
{
    int ret = 0;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    uint32_t status = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_STREAM, &status);
    if (ret != 0){
        TRACE(OV5647_ERROR, "%s set sensor stream %d error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV5647_INFO, "%s: set streaming %d\n", __func__, on);
    TRACE(OV5647_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiCreateSensorIss(IsiSensorInstanceConfig_t * pConfig)
{
    RESULT result = RET_SUCCESS;
    OV5647_Context_t *pSensorCtx;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    if (!pConfig || !pConfig->pSensor || !pConfig->HalHandle)
        return RET_NULL_POINTER;

    pSensorCtx = (OV5647_Context_t *) malloc(sizeof(OV5647_Context_t));
    if (!pSensorCtx)
        return RET_OUTOFMEM;

    memset(pSensorCtx, 0, sizeof(OV5647_Context_t));
    pSensorCtx->IsiCtx.HalHandle = pConfig->HalHandle;
    pSensorCtx->IsiCtx.pSensor   = pConfig->pSensor;
    pConfig->hSensor = (IsiSensorHandle_t) pSensorCtx;

    result = OV5647_IsiSensorSetPowerIss(pSensorCtx, BOOL_TRUE);
    if (result != RET_SUCCESS) {
        TRACE(OV5647_ERROR, "%s set power error\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    result = OV5647_IsiSensorGetClkIss(pSensorCtx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(OV5647_ERROR, "%s get clk error\n", __func__);
        return RET_FAILURE;
    }
    clk.status = 1;
    result = OV5647_IsiSensorSetClkIss(pSensorCtx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(OV5647_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }
    result = OV5647_IsiResetSensorIss(pSensorCtx);
    if (result != RET_SUCCESS) {
        TRACE(OV5647_ERROR, "%s retset sensor error\n", __func__);
        return RET_FAILURE;
    }

    IsiSensorMode_t SensorMode;
    SensorMode.index = pConfig->SensorModeIndex;
    result = OV5647_IsiSetSensorModeIss(pSensorCtx, &SensorMode);
    if (result != RET_SUCCESS) {
        TRACE(OV5647_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT OV5647_IsiReleaseSensorIss(IsiSensorHandle_t handle)
{
    TRACE(OV5647_INFO, "%s (enter) \n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    if (pSensorCtx == NULL)
        return (RET_WRONG_HANDLE);

    OV5647_IsiSensorSetStreamingIss(pSensorCtx, BOOL_FALSE);
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    OV5647_IsiSensorGetClkIss(pSensorCtx, &clk);
    clk.status = 0;
    OV5647_IsiSensorSetClkIss(pSensorCtx, &clk);
    OV5647_IsiSensorSetPowerIss(pSensorCtx, BOOL_FALSE);
    free(pSensorCtx);
    pSensorCtx = NULL;

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiHalQuerySensorIss(HalHandle_t HalHandle,
                                          IsiSensorModeInfoArray_t *pSensorMode)
{
    int ret = 0;

    TRACE(OV5647_INFO, "%s (enter) \n", __func__);

    if (HalHandle == NULL || pSensorMode == NULL)
        return RET_NULL_POINTER;

    HalContext_t *pHalCtx = (HalContext_t *)HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_QUERY, pSensorMode);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiQuerySensorIss(IsiSensorHandle_t handle,
                                       IsiSensorModeInfoArray_t *pSensorMode)
{
    RESULT result = RET_SUCCESS;

    TRACE(OV5647_INFO, "%s (enter) \n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    result = OV5647_IsiHalQuerySensorIss(pSensorCtx->IsiCtx.HalHandle,
                                         pSensorMode);
    if (result != RET_SUCCESS)
        TRACE(OV5647_ERROR, "%s: query sensor mode info error!\n", __func__);

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT OV5647_IsiGetCapsIss(IsiSensorHandle_t handle,
                                   IsiSensorCaps_t * pIsiSensorCaps)
{
    RESULT result = RET_SUCCESS;

    TRACE(OV5647_INFO, "%s (enter) \n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    IsiSensorModeInfoArray_t SensorModeInfo;
    memset(&SensorModeInfo, 0, sizeof(IsiSensorModeInfoArray_t));
    result = OV5647_IsiQuerySensorIss(handle, &SensorModeInfo);
    if (result != RET_SUCCESS) {
        TRACE(OV5647_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    pIsiSensorCaps->FieldSelection    = ISI_FIELDSEL_BOTH;
    pIsiSensorCaps->YCSequence        = ISI_YCSEQ_YCBYCR;
    pIsiSensorCaps->Conv422           = ISI_CONV422_NOCOSITED;
    pIsiSensorCaps->HPol              = ISI_HPOL_REFPOS;
    pIsiSensorCaps->VPol              = ISI_VPOL_NEG;
    pIsiSensorCaps->Edge              = ISI_EDGE_RISING;
    pIsiSensorCaps->supportModeNum    = SensorModeInfo.count;
    pIsiSensorCaps->currentMode       = pSensorCtx->CurMode.index;

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT OV5647_IsiSetupSensorIss(IsiSensorHandle_t handle,
                                       const IsiSensorCaps_t *pIsiSensorCaps )
{
    int ret = 0;
    RESULT result = RET_SUCCESS;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    if (pIsiSensorCaps->currentMode != pSensorCtx->CurMode.index) {
        IsiSensorMode_t SensorMode;
        memset(&SensorMode, 0, sizeof(IsiSensorMode_t));
        SensorMode.index = pIsiSensorCaps->currentMode;
        result = OV5647_IsiSetSensorModeIss(handle, &SensorMode);
        if (result != RET_SUCCESS) {
            TRACE(OV5647_ERROR, "%s:set sensor mode %d failed!\n",
                  __func__, SensorMode.index);
            return result;
        }
    }

#ifdef SUBDEV_V4L2
    struct v4l2_subdev_format format;
    memset(&format, 0, sizeof(struct v4l2_subdev_format));
    format.format.width  = pSensorCtx->CurMode.size.bounds_width;
    format.format.height = pSensorCtx->CurMode.size.bounds_height;
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    ret = ioctl(pHalCtx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret != 0){
        TRACE(OV5647_ERROR, "%s: sensor set format error!\n", __func__);
        return RET_FAILURE;
    }
#else
    ret = ioctrl(pHalCtx->sensor_fd, VVSENSORIOC_S_INIT, NULL);
    if (ret != 0){
        TRACE(OV5647_ERROR, "%s: sensor init error!\n", __func__);
        return RET_FAILURE;
    }
#endif

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetSensorRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    int ret = 0;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    if (pValue == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, pValue);
    if (ret != 0) {
        TRACE(OV5647_ERROR, "%s: get chip id error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiCheckSensorConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    uint32_t ChipId = 0;
    result = OV5647_IsiGetSensorRevisionIss(handle, &ChipId);
    if (result != RET_SUCCESS) {
        TRACE(OV5647_ERROR, "%s:get sensor chip id error!\n",__func__);
        return RET_FAILURE;
    }

    if (ChipId != 0x530841) {
        TRACE(OV5647_ERROR,
            "%s:ChipID=0x2770,while read sensor Id=0x%x error!\n",
             __func__, ChipId);
        return RET_FAILURE;
    }

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetAeInfoIss(IsiSensorHandle_t handle,
                                     IsiSensorAeInfo_t *pAeInfo)
{
    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    if (pAeInfo == NULL)
        return RET_NULL_POINTER;

    memcpy(pAeInfo, &pSensorCtx->AeInfo, sizeof(IsiSensorAeInfo_t));

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSetHdrRatioIss(IsiSensorHandle_t handle,
                                       uint8_t hdrRatioNum,
                                       uint32_t HdrRatio[])
{
    int ret = 0;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    struct sensor_hdr_artio_s hdr_ratio;
    if (hdrRatioNum == 2) {
        hdr_ratio.ratio_s_vs = HdrRatio[1];
        hdr_ratio.ratio_l_s = HdrRatio[0];
    }else {
        hdr_ratio.ratio_s_vs = HdrRatio[0];
        hdr_ratio.ratio_l_s = 0;
    }

    if (hdr_ratio.ratio_s_vs == pSensorCtx->CurMode.ae_info.hdr_ratio.ratio_s_vs &&
        hdr_ratio.ratio_l_s == pSensorCtx->CurMode.ae_info.hdr_ratio.ratio_l_s)
        return RET_SUCCESS;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_HDR_RADIO, &hdr_ratio);
    if (ret != 0) {
        TRACE(OV5647_ERROR,"%s: set hdr ratio error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s sensor_mode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(OV5647_ERROR,"%s: get mode info error!\n", __func__);
        return RET_FAILURE;
    }

    memcpy(&pSensorCtx->CurMode, &sensor_mode, sizeof (struct vvcam_mode_info_s));
    OV5647_UpdateIsiAEInfo(handle);

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    memcpy(pIntegrationTime, &pSensorCtx->IntTime, sizeof(IsiSensorIntTime_t));

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}

static RESULT OV5647_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    int ret = 0;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    uint32_t oneLineTime;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    if (pIntegrationTime == NULL)
        return RET_NULL_POINTER;

    oneLineTime =  pSensorCtx->AeInfo.oneLineExpTime;
    pSensorCtx->IntTime.expoFrmType = pIntegrationTime->expoFrmType;

    switch (pIntegrationTime->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            IntLine = (pIntegrationTime->IntegrationTime.linearInt +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pSensorCtx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor linear exp error!\n", __func__);
                    return RET_FAILURE;
                }
               pSensorCtx->IntLine = IntLine;
            }
            TRACE(OV5647_INFO, "%s set linear exp %d \n", __func__,IntLine);
            pSensorCtx->IntTime.IntegrationTime.linearInt =  IntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            IntLine = (pIntegrationTime->IntegrationTime.dualInt.dualIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pSensorCtx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor dual exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pSensorCtx->IntLine = IntLine;
            }

            if (pSensorCtx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                ShortIntLine = (pIntegrationTime->IntegrationTime.dualInt.dualSIntTime +
                               (oneLineTime / 2)) / oneLineTime;
                if (ShortIntLine != pSensorCtx->ShortIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                    if (ret != 0) {
                        TRACE(OV5647_ERROR,"%s:set sensor dual vsexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pSensorCtx->ShortIntLine = ShortIntLine;
                }
            } else {
                ShortIntLine = IntLine;
                pSensorCtx->ShortIntLine = ShortIntLine;
            }
            TRACE(OV5647_INFO, "%s set dual exp %d short_exp %d\n", __func__, IntLine, ShortIntLine);
            pSensorCtx->IntTime.IntegrationTime.dualInt.dualIntTime  = IntLine * oneLineTime;
            pSensorCtx->IntTime.IntegrationTime.dualInt.dualSIntTime = ShortIntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            if (pSensorCtx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                LongIntLine = (pIntegrationTime->IntegrationTime.triInt.triLIntTime +
                        (oneLineTime / 2)) / oneLineTime;
                if (LongIntLine != pSensorCtx->LongIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_EXP, &LongIntLine);
                    if (ret != 0) {
                        TRACE(OV5647_ERROR,"%s:set sensor tri lexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pSensorCtx->LongIntLine = LongIntLine;
                }
            } else {
                LongIntLine = (pIntegrationTime->IntegrationTime.triInt.triIntTime +
                       (oneLineTime / 2)) / oneLineTime;
                pSensorCtx->LongIntLine = LongIntLine;
            }

            IntLine = (pIntegrationTime->IntegrationTime.triInt.triIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pSensorCtx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor tri exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pSensorCtx->IntLine = IntLine;
            }
            
            ShortIntLine = (pIntegrationTime->IntegrationTime.triInt.triSIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (ShortIntLine != pSensorCtx->ShortIntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor tri vsexp error!\n", __func__);
                    return RET_FAILURE;
                }
                pSensorCtx->ShortIntLine = ShortIntLine;
            }
            TRACE(OV5647_INFO, "%s set tri long exp %d exp %d short_exp %d\n", __func__, LongIntLine, IntLine, ShortIntLine);
            pSensorCtx->IntTime.IntegrationTime.triInt.triLIntTime = LongIntLine * oneLineTime;
            pSensorCtx->IntTime.IntegrationTime.triInt.triIntTime = IntLine * oneLineTime;
            pSensorCtx->IntTime.IntegrationTime.triInt.triSIntTime = ShortIntLine * oneLineTime;
            break;
        default:
            return RET_FAILURE;
            break;
    }
    
    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    if (pGain == NULL)
        return RET_NULL_POINTER;
    memcpy(pGain, &pSensorCtx->SensorGain, sizeof(IsiSensorGain_t));

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    int ret = 0;
    uint32_t LongGain;
    uint32_t Gain;
    uint32_t ShortGain;

    TRACE(OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    if (pGain == NULL)
        return RET_NULL_POINTER;

    pSensorCtx->SensorGain.expoFrmType = pGain->expoFrmType;
    switch (pGain->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            Gain = pGain->gain.linearGainParas;
            if (pSensorCtx->SensorGain.gain.linearGainParas != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor linear gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            pSensorCtx->SensorGain.gain.linearGainParas = pGain->gain.linearGainParas;
            TRACE(OV5647_INFO, "%s set linear gain %d\n", __func__,pGain->gain.linearGainParas);
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            Gain = pGain->gain.dualGainParas.dualGain;
            if (pSensorCtx->SensorGain.gain.dualGainParas.dualGain != Gain) {
                if (pSensorCtx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &Gain);
                }
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor dual gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.dualGainParas.dualSGain;
            if (pSensorCtx->SensorGain.gain.dualGainParas.dualSGain != ShortGain) {
                if (pSensorCtx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &ShortGain);
                }
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor dual vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(OV5647_INFO,"%s:set gain%d short gain %d!\n", __func__,Gain,ShortGain);
            pSensorCtx->SensorGain.gain.dualGainParas.dualGain = Gain;
            pSensorCtx->SensorGain.gain.dualGainParas.dualSGain = ShortGain;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            LongGain = pGain->gain.triGainParas.triLGain;
            if (pSensorCtx->SensorGain.gain.triGainParas.triLGain != LongGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &LongGain);
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            Gain = pGain->gain.triGainParas.triGain;
            if (pSensorCtx->SensorGain.gain.triGainParas.triGain != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.triGainParas.triSGain;
            if (pSensorCtx->SensorGain.gain.triGainParas.triSGain != ShortGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                if (ret != 0) {
                    TRACE(OV5647_ERROR,"%s:set sensor tri vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(OV5647_INFO,"%s:set long gain %d gain%d short gain %d!\n", __func__, LongGain, Gain, ShortGain);
            pSensorCtx->SensorGain.gain.triGainParas.triLGain = LongGain;
            pSensorCtx->SensorGain.gain.triGainParas.triGain = Gain;
            pSensorCtx->SensorGain.gain.triGainParas.triSGain = ShortGain;
            break;
        default:
            return RET_FAILURE;
            break;
    }

    TRACE(OV5647_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetSensorFpsIss(IsiSensorHandle_t handle, uint32_t * pfps)
{
    TRACE(OV5647_INFO, "%s: (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    if (pfps == NULL)
        return RET_NULL_POINTER;

    *pfps = pSensorCtx->CurMode.ae_info.cur_fps;

    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSetSensorFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    int ret = 0;

    TRACE(OV5647_INFO, "%s: (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_FPS, &fps);
    if (ret != 0) {
        TRACE(OV5647_ERROR,"%s:set sensor fps error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s SensorMode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &SensorMode);
    if (ret != 0) {
        TRACE(OV5647_ERROR,"%s:get sensor mode error!\n", __func__);
        return RET_FAILURE;
    }
    memcpy(&pSensorCtx->CurMode, &SensorMode, sizeof(struct vvcam_mode_info_s));
    OV5647_UpdateIsiAEInfo(handle);

    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSetSensorAfpsLimitsIss(IsiSensorHandle_t handle, uint32_t minAfps)
{
    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    TRACE(OV5647_INFO, "%s: (enter)\n", __func__);

    if ((minAfps > pSensorCtx->CurMode.ae_info.max_fps) ||
        (minAfps < pSensorCtx->CurMode.ae_info.min_fps))
        return RET_FAILURE;
    pSensorCtx->minAfps = minAfps;
    pSensorCtx->CurMode.ae_info.min_afps = minAfps;

    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetSensorIspStatusIss(IsiSensorHandle_t handle,
                               IsiSensorIspStatus_t *pSensorIspStatus)
{
    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    TRACE(OV5647_INFO, "%s: (enter)\n", __func__);

    if (pSensorCtx->CurMode.hdr_mode == SENSOR_MODE_HDR_NATIVE) {
        pSensorIspStatus->useSensorAWB = true;
        pSensorIspStatus->useSensorBLC = true;
    } else {
        pSensorIspStatus->useSensorAWB = false;
        pSensorIspStatus->useSensorBLC = false;
    }

    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

#ifndef ISI_LITE

static RESULT OV5647_IsiSensorGetCompressCurveIss(IsiSensorHandle_t handle, IsiSensorCompressCurve_t *pCompressCurve)
{
    int i = 0;
    TRACE(OV5647_INFO, "%s: (enter)\n", __func__);

    if (pCompressCurve == NULL)
        return RET_NULL_POINTER;

    if ((pCompressCurve->x_bit == 16) && (pCompressCurve->y_bit == 12)) {

        uint8_t compress_px[64] = {10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
        uint32_t compress_y_data[65] = {
                    0   ,390 ,670 ,894 ,1096,1254,1378,1493,1592,1683,1769,1849,1926,1999,2069,2136,
                    2200,2262,2322,2380,2436,2491,2544,2596,2646,2696,2744,2791,2837,2882,2926,2969,
                    3012,3054,3095,3135,3175,3214,3252,3290,3327,3364,3400,3436,3471,3506,3540,3574,
                    3608,3641,3674,3706,3738,3769,3801,3832,3862,3892,3922,3952,3981,4010,4039,4068,4095};
        pCompressCurve->compress_x_data[0] = 0;
        pCompressCurve->compress_y_data[0] = 0;
        for (i= 1; i < 65; i++) {
            pCompressCurve->compress_px[i-1] = compress_px[i-1];
            pCompressCurve->compress_x_data[i] = pCompressCurve->compress_x_data[i-1] + (1 << compress_px[i-1]);
            pCompressCurve->compress_y_data[i] = compress_y_data[i];
        }

    } else if ((pCompressCurve->x_bit == 20) && (pCompressCurve->y_bit == 12)) {
        return RET_FAILURE;
    } else {
        return RET_FAILURE;
    }

    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiSetTestPatternIss(IsiSensorHandle_t handle,
                                       IsiSensorTpgMode_e  tpgMode)
{
    int32_t ret = 0;

    TRACE( OV5647_INFO, "%s (enter)\n", __func__);

    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pSensorCtx->IsiCtx.HalHandle;

    struct sensor_test_pattern_s TestPattern;
    if (tpgMode == ISI_TPG_DISABLE) {
        TestPattern.enable = 0;
        TestPattern.pattern = 0;
    } else {
        TestPattern.enable = 1;
        TestPattern.pattern = (uint32_t)tpgMode - 1;
    }

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_TEST_PATTERN, &TestPattern);
    if (ret != 0)
    {
        TRACE(OV5647_ERROR, "%s: set test pattern enable[%d] mode[%d] error, max mode[%d]\n",
            __func__, TestPattern.enable, (uint32_t)tpgMode - 1, OV5647_ISI_TPG_MAX);
        return RET_FAILURE;
    }

    TRACE(OV5647_INFO, "%s: test pattern enable[%d] mode[%d]\n", __func__, TestPattern.enable, TestPattern.pattern);

    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV5647_IsiFocusSetupIss(IsiSensorHandle_t handle)
{
    TRACE(OV5647_INFO, "%s (enter)\n", __func__);
    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV5647_IsiFocusReleaseIss(IsiSensorHandle_t handle)
{
    TRACE(OV5647_INFO, "%s (enter)\n", __func__);
    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV5647_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE(OV5647_INFO, "%s (enter)\n", __func__);
    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV5647_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE(OV5647_INFO, "%s (enter)\n", __func__);
    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetFocusCalibrateIss(IsiSensorHandle_t handle, IsiFoucsCalibAttr_t *pFocusCalib)
{
    TRACE(OV5647_INFO, "%s (enter)\n", __func__);
    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV5647_IsiGetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t *pExposure)
{
    TRACE(OV5647_INFO, "%s (enter)\n", __func__);
    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    if (pSensorCtx->AEStartExposure == 0) {
        pSensorCtx->AEStartExposure =
            (uint64_t)pSensorCtx->CurMode.ae_info.start_exposure *
            pSensorCtx->CurMode.ae_info.one_line_exp_time_ns / 1000;
           
    }
    *pExposure =  pSensorCtx->AEStartExposure;
    TRACE(OV5647_INFO, "%s:get start exposure %d\n", __func__, pSensorCtx->AEStartExposure);

    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV5647_IsiSetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t exposure)
{
    TRACE( OV5647_INFO, "%s (enter)\n", __func__);
    OV5647_Context_t *pSensorCtx = (OV5647_Context_t *) handle;

    pSensorCtx->AEStartExposure = exposure;
    TRACE(OV5647_INFO, "set start exposure %d\n", __func__,pSensorCtx->AEStartExposure);
    TRACE(OV5647_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

#endif

RESULT OV5647_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    TRACE( OV5647_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor == NULL)
        return RET_NULL_POINTER;
    memset(pIsiSensor, 0, sizeof(IsiSensor_t));

    pIsiSensor->pszName                         = SensorName;
    pIsiSensor->pIsiSensorSetPowerIss           = OV5647_IsiSensorSetPowerIss;
    pIsiSensor->pIsiCreateSensorIss             = OV5647_IsiCreateSensorIss;
    pIsiSensor->pIsiReleaseSensorIss            = OV5647_IsiReleaseSensorIss;
    pIsiSensor->pIsiRegisterReadIss             = OV5647_IsiRegisterReadIss;
    pIsiSensor->pIsiRegisterWriteIss            = OV5647_IsiRegisterWriteIss;
    pIsiSensor->pIsiGetSensorModeIss            = OV5647_IsiGetSensorModeIss;
    pIsiSensor->pIsiSetSensorModeIss            = OV5647_IsiSetSensorModeIss;
    pIsiSensor->pIsiQuerySensorIss              = OV5647_IsiQuerySensorIss;
    pIsiSensor->pIsiGetCapsIss                  = OV5647_IsiGetCapsIss;
    pIsiSensor->pIsiSetupSensorIss              = OV5647_IsiSetupSensorIss;
    pIsiSensor->pIsiGetSensorRevisionIss        = OV5647_IsiGetSensorRevisionIss;
    pIsiSensor->pIsiCheckSensorConnectionIss    = OV5647_IsiCheckSensorConnectionIss;
    pIsiSensor->pIsiSensorSetStreamingIss       = OV5647_IsiSensorSetStreamingIss;
    pIsiSensor->pIsiGetAeInfoIss                = OV5647_IsiGetAeInfoIss;
    pIsiSensor->pIsiSetHdrRatioIss              = OV5647_IsiSetHdrRatioIss;
    pIsiSensor->pIsiGetIntegrationTimeIss       = OV5647_IsiGetIntegrationTimeIss;
    pIsiSensor->pIsiSetIntegrationTimeIss       = OV5647_IsiSetIntegrationTimeIss;
    pIsiSensor->pIsiGetGainIss                  = OV5647_IsiGetGainIss;
    pIsiSensor->pIsiSetGainIss                  = OV5647_IsiSetGainIss;
    pIsiSensor->pIsiGetSensorFpsIss             = OV5647_IsiGetSensorFpsIss;
    pIsiSensor->pIsiSetSensorFpsIss             = OV5647_IsiSetSensorFpsIss;
    pIsiSensor->pIsiSetSensorAfpsLimitsIss      = OV5647_IsiSetSensorAfpsLimitsIss;
    pIsiSensor->pIsiGetSensorIspStatusIss       = OV5647_IsiGetSensorIspStatusIss;
#ifndef ISI_LITE
    pIsiSensor->pIsiSensorGetCompressCurveIss    = OV5647_IsiSensorGetCompressCurveIss;
    pIsiSensor->pIsiActivateTestPatternIss       = OV5647_IsiSetTestPatternIss;
    pIsiSensor->pIsiFocusSetupIss                = OV5647_IsiFocusSetupIss;
    pIsiSensor->pIsiFocusReleaseIss              = OV5647_IsiFocusReleaseIss;
    pIsiSensor->pIsiFocusSetIss                  = OV5647_IsiFocusSetIss;
    pIsiSensor->pIsiFocusGetIss                  = OV5647_IsiFocusGetIss;
    pIsiSensor->pIsiGetFocusCalibrateIss         = OV5647_IsiGetFocusCalibrateIss;
    pIsiSensor->pIsiSetAeStartExposureIss        = OV5647_IsiSetAeStartExposureIs;
    pIsiSensor->pIsiGetAeStartExposureIss        = OV5647_IsiGetAeStartExposureIs;
#endif
    TRACE( OV5647_INFO, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x5647,
    .pIsiHalQuerySensor = OV5647_IsiHalQuerySensorIss,
    .pfIsiGetSensorIss = OV5647_IsiGetSensorIss,
};
