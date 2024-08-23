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

CREATE_TRACER( IMX219_INFO , "IMX219: ", INFO,    0);
CREATE_TRACER( IMX219_WARN , "IMX219: ", WARNING, 0);
CREATE_TRACER( IMX219_ERROR, "IMX219: ", ERROR,   1);

#ifdef SUBDEV_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
//#undef TRACE
//#define TRACE(x, ...)
//#define TRACE(a,...) (printf(__VA_ARGS__))

#endif

static const char SensorName[16] = "imx219";

typedef struct IMX219_Context_s
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
} IMX219_Context_t;

static RESULT IMX219_IsiSensorSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    int ret = 0;

    TRACE( IMX219_INFO, "%s: (enter)\n", __func__);
    TRACE( IMX219_INFO, "%s: set power %d\n", __func__,on);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    int32_t power = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_POWER, &power);
    if (ret != 0){
        TRACE(IMX219_ERROR, "%s set power %d error\n", __func__,power);
        return RET_FAILURE;
    }

    TRACE( IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSensorGetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( IMX219_INFO, "%s: (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (!pclk)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CLK, pclk);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s get clock error\n", __func__);
        return RET_FAILURE;
    } 
    
    TRACE( IMX219_INFO, "%s: status:%d sensor_mclk:%ld csi_max_pixel_clk:%ld\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);
    TRACE( IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSensorSetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( IMX219_INFO, "%s: (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pclk == NULL)
        return RET_NULL_POINTER;
    
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_CLK, &pclk);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( IMX219_INFO, "%s: status:%d sensor_mclk:%ld csi_max_pixel_clk:%ld\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);

    TRACE( IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiResetSensorIss(IsiSensorHandle_t handle)
{
    int ret = 0;

    TRACE( IMX219_INFO, "%s: (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_RESET, NULL);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s set reset error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiRegisterReadIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        uint32_t * pValue)
{
    int32_t ret = 0;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = 0;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_READ_REG, &sccb_data);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s: read sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    *pValue = sccb_data.data;

    TRACE(IMX219_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiRegisterWriteIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        const uint32_t value)
{
    int ret = 0;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = value;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_WRITE_REG, &sccb_data);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s: write sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    TRACE(IMX219_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_UpdateIsiAEInfo(IsiSensorHandle_t handle)
{
    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    uint32_t exp_line_time = pIMX219Ctx->CurMode.ae_info.one_line_exp_time_ns;

    IsiSensorAeInfo_t *pAeInfo = &pIMX219Ctx->AeInfo;
    pAeInfo->oneLineExpTime = (exp_line_time << ISI_EXPO_PARAS_FIX_FRACBITS) / 1000;

    if (pIMX219Ctx->CurMode.hdr_mode == SENSOR_MODE_LINEAR) {
        pAeInfo->maxIntTime.linearInt =
            pIMX219Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->minIntTime.linearInt =
            pIMX219Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->maxAGain.linearGainParas = pIMX219Ctx->CurMode.ae_info.max_again;
        pAeInfo->minAGain.linearGainParas = pIMX219Ctx->CurMode.ae_info.min_again;
        pAeInfo->maxDGain.linearGainParas = pIMX219Ctx->CurMode.ae_info.max_dgain;
        pAeInfo->minDGain.linearGainParas = pIMX219Ctx->CurMode.ae_info.min_dgain;
    } else {
        switch (pIMX219Ctx->CurMode.stitching_mode) {
            case SENSOR_STITCHING_DUAL_DCG:
            case SENSOR_STITCHING_3DOL:
            case SENSOR_STITCHING_LINEBYLINE:
                pAeInfo->maxIntTime.triInt.triSIntTime =
                    pIMX219Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triSIntTime =
                    pIMX219Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                
                pAeInfo->maxIntTime.triInt.triIntTime =
                    pIMX219Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triIntTime =
                    pIMX219Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pIMX219Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG) {
                    pAeInfo->maxIntTime.triInt.triLIntTime = pAeInfo->maxIntTime.triInt.triIntTime;
                    pAeInfo->minIntTime.triInt.triLIntTime = pAeInfo->minIntTime.triInt.triIntTime;
                } else {
                    pAeInfo->maxIntTime.triInt.triLIntTime =
                        pIMX219Ctx->CurMode.ae_info.max_longintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.triInt.triLIntTime =
                        pIMX219Ctx->CurMode.ae_info.min_longintegration_line * pAeInfo->oneLineExpTime;
                }

                pAeInfo->maxAGain.triGainParas.triSGain = pIMX219Ctx->CurMode.ae_info.max_short_again;
                pAeInfo->minAGain.triGainParas.triSGain = pIMX219Ctx->CurMode.ae_info.min_short_again;
                pAeInfo->maxDGain.triGainParas.triSGain = pIMX219Ctx->CurMode.ae_info.max_short_dgain;
                pAeInfo->minDGain.triGainParas.triSGain = pIMX219Ctx->CurMode.ae_info.min_short_dgain;

                pAeInfo->maxAGain.triGainParas.triGain = pIMX219Ctx->CurMode.ae_info.max_again;
                pAeInfo->minAGain.triGainParas.triGain = pIMX219Ctx->CurMode.ae_info.min_again;
                pAeInfo->maxDGain.triGainParas.triGain = pIMX219Ctx->CurMode.ae_info.max_dgain;
                pAeInfo->minDGain.triGainParas.triGain = pIMX219Ctx->CurMode.ae_info.min_dgain;

                pAeInfo->maxAGain.triGainParas.triLGain = pIMX219Ctx->CurMode.ae_info.max_long_again;
                pAeInfo->minAGain.triGainParas.triLGain = pIMX219Ctx->CurMode.ae_info.min_long_again;
                pAeInfo->maxDGain.triGainParas.triLGain = pIMX219Ctx->CurMode.ae_info.max_long_dgain;
                pAeInfo->minDGain.triGainParas.triLGain = pIMX219Ctx->CurMode.ae_info.min_long_dgain;
                break;
            case SENSOR_STITCHING_DUAL_DCG_NOWAIT:
            case SENSOR_STITCHING_16BIT_COMPRESS:
            case SENSOR_STITCHING_L_AND_S:
            case SENSOR_STITCHING_2DOL:
                pAeInfo->maxIntTime.dualInt.dualIntTime =
                    pIMX219Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.dualInt.dualIntTime =
                    pIMX219Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pIMX219Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime = pAeInfo->maxIntTime.dualInt.dualIntTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime = pAeInfo->minIntTime.dualInt.dualIntTime;
                } else {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime =
                        pIMX219Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime =
                        pIMX219Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                }
                
                if (pIMX219Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pIMX219Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pIMX219Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pIMX219Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pIMX219Ctx->CurMode.ae_info.min_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pIMX219Ctx->CurMode.ae_info.max_long_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pIMX219Ctx->CurMode.ae_info.min_long_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pIMX219Ctx->CurMode.ae_info.max_long_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pIMX219Ctx->CurMode.ae_info.min_long_dgain;
                } else {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pIMX219Ctx->CurMode.ae_info.max_short_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pIMX219Ctx->CurMode.ae_info.min_short_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pIMX219Ctx->CurMode.ae_info.max_short_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pIMX219Ctx->CurMode.ae_info.min_short_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pIMX219Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pIMX219Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pIMX219Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pIMX219Ctx->CurMode.ae_info.min_dgain;
                }
                
                break;
            default:
                break;
        }
    }
    pAeInfo->gainStep = pIMX219Ctx->CurMode.ae_info.gain_step;
    pAeInfo->currFps  = pIMX219Ctx->CurMode.ae_info.cur_fps;
    pAeInfo->maxFps   = pIMX219Ctx->CurMode.ae_info.max_fps;
    pAeInfo->minFps   = pIMX219Ctx->CurMode.ae_info.min_fps;
    pAeInfo->minAfps  = pIMX219Ctx->CurMode.ae_info.min_afps;
    pAeInfo->hdrRatio[0] = pIMX219Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s;
    pAeInfo->hdrRatio[1] = pIMX219Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs;

    pAeInfo->intUpdateDlyFrm = pIMX219Ctx->CurMode.ae_info.int_update_delay_frm;
    pAeInfo->gainUpdateDlyFrm = pIMX219Ctx->CurMode.ae_info.gain_update_delay_frm;

    if (pIMX219Ctx->minAfps != 0) {
        pAeInfo->minAfps = pIMX219Ctx->minAfps;
    } 
    return RET_SUCCESS;
}

static RESULT IMX219_IsiGetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    memcpy(pMode, &pIMX219Ctx->CurMode, sizeof(IsiSensorMode_t));

    TRACE(IMX219_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    int ret = 0;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    struct vvcam_mode_info_s sensor_mode;
    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    sensor_mode.index = pMode->index;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s set sensor mode failed", __func__);
        return RET_FAILURE;
    }
    memcpy(&pIMX219Ctx->CurMode, &sensor_mode, sizeof(struct vvcam_mode_info_s));
    IMX219_UpdateIsiAEInfo(handle);

    TRACE(IMX219_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                                              bool_t on)
{
    int ret = 0;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    uint32_t status = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_STREAM, &status);
    if (ret != 0){
        TRACE(IMX219_ERROR, "%s set sensor stream %d error\n", __func__,ret);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s: set streaming %d\n", __func__, on);
    TRACE(IMX219_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiCreateSensorIss(IsiSensorInstanceConfig_t * pConfig)
{
    RESULT result = RET_SUCCESS;
    IMX219_Context_t *pIMX219Ctx;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    if (!pConfig || !pConfig->pSensor || !pConfig->HalHandle)
        return RET_NULL_POINTER;

    pIMX219Ctx = (IMX219_Context_t *) malloc(sizeof(IMX219_Context_t));
    if (!pIMX219Ctx)
        return RET_OUTOFMEM;

    memset(pIMX219Ctx, 0, sizeof(IMX219_Context_t));
    pIMX219Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX219Ctx->IsiCtx.pSensor   = pConfig->pSensor;
    pConfig->hSensor = (IsiSensorHandle_t) pIMX219Ctx;

    result = IMX219_IsiSensorSetPowerIss(pIMX219Ctx, BOOL_TRUE);
    if (result != RET_SUCCESS) {
        TRACE(IMX219_ERROR, "%s set power error\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    result = IMX219_IsiSensorGetClkIss(pIMX219Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(IMX219_ERROR, "%s get clk error\n", __func__);
        return RET_FAILURE;
    }
    clk.status = 1;
    result = IMX219_IsiSensorSetClkIss(pIMX219Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(IMX219_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }
    result = IMX219_IsiResetSensorIss(pIMX219Ctx);
    if (result != RET_SUCCESS) {
        TRACE(IMX219_ERROR, "%s retset sensor error\n", __func__);
        return RET_FAILURE;
    }

    IsiSensorMode_t SensorMode;
    SensorMode.index = pConfig->SensorModeIndex;
    result = IMX219_IsiSetSensorModeIss(pIMX219Ctx, &SensorMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX219_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX219_IsiReleaseSensorIss(IsiSensorHandle_t handle)
{
    TRACE(IMX219_INFO, "%s (enter) \n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    if (pIMX219Ctx == NULL)
        return (RET_WRONG_HANDLE);

    IMX219_IsiSensorSetStreamingIss(pIMX219Ctx, BOOL_FALSE);
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    IMX219_IsiSensorGetClkIss(pIMX219Ctx, &clk);
    clk.status = 0;
    IMX219_IsiSensorSetClkIss(pIMX219Ctx, &clk);
    IMX219_IsiSensorSetPowerIss(pIMX219Ctx, BOOL_FALSE);
    free(pIMX219Ctx);
    pIMX219Ctx = NULL;

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiHalQuerySensorIss(HalHandle_t HalHandle,
                                          IsiSensorModeInfoArray_t *pSensorMode)
{
    int ret = 0;

    TRACE(IMX219_INFO, "%s (enter) \n", __func__);
	fprintf(stderr,  "%s (enter) \n", __func__);
    if (HalHandle == NULL || pSensorMode == NULL)
        return RET_NULL_POINTER;

    HalContext_t *pHalCtx = (HalContext_t *)HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_QUERY, pSensorMode);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiQuerySensorIss(IsiSensorHandle_t handle,
                                       IsiSensorModeInfoArray_t *pSensorMode)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX219_INFO, "%s (enter) \n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    result = IMX219_IsiHalQuerySensorIss(pIMX219Ctx->IsiCtx.HalHandle,
                                         pSensorMode);
    if (result != RET_SUCCESS)
        TRACE(IMX219_ERROR, "%s: query sensor mode info error!\n", __func__);

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX219_IsiGetCapsIss(IsiSensorHandle_t handle,
                                   IsiSensorCaps_t * pIsiSensorCaps)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX219_INFO, "%s (enter) \n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    IsiSensorModeInfoArray_t SensorModeInfo;
    memset(&SensorModeInfo, 0, sizeof(IsiSensorModeInfoArray_t));
    result = IMX219_IsiQuerySensorIss(handle, &SensorModeInfo);
    if (result != RET_SUCCESS) {
        TRACE(IMX219_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    pIsiSensorCaps->FieldSelection    = ISI_FIELDSEL_BOTH;
    pIsiSensorCaps->YCSequence        = ISI_YCSEQ_YCBYCR;
    pIsiSensorCaps->Conv422           = ISI_CONV422_NOCOSITED;
    pIsiSensorCaps->HPol              = ISI_HPOL_REFPOS;
    pIsiSensorCaps->VPol              = ISI_VPOL_NEG;
    pIsiSensorCaps->Edge              = ISI_EDGE_RISING;
    pIsiSensorCaps->supportModeNum    = SensorModeInfo.count;
    pIsiSensorCaps->currentMode       = pIMX219Ctx->CurMode.index;

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX219_IsiSetupSensorIss(IsiSensorHandle_t handle,
                                       const IsiSensorCaps_t *pIsiSensorCaps )
{
    int ret = 0;
    RESULT result = RET_SUCCESS;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    if (pIsiSensorCaps->currentMode != pIMX219Ctx->CurMode.index) {
        IsiSensorMode_t SensorMode;
        memset(&SensorMode, 0, sizeof(IsiSensorMode_t));
        SensorMode.index = pIsiSensorCaps->currentMode;
        result = IMX219_IsiSetSensorModeIss(handle, &SensorMode);
        if (result != RET_SUCCESS) {
            TRACE(IMX219_ERROR, "%s:set sensor mode %d failed!\n",
                  __func__, SensorMode.index);
            return result;
        }
    }

#ifdef SUBDEV_V4L2
    struct v4l2_subdev_format format;
    memset(&format, 0, sizeof(struct v4l2_subdev_format));
    format.format.width  = pIMX219Ctx->CurMode.size.bounds_width;
    format.format.height = pIMX219Ctx->CurMode.size.bounds_height;
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    ret = ioctl(pHalCtx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret != 0){
        TRACE(IMX219_ERROR, "%s: sensor set format error!\n", __func__);
        return RET_FAILURE;
    }
#else
    ret = ioctrl(pHalCtx->sensor_fd, VVSENSORIOC_S_INIT, NULL);
    if (ret != 0){
        TRACE(IMX219_ERROR, "%s: sensor init error!\n", __func__);
        return RET_FAILURE;
    }
#endif

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiGetSensorRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    int ret = 0;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pValue == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, pValue);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s: get chip id error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiCheckSensorConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    uint32_t ChipId = 0;
    result = IMX219_IsiGetSensorRevisionIss(handle, &ChipId);
    if (result != RET_SUCCESS) {
        TRACE(IMX219_ERROR, "%s:get sensor chip id error!\n",__func__);
        return RET_FAILURE;
    }

    if (ChipId != 0x2770) {
        TRACE(IMX219_ERROR,
            "%s:ChipID=0x2770,while read sensor Id=0x%x error!\n",
             __func__, ChipId);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiGetAeInfoIss(IsiSensorHandle_t handle,
                                     IsiSensorAeInfo_t *pAeInfo)
{
    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    if (pAeInfo == NULL)
        return RET_NULL_POINTER;

    memcpy(pAeInfo, &pIMX219Ctx->AeInfo, sizeof(IsiSensorAeInfo_t));

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSetHdrRatioIss(IsiSensorHandle_t handle,
                                       uint8_t hdrRatioNum,
                                       uint32_t HdrRatio[])
{
    int ret = 0;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    struct sensor_hdr_artio_s hdr_ratio;
    if (hdrRatioNum == 2) {
        hdr_ratio.ratio_s_vs = HdrRatio[1];
        hdr_ratio.ratio_l_s = HdrRatio[0];
    }else {
        hdr_ratio.ratio_s_vs = HdrRatio[0];
        hdr_ratio.ratio_l_s = 0;
    }

    if (hdr_ratio.ratio_s_vs == pIMX219Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs &&
        hdr_ratio.ratio_l_s == pIMX219Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s)
        return RET_SUCCESS;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_HDR_RADIO, &hdr_ratio);
    if (ret != 0) {
        TRACE(IMX219_ERROR,"%s: set hdr ratio error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s sensor_mode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX219_ERROR,"%s: get mode info error!\n", __func__);
        return RET_FAILURE;
    }

    memcpy(&pIMX219Ctx->CurMode, &sensor_mode, sizeof (struct vvcam_mode_info_s));
    IMX219_UpdateIsiAEInfo(handle);

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    memcpy(pIntegrationTime, &pIMX219Ctx->IntTime, sizeof(IsiSensorIntTime_t));

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}

static RESULT IMX219_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    int ret = 0;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    uint32_t oneLineTime;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pIntegrationTime == NULL)
        return RET_NULL_POINTER;

    oneLineTime =  pIMX219Ctx->AeInfo.oneLineExpTime;
    pIMX219Ctx->IntTime.expoFrmType = pIntegrationTime->expoFrmType;

    switch (pIntegrationTime->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            IntLine = (pIntegrationTime->IntegrationTime.linearInt +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pIMX219Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor linear exp error!\n", __func__);
                    return RET_FAILURE;
                }
               pIMX219Ctx->IntLine = IntLine;
            }
            TRACE(IMX219_INFO, "%s set linear exp %d \n", __func__,IntLine);
            pIMX219Ctx->IntTime.IntegrationTime.linearInt =  IntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            IntLine = (pIntegrationTime->IntegrationTime.dualInt.dualIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pIMX219Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor dual exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX219Ctx->IntLine = IntLine;
            }

            if (pIMX219Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                ShortIntLine = (pIntegrationTime->IntegrationTime.dualInt.dualSIntTime +
                               (oneLineTime / 2)) / oneLineTime;
                if (ShortIntLine != pIMX219Ctx->ShortIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                    if (ret != 0) {
                        TRACE(IMX219_ERROR,"%s:set sensor dual vsexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pIMX219Ctx->ShortIntLine = ShortIntLine;
                }
            } else {
                ShortIntLine = IntLine;
                pIMX219Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(IMX219_INFO, "%s set dual exp %d short_exp %d\n", __func__, IntLine, ShortIntLine);
            pIMX219Ctx->IntTime.IntegrationTime.dualInt.dualIntTime  = IntLine * oneLineTime;
            pIMX219Ctx->IntTime.IntegrationTime.dualInt.dualSIntTime = ShortIntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            if (pIMX219Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                LongIntLine = (pIntegrationTime->IntegrationTime.triInt.triLIntTime +
                        (oneLineTime / 2)) / oneLineTime;
                if (LongIntLine != pIMX219Ctx->LongIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_EXP, &LongIntLine);
                    if (ret != 0) {
                        TRACE(IMX219_ERROR,"%s:set sensor tri lexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pIMX219Ctx->LongIntLine = LongIntLine;
                }
            } else {
                LongIntLine = (pIntegrationTime->IntegrationTime.triInt.triIntTime +
                       (oneLineTime / 2)) / oneLineTime;
                pIMX219Ctx->LongIntLine = LongIntLine;
            }

            IntLine = (pIntegrationTime->IntegrationTime.triInt.triIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pIMX219Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor tri exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX219Ctx->IntLine = IntLine;
            }
            
            ShortIntLine = (pIntegrationTime->IntegrationTime.triInt.triSIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (ShortIntLine != pIMX219Ctx->ShortIntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor tri vsexp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX219Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(IMX219_INFO, "%s set tri long exp %d exp %d short_exp %d\n", __func__, LongIntLine, IntLine, ShortIntLine);
            pIMX219Ctx->IntTime.IntegrationTime.triInt.triLIntTime = LongIntLine * oneLineTime;
            pIMX219Ctx->IntTime.IntegrationTime.triInt.triIntTime = IntLine * oneLineTime;
            pIMX219Ctx->IntTime.IntegrationTime.triInt.triSIntTime = ShortIntLine * oneLineTime;
            break;
        default:
            return RET_FAILURE;
            break;
    }
    
    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiGetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    if (pGain == NULL)
        return RET_NULL_POINTER;
    memcpy(pGain, &pIMX219Ctx->SensorGain, sizeof(IsiSensorGain_t));

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    int ret = 0;
    uint32_t LongGain;
    uint32_t Gain;
    uint32_t ShortGain;

    TRACE(IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pGain == NULL)
        return RET_NULL_POINTER;

    pIMX219Ctx->SensorGain.expoFrmType = pGain->expoFrmType;
    switch (pGain->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            Gain = pGain->gain.linearGainParas;
            if (pIMX219Ctx->SensorGain.gain.linearGainParas != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor linear gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            pIMX219Ctx->SensorGain.gain.linearGainParas = pGain->gain.linearGainParas;
            TRACE(IMX219_INFO, "%s set linear gain %d\n", __func__,pGain->gain.linearGainParas);
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            Gain = pGain->gain.dualGainParas.dualGain;
            if (pIMX219Ctx->SensorGain.gain.dualGainParas.dualGain != Gain) {
                if (pIMX219Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &Gain);
                }
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor dual gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.dualGainParas.dualSGain;
            if (pIMX219Ctx->SensorGain.gain.dualGainParas.dualSGain != ShortGain) {
                if (pIMX219Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &ShortGain);
                }
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor dual vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(IMX219_INFO,"%s:set gain%d short gain %d!\n", __func__,Gain,ShortGain);
            pIMX219Ctx->SensorGain.gain.dualGainParas.dualGain = Gain;
            pIMX219Ctx->SensorGain.gain.dualGainParas.dualSGain = ShortGain;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            LongGain = pGain->gain.triGainParas.triLGain;
            if (pIMX219Ctx->SensorGain.gain.triGainParas.triLGain != LongGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &LongGain);
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            Gain = pGain->gain.triGainParas.triGain;
            if (pIMX219Ctx->SensorGain.gain.triGainParas.triGain != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.triGainParas.triSGain;
            if (pIMX219Ctx->SensorGain.gain.triGainParas.triSGain != ShortGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                if (ret != 0) {
                    TRACE(IMX219_ERROR,"%s:set sensor tri vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(IMX219_INFO,"%s:set long gain %d gain%d short gain %d!\n", __func__, LongGain, Gain, ShortGain);
            pIMX219Ctx->SensorGain.gain.triGainParas.triLGain = LongGain;
            pIMX219Ctx->SensorGain.gain.triGainParas.triGain = Gain;
            pIMX219Ctx->SensorGain.gain.triGainParas.triSGain = ShortGain;
            break;
        default:
            return RET_FAILURE;
            break;
    }

    TRACE(IMX219_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}


static RESULT IMX219_IsiGetSensorFpsIss(IsiSensorHandle_t handle, uint32_t * pfps)
{
    TRACE(IMX219_INFO, "%s: (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    if (pfps == NULL)
        return RET_NULL_POINTER;

    *pfps = pIMX219Ctx->CurMode.ae_info.cur_fps;

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSetSensorFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    int ret = 0;

    TRACE(IMX219_INFO, "%s: (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_FPS, &fps);
    if (ret != 0) {
        TRACE(IMX219_ERROR,"%s:set sensor fps error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s SensorMode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &SensorMode);
    if (ret != 0) {
        TRACE(IMX219_ERROR,"%s:get sensor mode error!\n", __func__);
        return RET_FAILURE;
    }
    memcpy(&pIMX219Ctx->CurMode, &SensorMode, sizeof(struct vvcam_mode_info_s));
    IMX219_UpdateIsiAEInfo(handle);

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
static RESULT IMX219_IsiSetSensorAfpsLimitsIss(IsiSensorHandle_t handle, uint32_t minAfps)
{
    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    TRACE(IMX219_INFO, "%s: (enter)\n", __func__);

    if ((minAfps > pIMX219Ctx->CurMode.ae_info.max_fps) ||
        (minAfps < pIMX219Ctx->CurMode.ae_info.min_fps))
        return RET_FAILURE;
    pIMX219Ctx->minAfps = minAfps;
    pIMX219Ctx->CurMode.ae_info.min_afps = minAfps;

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiGetSensorIspStatusIss(IsiSensorHandle_t handle,
                               IsiSensorIspStatus_t *pSensorIspStatus)
{
    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    TRACE(IMX219_INFO, "%s: (enter)\n", __func__);

    if (pIMX219Ctx->CurMode.hdr_mode == SENSOR_MODE_HDR_NATIVE) {
        pSensorIspStatus->useSensorAWB = true;
        pSensorIspStatus->useSensorBLC = true;
    } else {
        pSensorIspStatus->useSensorAWB = false;
        pSensorIspStatus->useSensorBLC = false;
    }

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
#ifndef ISI_LITE
static RESULT IMX219_IsiSensorSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t * pBlc)
{
    int32_t ret = 0;

    TRACE(IMX219_INFO, "%s: (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pBlc == NULL)
        return RET_NULL_POINTER;

    struct sensor_blc_s SensorBlc;
    SensorBlc.red = pBlc->red;
    SensorBlc.gb = pBlc->gb;
    SensorBlc.gr = pBlc->gr;
    SensorBlc.blue = pBlc->blue;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_BLC, &SensorBlc);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSensorSetWBIss(IsiSensorHandle_t handle, IsiSensorWB_t *pWb)
{
    int32_t ret = 0;

    TRACE(IMX219_INFO, "%s: (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pWb == NULL)
        return RET_NULL_POINTER;

    struct sensor_white_balance_s SensorWb;
    SensorWb.r_gain = pWb->r_gain;
    SensorWb.gr_gain = pWb->gr_gain;
    SensorWb.gb_gain = pWb->gb_gain;
    SensorWb.b_gain = pWb->b_gain;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_WB, &SensorWb);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSensorGetExpandCurveIss(IsiSensorHandle_t handle, IsiSensorExpandCurve_t *pExpandCurve)
{
    int32_t ret = 0;

    TRACE(IMX219_INFO, "%s: (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

    if (pExpandCurve == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_EXPAND_CURVE, pExpandCurve);
    if (ret != 0) {
        TRACE(IMX219_ERROR, "%s: get  expand cure error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSensorGetCompressCurveIss(IsiSensorHandle_t handle, IsiSensorCompressCurve_t *pCompressCurve)
{
    int i = 0;
    TRACE(IMX219_INFO, "%s: (enter)\n", __func__);

    if (pCompressCurve == NULL)
        return RET_NULL_POINTER;

    if ((pCompressCurve->x_bit == 16) && (pCompressCurve->y_bit == 12)) {
        uint8_t compress_px[64] = {10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};

        pCompressCurve->compress_x_data[0] = 0;
        pCompressCurve->compress_y_data[0] = 0;
        for (i= 1; i < 65; i++) {
            pCompressCurve->compress_px[i-1] = compress_px[i-1];
            pCompressCurve->compress_x_data[i] = pCompressCurve->compress_x_data[i-1] + (1 << compress_px[i-1]);
            if(pCompressCurve->compress_x_data[i] < 1024) {
                pCompressCurve->compress_y_data[i] = pCompressCurve->compress_x_data[i] / 2;
            } else if (pCompressCurve->compress_x_data[i] < 2048) {
                pCompressCurve->compress_y_data[i] = pCompressCurve->compress_x_data[i] / 4 + 256;
            } else if (pCompressCurve->compress_x_data[i] < 16384){
                pCompressCurve->compress_y_data[i] = pCompressCurve->compress_x_data[i] / 8 + 512;
            } else {
                pCompressCurve->compress_y_data[i] = pCompressCurve->compress_x_data[i] / 32 + 2048;
            }
        }

    } else if ((pCompressCurve->x_bit == 20) && (pCompressCurve->y_bit == 12)) {
        return RET_FAILURE;
    } else {
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiSetTestPatternIss(IsiSensorHandle_t handle,
                                       IsiSensorTpgMode_e  tpgMode)
{
    int32_t ret = 0;

    TRACE( IMX219_INFO, "%s (enter)\n", __func__);

    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX219Ctx->IsiCtx.HalHandle;

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
        TRACE(IMX219_ERROR, "%s: set test pattern %d error\n", __func__, tpgMode);
        return RET_FAILURE;
    }

    TRACE(IMX219_INFO, "%s: test pattern enable[%d] mode[%d]\n", __func__, TestPattern.enable, TestPattern.pattern);

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX219_IsiFocusSetupIss(IsiSensorHandle_t handle)
{
    TRACE( IMX219_INFO, "%s (enter)\n", __func__);
    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX219_IsiFocusReleaseIss(IsiSensorHandle_t handle)
{
    TRACE( IMX219_INFO, "%s (enter)\n", __func__);
    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX219_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( IMX219_INFO, "%s (enter)\n", __func__);
    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX219_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( IMX219_INFO, "%s (enter)\n", __func__);
    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX219_IsiGetFocusCalibrateIss(IsiSensorHandle_t handle, IsiFoucsCalibAttr_t *pFocusCalib)
{
    TRACE( IMX219_INFO, "%s (enter)\n", __func__);
    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX219_IsiGetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t *pExposure)
{
    TRACE( IMX219_INFO, "%s (enter)\n", __func__);
    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    if (pIMX219Ctx->AEStartExposure == 0) {
        pIMX219Ctx->AEStartExposure =
            (uint64_t)pIMX219Ctx->CurMode.ae_info.start_exposure *
            pIMX219Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;
           
    }
    *pExposure =  pIMX219Ctx->AEStartExposure;
    TRACE(IMX219_INFO, "%s:get start exposure %ld\n", __func__, pIMX219Ctx->AEStartExposure);

    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX219_IsiSetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t exposure)
{
    TRACE( IMX219_INFO, "%s (enter)\n", __func__);
    IMX219_Context_t *pIMX219Ctx = (IMX219_Context_t *) handle;

    pIMX219Ctx->AEStartExposure = exposure;
    TRACE(IMX219_INFO, "%s set start exposure %ld\n", __func__,pIMX219Ctx->AEStartExposure);
    TRACE(IMX219_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}
#endif

RESULT IMX219_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    TRACE( IMX219_INFO, "%s (enter)\n", __func__);
	fprintf(stderr, "Warning: IMX219 %s == !\n",__func__);

    if (pIsiSensor == NULL)
        return RET_NULL_POINTER;
     pIsiSensor->pszName                         = SensorName;
     pIsiSensor->pIsiSensorSetPowerIss           = IMX219_IsiSensorSetPowerIss;
     pIsiSensor->pIsiCreateSensorIss             = IMX219_IsiCreateSensorIss;
     pIsiSensor->pIsiReleaseSensorIss            = IMX219_IsiReleaseSensorIss;
     pIsiSensor->pIsiRegisterReadIss             = IMX219_IsiRegisterReadIss;
     pIsiSensor->pIsiRegisterWriteIss            = IMX219_IsiRegisterWriteIss;
     pIsiSensor->pIsiGetSensorModeIss            = IMX219_IsiGetSensorModeIss;
     pIsiSensor->pIsiSetSensorModeIss            = IMX219_IsiSetSensorModeIss;
     pIsiSensor->pIsiQuerySensorIss              = IMX219_IsiQuerySensorIss;
     pIsiSensor->pIsiGetCapsIss                  = IMX219_IsiGetCapsIss;
     pIsiSensor->pIsiSetupSensorIss              = IMX219_IsiSetupSensorIss;
     pIsiSensor->pIsiGetSensorRevisionIss        = IMX219_IsiGetSensorRevisionIss;
     pIsiSensor->pIsiCheckSensorConnectionIss    = IMX219_IsiCheckSensorConnectionIss;
     pIsiSensor->pIsiSensorSetStreamingIss       = IMX219_IsiSensorSetStreamingIss;
     pIsiSensor->pIsiGetAeInfoIss                = IMX219_IsiGetAeInfoIss;
     pIsiSensor->pIsiSetHdrRatioIss              = IMX219_IsiSetHdrRatioIss;
     pIsiSensor->pIsiGetIntegrationTimeIss       = IMX219_IsiGetIntegrationTimeIss;
     pIsiSensor->pIsiSetIntegrationTimeIss       = IMX219_IsiSetIntegrationTimeIss;
     pIsiSensor->pIsiGetGainIss                  = IMX219_IsiGetGainIss;
     pIsiSensor->pIsiSetGainIss                  = IMX219_IsiSetGainIss;
     pIsiSensor->pIsiGetSensorFpsIss             = IMX219_IsiGetSensorFpsIss;
     pIsiSensor->pIsiSetSensorFpsIss             = IMX219_IsiSetSensorFpsIss;
     pIsiSensor->pIsiSetSensorAfpsLimitsIss      = IMX219_IsiSetSensorAfpsLimitsIss;
     pIsiSensor->pIsiGetSensorIspStatusIss       = IMX219_IsiGetSensorIspStatusIss;
#ifndef ISI_LITE
    pIsiSensor->pIsiSensorSetBlcIss              = IMX219_IsiSensorSetBlcIss;
    pIsiSensor->pIsiSensorSetWBIss               = IMX219_IsiSensorSetWBIss;
    pIsiSensor->pIsiSensorGetExpandCurveIss      = IMX219_IsiSensorGetExpandCurveIss;
    pIsiSensor->pIsiSensorGetCompressCurveIss    = IMX219_IsiSensorGetCompressCurveIss;
    pIsiSensor->pIsiActivateTestPatternIss       = IMX219_IsiSetTestPatternIss;
    pIsiSensor->pIsiFocusSetupIss                = IMX219_IsiFocusSetupIss;
    pIsiSensor->pIsiFocusReleaseIss              = IMX219_IsiFocusReleaseIss;
    pIsiSensor->pIsiFocusSetIss                  = IMX219_IsiFocusSetIss;
    pIsiSensor->pIsiFocusGetIss                  = IMX219_IsiFocusGetIss;
    pIsiSensor->pIsiGetFocusCalibrateIss         = IMX219_IsiGetFocusCalibrateIss;
    pIsiSensor->pIsiSetAeStartExposureIss        = IMX219_IsiSetAeStartExposureIs;
    pIsiSensor->pIsiGetAeStartExposureIss        = IMX219_IsiGetAeStartExposureIs;
#endif
    TRACE( IMX219_INFO, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x0219,
    .pIsiHalQuerySensor = IMX219_IsiHalQuerySensorIss,
    .pfIsiGetSensorIss = IMX219_IsiGetSensorIss,
};
