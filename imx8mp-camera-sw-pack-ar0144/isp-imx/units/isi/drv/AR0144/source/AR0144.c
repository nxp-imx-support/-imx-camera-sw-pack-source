/****************************************************************************
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: MIT
 *
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

CREATE_TRACER( AR0144_INFO , "AR0144: ", INFO,    0);
CREATE_TRACER( AR0144_WARN , "AR0144: ", WARNING, 0);
CREATE_TRACER( AR0144_ERROR, "AR0144: ", ERROR,   1);

#ifdef SUBDEV_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#endif

static const char SensorName[16] = "ar0144";

typedef struct AR0144_Context_s
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
    int motor_fd;
    uint32_t focus_mode;
} AR0144_Context_t;

static inline int OpenMotorDevice(const vvcam_lens_t *pfocus_lens)
{
    int filep;
    char szFile[32];
    struct v4l2_capability caps;
    for (int i = 0; i < 20; i++) {
        sprintf(szFile, "/dev/v4l-subdev%d", i);
        filep = open(szFile, O_RDWR | O_NONBLOCK);
        if (filep < 0) {
            continue;
        }

        if (ioctl(filep, VIDIOC_QUERYCAP, &caps) < 0) {
            close(filep);
            continue;
        }

        if (strcmp((char*) caps.driver, (char*) pfocus_lens->name)
            || (atoi((char *)caps.bus_info) != pfocus_lens->id)) {
            close(filep);
            continue;
        } else {
            return filep;
        }
    }
    return -1;
}

static RESULT AR0144_IsiSensorSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    int ret = 0;

    TRACE( AR0144_INFO, "%s: (enter)\n", __func__);
    TRACE( AR0144_INFO, "%s: set power %d\n", __func__,on);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    int32_t power = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_POWER, &power);
    if (ret != 0){
        TRACE(AR0144_ERROR, "%s set power %d error\n", __func__,power);
        return RET_FAILURE;
    }

    TRACE( AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiSensorGetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( AR0144_INFO, "%s: (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    if (!pclk)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CLK, pclk);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s get clock error\n", __func__);
        return RET_FAILURE;
    } 
    
    TRACE( AR0144_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);
    TRACE( AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiSensorSetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( AR0144_INFO, "%s: (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    if (pclk == NULL)
        return RET_NULL_POINTER;
    
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_CLK, &pclk);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( AR0144_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);

    TRACE( AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiResetSensorIss(IsiSensorHandle_t handle)
{
    int ret = 0;

    TRACE( AR0144_INFO, "%s: (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_RESET, NULL);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s set reset error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiRegisterReadIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        uint32_t * pValue)
{
    int32_t ret = 0;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = 0;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_READ_REG, &sccb_data);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s: read sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    *pValue = sccb_data.data;

    TRACE(AR0144_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiRegisterWriteIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        const uint32_t value)
{
    int ret = 0;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = value;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_WRITE_REG, &sccb_data);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s: write sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    TRACE(AR0144_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_UpdateIsiAEInfo(IsiSensorHandle_t handle)
{
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    uint32_t exp_line_time = pAR0144Ctx->CurMode.ae_info.one_line_exp_time_ns;

    IsiSensorAeInfo_t *pAeInfo = &pAR0144Ctx->AeInfo;
    pAeInfo->oneLineExpTime = (exp_line_time << ISI_EXPO_PARAS_FIX_FRACBITS) / 1000;

    if (pAR0144Ctx->CurMode.hdr_mode == SENSOR_MODE_LINEAR) {
        pAeInfo->maxIntTime.linearInt =
            pAR0144Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->minIntTime.linearInt =
            pAR0144Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->maxAGain.linearGainParas = pAR0144Ctx->CurMode.ae_info.max_again;
        pAeInfo->minAGain.linearGainParas = pAR0144Ctx->CurMode.ae_info.min_again;
        pAeInfo->maxDGain.linearGainParas = pAR0144Ctx->CurMode.ae_info.max_dgain;
        pAeInfo->minDGain.linearGainParas = pAR0144Ctx->CurMode.ae_info.min_dgain;
    } else {
        switch (pAR0144Ctx->CurMode.stitching_mode) {
            case SENSOR_STITCHING_DUAL_DCG:
            case SENSOR_STITCHING_3DOL:
            case SENSOR_STITCHING_LINEBYLINE:
                pAeInfo->maxIntTime.triInt.triSIntTime =
                    pAR0144Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triSIntTime =
                    pAR0144Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                
                pAeInfo->maxIntTime.triInt.triIntTime =
                    pAR0144Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triIntTime =
                    pAR0144Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pAR0144Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG) {
                    pAeInfo->maxIntTime.triInt.triLIntTime = pAeInfo->maxIntTime.triInt.triIntTime;
                    pAeInfo->minIntTime.triInt.triLIntTime = pAeInfo->minIntTime.triInt.triIntTime;
                } else {
                    pAeInfo->maxIntTime.triInt.triLIntTime =
                        pAR0144Ctx->CurMode.ae_info.max_longintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.triInt.triLIntTime =
                        pAR0144Ctx->CurMode.ae_info.min_longintegration_line * pAeInfo->oneLineExpTime;
                }

                pAeInfo->maxAGain.triGainParas.triSGain = pAR0144Ctx->CurMode.ae_info.max_short_again;
                pAeInfo->minAGain.triGainParas.triSGain = pAR0144Ctx->CurMode.ae_info.min_short_again;
                pAeInfo->maxDGain.triGainParas.triSGain = pAR0144Ctx->CurMode.ae_info.max_short_dgain;
                pAeInfo->minDGain.triGainParas.triSGain = pAR0144Ctx->CurMode.ae_info.min_short_dgain;

                pAeInfo->maxAGain.triGainParas.triGain = pAR0144Ctx->CurMode.ae_info.max_again;
                pAeInfo->minAGain.triGainParas.triGain = pAR0144Ctx->CurMode.ae_info.min_again;
                pAeInfo->maxDGain.triGainParas.triGain = pAR0144Ctx->CurMode.ae_info.max_dgain;
                pAeInfo->minDGain.triGainParas.triGain = pAR0144Ctx->CurMode.ae_info.min_dgain;

                pAeInfo->maxAGain.triGainParas.triLGain = pAR0144Ctx->CurMode.ae_info.max_long_again;
                pAeInfo->minAGain.triGainParas.triLGain = pAR0144Ctx->CurMode.ae_info.min_long_again;
                pAeInfo->maxDGain.triGainParas.triLGain = pAR0144Ctx->CurMode.ae_info.max_long_dgain;
                pAeInfo->minDGain.triGainParas.triLGain = pAR0144Ctx->CurMode.ae_info.min_long_dgain;
                break;
            case SENSOR_STITCHING_DUAL_DCG_NOWAIT:
            case SENSOR_STITCHING_16BIT_COMPRESS:
            case SENSOR_STITCHING_L_AND_S:
            case SENSOR_STITCHING_2DOL:
                pAeInfo->maxIntTime.dualInt.dualIntTime =
                    pAR0144Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.dualInt.dualIntTime =
                    pAR0144Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pAR0144Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime = pAeInfo->maxIntTime.dualInt.dualIntTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime = pAeInfo->minIntTime.dualInt.dualIntTime;
                } else {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime =
                        pAR0144Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime =
                        pAR0144Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                }
                
                if (pAR0144Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pAR0144Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pAR0144Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pAR0144Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pAR0144Ctx->CurMode.ae_info.min_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pAR0144Ctx->CurMode.ae_info.max_long_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pAR0144Ctx->CurMode.ae_info.min_long_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pAR0144Ctx->CurMode.ae_info.max_long_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pAR0144Ctx->CurMode.ae_info.min_long_dgain;
                } else {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pAR0144Ctx->CurMode.ae_info.max_short_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pAR0144Ctx->CurMode.ae_info.min_short_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pAR0144Ctx->CurMode.ae_info.max_short_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pAR0144Ctx->CurMode.ae_info.min_short_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pAR0144Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pAR0144Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pAR0144Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pAR0144Ctx->CurMode.ae_info.min_dgain;
                }
                
                break;
            default:
                break;
        }
    }
    pAeInfo->gainStep = pAR0144Ctx->CurMode.ae_info.gain_step;
    pAeInfo->currFps  = pAR0144Ctx->CurMode.ae_info.cur_fps;
    pAeInfo->maxFps   = pAR0144Ctx->CurMode.ae_info.max_fps;
    pAeInfo->minFps   = pAR0144Ctx->CurMode.ae_info.min_fps;
    pAeInfo->minAfps  = pAR0144Ctx->CurMode.ae_info.min_afps;
    pAeInfo->hdrRatio[0] = pAR0144Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s;
    pAeInfo->hdrRatio[1] = pAR0144Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs;

    pAeInfo->intUpdateDlyFrm = pAR0144Ctx->CurMode.ae_info.int_update_delay_frm;
    pAeInfo->gainUpdateDlyFrm = pAR0144Ctx->CurMode.ae_info.gain_update_delay_frm;

    if (pAR0144Ctx->minAfps != 0) {
        pAeInfo->minAfps = pAR0144Ctx->minAfps;
    } 
    return RET_SUCCESS;
}

static RESULT AR0144_IsiGetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    memcpy(pMode, &pAR0144Ctx->CurMode, sizeof(IsiSensorMode_t));

    TRACE(AR0144_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiSetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    int ret = 0;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    struct vvcam_mode_info_s sensor_mode;
    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    sensor_mode.index = pMode->index;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s set sensor mode failed", __func__);
        return RET_FAILURE;
    }
    memcpy(&pAR0144Ctx->CurMode, &sensor_mode, sizeof(struct vvcam_mode_info_s));
    AR0144_UpdateIsiAEInfo(handle);

    TRACE(AR0144_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                                              bool_t on)
{
    int ret = 0;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    uint32_t status = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_STREAM, &status);
    if (ret != 0){
        TRACE(AR0144_ERROR, "%s set sensor stream %d error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s: set streaming %d\n", __func__, on);
    TRACE(AR0144_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiCreateSensorIss(IsiSensorInstanceConfig_t * pConfig)
{
    RESULT result = RET_SUCCESS;
    AR0144_Context_t *pAR0144Ctx;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    if (!pConfig || !pConfig->pSensor || !pConfig->HalHandle)
        return RET_NULL_POINTER;

    pAR0144Ctx = (AR0144_Context_t *) malloc(sizeof(AR0144_Context_t));
    if (!pAR0144Ctx)
        return RET_OUTOFMEM;

    memset(pAR0144Ctx, 0, sizeof(AR0144_Context_t));
    pAR0144Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pAR0144Ctx->IsiCtx.pSensor   = pConfig->pSensor;
    pConfig->hSensor = (IsiSensorHandle_t) pAR0144Ctx;

    result = AR0144_IsiSensorSetPowerIss(pAR0144Ctx, BOOL_TRUE);
    if (result != RET_SUCCESS) {
        TRACE(AR0144_ERROR, "%s set power error\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    result = AR0144_IsiSensorGetClkIss(pAR0144Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(AR0144_ERROR, "%s get clk error\n", __func__);
        return RET_FAILURE;
    }
    clk.status = 1;
    result = AR0144_IsiSensorSetClkIss(pAR0144Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(AR0144_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }
    result = AR0144_IsiResetSensorIss(pAR0144Ctx);
    if (result != RET_SUCCESS) {
        TRACE(AR0144_ERROR, "%s retset sensor error\n", __func__);
        return RET_FAILURE;
    }

    IsiSensorMode_t SensorMode;
    SensorMode.index = pConfig->SensorModeIndex;
    result = AR0144_IsiSetSensorModeIss(pAR0144Ctx, &SensorMode);
    if (result != RET_SUCCESS) {
        TRACE(AR0144_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT AR0144_IsiReleaseSensorIss(IsiSensorHandle_t handle)
{
    TRACE(AR0144_INFO, "%s (enter) \n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    if (pAR0144Ctx == NULL)
        return (RET_WRONG_HANDLE);

    AR0144_IsiSensorSetStreamingIss(pAR0144Ctx, BOOL_FALSE);
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    AR0144_IsiSensorGetClkIss(pAR0144Ctx, &clk);
    clk.status = 0;
    AR0144_IsiSensorSetClkIss(pAR0144Ctx, &clk);
    AR0144_IsiSensorSetPowerIss(pAR0144Ctx, BOOL_FALSE);
    free(pAR0144Ctx);
    pAR0144Ctx = NULL;

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiHalQuerySensorIss(HalHandle_t HalHandle,
                                          IsiSensorModeInfoArray_t *pSensorMode)
{
    int ret = 0;

    TRACE(AR0144_INFO, "%s (enter) \n", __func__);

    if (HalHandle == NULL || pSensorMode == NULL)
        return RET_NULL_POINTER;

    HalContext_t *pHalCtx = (HalContext_t *)HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_QUERY, pSensorMode);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiQuerySensorIss(IsiSensorHandle_t handle,
                                       IsiSensorModeInfoArray_t *pSensorMode)
{
    RESULT result = RET_SUCCESS;

    TRACE(AR0144_INFO, "%s (enter) \n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    result = AR0144_IsiHalQuerySensorIss(pAR0144Ctx->IsiCtx.HalHandle,
                                         pSensorMode);
    if (result != RET_SUCCESS)
        TRACE(AR0144_ERROR, "%s: query sensor mode info error!\n", __func__);

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT AR0144_IsiGetCapsIss(IsiSensorHandle_t handle,
                                   IsiSensorCaps_t * pIsiSensorCaps)
{
    RESULT result = RET_SUCCESS;

    TRACE(AR0144_INFO, "%s (enter) \n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    IsiSensorModeInfoArray_t SensorModeInfo;
    memset(&SensorModeInfo, 0, sizeof(IsiSensorModeInfoArray_t));
    result = AR0144_IsiQuerySensorIss(handle, &SensorModeInfo);
    if (result != RET_SUCCESS) {
        TRACE(AR0144_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    pIsiSensorCaps->FieldSelection    = ISI_FIELDSEL_BOTH;
    pIsiSensorCaps->YCSequence        = ISI_YCSEQ_YCBYCR;
    pIsiSensorCaps->Conv422           = ISI_CONV422_NOCOSITED;
    pIsiSensorCaps->HPol              = ISI_HPOL_REFPOS;
    pIsiSensorCaps->VPol              = ISI_VPOL_NEG;
    pIsiSensorCaps->Edge              = ISI_EDGE_RISING;
    pIsiSensorCaps->supportModeNum    = SensorModeInfo.count;
    pIsiSensorCaps->currentMode       = pAR0144Ctx->CurMode.index;

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT AR0144_IsiSetupSensorIss(IsiSensorHandle_t handle,
                                       const IsiSensorCaps_t *pIsiSensorCaps )
{
    int ret = 0;
    RESULT result = RET_SUCCESS;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    if (pIsiSensorCaps->currentMode != pAR0144Ctx->CurMode.index) {
        IsiSensorMode_t SensorMode;
        memset(&SensorMode, 0, sizeof(IsiSensorMode_t));
        SensorMode.index = pIsiSensorCaps->currentMode;
        result = AR0144_IsiSetSensorModeIss(handle, &SensorMode);
        if (result != RET_SUCCESS) {
            TRACE(AR0144_ERROR, "%s:set sensor mode %d failed!\n",
                  __func__, SensorMode.index);
            return result;
        }
    }

#ifdef SUBDEV_V4L2
    struct v4l2_subdev_format format;
    memset(&format, 0, sizeof(struct v4l2_subdev_format));
    format.format.width  = pAR0144Ctx->CurMode.size.bounds_width;
    format.format.height = pAR0144Ctx->CurMode.size.bounds_height;
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    ret = ioctl(pHalCtx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret != 0){
        TRACE(AR0144_ERROR, "%s: sensor set format error!\n", __func__);
        return RET_FAILURE;
    }
#else
    ret = ioctrl(pHalCtx->sensor_fd, VVSENSORIOC_S_INIT, NULL);
    if (ret != 0){
        TRACE(AR0144_ERROR, "%s: sensor init error!\n", __func__);
        return RET_FAILURE;
    }
#endif

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiGetSensorRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    int ret = 0;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    if (pValue == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, pValue);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s: get chip id error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiCheckSensorConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    uint32_t ChipId = 0;
    result = AR0144_IsiGetSensorRevisionIss(handle, &ChipId);
    if (result != RET_SUCCESS) {
        TRACE(AR0144_ERROR, "%s:get sensor chip id error!\n",__func__);
        return RET_FAILURE;
    }

    if (ChipId != 0x356) {
        TRACE(AR0144_ERROR,
            "%s:ChipID=0x356,while read sensor Id=0x%x error!\n",
             __func__, ChipId);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiGetAeInfoIss(IsiSensorHandle_t handle,
                                     IsiSensorAeInfo_t *pAeInfo)
{
    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    if (pAeInfo == NULL)
        return RET_NULL_POINTER;

    memcpy(pAeInfo, &pAR0144Ctx->AeInfo, sizeof(IsiSensorAeInfo_t));

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    memcpy(pIntegrationTime, &pAR0144Ctx->IntTime, sizeof(IsiSensorIntTime_t));

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}

static RESULT AR0144_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    int ret = 0;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    uint32_t oneLineTime;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    if (pIntegrationTime == NULL)
        return RET_NULL_POINTER;

    oneLineTime =  pAR0144Ctx->AeInfo.oneLineExpTime;
    pAR0144Ctx->IntTime.expoFrmType = pIntegrationTime->expoFrmType;

    switch (pIntegrationTime->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            IntLine = (pIntegrationTime->IntegrationTime.linearInt +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pAR0144Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor linear exp error!\n", __func__);
                    return RET_FAILURE;
                }
               pAR0144Ctx->IntLine = IntLine;
            }
            TRACE(AR0144_INFO, "%s set linear exp %d \n", __func__,IntLine);
            pAR0144Ctx->IntTime.IntegrationTime.linearInt =  IntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            IntLine = (pIntegrationTime->IntegrationTime.dualInt.dualIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pAR0144Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor dual exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pAR0144Ctx->IntLine = IntLine;
            }

            if (pAR0144Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                ShortIntLine = (pIntegrationTime->IntegrationTime.dualInt.dualSIntTime +
                               (oneLineTime / 2)) / oneLineTime;
                if (ShortIntLine != pAR0144Ctx->ShortIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                    if (ret != 0) {
                        TRACE(AR0144_ERROR,"%s:set sensor dual vsexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pAR0144Ctx->ShortIntLine = ShortIntLine;
                }
            } else {
                ShortIntLine = IntLine;
                pAR0144Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(AR0144_INFO, "%s set dual exp %d short_exp %d\n", __func__, IntLine, ShortIntLine);
            pAR0144Ctx->IntTime.IntegrationTime.dualInt.dualIntTime  = IntLine * oneLineTime;
            pAR0144Ctx->IntTime.IntegrationTime.dualInt.dualSIntTime = ShortIntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            if (pAR0144Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                LongIntLine = (pIntegrationTime->IntegrationTime.triInt.triLIntTime +
                        (oneLineTime / 2)) / oneLineTime;
                if (LongIntLine != pAR0144Ctx->LongIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_EXP, &LongIntLine);
                    if (ret != 0) {
                        TRACE(AR0144_ERROR,"%s:set sensor tri lexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pAR0144Ctx->LongIntLine = LongIntLine;
                }
            } else {
                LongIntLine = (pIntegrationTime->IntegrationTime.triInt.triIntTime +
                       (oneLineTime / 2)) / oneLineTime;
                pAR0144Ctx->LongIntLine = LongIntLine;
            }

            IntLine = (pIntegrationTime->IntegrationTime.triInt.triIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pAR0144Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor tri exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pAR0144Ctx->IntLine = IntLine;
            }
            
            ShortIntLine = (pIntegrationTime->IntegrationTime.triInt.triSIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (ShortIntLine != pAR0144Ctx->ShortIntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor tri vsexp error!\n", __func__);
                    return RET_FAILURE;
                }
                pAR0144Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(AR0144_INFO, "%s set tri long exp %d exp %d short_exp %d\n", __func__, LongIntLine, IntLine, ShortIntLine);
            pAR0144Ctx->IntTime.IntegrationTime.triInt.triLIntTime = LongIntLine * oneLineTime;
            pAR0144Ctx->IntTime.IntegrationTime.triInt.triIntTime = IntLine * oneLineTime;
            pAR0144Ctx->IntTime.IntegrationTime.triInt.triSIntTime = ShortIntLine * oneLineTime;
            break;
        default:
            return RET_FAILURE;
            break;
    }
    
    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiGetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    if (pGain == NULL)
        return RET_NULL_POINTER;
    memcpy(pGain, &pAR0144Ctx->SensorGain, sizeof(IsiSensorGain_t));

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiSetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    int ret = 0;
    uint32_t LongGain;
    uint32_t Gain;
    uint32_t ShortGain;

    TRACE(AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    if (pGain == NULL)
        return RET_NULL_POINTER;

    pAR0144Ctx->SensorGain.expoFrmType = pGain->expoFrmType;
    switch (pGain->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            Gain = pGain->gain.linearGainParas;
            if (pAR0144Ctx->SensorGain.gain.linearGainParas != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
				TRACE(AR0144_ERROR,"liaoming[%s:%d] Gain:%d\n", __func__, __LINE__,Gain);
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor linear gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            pAR0144Ctx->SensorGain.gain.linearGainParas = pGain->gain.linearGainParas;
            TRACE(AR0144_INFO, "%s set linear gain %d\n", __func__,pGain->gain.linearGainParas);
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            Gain = pGain->gain.dualGainParas.dualGain;
            if (pAR0144Ctx->SensorGain.gain.dualGainParas.dualGain != Gain) {
                if (pAR0144Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
					TRACE(AR0144_ERROR,"liaoming[%s:%d] Gain:%d\n", __func__, __LINE__,Gain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &Gain);
                }
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor dual gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.dualGainParas.dualSGain;
            if (pAR0144Ctx->SensorGain.gain.dualGainParas.dualSGain != ShortGain) {
                if (pAR0144Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &ShortGain);
					TRACE(AR0144_ERROR,"liaoming[%s:%d] ShortGain:%d\n", __func__, __LINE__,ShortGain);
                }
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor dual vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(AR0144_INFO,"%s:set gain%d short gain %d!\n", __func__,Gain,ShortGain);
            pAR0144Ctx->SensorGain.gain.dualGainParas.dualGain = Gain;
            pAR0144Ctx->SensorGain.gain.dualGainParas.dualSGain = ShortGain;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            LongGain = pGain->gain.triGainParas.triLGain;
            if (pAR0144Ctx->SensorGain.gain.triGainParas.triLGain != LongGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &LongGain);
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            Gain = pGain->gain.triGainParas.triGain;
            if (pAR0144Ctx->SensorGain.gain.triGainParas.triGain != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
				TRACE(AR0144_ERROR,"liaoming[%s:%d] Gain:%d\n", __func__, __LINE__,Gain);
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.triGainParas.triSGain;
            if (pAR0144Ctx->SensorGain.gain.triGainParas.triSGain != ShortGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                if (ret != 0) {
                    TRACE(AR0144_ERROR,"%s:set sensor tri vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(AR0144_INFO,"%s:set long gain %d gain%d short gain %d!\n", __func__, LongGain, Gain, ShortGain);
            pAR0144Ctx->SensorGain.gain.triGainParas.triLGain = LongGain;
            pAR0144Ctx->SensorGain.gain.triGainParas.triGain = Gain;
            pAR0144Ctx->SensorGain.gain.triGainParas.triSGain = ShortGain;
            break;
        default:
            return RET_FAILURE;
            break;
    }

    TRACE(AR0144_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}


static RESULT AR0144_IsiGetSensorFpsIss(IsiSensorHandle_t handle, uint32_t * pfps)
{
    TRACE(AR0144_INFO, "%s: (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    if (pfps == NULL)
        return RET_NULL_POINTER;

    *pfps = pAR0144Ctx->CurMode.ae_info.cur_fps;

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiSetSensorFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    int ret = 0;

    TRACE(AR0144_INFO, "%s: (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_FPS, &fps);
    if (ret != 0) {
        TRACE(AR0144_ERROR,"%s:set sensor fps error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s SensorMode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &SensorMode);
    if (ret != 0) {
        TRACE(AR0144_ERROR,"%s:get sensor mode error!\n", __func__);
        return RET_FAILURE;
    }
    memcpy(&pAR0144Ctx->CurMode, &SensorMode, sizeof(struct vvcam_mode_info_s));
    AR0144_UpdateIsiAEInfo(handle);

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
static RESULT AR0144_IsiSetSensorAfpsLimitsIss(IsiSensorHandle_t handle, uint32_t minAfps)
{
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    TRACE(AR0144_INFO, "%s: (enter)\n", __func__);

    if ((minAfps > pAR0144Ctx->CurMode.ae_info.max_fps) ||
        (minAfps < pAR0144Ctx->CurMode.ae_info.min_fps))
        return RET_FAILURE;
    pAR0144Ctx->minAfps = minAfps;
    pAR0144Ctx->CurMode.ae_info.min_afps = minAfps;

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiGetSensorIspStatusIss(IsiSensorHandle_t handle,
                               IsiSensorIspStatus_t *pSensorIspStatus)
{
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    TRACE(AR0144_INFO, "%s: (enter)\n", __func__);

    if (pAR0144Ctx->CurMode.hdr_mode == SENSOR_MODE_HDR_NATIVE) {
        pSensorIspStatus->useSensorAWB = true;
        pSensorIspStatus->useSensorBLC = true;
    } else {
        pSensorIspStatus->useSensorAWB = false;
        pSensorIspStatus->useSensorBLC = false;
    }

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

#ifndef ISI_LITE
static RESULT AR0144_IsiSensorSetWBIss(IsiSensorHandle_t handle, IsiSensorWB_t *pWb)
{
    int32_t ret = 0;

    TRACE(AR0144_INFO, "%s: (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

    if (pWb == NULL)
        return RET_NULL_POINTER;

    struct sensor_white_balance_s SensorWb;
    SensorWb.r_gain = pWb->r_gain;
    SensorWb.gr_gain = pWb->gr_gain;
    SensorWb.gb_gain = pWb->gb_gain;
    SensorWb.b_gain = pWb->b_gain;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_WB, &SensorWb);
    if (ret != 0) {
        TRACE(AR0144_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiSetTestPatternIss(IsiSensorHandle_t handle,
                                       IsiSensorTpgMode_e  tpgMode)
{
    int32_t ret = 0;

    TRACE( AR0144_INFO, "%s (enter)\n", __func__);

    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;

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
        TRACE(AR0144_ERROR, "%s: set test pattern %d error\n", __func__, tpgMode);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s: test pattern enable[%d] mode[%d]\n", __func__, TestPattern.enable, TestPattern.pattern);

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT AR0144_IsiFocusSetupIss(IsiSensorHandle_t handle)
{
    TRACE( AR0144_INFO, "%s (enter)\n", __func__);
    
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pAR0144Ctx->IsiCtx.HalHandle;
    vvcam_lens_t pfocus_lens;

    if (ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_LENS, &pfocus_lens) < 0) {
        TRACE(AR0144_ERROR, "%s  sensor do not have focus-lens \n", __func__);
        return RET_NOTSUPP;
    }

    if (pAR0144Ctx->motor_fd <= 0) {
        pAR0144Ctx->motor_fd = OpenMotorDevice(&pfocus_lens);
        if (pAR0144Ctx->motor_fd < 0) {
            TRACE(AR0144_ERROR, "%s open sensor focus-lens fail\n", __func__);
            return RET_FAILURE;
        }
    } else {
        TRACE(AR0144_INFO, "%s sensor focus-lens already open\n", __func__);
    }

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT AR0144_IsiFocusReleaseIss(IsiSensorHandle_t handle)
{
    TRACE( AR0144_INFO, "%s (enter)\n", __func__);
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    if (pAR0144Ctx->motor_fd <= 0) {
        return RET_NOTSUPP;
    }

    if (close(pAR0144Ctx->motor_fd) < 0) {
        TRACE(AR0144_ERROR,"%s close motor device failed\n", __func__);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT AR0144_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( AR0144_INFO, "%s (enter)\n", __func__);

    struct v4l2_control ctrl;
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    
    if (pAR0144Ctx->motor_fd <= 0) {
        return RET_NOTSUPP;
    }

    memset(&ctrl, 0, sizeof(ctrl));
    if (pAR0144Ctx->focus_mode & (1 << (pPos->mode))) {
        if (pPos->mode == ISI_FOUCUS_MODE_ABSOLUTE) {
            ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
        } else if (pPos->mode == ISI_FOUCUS_MODE_RELATIVE) {
            ctrl.id = V4L2_CID_FOCUS_RELATIVE;
        }
    } else {
        TRACE(AR0144_ERROR, "%s invalid Focus mode %d\n", __func__, pPos->mode);
        return RET_FAILURE;
    }

    if (ioctl(pAR0144Ctx->motor_fd, VIDIOC_G_CTRL, &ctrl) < 0) {
        TRACE(AR0144_ERROR, "%s get moto pos failed\n", __func__);
        return RET_FAILURE;
    }

    pPos->Pos = ctrl.value;
    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT AR0144_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( AR0144_INFO, "%s (enter)\n", __func__);

    struct v4l2_control ctrl;
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    if (pAR0144Ctx->motor_fd <= 0) {
        return RET_NOTSUPP;
    }

    memset(&ctrl, 0, sizeof(ctrl));
    if (pAR0144Ctx->focus_mode & (1 << (pPos->mode))) {
        if (pPos->mode == ISI_FOUCUS_MODE_ABSOLUTE) {
            ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
            ctrl.value = pPos->Pos;
        } else if (pPos->mode == ISI_FOUCUS_MODE_RELATIVE) {
            ctrl.id = V4L2_CID_FOCUS_RELATIVE;
            ctrl.value = pPos->Pos;
        }
    } else {
        TRACE(AR0144_ERROR, "%s invalid Focus mode %d\n", __func__, pPos->mode);
        return RET_FAILURE;
    }

    if (ioctl(pAR0144Ctx->motor_fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        TRACE(AR0144_ERROR, "%s set moto pos failed\n", __func__);
        return RET_FAILURE;
    }

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT AR0144_IsiGetFocusCalibrateIss(IsiSensorHandle_t handle, IsiFoucsCalibAttr_t *pFocusCalib)
{
    TRACE( AR0144_INFO, "%s (enter)\n", __func__);
    struct v4l2_queryctrl qctrl;
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;
    RESULT result = RET_SUCCESS;

    if (pAR0144Ctx->motor_fd <= 0) {
        return RET_NOTSUPP;
    }

    memset(&qctrl, 0, sizeof(qctrl));
    qctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
    if (ioctl(pAR0144Ctx->motor_fd, VIDIOC_QUERYCTRL, &qctrl) >= 0) {
            pAR0144Ctx->focus_mode |= 1 << ISI_FOUCUS_MODE_ABSOLUTE;
            pFocusCalib->minPos = qctrl.minimum;
            pFocusCalib->maxPos = qctrl.maximum;
            pFocusCalib->minStep = qctrl.step;
    } else {
        qctrl.id = V4L2_CID_FOCUS_RELATIVE;
        if (ioctl(pAR0144Ctx->motor_fd, VIDIOC_QUERYCTRL, &qctrl) >= 0) {
                pAR0144Ctx->focus_mode |= 1 << ISI_FOUCUS_MODE_RELATIVE;
                pFocusCalib->minPos = qctrl.minimum;
                pFocusCalib->maxPos = qctrl.maximum;
                pFocusCalib->minStep = qctrl.step;
        } else {
            result = RET_FAILURE;
        }
    }

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);
    return result;
}

static RESULT AR0144_IsiGetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t *pExposure)
{
    TRACE( AR0144_INFO, "%s (enter)\n", __func__);
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    if (pAR0144Ctx->AEStartExposure == 0) {
        pAR0144Ctx->AEStartExposure =
            (uint64_t)pAR0144Ctx->CurMode.ae_info.start_exposure *
            pAR0144Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;
           
    }
    *pExposure =  pAR0144Ctx->AEStartExposure;
    TRACE(AR0144_INFO, "%s:get start exposure %d\n", __func__, pAR0144Ctx->AEStartExposure);

    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT AR0144_IsiSetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t exposure)
{
    TRACE( AR0144_INFO, "%s (enter)\n", __func__);
    AR0144_Context_t *pAR0144Ctx = (AR0144_Context_t *) handle;

    pAR0144Ctx->AEStartExposure = exposure;
    TRACE(AR0144_INFO, "set start exposure %d\n", __func__,pAR0144Ctx->AEStartExposure);
    TRACE(AR0144_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}
#endif

RESULT AR0144_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    TRACE( AR0144_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor == NULL)
        return RET_NULL_POINTER;
     pIsiSensor->pszName                         = SensorName;
     pIsiSensor->pIsiSensorSetPowerIss           = AR0144_IsiSensorSetPowerIss;
     pIsiSensor->pIsiCreateSensorIss             = AR0144_IsiCreateSensorIss;
     pIsiSensor->pIsiReleaseSensorIss            = AR0144_IsiReleaseSensorIss;
     pIsiSensor->pIsiRegisterReadIss             = AR0144_IsiRegisterReadIss;
     pIsiSensor->pIsiRegisterWriteIss            = AR0144_IsiRegisterWriteIss;
     pIsiSensor->pIsiGetSensorModeIss            = AR0144_IsiGetSensorModeIss;
     pIsiSensor->pIsiSetSensorModeIss            = AR0144_IsiSetSensorModeIss;
     pIsiSensor->pIsiQuerySensorIss              = AR0144_IsiQuerySensorIss;
     pIsiSensor->pIsiGetCapsIss                  = AR0144_IsiGetCapsIss;
     pIsiSensor->pIsiSetupSensorIss              = AR0144_IsiSetupSensorIss;
     pIsiSensor->pIsiGetSensorRevisionIss        = AR0144_IsiGetSensorRevisionIss;
     pIsiSensor->pIsiCheckSensorConnectionIss    = AR0144_IsiCheckSensorConnectionIss;
     pIsiSensor->pIsiSensorSetStreamingIss       = AR0144_IsiSensorSetStreamingIss;
     pIsiSensor->pIsiGetAeInfoIss                = AR0144_IsiGetAeInfoIss;
     pIsiSensor->pIsiGetIntegrationTimeIss       = AR0144_IsiGetIntegrationTimeIss;
     pIsiSensor->pIsiSetIntegrationTimeIss       = AR0144_IsiSetIntegrationTimeIss;
     pIsiSensor->pIsiGetGainIss                  = AR0144_IsiGetGainIss;
     pIsiSensor->pIsiSetGainIss                  = AR0144_IsiSetGainIss;
     pIsiSensor->pIsiGetSensorFpsIss             = AR0144_IsiGetSensorFpsIss;
     pIsiSensor->pIsiSetSensorFpsIss             = AR0144_IsiSetSensorFpsIss;
     pIsiSensor->pIsiSetSensorAfpsLimitsIss      = AR0144_IsiSetSensorAfpsLimitsIss;
     pIsiSensor->pIsiGetSensorIspStatusIss       = AR0144_IsiGetSensorIspStatusIss;
#ifndef ISI_LITE
    pIsiSensor->pIsiSensorSetWBIss               = AR0144_IsiSensorSetWBIss;
    pIsiSensor->pIsiActivateTestPatternIss       = AR0144_IsiSetTestPatternIss;
    pIsiSensor->pIsiFocusSetupIss                = AR0144_IsiFocusSetupIss;
    pIsiSensor->pIsiFocusReleaseIss              = AR0144_IsiFocusReleaseIss;
    pIsiSensor->pIsiFocusSetIss                  = AR0144_IsiFocusSetIss;
    pIsiSensor->pIsiFocusGetIss                  = AR0144_IsiFocusGetIss;
    pIsiSensor->pIsiGetFocusCalibrateIss         = AR0144_IsiGetFocusCalibrateIss;
    pIsiSensor->pIsiSetAeStartExposureIss        = AR0144_IsiSetAeStartExposureIs;
    pIsiSensor->pIsiGetAeStartExposureIss        = AR0144_IsiGetAeStartExposureIs;
#endif
    TRACE( AR0144_INFO, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x2770,
    .pIsiHalQuerySensor = AR0144_IsiHalQuerySensorIss,
    .pfIsiGetSensorIss = AR0144_IsiGetSensorIss,
};
