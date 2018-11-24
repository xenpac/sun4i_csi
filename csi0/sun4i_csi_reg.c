/*
 * drivers/media/video/sun4i_csi/csi0/sun4i_csi_reg.c
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Sun4i Camera register read/write interface
 * Author:raymonxiu
*/
#include <linux/io.h>
#include <linux/delay.h>

#include "../include/sun4i_csi_core.h"
#include "sun4i_csi_reg.h"

/* open module */
void bsp_csi_open(struct csi_camera *pcam)
{
    W(pcam->regs+CSI_REG_EN, 0x1);
}

void bsp_csi_close(struct csi_camera *pcam)
{
    C(pcam->regs+CSI_REG_EN, 0X1 << 0);
}

/* configure */
void bsp_csi_configure(struct csi_camera *pcam,struct csi_config *mode)
{
    u32 t;
    W(pcam->regs+CSI_REG_CONF, mode->input_fmt << 20 | /* [21:20] */
      mode->output_fmt<< 16 | /* [18:16] */
      mode->field_sel << 10 | /* [11:10] */
      mode->seq       << 8  | /* [9:8] */
      mode->vref      << 2  | /* [2] */
      mode->href      << 1  | /* [1] */
      mode->clock     << 0    /* [0] */
     );

    t = R(pcam->regs+CSI_REG_CONF);

}

/* buffer */
u32 static inline bsp_csi_get_buffer_address(struct csi_camera *pcam,enum csi_buf_e buf)
{
    u32 t;
    t = R(pcam->regs+CSI_REG_BUF_0_A + (buf<<2));
    return t;
}

void bsp_csi_double_buffer_enable(struct csi_camera *pcam)
{
    S(pcam->regs+CSI_REG_BUF_CTRL, 0X1<<0);
}

void bsp_csi_double_buffer_disable(struct csi_camera *pcam)
{
    C(pcam->regs+CSI_REG_BUF_CTRL, 0X1<<0);
}

void static inline bsp_csi_double_buffer_select_next(struct csi_camera *pcam,enum csi_double_buf_e type)
{
    if (CSI_BUF_A == type)
    {
        C(pcam->regs+CSI_REG_BUF_CTRL, 0x1<<2);
    }
    else
    {
        S(pcam->regs+CSI_REG_BUF_CTRL, 0x1<<2);
    }
}

void static inline bsp_csi_double_buffer_get_status(struct csi_camera *pcam,struct csi_double_buf_status * status)
{
    u32 t;
    t = R(pcam->regs+CSI_REG_BUF_CTRL);
    status->enable = t&0x1;
    status->cur  = (enum csi_double_buf_e)(t&(0x1<<1));
    status->next = (enum csi_double_buf_e)(t&(0x1<<2));

}

/* capture */
void bsp_csi_capture_video_start(struct csi_camera *pcam)
{
    S(pcam->regs+CSI_REG_CTRL, 0X1<<1);
}

void bsp_csi_capture_video_stop(struct csi_camera *pcam)
{
    C(pcam->regs+CSI_REG_CTRL, 0X1<<1);
}

void bsp_csi_capture_picture(struct csi_camera *pcam)
{
    S(pcam->regs+CSI_REG_CTRL, 0X1<<0);
}

void bsp_csi_capture_get_status(struct csi_camera *pcam,struct csi_capture_status * status)
{
    u32 t;
    t = R(pcam->regs+CSI_REG_STATUS);
    status->picture_in_progress = t&0x1;
    status->video_in_progress   = (t>>1)&0x1;
}

/* set window size 
length_h = number of pixelclocks per line, 
length_v = number of lines = height
buf_length_h = fifo length, the max of the three fifos in planar mode, or the bytesperline

*/
void bsp_csi_set_size(struct csi_camera *pcam, u32 length_h, u32 length_v, u32 buf_length_h)
{
    /* make sure yuv422 input 2 byte(clock) output 1 pixel */
    u32 t;

    t = R(pcam->regs+CSI_REG_RESIZE_H);
    t = (t&0x0000ffff)|(length_h<<16);
    W(pcam->regs+CSI_REG_RESIZE_H, t);

    t = R(pcam->regs+CSI_REG_RESIZE_H);
    t = (t&0x0000ffff)|(length_v<<16);
    W(pcam->regs+CSI_REG_RESIZE_V, t);

    W(pcam->regs+CSI_REG_BUF_LENGTH, buf_length_h);
}


/* offset */
void bsp_csi_set_offset(struct csi_camera *pcam,u32 start_h, u32 start_v)
{
    u32 t;

    t = R(pcam->regs+CSI_REG_RESIZE_H);
    t = (t&0xffff0000)|start_h;
    W(pcam->regs+CSI_REG_RESIZE_H, t);

    t = R(pcam->regs+CSI_REG_RESIZE_V);
    t = (t&0xffff0000)|start_v;
    W(pcam->regs+CSI_REG_RESIZE_V, t);
}


/* interrupt */
void bsp_csi_int_enable(struct csi_camera *pcam,enum csi_int_e interrupt)
{
    S(pcam->regs+CSI_REG_INT_EN, interrupt);
}

void bsp_csi_int_disable(struct csi_camera *pcam,enum csi_int_e interrupt)
{
    C(pcam->regs+CSI_REG_INT_EN, interrupt);
}

void static inline bsp_csi_int_get_status(struct csi_camera *pcam,struct csi_int_status * status)
{
    u32 t;
    t = R(pcam->regs+CSI_REG_INT_STATUS);

    status->capture_done     = t&CSI_INT_CAPTURE_DONE;
    status->frame_done       = t&CSI_INT_FRAME_DONE;
    status->buf_0_overflow   = t&CSI_INT_BUF_0_OVERFLOW;
    status->buf_1_overflow   = t&CSI_INT_BUF_1_OVERFLOW;
    status->buf_2_overflow   = t&CSI_INT_BUF_2_OVERFLOW;
    status->protection_error = t&CSI_INT_PROTECTION_ERROR;
    status->hblank_overflow  = t&CSI_INT_HBLANK_OVERFLOW;
    status->vsync_trig		 = t&CSI_INT_VSYNC_TRIG;

}
