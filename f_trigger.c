/*
 * Copyright (c) 2017 Ramiro Polla
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * filter for triggering which frame passes in the filterchain
 */

#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <termios.h>  // POSIX terminal control definitions

#ifndef CRTSCTS
#define CRTSCTS 020000000000
#endif

#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "internal.h"

typedef struct TriggerContext {
    const AVClass *class;
    const char *filename;   ///< serial port path
    AVRational framerate;   ///< target framerate
    int wait_for_serial;    ///< time to wait for serial port (microseconds)
    int expected;           ///< signal to start trigger sequence (0 or 1)
    int delay;              ///< delay between signal and trigger (frames)
    int frames_out;         ///< number of frames on output
    int frames_in;          ///< number of frames on input
    int fd;                 ///< serial file descriptor
    int test_mode;          ///< test mode
    int get_frame_in;
} TriggerContext;

static int trigger_filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    TriggerContext *tctx = ctx->priv;
    uint8_t code;
    int n;

    /* increment input counter */
    tctx->frames_in++;

    /* read serial port to check for sensor signal */
    n = read(tctx->fd, &code, 1);
    if ( n == 1 && code == ('0' + tctx->expected) )
    {
        av_log(tctx, AV_LOG_DEBUG,
               "got '%c' signal (in %d); setting delay to %d frames\n",
               code, tctx->frames_in, tctx->delay);
        tctx->get_frame_in = tctx->delay;
    }

    if ( tctx->test_mode )
    {
        /* when on test mode, we pass all frames */
        tctx->get_frame_in = 0;
        /* add frame metadata on signals */
        if ( n == 1 )
        {
            char value[2];
            value[0] = code;
            value[1] = '\0';
            av_dict_set(&frame->metadata, "signal", value, 0);
        }
    }

    if ( tctx->get_frame_in > 0 )
    {
        /* decrement delay counter */
        tctx->get_frame_in--;
    }
    else if ( tctx->get_frame_in == 0 )
    {
        /* capture frame */
        if ( !tctx->test_mode )
            frame->pts = tctx->frames_out; /* set output frame number */
        tctx->frames_out++;
        av_log(tctx, AV_LOG_DEBUG,
               "capturing frame (in %d out %d)\n",
               tctx->frames_in, tctx->frames_out);
        tctx->get_frame_in--;
        return ff_filter_frame(outlink, frame);
    }

    /* skip frame */
    av_frame_free(&frame);
    return 0;
}

static int trigger_config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    TriggerContext *tctx = ctx->priv;

    if ( !tctx->test_mode )
    {
        /* when not on test mode, we output a fixed frame rate */
        outlink->time_base  = av_inv_q(tctx->framerate);
        outlink->frame_rate = tctx->framerate;
    }

    return 0;
}

#define OFFSET(x) offsetof(TriggerContext, x)
#define V AV_OPT_FLAG_VIDEO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
static const AVOption trigger_options[] = {
    { "path",   "Serial port file name",                            OFFSET(filename),        AV_OPT_TYPE_STRING,     { .str = "/dev/ttyACM0" }, .flags = V|F },
    { "wait",   "Time to wait for serial port (microseconds)",      OFFSET(wait_for_serial), AV_OPT_TYPE_INT,        { .i64 = 0 }, 0, INT_MAX, V|F },
    { "signal", "Signal to start trigger sequence (0 or 1)",        OFFSET(expected),        AV_OPT_TYPE_INT,        { .i64 = 0 }, 0, 1, V|F },
    { "delay",  "Delay between signal and trigger (frames)",        OFFSET(delay),           AV_OPT_TYPE_INT,        { .i64 = 1 }, 0, INT_MAX, V|F },
    { "fps",    "A string describing the desired output framerate", OFFSET(framerate),       AV_OPT_TYPE_VIDEO_RATE, { .str = "24" }, 0, INT_MAX, V|F },
    { "test",   "Run in test mode (pass all frames, record signal", OFFSET(test_mode),       AV_OPT_TYPE_INT,        { .i64 = 0 }, 0, 1, V|F },
    { NULL }
};

AVFILTER_DEFINE_CLASS(trigger);

static int init_serial(TriggerContext *tctx, const char *fname)
{
    // based on:
    //
    // arduino-serial-lib -- simple library for reading/writing serial ports
    //
    // 2006-2013, Tod E. Kurt, http://todbot.com/blog/

    struct termios toptions;
    int fd;

    fd = open(fname, O_RDWR | O_NONBLOCK);

    if ( fd == -1 )
    {
        av_log(tctx, AV_LOG_ERROR, "Could not open serial at '%s'\n", fname);
        return AVERROR(EIO);
    }

    if ( tcgetattr(fd, &toptions) < 0 )
    {
        av_log(tctx, AV_LOG_ERROR, "tcgetattr() failed\n");
        return AVERROR(EIO);
    }

    // set baud rate
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0 )
    {
        av_log(tctx, AV_LOG_ERROR, "tcsetattr() failed\n");
        return AVERROR(EIO);
    }

    return fd;
}

static av_cold int trigger_init(AVFilterContext *ctx)
{
    TriggerContext *tctx = ctx->priv;
    int n;

    /* initialise serial */
    int fd = init_serial(tctx, tctx->filename);
    if ( fd < 0 )
        return fd;
    tctx->fd = fd;

    if ( tctx->wait_for_serial != 0 )
    {
        av_log(tctx, AV_LOG_INFO, "waiting for arduino to reboot\n");
        /* wait for the arduino to reboot */
        av_usleep(tctx->wait_for_serial);
    }

    av_log(tctx, AV_LOG_INFO, "turning camera on\n");

    /* turn camera on */
    n = write(fd, "1", 1);
    if ( n != 1 )
    {
        av_log(tctx, AV_LOG_ERROR, "turning camera on failed\n");
        return AVERROR(EIO);
    }

    /* reset frame delay */
    tctx->get_frame_in = -1;

    return 0;
}

static av_cold void trigger_uninit(AVFilterContext *ctx)
{
    TriggerContext *tctx = ctx->priv;

    if ( tctx->fd != 0 )
    {
        int n;
        av_log(tctx, AV_LOG_INFO, "turning camera off\n");
        /* turn camera off */
        n = write(tctx->fd, "0", 1);
        if ( n != 1 )
            av_log(tctx, AV_LOG_ERROR, "turning camera off failed\n");
        close(tctx->fd);
    }
}

static const AVFilterPad avfilter_vf_trigger_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = trigger_filter_frame,
    },
    { NULL }
};

static const AVFilterPad avfilter_vf_trigger_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = trigger_config_output,
    },
    { NULL }
};

AVFilter ff_vf_trigger = {
    .name        = "trigger",
    .description = NULL_IF_CONFIG_SMALL("Trigger video frames to pass in output."),
    .init        = trigger_init,
    .uninit      = trigger_uninit,
    .priv_size   = sizeof(TriggerContext),
    .priv_class  = &trigger_class,
    .inputs      = avfilter_vf_trigger_inputs,
    .outputs     = avfilter_vf_trigger_outputs,
};
