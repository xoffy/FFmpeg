/*
 * Copyright (c) 2019 Valery Khabarov / Timiriliyev
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
 * Filter to simulate so-called "SECAM fires"
 *
 * @see https://www.radiomuseum.org/forum/secam_feuer_es_brennt_wirklich.html
 */

#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/lfg.h"
#include "libswscale/swscale.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

#define COLOR_CLAMP(x) ((x) < 0 ? 0 : ((x) > 255 ? 255 : (x)))

typedef struct {
    AVLFG lfg;
    double *rnd;
} RandomizeContext;

typedef struct {
    struct SwsContext *sws;
    uint8_t *delta;
    uint8_t *miniature;
} DeltaContext;

typedef struct {
    int lw, lh;
    int cw, ch;
    uint8_t *cbuf;
    DeltaContext dc;
} FrameContext;

typedef struct SecamizeContext {
    const AVClass *class;
    
    double threshold;
    double shift;
    double reception;

    RandomizeContext rc;
    FrameContext fc;
} SecamizeContext;

#define OFFSET(x) offsetof(SecamizeContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption secamize_options[] = {
    { "threshold", "set saturation threshold", OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=0.024}, 0, 1, FLAGS },
    { "shift", "set simulated RF shift", OFFSET(shift), AV_OPT_TYPE_DOUBLE, {.dbl=0.024}, 0, 1, FLAGS },
    { "reception", "set simulated RF reception", OFFSET(reception), AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(secamize);

#define MAX_NOISE_VERTICES 4096
#define MAX_NOISE_VERTICES_MASK (MAX_NOISE_VERTICES - 1)

static av_cold int noise_init(RandomizeContext *rand_ctx)
{
    int i;

    rand_ctx->rnd = av_malloc(MAX_NOISE_VERTICES * sizeof(double));

    if (!rand_ctx->rnd)
        return AVERROR(ENOMEM);
    
    av_lfg_init(&rand_ctx->lfg, 0xDEADCAFE);

    for (i = 0; i < MAX_NOISE_VERTICES; i++)
        rand_ctx->rnd[i] = av_lfg_get(&rand_ctx->lfg) / (double)(UINT_MAX + 1.0);

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    SecamizeContext *s = ctx->priv;
    int ret;

    if ((ret = noise_init(&s->rc)) < 0)
        return ret;

    return 0;
}

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV410P,
        AV_PIX_FMT_NONE
    };
    AVFilterFormats *fmts_list;

    fmts_list = ff_make_format_list(pix_fmts);

    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static int config_props(AVFilterLink *inlink)
{
    int c;
    AVFilterContext *ctx = inlink->dst;
    SecamizeContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);

    s->fc.lw = inlink->w;
    s->fc.lh = inlink->h;
    s->fc.cw = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
    s->fc.ch = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);

    s->fc.cbuf = av_malloc(sizeof(uint8_t) * s->fc.cw * s->fc.ch);
    if (!s->fc.cbuf) {
        return AVERROR(ENOMEM);
    }

    s->fc.dc.sws = sws_getContext(s->fc.lw, s->fc.lh, AV_PIX_FMT_GRAY8,
        s->fc.cw, s->fc.ch, AV_PIX_FMT_GRAY8, SWS_FAST_BILINEAR,
        NULL, NULL, NULL);
    s->fc.dc.delta = av_malloc(sizeof(uint8_t) * s->fc.cw * s->fc.ch);
    s->fc.dc.miniature = av_malloc(sizeof(uint8_t) * s->fc.cw * s->fc.ch);

    if (!s->fc.dc.sws || !s->fc.dc.delta || !s->fc.dc.miniature)
        return AVERROR(ENOMEM);

    return 0;
}

static void make_delta(SecamizeContext *s,
                         const uint8_t *luma, int luma_lsz)
{
    uint8_t *miniature = s->fc.dc.miniature;
    uint8_t *delta = s->fc.dc.delta;
    struct SwsContext *sws = s->fc.dc.sws;
    int cx, cy;

    const uint8_t *const luma_ar[4]           = { luma };
    int                  luma_lsz_ar[4]       = { luma_lsz };
    uint8_t	            *miniature_ar[4]      = { miniature };
    int                  miniature_lsz_ar[4]  = { s->fc.cw };

    sws_scale(sws, luma_ar, luma_lsz_ar, 0, s->fc.lh,
        miniature_ar, miniature_lsz_ar);

    for (cy = 0; cy < s->fc.ch; cy++) {
        delta[0] = miniature[0];
        for (cx = 1; cx < s->fc.cw; cx++)
            delta[cx] = abs(miniature[cx - 1] - miniature[cx]) / 2;
        delta += s->fc.cw;
        miniature += s->fc.cw;
    }
}

static double noise2(RandomizeContext *rnd, double x, double amplitude, double scale)
{
    double xs, t, ts, y;
    int xf, xmin, xmax;
    
    xs = x * scale;
    xf = floor(xs);
    t = xs - xf;
    ts = t * t * (3 - 2 * t);
    xmin = xf & MAX_NOISE_VERTICES_MASK;
    xmax = (xmin + 1) & MAX_NOISE_VERTICES_MASK;
    y = rnd->rnd[xmin] * (1 - ts) + rnd->rnd[xmax] * ts;
    
    return y * amplitude;
}

static void chroma_noise(SecamizeContext *s,
                              uint8_t *dst, int dst_lsz,
                        const uint8_t *src, int src_lsz)
{
    const double amp = 36.0;
    const double scale = 0.48;
    int cx, cy;

    for (cy = 0; cy < s->fc.ch; cy++) {
        unsigned int r = av_lfg_get(&s->rc.lfg);
        for (cx = 0; cx < s->fc.cw; cx++) {
            double v = noise2(&s->rc, r + s->fc.cw * cy + cx, amp, scale)
                     - (amp * 0.5);
            dst[cx] = COLOR_CLAMP(src[cx] + v);
        }
        dst += dst_lsz;
        src += src_lsz;
    }
}

static double frand(RandomizeContext *ctx)
{
    return av_lfg_get(&ctx->lfg) / (double)UINT_MAX;
}

static void burn(SecamizeContext *s,
                 uint8_t *dst, int dst_lsz,
           const uint8_t *src, int src_lsz)
{
    const uint8_t *delta = s->fc.dc.delta;
    const double threshold = s->threshold;
    int cx, cy;

    for (cy = 0; cy < s->fc.ch; cy++) {
        for (cx = 0; cx < s->fc.cw; cx++) {
            int point;
            int gain, hs;
            double ethrshld, fire, d;

            dst[cx] = src[cx];

            if (cx == 0) {
                point = -1;
                continue;
            }

            d = delta[cx] / 256.0;
            gain = point == -1 ? 0x0C * 1.5 : cx - point;
            hs = 0x0C + frand(&s->rc) * (0x0C * 10.5);
            ethrshld = threshold + (frand(&s->rc) * threshold - threshold * 0.5);

            if ((d * frand(&s->rc) > ethrshld) && (gain > hs))
                point = cx;

            if (point < 0)
                continue;

            fire = 320.0 / (gain + 1.0) - 1.0;
            if (fire < 0.0) {
                point = -1;
                continue;
            }

            dst[cx] = COLOR_CLAMP(dst[cx] + fire);
        }
        dst += dst_lsz;
        src += src_lsz;
        delta += s->fc.cw;
    }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    SecamizeContext *s = ctx->priv;
    int c, direct = 0;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        direct = 1;
        out = in;
    } else {
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    /* luma remains unchanged */
    av_image_copy_plane(out->data[0], out->linesize[0],
                        in->data[0], in->linesize[0],
                        s->fc.lw, s->fc.lh);

    make_delta(s, in->data[0], in->linesize[0]);

    for (c = 0; c < 2; c++) {
        int p = c + 1;
        uint8_t *cbuf = s->fc.cbuf;

        chroma_noise(s, cbuf, s->fc.cw, in->data[p], in->linesize[p]);
        burn(s, out->data[p], out->linesize[p], cbuf, s->fc.cw);
    }

    if (!direct)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    int c;
    SecamizeContext *s = ctx->priv;

    av_freep(&s->fc.cbuf);

    av_freep(&s->fc.dc.delta);
    av_freep(&s->fc.dc.miniature);
    sws_freeContext(s->fc.dc.sws);

    av_freep(&s->rc.rnd);
}

static const AVFilterPad secamize_inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_VIDEO,
        .config_props   = config_props,
        .filter_frame   = filter_frame,
    },
    { NULL }
};

static const AVFilterPad secamize_outputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter ff_vf_secamize = {
    .name               = "secamize",
    .description        = NULL_IF_CONFIG_SMALL("Let it burn."),
    .priv_size          = sizeof(SecamizeContext),
    .init               = init,
    .uninit             = uninit,
    .query_formats      = query_formats,
    .inputs             = secamize_inputs,
    .outputs            = secamize_outputs,
    .priv_class         = &secamize_class,
    .flags              = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
};
