// Drone OS — ドローンショーシミュレータ   (C++ / WASM)
//
// 写真から拾った光点(tools/mkpoints.cpp)を目標に、機体を飛ばす。
//
// 本物のドローンショーの「あのゆったりした動き」と「全機が同時にピタッと揃う」感じは、
// 演出ではなく制約から出てくる。
//
//  1) 最小ジャーク軌道
//     両端で速度も加速度もゼロになる軌道を使う。機体に無理をさせない標準的なやり方で、
//         s(τ) = 10τ³ − 15τ⁴ + 6τ⁵      (τ = t/T, 0→1)
//     これだと最大速度と最大加速度が距離 d と所要時間 T で決まる:
//         v_max = 1.875 d / T,    a_max = 5.7735 d / T²
//
//  2) 全機同時到着
//     T を「いちばん遠い機体が速度と加速度の上限を守れる最小値」に取り、
//     全機がその同じ T を使う。だから近い機体はゆっくり動き、全員が同時に到着する。
//     振り付けたのではなく、制約から出てくる。
//
//  3) 割り当て
//     どの機体がどの点へ行くかは、総移動距離が短くなるように交換で改善する。
//     こうすると軌跡が交差しにくくなり、実際の運用と同じく衝突が起きにくい。
//
// 計算も描画もすべて C++。olive.c は文字だけに使っている。
#define OLIVEC_IMPLEMENTATION
#include "olive.c"
#include "show_pts.h"
#include "show2_pts.h"
// 好きな絵から作った編隊(tools/mkart.cpp が生成)。
// 手元に art_pts.h があるときだけ有効になる。
#if __has_include("art_pts.h")
  #include "art_pts.h"
  #define HAVE_ART 1
#else
  static const int ART_N = 1;
  static const float ART_PTS[][5] = {{0.5f,0.5f,255,255,255}};
  #define HAVE_ART 0
#endif
#include <vector>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <algorithm>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define KEEP EMSCRIPTEN_KEEPALIVE
#else
#define KEEP
#endif

static const int FW = 1024, FH = 640;
static const int SUBSTEPS = 2;
static const double DT = 1.0 / (60.0 * SUBSTEPS);

// 機体の性能。実機のショー用ドローンはこのあたり
static const double VMAX = 6.0;      // [m/s]
static const double AMAX = 3.5;      // [m/s^2]
static const double SEP  = 2.0;      // 最小間隔 [m]

// 会場。編隊は幅160m、高さ90〜200m あたりに作る
static const double FIELD_W  = 160.0;
static const double FORM_SIZE = 150.0;   // 編隊の長辺 [m]
static const double FORM_Y    = 120.0;   // 編隊の中心の高さ [m]
static const double CAMD    = 330.0;   // 観客からの距離 [m]
static const double CAMY    = 118.0;    // カメラが向いている高さ [m](やや見上げる)
static double FOC = 0.0;

// ---------------------------------------------------------------- 乱数
static uint32_t rng = 987654321u;
static inline double rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return (rng & 0xFFFFFF) / (double)0x1000000; }
static inline double rnds() { return rnd() * 2.0 - 1.0; }
static inline int clampi(int v, int a, int b) { return v < a ? a : (v > b ? b : v); }
static inline uint32_t rgb(int r, int g, int b) {
    return 0xFF000000u | ((uint32_t)clampi(b,0,255) << 16) | ((uint32_t)clampi(g,0,255) << 8) | (uint32_t)clampi(r,0,255);
}

static inline void project(double x, double y, double z, double& sx, double& sy, double& sc) {
    double zz = z + CAMD;
    if (zz < 1.0) zz = 1.0;
    sc = FOC / zz;
    sx = FW * 0.5 + x * sc;
    sy = FH * 0.5 - (y - CAMY) * sc;
}

// ---------------------------------------------------------------- 機体
struct Drone {
    double x, y, z;          // 現在位置 [m]
    double ax, ay, az;       // 出発点
    double bx, by, bz;       // 目標点
    float  r, g, b;          // LED
    float  r0, g0, b0;       // 前の色(遷移中に混ぜる)
    double jx, jy, jz;       // 位置保持のゆらぎ
    double jvx, jvy, jvz;
};
static std::vector<Drone> drones;
static std::vector<int>   perm;      // 割り当て

// 編隊
enum { F_GROUND = 0, F_SHOW, F_SHOW2, F_ART, F_SPHERE, F_RING, F_N };
static int  curForm = F_GROUND;
static int  phase = 0;               // 0=待機 1=移動中
static double moveT = 1.0, moveElapsed = 0.0;
static double holdT = 0.0;

static double simTime = 0.0;
static std::vector<uint32_t> px;
static std::vector<float>    glow;   // 加算バッファ

static double p_speed = 1.0;
static double p_glow  = 1.0;
static double p_hud   = 1.0;
static double p_auto  = 1.0;

// 統計(実測)
static double statVmax = 0, statAmax = 0, statSep = 1e9;

// ---------------------------------------------------------------- 編隊の座標
static void formation_pos(int form, int i, int n, double& x, double& y, double& z,
                          float& r, float& g, float& b) {
    if (form == F_SHOW || form == F_SHOW2 || form == F_ART) {
        // 写真や絵から拾った点。外接矩形で正規化して会場いっぱいに広げる
        int which = (form == F_SHOW) ? 0 : (form == F_SHOW2 ? 1 : 2);
        const float (*P)[5] = (which == 0) ? SHOW_PTS : (which == 1) ? SHOW2_PTS : ART_PTS;
        int NP = (which == 0) ? SHOW_N : (which == 1) ? SHOW2_N : ART_N;
        static double bb[3][4] = {{1e9,-1e9,1e9,-1e9},{1e9,-1e9,1e9,-1e9},{1e9,-1e9,1e9,-1e9}};
        double* B = bb[which];
        if (B[1] < B[0]) {
            for (int k = 0; k < NP; ++k) {
                B[0] = std::min(B[0], (double)P[k][0]); B[1] = std::max(B[1], (double)P[k][0]);
                B[2] = std::min(B[2], (double)P[k][1]); B[3] = std::max(B[3], (double)P[k][1]);
            }
        }
        int k = i % NP;
        // 縦横のどちらが長くても画面に収まるよう、長いほうを基準に縮める。
        // 縦の中心はカメラが向いている高さに合わせる(固定値だと縦長の絵が見切れる)
        double sc = FORM_SIZE / std::max(B[1] - B[0], B[3] - B[2]);
        double cx = (B[0] + B[1]) * 0.5, cy = (B[2] + B[3]) * 0.5;
        x = ((double)P[k][0] - cx) * sc;
        y = FORM_Y - ((double)P[k][1] - cy) * sc;
        z = 0.0;                                 // 絵を見せる編隊は観客に正対した平面
        r = P[k][2] / 255.0f; g = P[k][3] / 255.0f; b = P[k][4] / 255.0f;
        return;
    }
    if (form == F_GROUND) {
        int cols = 32;
        int cx = i % cols, cy = i / cols;
        x = (cx - (cols - 1) * 0.5) * 3.0;
        z = (cy - 7.0) * 3.0;
        y = 0.6;
        r = 0.25f; g = 0.25f; b = 0.30f;
        return;
    }
    if (form == F_SPHERE) {
        double yy = 1.0 - 2.0 * (i + 0.5) / n;
        double rr = sqrt(std::max(0.0, 1.0 - yy * yy));
        double ph = i * 2.39996322973;
        double R = 55.0;
        x = R * rr * cos(ph); y = FORM_Y + R * yy; z = R * rr * sin(ph);
        double t = 0.5 + 0.5 * yy;
        r = (float)(0.15 + 0.55 * t); g = (float)(0.55 - 0.25 * t); b = (float)(0.95 - 0.2 * t);
        return;
    }
    // F_RING — 傾いた輪
    double th = 6.2831853 * i / n;
    double R = 70.0;
    x = R * cos(th);
    y = FORM_Y + R * sin(th) * 0.42;
    z = R * sin(th) * 0.90;
    r = 1.0f; g = (float)(0.35 + 0.5 * (0.5 + 0.5 * sin(th * 3))); b = 0.15f;
}

// ---------------------------------------------------------------- 割り当て
// 総移動距離が短くなるよう、2機ずつ交換して改善する。
// 軌跡が交差しにくくなり、実際の運用と同じく衝突しにくい配置になる。
static void assign(int form) {
    int n = (int)drones.size();
    std::vector<double> tx(n), ty(n), tz(n);
    std::vector<float> tr(n), tg(n), tb(n);
    for (int i = 0; i < n; ++i) formation_pos(form, i, n, tx[i], ty[i], tz[i], tr[i], tg[i], tb[i]);

    // まず上下→左右の順に並べて素直に対応づける
    std::vector<int> a(n), b(n);
    for (int i = 0; i < n; ++i) { a[i] = i; b[i] = i; }
    std::sort(a.begin(), a.end(), [](int p, int q) {
        if (fabs(drones[p].y - drones[q].y) > 1e-6) return drones[p].y < drones[q].y;
        return drones[p].x < drones[q].x; });
    std::sort(b.begin(), b.end(), [&](int p, int q) {
        if (fabs(ty[p] - ty[q]) > 1e-6) return ty[p] < ty[q];
        return tx[p] < tx[q]; });
    perm.assign(n, 0);
    for (int i = 0; i < n; ++i) perm[a[i]] = b[i];

    auto d2 = [&](int di, int ti) {
        double dx = drones[di].x - tx[ti], dy = drones[di].y - ty[ti], dz = drones[di].z - tz[ti];
        return dx*dx + dy*dy + dz*dz;
    };
    for (int it = 0; it < n * 60; ++it) {
        int i = (int)(rnd() * n), j = (int)(rnd() * n);
        if (i == j) continue;
        if (d2(i, perm[i]) + d2(j, perm[j]) > d2(i, perm[j]) + d2(j, perm[i]) + 1e-9)
            std::swap(perm[i], perm[j]);
    }

    // 出発点と目標点を確定し、所要時間を決める
    double dmax = 0;
    for (int i = 0; i < n; ++i) {
        Drone& d = drones[i];
        d.ax = d.x; d.ay = d.y; d.az = d.z;
        int t = perm[i];
        d.bx = tx[t]; d.by = ty[t]; d.bz = tz[t];
        d.r0 = d.r; d.g0 = d.g; d.b0 = d.b;
        d.r = tr[t]; d.g = tg[t]; d.b = tb[t];
        double dx = d.bx - d.ax, dy = d.by - d.ay, dz = d.bz - d.az;
        dmax = std::max(dmax, sqrt(dx*dx + dy*dy + dz*dz));
    }
    // 最小ジャーク軌道の上限から、いちばん遠い機体が守れる最小の T を出す。
    // 全機がこの T を共有するので、必ず同時に到着する。
    double tv = 1.875 * dmax / VMAX;
    double ta = sqrt(5.7735 * dmax / AMAX);
    moveT = std::max(2.0, std::max(tv, ta));
    moveElapsed = 0.0;
    phase = 1;
    curForm = form;
}

// ---------------------------------------------------------------- ABI
extern "C" {

KEEP int sim_w() { return FW; }
KEEP int sim_h() { return FH; }

KEEP void sim_reset() {
    int n = std::max(SHOW_N, std::max(SHOW2_N, ART_N));   // いちばん多い編隊に合わせる
    drones.assign(n, Drone{});
    for (int i = 0; i < n; ++i) {
        Drone& d = drones[i];
        float r, g, b;
        formation_pos(F_GROUND, i, n, d.x, d.y, d.z, r, g, b);
        d.ax = d.bx = d.x; d.ay = d.by = d.y; d.az = d.bz = d.z;
        d.r = d.r0 = r; d.g = d.g0 = g; d.b = d.b0 = b;
    }
    curForm = F_GROUND; phase = 0; holdT = 1.5;
    moveElapsed = 0; moveT = 1;
    simTime = 0;
    statVmax = statAmax = 0; statSep = 1e9;
}

KEEP int sim_init(int, int) {
    // 編隊の幅の 2.6 倍が画面に入る画角。写真と同じくらいの引き
    FOC = FW * CAMD / (FIELD_W * 2.05);
    px.assign((size_t)FW * FH, 0);
    glow.assign((size_t)FW * FH * 3, 0.0f);
    sim_reset();
    return 1;
}

KEEP void sim_set(int id, double v) {
    switch (id) {
    case 0: p_speed = v; break;
    case 1: p_glow  = v; break;
    case 2: p_hud   = v; break;
    case 3: p_auto  = v; break;
    }
}
KEEP double sim_get(int id) {
    switch (id) {
    case 0: return (double)drones.size();
    case 1: return statVmax;
    case 2: return statAmax;
    case 3: return statSep;
    case 4: return (double)curForm;
    case 5: return moveT;
    case 6: return phase ? (moveElapsed / moveT) : 1.0;
    }
    return 0.0;
}
KEEP void sim_action(int id) {
    if (id == 0) assign(F_SHOW);
    else if (id == 1) assign(F_SHOW2);
    else if (id == 2) assign(F_ART);
    else if (id == 3) assign(F_SPHERE);
    else if (id == 4) assign(F_RING);
    else if (id == 5) assign(F_GROUND);
    else if (id == 6) sim_reset();
}
KEEP void sim_click(double, double) { assign(F_SHOW); }

KEEP void sim_step(int frames) {
    for (int fr = 0; fr < frames; ++fr) {
        for (int ss = 0; ss < SUBSTEPS; ++ss) {
            const double dt = DT * p_speed;
            simTime += dt;

            if (phase == 1) {
                moveElapsed += dt;
                double tau = moveElapsed / moveT;
                if (tau >= 1.0) { tau = 1.0; phase = 0; holdT = 3.0; }
                // 最小ジャーク: 両端で速度も加速度もゼロ
                double s  = tau*tau*tau * (10.0 + tau * (-15.0 + 6.0 * tau));
                double sd = 30.0 * tau*tau * (1.0 - tau) * (1.0 - tau) / moveT;
                double sa = 60.0 * tau * (1.0 - 3.0*tau + 2.0*tau*tau) / (moveT * moveT);
                double vm = 0, am = 0;
                for (auto& d : drones) {
                    double dx = d.bx - d.ax, dy = d.by - d.ay, dz = d.bz - d.az;
                    d.x = d.ax + dx * s; d.y = d.ay + dy * s; d.z = d.az + dz * s;
                    double L = sqrt(dx*dx + dy*dy + dz*dz);
                    vm = std::max(vm, L * sd);
                    am = std::max(am, L * fabs(sa));
                }
                statVmax = std::max(statVmax, vm);
                statAmax = std::max(statAmax, am);
                // 定点保持のゆらぎ。RTK-GPS の精度と風で、実機は常に数十cm揺れている。
                // ばね＋減衰で目標に引き戻されながら、風にランダムに押される
            } else {
                holdT -= dt;
                if (holdT <= 0 && p_auto > 0.5) {
                    int nxt = (curForm == F_SHOW)   ? F_SPHERE
                            : (curForm == F_SPHERE) ? F_SHOW2
                            : (curForm == F_SHOW2)  ? F_ART
                            : (curForm == F_ART)    ? F_RING
                            : F_SHOW;
                    assign(nxt);
                }
            }
        }
    }
    // 定点保持のゆらぎ(ばね＋減衰＋風のランダム外乱)。実機は常に数十cm揺れている
    for (auto& d : drones) {
        const double kSp = 9.0, cDa = 3.4, wind = 0.55;
        double dt2 = 1.0 / 60.0;
        d.jvx += (-kSp * d.jx - cDa * d.jvx + rnds() * wind) * dt2;
        d.jvy += (-kSp * d.jy - cDa * d.jvy + rnds() * wind * 0.6) * dt2;
        d.jvz += (-kSp * d.jz - cDa * d.jvz + rnds() * wind) * dt2;
        d.jx += d.jvx * dt2; d.jy += d.jvy * dt2; d.jz += d.jvz * dt2;
    }

    // 最接近距離(そこそこの間引きで見る)
    int n = (int)drones.size();
    double mn = 1e9;
    for (int i = 0; i < n; i += 3) for (int j = i + 1; j < n; j += 3) {
        double dx = drones[i].x - drones[j].x, dy = drones[i].y - drones[j].y, dz = drones[i].z - drones[j].z;
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < mn) mn = d2;
    }
    if (mn < 1e8) statSep = sqrt(mn);
}

KEEP uint8_t* sim_render() {
    // --- 夜空と街
    for (int y = 0; y < FH; ++y) {
        double t = (double)y / FH;
        int r = (int)(6 + 16 * t * t), g = (int)(10 + 22 * t * t), b = (int)(20 + 40 * t * t);
        uint32_t c = rgb(r, g, b);
        uint32_t* row = &px[(size_t)y * FW];
        for (int x = 0; x < FW; ++x) row[x] = c;
    }
    {   // 街並みのシルエットと窓明かり(地平線の位置は投影から出す)
        uint32_t sil = rgb(10, 12, 20);
        double gx, gy, gsc; project(0, 0, 0, gx, gy, gsc);
        double base = gy;
        for (int x = 0; x < FW; ++x) {
            double u = x * 0.021;
            double hgt = 34 + 26 * sin(u * 0.7) + 18 * sin(u * 1.9 + 1.1) + 12 * sin(u * 4.3 + 2.2);
            int top = (int)(base - hgt);
            for (int y = std::max(0, top); y < FH; ++y) px[(size_t)y * FW + x] = sil;
            if (((x / 7) % 3) == 0 && rnd() < 0.004) {}
        }
        uint32_t win = rgb(120, 96, 44);
        uint32_t sv = rng; rng = 13572468u;
        for (int k = 0; k < 900; ++k) {
            int x = (int)(rnd() * FW);
            double u = x * 0.021;
            double hgt = 34 + 26 * sin(u * 0.7) + 18 * sin(u * 1.9 + 1.1) + 12 * sin(u * 4.3 + 2.2);
            int top = (int)(base - hgt);
            int y = top + (int)(rnd() * (FH - top));
            if (y < 0 || y >= FH || x < 0 || x >= FW) continue;
            px[(size_t)y * FW + x] = win;
            if (x + 1 < FW) px[(size_t)y * FW + x + 1] = win;
        }
        rng = sv;
    }

    // --- 機体の光を加算
    std::fill(glow.begin(), glow.end(), 0.0f);
    for (auto& d : drones) {
        double sx, sy, sc;
        project(d.x + d.jx, d.y + d.jy, d.z + d.jz, sx, sy, sc);
        if (sx < -20 || sy < -20 || sx > FW + 20 || sy > FH + 20) continue;
        double br = (d.y < 3.0) ? 0.22 : 1.0;             // 地上待機は暗い
        double dep = sc / FOC * 330.0;
        // 実際の写真では、LED は「白く飛んだ芯」と「広がるにじみ」の二重に写る。
        // 芯(狭くて強い)と暈(広くて弱い)を重ねると、あの光り方になる。
        int cx = (int)sx, cy = (int)sy;
        struct { int R; double amp, sig; } lobes[2] = { {2, 7.0, 1.45}, {9, 1.15, 0.048} };
        for (int L = 0; L < 2; ++L) {
            int R = lobes[L].R;
            double amp = lobes[L].amp * br * p_glow * dep;
            for (int dy = -R; dy <= R; ++dy) for (int dx = -R; dx <= R; ++dx) {
                int X = cx + dx, Y = cy + dy;
                if (X < 0 || Y < 0 || X >= FW || Y >= FH) continue;
                double px2 = X + 0.5 - sx, py2 = Y + 0.5 - sy;
                double w = exp(-(px2*px2 + py2*py2) * lobes[L].sig) * amp;
                float* p = &glow[((size_t)Y * FW + X) * 3];
                p[0] += (float)(d.r * w); p[1] += (float)(d.g * w); p[2] += (float)(d.b * w);
            }
        }
    }
    for (size_t i = 0, np = (size_t)FW * FH; i < np; ++i) {
        float r = glow[i*3], g = glow[i*3+1], b = glow[i*3+2];
        if (r + g + b < 0.004f) continue;
        uint32_t c = px[i];
        int R = (int)((c & 255)        + 255.0f * (r / (1.0f + r)));
        int G = (int)(((c >> 8) & 255) + 255.0f * (g / (1.0f + g)));
        int B = (int)(((c >> 16) & 255)+ 255.0f * (b / (1.0f + b)));
        px[i] = rgb(R, G, B);
    }

    if (p_hud >= 0.5) {
        Olivec_Canvas oc = olivec_canvas(px.data(), FW, FH, FW);
        char buf[200];
        const char* fn[F_N] = { "GROUND", "PHOTO 1", "PHOTO 2", "ARTWORK", "SPHERE", "RING" };
        snprintf(buf, sizeof(buf), "%d DRONES   %s   T=%.1fs  %.0f%%",
                 (int)drones.size(), fn[curForm], moveT, sim_get(6) * 100);
        olivec_text(oc, buf, 14, 12, olivec_default_font, 2, rgb(150, 190, 220));
        snprintf(buf, sizeof(buf), "vmax %.1f/%.0f m/s   amax %.1f/%.1f m/s2   min sep %.1f m",
                 statVmax, VMAX, statAmax, AMAX, statSep);
        olivec_text(oc, buf, 14, FH - 26, olivec_default_font, 2, rgb(110, 150, 180));
    }
    return (uint8_t*)px.data();
}

}  // extern "C"

// ---------------------------------------------------------------- native self-test
#ifndef __EMSCRIPTEN__
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    sim_init(0, 0);
    int steps = argc > 1 ? atoi(argv[1]) : 400;
    const char* out = argc > 2 ? argv[2] : "preview.png";
    if (argc > 3) sim_set(2, atof(argv[3]));
    sim_action(argc > 4 ? atoi(argv[4]) : 0);   // 5番目 = 編隊(0写真1 1写真2 2ドラえもん 3球 4輪)
    for (int i = 0; i < steps; ++i) { sim_step(1); sim_render(); }
    uint8_t* p = sim_render();
    printf("drone_os: %dx%d frames=%d drones=%d form=%d\n", FW, FH, steps, (int)drones.size(), curForm);
    printf("  実測 vmax %.2f m/s (上限%.0f)  amax %.2f m/s2 (上限%.1f)  最接近 %.2f m  T=%.1fs\n",
           statVmax, VMAX, statAmax, AMAX, statSep, moveT);
    stbi_write_png(out, FW, FH, 4, p, FW * 4);
    return 0;
}
#endif
