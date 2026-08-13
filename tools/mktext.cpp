// mktext — ロゴや文字(べた塗りの一色)から、ドローンショーの編隊を作る。
//
// 輪郭を拾う mkart とは狙いが違う。文字は線ではなく**面**なので、輪郭だけ拾うと
// 中抜きの縁取りになって読みにくい。ここでは面を塗る:
//   1) その色の画素だけを選ぶ(背景と、ほかの色は捨てる)
//   2) 縁に等間隔で置いて字形をはっきりさせる
//   3) 内側を三角格子で埋める(実機のショーと同じ。揺れても並びが崩れて見えない)
//
// ハマりどころ。**暗い色は「黒い線」に見える。** 例えば #B01F22 の赤は
// 輝度 R×0.30+G×0.59+B×0.11 が 75 しかないので、輝度でしきい値を切る作りだと
// 文字ぜんぶが「ドローンが描けない黒」と判定されて機体が白くなる。
// 明るさではなく**背景色からの距離**で見れば、暗い赤でも暗い青でも同じに拾える。
//
// しきい値は絵ごとに変えたくないので、許容差は「文字色と背景色の距離」に対する割合で持つ。
// 縁のなめらかな画素(背景と文字の中間色)は、この割合で切れる。
//
//   clang++ -O2 -std=c++17 -I. tools/mktext.cpp -o /tmp/mktext
//   /tmp/mktext vavant.png src/text_pts.h /tmp/text.png 420 TEXT b01f22
//   #         入力         出力ヘッダ      確認用       機数 記号 文字色(auto可)
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>

struct Pt { double x, y; int r, g, b; };

int main(int argc, char** argv) {
    const char* in   = argc > 1 ? argv[1] : "vavant.png";
    const char* out  = argc > 2 ? argv[2] : "src/text_pts.h";
    const char* dbg  = argc > 3 ? argv[3] : nullptr;
    int    want      = argc > 4 ? atoi(argv[4]) : 420;
    const char* sym  = argc > 5 ? argv[5] : "TEXT";
    const char* hex  = argc > 6 ? argv[6] : "auto";
    double tolFrac   = argc > 7 ? atof(argv[7]) : 0.55;   // 文字色と背景色の距離に対する割合

    int w, h, n;
    unsigned char* img = stbi_load(in, &w, &h, &n, 3);
    if (!img) { fprintf(stderr, "load failed: %s\n", in); return 1; }
    fprintf(stderr, "loaded %dx%d\n", w, h);

    auto at = [&](int x, int y, int c) {
        x = std::max(0, std::min(w - 1, x)); y = std::max(0, std::min(h - 1, y));
        return (int)img[((size_t)y * w + x) * 3 + c];
    };
    auto dist = [](const int a[3], const int b[3]) {
        double dr = a[0]-b[0], dg = a[1]-b[1], db = a[2]-b[2];
        return sqrt(dr*dr + dg*dg + db*db);
    };

    // --- 背景色。外周の枠(短辺の3%)の中央値。グラデーションでも中央値なら真ん中に落ちる
    int bg[3];
    {
        int m = std::max(1, (int)(std::min(w, h) * 0.03));
        std::vector<int> v[3];
        for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
            if (x >= m && x < w - m && y >= m && y < h - m) continue;
            for (int c = 0; c < 3; ++c) v[c].push_back(at(x, y, c));
        }
        for (int c = 0; c < 3; ++c) {
            std::sort(v[c].begin(), v[c].end());
            bg[c] = v[c][v[c].size() / 2];
        }
    }

    // --- 文字色。指定が無ければ、背景から離れた画素をRGBの粗い箱に入れて、
    //     いちばん人数の多い箱の平均を採る(反アリアスの中間色は散らばるので勝たない)
    int fg[3] = {255, 255, 255};
    if (strcmp(hex, "auto") != 0) {
        unsigned v = (unsigned)strtoul(hex, nullptr, 16);
        fg[0] = (v >> 16) & 255; fg[1] = (v >> 8) & 255; fg[2] = v & 255;
    } else {
        const int B = 8, BN = 256 / B;   // 32刻みの箱
        std::vector<int> cnt((size_t)BN*BN*BN, 0);
        std::vector<long> sr((size_t)BN*BN*BN, 0), sg((size_t)BN*BN*BN, 0), sb((size_t)BN*BN*BN, 0);
        for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
            int p[3] = { at(x,y,0), at(x,y,1), at(x,y,2) };
            if (dist(p, bg) < 60.0) continue;
            size_t k = ((size_t)(p[0]/(256/BN))*BN + p[1]/(256/BN))*BN + p[2]/(256/BN);
            cnt[k]++; sr[k] += p[0]; sg[k] += p[1]; sb[k] += p[2];
        }
        size_t bestk = 0;
        for (size_t k = 0; k < cnt.size(); ++k) if (cnt[k] > cnt[bestk]) bestk = k;
        if (cnt[bestk] > 0) {
            fg[0] = (int)(sr[bestk] / cnt[bestk]);
            fg[1] = (int)(sg[bestk] / cnt[bestk]);
            fg[2] = (int)(sb[bestk] / cnt[bestk]);
        }
    }
    double D = std::max(1.0, dist(fg, bg));
    double tol = D * tolFrac;
    fprintf(stderr, "背景 (%d,%d,%d)  文字 (%d,%d,%d)  距離 %.0f  許容 %.0f\n",
            bg[0], bg[1], bg[2], fg[0], fg[1], fg[2], D, tol);

    // --- マスク: 文字色に近い画素
    std::vector<unsigned char> mask((size_t)w * h, 0);
    long area = 0;
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
        int p[3] = { at(x,y,0), at(x,y,1), at(x,y,2) };
        if (dist(p, fg) < tol) { mask[(size_t)y*w + x] = 1; area++; }
    }
    fprintf(stderr, "文字の面積 %ld 画素 (%.1f%%)\n", area, 100.0 * area / ((double)w*h));
    if (area < 16) { fprintf(stderr, "文字が見つからない。色か許容差を指定してほしい\n"); return 1; }

    // --- 縁(4近傍にマスク外がある画素)。ここを等間隔で埋めると字形がはっきりする
    std::vector<int> edge;
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
        if (!mask[(size_t)y*w + x]) continue;
        bool b = false;
        for (int k = 0; k < 4 && !b; ++k) {
            int xx = x + (k==0) - (k==1), yy = y + (k==2) - (k==3);
            b = (xx < 0 || yy < 0 || xx >= w || yy >= h) || !mask[(size_t)yy*w + xx];
        }
        if (b) edge.push_back(y*w + x);
    }
    fprintf(stderr, "縁 %d 画素\n", (int)edge.size());

    // --- 機体の色。周りのマスク画素の平均を採って、彩度を上げて LED らしくする
    auto pick_color = [&](int cx, int cy, int& R, int& G, int& B) {
        long s[3] = {0,0,0}; int cnt = 0;
        for (int dy = -2; dy <= 2; ++dy) for (int dx = -2; dx <= 2; ++dx) {
            int x = cx + dx, y = cy + dy;
            if (x < 0 || y < 0 || x >= w || y >= h) continue;
            if (!mask[(size_t)y*w + x]) continue;
            for (int c = 0; c < 3; ++c) s[c] += at(x, y, c);
            cnt++;
        }
        if (cnt < 1) { R = fg[0]; G = fg[1]; B = fg[2]; }
        else { R = (int)(s[0]/cnt); G = (int)(s[1]/cnt); B = (int)(s[2]/cnt); }
        int mn = std::min(R, std::min(G, B)), mx = std::max(R, std::max(G, B));
        if (mx - mn > 26) {
            const double K = 0.55;
            R = (int)std::max(0.0, (R - K * mn) / (1 - K));
            G = (int)std::max(0.0, (G - K * mn) / (1 - K));
            B = (int)std::max(0.0, (B - K * mn) / (1 - K));
            int m2 = std::max(R, std::max(G, B)); if (m2 < 1) m2 = 1;
            R = std::min(255, R * 255 / m2); G = std::min(255, G * 255 / m2); B = std::min(255, B * 255 / m2);
        } else { R = G = B = 255; }
    };

    // --- 最小間隔 R で、縁 → 内側の順に置く。欲しい機数になるよう R を二分探索する。
    //     格子の間隔も R なので、隣同士のちょうど R は通したい。判定は 0.92R で見る。
    std::vector<Pt> best;
    double lo = 1.0, hi = std::max(w, h) * 0.5;
    for (int iter = 0; iter < 30; ++iter) {
        double R = 0.5 * (lo + hi);
        double Rc = R * 0.92, Rc2 = Rc * Rc;
        int gw = std::max(1, (int)(w / R) + 1), gh = std::max(1, (int)(h / R) + 1);
        std::vector<std::vector<int>> grid((size_t)gw * gh);
        std::vector<Pt> pts;
        auto try_put = [&](double x, double y) {
            int gx = (int)(x / R), gy = (int)(y / R);
            for (int j = gy - 1; j <= gy + 1; ++j) for (int i = gx - 1; i <= gx + 1; ++i) {
                if (i < 0 || j < 0 || i >= gw || j >= gh) continue;
                for (int p : grid[(size_t)j * gw + i]) {
                    double dx = pts[p].x - x, dy = pts[p].y - y;
                    if (dx*dx + dy*dy < Rc2) return;
                }
            }
            Pt p; p.x = x; p.y = y;
            pick_color((int)x, (int)y, p.r, p.g, p.b);
            grid[(size_t)gy * gw + gx].push_back((int)pts.size());
            pts.push_back(p);
        };
        for (int idx : edge) try_put(idx % w, idx / w);           // 縁
        double dy = R * 0.86602540378;                            // 三角格子(行間 R√3/2)
        for (int row = 0; (row + 0.5) * dy < h; ++row) {           // 内側
            double y = (row + 0.5) * dy, off = (row & 1) ? R * 0.5 : 0.0;
            for (double x = off + R * 0.5; x < w; x += R)
                if (mask[(size_t)(int)y * w + (int)x]) try_put(x, y);
        }
        if ((int)pts.size() > want) lo = R; else hi = R;
        best = pts;
        if (abs((int)pts.size() - want) < std::max(4, want / 50)) break;
    }
    fprintf(stderr, "配置 %d 機 (目標 %d)\n", (int)best.size(), want);

    if (dbg) {
        std::vector<unsigned char> ov((size_t)w * h * 3, 0);
        for (auto& p : best) {
            int cx = (int)p.x, cy = (int)p.y;
            for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                int x = cx + dx, y = cy + dy;
                if (x < 0 || y < 0 || x >= w || y >= h) continue;
                size_t o = ((size_t)y * w + x) * 3;
                ov[o] = (unsigned char)p.r; ov[o+1] = (unsigned char)p.g; ov[o+2] = (unsigned char)p.b;
            }
        }
        stbi_write_png(dbg, w, h, 3, ov.data(), w * 3);
    }

    FILE* f = fopen(out, "w");
    if (f) {
        fprintf(f, "// 自動生成 (tools/mktext.cpp)。文字/ロゴを塗って作ったドローンの配置。\n");
        fprintf(f, "static const int %s_N = %d;\n", sym, (int)best.size());
        fprintf(f, "static const float %s_PTS[][5] = {\n", sym);
        for (auto& p : best)
            fprintf(f, "{%.5ff,%.5ff,%d,%d,%d},\n", p.x / w, p.y / w, p.r, p.g, p.b);
        fprintf(f, "};\n");
        fclose(f);
        fprintf(stderr, "wrote %s\n", out);
    }
    stbi_image_free(img);
    return 0;
}
