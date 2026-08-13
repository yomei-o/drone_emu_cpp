// mkart — ふつうの絵から、ドローンショーの編隊を作る。
//
// 実際のショー制作と同じ手順:
//   1) 輪郭を拾う（色と明度の勾配）
//   2) 輪郭上に等間隔で機体を置く（近すぎると衝突するので最小間隔を守る）
//   3) 各機に色を割り当てる
//
// ここで物理的な制約がひとつある。**ドローンは黒を描けない**。
// 光を出すものなので、黒い輪郭線は「機体がいない場所」にしかならない。
// なので輪郭の位置には、隣接する面の色(青い体なら青、白い顔なら白)を置く。
//
//   clang++ -O2 -std=c++17 -I. tools/mkart.cpp -o /tmp/mkart
//   /tmp/mkart doraemon.jpg src/art_pts.h /tmp/art.png 500 ART
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>

struct Pt { double x, y; int r, g, b; };

int main(int argc, char** argv) {
    const char* in   = argc > 1 ? argv[1] : "doraemon.jpg";
    const char* out  = argc > 2 ? argv[2] : "src/art_pts.h";
    const char* dbg  = argc > 3 ? argv[3] : nullptr;
    int    want      = argc > 4 ? atoi(argv[4]) : 500;
    const char* sym  = argc > 5 ? argv[5] : "ART";

    int w, h, n;
    unsigned char* img = stbi_load(in, &w, &h, &n, 3);
    if (!img) { fprintf(stderr, "load failed: %s\n", in); return 1; }
    fprintf(stderr, "loaded %dx%d\n", w, h);

    auto at = [&](int x, int y, int c) {
        x = std::max(0, std::min(w - 1, x)); y = std::max(0, std::min(h - 1, y));
        return (int)img[((size_t)y * w + x) * 3 + c];
    };
    // --- 背景を特定する。外周から「明るくて色のない画素」を塗りつぶしていく。
    //     こうしないと、外側の輪郭の色を拾うときに背景の白が混ざって白くなってしまう。
    //     (顔の白は輪郭線で囲まれているので、ここには含まれない)
    std::vector<unsigned char> bgm((size_t)w * h, 0);
    {
        std::vector<int> st;
        auto push = [&](int x, int y) {
            if (x < 0 || y < 0 || x >= w || y >= h) return;
            size_t i = (size_t)y * w + x;
            if (bgm[i]) return;
            int r = at(x,y,0), g = at(x,y,1), b = at(x,y,2);
            int lum = (r*30 + g*59 + b*11) / 100;
            int mn = std::min(r, std::min(g, b)), mx = std::max(r, std::max(g, b));
            if (lum < 205 || mx - mn > 26) return;      // 明るくて色が無い = 背景
            bgm[i] = 1; st.push_back((int)i);
        };
        for (int x = 0; x < w; ++x) { push(x, 0); push(x, h - 1); }
        for (int y = 0; y < h; ++y) { push(0, y); push(w - 1, y); }
        while (!st.empty()) {
            int i = st.back(); st.pop_back();
            int x = i % w, y = i / w;
            push(x-1,y); push(x+1,y); push(x,y-1); push(x,y+1);
        }
    }

    // --- 輪郭: RGB それぞれの Sobel の最大値。黒い主線も色の境界も同じように拾える
    std::vector<float> edge((size_t)w * h, 0.0f);
    float emax = 1e-6f;
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
        float best = 0;
        for (int c = 0; c < 3; ++c) {
            int gx = -at(x-1,y-1,c) - 2*at(x-1,y,c) - at(x-1,y+1,c)
                     + at(x+1,y-1,c) + 2*at(x+1,y,c) + at(x+1,y+1,c);
            int gy = -at(x-1,y-1,c) - 2*at(x,y-1,c) - at(x+1,y-1,c)
                     + at(x-1,y+1,c) + 2*at(x,y+1,c) + at(x+1,y+1,c);
            float m = sqrtf((float)(gx*gx + gy*gy));
            best = std::max(best, m);
        }
        edge[(size_t)y * w + x] = best;
        emax = std::max(emax, best);
    }
    // 候補: 勾配が強い画素を、強い順に並べる
    std::vector<int> cand;
    for (size_t i = 0; i < (size_t)w * h; ++i)
        if (edge[i] > emax * 0.18f) cand.push_back((int)i);
    std::sort(cand.begin(), cand.end(), [&](int a, int b) { return edge[a] > edge[b]; });
    fprintf(stderr, "輪郭の候補 %d 画素\n", (int)cand.size());

    // --- 機体の色。黒い線の上に置くので、線を避けて周りの面の色を拾う
    auto pick_color = [&](int cx, int cy, int& R, int& G, int& B) {
        long sr = 0, sg = 0, sb = 0; int cnt = 0;
        for (int dy = -4; dy <= 4; ++dy) for (int dx = -4; dx <= 4; ++dx) {
            int x = cx + dx, y = cy + dy;
            if (x < 0 || y < 0 || x >= w || y >= h) continue;
            if (bgm[(size_t)y * w + x]) continue;   // 背景は数えない
            int r = at(x,y,0), g = at(x,y,1), b = at(x,y,2);
            int lum = (r*30 + g*59 + b*11) / 100;
            if (lum < 80) continue;              // 黒い主線は捨てる(ドローンは黒を描けない)
            sr += r; sg += g; sb += b; cnt++;
        }
        if (cnt < 3) { R = G = B = 255; return; }   // 周りが真っ黒なら白で描く
        R = (int)(sr / cnt); G = (int)(sg / cnt); B = (int)(sb / cnt);
        // 彩度を上げて LED らしくする。白っぽい面はそのまま白
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

    // --- 最小間隔を守って置く(近すぎると衝突するので実運用でも必須)。
    //     欲しい機数になるよう間隔を二分探索する。
    std::vector<Pt> best;
    double lo = 1.0, hi = std::max(w, h) * 0.5;
    for (int iter = 0; iter < 26; ++iter) {
        double R = 0.5 * (lo + hi), R2 = R * R;
        int gw = std::max(1, (int)(w / R) + 1), gh = std::max(1, (int)(h / R) + 1);
        std::vector<int> grid((size_t)gw * gh, -1);
        std::vector<Pt> pts;
        for (int idx : cand) {
            int x = idx % w, y = idx / w;
            int gx = (int)(x / R), gy = (int)(y / R);
            bool ok = true;
            for (int j = gy - 1; j <= gy + 1 && ok; ++j) for (int i = gx - 1; i <= gx + 1 && ok; ++i) {
                if (i < 0 || j < 0 || i >= gw || j >= gh) continue;
                int p = grid[(size_t)j * gw + i];
                if (p < 0) continue;
                double dx = pts[p].x - x, dy = pts[p].y - y;
                if (dx*dx + dy*dy < R2) ok = false;
            }
            if (!ok) continue;
            Pt p; p.x = x; p.y = y;
            pick_color(x, y, p.r, p.g, p.b);
            grid[(size_t)gy * gw + gx] = (int)pts.size();
            pts.push_back(p);
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
        fprintf(f, "// 自動生成 (tools/mkart.cpp)。絵の輪郭から作ったドローンの配置。\n");
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
