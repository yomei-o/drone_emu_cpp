// mkpoints — ドローンショーの写真から光点(=1機ずつのドローン)を拾う。
//
// 夜空に浮かぶ LED の点は「暗い背景の中の、小さくて明るい塊」なので、
// 明るさで2値化して連結成分を取れば1機ずつ分離できる。
// 街明かりは拾いたくないので、空の領域だけを見る。
//
//   clang++ -O2 -std=c++17 -I. tools/mkpoints.cpp -o /tmp/mkpoints
//   /tmp/mkpoints assets/show.png src/show_pts.h /tmp/pts.png
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>

struct Pt { double x, y; int r, g, b, area; };

int main(int argc, char** argv) {
    const char* in  = argc > 1 ? argv[1] : "assets/show.png";
    const char* out = argc > 2 ? argv[2] : "src/show_pts.h";
    const char* dbg = argc > 3 ? argv[3] : nullptr;
    double SKYR    = argc > 4 ? atof(argv[4]) : 0.655;   // 空として見る高さの割合
    const char* sym = argc > 5 ? argv[5] : "SHOW";        // 出力する C のシンボル名

    int w, h, n;
    unsigned char* img = stbi_load(in, &w, &h, &n, 3);
    if (!img) { fprintf(stderr, "load failed: %s\n", in); return 1; }
    fprintf(stderr, "loaded %dx%d\n", w, h);

    // 空だけを見る(下は街明かりや観客)。写真ごとに違うので比率で切る。
    int hy = (int)(h * SKYR);

    // 2値化。薄暮だと空そのものが明るいので、固定のしきい値では使えない。
    // ドローンの点より十分大きい窓でぼかした「局所背景」を作り、そこから
    // 突出している画素だけを取る。これなら夜空でも薄暮でも同じ設定で拾える。
    // 明るさの指標に輝度 (R*0.30 + G*0.59 + B*0.11) を使うと、
    // 青の寄与が 11% しかないので純青の LED を取りこぼす。
    // LED の検出は max(R,G,B) で見る(どの色も対等に扱える)。
    std::vector<float> val((size_t)w * h);
    for (size_t i = 0; i < (size_t)w * h; ++i)
        val[i] = (float)std::max(img[i*3], std::max(img[i*3+1], img[i*3+2]));

    // 局所背景。空の色そのものを引きたいので RGB それぞれで作る
    const int BR = 24;
    std::vector<float> bgc[3], tmp((size_t)w * h);
    for (int c = 0; c < 3; ++c) {
        bgc[c].assign((size_t)w * h, 0.0f);
        for (int y = 0; y < h; ++y) {
            double acc = 0; int cnt = 0;
            for (int x = -BR; x < w; ++x) {
                if (x + BR < w) { acc += img[((size_t)y*w + x + BR)*3 + c]; cnt++; }
                if (x - BR - 1 >= 0) { acc -= img[((size_t)y*w + x - BR - 1)*3 + c]; cnt--; }
                if (x >= 0) tmp[(size_t)y*w + x] = (float)(acc / std::max(1, cnt));
            }
        }
        for (int x = 0; x < w; ++x) {
            double acc = 0; int cnt = 0;
            for (int y = -BR; y < h; ++y) {
                if (y + BR < h) { acc += tmp[(size_t)(y + BR)*w + x]; cnt++; }
                if (y - BR - 1 >= 0) { acc -= tmp[(size_t)(y - BR - 1)*w + x]; cnt--; }
                if (y >= 0) bgc[c][(size_t)y*w + x] = (float)(acc / std::max(1, cnt));
            }
        }
    }
    std::vector<unsigned char> bin((size_t)w * h, 0);
    for (int y = 0; y < hy; ++y) for (int x = 0; x < w; ++x) {
        size_t i = (size_t)y * w + x;
        float bgv = std::max(bgc[0][i], std::max(bgc[1][i], bgc[2][i]));
        if (val[i] - bgv > 22.0f) bin[i] = 1;
    }

    // 連結成分(8近傍)
    std::vector<int> lab((size_t)w * h, -1);
    std::vector<Pt> pts;
    std::vector<int> stack;
    for (int y = 0; y < hy; ++y) for (int x = 0; x < w; ++x) {
        size_t s0 = (size_t)y * w + x;
        if (!bin[s0] || lab[s0] >= 0) continue;
        int id = (int)pts.size();
        stack.clear(); stack.push_back((int)s0); lab[s0] = id;
        double sx = 0, sy = 0; long sr = 0, sg = 0, sb = 0; int area = 0;
        // LED の芯は白飛びしているので、そこの色を取ると全部白になる。
        // 飽和していない周辺部(max<245)の色を平均して本来の色相を拾う。
        long ur = 0, ug = 0, ub = 0; int ucnt = 0;
        // 明るい画素だけの重心だと点がにじむので、輝度で重み付けする
        double wsum = 0;
        while (!stack.empty()) {
            int i = stack.back(); stack.pop_back();
            int cx = i % w, cy = i / w;
            int r = img[(size_t)i*3], g = img[(size_t)i*3+1], b = img[(size_t)i*3+2];
            double lw = std::max(r, std::max(g, b));   // 青も対等に扱う
            sx += cx * lw; sy += cy * lw; wsum += lw;
            sr += r; sg += g; sb += b; area++;
            // 空の色がにじみに乗るので、局所背景を引いた「LED が足した分」で色を見る。
            // これをやらないと、薄暮の青い空で白い LED まで青くなる。
            if (std::max(r, std::max(g, b)) < 248) {
                size_t q = (size_t)i;
                ur += std::max(0, r - (int)bgc[0][q]);
                ug += std::max(0, g - (int)bgc[1][q]);
                ub += std::max(0, b - (int)bgc[2][q]);
                ucnt++;
            }
            for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                int nx = cx + dx, ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= w || ny >= hy) continue;
                size_t j = (size_t)ny * w + nx;
                if (bin[j] && lab[j] < 0) { lab[j] = id; stack.push_back((int)j); }
            }
        }
        if (wsum < 1e-9) continue;
        Pt p;
        p.x = sx / wsum; p.y = sy / wsum;
        // 飽和していない画素の平均から色相を取り、いちばん強いチャンネルを 255 に
        // 正規化して LED 本来の色に戻す(にじみで薄まったぶんを戻す)
        int cr, cg, cb;
        if (ucnt > 2) { cr = (int)(ur / ucnt); cg = (int)(ug / ucnt); cb = (int)(ub / ucnt); }
        else          { cr = (int)(sr / area); cg = (int)(sg / area); cb = (int)(sb / area); }
        // 背景を引いた時点でだいぶ素直な色になっているので、彩度の持ち上げは控えめに
        int mn = std::min(cr, std::min(cg, cb));
        const double K = 0.35;
        cr = (int)((cr - K * mn) / (1.0 - K));
        cg = (int)((cg - K * mn) / (1.0 - K));
        cb = (int)((cb - K * mn) / (1.0 - K));
        cr = std::max(0, cr); cg = std::max(0, cg); cb = std::max(0, cb);
        int mx = std::max(cr, std::max(cg, cb)); if (mx < 1) mx = 1;
        p.r = std::min(255, cr * 255 / mx);
        p.g = std::min(255, cg * 255 / mx);
        p.b = std::min(255, cb * 255 / mx);
        p.area = area;
        pts.push_back(p);
    }
    fprintf(stderr, "連結成分 %d 個\n", (int)pts.size());

    // しきい値を下げると隣の LED とくっついて、ひとつの塊として数えられてしまう。
    // なので連結成分ではなく「極大点」を取る。くっついていても1機ずつ分離できる。
    // 最小間隔は、孤立している小さい塊の大きさ(面積の25%点)から見積もる。
    std::vector<int> areas;
    for (auto& p : pts) areas.push_back(p.area);
    std::sort(areas.begin(), areas.end());
    int a25 = areas.empty() ? 9 : areas[areas.size() / 4];
    double Rmin = std::max(3.0, 1.15 * sqrt(4.0 * a25 / M_PI));
    fprintf(stderr, "面積の25%%点 %d px → 最小間隔 %.1f px\n", a25, Rmin);

    std::vector<int> peak;
    for (int y = 1; y < hy - 1; ++y) for (int x = 1; x < w - 1; ++x) {
        size_t i = (size_t)y * w + x;
        if (!bin[i]) continue;
        float v = val[i];
        bool isMax = true;
        for (int dy = -1; dy <= 1 && isMax; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            if (!dx && !dy) continue;
            if (val[(size_t)(y+dy)*w + x+dx] > v) { isMax = false; break; }
        }
        if (isMax) peak.push_back((int)i);
    }
    std::sort(peak.begin(), peak.end(), [&](int a, int b) {
        float ea = val[a] - std::max(bgc[0][a], std::max(bgc[1][a], bgc[2][a]));
        float eb = val[b] - std::max(bgc[0][b], std::max(bgc[1][b], bgc[2][b]));
        return ea > eb; });

    int gw = std::max(1, (int)(w / Rmin) + 1), gh = std::max(1, (int)(h / Rmin) + 1);
    std::vector<int> grid((size_t)gw * gh, -1);
    std::vector<Pt> keep;
    for (int idx : peak) {
        int x = idx % w, y = idx / w;
        int gx = (int)(x / Rmin), gy = (int)(y / Rmin);
        bool ok = true;
        for (int j = gy - 1; j <= gy + 1 && ok; ++j) for (int i2 = gx - 1; i2 <= gx + 1 && ok; ++i2) {
            if (i2 < 0 || j < 0 || i2 >= gw || j >= gh) continue;
            int q = grid[(size_t)j * gw + i2];
            if (q < 0) continue;
            double dx = keep[q].x - x, dy = keep[q].y - y;
            if (dx*dx + dy*dy < Rmin * Rmin) ok = false;
        }
        if (!ok) continue;
        // 色はこの極大点のまわりから拾う(飽和していない画素の、背景を引いた分)
        long ur = 0, ug = 0, ub = 0; int ucnt = 0;
        int RR = (int)(Rmin * 0.75);
        for (int dy = -RR; dy <= RR; ++dy) for (int dx = -RR; dx <= RR; ++dx) {
            int X = x + dx, Y = y + dy;
            if (X < 0 || Y < 0 || X >= w || Y >= h) continue;
            size_t q = (size_t)Y * w + X;
            int r = img[q*3], g = img[q*3+1], b = img[q*3+2];
            if (std::max(r, std::max(g, b)) >= 248) continue;
            if (val[q] - std::max(bgc[0][q], std::max(bgc[1][q], bgc[2][q])) < 8.0f) continue;
            ur += std::max(0, r - (int)bgc[0][q]);
            ug += std::max(0, g - (int)bgc[1][q]);
            ub += std::max(0, b - (int)bgc[2][q]);
            ucnt++;
        }
        Pt p; p.x = x; p.y = y; p.area = 1;
        int cr, cg, cb;
        if (ucnt > 1) { cr = (int)(ur/ucnt); cg = (int)(ug/ucnt); cb = (int)(ub/ucnt); }
        else { cr = img[(size_t)idx*3]; cg = img[(size_t)idx*3+1]; cb = img[(size_t)idx*3+2]; }
        int mn = std::min(cr, std::min(cg, cb));
        const double K = 0.35;
        cr = std::max(0, (int)((cr - K*mn)/(1-K)));
        cg = std::max(0, (int)((cg - K*mn)/(1-K)));
        cb = std::max(0, (int)((cb - K*mn)/(1-K)));
        int mx = std::max(cr, std::max(cg, cb)); if (mx < 1) mx = 1;
        p.r = std::min(255, cr*255/mx); p.g = std::min(255, cg*255/mx); p.b = std::min(255, cb*255/mx);
        grid[(size_t)gy * gw + gx] = (int)keep.size();
        keep.push_back(p);
    }
    // 外れ値を捨てる。地平線の雲や観客のスマホ画面を拾ってしまうと、
    // 外接矩形が広がって本体が小さく配置されてしまう。
    // それらは密集しているので「孤立点を落とす」だけでは残る。
    // そこで点を塊にまとめ(近いもの同士を繋ぐ)、本体から離れた塊ごと捨てる。
    {
        int m = (int)keep.size();
        std::vector<int> par(m);
        for (int i = 0; i < m; ++i) par[i] = i;
        std::function<int(int)> find = [&](int a) { while (par[a] != a) { par[a] = par[par[a]]; a = par[a]; } return a; };
        double lim = Rmin * 6.0, lim2 = lim * lim;   // 絵の中の飛び石も同じ塊として繋ぐ
        for (int i = 0; i < m; ++i) for (int j = i + 1; j < m; ++j) {
            double dx = keep[i].x - keep[j].x, dy = keep[i].y - keep[j].y;
            if (dx*dx + dy*dy < lim2) { int a = find(i), b = find(j); if (a != b) par[a] = b; }
        }
        std::vector<int> cnt(m, 0);
        for (int i = 0; i < m; ++i) cnt[find(i)]++;
        int big = 0; for (int i = 0; i < m; ++i) if (cnt[i] > cnt[big]) big = i;
        // 本体の外接矩形
        double mx0=1e9,mx1=-1e9,my0=1e9,my1=-1e9;
        for (int i = 0; i < m; ++i) if (find(i) == big) {
            mx0=std::min(mx0,keep[i].x); mx1=std::max(mx1,keep[i].x);
            my0=std::min(my0,keep[i].y); my1=std::max(my1,keep[i].y);
        }
        double diag = sqrt((mx1-mx0)*(mx1-mx0) + (my1-my0)*(my1-my0));
        double margin = diag * 0.12;
        std::vector<Pt> ok;
        for (int i = 0; i < m; ++i) {
            if (find(i) == big) { ok.push_back(keep[i]); continue; }
            if (cnt[find(i)] < 4) continue;                 // 小さすぎる塊は捨てる
            // 本体の近くにある塊だけ残す(みゃくみゃくの周りの星のような飾り)
            double dx = std::max({mx0 - keep[i].x, 0.0, keep[i].x - mx1});
            double dy = std::max({my0 - keep[i].y, 0.0, keep[i].y - my1});
            if (sqrt(dx*dx + dy*dy) < margin) ok.push_back(keep[i]);
        }
        fprintf(stderr, "塊 → 本体%d機 / 採用 %d → %d 機\n", cnt[big], m, (int)ok.size());
        keep.swap(ok);
    }
    fprintf(stderr, "極大点 %d 個 → 採用 %d 機\n", (int)peak.size(), (int)keep.size());

    if (dbg) {
        std::vector<unsigned char> ov((size_t)w * h * 3);
        for (size_t i = 0; i < (size_t)w * h * 3; ++i) ov[i] = img[i] / 3;
        for (auto& p : keep) {
            int cx = (int)p.x, cy = (int)p.y;
            for (int d = -4; d <= 4; ++d) {
                int a = cx + d, b = cy + d;
                if (a >= 0 && a < w && cy >= 0 && cy < h) {
                    size_t o = ((size_t)cy * w + a) * 3; ov[o]=0; ov[o+1]=255; ov[o+2]=0; }
                if (b >= 0 && b < h && cx >= 0 && cx < w) {
                    size_t o = ((size_t)b * w + cx) * 3; ov[o]=0; ov[o+1]=255; ov[o+2]=0; }
            }
        }
        stbi_write_png(dbg, w, h, 3, ov.data(), w * 3);
    }

    FILE* f = fopen(out, "w");
    if (f) {
        fprintf(f, "// 自動生成 (tools/mkpoints.cpp)。写真から拾ったドローンの配置。\n");
        fprintf(f, "// x,y は 0..1 に正規化(画面比で持つ)。rgb は LED の色。\n");
        fprintf(f, "static const int %s_N = %d;\n", sym, (int)keep.size());
        fprintf(f, "static const float %s_PTS[][5] = {\n", sym);
        for (auto& p : keep)
            fprintf(f, "{%.5ff,%.5ff,%d,%d,%d},\n", p.x / w, p.y / w, p.r, p.g, p.b);
        fprintf(f, "};\n");
        fclose(f);
        fprintf(stderr, "wrote %s (%d 機)\n", out, (int)keep.size());
    }
    stbi_image_free(img);
    return 0;
}
