// planner.cpp — ARA* footstep planner for the hexapod.
//
// Receding-horizon outer loop: at each tripod switch, run a bounded-depth
// ARA* from the current stance state. Take the first action, execute, repeat.
// Horizon N is configurable; N=1 reduces to greedy action selection.
//
// State at each node is a "double-support" configuration (all 6 feet on the
// ground). Transitions are one full tripod swing: body pose changes, the
// inactive tripod re-places its feet at default offsets from the new body
// pose (z from terrain). Active tripod alternates.
//
// Usage:
//   planner <map_dir> [--horizon N] [--eps E] [--out plan.txt]

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hex {

// ─── Robot config (matches config.yaml) ─────────────────────────────────────

static constexpr double COXA_LAT  = 54.0;
static constexpr double COXA_DROP = 34.0;
static constexpr double FEMUR     = 74.0;
static constexpr double TIBIA     = 107.0;
static constexpr double STAND_Z   = 80.0;   // body center height above ground (flat)

// Leg order: LF, LM, LB, RF, RM, RB
enum Leg : int { LF=0, LM=1, LB=2, RF=3, RM=4, RB=5, NLEG=6 };
static const char* LEG_NAMES_ARR[NLEG] = {"LF","LM","LB","RF","RM","RB"};

// Mount pose in body frame: (x_mm, y_mm, yaw_deg)
struct Mount { double x, y, yaw_deg; };
static const std::array<Mount, NLEG> MOUNTS = {{
    { 45.0,  68.6,   45.0},  // LF
    {  0.0,  73.6,   90.0},  // LM
    {-45.0,  68.6,  135.0},  // LB
    { 45.0, -68.6,  -45.0},  // RF
    {  0.0, -73.6,  -90.0},  // RM
    {-45.0, -68.6, -135.0},  // RB
}};

// Default foot position in BODY frame for walking stance (pan_geom=0 per leg).
// Using bias=0 here (not the stand-pose 8° front/back bias) so that each leg
// sits at the center of its pan range — giving maximum forward/backward margin
// for body translation between swings. Derived: foot_body = mount + R(mount_yaw)
// @ (110, 0, -80) for each leg.
// Body z=0 by convention; foot z=-STAND_Z when on flat ground.
static const std::array<std::array<double, 3>, NLEG> DEFAULT_FOOT_BODY = {{
    { 122.78,  146.38, -STAND_Z},  // LF
    {   0.00,  183.60, -STAND_Z},  // LM
    {-122.78,  146.38, -STAND_Z},  // LB
    { 122.78, -146.38, -STAND_Z},  // RF
    {   0.00, -183.60, -STAND_Z},  // RM
    {-122.78, -146.38, -STAND_Z},  // RB
}};

// Tripod groups. Each tripod is the front+back of one side paired with the
// center leg of the opposite side — the three feet form a triangle whose
// centroid straddles the body midline, giving good stability margin.
// Tripod 0 = {RF, RB, LM}, Tripod 1 = {LF, LB, RM}.
static const std::array<std::array<int, 3>, 2> TRIPODS = {{
    {RF, RB, LM},
    {LF, LB, RM},
}};

// Joint limits (servo degrees) — see config.yaml.
static constexpr double PAN_INWARD_DEG = 15.0;
static constexpr double PAN_OUT_FB_DEG = 45.0;
static constexpr double PAN_OUT_MID_DEG = 15.0;
static constexpr double HIP_LO = 10.0, HIP_HI = 170.0;
static constexpr double KNEE_LO = 10.0, KNEE_HI = 170.0;

static inline double deg2rad(double d) { return d * M_PI / 180.0; }
static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

// ─── Terrain ─────────────────────────────────────────────────────────────────

struct Terrain {
    int nx = 0, ny = 0;
    double cell = 0.0;
    double origin_x = 0.0, origin_y = 0.0;  // world coords of cell (0,0)
    std::vector<double> h;                  // row-major, h[j*nx + i]

    double height_at(double x_mm, double y_mm) const {
        // Bilinear interpolation, clamped at edges.
        double fi = (x_mm - origin_x) / cell;
        double fj = (y_mm - origin_y) / cell;
        int i0 = std::max(0, std::min(nx - 1, (int)std::floor(fi)));
        int j0 = std::max(0, std::min(ny - 1, (int)std::floor(fj)));
        int i1 = std::min(nx - 1, i0 + 1);
        int j1 = std::min(ny - 1, j0 + 1);
        double tx = std::max(0.0, std::min(1.0, fi - i0));
        double ty = std::max(0.0, std::min(1.0, fj - j0));
        double h00 = h[j0 * nx + i0];
        double h10 = h[j0 * nx + i1];
        double h01 = h[j1 * nx + i0];
        double h11 = h[j1 * nx + i1];
        double h0 = h00 * (1 - tx) + h10 * tx;
        double h1 = h01 * (1 - tx) + h11 * tx;
        return h0 * (1 - ty) + h1 * ty;
    }

    double roughness_at(double x_mm, double y_mm, double radius_mm = 20.0) const {
        // Approximate local terrain variation within a small window.
        double hmin = 1e9, hmax = -1e9;
        int span = std::max(1, (int)std::round(radius_mm / cell));
        int ci = (int)std::round((x_mm - origin_x) / cell);
        int cj = (int)std::round((y_mm - origin_y) / cell);
        for (int dj = -span; dj <= span; dj++) for (int di = -span; di <= span; di++) {
            int i = ci + di, j = cj + dj;
            if (i < 0 || i >= nx || j < 0 || j >= ny) continue;
            double v = h[j * nx + i];
            hmin = std::min(hmin, v);
            hmax = std::max(hmax, v);
        }
        return (hmax > hmin) ? (hmax - hmin) : 0.0;
    }
};

// ─── Map (meta.yaml is parsed by hand — tiny format) ─────────────────────────

struct Map {
    std::string name;
    Terrain terrain;
    double start_x = 0, start_y = 0, start_yaw_deg = 0;
    double goal_x = 0, goal_y = 0, goal_tol = 40.0;
};

// Precomputed 2D admissible heuristic — cost-to-go from each body (x,y) cell
// to the goal, assuming a relaxed "point-robot on terrain" problem. Built
// once by backward Dijkstra from the goal over the heightmap; queried by
// the real planner via bilinear lookup. Replaces the old Euclidean heuristic
// which was blind to terrain and mis-priced detours around full-span obstacles.
struct HeuristicGrid {
    int nx = 0, ny = 0;
    double cell = 0.0;
    double origin_x = 0.0, origin_y = 0.0;
    std::vector<double> h;  // row-major, h[j*nx + i] = cost-to-go

    double lookup(double x_mm, double y_mm) const {
        double fi = (x_mm - origin_x) / cell;
        double fj = (y_mm - origin_y) / cell;
        int i0 = std::max(0, std::min(nx - 1, (int)std::floor(fi)));
        int j0 = std::max(0, std::min(ny - 1, (int)std::floor(fj)));
        int i1 = std::min(nx - 1, i0 + 1);
        int j1 = std::min(ny - 1, j0 + 1);
        double tx = std::max(0.0, std::min(1.0, fi - i0));
        double ty = std::max(0.0, std::min(1.0, fj - j0));
        double h00 = h[j0 * nx + i0];
        double h10 = h[j0 * nx + i1];
        double h01 = h[j1 * nx + i0];
        double h11 = h[j1 * nx + i1];
        // If any corner is inf, fall back to nearest finite corner to keep
        // the interpolation well-defined near unreachable regions.
        double best = std::min(std::min(h00, h10), std::min(h01, h11));
        if (!std::isfinite(best)) return std::numeric_limits<double>::infinity();
        auto fx = [&](double v){ return std::isfinite(v) ? v : best; };
        double h0 = fx(h00) * (1 - tx) + fx(h10) * tx;
        double h1 = fx(h01) * (1 - tx) + fx(h11) * tx;
        return h0 * (1 - ty) + h1 * ty;
    }
};

static std::string strip(std::string s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, b - a + 1);
}

static bool get_kv(const std::string& line, std::string& key, std::string& val) {
    size_t colon = line.find(':');
    if (colon == std::string::npos) return false;
    key = strip(line.substr(0, colon));
    val = strip(line.substr(colon + 1));
    // strip inline comment
    size_t hash = val.find('#');
    if (hash != std::string::npos) val = strip(val.substr(0, hash));
    return !key.empty();
}

static Map load_map(const std::string& dir) {
    Map m;
    std::ifstream meta(dir + "/meta.yaml");
    if (!meta) { std::cerr << "Cannot open " << dir << "/meta.yaml\n"; std::exit(1); }
    std::string line, section;
    while (std::getline(meta, line)) {
        // Track top-level section by indentation.
        if (line.empty() || line[0] == '#') continue;
        if (line[0] != ' ' && line[0] != '\t') {
            std::string k, v;
            if (get_kv(line, k, v)) {
                if (v.empty()) { section = k; continue; }
                if (k == "name") m.name = v;
                section = "";
            }
        } else {
            std::string k, v;
            if (!get_kv(line, k, v)) continue;
            double dv = 0.0;
            try { dv = std::stod(v); } catch (...) { continue; }
            if (section == "grid") {
                if (k == "nx") m.terrain.nx = (int)dv;
                else if (k == "ny") m.terrain.ny = (int)dv;
                else if (k == "cell_size_mm") m.terrain.cell = dv;
                else if (k == "origin_x_mm") m.terrain.origin_x = dv;
                else if (k == "origin_y_mm") m.terrain.origin_y = dv;
            } else if (section == "start") {
                if      (k == "x_mm")    m.start_x = dv;
                else if (k == "y_mm")    m.start_y = dv;
                else if (k == "yaw_deg") m.start_yaw_deg = dv;
            } else if (section == "goal") {
                if      (k == "x_mm")         m.goal_x = dv;
                else if (k == "y_mm")         m.goal_y = dv;
                else if (k == "tolerance_mm") m.goal_tol = dv;
            }
        }
    }

    // Load heightmap CSV.
    std::ifstream hf(dir + "/heightmap.csv");
    if (!hf) { std::cerr << "Cannot open " << dir << "/heightmap.csv\n"; std::exit(1); }
    m.terrain.h.assign((size_t)m.terrain.nx * m.terrain.ny, 0.0);
    int j = 0;
    while (std::getline(hf, line) && j < m.terrain.ny) {
        std::stringstream ss(line);
        std::string tok;
        int i = 0;
        while (std::getline(ss, tok, ',') && i < m.terrain.nx) {
            try { m.terrain.h[j * m.terrain.nx + i] = std::stod(tok); } catch (...) {}
            i++;
        }
        j++;
    }
    return m;
}

// ─── State ───────────────────────────────────────────────────────────────────

// A stance configuration: body pose + 6 foot (x,y,z) + which tripod is next
// to swing (active_tripod ∈ {0,1}). Feet are stored as world-frame doubles;
// the hash uses a grid quantization so A* can deduplicate.
struct State {
    double bx, by, bz, byaw_deg;
    std::array<std::array<double, 3>, NLEG> feet;
    int active_tripod;
};

static double body_z_from_feet(const std::array<std::array<double, 3>, NLEG>& feet) {
    double sum = 0.0;
    for (const auto& f : feet) sum += f[2];
    return sum / (double)NLEG + STAND_Z;
}

// Grid-quantized key for hashing/equality.
struct StateKey {
    int bx, by, byaw, active;
    std::array<std::array<int, 2>, NLEG> feet_xy;   // z ignored (from terrain)
    bool operator==(const StateKey& o) const {
        if (bx != o.bx || by != o.by || byaw != o.byaw || active != o.active) return false;
        for (int i = 0; i < NLEG; i++)
            if (feet_xy[i] != o.feet_xy[i]) return false;
        return true;
    }
};

struct StateKeyHash {
    size_t operator()(const StateKey& k) const noexcept {
        size_t h = 1469598103934665603ull;
        auto mix = [&](int v) { h ^= (size_t)v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2); };
        mix(k.bx); mix(k.by); mix(k.byaw); mix(k.active);
        for (int i = 0; i < NLEG; i++) { mix(k.feet_xy[i][0]); mix(k.feet_xy[i][1]); }
        return h;
    }
};

struct Params {
    double body_step_mm = 30.0;      // Δxy per tripod swing
    double yaw_step_deg = 8.0;       // Δyaw options
    double body_quant_mm = 10.0;     // quantization for hashing body pose
    double foot_quant_mm = 10.0;     // quantization for hashing foot positions
    double yaw_quant_deg = 4.0;
    int    horizon = 1;              // ARA* depth (N=1 → greedy)
    double eps = 1.5;                // heuristic inflation
    // Cost weights. Every tripod swing costs `c_swing` regardless of step
    // distance (each swing takes the same physical time). Progress toward the
    // goal is encoded in the heuristic (distance / body_step_mm → step count),
    // not in the step cost, so the planner is free to pick the action that
    // advances the most per swing without being penalized.
    double c_swing    = 1.0;         // base cost per tripod swing
    double w_yaw_deg  = 0.02;        // small penalty per deg of yaw to discourage churn
    double w_elev     = 0.20;        // per-mm elevation gain of new feet
    double w_stab     = 1.0;         // per-mm stability deficit
    double w_rough    = 0.08;        // per-mm local terrain variation under new feet
    double stab_margin_mm = 15.0;    // CoM must stay this far from nearest support edge
};

static StateKey make_key(const State& s, const Params& P) {
    StateKey k;
    k.bx = (int)std::round(s.bx / P.body_quant_mm);
    k.by = (int)std::round(s.by / P.body_quant_mm);
    k.byaw = (int)std::round(s.byaw_deg / P.yaw_quant_deg);
    k.active = s.active_tripod;
    for (int i = 0; i < NLEG; i++) {
        k.feet_xy[i][0] = (int)std::round(s.feet[i][0] / P.foot_quant_mm);
        k.feet_xy[i][1] = (int)std::round(s.feet[i][1] / P.foot_quant_mm);
    }
    return k;
}

// ─── Kinematics feasibility ──────────────────────────────────────────────────

// Transform foot from world to leg-local frame, then check reachability.
// Returns true if the leg can physically achieve this foot position at the
// given body pose. Body is assumed to hover `body_z` above world origin.
static bool leg_reachable(int leg_id,
                          double body_x, double body_y, double body_z, double body_yaw_deg,
                          double foot_x, double foot_y, double foot_z)
{
    double cb = std::cos(deg2rad(body_yaw_deg));
    double sb = std::sin(deg2rad(body_yaw_deg));

    // Foot in body frame.
    double fx_b =  cb * (foot_x - body_x) + sb * (foot_y - body_y);
    double fy_b = -sb * (foot_x - body_x) + cb * (foot_y - body_y);
    double fz_b = foot_z - body_z;

    // Foot relative to mount origin in body frame.
    const Mount& M = MOUNTS[leg_id];
    double dx = fx_b - M.x;
    double dy = fy_b - M.y;
    double cm = std::cos(deg2rad(M.yaw_deg));
    double sm = std::sin(deg2rad(M.yaw_deg));
    // Rotate into leg-local frame (inverse of mount_yaw).
    double xl =  cm * dx + sm * dy;
    double yl = -sm * dx + cm * dy;
    double zl = fz_b;

    // Pan limits.
    double pan_geom = rad2deg(std::atan2(yl, xl));
    double pan_out = (leg_id == LM || leg_id == RM) ? PAN_OUT_MID_DEG : PAN_OUT_FB_DEG;
    if (pan_geom < -PAN_INWARD_DEG - 0.5 || pan_geom > pan_out + 0.5) return false;

    // Reduce to (r, z) and solve 2-link for femur/tibia.
    double r = std::hypot(xl, yl);
    double dr = r - COXA_LAT;
    double dz = zl - (-COXA_DROP);
    double D = std::hypot(dr, dz);
    double dmin = std::fabs(FEMUR - TIBIA) + 1.0;
    double dmax = (FEMUR + TIBIA) - 1.0;
    if (D < dmin || D > dmax) return false;

    // Hip / knee angles within servo limits.
    double cos_knee = (FEMUR*FEMUR + TIBIA*TIBIA - D*D) / (2.0 * FEMUR * TIBIA);
    cos_knee = std::max(-1.0, std::min(1.0, cos_knee));
    double knee_deg = rad2deg(std::acos(cos_knee));
    if (knee_deg < KNEE_LO || knee_deg > KNEE_HI) return false;

    double alpha = rad2deg(std::atan2(dz, dr));
    double cos_beta = (FEMUR*FEMUR + D*D - TIBIA*TIBIA) / (2.0 * FEMUR * D);
    cos_beta = std::max(-1.0, std::min(1.0, cos_beta));
    double beta = rad2deg(std::acos(cos_beta));
    double hip_geom = alpha + beta;
    double hip_deg = 90.0 + hip_geom;
    if (hip_deg < HIP_LO || hip_deg > HIP_HI) return false;

    return true;
}

// ─── Stability (CoM vs support triangle of the 3 anchored feet) ──────────────

// Signed distance from point (px, py) to triangle (a, b, c). Positive = inside.
// Value equals the minimum distance to the three edges (with correct sign).
static double signed_dist_to_triangle(double px, double py,
                                      const std::array<double,3>& a,
                                      const std::array<double,3>& b,
                                      const std::array<double,3>& c)
{
    // Check orientation; we want CCW.
    double o = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]);
    std::array<std::array<double,3>,3> tri = {a, b, c};
    if (o < 0) std::swap(tri[1], tri[2]);

    double best = std::numeric_limits<double>::infinity();
    bool inside = true;
    for (int i = 0; i < 3; i++) {
        auto& p0 = tri[i];
        auto& p1 = tri[(i+1)%3];
        double ex = p1[0] - p0[0], ey = p1[1] - p0[1];
        // Left-turn normal of a CCW edge points INTO the polygon.
        double nx = -ey, ny = ex;
        double len = std::hypot(nx, ny);
        if (len < 1e-9) return -1e9;
        nx /= len; ny /= len;
        double d = nx * (px - p0[0]) + ny * (py - p0[1]);
        if (d < 0) inside = false;          // outside this edge's halfplane
        best = std::min(best, std::fabs(d));
    }
    return inside ? best : -best;
}

// ─── Successor generation ───────────────────────────────────────────────────

struct ActionTemplate {
    double dx, dy, dyaw_deg;
    const char* name;
};

static const std::vector<ActionTemplate> ACTIONS = {
    { 1.00,  0.00,  0.0, "fwd"      },
    { 0.85,  0.00,  1.0, "fwd+yawL" },
    { 0.85,  0.00, -1.0, "fwd+yawR" },
    { 0.00,  1.00,  0.0, "side-L"   },   // pure lateral strafe
    { 0.00, -1.00,  0.0, "side-R"   },
    { 0.00,  0.00,  1.0, "yawL"     },
    { 0.00,  0.00, -1.0, "yawR"     },
};

// Apply an action in body frame to produce the next stance state.
static State apply_action(const State& s, const ActionTemplate& a,
                          const Params& P, const Terrain& T)
{
    State out = s;
    double c = std::cos(deg2rad(s.byaw_deg));
    double sn = std::sin(deg2rad(s.byaw_deg));
    double dxb = a.dx * P.body_step_mm;
    double dyb = a.dy * P.body_step_mm;
    out.bx = s.bx + c * dxb - sn * dyb;
    out.by = s.by + sn * dxb + c * dyb;
    out.byaw_deg = s.byaw_deg + a.dyaw_deg * P.yaw_step_deg;

    // The active tripod lifts and lands at default offsets from the NEW body
    // pose. The other tripod stays anchored (feet unchanged).
    double cn = std::cos(deg2rad(out.byaw_deg));
    double snn = std::sin(deg2rad(out.byaw_deg));
    for (int leg : TRIPODS[s.active_tripod]) {
        const auto& d = DEFAULT_FOOT_BODY[leg];
        double wx = out.bx + cn * d[0] - snn * d[1];
        double wy = out.by + snn * d[0] + cn * d[1];
        double wz = T.height_at(wx, wy);   // foot rests on terrain
        out.feet[leg] = { wx, wy, wz };
    }
    out.bz = body_z_from_feet(out.feet);
    out.active_tripod = 1 - s.active_tripod;
    return out;
}

static const char* REJECT_REASON = "";

// Return +∞ if infeasible; otherwise transition cost.
static double transition_cost(const State& from, const State& to,
                              const ActionTemplate& a,
                              const Params& P, const Terrain& T)
{
    // (1) Kinematic reachability of ALL 6 feet at the new body pose.
    for (int leg = 0; leg < NLEG; leg++) {
        const auto& f = to.feet[leg];
        if (!leg_reachable(leg, to.bx, to.by, to.bz, to.byaw_deg, f[0], f[1], f[2])) {
            REJECT_REASON = LEG_NAMES_ARR[leg];
            return std::numeric_limits<double>::infinity();
        }
    }

    // (2) Stability during the swing: the tripod that lifted in this step
    // (from.active_tripod) was the supporting set. But we're transitioning
    // TO `to`, where those feet have now been re-placed. The support during
    // swing is the NON-active-at-`from` tripod, which is anchored from prior
    // step. Check CoM at `to` stays inside that triangle.
    int anchored_tri = 1 - from.active_tripod;
    auto& A = from.feet[TRIPODS[anchored_tri][0]];
    auto& B = from.feet[TRIPODS[anchored_tri][1]];
    auto& C = from.feet[TRIPODS[anchored_tri][2]];
    double sd = signed_dist_to_triangle(to.bx, to.by, A, B, C);
    if (sd <= 0.0) {
        REJECT_REASON = "stability";
        return std::numeric_limits<double>::infinity();   // falls over
    }
    double stab_deficit = std::max(0.0, P.stab_margin_mm - sd);

    // (3) Yaw-change penalty. Only penalise CW (rightward) turns; CCW turns are
    // free. This breaks the yawL/yawR tie so the planner consistently commits
    // to turning left when it must yaw in place (e.g., to squeeze through a
    // narrow gap), rather than oscillating between the two yaw directions.
    double delta_yaw = to.byaw_deg - from.byaw_deg;
    double dyaw = std::max(0.0, -delta_yaw);  // positive only for CW turns

    // (4) Elevation gain of NEW feet (feet placed in this step).
    double elev = 0.0, rough = 0.0;
    for (int leg : TRIPODS[from.active_tripod]) {
        double dz = std::max(0.0, to.feet[leg][2] - from.feet[leg][2]);
        elev += dz;
        rough += T.roughness_at(to.feet[leg][0], to.feet[leg][1]);
    }

    (void)a;
    return P.c_swing
         + P.w_yaw_deg * dyaw
         + P.w_elev * elev
         + P.w_stab * stab_deficit
         + P.w_rough * rough;
}

// Build the 2D admissible heuristic by running backward Dijkstra from the
// goal over the terrain grid. Edge cost = (cell_dist / body_step) * c_swing
// + w_elev * max(0, Δh_going_forward). This is a strict lower bound on the
// true per-swing cost the real planner pays (it ignores yaw, kinematics,
// stability, and sums elevation over multiple feet → always admissible).
static HeuristicGrid build_heuristic_grid(const Map& m, const Params& P) {
    HeuristicGrid G;
    G.nx = m.terrain.nx;
    G.ny = m.terrain.ny;
    G.cell = m.terrain.cell;
    G.origin_x = m.terrain.origin_x;
    G.origin_y = m.terrain.origin_y;
    G.h.assign((size_t)G.nx * G.ny, std::numeric_limits<double>::infinity());

    struct PQEntry { double cost; int i, j; };
    auto cmp = [](const PQEntry& a, const PQEntry& b){ return a.cost > b.cost; };
    std::priority_queue<PQEntry, std::vector<PQEntry>, decltype(cmp)> pq(cmp);

    // Seed: every cell within goal_tol of the goal has h = 0.
    int seeded = 0;
    for (int j = 0; j < G.ny; j++) for (int i = 0; i < G.nx; i++) {
        double wx = G.origin_x + i * G.cell;
        double wy = G.origin_y + j * G.cell;
        if (std::hypot(wx - m.goal_x, wy - m.goal_y) <= m.goal_tol) {
            G.h[j * G.nx + i] = 0.0;
            pq.push({0.0, i, j});
            seeded++;
        }
    }
    // Safety: if the goal-tol radius is smaller than a cell, seed the single
    // nearest cell so the grid isn't empty.
    if (seeded == 0) {
        int ci = (int)std::round((m.goal_x - G.origin_x) / G.cell);
        int cj = (int)std::round((m.goal_y - G.origin_y) / G.cell);
        ci = std::max(0, std::min(G.nx - 1, ci));
        cj = std::max(0, std::min(G.ny - 1, cj));
        G.h[cj * G.nx + ci] = 0.0;
        pq.push({0.0, ci, cj});
    }

    // 8-connected neighbours.
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    while (!pq.empty()) {
        PQEntry e = pq.top(); pq.pop();
        int i = e.i, j = e.j;
        if (e.cost > G.h[j * G.nx + i]) continue;  // stale
        double z_here = m.terrain.h[j * G.nx + i];
        for (int k = 0; k < 8; k++) {
            int ni = i + dx[k], nj = j + dy[k];
            if (ni < 0 || ni >= G.nx || nj < 0 || nj >= G.ny) continue;
            double z_next = m.terrain.h[nj * G.nx + ni];
            double step_mm = G.cell * std::hypot((double)dx[k], (double)dy[k]);
            // Forward direction is (ni,nj) → (i,j) since we're expanding
            // backward from the goal, so elev gain = max(0, z_here - z_next).
            double elev = std::max(0.0, z_here - z_next);
            double step_cost = (step_mm / P.body_step_mm) * P.c_swing
                             + P.w_elev * elev;
            double nc = e.cost + step_cost;
            if (nc < G.h[nj * G.nx + ni]) {
                G.h[nj * G.nx + ni] = nc;
                pq.push({nc, ni, nj});
            }
        }
    }
    return G;
}

// LRTA* learned bound: records per-state lower bounds discovered during
// the real search. After an outer step from s, we set
//     learned[s] ← max(learned[s], best_f_from_inner_search)
// which is admissible because best_f is a Bellman backup of admissible
// terminal heuristics. Over repeated visits to a local minimum, this
// monotonically raises h until the planner gives up on the dead end.
using LearnedH = std::unordered_map<StateKey, double, StateKeyHash>;

// Recency / tabu set: the set of quantized state keys visited in the
// last TABU_WINDOW outer steps. Any successor inside the inner search
// whose key is in this set gets a per-edge penalty added to its step
// cost — steering the planner away from immediately-revisiting states.
// This is what breaks 2-cycle oscillation at a local minimum: the
// Dijkstra heuristic says "back that way is closer" but the tabu
// penalty overwhelms the h-difference for one or two steps, long
// enough to commit past the minimum.
using TabuSet = std::unordered_set<StateKey, StateKeyHash>;
static constexpr double TABU_PENALTY = 8.0;  // ~8 free swings' worth

static double heuristic(const State& s, const Map&, const Params& P,
                        const HeuristicGrid& G, const LearnedH& L) {
    double base = G.lookup(s.bx, s.by);
    auto it = L.find(make_key(s, P));
    return (it != L.end()) ? std::max(base, it->second) : base;
}

static bool at_goal(const State& s, const Map& m) {
    double dx = m.goal_x - s.bx, dy = m.goal_y - s.by;
    return std::hypot(dx, dy) <= m.goal_tol;
}

// ─── ARA* inner search (bounded depth) ───────────────────────────────────────
//
// Returns the best first action to take from `start`. For horizon=1 this is
// pure greedy (pick argmin_a [ c(s, s') + eps * h(s') ]). For horizon>1 it
// runs a depth-limited A* and returns the first action of the best found
// path.
//
// `expansions` and `best_f` are reported out for diagnostics.

struct SearchResult {
    bool found;
    int first_action_idx;
    State first_next;
    double best_f;
    int expansions;
};

static SearchResult inner_search(const State& start, const Map& m,
                                 const Params& P, const HeuristicGrid& G,
                                 const LearnedH& L, const TabuSet& tabu) {
    SearchResult R{};
    R.first_action_idx = -1;
    R.best_f = std::numeric_limits<double>::infinity();
    R.expansions = 0;

    struct Node {
        State s;
        double g;
        int depth;
        int first_action_idx;    // action taken at depth 0
        State first_next;        // state after first action
    };

    struct PQEntry { double f; size_t id; };
    auto cmp = [](const PQEntry& a, const PQEntry& b){ return a.f > b.f; };
    std::priority_queue<PQEntry, std::vector<PQEntry>, decltype(cmp)> open(cmp);
    std::vector<Node> nodes;
    std::unordered_map<StateKey, double, StateKeyHash> best_g;

    auto start_key = make_key(start, P);
    best_g[start_key] = 0.0;
    nodes.push_back({start, 0.0, 0, -1, start});
    open.push({P.eps * heuristic(start, m, P, G, L), 0});

    while (!open.empty()) {
        auto [f, id] = open.top(); open.pop();
        Node node = nodes[id];
        R.expansions++;

        // Terminate paths at goal OR at max depth.
        bool terminate = at_goal(node.s, m) || node.depth >= P.horizon;
        if (terminate) {
            if (node.depth == 0) continue;   // start itself isn't a plan
            if (f < R.best_f) {
                R.best_f = f;
                R.first_action_idx = node.first_action_idx;
                R.first_next = node.first_next;
                R.found = true;
            }
            continue;
        }

        for (size_t ai = 0; ai < ACTIONS.size(); ai++) {
            State s2 = apply_action(node.s, ACTIONS[ai], P, m.terrain);
            REJECT_REASON = "-";
            double step = transition_cost(node.s, s2, ACTIONS[ai], P, m.terrain);
            // Tabu penalty: landing in a recently-visited state costs extra.
            if (std::isfinite(step) && tabu.count(make_key(s2, P))) {
                step += TABU_PENALTY;
            }
            if (getenv("PLANNER_DEBUG") && node.depth == 0) {
                std::cerr << "  [d0] " << ACTIONS[ai].name
                          << " step_cost=" << step
                          << "  h=" << heuristic(s2, m, P, G, L)
                          << "  f=" << (std::isfinite(step) ? step + P.eps*heuristic(s2, m, P, G, L) : step)
                          << "  reject=" << REJECT_REASON << "\n";
            }
            if (!std::isfinite(step)) continue;
            double g2 = node.g + step;
            auto key = make_key(s2, P);
            auto it = best_g.find(key);
            if (it != best_g.end() && g2 >= it->second) continue;
            best_g[key] = g2;

            int first_idx = (node.depth == 0) ? (int)ai : node.first_action_idx;
            State first_nxt = (node.depth == 0) ? s2 : node.first_next;
            nodes.push_back({s2, g2, node.depth + 1, first_idx, first_nxt});
            double f2 = g2 + P.eps * heuristic(s2, m, P, G, L);
            open.push({f2, nodes.size() - 1});
        }
    }
    return R;
}

// ─── Initial state from map start ────────────────────────────────────────────

static State make_initial_state(const Map& m) {
    State s;
    s.bx = m.start_x;
    s.by = m.start_y;
    s.byaw_deg = m.start_yaw_deg;
    s.active_tripod = 0;
    double c = std::cos(deg2rad(s.byaw_deg));
    double sn = std::sin(deg2rad(s.byaw_deg));
    for (int leg = 0; leg < NLEG; leg++) {
        const auto& d = DEFAULT_FOOT_BODY[leg];
        double wx = s.bx + c * d[0] - sn * d[1];
        double wy = s.by + sn * d[0] + c * d[1];
        double wz = m.terrain.height_at(wx, wy);
        s.feet[leg] = { wx, wy, wz };
    }
    s.bz = body_z_from_feet(s.feet);
    return s;
}

// ─── Plan output ─────────────────────────────────────────────────────────────

static void write_plan(const std::string& path,
                       const Map& m,
                       const std::vector<State>& plan,
                       const std::vector<int>& action_hist,
                       double total_cost,
                       int total_expansions,
                       const Params& P)
{
    std::ofstream f(path);
    f << "# hexapod plan\n";
    f << "# map: " << m.name << "\n";
    f << "# grid nx=" << m.terrain.nx << " ny=" << m.terrain.ny
      << " cell_mm=" << m.terrain.cell
      << " origin_x=" << m.terrain.origin_x
      << " origin_y=" << m.terrain.origin_y << "\n";
    f << "# start " << m.start_x << " " << m.start_y << " " << m.start_yaw_deg << "\n";
    f << "# goal  " << m.goal_x << " " << m.goal_y << " tol=" << m.goal_tol << "\n";
    f << "# horizon=" << P.horizon << " eps=" << P.eps
      << " body_step_mm=" << P.body_step_mm
      << " yaw_step_deg=" << P.yaw_step_deg << "\n";
    f << "# weights: c_swing=" << P.c_swing
      << " w_yaw_deg=" << P.w_yaw_deg
      << " w_elev=" << P.w_elev
      << " w_stab=" << P.w_stab
      << " w_rough=" << P.w_rough
      << " stab_margin_mm=" << P.stab_margin_mm << "\n";
    f << "# steps=" << plan.size() - 1
      << " total_cost=" << total_cost
      << " total_expansions=" << total_expansions << "\n";
    f << "# columns: step action body_x body_y body_yaw_deg active_tripod "
         "LFx LFy LFz LMx LMy LMz LBx LBy LBz RFx RFy RFz RMx RMy RMz RBx RBy RBz\n";
    for (size_t i = 0; i < plan.size(); i++) {
        const State& s = plan[i];
        const char* an = (i == 0) ? "START"
                                  : (action_hist[i-1] >= 0 ? ACTIONS[action_hist[i-1]].name : "?");
        f << i << " " << an << " "
          << s.bx << " " << s.by << " " << s.byaw_deg << " " << s.active_tripod;
        for (int leg = 0; leg < NLEG; leg++)
            f << " " << s.feet[leg][0] << " " << s.feet[leg][1] << " " << s.feet[leg][2];
        f << "\n";
    }
}

}  // namespace hex

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    using namespace hex;
    if (argc < 2) {
        std::cerr << "usage: planner <map_dir> [--horizon N] [--eps E] [--out path] [--max-steps K]\n";
        return 1;
    }

    std::string map_dir = argv[1];
    std::string out_path = "plan.txt";
    Params P;
    int max_steps = 200;

    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--horizon" && i+1 < argc) P.horizon = std::atoi(argv[++i]);
        else if (a == "--eps" && i+1 < argc) P.eps = std::atof(argv[++i]);
        else if (a == "--out" && i+1 < argc) out_path = argv[++i];
        else if (a == "--max-steps" && i+1 < argc) max_steps = std::atoi(argv[++i]);
        else if (a == "--w-stab" && i+1 < argc) P.w_stab = std::atof(argv[++i]);
        else if (a == "--w-elev" && i+1 < argc) P.w_elev = std::atof(argv[++i]);
        else if (a == "--body-step" && i+1 < argc) P.body_step_mm = std::atof(argv[++i]);
    }

    Map m = load_map(map_dir);
    std::cout << "Loaded map '" << m.name << "' "
              << m.terrain.nx << "x" << m.terrain.ny
              << " cell=" << m.terrain.cell << "mm\n";
    std::cout << "Start (" << m.start_x << "," << m.start_y << ") yaw=" << m.start_yaw_deg << "\n";
    std::cout << "Goal  (" << m.goal_x << "," << m.goal_y << ") tol=" << m.goal_tol << "\n";
    std::cout << "Horizon=" << P.horizon << " eps=" << P.eps << "\n";

    HeuristicGrid G = build_heuristic_grid(m, P);
    double h_start = G.lookup(m.start_x, m.start_y);
    std::cout << "Heuristic grid built (" << G.nx << "x" << G.ny
              << ")  h(start) = " << h_start << "\n\n";

    State s = make_initial_state(m);
    std::vector<State> plan = { s };
    std::vector<int> action_hist;
    double total_cost = 0.0;
    int total_expansions = 0;

    // Oscillation detection + tabu window: track recent state keys.
    // • If the same key appears OSC_LIMIT times in the last OSC_WINDOW
    //   steps, we're stuck and terminate.
    // • The last TABU_WINDOW keys are passed to inner_search as a
    //   recency set; any successor hashing to one of them pays
    //   TABU_PENALTY on its step cost. This forces the planner to
    //   commit past 2-cycles in local minima rather than shuffle
    //   back and forth.
    static constexpr int OSC_WINDOW  = 20;
    static constexpr int OSC_LIMIT   = 5;
    static constexpr int TABU_WINDOW = 8;
    std::deque<StateKey> recent_keys;

    // LRTA* learned lower bounds on cost-to-go, keyed by quantized state.
    // After each outer step we back up the best f-value returned by the
    // horizon-limited search. Each such f = g(path) + eps*h(leaf) is an
    // (eps-)admissible estimate of true cost-to-go from the state we
    // searched FROM; taking max with any previous value only tightens
    // the bound and cannot break admissibility. This is what breaks
    // oscillation in dead-end regions: each revisit of a local minimum
    // raises its h value until the planner prefers an alternate route.
    LearnedH learned;

    for (int step = 0; step < max_steps; step++) {
        if (at_goal(s, m)) {
            std::cout << "Reached goal at step " << step << "\n";
            break;
        }

        // Oscillation check.
        StateKey cur_key = make_key(s, P);
        recent_keys.push_back(cur_key);
        if ((int)recent_keys.size() > OSC_WINDOW) recent_keys.pop_front();
        if ((int)recent_keys.size() == OSC_WINDOW) {
            int repeats = 0;
            for (auto& k : recent_keys) if (k == cur_key) repeats++;
            if (repeats >= OSC_LIMIT) {
                std::cout << "Oscillation detected at step " << step
                          << " — state repeated " << repeats << "x in last "
                          << OSC_WINDOW << " steps. Terminating.\n";
                break;
            }
        }

        // Build the tabu set from the last TABU_WINDOW visited keys.
        TabuSet tabu;
        int tabu_start = std::max(0, (int)recent_keys.size() - TABU_WINDOW);
        for (int k = tabu_start; k < (int)recent_keys.size(); k++) {
            tabu.insert(recent_keys[k]);
        }

        SearchResult r = inner_search(s, m, P, G, learned, tabu);
        total_expansions += r.expansions;
        if (!r.found) {
            std::cout << "No feasible action from step " << step
                      << " — stuck at (" << s.bx << "," << s.by << ")\n";
            // Diagnostic: try each action directly, print reject reason.
            for (size_t ai = 0; ai < ACTIONS.size(); ai++) {
                State s2 = apply_action(s, ACTIONS[ai], P, m.terrain);
                REJECT_REASON = "?";
                double c = transition_cost(s, s2, ACTIONS[ai], P, m.terrain);
                std::cout << "  action " << ACTIONS[ai].name
                          << " -> body=(" << s2.bx << "," << s2.by << "," << s2.byaw_deg
                          << ")  cost=" << c
                          << "  reject=" << REJECT_REASON << "\n";
            }
            break;
        }

        // LRTA* Bellman backup. r.best_f is the best (eps-inflated) cost
        // of any path found within the horizon from `s`. Deflate by eps
        // to recover an admissible lower bound on true cost-to-go from s,
        // then take max with any previously learned value. On subsequent
        // visits to `s` (or anything hashing to the same key), heuristic()
        // returns this inflated value, steering the planner away from
        // revisited local minima.
        double backed_up = r.best_f / std::max(1e-9, P.eps);
        auto le = learned.find(cur_key);
        double prev = (le != learned.end()) ? le->second : 0.0;
        learned[cur_key] = std::max(prev, backed_up);

        double step_cost = transition_cost(s, r.first_next, ACTIONS[r.first_action_idx], P, m.terrain);
        total_cost += step_cost;
        s = r.first_next;
        plan.push_back(s);
        action_hist.push_back(r.first_action_idx);

        std::cout << "step " << step
                  << " action=" << ACTIONS[r.first_action_idx].name
                  << " body=(" << s.bx << "," << s.by << "," << s.byaw_deg << ")"
                  << " cost=" << step_cost
                  << " exp=" << r.expansions << "\n";
    }

    std::cout << "\nTotal steps: " << plan.size() - 1
              << "  total_cost=" << total_cost
              << "  total_expansions=" << total_expansions << "\n";
    std::cout << "Goal reached: " << (at_goal(s, m) ? "YES" : "NO") << "\n";
    write_plan(out_path, m, plan, action_hist, total_cost, total_expansions, P);
    std::cout << "Wrote " << out_path << "\n";
    return at_goal(s, m) ? 0 : 2;
}
