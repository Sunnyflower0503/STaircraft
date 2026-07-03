#include "aircraft.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gj2 {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double x, double lo, double hi) {
    return std::min(std::max(x, lo), hi);
}

double wrapToPi(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a < 0.0) {
        a += 2.0 * kPi;
    }
    return a - kPi;
}

Vec3 satVec(const Vec3& x, const Vec3& lim) {
    return {clamp(x[0], -lim[0], lim[0]), clamp(x[1], -lim[1], lim[1]), clamp(x[2], -lim[2], lim[2])};
}

Vec3 satVec(const Vec3& x, double lo, double hi) {
    return {clamp(x[0], lo, hi), clamp(x[1], lo, hi), clamp(x[2], lo, hi)};
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 scale(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
}

Vec3 matVec(const Mat3& A, const Vec3& x) {
    return {
        A[0][0] * x[0] + A[0][1] * x[1] + A[0][2] * x[2],
        A[1][0] * x[0] + A[1][1] * x[1] + A[1][2] * x[2],
        A[2][0] * x[0] + A[2][1] * x[1] + A[2][2] * x[2],
    };
}

Mat3 transpose(const Mat3& A) {
    return {{{A[0][0], A[1][0], A[2][0]},
             {A[0][1], A[1][1], A[2][1]},
             {A[0][2], A[1][2], A[2][2]}}};
}

Vec3 solve3(const Mat3& A, const Vec3& b) {
    const double a00 = A[0][0], a01 = A[0][1], a02 = A[0][2];
    const double a10 = A[1][0], a11 = A[1][1], a12 = A[1][2];
    const double a20 = A[2][0], a21 = A[2][1], a22 = A[2][2];
    const double det = a00 * (a11 * a22 - a12 * a21)
                     - a01 * (a10 * a22 - a12 * a20)
                     + a02 * (a10 * a21 - a11 * a20);
    if (std::abs(det) < 1e-14) {
        return {0.0, 0.0, 0.0};
    }
    const double invdet = 1.0 / det;
    return {
        invdet * (b[0] * (a11 * a22 - a12 * a21) - a01 * (b[1] * a22 - a12 * b[2]) + a02 * (b[1] * a21 - a11 * b[2])),
        invdet * (a00 * (b[1] * a22 - a12 * b[2]) - b[0] * (a10 * a22 - a12 * a20) + a02 * (a10 * b[2] - b[1] * a20)),
        invdet * (a00 * (a11 * b[2] - b[1] * a21) - a01 * (a10 * b[2] - b[1] * a20) + b[0] * (a10 * a21 - a11 * a20)),
    };
}

Vec4 normalizeQuat(Vec4 q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n <= 0.0) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    for (double& v : q) {
        v /= n;
    }
    return q;
}

Mat3 quatToRotm(Vec4 q) {
    q = normalizeQuat(q);
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)},
             {2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
             {2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)}}};
}

Vec4 quatConj(const Vec4& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

Vec4 quatMultiply(const Vec4& a, const Vec4& b) {
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
}

Vec4 eul2quatZYX(double yaw, double pitch, double roll) {
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    return normalizeQuat({
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    });
}

double yawFromQuatZYX(const Vec4& q_in) {
    const Vec4 q = normalizeQuat(q_in);
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double interp(const double* bp, const double* tab, int n, double x) {
    if (x <= bp[0]) {
        return tab[0];
    }
    if (x >= bp[n - 1]) {
        return tab[n - 1];
    }
    int idx = 0;
    while (idx + 1 < n && bp[idx + 1] <= x) {
        ++idx;
    }
    const double frac = (x - bp[idx]) / (bp[idx + 1] - bp[idx]);
    return tab[idx] + frac * (tab[idx + 1] - tab[idx]);
}

template <int N>
double interpArr(const std::array<double, N>& bp, const std::array<double, N>& tab, double x) {
    return interp(bp.data(), tab.data(), N, x);
}

double aeroEnable(int mode_out) {
    if (mode_out < 0) {
        return 1.0;
    }
    if (mode_out == 2) {
        return 0.0;
    }
    return (mode_out == 0 || mode_out == 1 || mode_out == 3) ? 1.0 : 0.0;
}

double aeroCl(const Param& p, double alpha) {
    if (p.aero_extend_flat_plate) {
        const double a = wrapToPi(alpha);
        return interpArr<361>(p.aero_fullalpha_arr, p.aero.CL_full_arr, a);
    }
    return interpArr<19>(p.bigalpha_arr, p.aero.CL_arr, alpha);
}

double aeroCd(const Param& p, double alpha) {
    if (p.aero_extend_flat_plate) {
        const double a = wrapToPi(alpha);
        return interpArr<361>(p.aero_fullalpha_arr, p.aero.CD_full_arr, a);
    }
    return p.aero.CD0 + interpArr<19>(p.bigalpha_arr, p.aero.CD_arr, alpha);
}

double smoothstep(double x) {
    x = clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

double sigmaFlatPlate(double alpha0, double k, double alpha) {
    return (1.0 + std::tanh(k * alpha0 * alpha0 - k * alpha * alpha)) /
           (1.0 + std::tanh(k * alpha0 * alpha0));
}

struct FlatPlateFit {
    double c0{};
    double c1{};
    double c2{};
    double c3{};
    double alpha0{};
    double kL{};
    double kD{};
    double trusted_min{};
    double trusted_max{};
};

std::pair<double, double> flatPlateCoeff(double alpha, const FlatPlateFit& fp) {
    const double denom = (fp.c2 - fp.c3) * std::cos(alpha) * std::cos(alpha) + fp.c3;
    const double CL_s = 0.5 * fp.c2 * fp.c2 / denom * std::sin(2.0 * alpha);
    const double CD_s = fp.c0 + (fp.c2 * fp.c3 / denom) * std::sin(alpha) * std::sin(alpha);
    const double CL_l = fp.c1 * std::sin(2.0 * alpha);
    const double CD_l = fp.c0 + 2.0 * fp.c1 * std::sin(alpha) * std::sin(alpha);
    const double sigma_L = sigmaFlatPlate(fp.alpha0, fp.kL, alpha);
    const double sigma_D = sigmaFlatPlate(fp.alpha0, fp.kD, alpha);
    const double CL = CL_s * sigma_L + CL_l * (1.0 - sigma_L);
    const double CD = CD_s * sigma_D + CD_l * (1.0 - sigma_D);
    return {CD, CL};
}

Vec3 aeroForceToBody(double alpha, double beta, double CD, double CY, double CL) {
    return {
        -CD * std::cos(alpha) * std::cos(beta) - CY * std::cos(alpha) * std::sin(beta) + CL * std::sin(alpha),
        -CD * std::sin(beta) + CY * std::cos(beta),
        -CD * std::sin(alpha) * std::cos(beta) - CY * std::sin(alpha) * std::sin(beta) - CL * std::cos(alpha),
    };
}

}  // namespace

double sat(double x, double x_min, double x_max) {
    return clamp(x, x_min, x_max);
}

Vec3 sat(const Vec3& x, double x_min, double x_max) {
    return satVec(x, x_min, x_max);
}

RotorThrust tandemRotorThrust(double dt, double alpha, double beta, double tas, double prop_spin, const Param& p, double prop_angle) {
    const double n_rpm = (-3424.0 * dt + 12810.0) * dt - 4.0;
    const double n_rps = std::max(n_rpm / 60.0, 0.0);
    if (n_rps <= 0.0) {
        return {};
    }
    const double ca = std::cos(prop_angle);
    const double sa = std::sin(prop_angle);
    const double v_axial = tas * std::cos(alpha) * std::cos(beta) * ca - tas * std::sin(alpha) * std::cos(beta) * sa;
    const double J = v_axial / (n_rps * p.prop_D);
    double CT = 0.0;
    double CM = 0.0;
    if (J <= 0.8) {
        CT = ((0.3101 * J - 0.5816) * J + 0.1747) * J + 0.09582;
        CM = (-0.0169 * J + 0.0071) * J + 0.0103;
    }
    if (CT <= 0.0) {
        return {0.0, 0.0, n_rps, 0.0};
    }
    const double n2 = n_rps * n_rps;
    return {
        n2 * CT * 0.0017048839192575996 * 1.225,
        prop_spin * CM * n2 * 0.00034643241239314424 * 1.225,
        n_rps,
        CT,
    };
}

ForceMoment tandemAddpropFm(double dt_left, double dt_right, const Param& p) {
    auto thrust = [&](double dt) {
        const double n = ((-3424.0 * dt + 12810.0) * dt - 4.0) / 60.0;
        if (n <= 0.0) {
            return 0.0;
        }
        const double CT = ((5.87e-8 * n - 1.61e-5) * n + 0.0014) * n + 0.1137;
        return CT * p.rho * n * n * std::pow(p.addprop_D, 4.0);
    };
    const double T_left = thrust(dt_left);
    const double T_right = thrust(dt_right);
    ForceMoment fm;
    fm.f = {0.0, 0.0, -T_left - T_right};
    fm.m = {T_left * p.addprop_y - T_right * p.addprop_y,
            T_left * p.addprop_x + T_right * p.addprop_x,
            0.0};
    return fm;
}

ForceMoment tandemRotorFm(const std::array<double, 10>& delta_t, const Vec3& ve, const Mat3& R_eb, const Param& p) {
    const Mat3 R_be = transpose(R_eb);
    const Vec3 va_b = matVec(R_be, sub(ve, p.wind));
    const double va = norm(va_b);
    const double alpha = va < 1e-6 ? 0.0 : std::atan2(va_b[2], va_b[0]);
    const double beta = va < 1e-6 ? 0.0 : std::atan2(va_b[1], std::hypot(va_b[0], va_b[2]));
    ForceMoment fm;
    for (int i = 0; i < 8; ++i) {
        const double dt_i = clamp(delta_t[i], p.rotor_throttle_min, 1.0);
        const RotorThrust rt = tandemRotorThrust(dt_i, alpha, beta, va, p.prop_spin[i], p, p.prop_angle[i]);
        if (rt.thrust <= 0.0) {
            continue;
        }
        const double ca = std::cos(p.prop_angle[i]);
        const double sa = std::sin(p.prop_angle[i]);
        const Vec3 f_i{rt.thrust * ca, 0.0, -rt.thrust * sa};
        fm.f = add(fm.f, f_i);
        fm.m = add(fm.m, cross(p.prop_pos[i], f_i));
        fm.m = add(fm.m, {rt.m_prop * ca, 0.0, rt.m_prop * sa});
    }
    const ForceMoment addprop = tandemAddpropFm(delta_t[8], delta_t[9], p);
    fm.f = add(fm.f, addprop.f);
    fm.m = add(fm.m, addprop.m);
    return fm;
}

ForceMoment tandemAeroFm(const Vec3& ve, const Mat3& R_eb, const Vec3& omega_b, double daeL, double daeR,
                         const std::array<double, 8>& /*delta_t*/, const Param& p, int mode_out) {
    const Mat3 R_be = transpose(R_eb);
    const Vec3 va_b = matVec(R_be, sub(ve, p.wind));
    const double va = norm(va_b);
    if (va < 1e-9) {
        return {};
    }
    const double alpha = std::atan2(va_b[2], va_b[0]);
    const double beta = std::atan2(va_b[1], std::hypot(va_b[0], va_b[2]));
    const double qbar = 0.5 * p.rho * va * va;
    const double alpha_dyn = clamp(alpha, -0.5, 0.5);
    const double p_hat = omega_b[0] * p.b / (2.0 * va);
    const double q_hat = omega_b[1] * p.MAC / (2.0 * va);
    const double r_hat = omega_b[2] * p.b / (2.0 * va);
    const double K = aeroEnable(mode_out);

    const double CD_de_L = p.aero.CDdelta_aeL * daeL;
    const double CL_de_L = p.aero.CLdelta_aeL * daeL;
    const double CD_de_R = p.aero.CDdelta_aeR * daeR;
    const double CL_de_R = p.aero.CLdelta_aeR * daeR;
    const Vec3 F_de_L = scale(aeroForceToBody(alpha, beta, CD_de_L, 0.0, CL_de_L), qbar * p.S);
    const Vec3 F_de_R = scale(aeroForceToBody(alpha, beta, CD_de_R, 0.0, CL_de_R), qbar * p.S);
    const Vec3 F_elevon = add(F_de_L, F_de_R);
    const Vec3 M_elevon = scale({
        p.b * (p.aero.Cldelta_aeL * daeL + p.aero.Cldelta_aeR * daeR),
        p.MAC * (p.aero.Cmdelta_aeL * daeL + p.aero.Cmdelta_aeR * daeR),
        p.b * (p.aero.Cndelta_aeL * daeL + p.aero.Cndelta_aeR * daeR),
    }, qbar * p.S);

    const double CD_static = aeroCd(p, alpha);
    const double CL_static = aeroCl(p, alpha);
    const double Cm_static = interpArr<19>(p.bigalpha_arr, p.aero.Cm_arr, alpha);
    const Vec3 F_sfree = scale(aeroForceToBody(alpha, beta, CD_static, 0.0, CL_static), qbar * p.S_free);
    const Vec3 M_sfree = {0.0, K * qbar * p.S_free * p.MAC * Cm_static, 0.0};

    const double CY_dyn = p.aero.CYbeta * beta
        + interpArr<14>(p.alpha_arr, p.aero.CYp_arr, alpha_dyn) * p_hat
        + interpArr<14>(p.alpha_arr, p.aero.CYr_arr, alpha_dyn) * r_hat;
    const double CL_dyn = p.aero.CLq * q_hat + p.aero.CLalpdot * p.alpha_dot_hat;
    const Vec3 F_sw = scale(aeroForceToBody(alpha, beta, 0.0, CY_dyn, CL_dyn), K * qbar * p.S);
    const double Cl_dyn = interpArr<14>(p.alpha_arr, p.aero.Clbeta_arr, alpha_dyn) * beta
        + interpArr<14>(p.alpha_arr, p.aero.Clp_arr, alpha_dyn) * p_hat
        + interpArr<14>(p.alpha_arr, p.aero.Clr_arr, alpha_dyn) * r_hat;
    const double Cm_dyn = p.aero.Cmq * q_hat + p.aero.Cmalpdot * p.alpha_dot_hat;
    const double Cn_dyn = interpArr<14>(p.alpha_arr, p.aero.Cnbeta_arr, alpha_dyn) * beta
        + interpArr<14>(p.alpha_arr, p.aero.Cnp_arr, alpha_dyn) * p_hat
        + interpArr<14>(p.alpha_arr, p.aero.Cnr_arr, alpha_dyn) * r_hat;
    const Vec3 M_sw = scale({p.b * Cl_dyn, p.MAC * Cm_dyn, p.b * Cn_dyn}, K * qbar * p.S);

    Vec3 F_slip{0.0, 0.0, 0.0};
    for (int i = 0; i < 8; ++i) {
        const double alpha_slip = p.alpha_slip_switch > 0.5 ? std::atan2(va_b[2], va_b[0]) : alpha;
        const double qbar_slip = p.DP_slip_switch > 0.5 ? 0.5 * p.rho * dot(va_b, va_b) : qbar;
        const double CD_slip = aeroCd(p, alpha_slip);
        const double CL_slip = aeroCl(p, alpha_slip);
        F_slip = add(F_slip, scale(aeroForceToBody(alpha, beta, CD_slip, 0.0, CL_slip), qbar_slip * p.S_slip[i]));
    }

    return {add(add(add(F_elevon, F_sfree), F_sw), F_slip),
            add(add(M_elevon, M_sfree), M_sw)};
}

Vec3 zxGroundContactForce(const Vec3& pe, const Vec3& ve, const Vec3& f_non_ground_e, const Param& p) {
    const auto& g = p.ground;
    const double penetration = std::max(pe[2] - g.z, 0.0);
    const bool near_ground = pe[2] >= g.z - g.tol;
    Vec3 f{0.0, 0.0, 0.0};
    if (near_ground || penetration > 0.0) {
        const double normal = std::max(0.0, f_non_ground_e[2] + g.c * std::max(ve[2], 0.0) + g.k * penetration);
        f[2] = -normal;
        const Vec3 f_fric_des{-g.xy_damping * ve[0], -g.xy_damping * ve[1], 0.0};
        const double fric_norm = std::hypot(f_fric_des[0], f_fric_des[1]);
        const double fric_lim = g.mu * normal;
        if (fric_norm > fric_lim && fric_norm > 1e-12) {
            f[0] = f_fric_des[0] * fric_lim / fric_norm;
            f[1] = f_fric_des[1] * fric_lim / fric_norm;
        } else {
            f[0] = f_fric_des[0];
            f[1] = f_fric_des[1];
        }
    }
    return f;
}

namespace {

Vec3 totalRollPitchMoment(const std::array<double, 2>& dae, const std::array<double, 8>& u_main, const State& x, const Param& p) {
    const Vec4 q{x[6], x[7], x[8], x[9]};
    const Mat3 R_eb = quatToRotm(q);
    const Vec3 ve{x[3], x[4], x[5]};
    const Vec3 omega{x[10], x[11], x[12]};
    std::array<double, 10> delta{};
    std::array<double, 8> delta8{};
    for (int i = 0; i < 8; ++i) {
        delta[i] = u_main[i];
        delta8[i] = u_main[i];
    }
    const ForceMoment rotor = tandemRotorFm(delta, ve, R_eb, p);
    const ForceMoment aero = tandemAeroFm(ve, R_eb, omega, dae[0], dae[1], delta8, p);
    return {rotor.m[0] + aero.m[0], rotor.m[1] + aero.m[1], 0.0};
}

void applyPitchThrottleDelta(std::array<double, 8>& u_main, double delta, const Param& p) {
    for (int idx : p.fw_pitch_throttle_front_idx) {
        u_main[idx] += delta;
    }
    for (int idx : p.fw_pitch_throttle_rear_idx) {
        u_main[idx] -= delta;
    }
    for (double& v : u_main) {
        v = clamp(v, p.fw_throttle_min, p.fw_throttle_max);
    }
}

double signedPitchDelta(const std::array<double, 8>& u_pitch, const std::array<double, 8>& u_base, const Param& p) {
    double d_front = 0.0;
    double d_rear = 0.0;
    for (int idx : p.fw_pitch_throttle_front_idx) {
        d_front += u_pitch[idx] - u_base[idx];
    }
    for (int idx : p.fw_pitch_throttle_rear_idx) {
        d_rear += u_base[idx] - u_pitch[idx];
    }
    return 0.5 * (d_front / 4.0 + d_rear / 4.0);
}

}  // namespace

Param initParamZx() {
    Param p;
    const double Ixx = 0.3236731;
    const double Iyy = 0.45521417;
    const double Izz = 0.616930086;
    const double Ixz = 0.167493906;
    p.J = {{{Ixx, 0.0, -Ixz}, {0.0, Iyy, 0.0}, {-Ixz, 0.0, Izz}}};
    p.S_slip = {0.03048, 0.06858, 0.06858, 0.03048, 0.03048, 0.06858, 0.06858, 0.03048};
    p.S_slip_y = {-0.58, -0.175, 0.175, 0.58, -0.58, -0.175, 0.175, 0.58};
    p.S_free = p.S;
    for (double s : p.S_slip) {
        p.S_free -= s;
    }
    p.prop_pos = {{{0.371, -0.5845, 0.175}, {0.371, -0.175, 0.175}, {0.371, 0.175, 0.175}, {0.371, 0.5845, 0.175},
                   {-0.329, -0.5845, -0.175}, {-0.329, -0.175, -0.175}, {-0.329, 0.175, -0.175}, {-0.329, 0.5845, -0.175}}};
    p.prop_angle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    p.prop_spin = {-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0};

    p.aero.CD0 = 0.0;
    p.aero.CDdelta_aeL = 0.01905225;
    p.aero.CDdelta_aeR = 0.01905225;
    p.aero.CYbeta = -0.245;
    p.aero.CLdelta_aeL = 0.2066238;
    p.aero.CLdelta_aeR = 0.2066238;
    p.aero.CLq = 6.27;
    p.aero.CLalpdot = 0.627;
    p.aero.Cldelta_aeL = 0.054435;
    p.aero.Cldelta_aeR = -0.054435;
    p.aero.Cmdelta_aeL = -0.33234;
    p.aero.Cmdelta_aeR = -0.33234;
    p.aero.Cmq = -11.019236;
    p.aero.Cmalpdot = -4.4076944;
    p.aero.Cndelta_aeL = -0.0051;
    p.aero.Cndelta_aeR = 0.0051;
    p.bigalpha_arr = {-6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 15, 20, 25, 30, 35, 40, 45, 50};
    for (double& a : p.bigalpha_arr) {
        a *= p.D2R;
    }
    p.alpha_arr = {-4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    for (double& a : p.alpha_arr) {
        a *= p.D2R;
    }
    p.aero.CD_arr = {0.05703, 0.05363, 0.05478, 0.06171, 0.07262, 0.08715, 0.10403, 0.12350, 0.14423, 0.16806, 0.19569, 0.21104, 0.33602, 0.44497, 0.66637, 0.89926, 1.11543, 1.28046, 1.48742};
    p.aero.CL_arr = {0.13177, 0.23946, 0.35383, 0.45618, 0.56180, 0.66207, 0.76109, 0.85925, 0.95104, 1.03631, 1.11760, 1.15681, 1.33557, 1.49766, 1.64227, 1.75166, 1.78420, 1.73537, 1.60000};
    p.aero.Cm_arr = {0.105, 0.1162, 0.1037, 0.0368, 0.0224, 0.0063, -0.0137, -0.0442, -0.0922, -0.1522, -0.2226, -0.2787, -0.3129, -0.3380, -0.3631, -0.3882, -0.4133, -0.4385, -0.4829};
    p.aero.CYp_arr = {-0.099517, -0.073394, -0.047124, -0.020741, 0.005724, 0.032238, 0.058769, 0.085285, 0.111754, 0.138142, 0.164419, 0.190552, 0.216509, 0.242258};
    p.aero.CYr_arr = {0.250408, 0.255041, 0.258759, 0.261558, 0.263434, 0.264386, 0.264412, 0.263512, 0.261687, 0.258939, 0.255273, 0.250691, 0.245200, 0.238807};
    p.aero.Clbeta_arr = {-0.068413, -0.077799, -0.087178, -0.096538, -0.105868, -0.115156, -0.124393, -0.133565, -0.142662, -0.151673, -0.160586, -0.169392, -0.178079, -0.186637};
    p.aero.Clp_arr = {-0.35762, -0.354978, -0.352067, -0.348891, -0.345458, -0.341774, -0.337848, -0.333687, -0.3293, -0.324696, -0.319885, -0.314875, -0.309678, -0.304303};
    p.aero.Clr_arr = {0.116171, 0.130706, 0.145105, 0.159353, 0.17344, 0.187353, 0.201079, 0.214609, 0.22793, 0.241034, 0.253909, 0.266546, 0.278938, 0.291074};
    p.aero.Cnbeta_arr = {0.094369, 0.09441, 0.094779, 0.095474, 0.096496, 0.097843, 0.099513, 0.101505, 0.103816, 0.106442, 0.109381, 0.11263, 0.116184, 0.120039};
    p.aero.Cnp_arr = {0.059446, 0.054013, 0.048466, 0.042799, 0.037006, 0.03108, 0.025015, 0.018808, 0.012452, 0.005943, -0.000722, -0.007548, -0.014536, -0.02169};
    p.aero.Cnr_arr = {-0.103778, -0.106942, -0.110236, -0.113654, -0.117188, -0.120832, -0.124577, -0.128416, -0.13234, -0.136341, -0.140409, -0.144535, -0.148709, -0.15292};
    double sum_a = 0.0;
    double sum_cl = 0.0;
    double sum_aa = 0.0;
    double sum_acl = 0.0;
    int n_fit = 0;
    for (int i = 0; i < 19; ++i) {
        if (std::abs(p.bigalpha_arr[i]) <= 6.0 * p.D2R) {
            const double a = p.bigalpha_arr[i];
            const double cl = p.aero.CL_arr[i];
            sum_a += a;
            sum_cl += cl;
            sum_aa += a * a;
            sum_acl += a * cl;
            ++n_fit;
        }
    }
    const double slope = (n_fit * sum_acl - sum_a * sum_cl) / (n_fit * sum_aa - sum_a * sum_a);
    FlatPlateFit fp;
    fp.c0 = interpArr<19>(p.bigalpha_arr, p.aero.CD_arr, 0.0);
    fp.c2 = std::max(std::abs(slope), 0.1);
    fp.c3 = std::max(0.25 * fp.c2, 0.1);
    const double alpha_hi = p.bigalpha_arr.back();
    const double sin2_hi = std::sin(2.0 * alpha_hi);
    fp.c1 = std::abs(sin2_hi) > 1e-6 ? std::max(std::abs(p.aero.CL_arr.back() / sin2_hi), 0.1) : 0.9;
    fp.alpha0 = 3.0 * p.D2R;
    fp.kL = 38.0;
    fp.kD = 48.0;
    fp.trusted_min = p.bigalpha_arr.front();
    fp.trusted_max = p.bigalpha_arr.back();
    const double blend = 10.0 * p.D2R;
    const double CL_min = p.aero.CL_arr.front();
    const double CD_min = p.aero.CD_arr.front();
    const double CL_max = p.aero.CL_arr.back();
    const double CD_max = p.aero.CD_arr.back();

    for (int i = 0; i < 361; ++i) {
        const double a = (-180.0 + i) * p.D2R;
        p.aero_fullalpha_arr[i] = a;
        if (a >= fp.trusted_min && a <= fp.trusted_max) {
            p.aero.CL_full_arr[i] = interpArr<19>(p.bigalpha_arr, p.aero.CL_arr, a);
            p.aero.CD_full_arr[i] = interpArr<19>(p.bigalpha_arr, p.aero.CD_arr, a);
            continue;
        }

        auto emp = flatPlateCoeff(a, fp);
        double CD_emp = emp.first;
        double CL_emp = emp.second;
        if (a > fp.trusted_max) {
            auto edge = flatPlateCoeff(fp.trusted_max, fp);
            if (std::abs(edge.second) > 1e-9) {
                CL_emp = CL_emp * CL_max / edge.second;
            }
            if (std::abs(edge.first - fp.c0) > 1e-9) {
                CD_emp = fp.c0 + (CD_emp - fp.c0) * (CD_max - fp.c0) / (edge.first - fp.c0);
            }
            const double w = smoothstep(std::min((a - fp.trusted_max) / blend, 1.0));
            p.aero.CL_full_arr[i] = (1.0 - w) * CL_max + w * CL_emp;
            p.aero.CD_full_arr[i] = (1.0 - w) * CD_max + w * CD_emp;
        } else {
            const double w = smoothstep(std::min((fp.trusted_min - a) / blend, 1.0));
            p.aero.CL_full_arr[i] = (1.0 - w) * CL_min + w * CL_emp;
            p.aero.CD_full_arr[i] = (1.0 - w) * CD_min + w * CD_emp;
        }
        p.aero.CD_full_arr[i] = std::max(p.aero.CD_full_arr[i], 0.001);
    }
    p.init_Xe = {0.0, 0.0, -1.8};
    p.init_Vb = {0.1, 0.0, 0.0};
    p.init_Euler = {0.0, 30.0 * p.D2R, 0.0};
    p.init_pqr = {0.0, 0.0, 0.0};
    p.delta_ae_lim = {-30.0 * p.D2R, 30.0 * p.D2R};
    p.wind = {0.0, 0.0, 0.0};
    return p;
}

Param initParamZxFwCtrl() {
    Param p = initParamZx();
    p.fw_gamma_max = 25.0 * p.D2R;
    p.fw_pitch_min = -20.0 * p.D2R;
    p.fw_pitch_max = 35.0 * p.D2R;
    p.fw_theta_trim = 0.37 * p.D2R;
    p.fw_roll_max = 35.0 * p.D2R;
    p.K_theta = {{{8.0, 0.0, 0.0}, {0.0, 7.5, 0.0}, {0.0, 0.0, 0.2}}};
    p.K_omega_p = {{{10.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {0.0, 0.0, 0.2}}};
    p.K_omega_i = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
    p.omega_cmd_max = {2.0, 1.8, 0.3};
    p.att_ff_max = {1.0, 1.0, 0.2};
    p.omega_int_max = {0.3, 0.3, 0.1};
    p.moment_max = {2.2, 2.2, 0.0};
    p.fw_alloc_fd_step = 0.5 * p.D2R;
    p.fw_alloc_du_max = 8.0 * p.D2R;
    p.fw_alloc_weights = {{{2.0, 0.0, 0.0}, {0.0, 3.0, 0.0}, {0.0, 0.0, 1.0}}};
    p.fw_delta_ae_trim = 1.35 * p.D2R;
    return p;
}

StateDerivative tandemZxDynamics(double, const State& x, const Control& u, const Param& p) {
    const Vec3 pe{x[0], x[1], x[2]};
    const Vec3 ve{x[3], x[4], x[5]};
    const Vec4 q = normalizeQuat({x[6], x[7], x[8], x[9]});
    const Vec3 omega{x[10], x[11], x[12]};
    const Mat3 R_eb = quatToRotm(q);
    const Mat3 R_be = transpose(R_eb);
    std::array<double, 8> delta8{};
    std::array<double, 10> delta10{};
    for (int i = 0; i < 8; ++i) {
        delta8[i] = clamp(u[i], 0.0, 1.0);
        delta10[i] = delta8[i];
    }
    delta10[8] = clamp(u[8], 0.0, 1.0);
    delta10[9] = clamp(u[9], 0.0, 1.0);
    const double daeL = clamp(u[10], p.delta_ae_lim[0], p.delta_ae_lim[1]);
    const double daeR = clamp(u[11], p.delta_ae_lim[0], p.delta_ae_lim[1]);
    const ForceMoment rotor = tandemRotorFm(delta10, ve, R_eb, p);
    const ForceMoment aero = tandemAeroFm(ve, R_eb, omega, daeL, daeR, delta8, p);
    const Vec3 fg_b = matVec(R_be, {0.0, 0.0, p.m * p.g});
    Vec3 f_total = add(add(rotor.f, aero.f), fg_b);
    Vec3 m_total = add(rotor.m, aero.m);
    if (p.ground.enable) {
        const Vec3 f_non_ground_e = matVec(R_eb, f_total);
        const Vec3 f_ground_e = zxGroundContactForce(pe, ve, f_non_ground_e, p);
        f_total = add(f_total, matVec(R_be, f_ground_e));
    }
    const Vec3 dv = scale(matVec(R_eb, f_total), 1.0 / p.m);
    const Vec4 dq{
        0.5 * (-omega[0] * q[1] - omega[1] * q[2] - omega[2] * q[3]),
        0.5 * ( omega[0] * q[0] + omega[2] * q[2] - omega[1] * q[3]),
        0.5 * ( omega[1] * q[0] - omega[2] * q[1] + omega[0] * q[3]),
        0.5 * ( omega[2] * q[0] + omega[1] * q[1] - omega[0] * q[2]),
    };
    const Vec3 Jw = matVec(p.J, omega);
    const Vec3 rhs = sub(m_total, cross(omega, Jw));
    const Vec3 domega = solve3(p.J, rhs);
    return {ve[0], ve[1], ve[2], dv[0], dv[1], dv[2], dq[0], dq[1], dq[2], dq[3], domega[0], domega[1], domega[2]};
}

State rk4Step(double t, double dt, const State& x, const Control& u, const Param& p) {
    auto addScaled = [](const State& a, const StateDerivative& k, double s) {
        State y{};
        for (int i = 0; i < kStateSize; ++i) {
            y[i] = a[i] + s * k[i];
        }
        return y;
    };
    const StateDerivative k1 = tandemZxDynamics(t, x, u, p);
    const StateDerivative k2 = tandemZxDynamics(t + 0.5 * dt, addScaled(x, k1, 0.5 * dt), u, p);
    const StateDerivative k3 = tandemZxDynamics(t + 0.5 * dt, addScaled(x, k2, 0.5 * dt), u, p);
    const StateDerivative k4 = tandemZxDynamics(t + dt, addScaled(x, k3, dt), u, p);
    State y{};
    for (int i = 0; i < kStateSize; ++i) {
        y[i] = x[i] + dt * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
    }
    const Vec4 qn = normalizeQuat({y[6], y[7], y[8], y[9]});
    y[6] = qn[0]; y[7] = qn[1]; y[8] = qn[2]; y[9] = qn[3];
    return y;
}

void FixedWingController::reset(const Param& p) {
    for (int i = 0; i < 8; ++i) {
        u_prev_[i] = p.fw_throttle_trim;
    }
    u_prev_[8] = 0.0;
    u_prev_[9] = 0.0;
    u_prev_[10] = p.fw_delta_ae_trim;
    u_prev_[11] = p.fw_delta_ae_trim;
    omega_err_int_ = {0.0, 0.0, 0.0};
    initialized_ = true;
}

Control FixedWingController::step(double t, const State& x, const FwRef& ref, const Param& p, FwDebug* dbg) {
    if (!initialized_ || t <= 0.0) {
        reset(p);
    }
    const Vec3 pe{x[0], x[1], x[2]};
    const Vec3 ve{x[3], x[4], x[5]};
    const Vec4 q = normalizeQuat({x[6], x[7], x[8], x[9]});
    const Vec3 omega{x[10], x[11], x[12]};
    const Mat3 R_be = transpose(quatToRotm(q));
    const Vec3 va_b = matVec(R_be, sub(ve, p.wind));
    const double Va = norm(va_b);
    const double gs_xy = std::hypot(ve[0], ve[1]);
    const double chi = gs_xy > 1e-6 ? std::atan2(ve[1], ve[0]) : yawFromQuatZYX(q);
    const double h = -pe[2];
    const double h_dot = -ve[2];
    double dt_main = p.fw_throttle_trim + p.fw_speed_kp * (ref.Va_d - Va);
    dt_main = clamp(dt_main, p.fw_throttle_min, p.fw_throttle_max);
    if (ref.has_throttle_d) {
        dt_main = clamp(ref.throttle_d, p.fw_throttle_min, p.fw_throttle_max);
    } else if (ref.has_throttle_ff) {
        dt_main = clamp(dt_main + ref.throttle_ff, p.fw_throttle_min, p.fw_throttle_max);
    }
    const double gamma_cmd = clamp(p.fw_alt_kp * (ref.h_d - h) + p.fw_alt_kd * (ref.h_dot_d - h_dot),
                                   -p.fw_gamma_max, p.fw_gamma_max);
    double theta_cmd = p.fw_theta_trim + gamma_cmd;
    if (ref.has_theta_d) {
        theta_cmd = ref.theta_d;
    }
    theta_cmd = clamp(theta_cmd, p.fw_pitch_min, p.fw_pitch_max);
    const double phi_cmd = clamp(p.fw_course_kp * wrapToPi(ref.chi_d - chi), -p.fw_roll_max, p.fw_roll_max);
    const Vec4 q_des = eul2quatZYX(ref.chi_d, theta_cmd, phi_cmd);

    Vec4 q_err = quatMultiply(quatConj(q_des), q);
    if (q_err[0] < 0.0) {
        for (double& v : q_err) {
            v = -v;
        }
    }
    q_err = normalizeQuat(q_err);
    const double theta = wrapToPi(2.0 * std::acos(clamp(q_err[0], -1.0, 1.0)));
    Vec3 theta_vec{0.0, 0.0, 0.0};
    if (std::abs(theta) >= 1e-10) {
        const double s = std::sin(theta / 2.0);
        theta_vec = {theta * q_err[1] / s, theta * q_err[2] / s, theta * q_err[3] / s};
    }
    Vec3 omega_cmd = scale(matVec(p.K_theta, theta_vec), -1.0);
    omega_cmd = satVec(omega_cmd, p.omega_cmd_max);
    const Vec3 omega_err = sub(omega, omega_cmd);
    omega_err_int_ = add(omega_err_int_, scale(omega_err, p.ctrl_dt));
    omega_err_int_ = satVec(omega_err_int_, p.omega_int_max);
    Vec3 m_des = matVec(p.K_omega_p, omega_err);
    m_des = add(m_des, matVec(p.K_omega_i, omega_err_int_));
    m_des = scale(matVec(p.J, m_des), -1.0);
    m_des = satVec(m_des, p.moment_max);

    std::array<double, 8> u_main{};
    u_main.fill(dt_main);
    std::array<double, 2> dae{clamp(u_prev_[10], p.delta_ae_lim[0], p.delta_ae_lim[1]),
                              clamp(u_prev_[11], p.delta_ae_lim[0], p.delta_ae_lim[1])};
    const Vec3 m0 = totalRollPitchMoment(dae, u_main, x, p);
    double B[2][3]{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    for (int i = 0; i < 2; ++i) {
        std::array<double, 2> dp = dae;
        dp[i] = std::min(dp[i] + p.fw_alloc_fd_step, p.delta_ae_lim[1]);
        double denom = dp[i] - dae[i];
        if (denom > 1e-12) {
            const Vec3 mt = totalRollPitchMoment(dp, u_main, x, p);
            B[0][i] = (mt[0] - m0[0]) / denom;
            B[1][i] = (mt[1] - m0[1]) / denom;
        }
    }
    int n_alloc = 2;
    if (p.fw_pitch_throttle_enable) {
        std::array<double, 8> u_pitch = u_main;
        applyPitchThrottleDelta(u_pitch, p.fw_pitch_throttle_fd_step, p);
        const double denom = signedPitchDelta(u_pitch, u_main, p);
        if (std::abs(denom) > 1e-12) {
            const Vec3 mt = totalRollPitchMoment(dae, u_pitch, x, p);
            B[0][2] = (mt[0] - m0[0]) / denom;
            B[1][2] = (mt[1] - m0[1]) / denom;
        }
        n_alloc = 3;
    }
    const double err0 = m_des[0] - m0[0];
    const double err1 = m_des[1] - m0[1];
    Mat3 H{{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
    Vec3 rhs{0.0, 0.0, 0.0};
    for (int i = 0; i < n_alloc; ++i) {
        for (int j = 0; j < n_alloc; ++j) {
            H[i][j] = 2.0 * 2.0 * B[0][i] * B[0][j] + 3.0 * 3.0 * B[1][i] * B[1][j];
        }
        H[i][i] += p.fw_alloc_lambda;
        rhs[i] = 2.0 * 2.0 * B[0][i] * err0 + 3.0 * 3.0 * B[1][i] * err1;
    }
    const Vec3 delta = solve3(H, rhs);
    dae[0] = clamp(dae[0] + clamp(delta[0], -p.fw_alloc_du_max, p.fw_alloc_du_max), p.delta_ae_lim[0], p.delta_ae_lim[1]);
    dae[1] = clamp(dae[1] + clamp(delta[1], -p.fw_alloc_du_max, p.fw_alloc_du_max), p.delta_ae_lim[0], p.delta_ae_lim[1]);
    double pitch_delta = 0.0;
    if (p.fw_pitch_throttle_enable) {
        pitch_delta = clamp(delta[2], -p.fw_pitch_throttle_du_max, p.fw_pitch_throttle_du_max);
        applyPitchThrottleDelta(u_main, pitch_delta, p);
    }
    Control u{};
    for (int i = 0; i < 8; ++i) {
        u[i] = u_main[i];
    }
    u[8] = 0.0;
    u[9] = 0.0;
    u[10] = dae[0];
    u[11] = dae[1];
    u_prev_ = u;
    if (dbg) {
        dbg->t = t;
        dbg->Va = Va;
        dbg->h = h;
        dbg->h_dot = h_dot;
        dbg->chi = chi;
        dbg->dt_main = dt_main;
        dbg->phi_cmd = phi_cmd;
        dbg->theta_cmd = theta_cmd;
        dbg->q_des = q_des;
        dbg->m_des = m_des;
        dbg->u = u;
    }
    return u;
}

State defaultFwSmokeInitialState(const Param& p) {
    State x{};
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = -30.0;
    x[3] = p.fw_airspeed_trim;
    x[4] = 0.0;
    x[5] = 0.0;
    x[6] = 1.0;
    x[7] = 0.0;
    x[8] = 0.0;
    x[9] = 0.0;
    x[10] = 0.0;
    x[11] = 0.0;
    x[12] = 0.0;
    return x;
}

std::string version() {
    return "GJ2 C++ model/controller v1";
}

}  // namespace gj2
