#pragma once

#include <array>
#include <string>

namespace gj2 {

constexpr int kStateSize = 13;
constexpr int kControlSize = 12;

using State = std::array<double, kStateSize>;
using StateDerivative = std::array<double, kStateSize>;
using Control = std::array<double, kControlSize>;
using Vec3 = std::array<double, 3>;
using Vec4 = std::array<double, 4>;
using Mat3 = std::array<std::array<double, 3>, 3>;

struct AeroParam {
    double CD0{};
    double CDdelta_aeL{};
    double CDdelta_aeR{};
    double CYbeta{};
    double CLdelta_aeL{};
    double CLdelta_aeR{};
    double CLq{};
    double CLalpdot{};
    double Cldelta_aeL{};
    double Cldelta_aeR{};
    double Cmdelta_aeL{};
    double Cmdelta_aeR{};
    double Cmq{};
    double Cmalpdot{};
    double Cndelta_aeL{};
    double Cndelta_aeR{};
    std::array<double, 19> CD_arr{};
    std::array<double, 19> CL_arr{};
    std::array<double, 19> Cm_arr{};
    std::array<double, 14> CYp_arr{};
    std::array<double, 14> CYr_arr{};
    std::array<double, 14> Clbeta_arr{};
    std::array<double, 14> Clp_arr{};
    std::array<double, 14> Clr_arr{};
    std::array<double, 14> Cnbeta_arr{};
    std::array<double, 14> Cnp_arr{};
    std::array<double, 14> Cnr_arr{};
    std::array<double, 361> CD_full_arr{};
    std::array<double, 361> CL_full_arr{};
};

struct GroundParam {
    bool enable{false};
    double z{0.0};
    double tol{1e-4};
    double k{700.0};
    double c{70.0};
    double mu{0.55};
    double xy_damping{35.0};
};

struct ForceMoment {
    Vec3 f{};
    Vec3 m{};
};

struct RotorThrust {
    double thrust{};
    double m_prop{};
    double n_rps{};
    double ct{};
};

struct Param {
    double g{9.8};
    double rho{1.225};
    double D2R{3.14159265358979323846 / 180.0};
    double R2D{180.0 / 3.14159265358979323846};
    double m{3.2};
    Mat3 J{};
    double S{0.732};
    double b{1.2};
    double MAC{0.3};
    std::array<double, 8> S_slip{};
    std::array<double, 8> S_slip_y{};
    double S_free{};
    double alpha_slip_switch{1.0};
    double DP_slip_switch{1.0};
    double alpha_dot_hat{0.0};
    std::array<Vec3, 8> prop_pos{};
    double prop_D{0.2032};
    std::array<double, 8> prop_angle{};
    std::array<double, 8> prop_spin{};
    double addprop_x{0.3};
    double addprop_y{0.65};
    double addprop_D{0.127};
    AeroParam aero{};
    std::array<double, 19> bigalpha_arr{};
    std::array<double, 14> alpha_arr{};
    bool aero_extend_flat_plate{true};
    std::array<double, 361> aero_fullalpha_arr{};
    Vec3 init_Xe{};
    Vec3 init_Vb{};
    Vec3 init_Euler{};
    Vec3 init_pqr{};
    double thrust_min{0.0};
    double thrust_max{25.0};
    double rotor_throttle_min{0.0};
    std::array<double, 2> delta_ae_lim{};
    Vec3 wind{};
    GroundParam ground{};
    double ctrl_dt{0.01};

    double fw_airspeed_trim{12.0};
    double fw_throttle_trim{0.438};
    double fw_throttle_min{0.10};
    double fw_throttle_max{1.00};
    double fw_speed_kp{0.055};
    double fw_alt_kp{0.08};
    double fw_alt_kd{0.04};
    double fw_gamma_max{};
    double fw_pitch_min{};
    double fw_pitch_max{};
    double fw_theta_trim{};
    double fw_course_kp{1.2};
    double fw_roll_max{};
    Mat3 K_theta{};
    Mat3 K_omega_p{};
    Mat3 K_omega_i{};
    Vec3 omega_cmd_max{};
    Vec3 att_ff_max{};
    Vec3 omega_int_max{};
    Vec3 moment_max{};
    double fw_alloc_lambda{1e-5};
    double fw_alloc_fd_step{};
    double fw_alloc_du_max{};
    Mat3 fw_alloc_weights{};
    Vec3 fw_addprop_trim{};
    double fw_delta_ae_trim{};
    bool fw_pitch_throttle_enable{true};
    std::array<int, 4> fw_pitch_throttle_front_idx{{0, 1, 2, 3}};
    std::array<int, 4> fw_pitch_throttle_rear_idx{{4, 5, 6, 7}};
    double fw_pitch_throttle_fd_step{0.02};
    double fw_pitch_throttle_du_max{0.18};
};

struct FwRef {
    double Va_d{12.0};
    double h_d{30.0};
    double h_dot_d{0.0};
    double chi_d{0.0};
    bool has_theta_d{false};
    double theta_d{0.0};
    bool has_throttle_d{false};
    double throttle_d{0.0};
    bool has_throttle_ff{false};
    double throttle_ff{0.0};
};

struct FwDebug {
    double t{};
    double Va{};
    double h{};
    double h_dot{};
    double chi{};
    double dt_main{};
    double phi_cmd{};
    double theta_cmd{};
    Vec4 q_des{};
    Vec3 m_des{};
    Control u{};
};

Param initParamZx();
Param initParamZxFwCtrl();

double sat(double x, double x_min, double x_max);
Vec3 sat(const Vec3& x, double x_min, double x_max);

RotorThrust tandemRotorThrust(double delta_t, double alpha, double beta, double tas,
                              double prop_spin, const Param& p, double prop_angle);
ForceMoment tandemAddpropFm(double dt_left, double dt_right, const Param& p);
ForceMoment tandemRotorFm(const std::array<double, 10>& delta_t, const Vec3& ve,
                          const Mat3& R_eb, const Param& p);
ForceMoment tandemAeroFm(const Vec3& ve, const Mat3& R_eb, const Vec3& omega_b,
                         double delta_aeL, double delta_aeR,
                         const std::array<double, 8>& delta_t,
                         const Param& p, int mode_out = -1);
Vec3 zxGroundContactForce(const Vec3& p_e, const Vec3& v_e,
                          const Vec3& f_non_ground_e, const Param& p);

StateDerivative tandemZxDynamics(double t, const State& x, const Control& u, const Param& p);
State rk4Step(double t, double dt, const State& x, const Control& u, const Param& p);

class FixedWingController {
public:
    FixedWingController() = default;
    void reset(const Param& p);
    Control step(double t, const State& x, const FwRef& ref, const Param& p, FwDebug* dbg = nullptr);

private:
    bool initialized_{false};
    Control u_prev_{};
    Vec3 omega_err_int_{{0.0, 0.0, 0.0}};
};

State defaultFwSmokeInitialState(const Param& p);
std::string version();

}  // namespace gj2
