#include "aircraft.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::string out_csv = "cpp_closedloop.csv";
    if (argc >= 2) {
        out_csv = argv[1];
    }

    const gj2::Param param = gj2::initParamZxFwCtrl();
    gj2::FwRef ref;
    ref.Va_d = param.fw_airspeed_trim;
    ref.h_d = 30.0;
    ref.h_dot_d = 0.0;
    ref.chi_d = 0.0;

    gj2::FixedWingController controller;
    gj2::State x = gj2::defaultFwSmokeInitialState(param);
    gj2::Control u{};
    const double dt = param.ctrl_dt;
    const int n = static_cast<int>(2.0 / dt) + 1;

    std::ofstream out(out_csv);
    if (!out) {
        std::cerr << "Cannot open output CSV: " << out_csv << "\n";
        return 2;
    }

    out << "t";
    for (int i = 0; i < gj2::kStateSize; ++i) {
        out << ",x" << (i + 1);
    }
    for (int i = 0; i < gj2::kControlSize; ++i) {
        out << ",u" << (i + 1);
    }
    out << "\n";
    out << std::setprecision(17);

    for (int k = 0; k < n; ++k) {
        const double t = k * dt;
        if (k < n - 1) {
            u = controller.step(t, x, ref, param);
        }
        out << t;
        for (double v : x) {
            out << "," << v;
        }
        for (double v : u) {
            out << "," << v;
        }
        out << "\n";
        if (k < n - 1) {
            x = gj2::rk4Step(t, dt, x, u, param);
        }
    }

    std::cout << "Saved " << out_csv << "\n";
    return 0;
}
