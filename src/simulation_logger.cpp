#include "simulation_logger.h"

#include "components/inverter_model.h"
#include "components/motor_model.h"
#include "components/pump_model.h"
#include "components/shaft_model.h"
#include "components/turbine_model.h"

#include <iomanip>

namespace {
class CsvRowWriter {
public:
    explicit CsvRowWriter(std::ofstream& stream) : ofs(stream) {}

    template <typename T>
    void add(const T& value) {
        if (!first) {
            ofs << ',';
        }
        ofs << value;
        first = false;
    }

    void finish() {
        ofs << '\n';
    }

private:
    std::ofstream& ofs;
    bool first = true;
};
}  // namespace

SimulationLogger::SimulationLogger(const std::string& path) : ofs(path) {
    ofs << std::fixed << std::setprecision(6);
}

bool SimulationLogger::is_open() const {
    return ofs.is_open();
}

void SimulationLogger::write_header() {
    ofs << "t,ud_ref,uq_ref,ua,ub,uc,v_dc,duty_a,duty_b,duty_c,gate_u_a,gate_l_a,gate_u_b,gate_l_b,gate_u_c,gate_l_c,i_frontend,i_batt,omega_mech,theta_mech,theta_e,ia,ib,ic,id,iq,T_em,T_turb,T_pump,p_plenum,p_downstream_turb,m_dot_in,m_dot_turb,m_dot_out,p_discharge,Q_pump,Q_out" << '\n';
}

void SimulationLogger::log_snapshot(double t,
                                    double ud_ref,
                                    double uq_ref,
                                    const MotorModel& motor,
                                    const TurbineModel& turb,
                                    const PumpModel& pump,
                                    const ShaftModel& shaft,
                                    const InverterModel& inverter) {
    CsvRowWriter row(ofs);
    row.add(t);
    row.add(ud_ref);
    row.add(uq_ref);
    row.add(motor.ua);
    row.add(motor.ub);
    row.add(motor.uc);
    row.add(inverter.v_dc);
    row.add(inverter.duty_a);
    row.add(inverter.duty_b);
    row.add(inverter.duty_c);
    row.add(inverter.gate_u_a);
    row.add(inverter.gate_l_a);
    row.add(inverter.gate_u_b);
    row.add(inverter.gate_l_b);
    row.add(inverter.gate_u_c);
    row.add(inverter.gate_l_c);
    row.add(inverter.i_frontend);
    row.add(inverter.i_batt);
    row.add(shaft.omega_mech);
    row.add(shaft.theta_mech);
    row.add(motor.theta_e);
    row.add(motor.ia);
    row.add(motor.ib);
    row.add(motor.ic);
    row.add(motor.id);
    row.add(motor.iq);
    row.add(motor.T_em);
    row.add(turb.T_turb);
    row.add(pump.T_pump);
    row.add(turb.p_plenum);
    row.add(turb.p_downstream);
    row.add(turb.m_dot_in);
    row.add(turb.m_dot_turb);
    row.add(turb.m_dot_out);
    row.add(pump.p_discharge);
    row.add(pump.Q_pump);
    row.add(pump.Q_out);
    row.finish();
}
