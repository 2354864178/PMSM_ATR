#include "simulation_logger.h"

#include "components/inverter_model.h"
#include "components/motor_model.h"
#include "components/pump_model.h"
#include "components/shaft_model.h"
#include "components/turbine_model.h"

#include <iomanip>

SimulationLogger::SimulationLogger(const std::string& path) : ofs(path) {
    ofs << std::fixed << std::setprecision(6);
}

bool SimulationLogger::is_open() const {
    return ofs.is_open();
}

void SimulationLogger::write_header() {
    ofs << "t,ud_ref,uq_ref,ua,ub,uc,v_dc,duty_a,duty_b,duty_c,gate_u_a,gate_l_a,gate_u_b,gate_l_b,gate_u_c,gate_l_c,i_frontend,i_batt,omega_mech,theta_mech,theta_e,ia,ib,ic,id,iq,T_em,T_turb,T_pump,p_plenum,m_dot_in,m_dot_turb,p_discharge,Q_pump,Q_out" << '\n';
}

void SimulationLogger::log_snapshot(double t,
                                    double ud_ref,
                                    double uq_ref,
                                    const MotorModel& motor,
                                    const TurbineModel& turb,
                                    const PumpModel& pump,
                                    const ShaftModel& shaft,
                                    const InverterModel& inverter) {
    ofs << t << ','
        << ud_ref << ',' << uq_ref << ','
        << motor.ua << ',' << motor.ub << ',' << motor.uc << ','
        << inverter.v_dc << ',' << inverter.duty_a << ',' << inverter.duty_b << ',' << inverter.duty_c << ','
        << inverter.gate_u_a << ',' << inverter.gate_l_a << ','
        << inverter.gate_u_b << ',' << inverter.gate_l_b << ','
        << inverter.gate_u_c << ',' << inverter.gate_l_c << ','
        << inverter.i_frontend << ',' << inverter.i_batt << ','
        << shaft.omega_mech << ',' << shaft.theta_mech << ','
        << motor.theta_e << ','
        << motor.ia << ',' << motor.ib << ',' << motor.ic << ','
        << motor.id << ',' << motor.iq << ','
        << motor.T_em << ',' << turb.T_turb << ',' << pump.T_pump << ','
        << turb.p_plenum << ',' << turb.m_dot_in << ',' << turb.m_dot_turb << ','
        << pump.p_discharge << ',' << pump.Q_pump << ',' << pump.Q_out
        << '\n';
}
