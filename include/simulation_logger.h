#pragma once

#include <fstream>
#include <string>

class MotorModel;
class TurbineModel;
class PumpModel;
class ShaftModel;
class InverterModel;

class SimulationLogger {
public:
    explicit SimulationLogger(const std::string& path);

    bool is_open() const;
    void write_header();

    void log_snapshot(double t,
                      double ud_ref,
                      double uq_ref,
                      const MotorModel& motor,
                      const TurbineModel& turb,
                      const PumpModel& pump,
                      const ShaftModel& shaft,
                      const InverterModel& inverter);

private:
    std::ofstream ofs;
};
