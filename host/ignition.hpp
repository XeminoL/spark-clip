#pragma once

#include <memory>
#include <string>

#include "catalog.hpp"

class Ignition {
public:
    explicit Ignition(const IgnitionSpec& spec) : spec_(spec) {}
    virtual ~Ignition() = default;

    virtual std::string name() const = 0;
    virtual double open_circuit_kv(double rpm, double battery_volts) const = 0;
    virtual bool clippable() const { return true; }
    virtual std::string sag_reason() const { return "low battery voltage starves the coil"; }
    virtual bool needs_fast_sampling() const { return spec_.rise_us < 10.0; }

    double rise_seconds() const { return spec_.rise_us * 1e-6; }
    double rise_shape(double fraction) const;
    double rise_fraction(double share) const;

protected:
    IgnitionSpec spec_;
};

class AcCdi : public Ignition {
public:
    using Ignition::Ignition;
    std::string name() const override { return "AC-CDI"; }
    std::string sag_reason() const override { return "slow cranking starves the CDI"; }
    double open_circuit_kv(double rpm, double battery_volts) const override;
};

class DcCdi : public Ignition {
public:
    using Ignition::Ignition;
    std::string name() const override { return "DC-CDI"; }
    double open_circuit_kv(double rpm, double battery_volts) const override;
};

class Tci : public Ignition {
public:
    using Ignition::Ignition;
    std::string name() const override { return "TCI"; }
    double open_circuit_kv(double rpm, double battery_volts) const override;
};

class CoilOnPlug : public Tci {
public:
    using Tci::Tci;
    std::string name() const override { return "coil on plug"; }
    bool clippable() const override { return false; }
};

std::unique_ptr<Ignition> make_ignition(const IgnitionSpec& spec);