import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from physics import clip, geometry, ignition
from physics.catalog import load_bikes
from physics.cranking import Battery, CrankingModel, Cylinder, simulate
from physics.inertia import rotor_by_integral, rotor_closed_form
from physics.paschen import (density_from_kv, fit_air, load_air, plug_kv, three_parameter_singular_values,
                             uniform_kv)


@pytest.fixture(scope="module")
def bikes():
    return load_bikes()


@pytest.fixture(scope="module")
def air():
    return load_air()


def test_rotor_triple_integral_matches_closed_form(bikes):
    for bike in bikes.values():
        assert rotor_by_integral(bike.rotating) == pytest.approx(rotor_closed_form(bike.rotating), rel=1e-9)


def test_volume_rate_is_derivative_of_volume(bikes):
    engine = bikes["cub110"].engine
    for theta in np.linspace(0.1, 6.2, 12):
        step = 1e-6
        slope = (geometry.volume(engine, theta + step) - geometry.volume(engine, theta - step)) / (2 * step)
        assert geometry.volume_rate(engine, theta) == pytest.approx(slope, rel=1e-6, abs=1e-12)


def test_swept_volume(bikes):
    engine = bikes["cub110"].engine
    swept = geometry.volume(engine, np.pi) - geometry.volume(engine, 0.0)
    assert swept * 1e6 == pytest.approx(109.17, abs=0.01)


def test_air_fit_and_rank(air):
    fitted, residuals = fit_air()
    assert np.max(np.abs(residuals)) < 0.035
    assert fitted.ionisation_over_k == pytest.approx(air.ionisation_over_k, rel=1e-6)
    singular = three_parameter_singular_values()
    assert singular[2] / singular[0] < 1e-6


def test_flat_tip_gives_paschen(air):
    assert plug_kv(air, 0.97, 0.7e-3, tip_radius=1e3) == pytest.approx(uniform_kv(air, 0.97 * 0.07), rel=1e-4)


def test_density_inversion(air):
    for density in (1.0, 4.0, 7.5):
        assert density_from_kv(air, plug_kv(air, density, 0.8e-3), 0.8e-3) == pytest.approx(density, rel=1e-6)


def test_ratio_barely_depends_on_gap(air):
    ratios = [plug_kv(air, 7.0, gap) / plug_kv(air, 0.97, gap) for gap in (0.6e-3, 1.1e-3)]
    assert abs(ratios[1] / ratios[0] - 1) < 0.02


def test_sealed_adiabatic_gas_keeps_entropy(bikes):
    model = CrankingModel(bikes["cub110"], Battery(12.8, 0.015), Cylinder(0.0, 0.0))
    engine = bikes["cub110"].engine
    for theta in np.radians([600.0, 650.0, 700.0, 730.0]):
        speed, mass, temperature = 40.0, 1.1e-4, 350.0
        _, _, mass_rate, temperature_rate = model.rates(theta, speed, mass, temperature, True, None)
        volume_log_rate = geometry.volume_rate(engine, theta) * speed / geometry.volume(engine, theta)
        assert mass_rate == 0.0
        assert temperature_rate / temperature == pytest.approx(-0.4 * volume_log_rate, rel=1e-9)


def test_leak_lowers_trapped_share(bikes):
    shares = []
    for area in (0.02e-6, 0.3e-6, 0.9e-6):
        model = CrankingModel(bikes["cub110"], Battery(12.8, 0.015), Cylinder(area, 60.0))
        record = simulate(model, np.radians(100), np.random.default_rng(0), jitter_deg=0.0, duration=1.5)
        shares.append(np.mean(record.trapped))
    assert shares[0] > shares[1] > shares[2]


def test_weak_battery_cranks_slower(bikes):
    periods = []
    for battery in (Battery(12.8, 0.015), Battery(12.3, 0.04)):
        record = simulate(CrankingModel(bikes["cub110"], battery, Cylinder(0.1e-6, 60.0)), np.radians(100), np.random.default_rng(0), jitter_deg=0.0)
        periods.append(np.mean(np.diff([s.time for s in record.sparks])[-4:]))
    assert periods[1] > periods[0]


def test_misfire_when_breakdown_exceeds_coil(bikes):
    coil = bikes["cub110"].ignition
    times = np.arange(0, 20e-6, 2e-9)
    _, fired = ignition.spark_waveform(coil, 15.0, 18.0, times, np.random.default_rng(0))
    assert not fired
    _, fired = ignition.spark_waveform(coil, 22.0, 16.0, times, np.random.default_rng(0))
    assert fired


def test_clip_coupling_falls_with_air_gap():
    assert clip.coupling_capacitance(0.05e-3) > clip.coupling_capacitance(0.4e-3)
    assert 1e-12 < clip.coupling_capacitance(0.2e-3) < 3e-12