"""URDF↔MJCF parity checks.

Pairs are matched by stem within a single robot directory, so converted
companion files (e.g. ``lite.urdf`` ↔ ``lite.xml``) are compared and unrelated
MJCF variants (e.g. a ``*_frames.xml`` with no same-stem URDF) are skipped.
"""
import pytest

from .conftest import (
    TOLERANCE,
    joint_name_set,
    parity_params,
    total_mass,
)

PARITY_PARAMS, PARITY_IDS = parity_params()


@pytest.mark.skipif(not PARITY_PARAMS, reason="no URDF/MJCF pair to compare")
@pytest.mark.parametrize(("urdf_model", "mjcf_model"), PARITY_PARAMS, ids=PARITY_IDS)
def test_total_mass_parity(urdf_model, mjcf_model):
    u_mass = total_mass(urdf_model)
    m_mass = total_mass(mjcf_model)
    diff = abs(u_mass - m_mass)
    assert diff <= TOLERANCE, (
        f"[{urdf_model.label} <-> {mjcf_model.label}] total mass mismatch: "
        f"urdf={u_mass} mjcf={m_mass} (|Δ|={diff})"
    )


@pytest.mark.skipif(not PARITY_PARAMS, reason="no URDF/MJCF pair to compare")
@pytest.mark.parametrize(("urdf_model", "mjcf_model"), PARITY_PARAMS, ids=PARITY_IDS)
def test_joint_set_parity(urdf_model, mjcf_model):
    u_joints = joint_name_set(urdf_model)
    m_joints = joint_name_set(mjcf_model)
    only_urdf = u_joints - m_joints
    only_mjcf = m_joints - u_joints
    assert not only_urdf and not only_mjcf, (
        f"[{urdf_model.label} <-> {mjcf_model.label}] joint set mismatch: "
        f"only_in_urdf={sorted(only_urdf)} only_in_mjcf={sorted(only_mjcf)}"
    )
