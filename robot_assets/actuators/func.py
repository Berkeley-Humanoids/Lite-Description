"""
Functions to compute actuator parameters.

These equations are derived from:
BeyondMimic: From Motion Tracking to Versatile Humanoid Control via Guided Diffusion, https://arxiv.org/abs/2508.08241
"""

from math import pi


def compute_stiffness(actuator_params: dict[str, float], natural_frequency: float = 10 * 2 * pi) -> float:
    """Compute the stiffness of an actuator.

    Args:
        actuator_params: The parameters of the actuator.
        natural_frequency: The natural frequency of the actuator in rad/s.
    """
    return actuator_params["armature"] * natural_frequency**2


def compute_damping(actuator_params: dict[str, float], natural_frequency: float = 10 * 2 * pi, damping_ratio: float = 2.0) -> float:
    """Compute the damping of an actuator.

    Args:
        actuator_params: The parameters of the actuator.
        natural_frequency: The natural frequency of the actuator in rad/s.
        damping_ratio: The damping ratio of the actuator.
    """
    return 2.0 * damping_ratio * actuator_params["armature"] * natural_frequency


def compute_action_scale(actuator_params: dict[str, float], natural_frequency: float = 10 * 2 * pi, action_scale_coefficient: float = 0.25) -> float:
    """Compute the action scale from natural frequency and action scale coefficient.

    Args:
        actuator_params: The parameters of the actuator.
        natural_frequency: The natural frequency of the actuator in rad/s.
        action_scale_coefficient: The action scale coefficient.
    """
    stiffness = compute_stiffness(actuator_params, natural_frequency)
    return action_scale_coefficient * actuator_params["effort_limit"] / stiffness
