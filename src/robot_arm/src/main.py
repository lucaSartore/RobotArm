#!/usr/bin/env python3
import argparse

from data_processing.full_simulation import run_full_simulation
from data_processing.inverse_kinematic import (
    test_inverse_kinematics_with_postural,
    test_inverse_kinematics,
)
from data_processing.dynamics import (
    test_dynamics,
    test_dynamics_with_initial_velocity,
)


def main():
    parser = argparse.ArgumentParser(
        description="Run different robotics simulation tests."
    )
    parser.add_argument(
        "mode",
        choices=[
            "run_full_simulation",
            "test_inverse_kinematics_with_postural",
            "test_inverse_kinematics",
            "test_dynamics",
            "test_dynamics_with_initial_velocity",
        ],
        nargs="?",
        default="run_full_simulation",
        help="Which function to run (default: run_full_simulation)",
    )

    args = parser.parse_args()

    if args.mode == "run_full_simulation":
        run_full_simulation()
    elif args.mode == "test_inverse_kinematics_with_postural":
        test_inverse_kinematics_with_postural()
    elif args.mode == "test_inverse_kinematics":
        test_inverse_kinematics()
    elif args.mode == "test_dynamics":
        test_dynamics()
    elif args.mode == "test_dynamics_with_initial_velocity":
        test_dynamics_with_initial_velocity()


if __name__ == "__main__":
    main()
