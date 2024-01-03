import json
import math

config = {
    "grid": {
        "x": 1.75,
        "ys": [5.01, 4.44, 3.88, 3.33, 2.76, 2.20, 1.63, 1.07, 0.51],
        "placement_x": 2,
    },
    "piece": {"x": 6.63, "ys": [4.60, 3.375, 2.15, 0.925], "placement_x": 6.38},
    "charge_station": {"x": 3.89, "y": 2.76, "dx": 1.35},
}


def grid(idx: int, end: bool, ccw: bool):
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": True,
        "control_interval_count": 100 if end else 0,
        "heading": math.pi if ccw else -math.pi,
        "heading_constrained": True,
        "name": f"Grid {idx}",
        "velocity_magnitude_constrained": True,
        "velocity_x": 0.0,
        "velocity_x_constrained": True,
        "velocity_y": 0.0,
        "velocity_y_constrained": True,
        "waypoint_type": "custom",
        "x": config["grid"]["x"],
        "x_constrained": True,
        "y": config["grid"]["ys"][idx - 1],
        "y_constrained": True,
    }


def pre_grid(idx: int, ccw: bool):
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": True,
        "control_interval_count": 100,
        "heading": math.pi if ccw else -math.pi,
        "heading_constrained": True,
        "name": f"Pre Grid {idx}",
        "velocity_magnitude_constrained": False,
        "velocity_x": 0.0,
        "velocity_x_constrained": False,
        "velocity_y": 0.0,
        "velocity_y_constrained": True,
        "waypoint_type": "custom",
        "x": config["grid"]["placement_x"],
        "x_constrained": True,
        "y": config["grid"]["ys"][idx - 1],
        "y_constrained": False,
    }


def off_grid(idx: int, ccw: bool):
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": True,
        "control_interval_count": 100,
        "heading": math.pi if ccw else -math.pi,
        "heading_constrained": True,
        "name": f"Pre Grid {idx}",
        "velocity_magnitude_constrained": False,
        "velocity_x": 0.0,
        "velocity_x_constrained": False,
        "velocity_y": 0.0,
        "velocity_y_constrained": True,
        "waypoint_type": "custom",
        "x": config["grid"]["placement_x"],
        "x_constrained": True,
        "y": config["grid"]["ys"][idx - 1],
        "y_constrained": True,
    }


def pre_piece(idx: int):
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": True,
        "control_interval_count": 100,
        "heading": 0.0,
        "heading_constrained": True,
        "name": f"Pre Piece {idx}",
        "velocity_magnitude_constrained": False,
        "velocity_x": 0.0,
        "velocity_x_constrained": False,
        "velocity_y": 0.0,
        "velocity_y_constrained": True,
        "waypoint_type": "custom",
        "x": config["piece"]["placement_x"],
        "x_constrained": True,
        "y": config["piece"]["ys"][idx - 1],
        "y_constrained": True,
    }


def piece(idx: int, end: bool):
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": True,
        "control_interval_count": 100 if end else 0,
        "heading": 0.0,
        "heading_constrained": True,
        "name": f"Piece {idx}",
        "velocity_magnitude_constrained": True,
        "velocity_x": 0.0,
        "velocity_x_constrained": True,
        "velocity_y": 0.0,
        "velocity_y_constrained": True,
        "waypoint_type": "custom",
        "x": config["piece"]["x"],
        "x_constrained": True,
        "y": config["piece"]["ys"][idx - 1],
        "y_constrained": True,
    }


def charge_station_obstacle():
    return {
        "center_x": 3.951757860048448,
        "center_y": 2.75145734088595,
        "length": 2.0,
        "name": "charge_station",
        "obstacle_type": "rectangle",
        "rotate_angle": 0.0,
        "safety_distance": 0.1,
        "width": 2.5,
    }


def grid_obstacle():
    return {
        "center_x": 0.589353,
        "center_y": 2.766604,
        "length": 1.5,
        "name": "grid",
        "obstacle_type": "rectangle",
        "rotate_angle": 0.0,
        "safety_distance": 0.0,
        "width": 5.5,
    }


def wall_obstacle():
    return {
        "center_x": 1.8493134065281556,
        "center_y": 6.528782918506754,
        "length": 3.0,
        "name": "divider",
        "obstacle_type": "rectangle",
        "rotate_angle": 0.0,
        "safety_distance": 0.0,
        "width": 2.0,
    }


def pre_charge_station(heading: float, from_grid: bool, y: float):
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": True,
        "control_interval_count": 100,
        "heading": heading,
        "heading_constrained": True,
        "name": "Inner Charging Station" if from_grid else "Outer Charging Station",
        "velocity_magnitude_constrained": True,
        "velocity_x": 0.0,
        "velocity_x_constrained": False,
        "velocity_y": 0.0,
        "velocity_y_constrained": True,
        "waypoint_type": "custom",
        "x": config["charge_station"]["x"]
        + config["charge_station"]["dx"] * (-1 if from_grid else 1),
        "x_constrained": True,
        "y": y,
        "y_constrained": True,
    }


def charge_station(heading: float, y: float):
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": True,
        "control_interval_count": 100,
        "heading": heading,
        "heading_constrained": True,
        "name": f"Charging Station",
        "velocity_magnitude_constrained": True,
        "velocity_x": 0.0,
        "velocity_x_constrained": True,
        "velocity_y": 0.0,
        "velocity_y_constrained": True,
        "waypoint_type": "custom",
        "x": config["charge_station"]["x"],
        "x_constrained": True,
        "y": y,
        "y_constrained": True,
    }


def upper_guess():
    return {
        "heading": -2.28962632641652,
        "name": "Upper Guess Waypoint",
        "waypoint_type": "initial_guess",
        "x": 2.2396938419076555,
        "y": 4.751895191708549,
    }


def upper_constraint():
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": False,
        "control_interval_count": 100,
        "heading": -2.790561766847373,
        "heading_constrained": True,
        "name": "Upper Constraint",
        "velocity_magnitude_constrained": False,
        "velocity_x": 0.0,
        "velocity_x_constrained": False,
        "velocity_y": 0.0,
        "velocity_y_constrained": False,
        "waypoint_type": "custom",
        "x": 5.246891564424193,
        "x_constrained": True,
        "y": 4.822102142428779,
        "y_constrained": False,
    }


def lower_guess():
    return {
        "heading": 2.28962632641652,
        "name": "Lower Guess Waypoint",
        "waypoint_type": "initial_guess",
        "x": 2.169486891187425,
        "y": 0.7735013175621568,
    }


def lower_constraint():
    return {
        "angular_velocity": 0.0,
        "angular_velocity_constrained": False,
        "control_interval_count": 100,
        "heading": 2.790561766847373,
        "heading_constrained": True,
        "name": "Upper Constraint",
        "velocity_magnitude_constrained": False,
        "velocity_x": 0.0,
        "velocity_x_constrained": False,
        "velocity_y": 0.0,
        "velocity_y_constrained": False,
        "waypoint_type": "custom",
        "x": 5.246891564424193,
        "x_constrained": True,
        "y": 0.7032943668419263,
        "y_constrained": False,
    }


def grid_to_piece(
    grid_idx: int, piece_idx: int, around_charge_station: bool, pos: bool
):
    if around_charge_station:
        return (
            f"Grid {grid_idx} to Piece {piece_idx} (around CS)",
            {
                "obstacles": [charge_station_obstacle(), wall_obstacle()],
                "waypoints": [
                    grid(grid_idx, False, pos),
                    lower_guess(),
                    lower_constraint(),
                    pre_piece(piece_idx),
                    piece(piece_idx, True),
                ]
                if piece_idx > 2
                else [
                    grid(grid_idx, False, pos),
                    upper_guess(),
                    upper_constraint(),
                    pre_piece(piece_idx),
                    piece(piece_idx, True),
                ],
            },
        )
    else:
        return (
            f"Grid {grid_idx} to Piece {piece_idx} (over CS)",
            {
                "obstacles": [],
                "waypoints": [
                    grid(grid_idx, False, pos),
                    pre_charge_station(
                        math.pi if pos else -math.pi,
                        False,
                        config["grid"]["ys"][grid_idx - 1],
                    ),
                    pre_piece(piece_idx),
                    piece(piece_idx, True),
                ],
            },
        )


def piece_to_grid(
    piece_idx: int, grid_idx: int, around_charge_station: bool, pos: bool
):
    if around_charge_station:
        return (
            f"Piece {piece_idx} to Grid {grid_idx} (around CS)",
            {
                "obstacles": [charge_station_obstacle(), wall_obstacle()],
                "waypoints": [
                    piece(piece_idx, False),
                    lower_constraint(),
                    lower_guess(),
                    pre_grid(grid_idx, pos),
                    grid(grid_idx, True, pos),
                ]
                if piece_idx > 2
                else [
                    piece(piece_idx, False),
                    upper_constraint(),
                    upper_guess(),
                    pre_grid(grid_idx, pos),
                    grid(grid_idx, True, pos),
                ],
            },
        )
    else:
        return (
            f"Piece {piece_idx} to Grid {grid_idx} (over CS)",
            {
                "obstacles": [],
                "waypoints": [
                    piece(piece_idx, False),
                    pre_charge_station(
                        math.pi if pos else -math.pi,
                        False,
                        config["grid"]["ys"][grid_idx - 1],
                    ),
                    grid(grid_idx, True, pos),
                ],
            },
        )


def grid_to_charge_station(grid_idx: int, pos: bool):
    return (
        f"Grid {grid_idx} to CS",
        {
            "obstacles": [],
            "waypoints": [
                grid(grid_idx, False, pos),
                pre_charge_station(
                    math.pi if pos else -math.pi,
                    True,
                    config["grid"]["ys"][grid_idx - 1],
                ),
                charge_station(
                    math.pi if pos else -math.pi, config["grid"]["ys"][grid_idx - 1]
                ),
            ],
        },
    )


def piece_to_charge_station(piece_idx: int, pos: bool):
    return (
        f"Piece {piece_idx} to CS",
        {
            "obstacles": [],
            "waypoints": [
                piece(piece_idx, False),
                pre_charge_station(0, False, config["piece"]["ys"][piece_idx - 1]),
                charge_station(0, config["piece"]["ys"][piece_idx - 1]),
            ],
        },
    )


paths = [
    grid_to_piece(1, 1, True, False),
    grid_to_piece(2, 1, True, False),
    grid_to_piece(3, 1, True, False),
    grid_to_piece(1, 2, True, False),
    grid_to_piece(2, 2, True, False),
    grid_to_piece(3, 2, True, False),
    grid_to_piece(4, 2, False, False),
    grid_to_piece(4, 3, False, True),
    grid_to_piece(5, 2, False, False),
    grid_to_piece(5, 3, False, True),
    grid_to_piece(6, 2, False, False),
    grid_to_piece(6, 3, False, True),
    grid_to_piece(7, 3, True, True),
    grid_to_piece(8, 3, True, True),
    grid_to_piece(9, 3, True, True),
    grid_to_piece(7, 4, True, True),
    grid_to_piece(8, 4, True, True),
    grid_to_piece(9, 4, True, True),
    piece_to_grid(1, 1, True, False),
    piece_to_grid(1, 2, True, False),
    piece_to_grid(1, 3, True, False),
    piece_to_grid(2, 1, True, False),
    piece_to_grid(2, 2, True, False),
    piece_to_grid(2, 3, True, False),
    piece_to_grid(2, 4, False, True),
    piece_to_grid(2, 5, False, True),
    piece_to_grid(2, 6, False, True),
    piece_to_grid(3, 4, False, False),
    piece_to_grid(3, 5, False, False),
    piece_to_grid(3, 6, False, False),
    piece_to_grid(3, 7, True, True),
    piece_to_grid(3, 8, True, True),
    piece_to_grid(3, 9, True, True),
    piece_to_grid(4, 7, True, True),
    piece_to_grid(4, 8, True, True),
    piece_to_grid(4, 9, True, True),
    grid_to_charge_station(4, True),
    grid_to_charge_station(5, True),
    grid_to_charge_station(6, False),
    piece_to_charge_station(2, True),
    piece_to_charge_station(3, False),
]

proj = {
    "field_image": {"default": "Charged Up"},
    "paths": {},
    "robot_configuration": {
        "bumper_length": 0.6858,
        "bumper_width": 0.6858,
        "mass": 54.43,
        "moment_of_inertia": 4.0,
        "motor_max_angular_speed": 80,
        "motor_max_torque": 1.0,
        "team_number": 3538,
        "wheel_horizontal_distance": 0.4445,
        "wheel_radius": 0.051,
        "wheel_vertical_distance": 0.4953,
    },
    "selected_path_index": 0,
    "unit_preferences": {
        "acceleration_unit": "metre_per_second",
        "angle_unit": "degree_angle",
        "angular_speed_unit": "revolution_per_minute",
        "length_unit": "metre",
        "mass_unit": "kilogram",
        "moment_of_inertia_unit": "kilogram_square_metre",
        "speed_unit": "metre_per_second",
        "time_unit": "second",
        "torque_unit": "newton_metre",
    },
}

for path in paths:
    proj["paths"][path[0]] = path[1]

with open("project.json", "w") as f:
    json.dump(proj, f)
