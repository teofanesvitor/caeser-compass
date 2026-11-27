import pytest

from caesar.planner import (
    AntiCollisionChecker,
    AntiCollisionConfig,
    PlanningConstraints,
    ReferenceWell,
    MultiTargetPlanner,
    Target,
)


def survey_point(md, tvd, ns, ew):
    return {"depth": md, "verticalDepth": tvd, "ns": ns, "ew": ew}


def checker(wells, clearance=5.0):
    return AntiCollisionChecker(
        AntiCollisionConfig(
            minimum_clearance_m=clearance,
            coordinate_system="UTM 24S",
            vertical_datum="MSL",
        ),
        wells,
    )


def reference(name, ew, coordinate_system="UTM 24S", vertical_datum="MSL"):
    return ReferenceWell(
        name=name,
        survey=(
            survey_point(1000, 1000, 0, ew),
            survey_point(1100, 1100, 0, ew),
        ),
        coordinate_system=coordinate_system,
        vertical_datum=vertical_datum,
    )


def test_minimum_distance_between_parallel_3d_segments():
    report = checker([reference("WELL-A", 4.0)]).evaluate([
        survey_point(1000, 1000, 0, 0),
        survey_point(1100, 1100, 0, 0),
    ])
    assert report["distance_m"] == pytest.approx(4.0)
    assert report["planned_md"] == pytest.approx(1000.0)
    assert report["reference_md"] == pytest.approx(1000.0)
    assert report["reference_well"] == "WELL-A"
    assert report["valid"] is False
    assert report["uncertainty_evaluated"] is False


def test_multiple_reference_wells_select_the_closest():
    report = checker([
        reference("WELL-FAR", 12.0),
        reference("WELL-NEAR", 3.0),
    ], clearance=2.0).evaluate([
        survey_point(1000, 1000, 0, 0),
        survey_point(1100, 1100, 0, 0),
    ])
    assert report["reference_well"] == "WELL-NEAR"
    assert report["distance_m"] == pytest.approx(3.0)
    assert report["valid"] is True


def test_reference_well_requires_matching_coordinate_frame():
    with pytest.raises(ValueError, match="sistema de coordenadas incompatível"):
        checker([reference("WELL-A", 5.0, coordinate_system="LOCAL") ])


def test_reference_well_requires_matching_vertical_datum():
    with pytest.raises(ValueError, match="datum vertical incompatível"):
        checker([reference("WELL-A", 5.0, vertical_datum="KB")])


def test_planner_rejects_collision_before_candidate_selection():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3.0))
    start = {
        "depth": 1000.0, "verticalDepth": 1000.0, "ns": 0.0, "ew": 0.0,
        "inclination": 0.0, "azimuth": 0.0, "dls": 0.0,
    }
    target = Target("T1", tvd=1100.0, ns=0.0, ew=0.0)

    def solver(last_data, target, alignment_target):
        end = {
            "depth": 1100.0, "verticalDepth": 1100.0, "ns": 0.0, "ew": 0.0,
            "inclination": 0.0, "azimuth": 0.0, "dls": 0.0,
        }
        return [last_data, end]

    with pytest.raises(ValueError, match="afastamento de 4.000 m"):
        planner.plan(
            start,
            [target],
            {"shortest": solver},
            anti_collision_checker=checker([reference("WELL-A", 4.0)]),
        )


def test_planner_keeps_candidate_when_clearance_is_satisfied():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3.0))
    start = {
        "depth": 1000.0, "verticalDepth": 1000.0, "ns": 0.0, "ew": 0.0,
        "inclination": 0.0, "azimuth": 0.0, "dls": 0.0,
    }
    target = Target("T1", tvd=1100.0, ns=0.0, ew=0.0)

    def solver(last_data, target, alignment_target):
        return [last_data, {
            "depth": 1100.0, "verticalDepth": 1100.0, "ns": 0.0, "ew": 0.0,
            "inclination": 0.0, "azimuth": 0.0, "dls": 0.0,
        }]

    result = planner.plan(
        start,
        [target],
        {"route": solver},
        anti_collision_checker=checker([reference("WELL-A", 8.0)]),
    )
    assert result["best"].methods == ("route",)
    assert result["anti_collision"]["distance_m"] == pytest.approx(8.0)
    assert result["anti_collision"]["valid"] is True
