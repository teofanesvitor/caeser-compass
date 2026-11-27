import pytest

from caesar.planner import (
    AntiCollisionChecker, AntiCollisionConfig, PlanningConstraints,
    ReferenceWell, MultiTargetPlanner, Target,
)


def survey_point(md, tvd, ns, ew, inc=None, azi=None):
    point = {"depth": md, "verticalDepth": tvd, "ns": ns, "ew": ew}
    if inc is not None:
        point.update({"inclination": inc, "azimuth": azi})
    return point


def checker(wells, clearance=5.0, step=2.0):
    return AntiCollisionChecker(
        AntiCollisionConfig(clearance, "UTM 24S", "MSL", max_step_m=step), wells
    )


def reference(name, ew, coordinate_system="UTM 24S", vertical_datum="MSL"):
    return ReferenceWell(
        name=name,
        survey=(survey_point(1000, 1000, 0, ew), survey_point(1100, 1100, 0, ew)),
        coordinate_system=coordinate_system, vertical_datum=vertical_datum,
    )


def test_minimum_distance_between_parallel_3d_segments():
    report = checker([reference("WELL-A", 4.0)]).evaluate([
        survey_point(1000, 1000, 0, 0), survey_point(1100, 1100, 0, 0),
    ])
    assert report["distance_m"] == pytest.approx(4.0)
    assert report["planned_md"] == pytest.approx(1000.0, abs=2.0)
    assert report["reference_md"] == pytest.approx(1000.0, abs=2.0)
    assert report["reference_well"] == "WELL-A"
    assert report["valid"] is False
    assert report["uncertainty_evaluated"] is False


def test_reports_every_reference_well_sorted_by_distance():
    report = checker([reference("FAR", 12), reference("NEAR", 3)], clearance=2).evaluate([
        survey_point(1000, 1000, 0, 0), survey_point(1100, 1100, 0, 0),
    ])
    assert report["reference_wells_checked"] == 2
    assert [x["reference_well"] for x in report["reference_results"]] == ["NEAR", "FAR"]
    assert report["valid"] is True


def test_rejects_incompatible_coordinate_frame_and_datum():
    with pytest.raises(ValueError, match="sistema de coordenadas incompatível"):
        checker([reference("A", 5, coordinate_system="LOCAL")])
    with pytest.raises(ValueError, match="datum vertical incompatível"):
        checker([reference("A", 5, vertical_datum="KB")])


def test_minimum_curvature_sampling_follows_arc_not_endpoint_chord():
    planned = [
        survey_point(0, 0, 0, 0, 0, 0),
        survey_point(100, 63.6619772368, 0, 63.6619772368, 90, 90),
    ]
    samples, mode = __import__("caesar.planner.anti_collision", fromlist=["_densify"])._densify(planned, 1.0, 1.0)
    assert mode == "minimum_curvature"
    assert len(samples) > 90
    # The quarter-dogleg arc must not be represented by a single straight chord.
    assert max(point[1][2] for point in samples) > 20.0


def test_rejects_partial_or_mixed_directional_data():
    with pytest.raises(ValueError, match="INC/AZI"):
        checker([ReferenceWell("BAD", (
            survey_point(0, 0, 0, 0, 0, None),
            survey_point(10, 10, 0, 0),
        ), "UTM 24S", "MSL")])


def test_planner_rejects_collision_and_preserves_reason():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3.0))
    start = survey_point(1000, 1000, 0, 0, 0, 0)
    start.update({"inclination": 0.0, "azimuth": 0.0, "dls": 0.0})
    target = Target("T1", tvd=1100.0, ns=0.0, ew=0.0)

    def solver(last_data, target, alignment_target):
        end = survey_point(1100, 1100, 0, 0, 0, 0)
        end.update({"inclination": 0.0, "azimuth": 0.0, "dls": 0.0})
        return [last_data, end]

    with pytest.raises(ValueError, match="afastamento de 4.000 m"):
        planner.plan(start, [target], {"shortest": solver},
                     anti_collision_checker=checker([reference("WELL-A", 4.0)]))


def test_planner_keeps_candidate_when_all_clearances_are_satisfied():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3.0))
    start = survey_point(1000, 1000, 0, 0, 0, 0)
    start.update({"inclination": 0.0, "azimuth": 0.0, "dls": 0.0})
    target = Target("T1", tvd=1100.0, ns=0.0, ew=0.0)

    def solver(last_data, target, alignment_target):
        end = survey_point(1100, 1100, 0, 0, 0, 0)
        end.update({"inclination": 0.0, "azimuth": 0.0, "dls": 0.0})
        return [last_data, end]

    result = planner.plan(start, [target], {"route": solver},
                          anti_collision_checker=checker([reference("WELL-A", 8.0)]))
    assert result["best"].methods == ("route",)
    assert result["anti_collision"]["distance_m"] == pytest.approx(8.0)
    assert result["anti_collision"]["valid"] is True


def test_parallel_overlapping_segments_return_zero_not_endpoint_distance():
    from caesar.planner.anti_collision import _closest_segments
    distance, s, t = _closest_segments(
        (0, 0, 0), (10, 0, 0), (5, 0, 0), (15, 0, 0)
    )
    assert distance == pytest.approx(0.0)
    assert s == pytest.approx(0.5)
    assert t == pytest.approx(0.0)


def test_densify_keeps_each_original_station_boundary():
    from caesar.planner.anti_collision import _densify
    survey = [
        survey_point(0, 0, 0, 0),
        survey_point(10, 10, 0, 0),
        survey_point(20, 20, 10, 0),
    ]
    samples, mode = _densify(survey, 100.0, 1.0)
    assert mode == "linear_chord"
    assert [(md, xyz) for md, xyz in samples if md in (0, 10, 20)] == [
        (0.0, (0.0, 0.0, 0.0)),
        (10.0, (10.0, 0.0, 0.0)),
        (20.0, (20.0, 10.0, 0.0)),
    ]


def test_clearance_uses_conservative_sampling_error_bound():
    report = checker([reference("A", 5.03)], clearance=5.0).evaluate([
        survey_point(1000, 1000, 0, 0), survey_point(1100, 1100, 0, 0),
    ])
    assert report["distance_m"] == pytest.approx(5.03)
    assert report["sampling_error_bound_m"] == pytest.approx(0.0)
    assert report["clearance_lower_bound_m"] == pytest.approx(5.03)
    assert report["valid"] is True


def test_rejects_non_finite_and_invalid_sampling_configuration():
    with pytest.raises(ValueError, match="max_chord_error_m"):
        AntiCollisionConfig(5, "UTM", "MSL", max_chord_error_m=0)
    with pytest.raises(ValueError, match="Coordenadas"):
        checker([reference("A", 5)]).evaluate([
            survey_point(0, float("nan"), 0, 0), survey_point(10, 10, 0, 0),
        ])
