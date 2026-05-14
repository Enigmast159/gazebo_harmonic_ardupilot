from terrain_mapping_system.mission.conversions import enu_to_local_ned, local_ned_to_enu


def test_enu_to_local_ned():
    assert enu_to_local_ned(12.0, -3.0, 7.5) == (-3.0, 12.0, -7.5)


def test_local_ned_round_trip():
    enu = (4.25, 9.5, 13.0)
    assert local_ned_to_enu(*enu_to_local_ned(*enu)) == enu
