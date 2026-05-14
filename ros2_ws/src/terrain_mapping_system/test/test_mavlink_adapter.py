from terrain_mapping_system.mission.mavlink_adapter import MavlinkAdapter, MavlinkConfig


class _DummyMaster:
    def __init__(self, flightmode):
        self.flightmode = flightmode


def test_current_mode_returns_master_flightmode():
    adapter = MavlinkAdapter(MavlinkConfig(connection_string='tcp:127.0.0.1:5760'))
    adapter._master = _DummyMaster('GUIDED')

    assert adapter.current_mode() == 'GUIDED'


def test_current_mode_returns_none_when_empty():
    adapter = MavlinkAdapter(MavlinkConfig(connection_string='tcp:127.0.0.1:5760'))
    adapter._master = _DummyMaster('')

    assert adapter.current_mode() is None
