import airgoo
from airgoo import HidrawDevice


def dev(pid, vid=airgoo.AIRGOO_VID):
    return HidrawDevice(path="/dev/hidraw9", vid=vid, pid=pid, name="", serial="", phys="")


def test_supported_models():
    d16, d08 = dev(0x3216), dev(0x3F08)
    assert d16.is_airgoo and d16.model.name == "AG-DRGB16"
    assert d16.model.zone_count == 16 and d16.model.zone_names[0] == "A1"
    assert d16.model.zone_names[15] == "B8"
    assert d08.is_airgoo and d08.model.name == "AG-DRGB08"
    assert d08.model.zone_names == ("1", "2", "3", "4", "5", "6", "7", "8")


def test_unknown_devices_are_not_airgoo():
    assert not dev(0x3204).is_airgoo  # AG-DRGB04: untested, not listed
    assert not dev(0x3216, vid=0x1234).is_airgoo
    assert dev(0x1111).model is None


def test_models_fit_the_protocol():
    for model in airgoo.MODELS.values():
        assert model.zone_count <= airgoo.MAX_ZONES


def test_set_zone_by_name_uses_the_models_port_names():
    ctrl = airgoo.AirgooController.__new__(airgoo.AirgooController)
    ctrl._zones = [[] for _ in range(airgoo.MAX_ZONES)]
    ctrl._fd = None
    ctrl._keepalive_thread = None
    ctrl._keepalive_running = False
    ctrl.model = airgoo.MODELS[airgoo.AIRGOO_AG_DRGB08_PID]
    ctrl.set_zone_by_name("5", [(9, 9, 9)])
    assert ctrl._zones[4] == [(9, 9, 9)]
    assert ctrl.zone_count == 8
