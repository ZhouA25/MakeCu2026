"""motor_status: error decoding, hints and the reboot packet (CRC per the ROBOTIS manual example)."""
from pillbot.tools import motor_status as ms


def test_reboot_packet_and_errors():
    assert ms.reboot_packet(1).hex(" ") == "ff ff fd 00 01 03 00 08 2f 4e"
    assert ms.decode_errors(0) == "-"
    assert ms.decode_errors(1 << 5) == "OVERLOAD"
    row = {f: 0 for f in ms.FIELDS} | {"Hardware_Error_Status": 32, "Min_Position_Limit": 2047,
                                        "Max_Position_Limit": 2048, "Present_Temperature": 65}
    h = " ".join(ms.hints({"gripper": row}))
    assert "--reboot gripper" in h and "gripper_range" in h and "hot" in h
    assert "OVERLOAD" in ms.table({"gripper": row})
