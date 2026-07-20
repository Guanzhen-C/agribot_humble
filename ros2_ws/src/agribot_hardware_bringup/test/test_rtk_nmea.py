import importlib.util
import math
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "rtk_nmea_node.py"
SPEC = importlib.util.spec_from_file_location("rtk_nmea_node", MODULE_PATH)
RTK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RTK)


def test_nmea_checksum_and_coordinate_conversion():
    sentence = (
        "$GNGGA,081402.00,3958.66107245,N,11619.62194811,E,4,20,1.5,"
        "86.0899,M,-8.4292,M,1.0,439*4D"
    )
    assert RTK.nmea_checksum_valid(sentence)
    assert not RTK.nmea_checksum_valid(sentence[:-2] + "00")
    assert math.isclose(
        RTK.nmea_coordinate("3958.66107245", "N"), 39.97768454083333
    )
    assert math.isclose(
        RTK.nmea_coordinate("11619.62194811", "W"), -116.3270324685
    )


def test_ths_parsing_and_heading_conversion():
    heading_deg, valid = RTK.parse_ths_sentence("$GNTHS,179.0284,A*18")
    assert valid
    assert math.isclose(heading_deg, 179.0284)
    assert math.isclose(
        RTK.gnss_heading_to_enu_yaw(heading_deg), math.radians(-89.0284)
    )

    heading_deg, valid = RTK.parse_ths_sentence("$GPTHS,,V*0E")
    assert heading_deg is None
    assert not valid


def test_uniheading_crc_and_solution_parsing():
    sentence = (
        '#UNIHEADINGA,78,GPS,FINE,2428,116060000,0,0,18,11;'
        'SOL_COMPUTED,NARROW_INT,1.4773,179.0284,-2.4202,0.0000,'
        '0.8259,1.0921,"999",27,21,21,15,3,01,3,f3*5d424350'
    )
    solution = RTK.parse_uniheading_sentence(sentence)
    assert solution is not None
    assert solution.valid
    assert solution.position_type == "NARROW_INT"
    assert math.isclose(solution.heading_deg, 179.0284)
    assert math.isclose(solution.baseline_length_m, 1.4773)
    assert not RTK.novatel_crc_valid(sentence[:-1] + "1")
