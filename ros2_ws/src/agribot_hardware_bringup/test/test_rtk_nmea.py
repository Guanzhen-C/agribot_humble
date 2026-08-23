import importlib.util
import math
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "rtk_nmea_node.py"
SPEC = importlib.util.spec_from_file_location("rtk_nmea_node", MODULE_PATH)
RTK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RTK)


def nmea_sentence(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f"${body}*{checksum:02X}"


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


def test_gga_quality_metadata_is_preserved():
    sentence = (
        "$GNGGA,081402.00,3958.66107245,N,11619.62194811,E,4,20,1.5,"
        "86.0899,M,-8.4292,M,1.0,439*4D"
    )
    metadata = RTK.parse_gga_metadata(sentence)
    assert metadata is not None
    assert metadata.utc_time == "081402.00"
    assert metadata.quality == 4
    assert metadata.satellite_count == 20
    assert math.isclose(metadata.hdop, 1.5)
    assert math.isclose(metadata.differential_age_sec, 1.0)
    assert metadata.reference_station_id == "439"


def test_gga_metadata_preserves_missing_optional_fields():
    body = "GNGGA,081402.00,,,,,0,0,,,M,,M,,"
    metadata = RTK.parse_gga_metadata(nmea_sentence(body))
    assert metadata is not None
    assert metadata.quality == 0
    assert metadata.satellite_count == 0
    assert metadata.hdop is None
    assert metadata.differential_age_sec is None
    assert metadata.reference_station_id == ""


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
    assert math.isclose(solution.heading_std_deg, 0.8259)
    assert math.isclose(
        solution.measurement_time_unix_sec,
        RTK.GPS_EPOCH_UNIX_SEC + 2428 * 604800.0 + 116060.0 - 18.0,
    )
    assert math.isclose(
        RTK.heading_standard_deviation_deg(solution, 1.0, 5.0), 1.0
    )
    assert not RTK.novatel_crc_valid(sentence[:-1] + "1")


def test_float_heading_uses_conservative_uncertainty_floor():
    solution = RTK.UniHeadingSolution(
        solution_status="SOL_COMPUTED",
        position_type="L1_FLOAT",
        baseline_length_m=0.35,
        heading_deg=120.0,
        pitch_deg=0.0,
        heading_std_deg=2.5,
        pitch_std_deg=3.0,
    )
    assert math.isclose(
        RTK.heading_standard_deviation_deg(solution, 1.0, 5.0), 5.0
    )


def test_rmc_and_zda_supply_absolute_utc_measurement_time():
    rmc = nmea_sentence(
        "GNRMC,081402.25,A,3958.66107245,N,11619.62194811,E,0.0,0.0,"
        "200826,,,A"
    )
    zda = nmea_sentence("GNZDA,081402.25,20,08,2026,00,00")
    expected = 1787213642.25
    assert math.isclose(RTK.parse_rmc_datetime(rmc), expected)
    assert math.isclose(RTK.parse_zda_datetime(zda), expected)


def test_gga_time_combines_with_date_and_handles_midnight():
    from datetime import date

    assert math.isclose(
        RTK.combine_utc_date_and_time(date(2026, 8, 20), "235959.75"),
        1787270399.75,
    )
