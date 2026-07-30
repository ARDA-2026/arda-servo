"""local_to_latlon() 단위 테스트. arda-radar의 tests/test_site.py와 동일한 검증."""

import math

import pytest

from arda_servo.site import METERS_PER_DEG_LAT, local_to_latlon


def test_zero_offset_returns_site_latlon_unchanged():
    lat, lon = local_to_latlon(0.0, 0.0, site_lat=37.5, site_lon=127.0, heading_deg=0.0)
    assert lat == pytest.approx(37.5)
    assert lon == pytest.approx(127.0)


def test_heading_zero_forward_moves_north_right_moves_east():
    lat_fwd, lon_fwd = local_to_latlon(0.0, METERS_PER_DEG_LAT, site_lat=0.0, site_lon=0.0, heading_deg=0.0)
    assert lat_fwd == pytest.approx(1.0)
    assert lon_fwd == pytest.approx(0.0, abs=1e-9)

    lat_right, lon_right = local_to_latlon(METERS_PER_DEG_LAT, 0.0, site_lat=0.0, site_lon=0.0, heading_deg=0.0)
    assert lat_right == pytest.approx(0.0, abs=1e-9)
    assert lon_right == pytest.approx(1.0)


def test_heading_90_east_forward_moves_east_right_moves_south():
    lat_fwd, lon_fwd = local_to_latlon(0.0, METERS_PER_DEG_LAT, site_lat=0.0, site_lon=0.0, heading_deg=90.0)
    assert lat_fwd == pytest.approx(0.0, abs=1e-9)
    assert lon_fwd == pytest.approx(1.0)

    lat_right, lon_right = local_to_latlon(METERS_PER_DEG_LAT, 0.0, site_lat=0.0, site_lon=0.0, heading_deg=90.0)
    assert lat_right == pytest.approx(-1.0)
    assert lon_right == pytest.approx(0.0, abs=1e-9)


def test_heading_180_south_flips_both_signs():
    lat, lon = local_to_latlon(
        METERS_PER_DEG_LAT, METERS_PER_DEG_LAT, site_lat=0.0, site_lon=0.0, heading_deg=180.0
    )
    assert lat == pytest.approx(-1.0)
    assert lon == pytest.approx(-1.0)


def test_longitude_compresses_at_higher_latitude():
    site_lat = 60.0
    _, lon_high = local_to_latlon(METERS_PER_DEG_LAT, 0.0, site_lat=site_lat, site_lon=0.0, heading_deg=0.0)
    _, lon_low = local_to_latlon(METERS_PER_DEG_LAT, 0.0, site_lat=0.0, site_lon=0.0, heading_deg=0.0)

    delta_high = abs(lon_high - 0.0)
    delta_low = abs(lon_low - 0.0)
    assert delta_high > delta_low
    assert delta_high == pytest.approx(1.0 / math.cos(math.radians(site_lat)))
