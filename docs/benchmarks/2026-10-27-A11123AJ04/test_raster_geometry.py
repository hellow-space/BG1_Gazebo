import numpy as np
from raster_geometry import polygonize, rasterize


def test_exact_pixel_edges_preserve_hole_and_area():
    mask = np.ones((7, 9), np.uint8)
    mask[2:5, 3:6] = 0
    polygons = polygonize(mask, .05)
    assert len(polygons) == 1
    assert len(polygons[0]['geometry']['coordinates']) == 2
    assert polygons[0]['valid']
    assert abs(polygons[0]['area_m2']-mask.sum()*.05**2) < 1e-10
    assert np.array_equal(rasterize(polygons, mask.shape, .05), mask.astype(bool))


def test_disconnected_regions_remain_separate():
    mask = np.array([[1, 0, 1], [1, 0, 0]], np.uint8)
    polygons = polygonize(mask, 1.)
    assert sorted(p['area_m2'] for p in polygons) == [1., 2.]
    assert np.array_equal(rasterize(polygons, mask.shape, 1.), mask.astype(bool))
