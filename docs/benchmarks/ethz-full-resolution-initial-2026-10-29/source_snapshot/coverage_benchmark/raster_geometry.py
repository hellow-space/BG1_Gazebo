"""Exact pixel-edge polygon conversion via the installed GDAL C API."""
import ctypes as c
import ctypes.util
import json
import numpy as np

lib = c.CDLL(ctypes.util.find_library('gdal'))
ptr = c.c_void_p


def _bind(name, result, args):
    function = getattr(lib, name)
    function.restype, function.argtypes = result, args
    return function


_bind('GDALAllRegister', None, [])()
_bind('OGRRegisterAll', None, [])()
driver = _bind('GDALGetDriverByName', ptr, [c.c_char_p])
create = _bind('GDALCreate', ptr, [ptr, c.c_char_p, c.c_int, c.c_int, c.c_int, c.c_int, ptr])
transform = _bind('GDALSetGeoTransform', c.c_int, [ptr, c.POINTER(c.c_double)])
band = _bind('GDALGetRasterBand', ptr, [ptr, c.c_int])
io = _bind('GDALRasterIO', c.c_int, [ptr, c.c_int, c.c_int, c.c_int, c.c_int, c.c_int,
                                  ptr, c.c_int, c.c_int, c.c_int, c.c_int, c.c_int])
close = _bind('GDALClose', None, [ptr])
vdriver = _bind('OGRGetDriverByName', ptr, [c.c_char_p])
vcreate = _bind('OGR_Dr_CreateDataSource', ptr, [ptr, c.c_char_p, ptr])
layer = _bind('OGR_DS_CreateLayer', ptr, [ptr, c.c_char_p, ptr, c.c_int, ptr])
vclose = _bind('OGR_DS_Destroy', None, [ptr])
poly = _bind('GDALPolygonize', c.c_int, [ptr, ptr, ptr, c.c_int, ptr, ptr, ptr])
next_feature = _bind('OGR_L_GetNextFeature', ptr, [ptr])
geometry = _bind('OGR_F_GetGeometryRef', ptr, [ptr])
export = _bind('OGR_G_ExportToJson', ptr, [ptr])
valid = _bind('OGR_G_IsValid', c.c_int, [ptr])
area = _bind('OGR_G_Area', c.c_double, [ptr])
feature_close = _bind('OGR_F_Destroy', None, [ptr])
free = _bind('VSIFree', None, [ptr])
from_json = _bind('OGR_G_CreateGeometryFromJson', ptr, [c.c_char_p])
geometry_close = _bind('OGR_G_DestroyGeometry', None, [ptr])
burn = _bind('GDALRasterizeGeometries', c.c_int,
             [ptr, c.c_int, c.POINTER(c.c_int), c.c_int, c.POINTER(ptr), ptr, ptr,
              c.POINTER(c.c_double), ptr, ptr, ptr])


def _raster(values, resolution):
    height, width = values.shape
    dataset = create(driver(b'MEM'), b'', width, height, 1, 1, None)
    assert dataset
    assert transform(dataset, (c.c_double*6)(0, resolution, 0, height*resolution, 0, -resolution)) == 0
    target = band(dataset, 1)
    assert io(target, 1, 0, 0, width, height, values.ctypes.data, width, height, 1, 0, 0) == 0
    return dataset, target


def polygonize(mask, resolution):
    values = np.ascontiguousarray(mask, dtype=np.uint8)
    dataset, target = _raster(values, resolution)
    vector = vcreate(vdriver(b'Memory'), b'', None)
    output_layer = layer(vector, b'free', None, 3, None)
    result = []
    try:
        assert poly(target, target, output_layer, -1, None, None, None) == 0
        while True:
            feature = next_feature(output_layer)
            if not feature:
                break
            geom = geometry(feature)
            data = export(geom)
            try:
                result.append(dict(geometry=json.loads(c.string_at(data)),
                                   area_m2=area(geom), valid=bool(valid(geom))))
            finally:
                free(data)
                feature_close(feature)
        return sorted(result, key=lambda item: item['area_m2'], reverse=True)
    finally:
        vclose(vector)
        close(dataset)


def rasterize(polygons, shape, resolution):
    values = np.zeros(shape, np.uint8)
    dataset, target = _raster(values, resolution)
    geometries = [from_json(json.dumps(p['geometry']).encode()) for p in polygons]
    try:
        if geometries:
            assert all(geometries)
            assert burn(dataset, 1, (c.c_int*1)(1), len(geometries),
                        (ptr*len(geometries))(*geometries), None, None,
                        (c.c_double*len(geometries))(*([1.]*len(geometries))),
                        None, None, None) == 0
        height, width = shape
        assert io(target, 0, 0, 0, width, height, values.ctypes.data, width, height, 1, 0, 0) == 0
        return values.astype(bool)
    finally:
        for geom in geometries:
            geometry_close(geom)
        close(dataset)
