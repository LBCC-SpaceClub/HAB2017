# coding=utf-8
"""
This example demonstrate how to use the MBTilesMapSource provider.
It supports v1.1 version of .mbtiles of Mapbox.
See more at http://mbtiles.org/

It currently require a Kivy version that can load data from a buffer. This
is not the case on every platform at 1.8.1, but we're going to fix it.
"""

import sys
from kivy.base import runTouchApp

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from mapview import MapView
from mapview.mbtsource import MBTilesMapSource


source = MBTilesMapSource(sys.argv[1])
runTouchApp(MapView(
    map_source=source,
    lat=source.default_lat,
    lon=source.default_lon,
    zoom=source.default_zoom))
