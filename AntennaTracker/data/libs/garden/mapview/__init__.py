# coding=utf-8
"""
MapView
=======

.. author:: Mathieu Virbel <mat@kivy.org>

MapView is a Kivy widget that display maps.
"""
'''
	Modified by Kyle Prouty so that it runs embedded in application
	kyle@prouty.io
'''

__all__ = ["Coordinate", "Bbox", "MapView", "MapSource", "MapMarker",
           "MapLayer", "MarkerMapLayer", "MapMarkerPopup"]
__version__ = "0.2"

MIN_LATITUDE = -90.
MAX_LATITUDE = 90.
MIN_LONGITUDE = -180.
MAX_LONGITUDE = 180.
CACHE_DIR = "cache"

try:
    # fix if used within garden
    import sys
    sys.modules['mapview'] = sys.modules['data.libs.garden.mapview']
    del sys
except KeyError:
    pass

from mapview.types import Coordinate, Bbox
from mapview.source import MapSource
from mapview.view import MapView, MapMarker, MapLayer, MarkerMapLayer, \
    MapMarkerPopup
