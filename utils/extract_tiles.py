#!/usr/bin/env python3
"""
OSM tile extractor using the TileMapBase library.
"""

import sys
import json
import tilemapbase

import matplotlib.pyplot as plt

def main(tiles, lat_lon_list, zoom_level):

    lons = [entry['lon'] for entry in lat_lon_list]
    lats = [entry['lat'] for entry in lat_lon_list]

    extent = tilemapbase.Extent.from_lonlat(min(lons), max(lons),
                                            min(lats), max(lats))
    extent = extent.to_aspect(1.0)
    plotter = tilemapbase.Plotter(extent, tiles, zoom=zoom_level)

    fig, ax = plt.subplots()
    plotter.plot(ax, allow_large=True)


if __name__ == '__main__':
    # Get arguments: 1-json file, 2-zoom level.
    json_path = sys.argv[1]
    zoom_level = int(sys.argv[2])

    with open(json_path) as f:
        lat_lon_list = json.load(f)

    tilemapbase.start_logging()
    tilemapbase.init(create=True)
    tiles = tilemapbase.tiles.build_OSM()

    main(tiles, lat_lon_list, zoom_level)



