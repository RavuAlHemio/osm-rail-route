"use strict";

import * as geojson from "geojson";
import * as leaflet from "leaflet";
import "leaflet-providers";

export module TramRoute {
    let theMap: leaflet.Map;
    let segmentSlice: [number, number]|null = null;

    const DEFAULT_CENTER: [number, number] = [48.2083537, 16.3725042];
    const DEFAULT_ZOOM: number = 12;
    const INTEGER_REGEX_STR = '(?:0|[1-9][0-9]+)';
    const FLOAT_REGEX_STR = '(?:(?:0|[1-9][0-9]+)(?:[.][0-9]+)?)';
    const MAP_FRAG_REGEX = new RegExp(`map=(${INTEGER_REGEX_STR})/(${FLOAT_REGEX_STR})/(${FLOAT_REGEX_STR})`);

    export function initializeMap(): void {
        // get the name of the map
        let queryParams = new URLSearchParams(window.location.search);
        let mapName = queryParams.get("map");
        if (mapName === null || mapName === "") {
            let mapElem = document.getElementById("the-map");
            if (mapElem !== null) {
                mapElem.textContent = "You must specify a map to load.";
            }
            return;
        }
        let sliceString = queryParams.get("slice");
        if (sliceString !== null) {
            if (/^[0-9]+$/.test(sliceString)) {
                segmentSlice = [0, +sliceString];
            } else if (/^[0-9]+-[0-9]+$/.test(sliceString)) {
                segmentSlice = <[number, number]>sliceString.split("-").map(v => +v);
            }
        }

        // construct the map URL
        let myPathPieces = window.location.pathname.split("/");
        if (myPathPieces.length > 1) {
            myPathPieces.pop();
        }
        myPathPieces.push("maps");
        myPathPieces.push(encodeURIComponent(mapName));
        let mapURL = `${window.location.origin}${myPathPieces.join("/")}.geojson`;

        // fetch it
        let xhr = new XMLHttpRequest();
        xhr.addEventListener("load", () => mapDownloaded(xhr));
        xhr.open("GET", mapURL, true);
        xhr.send();
    }

    function mapDownloaded(xhr: XMLHttpRequest): void {
        // store downloaded map
        let geoJson: geojson.GeoJsonObject = JSON.parse(xhr.responseText);
        cutOffGeoJson(geoJson);

        // set up Leaflet
        let baseLayers = obtainBaseLayers();
        let trackLayer = leaflet.geoJSON(geoJson, {});

        let layers: leaflet.Layer[] = baseLayers.map(nameAndLayer => nameAndLayer[1]);
        layers.push(trackLayer);

        let center = DEFAULT_CENTER;
        let zoom = DEFAULT_ZOOM;
        let map_match = MAP_FRAG_REGEX.exec(window.location.hash);
        if (map_match !== null) {
            zoom = +map_match[1];
            center = [+map_match[2], +map_match[3]];
        }

        theMap = leaflet.map("the-map", {
            center: center,
            zoom: zoom,
            layers: layers,
        });
        let baseMaps: any = {};
        for (let nameAndLayer of baseLayers) {
            baseMaps[nameAndLayer[0]] = nameAndLayer[1];
        }
        let overlayMaps: any = {
            "track": trackLayer,
        };
        let layerControl = leaflet.control.layers(baseMaps, overlayMaps);
        layerControl.addTo(theMap);
        theMap.addEventListener("zoomend", mapSliceChanged);
        theMap.addEventListener("moveend", mapSliceChanged);
    }

    function obtainBaseLayers(): [string, leaflet.TileLayer.Provider][] {
        return [
            ["OSM (Translucent)", leaflet.tileLayer.provider("OpenStreetMap.Mapnik", { opacity: 0.5 })],
            ["OSM", leaflet.tileLayer.provider("OpenStreetMap.Mapnik")],
        ];
    }

    function sliceSegments<T>(array: T[]): T[] {
        if (segmentSlice === null) {
            return array;
        }
        return array.slice(segmentSlice[0], segmentSlice[1]);
    }

    function cutOffGeoJson(geoJson: geojson.GeoJsonObject) {
        if (segmentSlice === null) {
            return;
        }

        switch (geoJson.type) {
            case "LineString":
                let lineString = <geojson.LineString>geoJson;
                lineString.coordinates = sliceSegments(lineString.coordinates);
                break;

            case "MultiLineString":
                let multiLineString = <geojson.MultiLineString>geoJson;
                for (let i = 0; i < multiLineString.coordinates.length; i++) {
                    multiLineString.coordinates[i] = sliceSegments(multiLineString.coordinates[i]);
                }
                break;

            case "MultiPoint":
                let multiPoint = <geojson.MultiPoint>geoJson;
                multiPoint.coordinates = sliceSegments(multiPoint.coordinates);
                break;

            case "MultiPolygon":
                let multiPolygon = <geojson.MultiPolygon>geoJson;
                multiPolygon.coordinates = sliceSegments(multiPolygon.coordinates);
                break;

            case "Feature":
                let feature = <geojson.Feature>geoJson;
                cutOffGeoJson(feature.geometry);
                break;

            case "FeatureCollection":
                let featureCollection = <geojson.FeatureCollection>geoJson;
                for (let i = 0; i < featureCollection.features.length; i++) {
                    cutOffGeoJson(featureCollection.features[i]);
                }
                break;

            case "GeometryCollection":
                let geometryCollection = <geojson.GeometryCollection>geoJson;
                for (let i = 0; i < geometryCollection.geometries.length; i++) {
                    cutOffGeoJson(geometryCollection.geometries[i]);
                }
                break;
        }
    }

    function mapSliceChanged() {
        let center = theMap.getCenter();
        let zoomLevel = theMap.getZoom();

        window.location.hash = `#map=${zoomLevel}/${center.lat.toFixed(5)}/${center.lng.toFixed(5)}`;
    }
}

document.addEventListener("DOMContentLoaded", () => TramRoute.initializeMap());
