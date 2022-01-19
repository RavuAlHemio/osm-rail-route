"use strict";

import * as geojson from "geojson";
import * as leaflet from "leaflet";
import "leaflet-providers";

export module TramRoute {
    let theMap: leaflet.Map;
    let cutOffAfter: number|null = null;

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
        let cutOffAfterString = queryParams.get("cutoff");
        if (cutOffAfterString !== null && /^[0-9]+$/.test(cutOffAfterString)) {
            cutOffAfter = +cutOffAfterString;
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

        theMap = leaflet.map("the-map", {
            center: [48.2083537, 16.3725042],
            zoom: 12,
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
    }

    function obtainBaseLayers(): [string, leaflet.TileLayer.Provider][] {
        return [
            ["OSM (Translucent)", leaflet.tileLayer.provider("OpenStreetMap.Mapnik", { opacity: 0.5 })],
            ["OSM", leaflet.tileLayer.provider("OpenStreetMap.Mapnik")],
        ];
    }

    function cutOffGeoJson(geoJson: geojson.GeoJsonObject) {
        if (cutOffAfter === null) {
            return;
        }

        switch (geoJson.type) {
            case "LineString":
                let lineString = <geojson.LineString>geoJson;
                if (lineString.coordinates.length > cutOffAfter) {
                    lineString.coordinates.length = cutOffAfter;
                }
                break;

            case "MultiLineString":
                let multiLineString = <geojson.MultiLineString>geoJson;
                for (let i = 0; i < multiLineString.coordinates.length; i++) {
                    if (multiLineString.coordinates[i].length > cutOffAfter) {
                        multiLineString.coordinates[i].length = cutOffAfter;
                    }
                }
                break;

            case "MultiPoint":
                let multiPoint = <geojson.MultiPoint>geoJson;
                if (multiPoint.coordinates.length > cutOffAfter) {
                    multiPoint.coordinates.length = cutOffAfter;
                }
                break;

            case "MultiPolygon":
                let multiPolygon = <geojson.MultiPolygon>geoJson;
                if (multiPolygon.coordinates.length > cutOffAfter) {
                    multiPolygon.coordinates.length = cutOffAfter;
                }
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
}

document.addEventListener("DOMContentLoaded", () => TramRoute.initializeMap());
