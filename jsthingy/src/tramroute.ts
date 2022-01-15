"use strict";

import * as leaflet from "leaflet";
import "leaflet-providers";

export module TramRoute {
    let theMap: leaflet.Map;

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
        let geoJson = JSON.parse(xhr.responseText);

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
}

document.addEventListener("DOMContentLoaded", () => TramRoute.initializeMap());
