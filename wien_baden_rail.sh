#!/bin/sh
destdir="$HOME/Downloads"
osmosis="$HOME/osmosis/bin/osmosis.sh"
mapurl="https://download.geofabrik.de/europe/austria-latest.osm.pbf"
mapfile="austria-latest.osm.pbf"
tramfile="vienna-baden-trams-latest.osm.pbf"

bottom="47.9926791"
right="16.577513"
top="48.322668"
left="16.181831"

if [ "$1" = "-d" ]
then
    # download Austria extract from Geofabrik
    curl -L -o "$destdir/$mapfile" "$mapurl"
fi

# reduce down to Vienna + Baden trams
$osmosis \
    --read-pbf "$destdir/$mapfile" \
    --bounding-box "left=$left" "top=$top" "right=$right" "bottom=$bottom" \
    --tag-filter "accept-nodes" "railway=*" "public_transport=*" \
    --tag-filter "reject-ways" \
    --tag-filter "reject-relations" \
    \
    --read-pbf "$destdir/$mapfile" \
    --bounding-box "left=$left" "top=$top" "right=$right" "bottom=$bottom" \
    --tag-filter "accept-ways" "railway=*" \
    --tag-filter "reject-relations" \
    --used-node \
    \
    --merge \
    --write-pbf "$tramfile"
