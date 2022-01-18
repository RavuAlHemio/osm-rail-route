[CmdletBinding()]
Param(
    [switch]
    $Download
)

$destdir = "C:\Users\Ondra\Downloads"
$osmosis = "C:\Program Files\osmosis\bin\osmosis.bat"
$mapurl = "https://download.geofabrik.de/europe/austria-latest.osm.pbf"
$mapfile = "austria-latest.osm.pbf"
$tramfile = "vienna-baden-trams-latest.osm.pbf"

$bottom = "47.9926791"
$right = "16.577513"
$top = "48.322668"
$left = "16.181831"

If ($Download) {
    # download Austria extract from Geofabrik
    & curl.exe -L -o "$destdir\$mapfile" "$mapurl"
}

# reduce down to Vienna + Baden trams
& $osmosis `
    --read-pbf "$destdir\$mapfile" `
    --bounding-box "left=$left" "top=$top" "right=$right" "bottom=$bottom" `
    --tag-filter "accept-nodes" "railway=*" "public_transport=*" `
    --tag-filter "reject-ways" `
    --tag-filter "reject-relations" `
    `
    --read-pbf "$destdir\$mapfile" `
    --bounding-box "left=$left" "top=$top" "right=$right" "bottom=$bottom" `
    --tag-filter "accept-ways" "railway=*" `
    --tag-filter "reject-relations" `
    --used-node `
    `
    --merge `
    --write-pbf "$tramfile"
