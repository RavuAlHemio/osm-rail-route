use serde_json;

use crate::model::PathSegment;


pub fn path_to_geojson<'a, I: Iterator<Item = &'a PathSegment<'a>>>(segments: I) -> serde_json::Value {
    let mut line_string_coords = Vec::new();
    let mut node_ids = Vec::new();
    for segment in segments {
        if line_string_coords.len() == 0 {
            line_string_coords.push(serde_json::json!([
                segment.start_node.lon(),
                segment.start_node.lat(),
            ]));
            node_ids.push(segment.start_node.id.0);
        }
        line_string_coords.push(serde_json::json!([
            segment.end_node.lon(),
            segment.end_node.lat(),
        ]));
        node_ids.push(segment.end_node.id.0);
    }

    serde_json::json!({
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": line_string_coords,
        },
        "attributes": {
            "node_ids": node_ids,
        },
    })
}
