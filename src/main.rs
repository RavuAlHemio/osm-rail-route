use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::fs::File;
use std::path::PathBuf;
use std::time::Instant;

use clap::Parser;
use osmpbfreader::{Node, NodeId, OsmPbfReader, Tags, Way};


#[derive(Clone, Debug, Parser, PartialEq)]
struct Opts {
    #[clap(subcommand)] mode: OptsMode,
}
impl Opts {
    pub fn osm_path(&self) -> &PathBuf {
        match &self.mode {
            OptsMode::Route(r) => &r.osm_path,
            OptsMode::DumpWays(dw) => &dw.osm_path,
            OptsMode::ShortestPaths(sp) => &sp.osm_path,
            OptsMode::LongestShortestPath(lp) => &lp.osm_path,
        }
    }
}
#[derive(Clone, Debug, Parser, PartialEq)]
enum OptsMode {
    Route(RouteOpts),
    DumpWays(DumpWaysOpts),
    ShortestPaths(ShortestPathsOpts),
    LongestShortestPath(LongestShortestPathOpts),
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct RouteOpts {
    pub osm_path: PathBuf,
    pub start_lat: f64,
    pub start_lon: f64,
    pub dest_lat: f64,
    pub dest_lon: f64,
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct DumpWaysOpts {
    pub osm_path: PathBuf,
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct ShortestPathsOpts {
    pub osm_path: PathBuf,
    pub start_lat: f64,
    pub start_lon: f64,
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct LongestShortestPathOpts {
    pub osm_path: PathBuf,
}


#[derive(Clone, Debug, PartialEq)]
struct PathSegment<'a> {
    pub start_node: &'a Node,
    pub end_node: &'a Node,
    pub distance: f64,
}
impl<'a> PathSegment<'a> {
    pub fn new_with_sphere_distance(
        start_node: &'a Node,
        end_node: &'a Node,
    ) -> Self {
        let distance = sphere_distance_meters(
            start_node.lat(), start_node.lon(),
            end_node.lat(), end_node.lon(),
        );
        Self {
            start_node,
            end_node,
            distance,
        }
    }

    pub fn bearing(&self) -> f64 {
        bearing(&self.start_node, self.end_node)
    }
}


#[derive(Clone, Debug, PartialEq)]
struct DiscoveredPath<'a> {
    pub current_segments: Vec<PathSegment<'a>>,
    pub current_node: &'a Node,
    pub total_distance: f64,
    pub heuristic: f64,
}
impl<'a> DiscoveredPath<'a> {
    pub fn count_node_occurrences(&self, node_id: NodeId) -> usize {
        let mut count = 0;
        let mut first_node = true;
        for seg in &self.current_segments {
            if first_node {
                if seg.start_node.id == node_id {
                    count += 1;
                }
                first_node = false;
            }

            if seg.end_node.id == node_id {
                count += 1;
            }
        }
        count
    }

    pub fn to_nodes(&self) -> Vec<&Node> {
        let mut ret = Vec::new();
        let all_segments = &self.current_segments;
        let mut first_segment = true;
        for segment in all_segments {
            if first_segment {
                ret.push(segment.start_node);
                first_segment = false;
            }
            ret.push(segment.end_node);
        }
        ret
    }

    pub fn to_segments(&self) -> Vec<&PathSegment<'a>> {
        self.current_segments
            .iter()
            .collect()
    }

    pub fn previous_node(&self) -> Option<&Node> {
        self.to_segments().last().map(|seg| seg.start_node)
    }
}


#[derive(Clone, Debug, PartialEq)]
struct ShortestPath<'a> {
    pub current_segments: Vec<PathSegment<'a>>,
    pub current_node: &'a Node,
    pub total_distance: f64,
}
impl<'a> ShortestPath<'a> {
    pub fn to_nodes(&self) -> Vec<&Node> {
        let mut ret = Vec::new();
        let mut first_segment = true;
        for segment in &self.current_segments {
            if first_segment {
                ret.push(segment.start_node);
                first_segment = false;
            }
            ret.push(segment.end_node);
        }
        ret
    }
}


fn bearing(node_a: &Node, node_b: &Node) -> f64 {
    let lat_a = node_a.lat() * PI / 180.0;
    let lat_b = node_b.lat() * PI / 180.0;
    let lon_a = node_a.lon() * PI / 180.0;
    let lon_b = node_b.lon() * PI / 180.0;

    (lat_b - lat_a).atan2(lon_b - lon_a)
}


fn make_fake_node(lat: f64, lon: f64) -> Node {
    let decimicro_lat = (lat * 1e7) as i32;
    let decimicro_lon = (lon * 1e7) as i32;
    Node {
        id: NodeId(-1),
        tags: Tags::new(),
        decimicro_lat,
        decimicro_lon,
    }
}


#[allow(non_snake_case)]
fn geo_distance_meters(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) -> f64 {
    let lat1_rad = lat1_deg * PI / 180.0;
    let lat2_rad = lat2_deg * PI / 180.0;
    let lon1_rad = lon1_deg * PI / 180.0;
    let lon2_rad = lon2_deg * PI / 180.0;

    // Vincenty's formulae
    let a = 6378137.0; // WGS84
    let f = 1.0/298.257223563; // WGS84
    let b = (1.0 - f) * a;

    let U1 = ((1.0 - f) * lat1_rad.tan()).atan();
    let U2 = ((1.0 - f) * lat2_rad.tan()).atan();
    let L = lon2_rad - lon1_rad;

    let mut lambda = L;
    let mut cos2_alpha;
    let mut sin_sigma;
    let mut cos_sigma;
    let mut sigma;
    let mut cos_2sigmam;
    loop {
        let prev_lambda = lambda;
        sin_sigma = (
            (U2.cos() * lambda.sin()).powi(2)
            + (U1.cos() * U2.sin() - U1.sin() * U2.cos() * lambda.cos()).powi(2)
        ).sqrt();
        cos_sigma = U1.sin() * U2.sin() + U1.cos() * U2.cos() * lambda.cos();
        sigma = sin_sigma.atan2(cos_sigma);
        let sin_alpha = (U1.cos() * U2.cos() * lambda.sin()) / sigma.sin();
        cos2_alpha = 1.0 - sin_alpha.powi(2);
        cos_2sigmam = sigma.cos() - (2.0 * U1.sin() * U2.sin()) / cos2_alpha;
        let C = f / 16.0 * cos2_alpha * (4.0 + f * (4.0 - 3.0 * cos2_alpha));
        lambda = L + (1.0 - C) * f * sin_alpha * (
            sigma + C * sin_sigma * (
                cos_2sigmam + C * cos_sigma * (
                    -1.0 + 2.0 * cos_2sigmam.powi(2)
                )
            )
        );
        if (lambda - prev_lambda).abs() < 1e-6 {
            break;
        }
    }

    let u2 = cos2_alpha * (a.powi(2) - b.powi(2)) / b.powi(2);
    let A = 1.0 + u2 / 16384.0 * (4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2)));
    let B = u2 / 1024.0 * (256.0 + u2 * (128.0 + u2 * (74.0 - 47.0 * u2)));
    let delta_sigma = B * sin_sigma * (
        cos_2sigmam + 1.0/4.0 * B * (
            cos_sigma * (
                -1.0 + 2.0 * cos_2sigmam.powi(2)
            )
            - B/6.0 * cos_2sigmam * (-3.0 + 4.0 * sin_sigma.powi(2)) * (-3.0 + 4.0 * cos_2sigmam.powi(2))
        )
    );
    let s = b * A * (sigma - delta_sigma);

    s
}

fn sphere_distance_meters(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) -> f64 {
    let lat1_rad = lat1_deg * PI / 180.0;
    let lat2_rad = lat2_deg * PI / 180.0;
    let lon1_rad = lon1_deg * PI / 180.0;
    let lon2_rad = lon2_deg * PI / 180.0;

    // haversine formula
    let a = 6378137.0; // WGS84
    let f = 1.0/298.257223563; // WGS84
    let b = (1.0 - f) * a;
    let r = (a + b) / 2.0;

    fn haversine(theta: f64) -> f64 {
        (1.0 - theta.cos()) / 2.0
    }

    let hav = haversine(lat2_rad - lat1_rad) + lat1_rad.cos() * lat2_rad.cos() * haversine(lon2_rad - lon1_rad);
    let d = 2.0 * r * hav.sqrt().asin();

    d
}


fn bearing_diff_rad(bearing1_rad: f64, bearing2_rad: f64) -> f64 {
    let mut left = bearing1_rad - bearing2_rad;
    let mut right = bearing2_rad - bearing1_rad;

    if left < 0.0 {
        left += 2.0 * PI;
    }
    if right < 0.0 {
        right += 2.0 * PI;
    }

    if left < right {
        -left
    } else {
        right
    }
}


fn find_closest_node<'a, I: Iterator<Item = &'a Node>>(point: &Node, nodes: I) -> Option<&'a Node> {
    let point_lat: f64 = point.lat();
    let point_lon: f64 = point.lon();

    let mut closest_node_and_distance = None;
    for node in nodes {
        let node_lat: f64 = node.lat();
        let node_lon: f64 = node.lon();

        let distance = sphere_distance_meters(point_lat, point_lon, node_lat, node_lon);
        if let Some((_closest_node, min_distance)) = closest_node_and_distance {
            if min_distance > distance {
                closest_node_and_distance = Some((node, distance));
            }
        } else {
            closest_node_and_distance = Some((node, distance));
        }
    }

    closest_node_and_distance.map(|nd| nd.0)
}

fn remove_invalid_ways(id_to_node: &HashMap<NodeId, &Node>, ways: &mut Vec<&Way>) {
    ways.retain(|w| !w.nodes.iter().any(|w| !id_to_node.contains_key(w)));
}

fn calculate_neighbors(ways: &[&Way]) -> HashMap<NodeId, HashSet<NodeId>> {
    let mut node_to_neighbors = HashMap::new();
    for way in ways {
        if way.nodes.len() < 2 {
            continue;
        }

        for i in 0..way.nodes.len()-1 {
            let one_id = way.nodes[i];
            let other_id = way.nodes[i+1];

            let one_neighbors = node_to_neighbors.entry(one_id)
                .or_insert_with(|| HashSet::new());
            one_neighbors.insert(other_id);

            let other_neighbors = node_to_neighbors.entry(other_id)
                .or_insert_with(|| HashSet::new());
            other_neighbors.insert(one_id);
        }
    }
    node_to_neighbors
}

fn remove_while<T, P: FnMut(&T) -> bool>(what: &mut Vec<T>, mut pred: P) {
    let mut we_are_done = false;
    what.retain(|v| {
        if we_are_done {
            // predicate has returned false before; keep this item
            true
        } else if pred(v) {
            // predicate has returned true; remove it
            false
        } else {
            // predicate has returned false for the first time; keep the rest
            we_are_done = true;
            true
        }
    })
}

fn remove_from<T, P: FnMut(&T) -> bool>(what: &mut Vec<T>, mut pred: P) {
    let mut we_are_done = false;
    what.retain(|v| {
        if we_are_done {
            // predicate has returned true before; remove this item
            false
        } else if pred(v) {
            // predicate has returned true; start removing
            we_are_done = true;
            false
        } else {
            // predicate has returned false; keep this item
            true
        }
    })
}


fn whittle_down_neighbors(base_node: &Node, neighbors: &mut Vec<&Node>, prev_node_opt: Option<&Node>) {
    const MAX_CROSSING_BEARING_DIFF_DEG: f64 = 15.0;

    let debug = base_node.id.0 == 278895914;

    // allow anything if we don't know the previous node
    let prev_node = match prev_node_opt {
        Some(pn) => pn,
        None => {
            if debug { eprintln!("allowing everything"); }
            return;
        },
    };

    if neighbors.len() < 2 {
        // pass-through node or dead end; allow it
        return;
    }

    // initial crossing heuristic: even number of neighbors
    // additional crossing heuristic: approach the node from all neighbors
    // if each bearing has a plausible continuation, it's a crossing
    // if not, it might be a switch or a yard merge
    let assume_crossing = if neighbors.len() % 2 == 0 {
        let mut ac = true;

        for approach in neighbors.iter() {
            let approach_bearing = bearing(approach, base_node);

            let mut approach_continues = false;
            for depart in neighbors.iter() {
                if approach.id == depart.id {
                    continue;
                }

                let depart_bearing = bearing(base_node, depart);

                let bearing_delta = bearing_diff_rad(approach_bearing, depart_bearing).abs();
                if (bearing_delta * 180.0 / PI) < MAX_CROSSING_BEARING_DIFF_DEG {
                    approach_continues = true;
                    break;
                }
            }

            if !approach_continues {
                ac = false;
                break;
            }
        }

        ac
    } else {
        false
    };

    // remove the node we came from
    neighbors.retain(|n| n.id != prev_node.id);

    let prev_bearing = bearing(prev_node, base_node);

    if assume_crossing {
        // keep only the neighbor that is closest in terms of bearing
        let (_bearing_diff, bearing_neighbor) = neighbors.iter()
            .map(|n| {
                let neigh_bearing = bearing(base_node, n);
                let bearing_diff = bearing_diff_rad(prev_bearing, neigh_bearing).abs();
                (bearing_diff, *n)
            })
            .min_by(|(bd1, _n1), (bd2, _n2)| bd1.partial_cmp(bd2).unwrap())
            .unwrap();

        neighbors.retain(|n| n.id == bearing_neighbor.id);
        return;
    }

    // heuristic for switches and unmarked intersecting ways: max 30°
    neighbors.retain(|n| {
        let neigh_bearing = bearing(base_node, n);
        let bearing_diff_deg = bearing_diff_rad(prev_bearing, neigh_bearing).abs() * 180.0 / PI;
        bearing_diff_deg < 30.0
    });
}


fn kinda_astar_search<'a>(
    id_to_node: &'a HashMap<NodeId, &Node>,
    node_to_neighbors: &HashMap<NodeId, HashSet<NodeId>>,
    start_node_id: NodeId,
    dest_node_id: NodeId,
) -> Option<DiscoveredPath<'a>> {
    let start_node = id_to_node.get(&start_node_id).expect("start node not found");
    let dest_node = id_to_node.get(&dest_node_id).expect("destination node not found");
    let mut visited_segments: HashSet<(NodeId, NodeId)> = HashSet::new();
    let initial_heuristic = 0.0 + sphere_distance_meters(
        start_node.lat(), start_node.lon(),
        dest_node.lat(), dest_node.lon(),
    );

    let mut paths: Vec<DiscoveredPath<'a>> = vec![DiscoveredPath {
        current_segments: vec![],
        current_node: start_node,
        total_distance: 0.0,
        heuristic: initial_heuristic,
    }];

    while let Some(path) = paths.pop() {
        if dest_node_id == path.current_node.id {
            // found it!
            return Some(path);
        }

        // find neighbors
        let mut neighbors = if let Some(node_neighbors) = node_to_neighbors.get(&path.current_node.id) {
            node_neighbors.iter()
                .filter_map(|n| id_to_node.get(n))
                .map(|n| *n)
                .collect()
        } else {
            Vec::new()
        };

        let prev_node_opt = path.previous_node();
        whittle_down_neighbors(&path.current_node, &mut neighbors, prev_node_opt);
        for neighbor in &neighbors {
            if !visited_segments.insert((path.current_node.id, neighbor.id)) {
                // we have taken this segment before
                // prevent looping infinitely
                continue;
            }

            let segment = PathSegment::new_with_sphere_distance(
                path.current_node,
                neighbor,
            );
            let remain_distance_optimistic = sphere_distance_meters(
                neighbor.lat(), neighbor.lon(),
                dest_node.lat(), dest_node.lon(),
            );

            let mut new_path = path.clone();
            new_path.current_node = neighbor;
            new_path.total_distance += segment.distance;
            new_path.current_segments.push(segment);
            new_path.heuristic = new_path.total_distance + remain_distance_optimistic;

            paths.push(new_path);
        }

        // sort paths by heuristic, largest-first (will be taken last)
        paths.sort_unstable_by(|left, right| {
            right.heuristic.partial_cmp(&left.heuristic).unwrap()
        });
    }

    None
}


fn shortest_paths<'a>(
    id_to_node: &'a HashMap<NodeId, &Node>,
    node_to_neighbors: &HashMap<NodeId, HashSet<NodeId>>,
    start_node_id: NodeId,
    dest_nodes: &HashSet<NodeId>,
) -> HashMap<NodeId, ShortestPath<'a>> {
    let start_node = id_to_node.get(&start_node_id).expect("start node not found");
    let mut dest_node_to_path: HashMap<NodeId, ShortestPath> = HashMap::new();
    let mut visited_segments: HashSet<(NodeId, NodeId)> = HashSet::new();

    let mut paths: Vec<ShortestPath<'a>> = vec![ShortestPath {
        current_segments: vec![],
        current_node: start_node,
        total_distance: 0.0,
    }];

    while let Some(path) = paths.pop() {
        if dest_nodes.contains(&path.current_node.id) {
            // found a destination node!
            if let Some(cur_path) = dest_node_to_path.get(&path.current_node.id) {
                if cur_path.total_distance > path.total_distance {
                    dest_node_to_path.insert(path.current_node.id, path.clone());
                }
                // otherwise keep the shorter path
            } else {
                dest_node_to_path.insert(path.current_node.id, path.clone());
            }
        }

        // check neighborhood
        let mut neighbors: Vec<&Node> = node_to_neighbors
            .get(&path.current_node.id).unwrap()
            .iter()
            .map(|nid| *id_to_node.get(nid).unwrap())
            .collect();

        whittle_down_neighbors(
            &path.current_node,
            &mut neighbors,
            path.current_segments.last().map(|ls| ls.start_node),
        );

        for &neighbor in &neighbors {
            // have I taken this path before?
            if !visited_segments.insert((path.current_node.id, neighbor.id)) {
                // yes
                //eprintln!("    I've gone this path before");
                continue;
            }

            let new_seg = PathSegment::new_with_sphere_distance(
                path.current_node,
                neighbor,
            );

            // store new path
            //eprintln!("    I shall remember this");
            let mut new_path = path.clone();
            new_path.total_distance += new_seg.distance;
            new_path.current_segments.push(new_seg);
            new_path.current_node = neighbor;
            paths.push(new_path);
        }

        // sort paths, shortest last (because pop removes the last one)
        paths.sort_unstable_by(|left, right|
            right.total_distance.partial_cmp(&left.total_distance).unwrap()
        );
    }

    dest_node_to_path
}

fn dump_ways(id_to_node: &HashMap<NodeId, &Node>, ways: &[&Way]) {
    let geoways: Vec<serde_json::Value> = ways
        .iter()
        .map(|way| {
            let coords: Vec<serde_json::Value> = way.nodes
                .iter()
                .map(|nid| {
                    let node = id_to_node.get(nid).unwrap();
                    serde_json::json!([node.lon(), node.lat()])
                })
                .collect();
            serde_json::json!({
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": coords,
                },
            })
        })
        .collect();
    let geojson = serde_json::json!({
        "type": "FeatureCollection",
        "features": geoways,
    });
    let all_json_string = serde_json::to_string_pretty(&geojson)
        .expect("failed to serialize path to JSON");
    println!("{}", all_json_string);
}

macro_rules! eprintlntime {
    ($start_time:tt) => {
        eprintln!("[{:.6}]", (Instant::now() - $start_time).as_secs_f64());
    };
    ($start_time:tt, $($arg : tt) *) => {
        eprint!("[{:.6}] ", (Instant::now() - $start_time).as_secs_f64());
        eprintln!($($arg)*);
    };
}

fn path_to_geojson<'a, I: Iterator<Item = &'a PathSegment<'a>>>(segments: I) -> serde_json::Value {
    let mut line_string_coords = Vec::new();
    for segment in segments {
        if line_string_coords.len() == 0 {
            line_string_coords.push(serde_json::json!([
                segment.start_node.lon(),
                segment.start_node.lat(),
            ]));
        }
        line_string_coords.push(serde_json::json!([
            segment.end_node.lon(),
            segment.end_node.lat(),
        ]));
    }

    serde_json::json!({
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": line_string_coords,
        },
    })
}


fn main() {
    let opts = Opts::parse();

    let start_time = Instant::now();

    let railways = {
        let f = File::open(&opts.osm_path())
            .expect("failed to open file");
        let mut reader = OsmPbfReader::new(f);
        reader
            .get_objs_and_deps(|obj|
                obj.is_way()
                //&& obj.tags().get("railway").map(|r| r != "platform").unwrap_or(false)
                && (
                    obj.tags().get("railway").map(|r|
                        r == "tram" || r == "tram_stop"
                        || r == "light_rail" || r == "stop"
                    ).unwrap_or(false)
                    || obj.tags().get("public_transport").map(|pt|
                        pt == "stop_position"
                    ).unwrap_or(false)
                )
            )
            .expect("failed to obtain railways")
    };
    eprintlntime!(start_time, "railways loaded");

    let id_to_node: HashMap<NodeId, &Node> = railways.values()
        .filter_map(|o| o.node())
        .map(|n| (n.id, n))
        .collect();
    let mut ways: Vec<&Way> = railways.values()
        .filter_map(|o| o.way())
        .collect();
    let way_node_ids: HashSet<NodeId> = railways.values()
        .filter_map(|o| o.way())
        .flat_map(|w| w.nodes.iter())
        .map(|nid| *nid)
        .filter(|nid| id_to_node.contains_key(nid))
        .collect();
    let way_nodes: Vec<&Node> = way_node_ids.iter()
        .map(|i| *id_to_node.get(i).unwrap())
        .collect();
    let stops: Vec<&Node> = railways.values()
        .filter_map(|o| o.node())
        .filter(|n|
            n.tags.get("railway").map(|r|
                r == "tram_stop" || r == "stop"
            ).unwrap_or(false)
            || n.tags.get("public_transport").map(|pt|
                pt == "stop_position"
            ).unwrap_or(false)
        )
        .collect();
    eprintlntime!(start_time, "nodes and ways extracted");

    remove_invalid_ways(&id_to_node, &mut ways);
    eprintlntime!(start_time, "invalid ways removed");

    if let OptsMode::DumpWays(_) = &opts.mode {
        dump_ways(&id_to_node, &ways);
        return;
    }

    let node_to_neighbors = calculate_neighbors(&ways);
    eprintlntime!(start_time, "neighbors calculated");

    let geojson = match &opts.mode {
        OptsMode::Route(r) => {
            let start_fake_node = make_fake_node(r.start_lat, r.start_lon);
            let dest_fake_node = make_fake_node(r.dest_lat, r.dest_lon);

            let start_node = find_closest_node(&start_fake_node, way_nodes.iter().map(|n| *n))
                .expect("no start node found");

            let dest_node = find_closest_node(&dest_fake_node, way_nodes.iter().map(|n| *n))
                .expect("no destination node found");

            eprintlntime!(start_time, "finding path from {:?}", start_node);
            eprintlntime!(start_time, "finding path to {:?}", dest_node);
            let path_opt = kinda_astar_search(
                &id_to_node,
                &node_to_neighbors,
                start_node.id,
                dest_node.id,
            );
            eprintlntime!(start_time, "search completed");
            let path = match path_opt {
                Some(p) => p,
                None => panic!("no path found!"),
            };
            eprintlntime!(start_time, "path has total distance of {}", path.total_distance);

            path_to_geojson(path.to_segments().iter().map(|ps| *ps))
        },
        OptsMode::ShortestPaths(sp) => {
            let stop_closest_node_ids: HashSet<NodeId> = stops.iter()
                .filter_map(|s| find_closest_node(s, way_nodes.iter().map(|n| *n)))
                .map(|n| n.id)
                .collect();

            let start_fake_node = make_fake_node(sp.start_lat, sp.start_lon);
            let start_node = find_closest_node(&start_fake_node, way_nodes.iter().map(|n| *n))
                .expect("no start node found");
            eprintlntime!(start_time, "finding shortest paths from {:?}", start_node);
            let foundlings = shortest_paths(
                &id_to_node,
                &node_to_neighbors,
                start_node.id,
                &stop_closest_node_ids,
            );
            eprintlntime!(start_time, "search completed");

            let geoways: Vec<serde_json::Value> = foundlings
                .iter()
                .map(|(dest_node, shortest_path)| {
                    let coords: Vec<serde_json::Value> = shortest_path.to_nodes()
                        .iter()
                        .map(|node| {
                            serde_json::json!([node.lon(), node.lat()])
                        })
                        .collect();
                    serde_json::json!({
                        "type": "Feature",
                        "properties": {
                            "dest_node_id": dest_node.0,
                        },
                        "geometry": {
                            "type": "LineString",
                            "coordinates": coords,
                        },
                    })
                })
                .collect();
            serde_json::json!({
                "type": "FeatureCollection",
                "features": geoways,
            })
        },
        OptsMode::LongestShortestPath(_lp) => {
            let stop_closest_node_ids: HashSet<NodeId> = stops.iter()
                .filter_map(|s| find_closest_node(s, way_nodes.iter().map(|n| *n)))
                .map(|n| n.id)
                .collect();
            let mut longest_shortest_path: Option<(NodeId, NodeId, ShortestPath)> = None;

            for (i, &start_node_id) in stop_closest_node_ids.iter().enumerate() {
                let mut other_node_ids = stop_closest_node_ids.clone();
                other_node_ids.remove(&start_node_id);

                let mut foundlings = shortest_paths(
                    &id_to_node,
                    &node_to_neighbors,
                    start_node_id,
                    &other_node_ids,
                );
                eprintlntime!(start_time, "{}/{} from {:?}: {} other IDs, {} paths found", i+1, stop_closest_node_ids.len(), start_node_id, other_node_ids.len(), foundlings.len());

                for (dest_node_id, path) in foundlings.drain() {
                    if let Some((_, _, longest_shortest_path_detail)) = &longest_shortest_path {
                        if longest_shortest_path_detail.total_distance < path.total_distance {
                            eprintlntime!(start_time, "new longest shortest path is between {:?} and {:?} ({})", start_node_id, dest_node_id, &path.total_distance);
                            longest_shortest_path = Some((start_node_id, dest_node_id, path));
                        }
                        // only remember the shortest path that is the longest
                    } else {
                        eprintlntime!(start_time, "first longest shortest path is between {:?} and {:?} ({})", start_node_id, dest_node_id, &path.total_distance);
                        longest_shortest_path = Some((start_node_id, dest_node_id, path));
                    }
                }
            }

            let longest_shortest_path_detail = longest_shortest_path
                .map(|(_, _, sp)| sp)
                .expect("no path found");
            eprintlntime!(start_time, "done! longest shortest path distance: {}", longest_shortest_path_detail.total_distance);
            path_to_geojson(longest_shortest_path_detail.current_segments.iter())
        },
        _ => unreachable!(),
    };

    let path_json_string = serde_json::to_string_pretty(&geojson)
        .expect("failed to serialize path to JSON");
    println!("{}", path_json_string);
}


#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use super::{bearing_diff_rad, make_fake_node, PathSegment};

    #[inline]
    fn r2d(r: f64) -> f64 { r * 180.0 / PI }

    #[test]
    fn test_heading_diff() {
        let bottom = make_fake_node(48.1985525, 16.3060627);
        let center = make_fake_node(48.1985709, 16.3060451);
        let left = make_fake_node(48.1985968, 16.3060237);
        let right = make_fake_node(48.1985954, 16.3060272);

        let seg_bc = PathSegment::new_with_sphere_distance(&bottom, &center);
        let seg_cl = PathSegment::new_with_sphere_distance(&bottom, &left);
        let seg_cr = PathSegment::new_with_sphere_distance(&bottom, &right);
        let seg_lc = PathSegment::new_with_sphere_distance(&left, &bottom);
        let seg_rc = PathSegment::new_with_sphere_distance(&right, &bottom);
        let seg_cb = PathSegment::new_with_sphere_distance(&center, &bottom);

        let bc_bearing = seg_bc.bearing();
        let cl_bearing = seg_cl.bearing();
        let cr_bearing = seg_cr.bearing();
        let lc_bearing = seg_lc.bearing();
        let rc_bearing = seg_rc.bearing();
        let cb_bearing = seg_cb.bearing();

        let bc_cl_diff = bearing_diff_rad(bc_bearing, cl_bearing);
        let bc_cr_diff = bearing_diff_rad(bc_bearing, cr_bearing);
        let lc_cr_diff = bearing_diff_rad(lc_bearing, cr_bearing);
        panic!(
            "BC_CL={:.02}° BC_CR={:.02}° LC_CR={:.02}°",
            r2d(bc_cl_diff), r2d(bc_cr_diff), r2d(lc_cr_diff),
        );
    }
}
