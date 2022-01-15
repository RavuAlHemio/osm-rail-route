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
            OptsMode::LongestPath(lp) => &lp.osm_path,
        }
    }
}
#[derive(Clone, Debug, Parser, PartialEq)]
enum OptsMode {
    Route(RouteOpts),
    DumpWays(DumpWaysOpts),
    LongestPath(LongestPathOpts),
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct RouteOpts {
    pub osm_path: PathBuf,
    pub start_lat: f64,
    pub start_lon: f64,
    pub dest_lat: f64,
    pub dest_lon: f64,
    #[clap(short, long, default_value("30.0"))] pub max_bearing_diff_deg: f64,
    #[clap(short, long)] pub debug_bearing: bool,
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct DumpWaysOpts {
    pub osm_path: PathBuf,
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct LongestPathOpts {
    pub osm_path: PathBuf,
    #[clap(short, long, default_value("30.0"))] pub max_bearing_diff_deg: f64,
    #[clap(short, long)] pub debug_bearing: bool,
}


#[derive(Clone, Debug, PartialEq)]
struct PathSegment<'a> {
    pub start_node: &'a Node,
    pub end_node: &'a Node,
    pub distance: f64,
}
impl<'a> PathSegment<'a> {
    pub fn bearing(&self) -> f64 {
        // L is longitude, θ is latitude
        // ∆L = Lb - La
        // X = cos θb * sin ∆L
        // Y = cos θa * sin θb – sin θa * cos θb * cos ∆L

        let lat_a = self.start_node.lat() * PI / 180.0;
        let lat_b = self.end_node.lat() * PI / 180.0;
        let lon_a = self.start_node.lon() * PI / 180.0;
        let lon_b = self.end_node.lon() * PI / 180.0;

        let lon_delta = lon_b - lon_a;
        let x = lat_b.cos() * lon_delta.sin();
        let y = lat_a.cos() * lat_b.sin() - lat_a.sin() * lat_b.cos() * lon_delta.cos();

        // yeah, the variable names are swapped here
        x.atan2(y)
    }
}


#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct PathChainId(pub i64);


#[derive(Clone, Debug, PartialEq)]
struct PathChain<'a> {
    chain_id: PathChainId,
    segments: Vec<PathSegment<'a>>,
    total_distance: f64,
}
impl<'a> PathChain<'a> {
    pub fn chain_id(&self) -> PathChainId { self.chain_id }
    pub fn segments(&self) -> &Vec<PathSegment<'a>> { &self.segments }
    pub fn total_distance(&self) -> f64 { self.total_distance }

    pub fn new(chain_id: PathChainId, segments: Vec<PathSegment<'a>>) -> Self {
        let total_distance = segments.iter().map(|seg| seg.distance).sum();

        Self {
            chain_id,
            segments,
            total_distance,
        }
    }

    pub fn subchain(&self, start_node_id: NodeId, end_node_id: NodeId) -> Self {
        let mut sub = self.clone();
        remove_while(&mut sub.segments, |seg| seg.start_node.id != start_node_id);
        remove_from(&mut sub.segments, |seg| seg.start_node.id == end_node_id);
        sub.total_distance = sub.segments.iter().map(|seg| seg.distance).sum();
        sub
    }

    pub fn is_valid(&self) -> bool {
        if self.segments.len() == 0 {
            return true;
        }
        let mut prev_node_id = self.segments[0].end_node.id;
        for seg in self.segments.iter().skip(1) {
            if prev_node_id != seg.start_node.id {
                return false;
            }
            prev_node_id = seg.end_node.id;
        }
        true
    }

    pub fn contains_node(&self, node_id: NodeId) -> bool {
        if self.segments.len() == 0 {
            return false;
        }
        if self.segments[0].start_node.id == node_id {
            return true;
        }
        for seg in &self.segments {
            if seg.end_node.id == node_id {
                return true;
            }
        }
        false
    }

    pub fn start_node(&self) -> Option<&Node> {
        self.segments().first().map(|s| s.start_node)
    }

    pub fn end_node(&self) -> Option<&Node> {
        self.segments().last().map(|s| s.end_node)
    }
}


#[derive(Clone, Debug, PartialEq)]
struct DiscoveredPath<'a> {
    pub current_chains: Vec<PathChain<'a>>,
    pub current_node: &'a Node,
    pub total_distance: f64,
    pub heuristic: f64,
}
impl<'a> DiscoveredPath<'a> {
    pub fn count_node_occurrences(&self, node_id: NodeId) -> usize {
        let mut count = 0;
        for chain in &self.current_chains {
            if chain.segments().len() > 0 {
                if chain.segments()[0].start_node.id == node_id {
                    count += 1;
                }
            }
            for seg in chain.segments() {
                if seg.end_node.id == node_id {
                    count += 1;
                }
            }
        }
        count
    }

    pub fn to_nodes(&self) -> Vec<&Node> {
        let mut ret = Vec::new();
        let all_segments = self.current_chains
            .iter()
            .flat_map(|chain| chain.segments().iter());
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


fn find_closest_node<'a>(point: Node, nodes: &HashMap<NodeId, &'a Node>) -> Option<&'a Node> {
    let point_lat: f64 = point.lat();
    let point_lon: f64 = point.lon();

    let mut closest_node_and_distance = None;
    for node in nodes.values() {
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

    closest_node_and_distance.map(|nd| *nd.0)
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

fn calculate_way_chains<'a>(node_to_neighbors: &HashMap<NodeId, HashSet<NodeId>>, id_to_node: &'a HashMap<NodeId, &Node>) -> Vec<PathChain<'a>> {
    let mut next_path_chain_id = 1i64;
    let mut way_chains = Vec::new();

    // get the nodes that are not just pass-through
    let node_set_to_process: HashSet<NodeId> = node_to_neighbors
        .iter()
        .filter(|(_node_id, neighbors)| neighbors.len() != 2)
        .map(|(node_id, _neighbors)| *node_id)
        .collect();
    let mut nodes_to_process: Vec<NodeId> = node_set_to_process
        .iter()
        .map(|&nid| nid)
        .collect();

    while let Some(node_id) = nodes_to_process.pop() {
        // get this node
        let node = id_to_node.get(&node_id).unwrap();

        // gimme neighbors
        let empty_set = HashSet::new();
        let neighbors = node_to_neighbors.get(&node_id)
            .unwrap_or(&empty_set);

        for &neighbor_id in neighbors {
            let neighbor = id_to_node.get(&neighbor_id).unwrap();
            let initial_distance = sphere_distance_meters(
                node.lat(), node.lon(),
                neighbor.lat(), neighbor.lon(),
            );
            let initial_seg = PathSegment {
                start_node: node,
                end_node: neighbor,
                distance: initial_distance,
            };
            let mut segs: Vec<PathSegment> = vec![initial_seg];

            // follow the line until the end or we land at an intersection
            loop {
                let last_seg = segs.last().unwrap();
                let prev_node = last_seg.start_node;
                let cur_node = last_seg.end_node;

                let cur_neighbors = node_to_neighbors.get(&cur_node.id).unwrap();
                if cur_neighbors.len() != 2 {
                    // intersection or terminal!
                    break;
                }
                let &new_neighbor = cur_neighbors.iter()
                    .filter(|&&nid| nid != prev_node.id)
                    .nth(0)
                    .unwrap();
                let new_neighbor_node = id_to_node.get(&new_neighbor).unwrap();

                let distance = sphere_distance_meters(
                    cur_node.lat(), cur_node.lon(),
                    new_neighbor_node.lat(), new_neighbor_node.lon(),
                );
                segs.push(PathSegment {
                    start_node: cur_node,
                    end_node: &new_neighbor_node,
                    distance,
                });
            }

            let path_seg = PathChain::new(
                PathChainId(next_path_chain_id),
                segs,
            );
            next_path_chain_id += 1;
            way_chains.push(path_seg);
        }
    }

    way_chains
}

fn calculate_node_to_way_chains<'a>(way_chains: &'a [PathChain<'a>]) -> HashMap<NodeId, HashMap<PathChainId, &'a PathChain<'a>>> {
    let mut node_to_way_chains = HashMap::new();
    for chain in way_chains {
        for segment in chain.segments() {
            node_to_way_chains.entry(segment.start_node.id)
                .or_insert_with(|| HashMap::new())
                .entry(chain.chain_id())
                .or_insert(chain);
            node_to_way_chains.entry(segment.end_node.id)
                .or_insert_with(|| HashMap::new())
                .entry(chain.chain_id())
                .or_insert(chain);
        }
    }
    node_to_way_chains
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


fn bearing_ok_for_continuation(current_chains: &[PathChain], chain: &PathChain, max_bearing_diff_rad: f64, debug_bearing: bool) -> bool {
    if let Some(last_chain) = current_chains.last() {
        if let Some(last_seg) = last_chain.segments().last() {
            let last_bearing = last_seg.bearing();
            let next_bearing = chain.segments()[0].bearing();

            if (last_bearing - next_bearing).abs() > max_bearing_diff_rad {
                // the difference is too great; skip it
                if debug_bearing {
                    eprintln!(
                        "cannot continue from {}-{} (bearing {:.2}°) to {}-{} (bearing {:.2}°)",
                        last_seg.start_node.id.0, last_seg.end_node.id.0, last_bearing * 180.0 / PI,
                        chain.segments()[0].start_node.id.0, chain.segments()[0].end_node.id.0, next_bearing * 180.0 / PI,
                    );
                }
                return false;
            }
        }
    }
    true
}


fn kinda_astar_search<'a>(
    id_to_node: &'a HashMap<NodeId, &Node>,
    node_to_way_chains: &HashMap<NodeId, HashMap<PathChainId, &PathChain<'a>>>,
    start_node_id: NodeId,
    dest_node_id: NodeId,
    max_bearing_diff_rad: f64,
    mut debug_bearing: bool,
) -> Option<DiscoveredPath<'a>> {
    let start_node = id_to_node.get(&start_node_id).expect("start node not found");
    let dest_node = id_to_node.get(&dest_node_id).expect("destination node not found");
    let initial_heuristic = 0.0 + sphere_distance_meters(
        start_node.lat(), start_node.lon(),
        dest_node.lat(), dest_node.lon(),
    );

    let mut paths: Vec<DiscoveredPath<'a>> = vec![DiscoveredPath {
        current_chains: vec![],
        current_node: start_node,
        total_distance: 0.0,
        heuristic: initial_heuristic,
    }];

    while let Some(path) = paths.pop() {
        if dest_node_id == path.current_node.id {
            // found it!
            return Some(path);
        }
        /*
        if path.count_node_occurrences(path.current_node.id) > 1 {
            // we have returned to this node; it does not make sense to expand this path further
            continue;
        }
        */

        // find the way segments of which the node is a member
        let mut my_way_chains: Vec<PathChain<'a>> = if let Some(wcs) = node_to_way_chains.get(&path.current_node.id) {
            wcs
                .values()
                .map(|chain| {
                    // cut the chain to only contain the segment between our node and (if contained) the destination node
                    chain.subchain(path.current_node.id, dest_node_id)
                })
                .filter(|rel_chain| rel_chain.segments().len() > 0)
                .filter(|chain| bearing_ok_for_continuation(&path.current_chains, chain, max_bearing_diff_rad, debug_bearing))
                .collect()
        } else {
            Vec::new()
        };

        if debug_bearing && my_way_chains.len() == 0 {
            eprintln!("dead end at {:?}", path.current_node);
        }

        // calculate paths
        for chain in my_way_chains.drain(..) {
            assert!(chain.segments().len() > 0);

            // check if we can actually continue on this chain
            // (the angle is not too sharp)
            if let Some(last_chain) = path.current_chains.last() {
                if let Some(last_seg) = last_chain.segments().last() {
                    let last_bearing = last_seg.bearing();
                    let next_bearing = chain.segments()[0].bearing();

                    if (last_bearing - next_bearing).abs() > max_bearing_diff_rad {
                        // the difference is too great; skip it
                        if debug_bearing {
                            eprintln!(
                                "cannot continue from {}-{} (bearing {:.2}°) to {}-{} (bearing {:.2}°)",
                                last_seg.start_node.id.0, last_seg.end_node.id.0, last_bearing * 180.0 / PI,
                                chain.segments()[0].start_node.id.0, chain.segments()[0].end_node.id.0, next_bearing * 180.0 / PI,
                            );
                        }
                        continue;
                    }
                }
            }

            // calculate path segment's end's distance to the goal
            let last_seg = chain.segments().last().unwrap();
            let last_node = id_to_node.get(&last_seg.end_node.id).unwrap();
            let end_crow_dist_to_target = sphere_distance_meters(
                last_node.lat(), last_node.lon(),
                dest_node.lat(), dest_node.lon(),
            );
            let distance = path.total_distance + chain.total_distance();
            let heuristic = distance + end_crow_dist_to_target;

            let mut new_chains = path.current_chains.clone();
            new_chains.push(chain);

            let new_path = DiscoveredPath {
                current_chains: new_chains,
                current_node: last_node,
                total_distance: distance,
                heuristic,
            };
            paths.push(new_path);
        }

        // sort paths by heuristic, largest-first (will be taken last)
        paths.sort_unstable_by(|left, right| {
            right.heuristic.partial_cmp(&left.heuristic).unwrap()
        });
    }

    None
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
                && obj.tags().get("railway").map(|r| r == "tram").unwrap_or(false)
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
    eprintlntime!(start_time, "nodes and ways extracted");

    remove_invalid_ways(&id_to_node, &mut ways);
    eprintlntime!(start_time, "invalid ways removed");

    if let OptsMode::DumpWays(_) = &opts.mode {
        dump_ways(&id_to_node, &ways);
        return;
    }
    let (max_bearing_diff_deg, debug_bearing) = match &opts.mode {
        OptsMode::Route(r) => (r.max_bearing_diff_deg, r.debug_bearing),
        OptsMode::LongestPath(lp) => (lp.max_bearing_diff_deg, lp.debug_bearing),
        _ => unreachable!(),
    };
    let max_bearing_diff_rad = max_bearing_diff_deg * PI / 180.0;

    let node_to_neighbors = calculate_neighbors(&ways);
    eprintlntime!(start_time, "neighbors calculated");

    let way_chains = calculate_way_chains(&node_to_neighbors, &id_to_node);
    for way_chain in &way_chains {
        if !way_chain.is_valid() {
            panic!("way chain invalid! {:#?}", way_chain);
        }
    }
    eprintlntime!(start_time, "way chains calculated");

    let node_to_way_chains = calculate_node_to_way_chains(&way_chains);
    eprintlntime!(start_time, "way chain membership calculated");

    let node_pairs = match &opts.mode {
        OptsMode::Route(r) => {
            let start_fake_node = make_fake_node(r.start_lat, r.start_lon);
            let dest_fake_node = make_fake_node(r.dest_lat, r.dest_lon);

            let start_node = find_closest_node(start_fake_node, &id_to_node)
                .expect("no start node found");

            let dest_node = find_closest_node(dest_fake_node, &id_to_node)
                .expect("no destination node found");

            vec![(start_node, dest_node)]
        },
        OptsMode::LongestPath(_lp) => {
            let mut np = Vec::new();
            for chain1 in way_chains.iter() {
                if chain1.segments().len() == 0 {
                    continue;
                }

                for chain2 in way_chains.iter() {
                    if chain2.segments().len() == 0 {
                        continue;
                    }

                    let node_pairs = &[
                        (chain1.start_node().unwrap(), chain2.start_node().unwrap()),
                        (chain1.start_node().unwrap(), chain2.end_node().unwrap()),
                        (chain1.end_node().unwrap(), chain2.start_node().unwrap()),
                        (chain1.end_node().unwrap(), chain2.end_node().unwrap()),
                    ];
                    for (start_node, dest_node) in node_pairs {
                        if start_node.id == dest_node.id {
                            continue;
                        }
                        np.push((*start_node, *dest_node));
                    }
                }
            }
            np
        },
        _ => unreachable!(),
    };

    let mut longest_path: Option<DiscoveredPath> = None;
    for (start_node, dest_node) in node_pairs {
        eprintlntime!(start_time, "finding path from {:?}", start_node);
        eprintlntime!(start_time, "finding path to {:?}", dest_node);
        let path_opt = kinda_astar_search(
            &id_to_node,
            &node_to_way_chains,
            start_node.id,
            dest_node.id,
            max_bearing_diff_rad,
            debug_bearing,
        );
        eprintlntime!(start_time, "search completed");
        let path = match path_opt {
            Some(p) => p,
            None => panic!("no path found!"),
        };
        eprintlntime!(start_time, "path has total distance of {}", path.total_distance);

        if let Some(lp) = &longest_path {
            if lp.total_distance < path.total_distance {
                longest_path = Some(path);
            }
        } else {
            longest_path = Some(path);
        }
    }

    let path = longest_path.unwrap();
    let mut line_string_coords = Vec::new();
    for chain in &path.current_chains {
        if chain.segments().len() > 0 {
            let seg0 = &chain.segments()[0];
            line_string_coords.push(serde_json::json!([
                seg0.start_node.lon(),
                seg0.start_node.lat(),
            ]));
        }
        for seg in chain.segments() {
            line_string_coords.push(serde_json::json!([
                seg.start_node.lon(),
                seg.start_node.lat(),
            ]));
        }
    }

    let geojson = serde_json::json!({
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": line_string_coords,
        },
    });

    let path_json_string = serde_json::to_string_pretty(&geojson)
        .expect("failed to serialize path to JSON");
    println!("{}", path_json_string);
}