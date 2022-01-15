use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::path::PathBuf;
use std::time::Instant;

use clap::Parser;
use osmpbfreader::{Node, NodeId, OsmPbfReader, Tags, Way};


#[derive(Clone, Debug, Parser, PartialEq)]
struct Opts {
    pub osm_path: PathBuf,
    pub start_lat: f64,
    pub start_lon: f64,
    pub dest_lat: f64,
    pub dest_lon: f64,
}


#[derive(Clone, Debug, PartialEq)]
struct PathSegment<'a> {
    pub start_node: &'a Node,
    pub end_node: &'a Node,
    pub distance: f64,
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
}


#[derive(Clone, Debug, PartialEq)]
struct DiscoveredPath<'a> {
    pub current_chains: Vec<PathChain<'a>>,
    pub current_node: &'a Node,
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
fn geo_distance_meters(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    // Vincenty's formulae
    let a = 6378137.0; // WGS84
    let f = 1.0/298.257223563; // WGS84
    let b = (1.0 - f) * a;

    let U1 = ((1.0 - f) * lat1.tan()).atan();
    let U2 = ((1.0 - f) * lat2.tan()).atan();
    let L = lon2 - lon1;

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

fn sphere_distance_meters(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    // haversine formula
    let a = 6378137.0; // WGS84
    let f = 1.0/298.257223563; // WGS84
    let b = (1.0 - f) * a;
    let r = (a + b) / 2.0;

    fn haversine(theta: f64) -> f64 {
        (1.0 - theta.cos()) / 2.0
    }

    let hav = haversine(lat2 - lat1) + lat1.cos() * lat2.cos() * haversine(lon2 - lon1);
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


fn ids<'a>(id_to_node: &'a HashMap<NodeId, &Node>, node_to_way_chains: &HashMap<NodeId, HashMap<PathChainId, &PathChain<'a>>>, start_node_id: NodeId, dest_node_id: NodeId, max_depth: usize) -> Option<DiscoveredPath<'a>> {
    let start_node = id_to_node.get(&start_node_id).expect("start node not found");
    let dest_node = id_to_node.get(&dest_node_id).expect("destination node not found");
    let initial_heuristic = 0.0 + sphere_distance_meters(
        start_node.lat(), start_node.lon(),
        dest_node.lat(), dest_node.lon(),
    );

    let mut paths: Vec<DiscoveredPath<'a>> = vec![DiscoveredPath {
        current_chains: vec![],
        current_node: start_node,
        heuristic: initial_heuristic,
    }];

    while let Some(path) = paths.pop() {
        if path.current_chains.len() >= max_depth {
            // too far
            //println!("{:?}", path);
            continue;
        }

        if dest_node_id == path.current_node.id {
            // found it!
            return Some(path);
        }
        if path.count_node_occurrences(path.current_node.id) > 1 {
            // we have returned to this node; it does not make sense to expand this path further
            continue;
        }

        // find the way segments of which the node is a member
        let mut my_way_chains: Vec<PathChain<'a>> = if let Some(wcs) = node_to_way_chains.get(&path.current_node.id) {
            wcs
                .values()
                .map(|chain| {
                    // cut the chain to only contain the segment between our node and (if contained) the destination node
                    chain.subchain(path.current_node.id, dest_node_id)
                })
                .filter(|rel_chain| rel_chain.segments().len() > 0)
                .collect()
        } else {
            Vec::new()
        };

        // calculate paths
        for chain in my_way_chains.drain(..) {
            assert!(chain.segments().len() > 0);

            // calculate path segment's end's distance to the goal
            let last_seg = chain.segments().last().unwrap();
            let last_node = id_to_node.get(&last_seg.end_node.id).unwrap();
            let end_crow_dist_to_target = sphere_distance_meters(
                last_node.lat(), last_node.lon(),
                dest_node.lat(), dest_node.lon(),
            );
            let heuristic = chain.total_distance() + end_crow_dist_to_target;

            let mut new_chains = path.current_chains.clone();
            new_chains.push(chain);

            let new_path = DiscoveredPath {
                current_chains: new_chains,
                current_node: last_node,
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
        let f = File::open(&opts.osm_path)
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

    let start_fake_node = make_fake_node(opts.start_lat, opts.start_lon);
    let dest_fake_node = make_fake_node(opts.dest_lat, opts.dest_lon);

    // find closest point to both coordinates
    let start_node = find_closest_node(start_fake_node, &id_to_node)
        .expect("no start node found");
    eprintlntime!(start_time, "start node: {:?}", start_node);

    let dest_node = find_closest_node(dest_fake_node, &id_to_node)
        .expect("no destination node found");
    eprintlntime!(start_time, "destination node: {:?}", dest_node);

    let mut max_depth = 1024;
    let path = loop {
        eprintlntime!(start_time, "depth {}", max_depth);
        if let Some(path) = ids(&id_to_node, &node_to_way_chains, start_node.id, dest_node.id, max_depth) {
            break path;
        } else {
            max_depth += 1;
        }
    };

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
