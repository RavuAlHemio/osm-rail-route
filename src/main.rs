mod algorithms;
mod collections;
mod geo;
mod model;
mod util;


use std::collections::{BTreeSet, HashMap, HashSet};
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use clap::Parser;
use osmpbfreader::{Node, NodeId, OsmPbfReader, Tags, Way};
use rayon::prelude::*;
use toml;

use crate::algorithms::{calculate_chains, kinda_astar_search, longest_path_from, shortest_paths};
use crate::geo::sphere_distance_meters;
use crate::model::{LengthPath, Loop, LoopDefinitions, PreferredNeighbors};
use crate::util::path_to_geojson;


#[derive(Clone, Debug, Parser, PartialEq)]
struct Opts {
    pub osm_path: PathBuf,
    #[clap(subcommand)] mode: OptsMode,
    #[clap(short = 'D', long, takes_value = true, multiple_occurrences = true)] pub debug_node_ids: Vec<i64>,
    #[clap(short, long)] pub directional_segments: bool,
    #[clap(short = 'n', long)] pub preferred_neighbors: Option<PathBuf>,
}
#[derive(Clone, Debug, Parser, PartialEq)]
enum OptsMode {
    Route(RouteOpts),
    DumpWays,
    ShortestPaths(ShortestPathsOpts),
    LongestShortestPath,
    LongestPath(LongestPathOpts),
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct RouteOpts {
    pub start_lat: f64,
    pub start_lon: f64,
    pub dest_lat: f64,
    pub dest_lon: f64,
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct ShortestPathsOpts {
    pub start_lat: f64,
    pub start_lon: f64,
}
#[derive(Clone, Debug, Parser, PartialEq)]
struct LongestPathOpts {
    pub start_node_ids: Vec<i64>,
    #[clap(short = 'l', long)] pub loop_def_file: Option<PathBuf>,
    #[clap(short = 'p', long)] pub progress_file: Option<PathBuf>,
    #[clap(short = 'e', long, default_value = "8192")] pub progress_each: usize,
}


#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum OneWay {
    Opposite,
    No,
    Along,
}
impl Default for OneWay {
    fn default() -> Self { OneWay::No }
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

fn calculate_neighbors(ways: &[&Way]) -> (HashMap<NodeId, BTreeSet<NodeId>>, HashSet<(NodeId, NodeId)>) {
    let mut node_to_neighbors = HashMap::new();
    let mut one_way_pairs = HashSet::new();
    for way in ways {
        if way.nodes.len() < 2 {
            continue;
        }
        let one_way = if let Some(rpd) = way.tags.get("railway:preferred_direction") {
            if rpd == "forward" {
                OneWay::Along
            } else if rpd == "backward" {
                OneWay::Opposite
            } else {
                OneWay::No
            }
        } else if let Some(ow) = way.tags.get("oneway") {
            if ow == "yes" || ow == "true" || ow == "1" {
                OneWay::Along
            } else if ow == "-1" || ow == "reverse" {
                OneWay::Opposite
            } else {
                OneWay::No
            }
        } else {
            OneWay::No
        };

        for i in 0..way.nodes.len()-1 {
            let one_id = way.nodes[i];
            let other_id = way.nodes[i+1];

            let one_neighbors = node_to_neighbors.entry(one_id)
                .or_insert_with(|| BTreeSet::new());
            one_neighbors.insert(other_id);

            let other_neighbors = node_to_neighbors.entry(other_id)
                .or_insert_with(|| BTreeSet::new());
            other_neighbors.insert(one_id);

            if one_way == OneWay::Along {
                one_way_pairs.insert((one_id, other_id));
            } else if one_way == OneWay::Opposite {
                one_way_pairs.insert((other_id, one_id));
            }
        }
    }
    (node_to_neighbors, one_way_pairs)
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

fn load_preferred_neighbors(pref_neighs_path_opt: Option<&Path>) -> HashMap<NodeId, HashSet<NodeId>> {
    let pref_neighs_path = match pref_neighs_path_opt {
        Some(pnp) => pnp,
        None => return HashMap::new(),
    };
    let pref_neighs: PreferredNeighbors = {
        let mut f = File::open(pref_neighs_path)
            .expect("failed to open preferred neighbors file");
        let mut buf = Vec::new();
        f.read_to_end(&mut buf)
            .expect("failed to read preferred neighbors file");
        toml::from_slice(&buf)
            .expect("failed to parse preferred neighbors file")
    };
    pref_neighs.to_hash_map()
}

fn load_loops<'a>(
    loop_def_path: &Path,
    id_to_node: &'a HashMap<NodeId, &'a Node>,
    node_to_neighbors: &HashMap<NodeId, BTreeSet<NodeId>>,
    start_to_chain: &HashMap<(NodeId, NodeId), LengthPath<'a>>,
    one_way_pairs: &HashSet<(NodeId, NodeId)>,
    preferred_neighbors: &HashMap<NodeId, HashSet<NodeId>>,
    debug_node_ids: &HashSet<NodeId>,
    segment_directionality: bool,
) -> HashMap<(NodeId, NodeId), Loop<'a>> {
    let loop_defs: LoopDefinitions = {
        let mut f = File::open(loop_def_path)
            .expect("failed to open loop definition file");
        let mut buf = Vec::new();
        f.read_to_end(&mut buf)
            .expect("failed to read loop definition file");
        toml::from_slice(&buf)
            .expect("failed to parse loop definition file")
    };

    let mut newest_level_loops = HashMap::new();
    for level_loops in &loop_defs.levels_loops {
        let mut this_level_loops = HashMap::new();
        for (loop_name, loop_def) in level_loops.iter() {
            eprintln!("calculating loop {}", loop_name);
            let longest_loop_opt = longest_path_from(
                &id_to_node,
                &node_to_neighbors,
                &one_way_pairs,
                preferred_neighbors,
                NodeId(loop_def.start),
                Some(NodeId(loop_def.second)),
                Some(NodeId(loop_def.end)),
                start_to_chain,
                &newest_level_loops,
                &debug_node_ids,
                segment_directionality,
                None,
                0,
            );
            let longest_loop = match longest_loop_opt {
                Some(ll) => ll,
                None => panic!("no loop found for {}", loop_name),
            };

            this_level_loops.insert(
                (longest_loop.current_segments[0].start_node.id, longest_loop.current_segments[0].end_node.id),
                Loop {
                    name: loop_name.clone(),
                    length_path: longest_loop,
                },
            );
        }
        for (k, v) in this_level_loops.drain() {
            newest_level_loops.insert(k, v);
        }
    }
    newest_level_loops
}


fn main() {
    let opts = Opts::parse();
    let debug_node_ids: HashSet<NodeId> = opts.debug_node_ids.iter()
        .map(|dnid| NodeId(*dnid))
        .collect();

    let start_time = Instant::now();

    let railways = {
        let f = File::open(&opts.osm_path)
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

    if let OptsMode::DumpWays = &opts.mode {
        dump_ways(&id_to_node, &ways);
        return;
    }

    let (node_to_neighbors, one_way_pairs) = calculate_neighbors(&ways);
    eprintlntime!(start_time, "neighbors calculated");

    let preferred_neighbors = load_preferred_neighbors(
        opts.preferred_neighbors.as_ref().map(|pn| pn.as_path()),
    );

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
                &one_way_pairs,
                &preferred_neighbors,
                start_node.id,
                dest_node.id,
                &debug_node_ids,
                opts.directional_segments,
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
                &one_way_pairs,
                &preferred_neighbors,
                start_node.id,
                &stop_closest_node_ids,
                &debug_node_ids,
                opts.directional_segments,
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
        OptsMode::LongestShortestPath => {
            let stop_closest_node_ids: HashSet<NodeId> = stops.iter()
                .filter_map(|s| find_closest_node(s, way_nodes.iter().map(|n| *n)))
                .map(|n| n.id)
                .collect();

            let started_counter: AtomicUsize = AtomicUsize::new(0);
            let longest_shortest_path_opt = stop_closest_node_ids.par_iter()
                .inspect(|_| {
                    let counter = started_counter.fetch_add(1, Ordering::SeqCst);
                    eprintlntime!(start_time, "started item {}/{}", counter, stop_closest_node_ids.len());
                })
                .filter_map(|&start_node_id| {
                    let mut other_node_ids = stop_closest_node_ids.clone();
                    other_node_ids.remove(&start_node_id);

                    let mut foundlings = shortest_paths(
                        &id_to_node,
                        &node_to_neighbors,
                        &one_way_pairs,
                        &preferred_neighbors,
                        start_node_id,
                        &other_node_ids,
                        &debug_node_ids,
                        opts.directional_segments,
                    );
                    eprintlntime!(start_time, "{:?}: {} other IDs, {} paths found", start_node_id, other_node_ids.len(), foundlings.len());
                    foundlings
                        .drain()
                        .map(|(_i, p)| p)
                        .max_by(|p1, p2|
                            p1.total_distance.partial_cmp(&p2.total_distance).unwrap()
                        )
                })
                .max_by(|left, right|
                    left.total_distance.partial_cmp(&right.total_distance).unwrap()
                );

            let longest_shortest_path_detail = longest_shortest_path_opt
                .expect("no path found");
            eprintlntime!(start_time, "done! longest shortest path distance: {}", longest_shortest_path_detail.total_distance);
            path_to_geojson(longest_shortest_path_detail.current_segments.iter())
        },
        OptsMode::LongestPath(lpo) => {
            let start_id_to_node: HashMap<NodeId, &Node> = if lpo.start_node_ids.len() > 0 {
                lpo.start_node_ids.iter()
                    .filter_map(|&nid_i64| {
                        let node_id = NodeId(nid_i64);
                        if let Some(node) = id_to_node.get(&node_id) {
                            Some((node_id, *node))
                        } else {
                            None
                        }
                    })
                    .collect()
            } else {
                id_to_node.clone()
            };

            let start_to_chain = calculate_chains(
                &id_to_node,
                &node_to_neighbors,
                &one_way_pairs,
                &preferred_neighbors,
                opts.directional_segments,
            );
            eprintlntime!(start_time, "chains calculated");

            let start_to_loop: HashMap<(NodeId, NodeId), Loop> = lpo.loop_def_file
                .as_ref()
                .map(|ldf|
                    load_loops(
                        ldf.as_path(),
                        &id_to_node,
                        &node_to_neighbors,
                        &start_to_chain,
                        &one_way_pairs,
                        &preferred_neighbors,
                        &debug_node_ids,
                        opts.directional_segments,
                    )
                )
                .unwrap_or_else(|| HashMap::new());

            let longest_path_opt = start_id_to_node.keys()
                .filter_map(|&start_node_id| {
                    eprintlntime!(start_time, "calculating longest path from {:?}", start_node_id);
                    longest_path_from(
                        &id_to_node,
                        &node_to_neighbors,
                        &one_way_pairs,
                        &preferred_neighbors,
                        start_node_id,
                        None,
                        None,
                        &start_to_chain,
                        &start_to_loop,
                        &debug_node_ids,
                        opts.directional_segments,
                        lpo.progress_file.as_ref().map(|pf| pf.as_path()),
                        lpo.progress_each,
                    )
                })
                .max_by(|left, right| left.total_distance.partial_cmp(&right.total_distance).unwrap());

            let longest_path = longest_path_opt
                .expect("no path found");
            path_to_geojson(longest_path.current_segments.iter())
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
    use crate::geo::bearing_diff_rad;
    use crate::model::PathSegment;
    use super::make_fake_node;

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
