use std::collections::{BTreeSet, HashMap, HashSet};
use std::f64::consts::PI;
use std::fs::File;
use std::path::Path;

use log::debug;
use osmpbfreader::{Node, NodeId};

use crate::collections::NodePairSet;
use crate::geo::{bearing, bearing_diff_rad, sphere_distance_meters};
use crate::model::{LengthPath, Loop, PathSegment, RoutingPath};
use crate::util::path_to_geojson;


fn remove_one_way_neighbors(
    base_node: &Node,
    neighbors: &mut Vec<&Node>,
    one_way_pairs: &HashSet<(NodeId, NodeId)>,
    debug: bool,
) {
    // it is an invalid neighbor if there exists a one-way pair in the opposite direction
    neighbors.retain(
        |n| !one_way_pairs.contains(&(n.id, base_node.id))
    );
    if debug {
        let neigh_ids: Vec<i64> = neighbors.iter().map(|n| n.id.0).collect();
        debug!("  after removing against-one-ways: {:?}", neigh_ids);
    }
}

fn whittle_down_neighbors(
    base_node: &Node,
    neighbors: &mut Vec<&Node>,
    prev_node_opt: Option<&Node>,
    one_way_pairs: &HashSet<(NodeId, NodeId)>,
    preferred_neighbors: &HashMap<NodeId, HashSet<NodeId>>,
    debug: bool
) {
    const MAX_CROSSING_BEARING_DIFF_DEG: f64 = 15.0;
    const MAX_UNMARKED_BEARING_DIFF_DEG: f64 = 30.0;

    if debug { debug!("checking {}", base_node.id.0); }

    if neighbors.len() == 0 {
        if debug { debug!("  no neighbors?!"); }
        return;
    }

    if debug {
        let neigh_ids: Vec<i64> = neighbors.iter().map(|n| n.id.0).collect();
        debug!("  pre-sort neighbors: {:?}", neigh_ids);
    }
    // (stably) sort preferred neighbors last (so that they're taken first)
    if let Some(my_preferred_neighbors) = preferred_neighbors.get(&base_node.id) {
        neighbors.sort_by_cached_key(|neigh| {
            if my_preferred_neighbors.contains(&neigh.id) {
                1
            } else {
                0
            }
        });
    }
    if debug {
        let neigh_ids: Vec<i64> = neighbors.iter().map(|n| n.id.0).collect();
        debug!("  post-sort neighbors: {:?}", neigh_ids);
    }

    // allow anything if we don't know the previous node
    let prev_node = match prev_node_opt {
        Some(pn) => pn,
        None => {
            if debug { debug!("  allowing everything (except going against one-ways)"); }
            remove_one_way_neighbors(base_node, neighbors, one_way_pairs, debug);
            return;
        },
    };

    if debug { debug!("  previous: {}", prev_node.id.0); }

    if neighbors.len() == 1 {
        // dead end; nowhere to go
        if debug { debug!("  dead end"); }
        neighbors.clear();
        return;
    }

    if neighbors.len() == 2 {
        // pass-through node; remove the previous node
        neighbors.retain(|n| n.id != prev_node.id);
        if debug { debug!("  pass-through"); }
        remove_one_way_neighbors(base_node, neighbors, one_way_pairs, debug);
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

        if debug { debug!("  crossing; keeping best match"); }

        neighbors.retain(|n| n.id == bearing_neighbor.id);
        remove_one_way_neighbors(base_node, neighbors, one_way_pairs, debug);
        return;
    }

    // heuristic for switches and unmarked intersecting ways: upper limit
    if debug {
        let neigh_ids: Vec<i64> = neighbors.iter().map(|n| n.id.0).collect();
        debug!("  switch/intersection; keeping everything < {}° (before: {:?})", MAX_UNMARKED_BEARING_DIFF_DEG, neigh_ids);
    }
    neighbors.retain(|n| {
        let neigh_bearing = bearing(base_node, n);
        let bearing_diff_deg = bearing_diff_rad(prev_bearing, neigh_bearing).abs() * 180.0 / PI;
        bearing_diff_deg < MAX_UNMARKED_BEARING_DIFF_DEG
    });

    if debug {
        let neigh_ids: Vec<i64> = neighbors.iter().map(|n| n.id.0).collect();
        debug!("    after: {:?}", neigh_ids);
    }

    remove_one_way_neighbors(base_node, neighbors, one_way_pairs, debug);
}


pub fn calculate_chains<'a>(
    id_to_node: &'a HashMap<NodeId, &Node>,
    node_to_neighbors: &HashMap<NodeId, BTreeSet<NodeId>>,
    one_way_pairs: &HashSet<(NodeId, NodeId)>,
    preferred_neighbors: &HashMap<NodeId, HashSet<NodeId>>,
    directional_segments: bool,
) -> HashMap<(NodeId, NodeId), LengthPath<'a>> {
    let mut start_to_chain: HashMap<(NodeId, NodeId), LengthPath<'a>> = HashMap::new();
    for start_node in id_to_node.values() {
        let neighbor_ids = match node_to_neighbors.get(&start_node.id) {
            None => continue,
            Some(n) => n,
        };
        if neighbor_ids.len() == 0 {
            // isolated node; not interesting
            continue;
        }
        if neighbor_ids.len() == 2 {
            // pass-through node; not interesting
            continue;
        }

        let mut neighbors: Vec<&Node> = neighbor_ids.iter()
            .map(|nid| *id_to_node.get(nid).unwrap())
            .collect();

        whittle_down_neighbors(
            start_node,
            &mut neighbors,
            None,
            one_way_pairs,
            preferred_neighbors,
            false,
        );

        for neighbor in &neighbors {
            // prepare the initial path
            let init_segment = PathSegment::new_with_sphere_distance(
                start_node,
                neighbor,
            );
            let mut visited_segments = NodePairSet::new_with_directionality(directional_segments);
            visited_segments.insert(start_node.id, neighbor.id);
            let total_distance = init_segment.distance;

            let mut current_path = LengthPath {
                current_node: neighbor,
                current_segments: vec![init_segment],
                total_distance,
                visited_segments,
            };

            loop {
                let prev_node = current_path.current_segments
                    .last().unwrap()
                    .start_node;
                assert_eq!(current_path.current_segments.last().unwrap().end_node.id, current_path.current_node.id);

                // find all of the current node's neighbors
                let cur_neigh_ids = match node_to_neighbors.get(&current_path.current_node.id) {
                    Some(n) => n,
                    None => break,
                };
                if cur_neigh_ids.len() == 0 || cur_neigh_ids.len() == 1 {
                    // isolated node?! or dead-end node
                    break;
                }
                if cur_neigh_ids.len() > 2 {
                    // intersection; stop
                    break;
                }

                let mut neighbors: Vec<&Node> = cur_neigh_ids.iter()
                    .map(|nid| *id_to_node.get(nid).unwrap())
                    .collect();
                whittle_down_neighbors(
                    current_path.current_node,
                    &mut neighbors,
                    Some(prev_node),
                    one_way_pairs,
                    preferred_neighbors,
                    false,
                );

                if neighbors.len() == 0 {
                    // neighbors have been reduced to zero
                    break;
                }

                assert_eq!(neighbors.len(), 1);
                let chosen_neighbor = neighbors[0];

                let mut new_visited_segments = current_path.visited_segments.clone();
                new_visited_segments.insert(current_path.current_node.id, chosen_neighbor.id);
                let new_path = current_path.adding_segment(chosen_neighbor, new_visited_segments);
                current_path = new_path;
            }

            start_to_chain.insert(
                (
                    current_path.current_segments[0].start_node.id,
                    current_path.current_segments[0].end_node.id,
                ),
                current_path,
            );
        }
    }

    start_to_chain
}


pub fn kinda_astar_search<'a>(
    id_to_node: &'a HashMap<NodeId, &Node>,
    node_to_neighbors: &HashMap<NodeId, BTreeSet<NodeId>>,
    one_way_pairs: &HashSet<(NodeId, NodeId)>,
    preferred_neighbors: &HashMap<NodeId, HashSet<NodeId>>,
    start_node_id: NodeId,
    dest_node_id: NodeId,
    debug_node_ids: &HashSet<NodeId>,
    segment_directionality: bool,
) -> Option<RoutingPath<'a>> {
    let start_node = id_to_node.get(&start_node_id).expect("start node not found");
    let dest_node = id_to_node.get(&dest_node_id).expect("destination node not found");
    let initial_heuristic = 0.0 + sphere_distance_meters(
        start_node.lat(), start_node.lon(),
        dest_node.lat(), dest_node.lon(),
    );

    let mut paths: Vec<RoutingPath<'a>> = vec![RoutingPath {
        current_segments: vec![],
        visited_segments: NodePairSet::new_with_directionality(segment_directionality),
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
        whittle_down_neighbors(
            &path.current_node,
            &mut neighbors,
            prev_node_opt,
            one_way_pairs,
            preferred_neighbors,
            debug_node_ids.contains(&path.current_node.id),
        );
        for neighbor in &neighbors {
            let mut new_visited_segments = path.visited_segments.clone();
            if !new_visited_segments.insert(path.current_node.id, neighbor.id) {
                // we have taken this segment before
                // prevent looping infinitely
                continue;
            }

            let remain_distance_optimistic = sphere_distance_meters(
                neighbor.lat(), neighbor.lon(),
                dest_node.lat(), dest_node.lon(),
            );

            let new_path = path.adding_segment(
                neighbor,
                new_visited_segments,
                path.total_distance + remain_distance_optimistic,
            );
            paths.push(new_path);
        }

        // sort paths by heuristic, largest-first (will be taken last)
        paths.sort_unstable_by(|left, right| {
            right.heuristic.partial_cmp(&left.heuristic).unwrap()
        });
    }

    None
}

pub fn shortest_paths<'a>(
    id_to_node: &'a HashMap<NodeId, &Node>,
    node_to_neighbors: &HashMap<NodeId, BTreeSet<NodeId>>,
    one_way_pairs: &HashSet<(NodeId, NodeId)>,
    preferred_neighbors: &HashMap<NodeId, HashSet<NodeId>>,
    start_node_id: NodeId,
    dest_nodes: &HashSet<NodeId>,
    debug_node_ids: &HashSet<NodeId>,
    segment_directionality: bool,
) -> HashMap<NodeId, LengthPath<'a>> {
    let start_node = id_to_node.get(&start_node_id).expect("start node not found");
    let mut dest_node_to_path: HashMap<NodeId, LengthPath> = HashMap::new();

    let mut paths: Vec<LengthPath<'a>> = vec![LengthPath {
        current_segments: vec![],
        visited_segments: NodePairSet::new_with_directionality(segment_directionality),
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
            one_way_pairs,
            preferred_neighbors,
            debug_node_ids.contains(&path.current_node.id),
        );

        for &neighbor in &neighbors {
            // have I taken this path before?
            let mut new_visited_segments = path.visited_segments.clone();
            if !new_visited_segments.insert(path.current_node.id, neighbor.id) {
                // yes
                continue;
            }

            let new_path = path.adding_segment(
                neighbor,
                new_visited_segments,
            );
            paths.push(new_path);
        }

        // sort paths, shortest last (because pop removes the last one)
        paths.sort_unstable_by(|left, right|
            right.total_distance.partial_cmp(&left.total_distance).unwrap()
        );
    }

    dest_node_to_path
}


pub fn longest_path_from<'a>(
    id_to_node: &'a HashMap<NodeId, &Node>,
    node_to_neighbors: &HashMap<NodeId, BTreeSet<NodeId>>,
    one_way_pairs: &HashSet<(NodeId, NodeId)>,
    preferred_neighbors: &HashMap<NodeId, HashSet<NodeId>>,
    start_node_id: NodeId,
    second_node_id_opt: Option<NodeId>,
    end_node_id_opt: Option<NodeId>,
    start_to_chain: &HashMap<(NodeId, NodeId), LengthPath<'a>>,
    start_to_loop: &HashMap<(NodeId, NodeId), Loop<'a>>,
    debug_node_ids: &HashSet<NodeId>,
    segment_directionality: bool,
    progress_file_opt: Option<&Path>,
    progress_each: usize,
) -> Option<LengthPath<'a>> {
    let mut longest_path: Option<LengthPath<'a>> = None;
    let start_node = id_to_node.get(&start_node_id)
        .expect("start node not found");
    let second_node_opt = second_node_id_opt.map(|snid| {
        let sn = id_to_node.get(&snid)
            .expect("second node not found");
        let are_neighbors = node_to_neighbors
            .get(&start_node_id).expect("start node has no neighbors")
            .contains(&snid);
        if !are_neighbors {
            panic!("start and second node are not neighbors!");
        }
        sn
    });

    let initial_path = LengthPath {
        current_segments: vec![],
        visited_segments: NodePairSet::new_with_directionality(segment_directionality),
        current_node: start_node,
        total_distance: 0.0,
    };
    let mut paths: Vec<LengthPath<'a>> = vec![initial_path];

    let mut longest_distance = -1.0;
    let mut progress_counter = 0;
    let mut last_progress_length = -1.0;
    while let Some(path) = paths.pop() {
        let update_longest = end_node_id_opt
            .map(|enid| enid == path.current_node.id)
            .unwrap_or(true);
        if update_longest {
            if longest_distance < path.total_distance {
                if path.total_distance - longest_distance > 200.0 {
                    debug!("new maximum length: {}", path.total_distance);
                }
                longest_distance = path.total_distance;
                longest_path = Some(path.clone());
            }

            if end_node_id_opt.is_some() {
                // do not evaluate neighbors
                continue;
            }
        }

        if let Some(prog_file) = progress_file_opt {
            progress_counter += 1;
            if progress_counter >= progress_each {
                if let Some(lp) = &longest_path {
                    if last_progress_length < longest_distance {
                        debug!("outputting progress at {}", longest_distance);
                        let geojson = path_to_geojson(lp.current_segments.iter());
                        let f = File::create(prog_file)
                            .expect("failed to open progress file");
                        serde_json::to_writer_pretty(f, &geojson)
                            .expect("failed to write progress file");

                        last_progress_length = longest_distance;
                    }

                    progress_counter = 0;
                }
            }
        }

        // check neighborhood
        let mut neighbors: Vec<&Node> = node_to_neighbors
            .get(&path.current_node.id).unwrap()
            .iter()
            .map(|nid| *id_to_node.get(nid).unwrap())
            .collect();
        let prev_node_opt = path.current_segments.last()
            .map(|ls| ls.start_node);

        whittle_down_neighbors(
            &path.current_node,
            &mut neighbors,
            prev_node_opt,
            &one_way_pairs,
            preferred_neighbors,
            debug_node_ids.contains(&path.current_node.id),
        );

        for &neighbor in &neighbors {
            if path.current_segments.len() == 0 {
                // beginning of path
                if let Some(second_node) = second_node_opt {
                    // second node is chosen
                    if second_node.id != neighbor.id {
                        continue;
                    }
                }
            }

            // check for loop
            if let Some(lp) = start_to_loop.get(&(path.current_node.id, neighbor.id)) {
                // yes; append it if possible
                if let Some(path_with_loop) = path.extending_by_path(&lp.length_path) {
                    if debug_node_ids.contains(&path.current_node.id) {
                        debug!("appending loop {:?}", lp.name);
                    }
                    paths.push(path_with_loop);
                    continue;
                }
            }

            // check for chain
            if let Some(chain) = start_to_chain.get(&(path.current_node.id, neighbor.id)) {
                // yes; append it if possible
                if let Some(path_with_chain) = path.extending_by_path(chain) {
                    if debug_node_ids.contains(&path.current_node.id) {
                        debug!(
                            "appending chain from {:?} to {:?}",
                            chain.current_segments.first().unwrap().start_node.id,
                            chain.current_segments.last().unwrap().end_node.id,
                        );
                    }
                    paths.push(path_with_chain);
                    continue;
                }
            }

            if debug_node_ids.contains(&path.current_node.id) {
                debug!(
                    "non-chain neighbor! {:?} -> {:?}",
                    path.current_node.id, neighbor.id,
                );
            }
            let mut new_visited_segments = path.visited_segments.clone();
            if !new_visited_segments.insert(path.current_node.id, neighbor.id) {
                continue;
            }
            let new_path = path.adding_segment(
                neighbor,
                new_visited_segments,
            );
            paths.push(new_path);
        }
    }

    longest_path
}
