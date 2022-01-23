use std::collections::{HashMap, HashSet};

use osmpbfreader::{Node, NodeId};
use serde::{Deserialize, Serialize};

use crate::collections::NodePairSet;
use crate::geo::{bearing, sphere_distance_meters};


#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct LoopDefinitions {
    pub levels_loops: Vec<HashMap<String, LoopDefinition>>,
}
#[derive(Clone, Debug, Deserialize, Eq, Hash, PartialEq, Serialize)]
pub struct LoopDefinition {
    pub start: i64,
    pub second: i64,
    pub end: i64,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct PreferredNeighbors {
    pub preferred_neighbors: HashSet<PreferredNeighbor>,
}
impl PreferredNeighbors {
    pub fn to_hash_map(&self) -> HashMap<NodeId, HashSet<NodeId>> {
        let mut ret = HashMap::new();
        for pair in &self.preferred_neighbors {
            ret
                .entry(NodeId(pair.base))
                .or_insert_with(|| HashSet::new())
                .insert(NodeId(pair.neighbor));
        }
        ret
    }
}
#[derive(Clone, Debug, Deserialize, Eq, Hash, PartialEq, Serialize)]
pub struct PreferredNeighbor {
    pub base: i64,
    pub neighbor: i64,
}


#[derive(Clone, Debug, PartialEq)]
pub struct PathSegment<'a> {
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

    #[allow(unused)]
    pub fn bearing(&self) -> f64 {
        bearing(self.start_node, self.end_node)
    }
}


#[derive(Clone, Debug, PartialEq)]
pub struct RoutingPath<'a> {
    pub current_segments: Vec<PathSegment<'a>>,
    pub visited_segments: NodePairSet,
    pub current_node: &'a Node,
    pub total_distance: f64,
    pub heuristic: f64,
}
impl<'a> RoutingPath<'a> {
    #[allow(unused)]
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

    #[allow(unused)]
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

    pub fn adding_segment(
        &self,
        new_node: &'a Node,
        new_visited_segments: NodePairSet,
        new_heuristic: f64,
    ) -> Self {
        let new_seg = PathSegment::new_with_sphere_distance(
            self.current_node,
            new_node,
        );
        let new_total_distance = self.total_distance + new_seg.distance;
        let mut new_current_segments = self.current_segments.clone();
        new_current_segments.push(new_seg);
        Self {
            current_segments: new_current_segments,
            visited_segments: new_visited_segments,
            current_node: new_node,
            total_distance: new_total_distance,
            heuristic: new_heuristic,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Loop<'a> {
    pub name: String,
    pub length_path: LengthPath<'a>,
}


#[derive(Clone, Debug, PartialEq)]
pub struct LengthPath<'a> {
    pub current_segments: Vec<PathSegment<'a>>,
    pub visited_segments: NodePairSet,
    pub current_node: &'a Node,
    pub total_distance: f64,
}
impl<'a> LengthPath<'a> {
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

    pub fn adding_segment(
        &self,
        new_node: &'a Node,
        new_visited_segments: NodePairSet,
    ) -> Self {
        let new_seg = PathSegment::new_with_sphere_distance(
            self.current_node,
            new_node,
        );
        let new_total_distance = self.total_distance + new_seg.distance;
        let mut new_current_segments = self.current_segments.clone();
        new_current_segments.push(new_seg);
        Self {
            current_segments: new_current_segments,
            visited_segments: new_visited_segments,
            current_node: new_node,
            total_distance: new_total_distance,
        }
    }

    pub fn extending_by_path(
        &self,
        path_to_append: &Self,
    ) -> Option<Self> {
        if let Some(first_segment_to_append) = path_to_append.current_segments.first() {
            if first_segment_to_append.start_node.id != self.current_node.id {
                // not valid to append this segment
                return None;
            }
        }

        // calculate new visited segments
        let mut new_visited_segments = self.visited_segments.clone();
        for (n1, n2) in path_to_append.visited_segments.iter() {
            if !new_visited_segments.insert(n1, n2) {
                // cannot extend by this path; segments have already been visited
                return None;
            }
        }

        let mut new_current_segments = self.current_segments.clone();
        new_current_segments.extend_from_slice(&path_to_append.current_segments);

        Some(Self {
            current_node: path_to_append.current_node,
            current_segments: new_current_segments,
            total_distance: self.total_distance + path_to_append.total_distance,
            visited_segments: new_visited_segments,
        })
    }
}
