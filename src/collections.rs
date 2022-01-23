use std::collections::HashSet;
use std::collections::hash_set::Iter as HashSetIter;

use osmpbfreader::NodeId;


#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct OrderedNodePair {
    first_node: NodeId,
    second_node: NodeId,
}
impl OrderedNodePair {
    pub fn new(
        first_node: NodeId,
        second_node: NodeId,
    ) -> Self {
        Self {
            first_node,
            second_node,
        }
    }

    #[allow(unused)]
    pub fn first_node(&self) -> NodeId { self.first_node }

    #[allow(unused)]
    pub fn second_node(&self) -> NodeId { self.second_node }

    #[inline]
    pub fn as_node_pair(&self) -> (NodeId, NodeId) { (self.first_node, self.second_node) }
}
impl From<(NodeId, NodeId)> for OrderedNodePair {
    fn from(pair: (NodeId, NodeId)) -> Self {
        Self::new(pair.0, pair.1)
    }
}


#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct UnorderedNodePair {
    pair: OrderedNodePair,
}
impl UnorderedNodePair {
    pub fn new(
        one_node: NodeId,
        other_node: NodeId,
    ) -> Self {
        let pair = if one_node <= other_node {
            OrderedNodePair::new(one_node, other_node)
        } else {
            OrderedNodePair::new(other_node, one_node)
        };

        Self {
            pair,
        }
    }

    #[allow(unused)]
    pub fn smaller_node(&self) -> NodeId { self.pair.first_node() }

    #[allow(unused)]
    pub fn greater_node(&self) -> NodeId { self.pair.second_node() }

    #[inline]
    pub fn as_node_pair(&self) -> (NodeId, NodeId) { self.pair.as_node_pair() }
}
impl From<(NodeId, NodeId)> for UnorderedNodePair {
    fn from(pair: (NodeId, NodeId)) -> Self {
        Self::new(pair.0, pair.1)
    }
}


#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NodePairSet {
    Ordered(HashSet<OrderedNodePair>),
    Unordered(HashSet<UnorderedNodePair>),
}
impl NodePairSet {
    pub fn new_ordered() -> Self {
        Self::Ordered(HashSet::new())
    }
    pub fn new_unordered() -> Self {
        Self::Unordered(HashSet::new())
    }
    pub fn new_with_directionality(directional: bool) -> Self {
        if directional {
            Self::new_ordered()
        } else {
            Self::new_unordered()
        }
    }

    /// Returns `true` if this node pair has been added to the set or `false` if it was already contained before.
    pub fn insert(&mut self, node1: NodeId, node2: NodeId) -> bool {
        // true if it's a new entry
        // false if it's already contained
        match self {
            Self::Ordered(os) => os.insert(OrderedNodePair::new(node1, node2)),
            Self::Unordered(us) => us.insert(UnorderedNodePair::new(node1, node2)),
        }
    }

    #[allow(unused)]
    pub fn contains(&self, node1: NodeId, node2: NodeId) -> bool {
        match self {
            Self::Ordered(os) => os.contains(&OrderedNodePair::new(node1, node2)),
            Self::Unordered(us) => us.contains(&UnorderedNodePair::new(node1, node2)),
        }
    }

    pub fn iter(&self) -> NodePairSetIter {
        match self {
            Self::Ordered(os) => NodePairSetIter::Ordered(os.iter()),
            Self::Unordered(os) => NodePairSetIter::Unordered(os.iter()),
        }
    }
}
pub enum NodePairSetIter<'a> {
    Ordered(HashSetIter<'a, OrderedNodePair>),
    Unordered(HashSetIter<'a, UnorderedNodePair>),
}
impl<'a> Iterator for NodePairSetIter<'a> {
    type Item = (NodeId, NodeId);

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Ordered(oi) => oi.next()
                .map(|np| np.as_node_pair()),
            Self::Unordered(ui) => ui.next()
                .map(|np| np.as_node_pair()),
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        match self {
            Self::Ordered(oi) => oi.size_hint(),
            Self::Unordered(ui) => ui.size_hint(),
        }
    }
}
impl<'a> ExactSizeIterator for NodePairSetIter<'a> {
    fn len(&self) -> usize {
        match self {
            Self::Ordered(oi) => oi.len(),
            Self::Unordered(ui) => ui.len(),
        }
    }
}
