// QuadtreeNode.cs

namespace QuadTreeHPAStar;

public class QuadtreeNode {
    private readonly int _maxLevel;
    private readonly int _minSize;

    public QuadtreeNode(Bounds bounds, int level, int maxLevel, int minSize, QuadtreeNode? parent = null) {
        Bounds = bounds;
        Level = level;
        _maxLevel = maxLevel;
        _minSize = minSize;
        Parent = parent;
    }

    public int Id { get; set; } = -1;
    public Bounds Bounds { get; }
    public int Level { get; }
    public QuadtreeNode? Parent { get; }
    public QuadtreeNode[]? Children { get; private set; }
    public bool IsLeaf { get; private set; } = true;
    public bool? IsWalkable { get; private set; }

    public NeighborList Neighbors { get; } = new();

    public void Subdivide(Map map) {
        bool? regionState = map.IsRegionUniform(Bounds);
        if (regionState != null) {
            IsWalkable = regionState.Value;
            return;
        }

        if (Level >= _maxLevel || Bounds.Width / 2 < _minSize || Bounds.Height / 2 < _minSize) {
            IsWalkable = false; // Mixed region that can't be subdivided further is treated as unwalkable cluster
            return;
        }

        IsLeaf = false;
        (int x, int y, int width, int height) = Bounds;
        int halfWidth = width / 2;
        int halfHeight = height / 2;

        Bounds[] childBounds = new Bounds[] {
            new(x, y, halfWidth, halfHeight), // NW
            new(x + halfWidth, y, width - halfWidth, halfHeight), // NE
            new(x, y + halfHeight, halfWidth, height - halfHeight), // SW
            new(x + halfWidth, y + halfHeight, width - halfWidth, height - halfHeight) // SE
        };

        Children = new QuadtreeNode[4];
        for (int i = 0; i < 4; i++) {
            Children[i] = new QuadtreeNode(childBounds[i], Level + 1, _maxLevel, _minSize, this);
            Children[i].Subdivide(map);
        }
    }

    public List<QuadtreeNode> GetLeafNodes() {
        List<QuadtreeNode> leaves = new();
        Stack<QuadtreeNode> stack = new();
        stack.Push(this);
        while (stack.Count > 0) {
            var node = stack.Pop();
            if (node.IsLeaf)
                leaves.Add(node);
            else if (node.Children != null)
                foreach (var child in node.Children)
                    stack.Push(child);
        }

        return leaves;
    }

    public QuadtreeNode? GetLeafForPoint(Point point) {
        if (!Bounds.Contains(point)) return null;

        var node = this;
        while (!node.IsLeaf) {
            int childIndex = node.GetChildIndexForPoint(point);
            if (childIndex == -1 || node.Children == null) return null; // Should not happen if bounds check passed
            node = node.Children[childIndex];
        }

        return node;
    }

    private int GetChildIndexForPoint(Point p) {
        int midX = Bounds.X + Bounds.Width / 2;
        int midY = Bounds.Y + Bounds.Height / 2;
        bool isTop = p.Y < midY;
        bool isLeft = p.X < midX;
        if (isTop) return isLeft ? 0 : 1; // 0: NW, 1: NE
        return isLeft ? 2 : 3; // 2: SW, 3: SE
    }

    /// <summary>
    ///     Finds and stores neighbors for all leaf nodes in the tree.
    ///     This is the new, efficient entry point for neighbor finding.
    /// </summary>
    public void FindAndStoreNeighbors() {
        List<QuadtreeNode> leaves = GetLeafNodes();
        foreach (var leaf in leaves) {
            // Find neighbors in all 4 directions
            List<QuadtreeNode> northNeighbors = GetAdjacentLeaves(leaf, Direction.N);
            List<QuadtreeNode> eastNeighbors = GetAdjacentLeaves(leaf, Direction.E);
            List<QuadtreeNode> southNeighbors = GetAdjacentLeaves(leaf, Direction.S);
            List<QuadtreeNode> westNeighbors = GetAdjacentLeaves(leaf, Direction.W);

            // Add neighbors and establish bidirectional links
            foreach (var neighbor in northNeighbors) {
                leaf.Neighbors.N.Add(neighbor);
                neighbor.Neighbors.S.Add(leaf);
            }

            foreach (var neighbor in eastNeighbors) {
                leaf.Neighbors.E.Add(neighbor);
                neighbor.Neighbors.W.Add(leaf);
            }

            foreach (var neighbor in southNeighbors) {
                leaf.Neighbors.S.Add(neighbor);
                neighbor.Neighbors.N.Add(leaf);
            }

            foreach (var neighbor in westNeighbors) {
                leaf.Neighbors.W.Add(neighbor);
                neighbor.Neighbors.E.Add(leaf);
            }
        }
    }

    /// <summary>
    ///     Gets adjacent leaf nodes in a specified direction using efficient tree traversal.
    /// </summary>
    private static List<QuadtreeNode> GetAdjacentLeaves(QuadtreeNode sourceNode, Direction direction) {
        List<QuadtreeNode> neighbors = new();
        if (sourceNode.Parent == null) return neighbors; // Root node has no neighbors

        // Determine which children are on which border
        (int pIdx1, int pIdx2, int cIdx1, int cIdx2) = direction switch {
            Direction.N => (2, 3, 0, 1), // Parent's South children border its North children
            Direction.S => (0, 1, 2, 3), // Parent's North children border its South children
            Direction.W => (1, 3, 0, 2), // Parent's East children border its West children
            Direction.E => (0, 2, 1, 3), // Parent's West children border its East children
            _ => throw new ArgumentOutOfRangeException(nameof(direction))
        };

        var currentNode = sourceNode;
        while (currentNode.Parent != null) {
            int childIndex = Array.IndexOf(currentNode.Parent.Children!, currentNode);

            // If the current node is on the "inner" side of the parent, its neighbor is a sibling.
            if (childIndex == pIdx1 || childIndex == pIdx2) {
                var adjacentSibling = currentNode.Parent.Children![childIndex == pIdx1 ? cIdx1 : cIdx2];
                FindLeavesOnBorder(adjacentSibling, direction, sourceNode.Bounds, neighbors);
                break;
            }

            // Otherwise, move up the tree
            currentNode = currentNode.Parent;
        }

        return neighbors;
    }

    /// <summary>
    ///     Recursively traverses down a branch to find all leaf nodes on a specific border
    ///     that intersect with the source node's bounds.
    /// </summary>
    private static void FindLeavesOnBorder(QuadtreeNode currentNode, Direction direction, Bounds sourceBounds,
        List<QuadtreeNode> neighbors) {
        if (currentNode.IsLeaf) {
            neighbors.Add(currentNode);
            return;
        }

        // Determine which children to explore based on the border we're interested in
        (int c1, int c2) = direction switch {
            Direction.N => (2, 3), // To find North neighbors, look at the South border of the adjacent node
            Direction.S => (0, 1), // To find South neighbors, look at the North border of the adjacent node
            Direction.W => (1, 3), // To find West neighbors, look at the East border of the adjacent node
            Direction.E => (0, 2), // To find East neighbors, look at the West border of the adjacent node
            _ => throw new ArgumentOutOfRangeException(nameof(direction))
        };

        // Recurse into the relevant children that overlap with the source node's bounds
        var child1 = currentNode.Children![c1];
        if (BoundsOverlap(child1.Bounds, sourceBounds, direction))
            FindLeavesOnBorder(child1, direction, sourceBounds, neighbors);

        var child2 = currentNode.Children![c2];
        if (BoundsOverlap(child2.Bounds, sourceBounds, direction))
            FindLeavesOnBorder(child2, direction, sourceBounds, neighbors);
    }

    /// <summary>
    ///     Checks if two bounds overlap on the axis perpendicular to the search direction.
    /// </summary>
    private static bool BoundsOverlap(Bounds b1, Bounds b2, Direction direction) {
        if (direction == Direction.N || direction == Direction.S)
            // Check for horizontal overlap
            return b1.X < b2.X + b2.Width && b1.X + b1.Width > b2.X;

        // E or W
        // Check for vertical overlap
        return b1.Y < b2.Y + b2.Height && b1.Y + b1.Height > b2.Y;
    }

    public class NeighborList {
        public List<QuadtreeNode> N { get; } = new();
        public List<QuadtreeNode> E { get; } = new();
        public List<QuadtreeNode> S { get; } = new();
        public List<QuadtreeNode> W { get; } = new();
    }

    private enum Direction {
        N,
        E,
        S,
        W
    }
}

// Add this helper extension to Bounds record for convenience