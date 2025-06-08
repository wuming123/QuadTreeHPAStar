//HPAStar.cs

using System.Collections.Concurrent;
using System.Diagnostics;

namespace QuadTreeHPAStar;

// Records are fine
public record HighLevelEdge(int NeighborId, int Cost, List<Point>? Path = null);

public record CameFromLink(int FromId, List<Point>? Path); // Path is the segment leading TO current node FROM FromId

public class HPAStar {
    private readonly Dictionary<int, QuadtreeNode> _clusters = new();
    private readonly Dictionary<Point, int> _entrancePointToId = new();

    private readonly Dictionary<int, List<Point>>
        _entrances = new(); // Stores Point objects for each cluster's entrances

    private readonly object _graphLock = new(); // Used for _entrances, _entrancePointToId, _nextEntranceId

    private readonly Dictionary<int, Dictionary<Point, Dictionary<Point, List<Point>>>> _intraClusterPaths = new();
    private readonly Map _map;
    private readonly int _maxLevel;
    private readonly int _minClusterSize;
    private Point[] _entranceIdToPoint = Array.Empty<Point>();

    private List<HighLevelEdge>[] _highLevelGraphInt = Array.Empty<List<HighLevelEdge>>();
    private int _nextEntranceId;
    private QuadtreeNode? _quadtree;

    public HPAStar(Map map, int maxLevel = 4, int minClusterSize = 10) {
        _map = map;
        _maxLevel = maxLevel;
        _minClusterSize = minClusterSize;
    }

    public void Precompute() {
        Console.WriteLine($"Starting HPA* precomputation on {Environment.ProcessorCount} cores...");
        var stopwatch = Stopwatch.StartNew();

        BuildClusters();

        // Data to be collected from parallel tasks
        var discoveredInterClusterEntrances = new ConcurrentBag<(Point p1, Point p2, int c1Id, int c2Id)>();
        var discoveredIntraClusterPaths =
            new ConcurrentBag<(int clusterId, Point from, Point to, int cost, List<Point> path)>();

        Console.WriteLine("- Discovering entrances and calculating intra-cluster paths in parallel...");
        Parallel.ForEach(_clusters.Values, cluster => {
            // Find inter-cluster entrances (bordering other clusters)
            var neighbors = cluster.Neighbors.N.Concat(cluster.Neighbors.E).Concat(cluster.Neighbors.S)
                .Concat(cluster.Neighbors.W).Distinct();
            foreach (var neighbor in neighbors) {
                if (cluster.Id >= neighbor.Id) continue; // Process each pair once
                CollectInterClusterEntrances(cluster, neighbor, discoveredInterClusterEntrances);
            }
        });

        // --- STAGE 1: Finalize all entrance points and their IDs ---
        // Process all discovered inter-cluster entrances to populate _entrances and _entrancePointToId
        foreach (var (p1, p2, c1Id, c2Id) in discoveredInterClusterEntrances) {
            lock (_graphLock) {
                AddEntranceToCluster(c1Id, p1);
                AddEntranceToCluster(c2Id, p2);
                GetOrAssignEntranceId(p1); // Ensure IDs are assigned
                GetOrAssignEntranceId(p2);
            }
        }

        // --- STAGE 2: Calculate intra-cluster paths now that all entrances are known ---
        Parallel.ForEach(_clusters.Values, cluster => {
            if (!_entrances.TryGetValue(cluster.Id, out var clusterEntrances) || clusterEntrances.Count < 2) return;

            Dictionary<Point, Dictionary<Point, List<Point>>> localClusterPathsTemp = new();
            foreach (var startEntrance in clusterEntrances) {
                // Create a set of goals: all other entrances in this cluster
                var goalEntrances = new HashSet<Point>(clusterEntrances);
                goalEntrances.Remove(startEntrance);

                if (goalEntrances.Count == 0) continue;

                // Run ONE search from startEntrance to all other entrances
                var paths = AStar.FindPathsToMultipleGoals(_map, startEntrance, goalEntrances, cluster.Bounds);

                // Add all found paths to our collections
                foreach (var (endEntrance, data) in paths) {
                    // data.path is from startEntrance to endEntrance
                    discoveredIntraClusterPaths.Add((cluster.Id, startEntrance, endEntrance, data.cost, data.path));

                    // For reconstruction lookup
                    if (!localClusterPathsTemp.ContainsKey(startEntrance))
                        localClusterPathsTemp[startEntrance] = new Dictionary<Point, List<Point>>();
                    localClusterPathsTemp[startEntrance][endEntrance] = data.path;
                }
            }

            if (localClusterPathsTemp.Count > 0) {
                lock (_graphLock) // _intraClusterPaths needs locking
                {
                    _intraClusterPaths[cluster.Id] = localClusterPathsTemp;
                }
            }
        });


        Console.WriteLine("- Building final high-level graph...");
        // --- STAGE 3: Build the integer-keyed high-level graph ---
        _highLevelGraphInt = new List<HighLevelEdge>[_nextEntranceId];
        _entranceIdToPoint = new Point[_nextEntranceId];
        for (int i = 0; i < _nextEntranceId; i++) _highLevelGraphInt[i] = new List<HighLevelEdge>();

        // Populate _entranceIdToPoint (needs to be done after all GetOrAssignEntranceId calls)
        // This was potentially problematic if GetOrAssignEntranceId was called after array init.
        // Now it's safer.
        foreach (var (p, id) in _entrancePointToId) {
            if (id < _entranceIdToPoint.Length) _entranceIdToPoint[id] = p;
            else {
                /* This would indicate an issue with _nextEntranceId logic */
            }
        }


        // Add inter-cluster edges (cost 1, no precomputed path list)
        foreach (var (p1, p2, _, _) in discoveredInterClusterEntrances) {
            int id1 = _entrancePointToId[p1];
            int id2 = _entrancePointToId[p2];
            AddHighLevelEdge(id1, id2, 1, null); // Path is null for inter-cluster
            AddHighLevelEdge(id2, id1, 1, null); // Path is null for inter-cluster
        }

        // Add intra-cluster edges (cost from A*, has precomputed path list)
        foreach (var (_, from, to, cost, path) in discoveredIntraClusterPaths) {
            int fromId = _entrancePointToId[from];
            int toId = _entrancePointToId[to];
            AddHighLevelEdge(fromId, toId, cost, path);

            var reversedPath = new List<Point>(path); // Create a new list for the reversed path
            reversedPath.Reverse();
            AddHighLevelEdge(toId, fromId, cost, reversedPath);
        }

        stopwatch.Stop();
        Console.WriteLine(
            $"Precomputation finished in {stopwatch.ElapsedMilliseconds} ms with {_nextEntranceId} entrances.");
    }

    private void BuildClusters() {
        _quadtree = new QuadtreeNode(new Bounds(0, 0, _map.Width, _map.Height), 0, _maxLevel, _minClusterSize);
        _quadtree.Subdivide(_map);
        var leaves = _quadtree.GetLeafNodes();
        int id = 0;
        foreach (var leaf in leaves) {
            leaf.Id = id;
            _clusters[id] = leaf;
            id++;
        }

        _quadtree.FindAndStoreNeighbors();
        Console.WriteLine($"- Built Quadtree with {_clusters.Count} clusters.");
    }

    // Renamed from CreateEntrancesAndEdges to be more specific
    private void CollectInterClusterEntrances(QuadtreeNode c1, QuadtreeNode c2,
        ConcurrentBag<(Point p1, Point p2, int c1Id, int c2Id)> interClusterEntrances) {
        var borderInfo = GetBorderInfo(c1.Bounds, c2.Bounds);
        if (borderInfo == null) return;

        var (orientation, p1StaticCoord, p2StaticCoord, start, end) = borderInfo;
        int currentSegmentStart = -1;
        for (int k = start; k <= end; k++) // Iterate along the shared border
        {
            Point p1 = orientation == 'v' ? new Point(p1StaticCoord, k) : new Point(k, p1StaticCoord);
            Point p2 = orientation == 'v' ? new Point(p2StaticCoord, k) : new Point(k, p2StaticCoord);
            bool isWalkablePair = _map.IsWalkable(p1) && _map.IsWalkable(p2);

            if (isWalkablePair && currentSegmentStart == -1) {
                currentSegmentStart = k;
            }

            if ((!isWalkablePair || k == end) && currentSegmentStart != -1) {
                int segmentEnd = isWalkablePair ? k : k - 1;
                int midPointCoord = currentSegmentStart + (segmentEnd - currentSegmentStart) / 2;
                Point eP1 = orientation == 'v'
                    ? new Point(p1StaticCoord, midPointCoord)
                    : new Point(midPointCoord, p1StaticCoord);
                Point eP2 = orientation == 'v'
                    ? new Point(p2StaticCoord, midPointCoord)
                    : new Point(midPointCoord, p2StaticCoord);

                interClusterEntrances.Add((eP1, eP2, c1.Id, c2.Id));
                currentSegmentStart = -1;
            }
        }
    }

    private BorderInfo? GetBorderInfo(Bounds b1, Bounds b2) // This logic seems fine
    {
        if (b1.X + b1.Width == b2.X) {
            int yStart = Math.Max(b1.Y, b2.Y);
            int yEnd = Math.Min(b1.Y + b1.Height, b2.Y + b2.Height);
            if (yStart < yEnd) return new BorderInfo('v', b1.X + b1.Width - 1, b2.X, yStart, yEnd - 1);
        }

        if (b2.X + b2.Width == b1.X) {
            int yStart = Math.Max(b1.Y, b2.Y);
            int yEnd = Math.Min(b1.Y + b1.Height, b2.Y + b2.Height);
            if (yStart < yEnd) return new BorderInfo('v', b1.X, b2.X + b2.Width - 1, yStart, yEnd - 1);
        } // Corrected: b1.X should be p1StaticCoord

        if (b1.Y + b1.Height == b2.Y) {
            int xStart = Math.Max(b1.X, b2.X);
            int xEnd = Math.Min(b1.X + b1.Width, b2.X + b2.Width);
            if (xStart < xEnd) return new BorderInfo('h', b1.Y + b1.Height - 1, b2.Y, xStart, xEnd - 1);
        }

        if (b2.Y + b2.Height == b1.Y) {
            int xStart = Math.Max(b1.X, b2.X);
            int xEnd = Math.Min(b1.X + b1.Width, b2.X + b2.Width);
            if (xStart < xEnd) return new BorderInfo('h', b1.Y, b2.Y + b2.Height - 1, xStart, xEnd - 1);
        } // Corrected: b1.Y should be p1StaticCoord

        return null;
    }

    // Renamed for clarity
    private void AddEntranceToCluster(int clusterId, Point point) {
        // This method is called under _graphLock in Precompute
        if (!_entrances.TryGetValue(clusterId, out var points)) {
            points = new List<Point>();
            _entrances[clusterId] = points;
        }

        if (!points.Contains(point)) {
            points.Add(point);
        }
    }

    private int GetOrAssignEntranceId(Point point) {
        // This method is called under _graphLock in Precompute
        if (!_entrancePointToId.TryGetValue(point, out int id)) // Use TryGetValue for efficiency
        {
            id = _nextEntranceId++;
            _entrancePointToId[point] = id;
        }

        return id;
    }

    private void AddHighLevelEdge(int fromId, int toId, int cost, List<Point>? path) {
        // Assumes _highLevelGraphInt is initialized. Called after _nextEntranceId is final.
        // No lock needed here if called sequentially during graph finalization.
        if (fromId < _highLevelGraphInt.Length && toId < _highLevelGraphInt.Length) {
            // Ensure we don't add duplicate edges if (A,B) and (B,A) are processed separately with same path object
            if (!_highLevelGraphInt[fromId].Any(e => e.NeighborId == toId && e.Cost == cost)) {
                _highLevelGraphInt[fromId].Add(new HighLevelEdge(toId, cost, path));
            }
        }
    }

    public List<Point>? FindPath(Point start, Point end) {
        if (_quadtree == null) throw new InvalidOperationException("Precomputation must be run before finding a path.");

        var startCluster = _quadtree.GetLeafForPoint(start);
        var endCluster = _quadtree.GetLeafForPoint(end);
        if (startCluster == null || endCluster == null) return null;

        if (startCluster.Id == endCluster.Id) return AStar.FindPath(_map, start, end, startCluster.Bounds);

        const int START_NODE_ID = -1; // Special ID for the conceptual start node in cameFrom
        int totalEntrances = _nextEntranceId;
        if (totalEntrances == 0) return AStar.FindPath(_map, start, end); // Fallback if no entrances (e.g. tiny map)


        var openSet = new PriorityQueue<int, int>();
        var gScore = new int[totalEntrances];
        Array.Fill(gScore, int.MaxValue);
        // cameFrom[i] stores the link (previous entrance ID and path segment) to reach entrance 'i'
        var cameFrom = new CameFromLink?[totalEntrances]; // Use nullable CameFromLink

        var endPaths =
            new Dictionary<int, (int cost, List<Point> path)>(); // Key: entranceId, Value: (cost from entrance to actual end, path from entrance to actual end)

        // --- CONNECT END NODE TO ITS CLUSTER ENTRANCES ---
        if (_entrances.TryGetValue(endCluster.Id, out var endClusterEntrances) && endClusterEntrances.Count > 0) {
            var pathsToEndNode =
                AStar.FindPathsToMultipleGoals(_map, end, new HashSet<Point>(endClusterEntrances), endCluster.Bounds);
            foreach (var (entrancePoint, data) in pathsToEndNode) {
                if (_entrancePointToId.TryGetValue(entrancePoint, out int entranceId)) {
                    // Path from AStar.FindPathsToMultipleGoals is from 'end' to 'entrancePoint'. We need 'entrancePoint' to 'end'.
                    var pathFromEntranceToEnd = new List<Point>(data.path);
                    pathFromEntranceToEnd.Reverse(); // Now it's from entrancePoint to end
                    endPaths[entranceId] = (data.cost, pathFromEntranceToEnd);
                }
            }
        }

        // If end point is not walkable and no entrances connect to it, no path.
        if (endPaths.Count == 0 && !_map.IsWalkable(end)) return null;


        // --- HEURISTIC CALCULATION (from any entrance 'i' to the actual 'end' point) ---
        var heuristicCache = new int[totalEntrances];
        for (int i = 0; i < totalEntrances; i++) {
            Point entrance_i_Point = _entranceIdToPoint[i];
            int minH = Heuristic(entrance_i_Point, end); // Direct heuristic to end

            foreach (var (targetEntranceId, data) in endPaths) // data.cost is cost from targetEntranceId to end
            {
                Point targetEntrancePoint = _entranceIdToPoint[targetEntranceId];
                int h = Heuristic(entrance_i_Point, targetEntrancePoint) + data.cost;
                if (h < minH) minH = h;
            }

            heuristicCache[i] = minH;
        }

        // --- CONNECT START NODE TO ITS CLUSTER ENTRANCES ---
        if (_entrances.TryGetValue(startCluster.Id, out var startClusterEntrances) && startClusterEntrances.Count > 0) {
            // Paths from 'start' to each 'entrancePoint' in its cluster
            var pathsFromStartNode = AStar.FindPathsToMultipleGoals(_map, start,
                new HashSet<Point>(startClusterEntrances), startCluster.Bounds);
            foreach (var (entrancePoint, data) in pathsFromStartNode) // data.path is from 'start' to 'entrancePoint'
            {
                if (_entrancePointToId.TryGetValue(entrancePoint, out int entranceId)) {
                    gScore[entranceId] = data.cost;
                    cameFrom[entranceId] =
                        new CameFromLink(START_NODE_ID, data.path); // Path is from start to this entrance
                    openSet.Enqueue(entranceId, data.cost + heuristicCache[entranceId]);
                }
            }
        }

        // --- MAIN A* LOOP (ON INTEGER IDs) ---
        int? pathFoundViaEntranceId = null;

        while (openSet.TryDequeue(out var currentEntranceId, out _)) {
            if (endPaths.ContainsKey(currentEntranceId)) {
                pathFoundViaEntranceId = currentEntranceId;
                break; // Goal reached
            }

            if (currentEntranceId >= _highLevelGraphInt.Length ||
                _highLevelGraphInt[currentEntranceId] == null) continue;

            foreach (var edge in _highLevelGraphInt
                         [currentEntranceId]) // edge.Path is from currentEntranceId to edge.NeighborId
            {
                int neighborId = edge.NeighborId;
                int tentativeGScore = gScore[currentEntranceId] + edge.Cost;

                if (tentativeGScore < gScore[neighborId]) {
                    gScore[neighborId] = tentativeGScore;
                    int fScore = tentativeGScore + heuristicCache[neighborId];
                    openSet.Enqueue(neighborId, fScore);
                    cameFrom[neighborId] = new CameFromLink(currentEntranceId, edge.Path);
                }
            }
        }

        if (pathFoundViaEntranceId.HasValue) {
            return ReconstructPathInt(cameFrom, endPaths[pathFoundViaEntranceId.Value].path,
                pathFoundViaEntranceId.Value, start);
        }

        // Fallback: if start/end are in different clusters but no HPA path found (e.g. no entrances)
        // This might happen if clusters are too large or map is very sparse.
        // A direct A* might still find a path if one exists, though it defeats HPA*'s purpose for this query.
        // For now, we'll return null as HPA* itself didn't find it.
        return null;
    }

    private List<Point> ReconstructPathInt(CameFromLink?[] cameFrom, List<Point> finalSegment, int lastEntranceId,
        Point actualStart) {
        const int START_NODE_ID = -1;
        int totalPoints = 0;

        // ======================= OPTIMIZATION START =======================

        // --- PASS 1: Calculate the exact size needed for the final path array ---
        totalPoints += finalSegment.Count;
        int currentId = lastEntranceId;

        while (currentId != START_NODE_ID) {
            var link = cameFrom[currentId]!;
            int fromId = link.FromId;

            if (link.Path != null) {
                // We subtract 1 because the start of this segment is the end of the previous one.
                totalPoints += link.Path.Count - 1;
            }
            else // This is an inter-cluster edge with no pre-calculated path. It's just two points.
            {
                // fromPoint -> currentPoint. We only add the 'from' point.
                totalPoints += 1; // Equivalent to (2 - 1)
            }

            currentId = fromId;
        }

        if (totalPoints == 0) return new List<Point>();

        // --- PASS 2: Allocate a single array and fill it from back to front ---
        var finalPath = new Point[totalPoints];
        int writeIndex = totalPoints - 1;

        // First, copy the last segment (from the last entrance to the actual end point)
        for (int i = finalSegment.Count - 1; i >= 0; i--) {
            finalPath[writeIndex--] = finalSegment[i];
        }

        // Now, walk the cameFrom chain backwards, filling the array
        currentId = lastEntranceId;
        while (currentId != START_NODE_ID) {
            var link = cameFrom[currentId]!;
            var segment = link.Path;
            int fromId = link.FromId;

            if (segment != null) {
                // Copy the segment, but skip the last point because it's already in the path
                // (it was the 'currentPoint' of the previous iteration).
                for (int i = segment.Count - 2; i >= 0; i--) {
                    finalPath[writeIndex--] = segment[i];
                }
            }
            else // Inter-cluster edge, path is just [fromPoint, currentPoint]
            {
                Point fromPoint = fromId == START_NODE_ID ? actualStart : _entranceIdToPoint[fromId];
                finalPath[writeIndex--] = fromPoint;
            }

            currentId = fromId;
        }

        // ======================== OPTIMIZATION END ========================

        return new List<Point>(finalPath); // Convert the final array to a List for the public API.
        // This is the only significant allocation.
    }

    private static int Heuristic(Point a, Point b) {
        return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
    }

    private record BorderInfo(char Orientation, int P1StaticCoord, int P2StaticCoord, int Start, int End);
}