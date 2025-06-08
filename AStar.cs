//AStar.cs

using System.Buffers;

namespace QuadTreeHPAStar;

public static class AStar {
    private static readonly Point[] Directions = { new(1, 0), new(-1, 0), new(0, 1), new(0, -1) };

    private static int Heuristic(Point a, Point b) {
        return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
    }

    public static List<Point>? FindPath(Map map, Point start, Point end, Bounds? bounds = null) {
        var searchBounds = bounds ?? new Bounds(0, 0, map.Width, map.Height);

        if (!searchBounds.Contains(start) || !searchBounds.Contains(end)) return null;

        PriorityQueue<Point, int> openSet = new();

        int boundsWidth = searchBounds.Width;
        int boundsHeight = searchBounds.Height;
        int arraySize = boundsWidth * boundsHeight;
        if (arraySize <= 0) return null; // Safety check for empty bounds

        int offsetX = searchBounds.X;
        int offsetY = searchBounds.Y;

        // ======================= OPTIMIZATION START =======================
        // Rent arrays from the shared pool instead of allocating new ones.
        bool[] closedSet = ArrayPool<bool>.Shared.Rent(arraySize);
        int[] gScore = ArrayPool<int>.Shared.Rent(arraySize);
        Point[] cameFrom = ArrayPool<Point>.Shared.Rent(arraySize);
        // ======================== OPTIMIZATION END ========================

        try // CRITICAL: Use a try/finally block to ensure arrays are always returned.
        {
            // Rented arrays may contain old data, so we must initialize them.
            Array.Clear(closedSet, 0, arraySize);
            Array.Fill(gScore, int.MaxValue, 0, arraySize);
            // cameFrom doesn't need clearing as it's value type, but it's good practice if it were a reference type.

            int PointToIndex(Point p) {
                return (p.Y - offsetY) * boundsWidth + (p.X - offsetX);
            }

            int startIdx = PointToIndex(start);
            gScore[startIdx] = 0;

            openSet.Enqueue(start, Heuristic(start, end));

            // ... (The entire A* while loop logic remains exactly the same) ...
            while (openSet.TryDequeue(out var current, out _)) {
                if (current.Equals(end)) {
                    List<Point> path = new();
                    var temp = current;
                    while (!temp.Equals(start)) {
                        path.Add(temp);
                        int tempIdx = PointToIndex(temp);
                        temp = cameFrom[tempIdx];
                    }

                    path.Add(start);
                    path.Reverse();
                    return path;
                }

                int currentIdx = PointToIndex(current);
                if (closedSet[currentIdx]) continue;
                closedSet[currentIdx] = true;

                foreach (var direction in Directions) {
                    var neighbor = new Point(current.X + direction.X, current.Y + direction.Y);

                    if (neighbor.X < searchBounds.X || neighbor.X >= searchBounds.X + searchBounds.Width ||
                        neighbor.Y < searchBounds.Y || neighbor.Y >= searchBounds.Y + searchBounds.Height)
                        continue;

                    if (!map.IsWalkable(neighbor)) continue;

                    int neighborIdx = PointToIndex(neighbor);
                    if (closedSet[neighborIdx]) continue;

                    int tentativeGScore = gScore[currentIdx] + 1;

                    if (tentativeGScore < gScore[neighborIdx]) {
                        cameFrom[neighborIdx] = current;
                        gScore[neighborIdx] = tentativeGScore;
                        int fScore = tentativeGScore + Heuristic(neighbor, end);
                        openSet.Enqueue(neighbor, fScore);
                    }
                }
            }

            return null;
        }
        finally {
            // ======================= OPTIMIZATION START =======================
            // Return the arrays to the pool so they can be reused by the next call.
            ArrayPool<bool>.Shared.Return(closedSet);
            ArrayPool<int>.Shared.Return(gScore);
            ArrayPool<Point>.Shared.Return(cameFrom);
            // ======================== OPTIMIZATION END ========================
        }
    }

    public static Dictionary<Point, (int cost, List<Point> path)> FindPathsToMultipleGoals(Map map, Point start,
        HashSet<Point> goals, Bounds bounds) {
        PriorityQueue<Point, int> openSet = new();

        int boundsWidth = bounds.Width;
        int boundsHeight = bounds.Height;
        int arraySize = boundsWidth * boundsHeight;
        if (arraySize <= 0) return new Dictionary<Point, (int cost, List<Point> path)>();

        int offsetX = bounds.X;
        int offsetY = bounds.Y;

        // Rent arrays
        int[] gScore = ArrayPool<int>.Shared.Rent(arraySize);
        Point[] cameFrom = ArrayPool<Point>.Shared.Rent(arraySize);

        try {
            // Initialize rented arrays
            Array.Fill(gScore, int.MaxValue, 0, arraySize);

            int PointToIndex(Point p) {
                return (p.Y - offsetY) * boundsWidth + (p.X - offsetX);
            }

            int startIdx = PointToIndex(start);
            gScore[startIdx] = 0;

            openSet.Enqueue(start, 0);

            Dictionary<Point, (int cost, List<Point> path)> foundGoals = new();
            HashSet<Point> goalsToFind = new(goals);

            // ... (The entire while loop logic remains exactly the same) ...
            while (openSet.TryDequeue(out var current, out _)) {
                if (goalsToFind.Contains(current)) {
                    List<Point> path = new();
                    var temp = current;
                    while (!temp.Equals(start)) {
                        path.Add(temp);
                        temp = cameFrom[PointToIndex(temp)];
                    }

                    path.Add(start);
                    path.Reverse();
                    foundGoals[current] = (gScore[PointToIndex(current)], path);
                    goalsToFind.Remove(current);
                    if (goalsToFind.Count == 0) break;
                }

                int currentIdx = PointToIndex(current);

                foreach (var direction in Directions) {
                    var neighbor = new Point(current.X + direction.X, current.Y + direction.Y);

                    if (neighbor.X < bounds.X || neighbor.X >= bounds.X + bounds.Width ||
                        neighbor.Y < bounds.Y || neighbor.Y >= bounds.Y + bounds.Height)
                        continue;

                    if (!map.IsWalkable(neighbor)) continue;

                    int neighborIdx = PointToIndex(neighbor);
                    int tentativeGScore = gScore[currentIdx] + 1;

                    if (tentativeGScore < gScore[neighborIdx]) {
                        cameFrom[neighborIdx] = current;
                        gScore[neighborIdx] = tentativeGScore;
                        openSet.Enqueue(neighbor, tentativeGScore);
                    }
                }
            }

            return foundGoals;
        }
        finally {
            // Return arrays
            ArrayPool<int>.Shared.Return(gScore);
            ArrayPool<Point>.Shared.Return(cameFrom);
        }
    }
}