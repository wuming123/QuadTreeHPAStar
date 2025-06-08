namespace QuadTreeHPAStar;

public class Map {
    private readonly byte[] _data;
    private readonly int[] _summedAreaTable;

    public Map(byte[,] grid) {
        Height = grid.GetLength(0);
        Width = grid.GetLength(1);
        _data = new byte[Width * Height];

        for (int y = 0; y < Height; y++)
        for (int x = 0; x < Width; x++)
            _data[y * Width + x] = grid[y, x];

        _summedAreaTable = BuildSummedAreaTable();
    }

    public int Width { get; }
    public int Height { get; }

    public bool IsWalkable(int x, int y) {
        if (x < 0 || x >= Width || y < 0 || y >= Height) return false;

        return _data[y * Width + x] == 0;
    }

    public bool IsWalkable(Point p) {
        return IsWalkable(p.X, p.Y);
    }

    /// <summary>
    ///     检查一个区域是完全可通行、完全不可通行，还是混合的。
    /// </summary>
    /// <returns>true: 完全可通行; false: 完全不可通行; null: 混合.</returns>
    public bool? IsRegionUniform(Bounds bounds) {
        int obstacleSum = GetObstacleSumInBounds(bounds);
        if (obstacleSum == 0) return true; // All walkable

        int totalTiles = bounds.Width * bounds.Height;
        if (obstacleSum == totalTiles) return false; // All unwalkable

        return null; // Mixed
    }

    private int[] BuildSummedAreaTable() {
        int[] sat = new int[Width * Height];
        for (int y = 0; y < Height; y++)
        for (int x = 0; x < Width; x++) {
            int index = y * Width + x;
            int obstacleValue = _data[index];
            int top = y > 0 ? sat[(y - 1) * Width + x] : 0;
            int left = x > 0 ? sat[y * Width + x - 1] : 0;
            int topLeft = y > 0 && x > 0 ? sat[(y - 1) * Width + x - 1] : 0;
            sat[index] = obstacleValue + top + left - topLeft;
        }

        return sat;
    }

    public void SetObstacle(Point p, bool isObstacle) {
        if (p.X < 0 || p.X >= Width || p.Y < 0 || p.Y >= Height) return;

        int index = p.Y * Width + p.X;
        byte oldValue = _data[index];
        byte newValue = (byte)(isObstacle ? 1 : 0);

        if (oldValue == newValue) return; // No change

        _data[index] = newValue;
        int diff = newValue - oldValue;

        // Efficiently update the Summed-Area Table
        for (int y = p.Y; y < Height; y++)
        for (int x = p.X; x < Width; x++)
            _summedAreaTable[y * Width + x] += diff;
    }

    private int GetObstacleSumInBounds(Bounds bounds) {
        int x1 = bounds.X - 1;
        int y1 = bounds.Y - 1;
        int x2 = bounds.X + bounds.Width - 1;
        int y2 = bounds.Y + bounds.Height - 1;

        int d = _summedAreaTable[y2 * Width + x2];
        int b = x1 >= 0 ? _summedAreaTable[y2 * Width + x1] : 0;
        int c = y1 >= 0 ? _summedAreaTable[y1 * Width + x2] : 0;
        int a = x1 >= 0 && y1 >= 0 ? _summedAreaTable[y1 * Width + x1] : 0;

        return d - b - c + a;
    }
}