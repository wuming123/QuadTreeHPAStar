namespace QuadTreeHPAStar;

/// <summary>
///     表示一个2D整数坐标点。
///     使用 record struct 以获得值类型性能、不变性和自动实现的相等性比较。
/// </summary>
public readonly record struct Point(int X, int Y) { }

/// <summary>
///     表示一个矩形区域。
/// </summary>
public readonly record struct Bounds(int X, int Y, int Width, int Height) {
    public bool Contains(Point p) {
        return p.X >= X && p.X < X + Width && p.Y >= Y && p.Y < Y + Height;
    }
}