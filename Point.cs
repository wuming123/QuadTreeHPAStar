namespace QuadTreeHPAStar;

public readonly record struct Point(int X, int Y) { }

public readonly record struct Bounds(int X, int Y, int Width, int Height) {
    public bool Contains(Point p) {
        return p.X >= X && p.X < X + Width && p.Y >= Y && p.Y < Y + Height;
    }
}