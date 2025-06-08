using System.Diagnostics;
using System.Text;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;

namespace QuadTreeHPAStar;

[RankColumn]
[MemoryDiagnoser]
[GcConcurrent]
[StatisticalTestColumn]
public class PathfindingBenchmarks {
    private readonly Point _end = new(1023, 1023);
    private readonly Point _start = new(3, 3);
    private HPAStar _hpaStar = null!;
    private Map _map = null!;

    [GlobalSetup]
    public void Setup() {
        byte[,] grid = InitMap();
        grid[_start.Y, _start.X] = 0;
        grid[_end.Y, _end.X] = 0;
        _map = new Map(grid);
    }


    [Benchmark]
    public List<Point>? HPAStar_FindPath() {
        if (_hpaStar == null) {
            _hpaStar = new HPAStar(_map, 4, 64);
            _hpaStar.Precompute();
        }

        return _hpaStar.FindPath(_start, _end);
    }

    [Benchmark]
    public List<Point>? StandardAStar_FindPath() {
        return AStar.FindPath(_map, _start, _end);
    }

    private static byte[,] InitMap() {
        string[] data = File.ReadAllLines("./maze.txt", Encoding.UTF8);
        byte[,] grid = new byte[data.Length, data.Length];
        for (int i = 0; i < data.Length; i++) {
            char[] str = data[i].ToCharArray();
            for (int j = 0; j < str.Length; j++)
                grid[i, j] = str[j] switch {
                    '#' => 0,
                    '■' => 1,
                    _ => 0
                };
        }

        return grid;
    }
}

public static class Program {
    public static void Main(string[] args) {
        var summary = BenchmarkRunner.Run<PathfindingBenchmarks>();
        test();
    }

    private static void test() {
        InitMap();
        byte[,] grid = InitMap();
        var start = new Point(3, 3);
        var end = new Point(1023, 1023);
        grid[start.Y, start.X] = 0;
        grid[end.Y, end.X] = 0;

        var map = new Map(grid);
        Console.WriteLine("--- HPA* Precomputation ---");
        var hpaStar = new HPAStar(map, 16, 64);
        var precomputeWatch = Stopwatch.StartNew();
        hpaStar.Precompute();
        precomputeWatch.Stop();
        Console.WriteLine($"Total Precomputation Time: {precomputeWatch.Elapsed.TotalMilliseconds:F3} ms");
        Console.WriteLine("\n\n--- Running HPA* Search ---");
        var hpaWatch = Stopwatch.StartNew();
        List<Point>? hpaPath = hpaStar.FindPath(start, end);
        hpaWatch.Stop();
        Console.WriteLine($"\nHPA* Search Completed in: {hpaWatch.Elapsed.TotalMilliseconds:F3} ms");
        if (hpaPath != null)
            Console.WriteLine($"Path found with {hpaPath.Count} steps.");
        else
            Console.WriteLine("No path found by HPA*.");
        Console.WriteLine("\n\n--- Running Standard A* Search (for comparison) ---");
        var aStarWatch = Stopwatch.StartNew();
        List<Point>? standardAPath = AStar.FindPath(map, start, end);
        aStarWatch.Stop();
        Console.WriteLine($"\nStandard A* Search Completed in: {aStarWatch.Elapsed.TotalMilliseconds:F3} ms");
        if (standardAPath != null)
            Console.WriteLine($"Path found with {standardAPath.Count} steps.");
        else
            Console.WriteLine("No path found by Standard A*.");
    }

    private static byte[,] InitMap() {
        string[] data = File.ReadAllLines("C:/Users/54936/Desktop/Project/ai/maze.txt", Encoding.UTF8);
        byte[,] grid = new byte[data.Length, data.Length];
        for (int i = 0; i < data.Length; i++) {
            char[] str = data[i].ToCharArray();
            for (int j = 0; j < str.Length; j++)
                switch (str[j]) {
                    case '#': {
                        grid[i, j] = 0;
                        break;
                    }
                    case '■': {
                        grid[i, j] = 1;
                        break;
                    }
                    default: {
                        grid[i, j] = 0;
                        break;
                    }
                }
        }

        return grid;
    }
}