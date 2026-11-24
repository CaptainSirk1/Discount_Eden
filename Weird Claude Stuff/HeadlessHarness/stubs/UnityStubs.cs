// Minimal stand-ins for Unity.Collections / Unity.Jobs / Unity.Burst / UnityEngine so the simulation's
// job structs can be compiled and run serially under plain .NET. This is a TEST ENVIRONMENT ONLY:
// it does not check Burst compatibility, job safety (read/write aliasing) or Unity behaviour.
using System;

namespace Unity.Burst
{
    [AttributeUsage(AttributeTargets.Struct | AttributeTargets.Class | AttributeTargets.Method)]
    public class BurstCompileAttribute : Attribute { }
}

namespace Unity.Collections
{
    public enum Allocator { Invalid, None, Temp, TempJob, Persistent }
    public enum NativeArrayOptions { UninitializedMemory, ClearMemory }
    [AttributeUsage(AttributeTargets.Field)] public class ReadOnlyAttribute : Attribute { }
    [AttributeUsage(AttributeTargets.Field)] public class WriteOnlyAttribute : Attribute { }
    [AttributeUsage(AttributeTargets.Field)] public class NativeDisableParallelForRestrictionAttribute : Attribute { }

    public struct NativeArray<T> : IDisposable where T : struct
    {
        T[] _a;
        public NativeArray(int n, Allocator a, NativeArrayOptions o = NativeArrayOptions.ClearMemory) { _a = new T[n]; }
        public NativeArray(T[] src, Allocator a) { _a = (T[])src.Clone(); }
        public T this[int i] { get => _a[i]; set => _a[i] = value; }
        public int Length => _a.Length;
        public bool IsCreated => _a != null;
        public void Dispose() { _a = null; }
        public T[] ToArray() => (T[])_a.Clone();
        public void CopyFrom(NativeArray<T> o) => Array.Copy(o._a, _a, _a.Length);
        public void CopyTo(NativeArray<T> o) => Array.Copy(_a, o._a, _a.Length);
    }

    public struct FixedList512Bytes<T> where T : struct
    {
        T[] _a; int _n;
        public int Length => _n;
        int Cap => 512 / System.Runtime.InteropServices.Marshal.SizeOf<T>() - 1;
        public void Add(T v) { if (_a == null) _a = new T[Cap]; if (_n >= Cap) throw new IndexOutOfRangeException("FixedList512 full"); _a[_n++] = v; }
        public T this[int i] { get { if (i >= _n) throw new IndexOutOfRangeException(); return _a[i]; } set { _a[i] = value; } }
        public void Clear() => _n = 0;
    }
    public struct FixedList128Bytes<T> where T : struct
    {
        T[] _a; int _n;
        public int Length => _n;
        int Cap => 128 / System.Runtime.InteropServices.Marshal.SizeOf<T>() - 1;
        public void Add(T v) { if (_a == null) _a = new T[Cap]; if (_n >= Cap) throw new IndexOutOfRangeException("FixedList128 full"); _a[_n++] = v; }
        public T this[int i] { get { if (i >= _n) throw new IndexOutOfRangeException(); return _a[i]; } set { _a[i] = value; } }
        public void Clear() => _n = 0;
    }
}

namespace Unity.Jobs
{
    public interface IJob { void Execute(); }
    public interface IJobParallelFor { void Execute(int index); }
    public struct JobHandle
    {
        public void Complete() { }
        public bool IsCompleted => true;
        public static JobHandle CombineDependencies(JobHandle a, JobHandle b) => default;
        public static void ScheduleBatchedJobs() { }
    }
    public static class IJobExtensions
    {
        public static void Run<T>(this T job) where T : struct, IJob => job.Execute();
        public static JobHandle Schedule<T>(this T job, JobHandle dep = default) where T : struct, IJob { job.Execute(); return default; }
    }
    public static class IJobParallelForExtensions
    {
        public static void Run<T>(this T job, int n) where T : struct, IJobParallelFor { for (int i = 0; i < n; i++) job.Execute(i); }
        public static JobHandle Schedule<T>(this T job, int n, int batch, JobHandle dep = default) where T : struct, IJobParallelFor
        { for (int i = 0; i < n; i++) job.Execute(i); return default; }
    }
}

namespace UnityEngine
{
    public class PropertyAttribute : Attribute { }
    public struct Color32 { public byte r, g, b, a; public Color32(byte r, byte g, byte b, byte a) { this.r = r; this.g = g; this.b = b; this.a = a; } }
    public struct Vector3
    {
        public float x, y, z;
        public Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public static implicit operator Vector3(Unity.Mathematics.float3 v) => new Vector3(v.x, v.y, v.z);
        public static implicit operator Unity.Mathematics.float3(Vector3 v) => new Unity.Mathematics.float3(v.x, v.y, v.z);
    }
}
