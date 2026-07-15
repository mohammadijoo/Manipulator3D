using System;
using System.Numerics;

namespace Manipulator3D.sim;

public sealed class LinearTrajectory
{
    private Vector3 _a;
    private Vector3 _b;
    private float _duration = 1.0f;
    private float _t;
    private float _alpha;
    private bool _finished = true;

    public void Reset(Vector3 from, Vector3 to, float durationSec)
    {
        _a = from;
        _b = to;
        _duration = durationSec > 1e-6f ? durationSec : 1e-6f;
        _t = 0.0f;
        _alpha = 0.0f;
        _finished = false;
    }

    public void Update(float dt)
    {
        if (_finished) return;
        _t += dt;
        _alpha = Math.Clamp(_t / _duration, 0.0f, 1.0f);
        if (_alpha >= 1.0f) _finished = true;
    }

    public Vector3 Position()
    {
        return _a + (_b - _a) * _alpha;
    }

    public bool Finished() => _finished;
    public float Alpha() => _alpha;
}
