#!/usr/bin/env python3
"""Independent F3 volume from cylinder inequalities, without kernel geometry.

For fixed x,y in the stock disc x*x+y*y<=4, the tilted cutter clips z to
three affine lower bounds and three affine upper bounds. Integrate their
positive overlap exactly in y after partitioning every line intersection;
only the outer x integral uses adaptive Simpson quadrature.
"""

import math


def overlap_section(x):
    y_max = math.sqrt(max(0.0, 4.0 - x * x))
    radial = math.sqrt(max(0.0, 30.25 - (x - 4.0) ** 2))
    root3 = math.sqrt(3.0)
    lower = [(0.0, 0.0), (1 / root3, 2 / root3 - 0.5),
             (-root3, -2 * root3 - 0.5 - 2 * radial)]
    upper = [(0.0, 3.5), (1 / root3, 2 / root3 - 0.5 + root3),
             (-root3, -2 * root3 - 0.5 + 2 * radial)]
    lines = lower + upper
    events = [-y_max, y_max]
    for i, (slope_a, intercept_a) in enumerate(lines):
        for slope_b, intercept_b in lines[i + 1:]:
            if slope_a != slope_b:
                crossing = (intercept_b - intercept_a) / (slope_a - slope_b)
                if -y_max < crossing < y_max:
                    events.append(crossing)
    events.sort()

    def height(y):
        lo = max(slope * y + intercept for slope, intercept in lower)
        hi = min(slope * y + intercept for slope, intercept in upper)
        return max(0.0, hi - lo)

    return sum((b - a) * (height(a) + height(b)) / 2
               for a, b in zip(events, events[1:]))


def integrate(function, a, b, epsilon):
    def simpson(a, b, fa, fm, fb):
        return (b - a) * (fa + 4 * fm + fb) / 6

    def refine(a, b, fa, fm, fb, whole, epsilon, depth):
        mid = (a + b) / 2
        fl = function((a + mid) / 2)
        fr = function((mid + b) / 2)
        left = simpson(a, mid, fa, fl, fm)
        right = simpson(mid, b, fm, fr, fb)
        delta = left + right - whole
        if abs(delta) <= 15 * epsilon:
            return left + right + delta / 15
        if depth == 0:
            raise RuntimeError("independent quadrature budget exhausted")
        return (refine(a, mid, fa, fl, fm, left, epsilon / 2, depth - 1)
                + refine(mid, b, fm, fr, fb, right, epsilon / 2, depth - 1))

    fa, fm, fb = function(a), function((a + b) / 2), function(b)
    return refine(a, b, fa, fm, fb, simpson(a, b, fa, fm, fb), epsilon, 40)


if __name__ == "__main__":
    # Quadratic approach to both x endpoints removes the square-root endpoint
    # singularities of the two circular cross-sections.
    def angular_section(angle):
        return overlap_section(0.25 + 1.75 * math.cos(angle)) * 1.75 * math.sin(angle)

    for tolerance in [1e-7, 1e-9, 1e-11]:
        overlap = integrate(angular_section, 0.0, math.pi, tolerance)
        print(f"epsilon={tolerance:g} overlap={overlap:.14f} cut={14 * math.pi - overlap:.14f}")
