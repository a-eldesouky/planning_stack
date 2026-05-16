#include "local_planner/cubic_spline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planner
{
void CubicSpline::evalPosition(const Segment & seg, double t, double & x, double & y)
{
  x = seg.ax + t * (seg.bx + t * (seg.cx + t * seg.dx));
  y = seg.ay + t * (seg.by + t * (seg.cy + t * seg.dy));
}

void CubicSpline::evalDerivative1(const Segment & seg, double t, double & dx, double & dy)
{
  dx = seg.bx + t * (2.0 * seg.cx + t * 3.0 * seg.dx);
  dy = seg.by + t * (2.0 * seg.cy + t * 3.0 * seg.dy);
}

void CubicSpline::evalDerivative2(const Segment & seg, double t, double & ddx, double & ddy)
{
  ddx = 2.0 * seg.cx + 6.0 * seg.dx * t;
  ddy = 2.0 * seg.cy + 6.0 * seg.dy * t;
}

// kArcLengthSamples: sub-divisions per segment for numerical arc-length integration.
// 20 samples is accurate to < 0.1% for typical 0.5-1 m spline segments.
static constexpr int kArcLengthSamples = 20;

double CubicSpline::computeArcLength(const Segment & seg, int samples)
{
  double length = 0.0;
  double prev_x, prev_y;
  evalPosition(seg, 0.0, prev_x, prev_y);

  for (int k = 1; k <= samples; ++k) {
    const double t = seg.chord * static_cast<double>(k) / static_cast<double>(samples);
    double cx, cy;
    evalPosition(seg, t, cx, cy);
    length += std::hypot(cx - prev_x, cy - prev_y);
    prev_x = cx;
    prev_y = cy;
  }

  return length;
}

CubicSpline CubicSpline::fit(const std::vector<TrajectoryPoint> & control_points)
{
  CubicSpline spline;
  spline.ctrl_ = control_points;

  const std::size_t n = control_points.size();

  if (n < 2) {
    return spline;  // valid_ stays false
  }

  // Step 1: Chord lengths h_i = distance between consecutive knots
  std::vector<double> h(n - 1);
  for (std::size_t i = 0; i < n - 1; ++i) {
    const double dx = control_points[i + 1].pose.x - control_points[i].pose.x;
    const double dy = control_points[i + 1].pose.y - control_points[i].pose.y;
    h[i] = std::hypot(dx, dy);
    if (h[i] < 1e-9) {
      h[i] = 1e-9;  // guard zero-length segments (coincident points)
    }
  }

  // Step 2: Solve tridiagonal system for second derivatives M
  //   Interior unknowns: M_1 … M_{n-2}  (M_0 = M_{n-1} = 0 by natural BC)
  std::vector<double> Mx(n, 0.0);
  std::vector<double> My(n, 0.0);

  const std::size_t m = n - 2;  // number of interior unknowns

  if (m > 0) {
    // Build the tridiagonal system: diag, lower, upper, rhs
    std::vector<double> diag(m), lower(m > 1 ? m - 1 : 0), upper(m > 1 ? m - 1 : 0);
    std::vector<double> rhsx(m), rhsy(m);

    for (std::size_t i = 0; i < m; ++i) {
      diag[i] = 2.0 * (h[i] + h[i + 1]);

      if (i > 0) {
        lower[i - 1] = h[i];
      }
      if (i < m - 1) {
        upper[i] = h[i + 1];
      }

      // Right-hand side = 6 * divided difference of first differences
      rhsx[i] = 6.0 * ((control_points[i + 2].pose.x - control_points[i + 1].pose.x) / h[i + 1] -
                       (control_points[i + 1].pose.x - control_points[i].pose.x) / h[i]);

      rhsy[i] = 6.0 * ((control_points[i + 2].pose.y - control_points[i + 1].pose.y) / h[i + 1] -
                       (control_points[i + 1].pose.y - control_points[i].pose.y) / h[i]);
    }

    // Thomas algorithm: forward elimination
    // Guard: if any pivot d_fwd[i] collapses to zero the system is singular
    // (degenerate control points). Return an invalid spline rather than divide by zero.
    std::vector<double> d_fwd(m);
    std::vector<double> rx_fwd(m), ry_fwd(m);

    d_fwd[0] = diag[0];
    rx_fwd[0] = rhsx[0];
    ry_fwd[0] = rhsy[0];

    for (std::size_t i = 1; i < m; ++i) {
      if (std::abs(d_fwd[i - 1]) < 1e-12) {
        return spline;  // singular pivot: valid_ stays false
      }
      const double w = lower[i - 1] / d_fwd[i - 1];
      d_fwd[i] = diag[i] - w * upper[i - 1];
      rx_fwd[i] = rhsx[i] - w * rx_fwd[i - 1];
      ry_fwd[i] = rhsy[i] - w * ry_fwd[i - 1];
    }

    // Back substitution: fills M_1 … M_{n-2}
    if (std::abs(d_fwd[m - 1]) < 1e-12) {
      return spline;  // singular pivot: valid_ stays false
    }
    Mx[m] = rx_fwd[m - 1] / d_fwd[m - 1];
    My[m] = ry_fwd[m - 1] / d_fwd[m - 1];

    for (int i = static_cast<int>(m) - 2; i >= 0; --i) {
      const auto ui = static_cast<std::size_t>(i);
      if (std::abs(d_fwd[ui]) < 1e-12) {
        return spline;  // singular pivot: valid_ stays false
      }
      Mx[ui + 1] = (rx_fwd[ui] - upper[ui] * Mx[ui + 2]) / d_fwd[ui];
      My[ui + 1] = (ry_fwd[ui] - upper[ui] * My[ui + 2]) / d_fwd[ui];
    }
  }

  // Step 3: Build cubic segments from M values
  spline.segments_.reserve(n - 1);
  spline.total_length_ = 0.0;

  for (std::size_t i = 0; i < n - 1; ++i) {
    Segment seg;
    seg.chord = h[i];

    // x(t) = ax + bx*t + cx*t² + dx*t³
    seg.ax = control_points[i].pose.x;
    seg.bx = (control_points[i + 1].pose.x - control_points[i].pose.x) / h[i] -
             h[i] * (2.0 * Mx[i] + Mx[i + 1]) / 6.0;
    seg.cx = Mx[i] / 2.0;
    seg.dx = (Mx[i + 1] - Mx[i]) / (6.0 * h[i]);

    // y(t) = ay + by*t + cy*t² + dy*t³
    seg.ay = control_points[i].pose.y;
    seg.by = (control_points[i + 1].pose.y - control_points[i].pose.y) / h[i] -
             h[i] * (2.0 * My[i] + My[i + 1]) / 6.0;
    seg.cy = My[i] / 2.0;
    seg.dy = (My[i + 1] - My[i]) / (6.0 * h[i]);

    seg.arc = computeArcLength(seg);
    spline.total_length_ += seg.arc;

    spline.segments_.push_back(seg);
  }

  spline.valid_ = (spline.total_length_ > 1e-6);
  return spline;
}

std::vector<TrajectoryPoint> CubicSpline::resample(double spacing_m) const
{
  if (!valid_ || spacing_m <= 0.0) {
    return {};
  }

  // Number of output points: at least start + end
  const std::size_t n_out =
    std::max(std::size_t(2), static_cast<std::size_t>(total_length_ / spacing_m) + 1);

  std::vector<TrajectoryPoint> out;
  out.reserve(n_out + 1);

  // Always include the exact first control point so the trajectory starts
  // precisely at the ego position (or A* start node), not 0.5 m ahead of it.
  out.push_back(ctrl_.front());

  // Arc-length budget remaining until the next sample point
  double budget = 0.0;

  // Sub-steps per segment for arc-length walking
  constexpr int kSubSteps = 20;

  std::size_t ctrl_idx = 0;  // nearest control point tracker

  for (std::size_t si = 0; si < segments_.size(); ++si) {
    const Segment & seg = segments_[si];
    const double dt = seg.chord / static_cast<double>(kSubSteps);

    double prev_x, prev_y;
    evalPosition(seg, 0.0, prev_x, prev_y);

    for (int k = 1; k <= kSubSteps; ++k) {
      const double t = dt * static_cast<double>(k);

      double cx, cy;
      evalPosition(seg, t, cx, cy);

      const double step = std::hypot(cx - prev_x, cy - prev_y);
      budget += step;

      // Emit a sample point every time we accumulate spacing_m of arc
      while (budget >= spacing_m && out.size() < n_out) {
        // Interpolate t back to where the sample falls
        const double overshoot = budget - spacing_m;
        const double frac = (step > 1e-9) ? overshoot / step : 0.0;
        const double t_sample = t - frac * dt;

        double sx, sy, dx, dy, ddx, ddy;
        evalPosition(seg, t_sample, sx, sy);
        evalDerivative1(seg, t_sample, dx, dy);
        evalDerivative2(seg, t_sample, ddx, ddy);

        const double speed_sq = dx * dx + dy * dy;
        const double speed = std::sqrt(speed_sq);

        TrajectoryPoint pt;
        pt.pose.x = sx;
        pt.pose.y = sy;

        // z and non-geometric fields from nearest control point
        const auto & src = ctrl_[std::min(ctrl_idx, ctrl_.size() - 1)];
        pt.pose.z = src.pose.z;

        // Heading from spline tangent
        pt.pose.heading = std::atan2(dy, dx);

        // Signed curvature κ = (x'y'' − y'x'') / |v|³
        pt.curvature = (speed > 1e-6) ? (dx * ddy - dy * ddx) / (speed_sq * speed) : 0.0;

        // Inherit speed constraint and stop flags from nearest control point
        pt.max_velocity_mps = src.max_velocity_mps;
        pt.is_stop_point = false;  // stop flag set explicitly below on last point
        pt.stop_reason = {};

        // Skip if too close to the last emitted point (guards against
        // duplicating the prepended start point on the first sample).
        const double d_last =
          std::hypot(pt.pose.x - out.back().pose.x, pt.pose.y - out.back().pose.y);
        if (d_last >= spacing_m * 0.5) {
          out.push_back(pt);
        }
        budget -= spacing_m;
      }

      prev_x = cx;
      prev_y = cy;
    }

    // Advance control point tracker as we finish each segment
    if (ctrl_idx + 1 < ctrl_.size()) {
      ++ctrl_idx;
    }
  }

  // Always include the exact final control point
  // This guarantees the stop point or goal point is preserved precisely,
  // regardless of how the arc-length sampling lands.
  if (!ctrl_.empty()) {
    const auto & last_ctrl = ctrl_.back();

    TrajectoryPoint last_pt;
    last_pt.pose = last_ctrl.pose;
    last_pt.curvature = out.empty() ? 0.0 : out.back().curvature;
    last_pt.max_velocity_mps = last_ctrl.max_velocity_mps;
    last_pt.is_stop_point = last_ctrl.is_stop_point;
    last_pt.stop_reason = last_ctrl.stop_reason;

    // Replace the last output point if it is very close to avoid duplicates,
    // otherwise append.
    if (!out.empty()) {
      const double d =
        std::hypot(out.back().pose.x - last_pt.pose.x, out.back().pose.y - last_pt.pose.y);
      if (d < spacing_m * 0.5) {
        out.back() = last_pt;
      } else {
        out.push_back(last_pt);
      }
    } else {
      out.push_back(last_pt);
    }
  }

  return out;
}

}  // namespace local_planner
