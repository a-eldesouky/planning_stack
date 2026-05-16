#ifndef LOCAL_PLANNER__CUBIC_SPLINE_HPP_
#define LOCAL_PLANNER__CUBIC_SPLINE_HPP_

#include <vector>

#include "local_planner/planning_types.hpp"

namespace local_planner
{

/**
 * @class CubicSpline
 * @brief Natural cubic spline fitted through a sequence of TrajectoryPoints.
 *
 * Fits a piecewise cubic polynomial through the (x, y) positions of the input
 * control points, using chord length as the parametric variable.  The "natural"
 * boundary condition sets the second derivative to zero at both ends, which
 * produces a smooth, non-oscillating curve.
 */
class CubicSpline
{
public:
  /**
   * @brief Fit a natural cubic spline through the given control points.
   *
   * Requires at least 2 points.  Returns an invalid spline (valid() == false)
   * if the input is degenerate (fewer than 2 points, or all points coincident).
   *
   * @param control_points  Ordered waypoints used as spline knots.
   *                        Position fields (pose.x, pose.y, pose.z) are used
   *                        for fitting; all other fields are carried through.
   * @return CubicSpline object ready for resampling.
   */
  static CubicSpline fit(const std::vector<TrajectoryPoint> & control_points);

  /**
   * @brief Resample the spline at a fixed arc-length spacing.
   *
   * Always includes the exact start and end control points.
   * Heading at each output point is computed from the spline tangent:
   *   θ = atan2(dy/ds, dx/ds)
   * Curvature at each output point is computed from the signed curvature formula:
   *   κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
   *
   * Non-geometric fields are inherited from the nearest input control point.
   * Stop-point flags are always preserved on the final output point.
   *
   * @param spacing_m  Desired arc-length between consecutive output points [m].
   *                   Must be > 0.  Typical value: 0.5 m.
   * @return Resampled trajectory.  Empty if the spline is invalid or spacing <= 0.
   */
  std::vector<TrajectoryPoint> resample(double spacing_m) const;

  /**
   * @brief True if the spline was successfully fitted and can be resampled.
   */
  bool valid() const { return valid_; }

  /**
   * @brief Total arc length of the fitted spline [m].
   */
  double length() const { return total_length_; }

private:
  /**
   * @brief One cubic segment between two consecutive control points.
   *
   * Parametric form (t ∈ [0, chord]):
   *   x(t) = ax + bx*t + cx*t² + dx*t³
   *   y(t) = ay + by*t + cy*t² + dy*t³
   */
  struct Segment
  {
    double ax{0.0}, bx{0.0}, cx{0.0}, dx{0.0};
    double ay{0.0}, by{0.0}, cy{0.0}, dy{0.0};
    double chord{0.0};  ///< parametric interval length (= chord distance between knots)
    double arc{0.0};    ///< approximate arc length of this segment [m]
  };

  /// Fitted segments (size = control_points.size() - 1)
  std::vector<Segment> segments_;

  /// Original control points: carried for non-geometric field inheritance
  std::vector<TrajectoryPoint> ctrl_;

  /// Approximate total arc length
  double total_length_{0.0};

  /// True after a successful fit
  bool valid_{false};

  /**
   * @brief Evaluate spline position at parameter t within a segment.
   */
  static void evalPosition(const Segment & seg, double t, double & x, double & y);

  /**
   * @brief Evaluate first derivative (tangent) at t within a segment.
   */
  static void evalDerivative1(const Segment & seg, double t, double & dx, double & dy);

  /**
   * @brief Evaluate second derivative at t within a segment.
   *        Used for curvature: κ = (x'y'' - y'x'') / |v|³
   */
  static void evalDerivative2(const Segment & seg, double t, double & ddx, double & ddy);

  /**
   * @brief Approximate arc length of one segment using sub-sampling.
   */
  static double computeArcLength(const Segment & seg, int samples = 20);
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__CUBIC_SPLINE_HPP_
