// Given a time between 0 and 1, evaluates a cubic polynomial with
// the given endpoint and tangent values at the beginning (0) and
// end (1) of the interval.  Optionally, one can request a derivative
// of the spline (0=no derivative, 1=first derivative, 2=2nd derivative).
template <class T>
inline T Spline<T>::cubicSplineUnitInterval(
    const T& position0, const T& position1, const T& tangent0,
    const T& tangent1, double normalizedTime, int derivative) {
  // TODO (Animation) Task 1a
  // evaluate time, squared, cubed
  double t_1 = normalizedTime, t_2 = normalizedTime * normalizedTime,
          t_3 = normalizedTime * normalizedTime * normalizedTime;
//  assert(derivative >= 0 && derivative <= 2);
//  assert(normalizedTime <= 1 && normalizedTime >= 0);
  // basis
  double h00, h10, h01, h11;
  if (derivative == 0) {
    h00 = 2.0 * t_3 - 3.0 * t_2 + 1.0;
    h10 = t_3 - 2.0 * t_2 + t_1;
    h01 = -2.0 * t_3 + 3.0 * t_2;
    h11 = t_3 - t_2;
  } else if (derivative == 1) {
    h00 = 6.0 * t_2 - 6.0 * t_1;
    h10 = 3.0 * t_2 - 4.0 * t_1 + 1.0;
    h01 = -6.0 * t_2 + 6.0 * t_1;
    h11 = 3.0 * t_2 - 2.0 * t_1;
  } else if (derivative == 2.0) {
    h00 = 12.0 * t_1 - 6.0;
    h10 = 6.0 * t_1 - 4.0;
    h01 = -12.0 * t_1 + 6.0;
    h11 = 6.0 * t_1 - 2.0;
  }
  return (position0 * h00 + h10 * tangent0 + h01 * position1 + h11 * tangent1);

}

// Returns a state interpolated between the values directly before and after the
// given time.
template <class T>
inline T Spline<T>::evaluate(double time, int derivative) {
  // TODO (Animation) Task 1b
  //If there are no knots at all in the spline,
  // interpolation should return the default value for the interpolated type
  if (knots.size() < 1) {
    return T();

  }

  else if (knots.size() == 1 || time <= knots.begin()->first) {
    //If there is only one knot in the spline,
    // interpolation should always return the value of that knot (independent of the time).

    // If the query time is less than or equal to the initial knot, return the initial knot's value
    if (derivative == 0)
      return knots.begin()->second;
    else
      return T();
  } else if (time >= knots.rbegin()->first) {
    //If the query time is greater than or equal to the final knot, return the final knot's value.
    if (derivative == 0)
      return knots.rbegin()->second;
    else
      return T();
  }




  T k1, k2 , k0, k3;
  typename std::map<double, T>::iterator n0, n1, n2, n3;
  double t0,t1,t2,t3;

  n2 = knots.upper_bound(time);
  k2 = n2->second;
  t2 = n2->first;
  if (knots.size() >= 2) {
    T k;
  }
  if (n2->first >= time && n2 != knots.begin()) {
    n1 = std::prev(n2);
    k1 = n1->second;
    t1 = n1->first;

    if (n1 == knots.begin()) {
      k0 = k1 - (k2 - k1);
      t0 = t1 - (t2 - t1);
    } else {
      n0 = std::prev(n1);
      k0 = n0->second;
      t0 = n0->first;
    }
    if (n2 == std::prev(knots.end())) {
      k3 = k2 + k2 - k1;
      t3 = t2 + t2 - t1;
    } else {
      n3 = std::next(n2);
      k3 = n3->second;
      t3 = n3->first;
    }
  } else {
    printf("shouldn't be here \n");
  }


  T m1 = (k2 - k0) / (t2 - t0),
          m2 = (k3 - k1) / (t3 - t1);
  double normalizedTime = (time - t1) / (t2 - t1);
  if (normalizedTime > 1 || normalizedTime < 0)
    printf("Time is wrong: %.2f\n", normalizedTime);
  return cubicSplineUnitInterval(k1, k2, m1 * (t2 - t1), m2 * (t2 - t1), normalizedTime, derivative);




}

// Removes the knot closest to the given time,
//    within the given tolerance..
// returns true iff a knot was removed.
template <class T>
inline bool Spline<T>::removeKnot(double time, double tolerance) {
  // Empty maps have no knots.
  if (knots.size() < 1) {
    return false;
  }

  // Look up the first element > or = to time.
  typename std::map<double, T>::iterator t2_iter = knots.lower_bound(time);
  typename std::map<double, T>::iterator t1_iter;
  t1_iter = t2_iter;
  t1_iter--;

  if (t2_iter == knots.end()) {
    t2_iter = t1_iter;
  }

  // Handle tolerance bounds,
  // because we are working with floating point numbers.
  double t1 = (*t1_iter).first;
  double t2 = (*t2_iter).first;

  double d1 = fabs(t1 - time);
  double d2 = fabs(t2 - time);

  if (d1 < tolerance && d1 < d2) {
    knots.erase(t1_iter);
    return true;
  }

  if (d2 < tolerance && d2 < d1) {
    knots.erase(t2_iter);
    return t2;
  }

  return false;
}

// Sets the value of the spline at a given time (i.e., knot),
// creating a new knot at this time if necessary.
template <class T>
inline void Spline<T>::setValue(double time, T value) {
  knots[time] = value;
}

template <class T>
inline T Spline<T>::operator()(double time) {
  return evaluate(time);
}
