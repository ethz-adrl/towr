/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/variables/spline.h>

#include <numeric> // std::accumulate

namespace towr {

Spline::Spline(const VecTimes& poly_durations, int n_dim)
{
  uint n_polys = poly_durations.size();

  cubic_polys_.assign(n_polys, CubicHermitePolynomial(n_dim));
  for (int i=0; i<cubic_polys_.size(); ++i) {
    cubic_polys_.at(i).SetDuration(poly_durations.at(i));
  }

  UpdatePolynomialCoeff();
}

int
Spline::GetSegmentID(double t_global, const VecTimes& durations)
{
  double eps = 1e-10; // double precision
  assert(t_global >= 0.0);

   double t = 0;
   int i=0;
   for (double d: durations) {
     t += d;

     if (t >= t_global-eps) // at junctions, returns previous spline (=)
       return i;

     i++;
   }

   assert(false); // this should never be reached
}

std::pair<int,double>
Spline::GetLocalTime (double t_global, const VecTimes& durations) const
{
  int id = GetSegmentID(t_global, durations);

  double t_local = t_global;
  for (int i=0; i<id; i++)
    t_local -= durations.at(i);

  return std::make_pair(id, t_local);
}

const State
Spline::GetPoint(double t_global) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, GetPolyDurations());

  return GetPoint(id, t_local);
}

const State
Spline::GetPoint(int poly_id, double t_local) const
{
  return cubic_polys_.at(poly_id).GetPoint(t_local);
}

void
Spline::UpdatePolynomialCoeff()
{
  for (auto& p : cubic_polys_)
    p.UpdateCoeff();
}

int
Spline::GetPolynomialCount () const
{
  return cubic_polys_.size();
}

Spline::VecTimes
Spline::GetPolyDurations() const
{
  VecTimes poly_durations;
  for (const auto& p : cubic_polys_)
    poly_durations.push_back(p.GetDuration());

  return poly_durations;
}

double
Spline::GetTotalTime() const
{
  auto v = GetPolyDurations();
  return std::accumulate(v.begin(), v.end(), 0.0);
}

} /* namespace towr */

