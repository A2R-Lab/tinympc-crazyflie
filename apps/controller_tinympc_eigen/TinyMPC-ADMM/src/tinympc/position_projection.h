#pragma once
#include <cmath>
#include <limits>
#include <Eigen/Dense>

// Exact active-set projection onto two vertical planes and an XY box.
// A strictly convex diagonal metric has its optimum inside, on one edge,
// or at a pairwise intersection. No alternating-projection approximation.
static inline bool tiny_ProjectXY(const Eigen::Vector2f& input,
    const Eigen::Vector2f* planes, const float* bounds, int count,
    const Eigen::Vector2f& lower, const Eigen::Vector2f& upper,
    const Eigen::Vector2f& metric, Eigen::Vector2f& output) {
  if (count < 0 || count > 2 || !input.allFinite() ||
      !metric.allFinite() || metric.minCoeff() <= 0 ||
      !lower.allFinite() || !upper.allFinite() ||
      (lower.array()>upper.array()).any()) return false;
  if(!count) {output=input.cwiseMax(lower).cwiseMin(upper);return true;}
  // In flight the XY box is effectively unbounded. First solve just the one
  // or two planes (at most four candidates), and accept if inside the box.
  // Only a box-active result needs the general six-boundary enumeration below.
  float tolerance[2];
  for(int i=0;i<count;++i) {
    const float norm=planes[i].norm();
    if(!planes[i].allFinite() || !std::isfinite(bounds[i]) || norm<1e-8f) return false;
    tolerance[i]=1e-5f*norm;
  }
  float planeBest=std::numeric_limits<float>::infinity();
  Eigen::Vector2f planeOutput;
  bool planeFound=false;
  auto considerPlane = [&](const Eigen::Vector2f& candidate) {
    if(!candidate.allFinite()) return;
    for(int j=0;j<count;++j)
      if(planes[j].dot(candidate)>bounds[j]+tolerance[j]) return;
    const Eigen::Vector2f delta=candidate-input;
    const float cost=delta.dot(metric.cwiseProduct(delta));
    if(cost<planeBest){planeBest=cost;planeOutput=candidate;planeFound=true;}
  };
  considerPlane(input);
  if(!planeFound) {
    for(int i=0;i<count;++i) {
      const Eigen::Vector2f direction=planes[i].cwiseQuotient(metric);
      considerPlane(input-direction*((planes[i].dot(input)-bounds[i])/planes[i].dot(direction)));
    }
    if(count==2) {
      const float det=planes[0].x()*planes[1].y()-planes[0].y()*planes[1].x();
      if(std::fabs(det)>1e-8f)
        considerPlane(Eigen::Vector2f((bounds[0]*planes[1].y()-planes[0].y()*bounds[1])/det,
          (planes[0].x()*bounds[1]-bounds[0]*planes[1].x())/det));
    }
  }
  if(planeFound && (planeOutput.array()>=lower.array()).all() &&
      (planeOutput.array()<=upper.array()).all()) {output=planeOutput;return true;}
  Eigen::Vector2f a[6]; float b[6]; int n=0;
  for (int i=0;i<count;++i) {
    const float norm=planes[i].norm();
    if (!planes[i].allFinite() || !std::isfinite(bounds[i]) || norm<1e-8f) return false;
    a[n]=planes[i]/norm; b[n++]=bounds[i]/norm;
  }
  a[n]=Eigen::Vector2f(1,0); b[n++]=upper.x();
  a[n]=Eigen::Vector2f(-1,0); b[n++]=-lower.x();
  a[n]=Eigen::Vector2f(0,1); b[n++]=upper.y();
  a[n]=Eigen::Vector2f(0,-1); b[n++]=-lower.y();
  float best=std::numeric_limits<float>::infinity(); bool found=false;
  auto consider = [&](const Eigen::Vector2f& candidate) {
    if (!candidate.allFinite()) return;
    for(int j=0;j<n;++j) if(a[j].dot(candidate)>b[j]+1e-5f) return;
    const Eigen::Vector2f delta=candidate-input;
    const float cost=delta.dot(metric.cwiseProduct(delta));
    if(cost<best){best=cost;output=candidate;found=true;}
  };
  consider(input);
  if(found) return true;
  for(int i=0;i<n;++i){
    const Eigen::Vector2f direction=a[i].cwiseQuotient(metric);
    consider(input-direction*((a[i].dot(input)-b[i])/a[i].dot(direction)));
    for(int j=0;j<i;++j){
      const float det=a[i].x()*a[j].y()-a[i].y()*a[j].x();
      if(std::fabs(det)<1e-8f) continue;
      consider(Eigen::Vector2f((b[i]*a[j].y()-a[i].y()*b[j])/det,
          (a[i].x()*b[j]-b[i]*a[j].x())/det));
    }
  }
  return found;
}
