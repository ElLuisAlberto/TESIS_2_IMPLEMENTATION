#include "thesis_jaco_perception/temporary_alignment.hpp"
#include <ceres/ceres.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace thesis_jaco_perception { namespace {
constexpr double kMaxTranslation=0.15, kMaxAngle=20.0*M_PI/180.0, kTruncation=0.08;
Point3 transform(const Point3 &p,const double *x) {
  const double cx=cos(x[3]),sx=sin(x[3]),cy=cos(x[4]),sy=sin(x[4]),cz=cos(x[5]),sz=sin(x[5]);
  return {x[0]+cz*cy*p.x+(cz*sy*sx-sz*cx)*p.y+(cz*sy*cx+sz*sx)*p.z,
          x[1]+sz*cy*p.x+(sz*sy*sx+cz*cx)*p.y+(sz*sy*cx-cz*sx)*p.z,
          x[2]-sy*p.x+cy*sx*p.y+cy*cx*p.z};
}
double residual(const Point3&p,const std::vector<Capsule3>&c) {
  double best=std::numeric_limits<double>::infinity();
  for(const auto&q:c) best=std::min(best,std::abs(point_segment_distance(p,q.start,q.end)-q.radius));
  return std::min(best,kTruncation);
}
struct Cost { Point3 p; const std::vector<Capsule3> &c; bool operator()(const double*x,double*r)const {r[0]=residual(transform(p,x),c);return true;} };
double cost(const std::vector<Point3>&p,const std::vector<Capsule3>&c,const double*x){double s=0;for(auto&q:p){auto r=residual(transform(q,x),c);s+=r*r;}return sqrt(s/p.size());}
}  // namespace
TemporaryAlignmentResult align_temporary_cloud(const std::vector<Point3>&p,const std::vector<Capsule3>&c,const std::array<double,6>&init){
  TemporaryAlignmentResult out; out.points_used=p.size(); if(p.size()<40){out.reason="INSUFFICIENT_POINTS";return out;} if(c.size()!=6){out.reason="INVALID_CAPSULES";return out;}
  double x[6]; for(int i=0;i<6;i++) x[i]=init[i]; out.initial_cost=cost(p,c,x);
  ceres::Problem problem; for(const auto&q:p) problem.AddResidualBlock(new ceres::NumericDiffCostFunction<Cost,ceres::CENTRAL,1,6>(new Cost{q,c}),new ceres::HuberLoss(0.02),x);
  for(int i=0;i<3;i++){problem.SetParameterLowerBound(x,i,init[i]-kMaxTranslation);problem.SetParameterUpperBound(x,i,init[i]+kMaxTranslation);} for(int i=3;i<6;i++){problem.SetParameterLowerBound(x,i,init[i]-kMaxAngle);problem.SetParameterUpperBound(x,i,init[i]+kMaxAngle);}
  ceres::Solver::Options o;o.max_num_iterations=80;o.num_threads=1;o.linear_solver_type=ceres::DENSE_QR;o.minimizer_progress_to_stdout=false; ceres::Solver::Summary s;ceres::Solve(o,&problem,&s); out.final_cost=cost(p,c,x);for(int i=0;i<6;i++)out.pose[i]=x[i];
  bool boundary=false;for(int i=0;i<6;i++){double lim=i<3?kMaxTranslation:kMaxAngle;if(std::abs(x[i]-init[i])>=lim-1e-5)boundary=true;} if(!s.IsSolutionUsable())out.reason="NOT_CONVERGED";else if(boundary)out.reason="AT_LIMIT";else if(!(out.final_cost+0.005<out.initial_cost))out.reason="NO_IMPROVEMENT";else if(out.final_cost>0.035)out.reason="LOW_CONFIDENCE";else out.reason="INSUFFICIENT_DISTINCT_POSES";return out;
}
}  // namespace thesis_jaco_perception
