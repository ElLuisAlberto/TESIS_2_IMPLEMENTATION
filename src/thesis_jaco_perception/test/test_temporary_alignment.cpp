#include <gtest/gtest.h>
#include "thesis_jaco_perception/temporary_alignment.hpp"
using namespace thesis_jaco_perception;
namespace { std::vector<Capsule3> c(){return {{{0,0,0},{0,0,1},.08},{{.4,0,0},{.8,.2,.5},.08},{{.8,.1,0},{1,.5,.7},.08},{{1.2,-.2,0},{1.5,.1,.8},.08},{{1.5,.3,0},{1.8,.5,.6},.08},{{2,-.2,0},{2.2,.2,.7},.08}};} std::vector<Point3> p(const std::array<double,6>&t){std::vector<Point3>v;for(auto&a:c())for(int i=0;i<10;i++){double f=i/9.;v.push_back({a.start.x+(a.end.x-a.start.x)*f+a.radius-t[0],a.start.y+(a.end.y-a.start.y)*f-t[1],a.start.z+(a.end.z-a.start.z)*f-t[2]});}return v;} }
TEST(TemporaryAlignment, OnePoseIsNeverAccepted){std::array<double,6>t{.04,-.03,.02,0,0,0};auto r=align_temporary_cloud(p(t),c(),{});EXPECT_FALSE(r.accepted);EXPECT_EQ(r.reason,"INSUFFICIENT_DISTINCT_POSES");}
TEST(TemporaryAlignment, RejectsInsufficientCloud){std::vector<Point3>v(3);auto r=align_temporary_cloud(v,c(),{});EXPECT_FALSE(r.accepted);EXPECT_EQ(r.reason,"INSUFFICIENT_POINTS");}
TEST(TemporaryAlignment, LimitIsNeverAccepted){std::array<double,6>t{.3,0,0,0,0,0};auto r=align_temporary_cloud(p(t),c(),{});EXPECT_FALSE(r.accepted);}
