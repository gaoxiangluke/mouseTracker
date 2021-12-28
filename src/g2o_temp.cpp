include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;

// state vertex x,y
class StateVertex : public g2o::BaseVertex<4, SE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override { _estimate = SE2(); }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        Eigen::Vector4d update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4],
            update[5];
        _estimate = SE2::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }
};


// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class PoseEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d,StateVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseEdge(Eigen::Vector2d pos) :_pos2d(pose) {}

  // 计算曲线模型误差
  virtual void computeError() override {
    const  StateVertex *v = static_cast<const  StateVertex *> (_vertices[0]);
    const Eigen::Vector3d T = v->estimate();
    _error(0, 0) = _measurement - T * _pos2d;
  }

//   // 计算雅可比矩阵
//   virtual void linearizeOplus() override {
//     const StateVertex *v = static_cast<const  StateVertex *> (_vertices[0]);
//     const Eigen::Vector3d abc = v->estimate();
//     double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
//     _jacobianOplusXi[0] = -_x * _x * y;
//     _jacobianOplusXi[1] = -_x * y;
//     _jacobianOplusXi[2] = -y;
//   }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  double _x;  // x 值， y 值为 _measurement
};
