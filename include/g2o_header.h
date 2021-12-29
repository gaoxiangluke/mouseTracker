#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
// state vertex x,y
class StateVertex : public g2o::BaseVertex<3, SE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
    _estimate = SE2();
  }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        Eigen::Matrix<double, 3, 1> update_eigen;
        update_eigen << update[0], update[1], update[2];
        _estimate = SE2::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }
};


// error-edge, error type and vertex type
class PoseEdge : public g2o::BaseUnaryEdge<2,Eigen::Vector2d,StateVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseEdge(Eigen::Vector2d pos) :_pos2d(pos){}

  // define error compuatation module
  virtual void computeError() override {
    const  StateVertex *v = static_cast<const  StateVertex *> (_vertices[0]);
	SE2 T = v->estimate();
	Eigen::Vector2d current;
	current = T *_pos2d;
	_error = _measurement - current;
  }
  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  Eigen::Vector2d _pos2d;  // x,y measurement
};
