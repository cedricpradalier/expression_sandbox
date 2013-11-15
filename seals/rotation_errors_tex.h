#ifndef ES_ROTATION_ERRORS_TEX_H
#define ES_ROTATION_ERRORS_TEX_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <iostream>
#include "typed_expressions/EigenLinalg.hpp"
#include "typed_expressions/EuclideanSpace.hpp"
#include "typed_expressions/UnitQuaternions.hpp"
#include "rotation_errors.h"

namespace cerise_tex {
  static Eigen::DiagonalMatrix<double, 3> S(cerise::MagnetometerErrorQuat::S[0], cerise::MagnetometerErrorQuat::S[1], cerise::MagnetometerErrorQuat::S[2]);

  void removeLocalQuaternionParametrization(const ceres::QuaternionParameterization & parametrization, const double* parameters, const Eigen::Matrix<double, 3, 3>& quatJ, double* jacobian) {
    //TODO fix ceres to accept 3x3 jacobians here
    double localParamJac[4 * 3];
    parametrization.ComputeJacobian(parameters, localParamJac);
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > jacMap(jacobian);

    /*
     * q(phi) = exp(phi) * q_0.
     * D_{phi=0} q = D(* q_0) * V.  with D(* (eps_0, eth_0) ) = [ (-eps_0^x + eth_0 * Id), eps_0; -eps_0^T eth_0 ]
     * => (D_{phi=0} q)^T = V^T * D(* q_0)^T = V^T * [ (eps_0^x + eth_0 * Id), -eps_0; eps_0^T eth_0 ] = V^T D(* conjugate(q_0))
     *
     * phi(q) = log(q / q_0)
     * D_{q = q_0} (phi) = V^T * D(* q_0^-1)
     */

    jacMap = quatJ * Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> >(localParamJac).transpose();
  }

  using namespace tex;

    struct SmoothnessConstraint : public ceres::CostFunction {
      SmoothnessConstraint(double weight)
          : weight(weight)
      {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(1);
      }

      // movement_parameters: 1D [ instantaneous propulsion acceleration: 1]
      virtual bool Evaluate(double const* const* parameters,
                            double* residuals,
                            double** jacobians) const {

        Diffable<Ref<Scalar<double>>, 0> movement_parameters_1(reinterpret_cast<const Scalar<double>&>(*parameters[0]));
        Diffable<Ref<Scalar<double>>, 1> movement_parameters_2(reinterpret_cast<const Scalar<double>&>(*parameters[1]));

        auto exp = Scalar<double>(weight) * (movement_parameters_2 - movement_parameters_1);
#ifndef NDEBUG
        std::cout << "exp(SmoothnessConstraint)=" << exp << std::endl; // XXX: debug output of exp
#endif

        if(jacobians){
          auto cache = createCache(exp);
          cache.update(exp);

          if(residuals)
            *residuals = cache.accessValue(exp);

          Eigen::Map<Eigen::Matrix<double, 1, 1>> J1(jacobians[0]);
          J1.setZero();
          evalFullDiffIntoCached(exp, movement_parameters_1, cache, J1);

          Eigen::Map<Eigen::Matrix<double, 1, 1>> J2(jacobians[1]);
          J2.setZero();
          evalFullDiffIntoCached(exp, movement_parameters_2, cache, J2);
        }
        else if(residuals){
            *residuals = evalExp(exp);
        }
        return true;
      }

      double weight;
    };

    struct AccelerometerErrorQuat : public ceres::CostFunction{
      AccelerometerErrorQuat(double depth, double a_x, double a_y, double a_z, double weight=1)
          : a({a_x, a_y, a_z}), g({cerise::AccelerometerErrorQuat::G[0], cerise::AccelerometerErrorQuat::G[1], cerise::AccelerometerErrorQuat::G[2] + cerise::AccelerometerErrorQuat::Kdepth * depth}), weight(weight)
      {
        set_num_residuals(3);
        mutable_parameter_block_sizes()->push_back(4);
        mutable_parameter_block_sizes()->push_back(1);
      }

      // movement_parameters: 5D [ Quaternion: 4, instantaneous propulsion acceleration: 1]

      virtual bool Evaluate(double const* const* parameters,
                            double* residuals,
                            double** jacobians) const {

          Eigen::Map<const Eigen::Matrix<double, 4, 1>> map(parameters[0]);
          UnitQuaternion q(map);
          Diffable<UnitQuaternion, 0> quaternion(q);

          Diffable<Ref<Scalar<double>>, 1> propulsion(reinterpret_cast<const Scalar<double>&>(*parameters[1]));

          // camera[0,1,2] are the angle-axis rotation.
          auto corrected_body_accel = a - (Ref<EuclideanPoint<3>>(EuclideanPoint<3>::getBasisVector<0>()) * propulsion);

          // The error is the difference between the predicted and a position.
          auto exp = (rotate(quaternion, corrected_body_accel) - g) * Scalar<double>(weight);

#ifndef NDEBUG
          std::cout << "exp(AccelerometerErrorQuat)=" << exp << std::endl; // XXX: debug output of exp
#endif

          assert(residuals);
          auto cache = createCache(exp, reinterpret_cast<EuclideanPoint<3> &>(*residuals));
          cache.update(exp);

          if(jacobians){
            Eigen::Matrix<double, 3, 3> quatJ;
//            Eigen::Map<Eigen::Matrix<double, 3, 3>> quatJ(jacobians[0]);
            quatJ.setZero();
            evalFullDiffIntoCached(exp, quaternion, cache, quatJ);
            removeLocalQuaternionParametrization(parametrization, parameters[0], quatJ, jacobians[0]);

            Eigen::Map<Eigen::Matrix<double, 3, 1>> jac(jacobians[1]);
            jac.setZero();
            evalFullDiffIntoCached(exp, propulsion, cache, jac);
          }
          return true;
        }

        EuclideanPoint<3> a, g;
        double weight;
        ceres::QuaternionParameterization parametrization;
    };

    struct MagnetometerErrorQuat : public ceres::CostFunction {

      MagnetometerErrorQuat(double m_x, double m_y, double m_z,
              double b_x, double b_y, double b_z,
              double weight)
          : b({b_x, b_y, b_z}),
          m({m_x, m_y, m_z}),
          weight(weight)
      {
        set_num_residuals(3);
        mutable_parameter_block_sizes()->push_back(4);
      }

      // movement_parameters: 5D [ Quaternion: 4]
      // common_parameters: 4D [ magnetometer scale: 3, buoyancy gain: 1]
      virtual bool Evaluate(double const* const* parameters,
                            double* residuals,
                            double** jacobians) const {

          Eigen::Map<const Eigen::Matrix<double, 4, 1>> map(parameters[0]);
          UnitQuaternion q(map);
          Diffable<UnitQuaternion, 0> quaternion(q);

          // camera[0,1,2] are the angle-axis rotation.
          auto corrected_body_mag = m && Ref<EuclideanPoint<3>>(reinterpret_cast<const EuclideanPoint<3> &>(cerise::MagnetometerErrorQuat::S));

//           The error is the difference between the predicted and a position.
          auto exp = (rotate(quaternion, corrected_body_mag) - b) * Scalar<double>(weight);
#ifndef NDEBUG
          std::cout << "exp(MagnetometerErrorQuat)=" << exp << std::endl; // XXX: debug output of exp
#endif

          if(jacobians){
            auto cache = createCache(exp, reinterpret_cast<EuclideanPoint<3> &>(*residuals));
            cache.update(exp);
            Eigen::Matrix<double, 3, 3> quatJ;
//            Eigen::Map<Eigen::Matrix<double, 3, 3>> quatJ(jacobians[0]);

            quatJ.setZero();
            evalFullDiffIntoCached(exp, quaternion, cache, quatJ);

//            auto & rot = cache.a.a.accessValue(exp.getA().getA());
//            double w2 = 2 * weight;
//            quatJ <<
//              0, w2 * rot[2], -w2 * rot[1],
//              -w2 * rot[2], 0, w2 * rot[0],
//              w2 * rot[1], -w2 * rot[0], 0;
            removeLocalQuaternionParametrization(parametrization, parameters[0], quatJ, jacobians[0]);
          }
          else if(residuals){
            Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::Unaligned>> resMap(residuals);
            resMap = evalExp(exp).getValue();
          }
          return true;
        }

        // Reference magnetic field
        EuclideanPoint<3> b;

        // Measured magnetic field
        EuclideanPoint<3> m;

        double weight;
        ceres::QuaternionParameterization parametrization;
    };
};


#endif // ES_ROTATION_ERRORS_TEX_H
