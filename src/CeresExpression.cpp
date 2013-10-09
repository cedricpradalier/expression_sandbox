
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace ceres;

struct TestErrorQuat {
    TestErrorQuat(double u, double v, double weight=1.0) 
        : u(u), v(v), weight(weight) 
    {
    }



    template <typename T>
        bool operator()(const T* const q_1_2,
                const T* const q_2_3,
                const T* const P,
                T* residuals) const {
            T p1[3], p2[3];
            ceres::QuaternionRotatePoint(q_2_3, P, p1);

            // A nice function to do that would be cleaner, but...
            T q_1_2_inv[4] = {q_1_2[0],-q_1_2[1],-q_1_2[2],-q_1_2[3]};
            ceres::QuaternionRotatePoint(q_2_3_inv, p1, p2);

            // The error is the difference between the predicted and a position.
            residuals[0] = T(weight)*(T(u) - p[0]/p[2]);
            residuals[1] = T(weight)*(T(v) - p[1]/p[2]);

            return true;
        }

    double u;
    double v;
    double weight;

    static void test_functor() {
        double q_1_2[4] = {1,0,0,0};
        double q_2_3[4] = {1,0,0,0};
        double p_3[3] = {1,0,0};
        double res[2] = {5.0, 2.0};

        double *parameters[] = {q_1_2,q_2_3,p_3};
        double J_q_1_2[8];
        double J_q_2_3[8];
        double J_p_3[6];
        double *jacobians[] = {J_q_1_2,J_q_2_3,J_p_3};

        CostFunction cost_function = new AutoDiffCostFunction<TestErrorQuat,2,4,4,3>(
                new TestErrorQuat(5.0,2.0,1.0));

        bool success = cost_function->Evaluate(parameters, res, jacobians);

        delete cost_function;
    }
};

// Same with local parameterization (not in the ceres way though)
struct TestErrorAngleAxis : public TestErrorQuat{
    TestErrorAngleAxis(double u, double v, double weight=1.0) 
        : TestErrorQuat(u,v,weight) { }

    template <typename T>
        bool operator()(const T* const aa_1_2,
                const T* const aa_2_3,
                const T* const P,
                T* residuals) const {
            T q_1_2[4], q_2_3[4];
            ceres::AngleAxisToQuaternion(aa_1_2,q_1_2);
            ceres::AngleAxisToQuaternion(aa_2_3,q_2_3);
            return TestErrorQuat::operator()(q_1_2,q_2_3,P,residuals);
        }

    static void test_functor() {
        double aa_1_2[3] = {0,0,0};
        double aa_2_3[3] = {0,0,0};
        double p_3[3] = {1,0,0};
        double res[2] = {5.0, 2.0};

        double *parameters[] = {aa_1_2,aa_2_3,p_3};
        double J_aa_1_2[6];
        double J_aa_2_3[6];
        double J_p_3[6];
        double *jacobians[] = {J_aa_1_2,J_aa_2_3,J_p_3};

        CostFunction cost_function = new AutoDiffCostFunction<TestErrorAngleAxis,2,3,3,3>(
                new TestErrorAngleAxis(5.0,2.0,1.0));

        bool success = cost_function->Evaluate(parameters, res, jacobians);

        delete cost_function;
    }
};

int main() 
{
    TestErrorQuat::test_functor();
    TestErrorAngleAxis::test_functor();

    return 0;
}

