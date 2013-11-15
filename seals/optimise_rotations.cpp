
#include <type_traits>
#include <stdlib.h>
#include <stdio.h>
#ifdef HAS_CGNUPLOT
#include "cgnuplot/CGnuplot.h"
#endif
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include <benchmark/StopWatch.hpp>


#define Repetitions 1

//#define USE_CERES_AD 1
//#define COMPARE_WITH_CERES 1
//#define COMPARE_VALUES 1
//#define MEASURE_IN_REPEAT 1

#if USE_CERES_AD
#include "rotation_errors.h"
#else
#include "rotation_errors_tex.h"
#endif
#include "states.h"

DEFINE_string(input, "", "Input File name");
DEFINE_string(output, "", "Input File name");
DEFINE_string(gps, "", "GPS File name");
DEFINE_bool(raw, false, "Use a raw input file (11 columns)");
DEFINE_string(trust_region_strategy, "levenberg_marquardt",
              "Options are: levenberg_marquardt, dogleg.");
DEFINE_string(dogleg, "traditional_dogleg", "Options are: traditional_dogleg,"
              "subspace_dogleg.");

DEFINE_bool(inner_iterations, false, "Use inner iterations to non-linearly "
            "refine each successful trust region step.");

DEFINE_string(blocks_for_inner_iterations, "automatic", "Options are: "
            "automatic, cameras, points, cameras,points, points,cameras");

DEFINE_string(linear_solver, "sparse_normal_cholesky", "Options are: "
              "sparse_schur, dense_schur, iterative_schur, sparse_normal_cholesky, "
              "dense_qr, dense_normal_cholesky and cgnr.");

DEFINE_string(preconditioner, "jacobi", "Options are: "
              "identity, jacobi, schur_jacobi, cluster_jacobi, "
              "cluster_tridiagonal.");

DEFINE_string(sparse_linear_algebra_library, "suite_sparse",
              "Options are: suite_sparse and cx_sparse.");

DEFINE_string(ordering, "automatic", "Options are: automatic, user.");

DEFINE_bool(robustify, false, "Use a robust loss function.");
DEFINE_bool(interactive, false, "Wait for user key presses.");
DEFINE_bool(display, false, "Display plot during progress");

DEFINE_double(ftol, 1e-7, "Function tolerance");
DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines the "
             "accuracy of each linear solve of the truncated newton step. "
             "Changing this parameter can affect solve performance.");

DEFINE_bool(use_local_parameterization, false, "For quaternions, use a local "
            "parameterization.");

DEFINE_int32(num_lines, -1, "Number of data lines to consider.");
DEFINE_int32(num_threads, 1, "Number of threads.");
DEFINE_int32(num_iterations, 100, "Number of iterations.");
DEFINE_double(max_solver_time, 1e32, "Maximum solve time in seconds.");
DEFINE_bool(nonmonotonic_steps, false, "Trust region algorithm can use"
            " nonmonotic steps.");
DEFINE_bool(mag,false,"Estimate the magnetic field vector");

DEFINE_string(solver_log, "", "File to record the solver execution to.");


using namespace ceres;


struct Durations {
  stop_watch::StopWatch::Duration aDuration, bDuration, nopDuration;
  friend ostream & operator << (ostream & out, const Durations & p){
    out << "a=" << p.aDuration - p.nopDuration << std::endl << "b=" << p.bDuration - p.nopDuration << std::endl<< "nop=" << p.nopDuration;
    return out;
  }
} smoothnessDuration, magnetometerDuration, accelerometerDuration;

stop_watch::StopWatch::Duration jacobiansDuration;
stop_watch::StopWatch::Duration residualsDuration;

stop_watch::StopWatch stopWatch;



struct RepeatingCostFunction : public CostFunction {
  RepeatingCostFunction(CostFunction * a) : a(a){
    set_num_residuals(a->num_residuals());
    *mutable_parameter_block_sizes() = a->parameter_block_sizes();
  }

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {

#if MEASURE_IN_REPEAT
    stopWatch.reset();
#endif
    for(int i = Repetitions; i > 0; i--){
      if(!a->Evaluate(parameters, residuals, jacobians)){
        return false;
      }
    }
#if MEASURE_IN_REPEAT
    stop_watch::StopWatch::Duration & d = jacobians ? jacobiansDuration : residualsDuration;
    d += stopWatch.readAndReset();
    stopWatch.reset(); // subtract clock overhead
    d -= stopWatch.readAndReset();
#endif
    return true;
  }
  virtual ~RepeatingCostFunction() {
    delete a;
  }
 private:
  CostFunction *a;
};

inline CostFunction * wrapRepeatingCostFunction(CostFunction *c){
#if Repetitions > 1 || MEASURE_IN_REPEAT
  return new RepeatingCostFunction(c);
#else
  return c;
#endif
}



struct ComparingCostFunction : public CostFunction {


  ComparingCostFunction(CostFunction * a, CostFunction * b, Durations & durationPair) : a(a), b(b), durationPair(durationPair){
    set_num_residuals(a->num_residuals());
    CHECK_EQ(num_residuals(), b->num_residuals());
    *mutable_parameter_block_sizes() = a->parameter_block_sizes();
    int i = 0;
    for(int16 s : parameter_block_sizes())
      CHECK_EQ(s, b->parameter_block_sizes()[i++]);
  }

  static void checkDoublesNear(const double * a, const double * b, size_t size){
    for(size_t i = 0 ; i< size; i++ ){
      CHECK_NEAR(a[i], b[i], 1E-9);
    }
  }

  static void printDoubles(ostream & out, const double * a, size_t size){
    out << "[";
    for(size_t i = 0 ; i< size; i++ ){
      if(i) out << ", ";
      out << a[i];
    }
    out << "]";
  }


  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {

    stopWatch.reset();
    if(!a->Evaluate(parameters, residuals, jacobians)){
      return false;
    }
    durationPair.aDuration += stopWatch.readAndReset();

    double bResiduals[num_residuals()];
    double *bJacobiansData[parameter_block_sizes().size()], **bJacobians = jacobians ? bJacobiansData : nullptr;
    int i = 0;
    if(jacobians){
      for(int16 s : parameter_block_sizes()){
        bJacobians[i] = jacobians[i] ? new double[num_residuals() * s] : nullptr;
        i++;
      }
    }
    stopWatch.reset();
    if(!b->Evaluate(parameters, bResiduals, bJacobians)){
      return false;
    }
    durationPair.bDuration += stopWatch.readAndReset();
    stopWatch.reset();
    durationPair.nopDuration += stopWatch.readAndReset();


#if COMPARE_VALUES
    std::cout << "residuals=";
    printDoubles(std::cout, residualsDuration, num_residuals());
    std::cout << std::endl; // XXX: debug output of residualsDuration

    std::cout << "bResiduals=";
    printDoubles(std::cout, bResiduals, num_residuals());
    std::cout << std::endl; // XXX: debug output of residualsDuration
    checkDoublesNear(residualsDuration, bResiduals, num_residuals());

    if(jacobiansDuration){
      i = 0;
      for(auto & jBOrg : bJacobiansData){
        if(jBOrg){
          int16 s = num_residuals() * parameter_block_sizes()[i];
          double * jA = jacobiansDuration[i], * jB = jBOrg;
          double localParamJac[4 * 3], jAS[3 * 3], jBS[3 * 3];
          if(s == 3 * 4){
            s = 3 * 3;
            ceres::QuaternionParameterization().ComputeJacobian(parameters[0], localParamJac);

            Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > locParamJac(localParamJac);

            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > jacMapA(jA);
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > jacMapB(jB);

            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > jASMap(jAS);
            jASMap = jacMapA * locParamJac;
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > jBSMap(jBS);
            jBSMap = jacMapB * locParamJac;

            jA = jAS;
            jB = jBS;
          }

          std::cout << "jA(" << i << ")=";
          printDoubles(std::cout, jA, s);
          std::cout << std::endl; // XXX: debug output of residualsDuration

          std::cout << "jB(" << i << ")=";
          printDoubles(std::cout, jB, s);
          std::cout << std::endl; // XXX: debug output of residualsDuration

          checkDoublesNear(jA, jB, s);
        }
        i++;
      }

    }
#endif
    if(bJacobians)
    for(auto & jBOrg : bJacobiansData){
      delete jBOrg;
    }
    return true;
  }
  virtual ~ComparingCostFunction() {
    delete a;
    delete b;
  }
 private:
  CostFunction *a, *b;
  Durations & durationPair;
};


#if COMPARE_WITH_CERES
#define CHECK_COST(a, b, durationPair) new ComparingCostFunction(a, b, durationPair)
#else
#define CHECK_COST(a, b, durationPair) a;
#endif


namespace cerise{ 
    class OptimiseRotation {
     public:
      virtual ~ OptimiseRotation() {}
        protected:
#ifdef HAS_CGNUPLOT
            cgnuplot::CGnuplot G;
#endif

            class DisplayCallback: public ceres::IterationCallback { 
                protected:
                    OptimiseRotation & problem;
                public: 
                    DisplayCallback(OptimiseRotation & p) : problem(p) {}
                    virtual ceres::CallbackReturnType operator()(const 
                            ceres::IterationSummary& summary) { 
                        problem.reportProgress(true);
                        // if (FLAGS_interactive) {
                        //     getchar();
                        // }
                        return ceres::SOLVER_CONTINUE;
                    } 
            };

            DisplayCallback display;
            std::vector<DataLine> lines; 
            typedef std::map<double,GPSLine> GPSMap; 
            GPSMap gpslines; 
            OptimisedOrientationSequence oos;


            Problem problem;
            void SetLinearSolver(Solver::Options* options) {
                CHECK(StringToLinearSolverType(FLAGS_linear_solver,
                            &options->linear_solver_type));
                CHECK(StringToPreconditionerType(FLAGS_preconditioner,
                            &options->preconditioner_type));
                CHECK(StringToSparseLinearAlgebraLibraryType(
                            FLAGS_sparse_linear_algebra_library,
                            &options->sparse_linear_algebra_library_type));
                options->num_linear_solver_threads = FLAGS_num_threads;
            }

            void SetMinimizerOptions(Solver::Options* options) {
                options->max_num_iterations = FLAGS_num_iterations;
                options->minimizer_progress_to_stdout = true;
                options->num_threads = FLAGS_num_threads;
                options->eta = FLAGS_eta;
                options->function_tolerance = FLAGS_ftol;
                options->max_solver_time_in_seconds = FLAGS_max_solver_time;
                options->use_nonmonotonic_steps = FLAGS_nonmonotonic_steps;
                if (FLAGS_display) {
                    options->callbacks.push_back(&display);
                }
                options->update_state_every_iteration = true; 

                CHECK(StringToTrustRegionStrategyType(FLAGS_trust_region_strategy,
                            &options->trust_region_strategy_type));
                CHECK(StringToDoglegType(FLAGS_dogleg, &options->dogleg_type));
                options->use_inner_iterations = FLAGS_inner_iterations;
            }

            const GPSLine & getClosestGPSFix(double ts) {
                assert(gpslines.size()>0);
                GPSMap::const_iterator it2 = gpslines.upper_bound(ts);

                if (it2 == gpslines.begin()) {
                    return it2->second;
                }

                GPSMap::const_iterator it1 = it2;
                it1 --;
                if (it2 == gpslines.end())  {
                    return it1->second;
                }
                if (fabs(it1->first-ts) > fabs(it2->first-ts)) {
                    return it2->second;
                } else {
                    return it1->second;
                }
            }

        public:
            OptimiseRotation() : display(*this) {}

            void optimise() {
                Solver::Options options;
                SetMinimizerOptions(&options);
                SetLinearSolver(&options);
                Solver::Summary summary;
                Solve(options, &problem, &summary);
                std::cout << summary.FullReport() << "\n";
                if (!FLAGS_output.empty()) {
                    oos.save(FLAGS_output);
                }
                reportProgress();
                // sleep(3);
            }

            bool load(const char * filename, const char *gps, int lineLimit=-1) {
                // Loading the data file
                lines.clear();
                FILE * fp = fopen(filename,"r");
                if(!fp) return false;
                while (!feof(fp)) {
                    if ((lineLimit>0) && ((signed)lines.size() >= lineLimit)) {
                        break;
                    }
                    char line[4096] = {0,};
                    std::vector<double> dline;
                    if (fgets(line,4095, fp)!= NULL) {
                        DataLine dl;
                        if (dl.load(line)) {
                            lines.push_back(dl);
                        }
                    }
                }
                fclose(fp);
                printf("Loaded %d lines\n",(int)lines.size());
                // prepare a map between dive_ids and variable numbers
                std::map<size_t,size_t> dive_ids;
                for (size_t i=0;i<lines.size();i++) {
                    if (dive_ids.find(lines[i].dive_id) == dive_ids.end()) {
                        dive_ids[lines[i].dive_id] = dive_ids.size();
                    }
                }

                // Loading GPS data, mostly to get the expected magnetic field
                fp = fopen(gps,"r");
                if (!fp) {
                    LOG(ERROR) << "Couldn't load gps file '" << gps << "'\n";
                    return false;
                }
                while (!feof(fp)) {
                    char line[4096] = {0,};
                    if (fgets(line,4095, fp)!= NULL) {
                        GPSLine dl;
                        if (dl.load(line)) {
                            gpslines.insert(GPSMap::value_type(dl.timestamp,dl));
                        }
                    }
                }
                fclose(fp);
                printf("Loaded %d gps lines\n",(int)gpslines.size());
                assert(gpslines.size() >= 2);

                // Now build CERES problem
                LocalParameterization* quaternion_parameterization = NULL;
                oos.initialise(filename, lines);
                if (FLAGS_use_local_parameterization) {
                    quaternion_parameterization = new QuaternionParameterization;
                }
                for (size_t i=0;i<lines.size();i++) {
                    DataLine & dl(lines[i]);
                    const GPSLine & gps = getClosestGPSFix(dl.timestamp);
                    OptimisedOrientation & oo(oos.states[i]);

                    LossFunction* loss_function;
                    CostFunction* cost_function;
                    // Acceleration
                    loss_function = FLAGS_robustify ? new HuberLoss(1.0) : NULL;
#if USE_CERES_AD || COMPARE_WITH_CERES
                    cost_function = wrapRepeatingCostFunction(new AutoDiffCostFunction<cerise::AccelerometerErrorQuat,3,4,1>(
                            new cerise::AccelerometerErrorQuat(dl.depth,dl.a[0],dl.a[1],dl.a[2], 100.0)));
#endif
#if !USE_CERES_AD || COMPARE_WITH_CERES
                    cost_function = CHECK_COST(wrapRepeatingCostFunction(new cerise_tex::AccelerometerErrorQuat(dl.depth,dl.a[0],dl.a[1],dl.a[2], 100.0)), cost_function, accelerometerDuration);
#endif
                    problem.AddResidualBlock(cost_function,loss_function, oo.rotation,oo.propulsion);

                    // Magnetic field
                    loss_function = FLAGS_robustify ? new HuberLoss(100.0) : NULL;
#if USE_CERES_AD || COMPARE_WITH_CERES
                   cost_function = wrapRepeatingCostFunction(new AutoDiffCostFunction<cerise::MagnetometerErrorQuat,3,4>(
                            new cerise::MagnetometerErrorQuat(dl.m[0],dl.m[1],dl.m[2], 
                                gps.B[0],gps.B[1],gps.B[2], 0.1)));
#endif
#if !USE_CERES_AD || COMPARE_WITH_CERES
                   cost_function = CHECK_COST(wrapRepeatingCostFunction(new cerise_tex::MagnetometerErrorQuat(dl.m[0],dl.m[1],dl.m[2], gps.B[0],gps.B[1],gps.B[2], 0.1)), cost_function, magnetometerDuration);
#endif
                    problem.AddResidualBlock(cost_function,loss_function,oo.rotation);

                    if (i>0) {
                        // Smoothness constraint
#if USE_CERES_AD || COMPARE_WITH_CERES
                        cost_function = wrapRepeatingCostFunction(new AutoDiffCostFunction<cerise::SmoothnessConstraint,1,1,1>(
                                new cerise::SmoothnessConstraint(5e1)));
#endif
#if !USE_CERES_AD || COMPARE_WITH_CERES
                        cost_function = CHECK_COST(wrapRepeatingCostFunction(new cerise_tex::SmoothnessConstraint(5e1)), cost_function, smoothnessDuration);
#endif

                        problem.AddResidualBlock(cost_function,NULL,oos.states[i-1].propulsion, oo.propulsion);
                    }
                    if (FLAGS_use_local_parameterization) {
                        problem.SetParameterization(oo.rotation, quaternion_parameterization);
                    }
                }
                return true;
            }

#if 1

            virtual void reportProgress(bool intermediate=false) {
                oos.save("X");
                FILE *fa = fopen("Ap","w"), *fb = fopen("Bp","w");
                for (size_t i=0;i<lines.size();i++) {
                    DataLine & dl(lines[i]);
                    OptimisedOrientation & oo(oos.states[i]);
                    double As[3], Ap[3], Bs[3], Bp[3];
                    double t = (dl.timestamp - lines[0].timestamp)*24*3600;
                    for (size_t j=0;j<3;j++) {
                        As[j] = dl.a[j];
                        Bs[j] = dl.m[j] * MagnetometerErrorQuat::S[j];
                    }
                    As[0] -= oo.propulsion[0]; // remove propulsion
                    QuaternionRotatePoint(oo.rotation,As,Ap);
                    Ap[2] -= AccelerometerErrorQuat::Kdepth*dl.depth;
                    QuaternionRotatePoint(oo.rotation,Bs,Bp);

                    fprintf(fa,"%e %e %e %e\n",t,Ap[0],Ap[1],Ap[2]);
                    fprintf(fb,"%e %e %e %e\n",t,Bp[0],Bp[1],Bp[2]);
                }
                fclose(fa); fclose(fb); 
#ifdef HAS_CGNUPLOT
                if (FLAGS_display) {
                    G.plot("set terminal x11 1;set grid");
                    G.plot("plot \"Bp\" u 1:2 w l, \"Bp\" u 1:3 w l, \"Bp\" u 1:4 w l");
                    if (!intermediate) {
                        G.plot("set terminal x11 3;set grid");
                        int n = FLAGS_num_lines;
                        if (n <= 0) n = lines.size();
                        int depth_col=9;
                        if (FLAGS_raw) {
                            depth_col=3;
                        }
                        G.plot("plot [0:%d] \"X\" u 0:6 w l, \"%s\" u 0:($%d/500) w l",n,oos.input_file.c_str(),depth_col);
                    } 
                    G.plot("set terminal x11 0;set grid");
                    G.plot("plot \"Ap\" u 1:2 w l, \"Ap\" u 1:3 w l, \"Ap\" u 1:4 w l");
                }
#endif
            }
#endif
    };
};


int main(int argc, char *argv[])
{
#if USE_CERES_AD
      std::cout << "Using ceres AD" << std::endl;
#else
      std::cout << "Using tex AD" << std::endl;
#endif

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    if (FLAGS_input.empty()) {
        LOG(ERROR) << "Usage: optimise_rotation --input=preload.txt" << endl << "\twhere preload has been prepared by the prepare.m matlab script\n";
        return 1;
    }



    cerise::OptimiseRotation problem;
    if(problem.load(FLAGS_input.c_str(),FLAGS_gps.c_str(),FLAGS_num_lines)) {
      problem.optimise();

#if COMPARE_WITH_CERES
      std::cout << "smoothnessDuration=" << std::endl << smoothnessDuration << std::endl;
      std::cout << "accelerometerDuration=" << std::endl << accelerometerDuration << std::endl;
      std::cout << "magnetometerDuration=" << std::endl << magnetometerDuration << std::endl;
#endif

#if MEASURE_IN_REPEAT
      std::cout << "jacobiansDuration " << jacobiansDuration << std::endl;
      std::cout << "residualsDuration " << residualsDuration << std::endl;
#endif

      if (FLAGS_interactive) {
          getchar();
      }
    } else {
      perror("could not load input");
    }

    return 0;
}


