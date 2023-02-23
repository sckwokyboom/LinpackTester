#ifndef LINPACKTESTER_HPP_SIMPLEITERATIONPARALLELTESTER_HPP
#define LINPACKTESTER_HPP_SIMPLEITERATIONPARALLELTESTER_HPP

#include <mpi.h>
#include <chrono>
#include "LinpackTester.hpp"

class SimpleIterationParallelTester : public LinpackTester {
private:
  const int matrixHeight_;
  long long floatOperationsCounter = 0;
  const double epsilon_;
  const double tau_;
  Matrix A;
  Matrix x;
  Matrix b;
  double normVector_b = 0;
  int rank_ = 0;
  char *isNeedToTerminate = nullptr;
public:
  explicit SimpleIterationParallelTester(const int &matrixSize = 500,
                                         const double &epsilon = 0.1, const double &tau = 0.0001) :
          matrixHeight_(matrixSize), epsilon_(epsilon), tau_(tau),
          A(Matrix(matrixSize, matrixSize, 't')), x(Matrix(matrixSize, 1, 'n')), b(Matrix(matrixSize, 1, 'h')) {
    MPI_Comm_rank(MPI_COMM_WORLD, &rank_);
    A.scatterMatrixData();
    b.broadCastMatrixData();
    x.broadCastMatrixData();
    normVector_b = b.calculateNormOfVector();
    isNeedToTerminate = new char(0);
  }

  ~SimpleIterationParallelTester() override {
   delete isNeedToTerminate;
  }

  double invokeTest() override {
    // The unique number of the current process in the MPI_COMM_WORLD communicator.


    // The total number of processes in the MPI_COMM_WORLD communicator.
    int commSize;
    MPI_Comm_size(MPI_COMM_WORLD, &commSize);


    auto t1 = std::chrono::high_resolution_clock::now();
    while (!isTerminationCriterion()) {
//      Matrix first = A * x;
      // 2 * other.width * other.height * height_
      floatOperationsCounter += 2 * matrixHeight_ * matrixHeight_ - matrixHeight_;
//      Matrix second = first - b;
      floatOperationsCounter += matrixHeight_;
//      Matrix thirst = tau_ * second;
      floatOperationsCounter += matrixHeight_;
//      x -= thirst;
      floatOperationsCounter += matrixHeight_;
      x -= (tau_ * ((A * x) - b));
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    double flopsResult = (double) floatOperationsCounter / ms_double.count();
    floatOperationsCounter = 0;
//    x = Matrix(matrixHeight_, 1, 'n');
    return flopsResult;
  }

  bool isTerminationCriterion() override {
//    Matrix first = A * x;
    floatOperationsCounter += 2 * matrixHeight_ * matrixHeight_ - matrixHeight_;
//    first.scatterMatrixData();
//    Matrix second = first - b;
    Matrix first = A * x - b;
    first.gatherMatrixData();
    floatOperationsCounter += matrixHeight_;

//    double terminationCriterionNumber = (((*A) * (*x)) - *b).calculateNormOfVector() / b->calculateNormOfVector();

    if (rank_ == 0) {
      double terminationCriterionNumber = (first.calculateNormOfVector()) / normVector_b;
      std::cout << terminationCriterionNumber << std::endl;
      floatOperationsCounter += 3 * matrixHeight_ + 1;
      if (terminationCriterionNumber < epsilon_) {
        *isNeedToTerminate = 1;
      }
    }
    MPI_Bcast(isNeedToTerminate, 1, MPI_CHAR, 0, MPI_COMM_WORLD);
    return *isNeedToTerminate;
  }
};

#endif //LINPACKTESTER_HPP_SIMPLEITERATIONPARALLELTESTER_HPP
