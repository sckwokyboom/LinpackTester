#ifndef LINPACKTESTER_HPP_SIMPLEITERATIONTESTER_HPP
#define LINPACKTESTER_HPP_SIMPLEITERATIONTESTER_HPP

#include <chrono>
#include "LinpackTester.hpp"

class SimpleIterationTester : public LinpackTester {
private:
  const long long matrixHeight_;
  long long floatOperationsCounter = 0;
  const double epsilon_;
  const double tau_;
  Matrix A;
  Matrix x;
  Matrix b;
public:
  explicit SimpleIterationTester(long long int matrixSize1, const long long int &matrixSize = 100,
                                 const double &epsilon = 0.000001, const double &tau = 0.00001) : LinpackTester(
          matrixSize1), matrixHeight_(matrixSize), epsilon_(epsilon), tau_(tau), A(Matrix(matrixSize, matrixSize,
                                                                                          't')),
                                                                                                  x(Matrix(
                                                                                                          matrixSize,
                                                                                                          1, 'n')),
                                                                                                  b(Matrix(
                                                                                                          matrixSize,
                                                                                                          1, 'h')) {

  }

  ~SimpleIterationTester() override = default;

  double invokeTest() override {
    auto t1 = std::chrono::high_resolution_clock::now();
    while (!isTerminationCriterion()) {
      Matrix first = A * x;
      // 2 * other.width * other.height * height_
      floatOperationsCounter += 2 * matrixHeight_ * matrixHeight_ - matrixHeight_;
      Matrix second = first - b;
      floatOperationsCounter += matrixHeight_;
      Matrix thirst = tau_ * second;
      floatOperationsCounter += matrixHeight_;
      x -= thirst;
      floatOperationsCounter += matrixHeight_;
//      *x = (*x) - (tau_ * (((*A) * (*x)) - *b));
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    double flopsResult = (double) floatOperationsCounter / ms_double.count();
    floatOperationsCounter = 0;
    x = Matrix(matrixHeight_, 1, 'n');
    return flopsResult;
  }

  bool isTerminationCriterion() override {
    Matrix first = A * x;
    floatOperationsCounter += 2 * matrixHeight_ * matrixHeight_ - matrixHeight_;
    Matrix second = first - b;
    floatOperationsCounter += matrixHeight_;

//    double terminationCriterionNumber = (((*A) * (*x)) - *b).calculateNormOfVector() / b->calculateNormOfVector();
    double terminationCriterionNumber = (second.calculateNormOfVector()) / (b.calculateNormOfVector());
    floatOperationsCounter += 3 * matrixHeight_ + 1;
    if (terminationCriterionNumber < epsilon_) {
      return true;
    }
    return false;
  }
};

#endif //LINPACKTESTER_HPP_SIMPLEITERATIONTESTER_HPP
