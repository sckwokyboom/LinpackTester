#include <iostream>
#include "SimpleIterationParallelTester.hpp"
#include "mpi.h"

int main(int argc, char **argv) {
  int rank;
  MPI_Init(nullptr, nullptr);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  SimpleIterationParallelTester tester;
  std::cout << tester.invokeTest() << " FLOpS " << "in process " << rank << std::endl;
  MPI_Finalize();

  return 0;
}