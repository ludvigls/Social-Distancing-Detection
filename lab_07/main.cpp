#include "social_distancing.h"
#include <iostream>

int main() try
{
  socialDistancing();

  return EXIT_SUCCESS;
}
catch (const std::exception& e)
{
  std::cerr << "Caught exception:\n"
            << e.what() << "\n";
  return EXIT_FAILURE;
}
catch (...)
{
  std::cerr << "Caught unknown exception\n";
  return EXIT_FAILURE;
}