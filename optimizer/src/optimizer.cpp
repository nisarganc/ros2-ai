#include <stdio.h>
#include <iostream>
#include <optimizer/Optimizer.h>
#include <stdio.h>

using std::placeholders::_1;

int main(int argc, char * argv[])
{  
  std::cout << "Start" << std::endl;
  
  // optimizer
  Optimizer optimizer;
  optimizer.startfg();
  
  return 0;

}