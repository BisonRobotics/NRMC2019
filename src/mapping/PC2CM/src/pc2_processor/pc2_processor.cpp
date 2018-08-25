

#include <pc2_processor/pc2_processor.h>

pc2cmProcessor::pc2cmProcessor(int one)
{
    isOne = one = 1;
}

bool pc2cmProcessor::getOne()
{
  return this->isOne;
}