#include <gazebo/common/FuelModelDatabase.hh>
#include <iostream>

int main()
{
  std::cout << gazebo::common::FuelModelDatabase::Instance()->Servers().size()
            << std::endl;
}
