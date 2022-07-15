#include <iostream>

class UnitTestClass;

class ToBeTested
{

private:

   int Calculate() { return 0; }
   friend UnitTestClass;
};

class UnitTestClass
{
public:
    void RunTest()
    {
        ToBeTested object_under_test;
        if (object_under_test.Calculate() == 0)
        {
            std::cout << "Test passed" << std::endl;
        }
        else
        {
            std::cout << "Test failed" << std::endl;;
        }
    }
};

int main()
{
  UnitTestClass unit_tester;
  unit_tester.RunTest();
 
  return 0;
}