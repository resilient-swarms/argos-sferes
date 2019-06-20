#include <cstdlib>
#include <fstream>
#include <iostream>
 
int main()
{
    std::system("./bin/baseline_behaviour experiments/baseline-behavs.argos"); // execute the UNIX command "ls -l >test.txt"
    std::cout << "Program ended";    
}
