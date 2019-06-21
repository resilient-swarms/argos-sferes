
#include <argos3/core/simulator/simulator.h>
class EnvironmentGenerator
{
  public:
    EnvironmentGenerator()
    {
    }
    virtual void generate(argos::CSimulator& cSimulator);
};


class ConfigurationBasedGenerator : public EnvironmentGenerator
{
private:
  std::string filename;// has its own associated configuration file
public:
  ConfigurationBasedGenerator(std::string file_name)
  {
    filename = file_name;
  }
  virtual void generate(argos::CSimulator& cSimulator);
};
