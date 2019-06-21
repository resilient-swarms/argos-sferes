class BaseLoopFunctions;

class EnvironmentGenerator
{
  public:
    EnvironmentGenerator()
    {
    }
    virtual void generate(BaseLoopFunctions* cLoopFunctions);
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
  virtual void generate(BaseLoopFunctions* cLoopFunctions);
};
