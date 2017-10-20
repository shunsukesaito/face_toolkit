// contains a base module class
#ifndef HFM_MODULE_H
#define HFM_MODULE_H

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "SPSCQueue.h"

using namespace rigtorp;

class Module;
typedef std::shared_ptr<Module> ModuleHandle;
typedef std::shared_ptr<SPSCQueue<std::string>> CmdQueueHandle;


/* Module is a basic data processing unit.
 Modules should only be created using create() method.
 Example of a module:

// modules/hello_world.h
class HelloWorldModule : public Module
{
     HelloWorldModule(const std::string &name,     // module name
                      const std::string &greeting) // module-specific parameter
         : greeting_(greeting)
     {
     }

public:
    virtual void process() const
    {
        std::cout << greeting_ << std::endl;
    }

    // create a new HelloWorldModule object
    static ModuleHandle create(const std::string &name, 
                               const std::string &greeting);

private:
    // greeting to use
    std::string greeting_;
}

// modules/hello_world.cc

// Creates and registers a new HelloWorldModule object.
ModuleHandle HelloWorldModule::create(const std::string &name, 
                                      const std::string &greeting)
{
    // create a new module
    ModuleHandle module(new HelloWorldModule(name, greeting));
    // register our new module,
    // so it can be obtained by its name
    ModuleRegistry::register_module(module);
    return module;
}

*/
class Module
{
    // default constructor is not implemented,
    // each module must have a unique name
    Module();
    // Disable copying
    Module(const Module &);
    const Module &operator = (const Module &);

protected:
    // initializes a module
    Module(const std::string &name);

public:
    virtual ~Module();

    // obtain module name
    const std::string &name() const;

    // does module specific work
    virtual void Process();

    // Provides a way to interrupt the process.
    // Default implementation does nothing.
    virtual void Stop();

    // construct a default module
    static ModuleHandle Create(const std::string &name);
private:
    // module name
    std::string name_;
};

// Provides access to all registered modules.
// A module is automatically deleted if no one is referencing it.
struct ModuleRegistry
{
    // initialize registry
    static void Initialize();

    // returns number of registered modules
    static int num_modules();

    // registers a new module.
    // returns false if module with this name is already registered
    static bool RegisterModule(ModuleHandle module);

    // access module by name
    // returns an invalid handle, if the module is not available
    static ModuleHandle GetModule(const std::string &name);
};

#endif
