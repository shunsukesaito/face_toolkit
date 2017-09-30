// contains basic module functionality

// std includes
#include <unordered_map>
#include <vector>

// internal includes
#include "module.hpp"

namespace
{
// contains references to existing modules
std::unordered_map<std::string, std::weak_ptr<Module>> module_registry;
} // anonymous namespace

// Module constructor,
// initializes base module
Module::Module(const std::string &name) : name_(name)
{
}

// default destructor
// XXX: log module destruction
Module::~Module()
{
}

// return module name
const std::string &Module::name() const
{
    return name_;
}

// stub
void Module::Process()
{
}

// Default implementation of the interruption point
void Module::Stop()
{
    // nothing to do
}

// Factory method for basic module.
// Creates a new modules and returns its handle to user
ModuleHandle Module::Create(const std::string &name)
{
    ModuleHandle handle(new Module(name));
    // add this module to the global registry
    ModuleRegistry::RegisterModule(handle);
    return handle;
}

// clears module list
void ModuleRegistry::Initialize()
{
    module_registry.clear();
}

// Returns number of modules in the registry.
// Has linear complexity in number of modules.
int ModuleRegistry::num_modules()
{
    // make sure that registry doesn't contain null pointers
    std::vector<std::string> invalid_entries;
    for (const auto &entry : module_registry) {
        if (entry.second.expired()) invalid_entries.push_back(entry.first);
    }
    for (const auto &name : invalid_entries) module_registry.erase(name);
    return static_cast<int>(module_registry.size());
}

// Registers a module with the global module registry.
// The registry stores weak pointer to the module,
// so when module is no longer being used, it will be automatically
// destroyed.
bool ModuleRegistry::RegisterModule(ModuleHandle module)
{
    if (module_registry.count(module->name())) return false;
    module_registry[module->name()] = std::weak_ptr<Module>(module);
    return true;
}

// Returns a handle to a registered module by name.
// Return nullptr if the module is not in the registry,
// or if it's not available anymore.
ModuleHandle ModuleRegistry::GetModule(const std::string &name)
{
    auto module = module_registry.find(name);
    if (module != module_registry.end()) {
       return module->second.lock(); // try to obtain shared pointer from a weak pointer
    }
    return ModuleHandle();
}
