// Implement very simple wrapper for boost::filesystem::path

%{
#include <boost/filesystem/path.hpp>
%}

namespace boost
{
namespace filesystem
{

%rename(FilesystemPath) path;
class path
{
public:

path(const std::string& s);
std::string string();

%pythoncode %{
def __str__(self):
    return self.string()
%}
};
}
}