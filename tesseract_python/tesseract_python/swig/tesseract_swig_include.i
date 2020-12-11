%{
#define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS
%}


%include <std_shared_ptr.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_pair.i>
%include <std_map.i>
%include <std_unordered_map.i>
%include <std_array.i>
%include <stdint.i>
%include <attribute.i>
%include <exception.i>
%include <pybuffer.i>

%include "eigen.i"
%include "shared_factory.i"
%include "json_typemaps.i"
%include "eigen_types.i"