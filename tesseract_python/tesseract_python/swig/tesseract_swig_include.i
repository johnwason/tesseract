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

%exception {
  try {
    $action
  }
  SWIG_CATCH_STDEXCEPT
}

%feature("director:except") {
    if ($error != NULL) {
        throw Swig::DirectorMethodException();
    }
}

%pythonnondynamic;

%include "eigen.i"
%include "shared_factory.i"
%include "json_typemaps.i"
%include "eigen_types.i"

%{
namespace std
{
  template<typename T> struct remove_reference<swig::SwigPySequence_Ref<T>>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const swig::SwigPySequence_Ref<T>>
  {
    typedef const T type;
  };

  template<typename T> struct remove_reference<SwigValueWrapper<T>>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const SwigValueWrapper<T>>
  {
    typedef const T type;
  };

  template<typename T> struct remove_reference<SwigValueWrapper<T>&>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const SwigValueWrapper<T>&>
  {
    typedef const T type;
  };

}
%}