// std_function typemaps to provide write only callback functions

// Inspired by https://stackoverflow.com/questions/32644268/how-to-use-swig-to-wrap-stdfunction-objects

%define %_formacro_2n(macro, arg1, arg2,...)macro(arg1, arg2)
#if #__VA_ARGS__ != "__fordone__"
,%_formacro_2n(macro, __VA_ARGS__)
#endif
%enddef

%define %formacro_2n(macro,...)%_formacro_2n(macro,__VA_ARGS__,__fordone__)%enddef

%define _tesseract_std_function_call_args(arg_type, arg_name) arg_type arg_name  %enddef
%define _tesseract_std_function_call_vars(arg_type, arg_name) arg_name  %enddef

%define %tesseract_std_function(Name, Namespace, Ret, ...)

%shared_ptr(Name##Base)
%feature("director") Name##Base;
%pythondynamic Name##Base;

%inline
{
    class Name##Base
    {
    public:
        virtual Ret call( %formacro_2n(_tesseract_std_function_call_args,__VA_ARGS__) ) = 0;        
    };
}

%typemap(in) Name##Base (void *argp, int res = 0, std::shared_ptr< Name##Base > temp1) {
    // tesseract_std_function %typemap(in)
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(std::shared_ptr< Name##Base > *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum);
  }
  if (!argp) {
    %argument_nullref("$type", $symname, $argnum);
  } else {
    temp1 = *(%reinterpret_cast(argp, std::shared_ptr< Name##Base > *));
    $1 = [temp1]( %formacro_2n(_tesseract_std_function_call_args,__VA_ARGS__) ) { return temp1->call(  %formacro_2n(_tesseract_std_function_call_vars,__VA_ARGS__)  ); };
    if (newmem & SWIG_CAST_NEW_MEMORY) delete %reinterpret_cast(argp, std::shared_ptr< Name##Base > *);
  }
}

namespace Namespace
{
using Name = ::Name##Base;
}

%pythoncode %{

class Name(Name##Base):
  def __init__(self,fn):
    super(Name,self).__init__()
    self._fn = fn

  def call(*args):
    return self._fn(*args)
%}

%enddef