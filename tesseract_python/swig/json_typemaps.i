%{
#include <jsoncpp/json/json.h>
%}

%typemap(in) const Json::Value& (Json::Value temp){
	std::string *ptr = (std::string *)0;
	int res = SWIG_AsPtr_std_string($input, &ptr);
	if (!SWIG_IsOK(res) || !ptr) { 
	  %argument_fail((ptr ? res : SWIG_TypeError), "$type", $symname, $argnum); 
	}
	std::string ptr_dat;
	ptr_dat.swap(*ptr);	
	if (SWIG_IsNewObj(res)) %delete(ptr);
	Json::Reader reader;
	bool parse_successful = reader.parse(ptr_dat, temp);
	if (!parse_successful)
	{
		PyErr_SetString(PyExc_ValueError, "Could not parse Json.");
		SWIG_fail;
	}
	$1 = &temp;
	
}

