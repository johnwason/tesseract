%template() std::vector<std::reference_wrapper<tesseract_planning::Instruction>>;

%typemap(in, noblock=0) std::vector<std::reference_wrapper<tesseract_planning::Instruction>> *(void  *argp = 0, int res = 0, std::vector<std::reference_wrapper<tesseract_planning::Instruction>> temp, std::vector<tesseract_planning::Instruction>* temp1) {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>>* in typemap
  res = SWIG_ConvertPtr($input, &argp,$descriptor(std::vector<tesseract_planning::Instruction>*), $disown | %convertptr_flags);
  if (!SWIG_IsOK(res)) { 
    %argument_fail(res, "$type", $symname, $argnum); 
  }
   temp1 = %reinterpret_cast(argp, std::vector<tesseract_planning::Instruction>*);
   for (auto& a: *temp1)
   {
       temp.push_back(a);
   }
   $1 = &temp;
}

%typemap(out) std::vector<std::reference_wrapper<tesseract_planning::Instruction>>* {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>>* out typemap

  std::vector<tesseract_planning::Instruction> temp_out;
  for(auto& a: *$1)
  {
      temp_out.push_back(a.get());
  }
  
  %set_output(SWIG_NewPointerObj(%new_copy(temp_out, std::vector<tesseract_planning::Instruction>), $descriptor(std::vector<tesseract_planning::Instruction>&), SWIG_POINTER_OWN | %newpointer_flags));
}

%typemap(out) std::vector<std::reference_wrapper<tesseract_planning::Instruction>> {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>> out typemap

  std::vector<tesseract_planning::Instruction> temp_out;
  for(auto& a: $1)
  {
      temp_out.push_back(a.get());
  }
  
  %set_output(SWIG_NewPointerObj(%new_copy(temp_out, std::vector<tesseract_planning::Instruction>), $descriptor(std::vector<tesseract_planning::Instruction>&), SWIG_POINTER_OWN | %newpointer_flags));
}

%typemap(out) std::vector<std::reference_wrapper<tesseract_planning::Instruction const>> {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>> out typemap

  std::vector<tesseract_planning::Instruction> temp_out;
  for(auto& a: *(%reinterpret_cast(&$1, $&ltype)))
  {
      temp_out.push_back(a.get());
  }
  
  %set_output(SWIG_NewPointerObj(%new_copy(temp_out, std::vector<tesseract_planning::Instruction>), $descriptor(std::vector<tesseract_planning::Instruction>&), SWIG_POINTER_OWN | %newpointer_flags));
}