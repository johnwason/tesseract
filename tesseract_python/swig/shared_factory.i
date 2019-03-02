// Based on https://stackoverflow.com/questions/27392602/swig-downcasting-from-base-to-derived?rq=1

%define %_shared_factory_dispatch(Type)
if (!dcast) {
  SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> dobj
          = SWIG_SHARED_PTR_QNAMESPACE::dynamic_pointer_cast<Type>($1);
  if (dobj) {
    dcast = 1;
    SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> *smartresult
            = dobj ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type>(dobj) : 0;
    %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult),
                                   $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> *),
                                   SWIG_POINTER_OWN));
  }
}%enddef

%define %_shared_factory_dispatch_ref(Type)
if (!dcast) {
  SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> dobj
          = SWIG_SHARED_PTR_QNAMESPACE::dynamic_pointer_cast<Type>(*$1);
  if (dobj) {
    dcast = 1;
    SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> *smartresult
            = dobj ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type>(dobj) : 0;
    %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult),
                                   $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> *),
                                   SWIG_POINTER_OWN));
  }
}%enddef

%define %_shared_factory_dispatch_pointer(Type)
if (!dcast) {
  Type* temp = dynamic_cast<Type*>($1);
  if (temp) {
    dcast = 1;
    SWIG_SHARED_PTR_QNAMESPACE::shared_ptr< Type > *smartresult = temp ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr< Type >(temp SWIG_NO_NULL_DELETER_$owner) : 0;
    %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult), $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type > *), $owner | SWIG_POINTER_OWN));
  }
}%enddef

%define %shared_factory(BaseType,Types...)
%typemap(out) SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> {
  int dcast = 0;
  %formacro(%_shared_factory_dispatch, Types)
  if (!dcast) {
      SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> *smartresult
              = $1 ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType>($1) : 0;
      %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult),
                                     $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> *),
                                     SWIG_POINTER_OWN));
  }
}

%typemap(out) SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType>&, const SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType>& {
  int dcast = 0;
  %formacro(%_shared_factory_dispatch_ref, Types)
  if (!dcast) {
      SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> *smartresult
              = *$1 ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType>(*$1) : 0;
      %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult),
                                     $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> *),
                                     SWIG_POINTER_OWN));
  }
}

%typemap(out, fragment="SWIG_null_deleter_python") BaseType* {
  int dcast = 0;
  %formacro(%_shared_factory_dispatch_pointer, Types)
  if (!dcast) {
  SWIG_SHARED_PTR_QNAMESPACE::shared_ptr< BaseType > *smartresult = $1 ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr< BaseType >($1 SWIG_NO_NULL_DELETER_$owner) : 0;
  %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult), $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType > *), $owner | SWIG_POINTER_OWN));
  }
}


%enddef