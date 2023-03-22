#undef BEGIN_ENUM
#undef BEGIN_ENUM_VALUE
#undef DECL_ENUM_ELEMENT
#undef DECL_ENUM_VALUE
#undef END_ENUM

#if defined(GEN_ENUM)
#define BEGIN_ENUM_VALUE(ENUM_NAME, type) BEGIN_ENUM(ENUM_NAME)
#define BEGIN_ENUM(ENUM_NAME)                                   \
  const char *get_string_##ENUM_NAME(enum tag##ENUM_NAME index) \
  {                                                             \
    switch (index)                                              \
    {
#define DECL_ENUM_ELEMENT(element) \
  case element:                    \
    return #element;               \
    break;
#define DECL_ENUM_VALUE(element, value) \
  case element:                    \
    return #element;               \
    break;
#define END_ENUM            \
  default:                  \
    return "Unknown value"; \
    }                       \
    }                       \
    ;
#elif defined(GEN_VALUES)
#define DECL_ENUM_ELEMENT(element) static_assert(false, #element " is does not have any associated values, use DECL_ENUM_VALUE instead")
#define BEGIN_ENUM_VALUE(ENUM_NAME, type_name) \
  type_name get_value_##ENUM_NAME(enum tag##ENUM_NAME index)       \
  {     \
    type_name ret;                                                                      \
    switch (index)                                                            \
    {
#define DECL_ENUM_VALUE(element, value) \
  case element:                         \
    ret = value;                       \
    break;
#define END_ENUM            \
    }                       \
    return ret;             \
    }                       \
    ;
#else
#define BEGIN_ENUM_VALUE(ENUM_NAME, type) BEGIN_ENUM(ENUM_NAME)
#define BEGIN_ENUM(ENUM_NAME) enum tag##ENUM_NAME
#define DECL_ENUM_ELEMENT(element) element,
#define DECL_ENUM_VALUE(element, value) element,
#define END_ENUM ;
#endif