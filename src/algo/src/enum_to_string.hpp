#undef BEGIN_ENUM
#undef DECL_ENUM_ELEMENT
#undef END_ENUM

#ifndef GEN_ENUM
#define BEGIN_ENUM(ENUM_NAME) enum tag##ENUM_NAME
#define DECL_ENUM_ELEMENT(element) element,
#define END_ENUM ;
#else
#define BEGIN_ENUM(ENUM_NAME)                                   \
  const char *get_string_##ENUM_NAME(enum tag##ENUM_NAME index) \
  {                                                             \
    switch (index)                                              \
    {
#define DECL_ENUM_ELEMENT(element) \
  case element:                    \
    return #element;               \
    break;
#define END_ENUM            \
  default:                  \
    return "Unknown value"; \
    }                       \
    }                       \
    ;
#endif