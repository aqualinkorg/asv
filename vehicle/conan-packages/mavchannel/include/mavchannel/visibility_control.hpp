#ifndef MAVCHANNEL__VISIBILITY_CONTROL_H_
#define MAVCHANNEL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MAVCHANNEL_EXPORT __attribute__ ((dllexport))
    #define MAVCHANNEL_IMPORT __attribute__ ((dllimport))
  #else
    #define MAVCHANNEL_EXPORT __declspec(dllexport)
    #define MAVCHANNEL_IMPORT __declspec(dllimport)
  #endif
  #ifdef MAVCHANNEL_BUILDING_LIBRARY
    #define MAVCHANNEL_PUBLIC MAVCHANNEL_EXPORT
  #else
    #define MAVCHANNEL_PUBLIC MAVCHANNEL_IMPORT
  #endif
  #define MAVCHANNEL_PUBLIC_TYPE MAVCHANNEL_PUBLIC
  #define MAVCHANNEL_LOCAL
#else
  #define MAVCHANNEL_EXPORT __attribute__ ((visibility("default")))
  #define MAVCHANNEL_IMPORT
  #if __GNUC__ >= 4
    #define MAVCHANNEL_PUBLIC __attribute__ ((visibility("default")))
    #define MAVCHANNEL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MAVCHANNEL_PUBLIC
    #define MAVCHANNEL_LOCAL
  #endif
  #define MAVCHANNEL_PUBLIC_TYPE
#endif

#endif  // MAVCHANNEL__VISIBILITY_CONTROL_H_
