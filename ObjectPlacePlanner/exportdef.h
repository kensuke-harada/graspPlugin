
#ifdef EXCADE_API
#undef EXCADE_API
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# ifdef EXCADE_OBJECTPLACEPLANNER_MAKE_DLL
#  define EXCADE_API __declspec(dllexport)
# else
#  define EXCADE_API __declspec(dllimport)
# endif
#else
# if __GNUC__ >= 4
#  define EXCADE_API __attribute__ ((visibility("default")))
# else
#  define EXCADE_API
# endif
#endif
