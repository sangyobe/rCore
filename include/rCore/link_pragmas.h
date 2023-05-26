
#ifndef core_link_pragmas_H
#define core_link_pragmas_H

// If we are building the DLL (_EXPORTS), do not link against the .lib files:
#if !defined(rcore_EXPORTS) && (defined(_MSC_VER) || defined(__BORLANDC__))
#	if defined(_DEBUG)
#		pragma comment (lib, "rcore" RLAB_DEBUG_POSTFIX ".lib")
#	else
#		pragma comment (lib, "rcore.lib")
#	endif
#endif

/*   The macros below for DLL import/export are required for Windows only.
    Mostly all the definitions in this file are copied or at least cored
     on the file wx/dlimpexp.h, written by Vadim Zeitlin and published
	 under the wxWindows licence.
*/
#if defined(RLAB_OS_WINDOWS)
    /*
       __declspec works in BC++ 5 and later, Watcom C++ 11.0 and later as well
       as VC++ and gcc
     */
#    if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__GNUC__) || defined(__WATCOMC__)
#        define RCORE_EXPORT __declspec(dllexport)
#        define RCORE_IMPORT __declspec(dllimport)
#    else /* compiler doesn't support __declspec() */
#        define RCORE_EXPORT
#        define RCORE_IMPORT
#    endif
#elif defined(RLAB_OS_OS2)		/* was __WXPM__ */
#    if defined (__WATCOMC__)
#        define RCORE_EXPORT __declspec(dllexport)
        /*
           __declspec(dllimport) prepends __imp to imported symbols. We do NOT
           want that!
         */
#        define RCORE_IMPORT
#    elif defined(__EMX__)
#        define RCORE_EXPORT
#        define RCORE_IMPORT
#    elif (!(defined(__VISAGECPP__) && (__IBMCPP__ < 400 || __IBMC__ < 400 )))
#        define RCORE_EXPORT _Export
#        define RCORE_IMPORT _Export
#    endif
#elif defined(RLAB_OS_APPLE)
#    ifdef __MWERKS__
#        define RCORE_EXPORT __declspec(export)
#        define RCORE_IMPORT __declspec(import)
#    endif
#elif defined(__CYGWIN__)
#    define RCORE_EXPORT __declspec(dllexport)
#    define RCORE_IMPORT __declspec(dllimport)
#endif

/* for other platforms/compilers we don't anything */
#ifndef RCORE_EXPORT
#    define RCORE_EXPORT
#    define RCORE_IMPORT
#endif

/*  Macros that map to export declaration when building the DLL, to import
	declaration if using it or to nothing at all if we are not compiling as DLL */
#if defined(RLAB_BUILT_AS_DLL)
#	if defined(rcore_EXPORTS)  /* Building the DLL */
#		define RCORE_IMPEXP RCORE_EXPORT
#	else  /* Using the DLL */
#		define RCORE_IMPEXP RCORE_IMPORT
#	endif
#else /* not making nor using DLL */
#    define RCORE_IMPEXP
#endif

// Finally this one allows exporting a class that inherits from a
// template in MS DLLs with both MSVC and GCC. To see how nasty is
// this, Google "class derived from template DLL export" and suffer...
#if defined(_MSC_VER)
#   define RCORE_IMPEXP_TEMPL
#else
    // Mostly for Mingw:
#   define RCORE_IMPEXP_TEMPL RCORE_IMPEXP
#endif


#endif