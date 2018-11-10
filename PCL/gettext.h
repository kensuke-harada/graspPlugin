/*
  This header should not be included in other header files,
  but included in the most bottom position of the inclusion part in the implementation (.cpp) files
  where the message internationalization (texts with _("...") form) is required.
*/


#ifdef CNOID_GETTEXT_DOMAIN_NAME
#undef CNOID_GETTEXT_DOMAIN_NAME
#endif
#define CNOID_GETTEXT_DOMAIN_NAME "PCL"

#ifdef _
#undef _
#endif

#define CNOID_ENABLE_GETTEXT 0

#if CNOID_ENABLE_GETTEXT

#include "libintl.h"
#define _(text) dgettext(CNOID_GETTEXT_DOMAIN_NAME, text)
#define N_(string) string

#else

namespace cnoid {
    inline const char* bindtextdomain(const char* domainname, const char* dirname) {
        return dirname;
    }
    inline const char* dgettext(const char* domainname, const char* msgid){
        return msgid;
    }
}

#define _(string) string
#define N_(string) string

#endif
