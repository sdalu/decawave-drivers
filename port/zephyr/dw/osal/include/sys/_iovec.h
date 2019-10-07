#ifndef _SYS__IOVEC_H_
#define _SYS__IOVEC_H_

#include <sys/_types.h>

#ifndef _SIZE_T_DECLARED
typedef __size_t        size_t;
#define _SIZE_T_DECLARED
#endif

struct iovec {
        void    *iov_base;      /* Base address. */
        size_t   iov_len;       /* Length. */
};

#endif /* !_SYS__IOVEC_H_ */
