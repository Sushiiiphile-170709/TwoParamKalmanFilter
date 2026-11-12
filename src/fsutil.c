#include "fsutil.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#define PATH_BUF_SIZE 4096

kf_status kf_make_dirs(const char *path) {
  if (path == NULL || path[0] == '\0') return KF_ERR_INVALID_ARG;
  char buf[PATH_BUF_SIZE];
  const size_t n = strlen(path);
  if (n >= sizeof buf) return KF_ERR_INVALID_ARG;
  memcpy(buf, path, n + 1);

  /* Create each missing parent in turn. Index 0 is skipped so that a leading
   * '/' (absolute path) is not treated as an empty first component. */
  for (char *p = buf + 1; *p != '\0'; ++p) {
    if (*p != '/') continue;
    *p = '\0';
    if (mkdir(buf, 0777) != 0 && errno != EEXIST) return KF_ERR_IO;
    *p = '/';
  }
  if (mkdir(buf, 0777) != 0 && errno != EEXIST) return KF_ERR_IO;

  struct stat sb;
  if (stat(buf, &sb) != 0 || !S_ISDIR(sb.st_mode)) return KF_ERR_IO;
  return KF_OK;
}

kf_status kf_path_join(char *buf, size_t buf_size, const char *dir, const char *name) {
  if (buf == NULL || buf_size == 0 || dir == NULL || name == NULL) return KF_ERR_INVALID_ARG;
  const size_t dir_len = strlen(dir);
  const char *sep = (dir_len > 0 && dir[dir_len - 1] == '/') ? "" : "/";
  const int n = snprintf(buf, buf_size, "%s%s%s", dir, sep, name);
  if (n < 0 || (size_t)n >= buf_size) return KF_ERR_INVALID_ARG;
  return KF_OK;
}
