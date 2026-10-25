# Deferred work

- [ ] Restore and validate the Odroid AArch64 cross toolchain. Deferred by the
  user on 2026-10-25 while simulator work takes priority. Preserve the original
  reason for using crosstool-NG: target the libc available on the Odroid,
  independently of the development PC's distribution and libc version.
  Record the Odroid OS, glibc, libstdc++ and libgcc versions; recover or create a
  pinned crosstool-NG configuration and compatible sysroot; then build and check
  the binary's required runtime symbol versions. A packaged cross compiler is
  not an automatic replacement. Keep the existing Odroid platform/toolchain
  structure until this task is resumed.
