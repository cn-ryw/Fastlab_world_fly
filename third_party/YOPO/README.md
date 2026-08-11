# Vendored YOPO subset

This directory contains the inference/configuration subset originally imported
from a local YOPO_360 derivative in repository commit `b74e066`. The public
reference implementation is the `YOPO-Simple` branch of
<https://github.com/cn-ryw/YOPO_360_X5_PR>; the branch revision reviewed while
preparing this public tree was `bb6ca30c2c7272f96b899fe12433a09221c6a6be`.

The local tree is not a byte-for-byte mirror. It adds the X5 cruise profile and
contains simulator/runtime adaptations in configuration, depth-mask,
trajectory primitive and network integration code. Repository history is the
authoritative record of those changes. The upstream MIT license is preserved
in `LICENSE`.
