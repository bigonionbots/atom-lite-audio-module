# libhelix (codec-helix) v0.9.4, vendored

Source: https://github.com/pschatzmann/codec-helix tag v0.9.4 (GPL-3.0, see License.txt; the
Helix decoders themselves are under the RealNetworks RPSL/RCSL). Only `src/`, the licence,
README and library.properties are kept.

One change, in `src/ConfigHelix.h`: SBR (HE-AAC) can be switched off with `-DHELIX_NO_AAC_SBR`.
Upstream enables it unconditionally on everything but the ESP8266. With SBR the AAC decoder needs
~90 KB including a 51 KB contiguous block; Bluetooth Classic leaves ~75 KB on the ATOM Lite.
Without SBR, HE-AAC files play their AAC-LC core (half the audio bandwidth).
