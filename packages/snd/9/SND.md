# SND
SND (later renamed to RwAudioCore when it was merged with RenderWare's audio libraries, and currently named EAAudioCore) was EA's cross-platform audio system library used across most of its first-party games of this era. It handles audio playback and processing at a lower level, leaving more complex features to the tech that interfaces with it (AEMS, Ginsu, etc.).

# Notes
- Mostly written in C, however a few pieces of the library make use of C++, using `extern "C"` for bridge functions that may need to be called in C.
- Seems to have been compiled with MSVC's `/O1` option.